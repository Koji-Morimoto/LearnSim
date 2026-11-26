"""Newton 1.6.1 バッチ GPU 空間 (`BatchedNewtonScene`)。"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Sequence, get_args

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from ..model import GeometricCollisionFilter, GeometricRobotModel
from ..model import transforms as tf
from ..usd.reader import _np, split_scale
from .newton_scene import GeometricScene, _EnvShapeEntry
from .newton_shapes import (
    _AUTO_FK_MIN_BATCH,
    _AUTO_FK_MIN_DOFS,
    CollisionBackend,
    FkBackend,
    _add_usd_collider_shape,
    _axis_rot_to_z,
    _discover_stage_colliders,
    _has_active_rigid_body,
    _import_newton,
    _is_timeline_playing,
    _mat4_to_wp_transform,
    _stage_up_axis,
    _try_fabric_hierarchy,
    _wp_body_q_to_mat4,
    resolve_newton_device,
)
from .types import CollisionScope, GeometricCollisionPair, PairFilter, PairKind, find_colliders

if TYPE_CHECKING:
    from ..runtime.robot import GeometricRobot
    from ..runtime.robot_view import GeometricRobotView

__all__ = [
    "BatchedNewtonScene",
]


# ==============================================================================
# BatchedNewtonScene: GeometricRobotView 用の multi-world GPU 並列バックエンド
# ==============================================================================


class BatchedNewtonScene:
    """同じ構造のロボット B 台を Newton の multi-world (`world 0..B-1` + 共有 `world=-1` 環境) で一括評価する。

    設計上の重要ポイント:
        - 1 台分のテンプレート `ModelBuilder` (`tmpl`) を組み立てたうえで
          `wb.replicate(tmpl, world_count=self._num_envs, spacing=(0.0, 0.0, 0.0))` を使う理由:
          Newton の各ワールド (`0..B-1`) 同士は衝突判定されず、各ロボットは「自分自身のリンク同士」および
          「`world=-1` に置いた共有環境障害物」とのみ判定される。B 台が同じ空間座標を探索しても
          互いを誤検出せず、`spacing=(0, 0, 0)` で全ロボットが同一ワールド座標系を共有できる。
        - `bake_adjacent_filter=True` (既定) にする理由:
          バッチ探索では隣接リンクのペアをブロードフェーズ段階で除外しておくことで GPU 接触数と
          メモリ消費を大幅に削減できる。
        - CUDA Graph (`wp.ScopedCapture(device=self._device)`) を使う理由:
          毎ステップの Python -> CUDA カーネル起動オーバーヘッドを消し、`eval_fk` および
          `eval_fk + collide + eval_rigid_contact_kinematics` を 1 回の `wp.capture_launch` で実行する。
          キャプチャ直前にウォームアップ実行 + `wp.synchronize_device(self._device)` を行うことで、
          Warp の JIT コンパイルや内部バッファ確保がキャプチャ内に混入して失敗するのを防ぐ。
    """

    def __init__(
        self,
        view: "GeometricRobotView",
        *,
        device: str = "auto",
        environment_roots: Sequence[str] | None = None,
        broad_phase: Literal["auto", "explicit", "nxn", "sap"] = "auto",
        convex_meshes: bool = True,
        bake_adjacent_filter: bool = True,
        bake_collision_filter: bool | None = None,
        use_cuda_graph: bool = True,
    ) -> None:
        newton, wp = _import_newton()
        self._view = view
        self._robots = list(view.robots)
        self._model_spec: GeometricRobotModel = self._robots[0].model
        self._stage = self._robots[0].stage
        self._num_envs = len(self._robots)
        self._device = resolve_newton_device(device)
        self._use_cuda_graph = bool(use_cuda_graph and self._device.startswith("cuda"))
        self._convex_meshes = convex_meshes
        self._bake_adjacent_filter = bool(bake_adjacent_filter)
        self._bake_collision_filter = (
            bool(bake_adjacent_filter) if bake_collision_filter is None else bool(bake_collision_filter)
        )
        self._current_gap = 0.0

        # テンプレート 1 台分を `GeometricRobotModel` + USD コライダーから組む (BFS トポロジカル順 + 根 FREE 関節)
        tmpl = newton.ModelBuilder(up_axis=_stage_up_axis(self._stage), gravity=wp.vec3(0.0, 0.0, 0.0))
        tmpl.rigid_gap = 0.0
        joint_filter_parent = self._bake_adjacent_filter and not self._bake_collision_filter
        shape_cfg = newton.ModelBuilder.ShapeConfig(
            margin=0.0, gap=0.0, collision_filter_parent=joint_filter_parent
        )
        unit_inertia = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

        first = self._robots[0]
        model = self._model_spec
        order = list(model._c["order"])
        parent_joint = model._c["parent_joint"]
        dof_of_joint = model._c["dof_of_joint"]

        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        local_link_to_body = np.full(model.num_links, -1, dtype=int)
        local_shape_link: list[str] = []
        local_link_shapes: dict[str, list[int]] = {}

        link_world: dict[int, np.ndarray] = {}
        for li in order:
            ln = model.links[li].name
            lp = first.link_path_map.get(ln)
            link_world[li] = (
                split_scale(_np(xc.GetLocalToWorldTransform(self._stage.GetPrimAtPath(lp))))[0]
                if lp and self._stage.GetPrimAtPath(lp).IsValid()
                else np.eye(4)
            )

        for li in order:
            ln = model.links[li].name
            bid = tmpl.add_link(
                xform=_mat4_to_wp_transform(link_world[li]),
                mass=1.0,
                inertia=unit_inertia,
                lock_inertia=True,
                label=ln,
            )
            local_link_to_body[li] = bid
            sids_for_link: list[int] = []
            for cp in first.collider_paths.get(ln, ()):
                sid, _ = _add_usd_collider_shape(
                    tmpl, bid, self._stage, cp, link_world[li], shape_cfg, xc, convex_meshes=convex_meshes
                )
                if sid is not None:
                    local_shape_link.append(ln)
                    sids_for_link.append(sid)
            if sids_for_link:
                local_link_shapes[ln] = sids_for_link

        # 全ロボットで共通して除外されているリンク組をテンプレートへ登録 (`wb.replicate()` が全ワールドへ自動複製する)
        if self._bake_collision_filter:
            active_links = list(local_link_shapes.keys())
            for idx_a in range(len(active_links)):
                la = active_links[idx_a]
                sids_a = local_link_shapes[la]
                for idx_b in range(idx_a + 1, len(active_links)):
                    lb = active_links[idx_b]
                    if all(r.collision_filter.is_ignored(la, lb) for r in self._robots):
                        for sa in sids_a:
                            for sb in local_link_shapes[lb]:
                                tmpl.add_shape_collision_filter_pair(sa, sb)

        root_body = int(local_link_to_body[order[0]])
        root_jid = tmpl.add_joint_free(parent=-1, child=root_body, label="root_free", collision_filter_parent=False)
        art_joints = [root_jid]
        j2nj: dict[int, int] = {}
        for li in order[1:]:
            j_idx = parent_joint[li]
            js = model.joints[j_idx]
            pb = int(local_link_to_body[model.get_link_index(js.parent_link)])
            cb = int(local_link_to_body[li])
            px, cx = _mat4_to_wp_transform(js.parent_frame), _mat4_to_wp_transform(js.child_frame)
            if js.joint_type == "revolute":
                nj = tmpl.add_joint_revolute(
                    parent=pb, child=cb, parent_xform=px, child_xform=cx, axis=wp.vec3(*js.axis),
                    label=js.name, collision_filter_parent=joint_filter_parent,
                )
            elif js.joint_type == "prismatic":
                nj = tmpl.add_joint_prismatic(
                    parent=pb, child=cb, parent_xform=px, child_xform=cx, axis=wp.vec3(*js.axis),
                    label=js.name, collision_filter_parent=joint_filter_parent,
                )
            else:
                nj = tmpl.add_joint_fixed(
                    parent=pb, child=cb, parent_xform=px, child_xform=cx,
                    label=js.name, collision_filter_parent=joint_filter_parent,
                )
            art_joints.append(nj)
            j2nj[j_idx] = nj
        tmpl.add_articulation(art_joints, label=model.name)

        # Global `world=-1` に共有環境障害物を登録してから、B 個の独立ワールド (`0..B-1`) を複製する
        wb = newton.ModelBuilder(up_axis=_stage_up_axis(self._stage), gravity=wp.vec3(0.0, 0.0, 0.0))
        wb.rigid_gap = 0.0

        excludes = [p for r in self._robots for p in r.link_path_map.values()]
        for r in self._robots:
            excludes.extend(getattr(r, "root_prim_paths", ()))
        if environment_roots is None:
            roots = ["/World"] if self._stage.GetPrimAtPath("/World").IsValid() else ["/"]
        else:
            roots = list(environment_roots)
        env_paths = _discover_stage_colliders(self._stage, roots, excludes) if roots else []

        self._env_entries: list[_EnvShapeEntry] = []
        self._env_shape_map: dict[int, str] = {}
        for cp in env_paths:
            sid, axis_rot = _add_usd_collider_shape(
                wb, -1, self._stage, cp, np.eye(4), shape_cfg, xc, convex_meshes=convex_meshes
            )
            if sid is not None:
                entry = _EnvShapeEntry(prim_path=cp, shape_id=sid, axis_rot=axis_rot)
                self._env_entries.append(entry)
                self._env_shape_map[sid] = cp

        self._num_env_shapes = len(self._env_entries)
        self._bodies_per_world = model.num_links
        self._local_link_to_body = local_link_to_body
        self._local_body_to_link: list[str] = [""] * model.num_links
        for li, bid in enumerate(local_link_to_body):
            self._local_body_to_link[int(bid)] = model.links[li].name

        wb.replicate(tmpl, world_count=self._num_envs, spacing=(0.0, 0.0, 0.0))

        self._model = wb.finalize(device=self._device)
        self._model.shape_gap.zero_()
        self._state = self._model.state()

        bp = broad_phase
        if bp == "auto":
            bp = "explicit" if int(self._model.shape_count) <= 16384 else "sap"
        self._pipeline = newton.CollisionPipeline(
            self._model, broad_phase=bp, include_static_kinematic_pairs=True, verify_buffers=False
        )
        self._contacts = self._pipeline.contacts()
        cmax = int(self._contacts.rigid_contact_max)
        self._dist_buf = wp.zeros(cmax, dtype=wp.float32, device=self._device)
        self._p0_buf = wp.zeros(cmax, dtype=wp.vec3, device=self._device)
        self._p1_buf = wp.zeros(cmax, dtype=wp.vec3, device=self._device)

        # World 0 の関節オフセットと 1 ワールドあたり座標数 (`_coords_per_world`) から全 B ワールド共通の配列形状を決定
        q_starts = self._model.joint_q_start.numpy()
        self._coords_per_world = int(self._model.joint_coord_count) // self._num_envs
        self._root_q_local = int(q_starts[root_jid])
        self._dof_to_local_q = np.full(model.num_dofs, -1, dtype=int)
        for j_idx, d_idx in dof_of_joint.items():
            self._dof_to_local_q[d_idx] = int(q_starts[j2nj[j_idx]])

        self._joint_q_np = np.zeros((self._num_envs, self._coords_per_world), dtype=np.float32)
        self._joint_q_wp = wp.zeros(self._model.joint_coord_count, dtype=wp.float32, device=self._device)
        self._joint_qd_wp = wp.zeros(self._model.joint_dof_count, dtype=wp.float32, device=self._device)

        # 全 B 台分の prim 順に対応する Newton body インデックス (`GeometricGpuFabricWriter` 用)
        prim_link_idx = first._prim_link_indices
        self._prim_body_indices = np.concatenate(
            [w * self._bodies_per_world + local_link_to_body[prim_link_idx] for w in range(self._num_envs)]
        ).astype(np.int32)

        self._shape_body_np = self._model.shape_body.numpy()

        # CUDA Graph キャプチャ (ウォームアップ + 同期でカーネルコンパイルを完了させてからキャプチャする)
        self._fk_graph = None
        self._fk_col_graph = None
        if self._use_cuda_graph:
            newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
            self._pipeline.collide(self._state, self._contacts)
            newton.eval_rigid_contact_kinematics(
                self._model, self._state, self._contacts,
                out_distance=self._dist_buf, out_point0_world=self._p0_buf, out_point1_world=self._p1_buf,
            )
            wp.synchronize_device(self._device)
            with wp.ScopedCapture(device=self._device) as cap_fk:
                newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
            self._fk_graph = cap_fk.graph
            with wp.ScopedCapture(device=self._device) as cap_fc:
                newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
                self._pipeline.collide(self._state, self._contacts)
                newton.eval_rigid_contact_kinematics(
                    self._model, self._state, self._contacts,
                    out_distance=self._dist_buf, out_point0_world=self._p0_buf, out_point1_world=self._p1_buf,
                )
            self._fk_col_graph = cap_fc.graph

    # ==========================================================================
    # バッチプロパティ・転送・FK・環境同期
    # ==========================================================================

    @property
    def device(self) -> str:
        return self._device

    @property
    def newton_model(self):
        return self._model

    @property
    def newton_state(self):
        return self._state

    @property
    def prim_body_indices(self) -> np.ndarray:
        return self._prim_body_indices

    def _upload_q(self, q_batch: np.ndarray, base_batch: np.ndarray, *, resolve_mimic: bool = False) -> None:
        """全 B 台分のベース姿勢と関節値を 1 回のホスト->デバイス転送 (`wp.copy`) で `self._joint_q_wp` へ送る。

        四元数は `tf.matrix_to_quat` の `(w, x, y, z)` から Newton FREE 関節の `(x, y, z, w)` へ並べ替える。
        """
        _, wp = _import_newton()
        if resolve_mimic:
            q_batch = self._model_spec.resolve_mimic(q_batch)
        r0 = self._root_q_local
        self._joint_q_np[:, r0 : r0 + 3] = base_batch[:, :3, 3].astype(np.float32)
        q_wxyz = tf.matrix_to_quat(base_batch[:, :3, :3]).astype(np.float32)
        self._joint_q_np[:, r0 + 3 : r0 + 6] = q_wxyz[:, 1:4]
        self._joint_q_np[:, r0 + 6] = q_wxyz[:, 0]
        if len(self._dof_to_local_q):
            self._joint_q_np[:, self._dof_to_local_q] = q_batch.astype(np.float32)
        wp.copy(self._joint_q_wp, wp.array(self._joint_q_np.reshape(-1), dtype=wp.float32, device="cpu"))

    def eval_fk_batch(
        self,
        q_batch: np.ndarray,
        base_batch: np.ndarray,
        *,
        resolve_mimic: bool = True,
        return_matrices: bool = True,
    ) -> np.ndarray | None:
        """全 B 台の FK を 1 回の GPU 起動 (CUDA Graph 有効時は `wp.capture_launch`) で評価する。

        直接呼び出しでも従動軸が落ちないよう `resolve_mimic=True` を既定としている
        (`GeometricRobotView` からは既に解決済みのため `resolve_mimic=False` で呼ぶ)。
        """
        newton, wp = _import_newton()
        self._upload_q(q_batch, base_batch, resolve_mimic=resolve_mimic)
        if self._fk_graph is not None:
            wp.capture_launch(self._fk_graph)
        else:
            newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
        if not return_matrices:
            return None
        bq = self._state.body_q.numpy().reshape(self._num_envs, self._bodies_per_world, 7)
        bq_ordered = bq[:, self._local_link_to_body, :]
        return _wp_body_q_to_mat4(bq_ordered)

    def sync_environment(self, *, use_fabric: bool | Literal["auto"] = "auto") -> None:
        """共有環境障害物 (`world=-1`) の姿勢を `model.shape_transform` でその場更新する。

        `use_fabric="auto"` (既定) の読み元判定は `GeometricScene.sync_environment` と同一
        (`Play=ON` 中の有効な `RigidBodyAPI` 障害物のみ Fabric、静的障害物・`Stop` 中は USD `XformCache`)。
        """
        if not self._env_entries:
            return
        _, wp = _import_newton()
        playing = _is_timeline_playing() if use_fabric == "auto" else False
        want_fabric_any = (use_fabric is True) or (use_fabric == "auto" and playing)
        hier, usdrt = _try_fabric_hierarchy(self._stage) if want_fabric_any else (None, None)
        if hier is not None:
            hier.update_world_xforms()
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        st_np = self._model.shape_transform.numpy()
        for entry in self._env_entries:
            prim = self._stage.GetPrimAtPath(entry.prim_path)
            if not prim or not prim.IsValid():
                continue
            read_fabric = (use_fabric is True) or (use_fabric == "auto" and playing and _has_active_rigid_body(prim))
            world_m = None
            if read_fabric and hier is not None and usdrt is not None:
                try:
                    fmat = np.array(hier.get_world_xform(usdrt.Sdf.Path(entry.prim_path)), dtype=float).reshape(4, 4).T
                    if np.all(np.isfinite(fmat)) and np.linalg.norm(fmat[:3, :3]) > 1e-12:
                        world_m = fmat
                except Exception:
                    world_m = None
            if world_m is None:
                world_m = _np(xc.GetLocalToWorldTransform(prim))
            rigid, _ = split_scale(world_m)
            rot = rigid[:3, :3] @ entry.axis_rot
            p = rigid[:3, 3]
            w, x, y, z = tf.matrix_to_quat(rot)
            st_np[entry.shape_id] = np.array([p[0], p[1], p[2], x, y, z, w], dtype=np.float32)
        wp.copy(self._model.shape_transform, wp.array(st_np, dtype=wp.transform, device="cpu"))

    # ==========================================================================
    # バッチ干渉チェック・クリアランス計算
    # ==========================================================================

    def _set_gap(self, max_distance: float) -> None:
        target = 0.0 if max_distance <= 0.0 else (0.5 * float(max_distance) + 1e-4)
        if abs(target - self._current_gap) > 1e-9:
            if target == 0.0:
                self._model.shape_gap.zero_()
            else:
                self._model.shape_gap.fill_(target)
            self._current_gap = target

    def check_collisions_batch(
        self,
        q_batch: np.ndarray,
        base_batch: np.ndarray,
        *,
        scope: CollisionScope = "all",
        clearance: float = 0.0,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
        resolve_mimic: bool = True,
    ) -> list[list[GeometricCollisionPair]]:
        """全 B 台分の干渉を 1 回の GPU パスでまとめて評価し、台数分の `list[GeometricCollisionPair]` を返す。

        直接呼び出しでも従動軸が正しく反映されるよう `resolve_mimic=True` を既定としている。
        """
        if scope not in get_args(CollisionScope):
            raise ValueError(f"scope は {get_args(CollisionScope)} のどれか: {scope!r}")
        newton, wp = _import_newton()
        if sync_env and scope in ("environment", "all") and self._env_entries:
            self.sync_environment()
        self._set_gap(clearance)
        self._upload_q(q_batch, base_batch, resolve_mimic=resolve_mimic)

        if self._fk_col_graph is not None:
            wp.capture_launch(self._fk_col_graph)
        else:
            newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
            self._pipeline.collide(self._state, self._contacts)
            newton.eval_rigid_contact_kinematics(
                self._model, self._state, self._contacts,
                out_distance=self._dist_buf, out_point0_world=self._p0_buf, out_point1_world=self._p1_buf,
            )

        # 接触数がバッファ上限 `rigid_contact_max` を超えてもカウンタは増え続けるため、必ずクランプしてからスライスする
        n = min(int(self._contacts.rigid_contact_count.numpy()[0]), int(self._contacts.rigid_contact_max))
        per_robot_within: list[dict[frozenset[str], GeometricCollisionPair]] = [{} for _ in range(self._num_envs)]
        per_robot_env: list[dict[tuple[str, str], GeometricCollisionPair]] = [{} for _ in range(self._num_envs)]
        if n == 0:
            return [[] for _ in range(self._num_envs)]

        s0 = self._contacts.rigid_contact_shape0.numpy()[:n]
        s1 = self._contacts.rigid_contact_shape1.numpy()[:n]
        dist = self._dist_buf.numpy()[:n]
        p0_core = self._p0_buf.numpy()[:n]
        p1_core = self._p1_buf.numpy()[:n]
        normals = self._contacts.rigid_contact_normal.numpy()[:n]
        m0 = self._contacts.rigid_contact_margin0.numpy()[:n]
        m1 = self._contacts.rigid_contact_margin1.numpy()[:n]
        ignore = tuple(p.rstrip("/") for p in ignore_prim_paths)

        def _shape_link(sid: int) -> tuple[int, str] | None:
            # multi-world で複製された剛体番号 `b` を `(ワールド番号 w, テンプレート内リンク名)` へ復号する
            b = int(self._shape_body_np[sid])
            if b < 0:
                return None
            w = b // self._bodies_per_world
            local_b = b % self._bodies_per_world
            if 0 <= w < self._num_envs:
                return w, self._local_body_to_link[local_b]
            return None

        for i in range(n):
            a, b = int(s0[i]), int(s1[i])
            if a < 0 or b < 0:
                continue
            sep = float(dist[i])
            if sep > clearance:
                continue
            nrm = normals[i]
            pt0 = p0_core[i] + float(m0[i]) * nrm
            pt1 = p1_core[i] - float(m1[i]) * nrm
            info_a = _shape_link(a)
            info_b = _shape_link(b)
            if info_a is not None and info_b is not None:
                if scope not in ("within", "all") or info_a[0] != info_b[0]:
                    continue
                w_idx, la, lb = info_a[0], info_a[1], info_b[1]
                robot = self._robots[w_idx]
                if la == lb or robot.collision_filter.is_ignored(la, lb):
                    continue
                if extra_filter and (extra_filter(la, lb) or extra_filter(lb, la)):
                    continue
                # `(link_a, link_b)` を辞書順へ正規化するとき、接触点と法線 (`a -> b`) も同時に反転して整合させる
                if la > lb:
                    la, lb, pt_a, pt_b, n_ab = lb, la, pt1, pt0, -nrm
                else:
                    pt_a, pt_b, n_ab = pt0, pt1, nrm
                key = frozenset((la, lb))
                prev = per_robot_within[w_idx].get(key)
                if prev is None or (prev.separation is not None and sep < prev.separation):
                    per_robot_within[w_idx][key] = GeometricScene._make_robot_pair(
                        robot, la, robot, lb, sep, pt_a, pt_b, n_ab
                    )
            elif info_a is not None or info_b is not None:
                if scope not in ("environment", "all"):
                    continue
                if info_a is not None:
                    w_idx, lname, env_sid, pt_a, pt_b, n_ab = info_a[0], info_a[1], b, pt0, pt1, nrm
                else:
                    assert info_b is not None
                    w_idx, lname, env_sid, pt_a, pt_b, n_ab = info_b[0], info_b[1], a, pt1, pt0, -nrm
                env_path = self._env_shape_map.get(env_sid)
                if env_path is None or any(env_path == p or env_path.startswith(p + "/") for p in ignore):
                    continue
                robot = self._robots[w_idx]
                ekey = (lname, env_path)
                prev = per_robot_env[w_idx].get(ekey)
                if prev is None or (prev.separation is not None and sep < prev.separation):
                    per_robot_env[w_idx][ekey] = GeometricScene._make_env_pair(
                        robot, lname, env_path, sep, pt_a, pt_b, n_ab
                    )

        results: list[list[GeometricCollisionPair]] = []
        for w_idx in range(self._num_envs):
            combined = list(per_robot_within[w_idx].values()) + list(per_robot_env[w_idx].values())
            combined.sort(key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))
            results.append(combined)
        return results

    def compute_clearances_batch(
        self,
        q_batch: np.ndarray,
        base_batch: np.ndarray,
        *,
        scope: CollisionScope = "all",
        max_distance: float = 0.05,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
        resolve_mimic: bool = True,
    ) -> list[list[GeometricCollisionPair]]:
        """全 B 台分について `max_distance` [m] 以内の組を `separation` 昇順 (負=めり込みが先頭) で返す。"""
        if max_distance < 0.0:
            raise ValueError(f"max_distance は 0 以上: {max_distance}")
        batch = self.check_collisions_batch(
            q_batch,
            base_batch,
            scope=scope,
            clearance=max_distance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
            sync_env=sync_env,
            resolve_mimic=resolve_mimic,
        )
        return [
            sorted(pairs, key=lambda p: (float("inf") if p.separation is None else p.separation, p.kind, p.link_a))
            for pairs in batch
        ]

    def close(self) -> None:
        self._fk_graph = None
        self._fk_col_graph = None
        self._model = None
        self._state = None
        self._pipeline = None
        self._contacts = None
