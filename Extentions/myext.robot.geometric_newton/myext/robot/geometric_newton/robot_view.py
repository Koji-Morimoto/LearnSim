"""GeometricRobotView: 同じ運動学構造のロボット複数台を一括駆動・一括干渉判定するビュー (Newton GPU バッチ対応)。

`isaacsim.core.experimental.prims.Articulation` とメソッド名・引数 (`indices=`, `dof_indices=`)・
配列形状 `(台数, num_dofs)`・戻り値型 (`wp.array`) を揃えているため、既存の複数台 `Articulation` 用コードを
最小限の書き換えで移行できる。

設計のポイント:
    - 順運動学は全 N 台まとめて 1 回で計算し、Transform の書き込みも全台分を一括で行う
      (台数ぶんの Python ループを回さない)。
    - `self._q` / `self._base` / `self._world` の各行を個別の `GeometricRobot` へ共有させることで、
      View 側とロボット単体側のどちらから読み書きしても状態が食い違わない。
    - `fk_backend="auto"` では台数が `_AUTO_FK_MIN_BATCH` (既定 16) 以上かつ CUDA 利用可能時に
      `BatchedNewtonScene` (multi-world + CUDA Graph) を自動選択し、0 フレーム遅延の並列 FK と
      `check_collisions_batch()` / `compute_clearances_batch()` を実行できる。
    - 構成ロボットの 1 台でも構造が再構築された場合 (Variant の切り替えなど) は、安全のため
      この View を失効扱い (`GeometricStructureChangedError`) とし、作り直しを促す。

座標・四元数・行列の規約:
    - `get_world_poses` / `set_world_poses` / `get_link_world_poses` の四元数は `(w, x, y, z)`。
    - `get_link_world_matrices` は `(N, num_links, 4, 4)` の列ベクトル形式・スケール抜き・倍精度 NumPy 配列。
"""

from __future__ import annotations

from typing import Sequence, get_args

import numpy as np
import warp as wp
from pxr import Usd

from .collision import CollisionScope, GeometricCollisionPair, PairFilter
from .layer import GeometricLayer, GeometricLayerOptions, _setting
from .model import AngleUnit
from .model import transforms as tf
from .model.command import check_angle_unit
from .newton_backend import BatchedNewtonScene, CollisionBackend, FkBackend, _AUTO_FK_MIN_BATCH
from .robot import GeometricRobot, GeometricStructureChangedError, _as_indices
from .usd_reader import UsdArticulationSource
from .writer import GeometricFabricWriter, GeometricGpuFabricWriter, GeometricUsdWriter, LocalPoseSolver, WriteTarget

__all__ = ["GeometricRobotView"]

# ==============================================================================
# 内部ヘルパー
# ==============================================================================


def _np(value) -> np.ndarray:
    """`wp.array` または配列風オブジェクトを倍精度 `np.ndarray` へ統一する。"""
    return value.numpy() if isinstance(value, wp.array) else np.asarray(value, dtype=float)


# ==============================================================================
# GeometricRobotView 本体
# ==============================================================================


class GeometricRobotView:
    """同一構造の `GeometricRobot` 複数台を 1 枚のバッチ配列として扱う管理クラス。"""

    def __init__(
        self,
        robots: Sequence[GeometricRobot],
        *,
        write_target: WriteTarget | None = None,
        angle_unit: AngleUnit | None = None,
        collision_backend: CollisionBackend | None = None,
        fk_backend: FkBackend | None = None,
        newton_device: str | None = None,
        convex_meshes: bool | None = None,
        environment_roots: Sequence[str] | None = None,
        use_cuda_graph: bool = True,
        use_gpu_fabric_writer: bool = False,
    ) -> None:
        """`robots` は同一の運動学構造を持ち、同じ `GeometricLayer` を共有している必要がある。

        通常は `GeometricRobotView.from_articulations()` を使うと共有 `GeometricLayer` ごと一括生成できる。
        `convex_meshes` は `BatchedNewtonScene` で `UsdGeom.Mesh` を単一凸包化 (`True`、既定) するか
        元の三角形メッシュ (`False`) で判定するかを選ぶ。
        `use_gpu_fabric_writer=True` かつ Newton FK が有効な場合、Newton の `state.body_q` を CPU へ
        コピーせずに `GeometricGpuFabricWriter` で直接 `omni:fabric:worldMatrix` へ書き込む。
        """
        if not robots:
            raise ValueError("ロボットが 1 台も無い")
        first = robots[0]
        for robot in robots[1:]:
            if not first.model.is_kinematically_equal(robot.model):
                raise ValueError(f"構造が違うロボットは 1 つの View にまとめられない: {first.name} と {robot.name}")
            if robot.layer is not first.layer:
                raise ValueError("View のロボットは同じ Sublayer を共有すること(from_articulations か、layer= を渡して作る)")
        target = write_target if write_target is not None else _setting("write_target", "fabric")
        if target not in get_args(WriteTarget):
            raise ValueError(f"write_target は {get_args(WriteTarget)} のどれか: {target!r}")
        col_be = collision_backend if collision_backend is not None else _setting("collision_backend", "newton")
        if col_be not in get_args(CollisionBackend):
            raise ValueError(f"collision_backend は {get_args(CollisionBackend)} のどれか: {col_be!r}")
        fk_be = fk_backend if fk_backend is not None else _setting("fk_backend", "auto")
        if fk_be not in get_args(FkBackend):
            raise ValueError(f"fk_backend は {get_args(FkBackend)} のどれか: {fk_be!r}")

        self._robots = list(robots)
        self._model = first.model
        self._layer = first.layer
        self._write_target: WriteTarget = target
        self._collision_backend: CollisionBackend = col_be
        self._fk_backend: FkBackend = fk_be
        self._newton_device: str = newton_device if newton_device is not None else _setting("newton_device", "auto")
        self._convex_meshes: bool = bool(convex_meshes) if convex_meshes is not None else bool(first.convex_meshes)
        self._environment_roots = None if environment_roots is None else list(environment_roots)
        self._use_cuda_graph = bool(use_cuda_graph)
        self._use_gpu_fabric_writer = bool(use_gpu_fabric_writer)
        self._newton_scene: BatchedNewtonScene | None = None
        self._newton_scene_collider_version: int = -1
        self._gpu_fabric_writer: GeometricGpuFabricWriter | None = None
        self._gpu_fabric_failed = False

        self._angle_unit: AngleUnit = check_angle_unit(angle_unit if angle_unit is not None else first.angle_unit)
        self._revolute = first._revolute.copy()
        self._stale_reason: str | None = None
        n, d = len(robots), first.num_dofs
        # 全ロボットの関節値・ベース姿勢・可動範囲を 1 枚の連続配列にまとめる。
        # `_push()` で `robot._q = self._q[i]` のようにビュー行を差し戻すため、単体 API と値が同期する。
        self._q = np.stack([r._q for r in robots])
        self._base = np.stack([r._base for r in robots])
        self._lower = np.stack([r._lower for r in robots])
        self._upper = np.stack([r._upper for r in robots])
        self._prim_link_indices = first._prim_link_indices
        self._prim_paths = [p for r in robots for p in r._prim_paths]
        self._solver = LocalPoseSolver(first.stage, self._prim_paths)
        self._usd_writer = GeometricUsdWriter(self._layer, self._prim_paths)
        self._fabric_writer: GeometricFabricWriter | None = None
        self._usd_dirty = False
        self._world = self._base[:, None] @ self._model.forward_kinematics(self._q, resolve_mimic=False)
        self._shape = (n, d)
        # Sublayer を付けた直後は USD -> Fabric の遅延同期で古い姿勢へ巻き戻ることがあるため、
        # 最初の数フレームは `_on_update` で Fabric へ書き直す。
        self._repush_frames = 3
        self._update_subscription = None
        self._layer.add_listener(self._on_layer_changed)
        for robot in self._robots:
            # 単体ロボット側で `flush_to_usd()` が呼ばれても、View 全体で 1 回の `Sdf.ChangeBlock` に集約する
            robot._flush_delegate = self.flush_to_usd
            robot.add_structure_listener(self._on_structure_changed)
        try:
            import omni.kit.app

            self._update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update, name="myext.robot.geometric_newton.view"
            )
        except Exception:
            pass
        self._push()

    # ==========================================================================
    # 鮮度検証・バックエンド設定・イベントコールバック
    # ==========================================================================

    def _on_structure_changed(self, old_names: list[str], new_names: list[str]) -> None:
        self._stale_reason = f"中のロボットが作り直された(DOF {len(old_names)} -> {len(new_names)})"
        for robot in self._robots:
            robot._flush_delegate = None

    def _check_fresh(self) -> None:
        """全ロボットの USD 構造変化を確認し、1 台でも再構築されていれば `GeometricStructureChangedError` を送出する。"""
        for robot in self._robots:
            robot._ensure_current()  # 外から Variant が変えられていれば、ここで再構築が走り `_stale_reason` が立つ
        if self._stale_reason is not None:
            raise GeometricStructureChangedError(f"{self._stale_reason}。GeometricRobotView を作り直す")

    @property
    def angle_unit(self) -> AngleUnit:
        return self._angle_unit

    @angle_unit.setter
    def angle_unit(self, value: AngleUnit) -> None:
        self._angle_unit = check_angle_unit(value)

    @property
    def collision_backend(self) -> CollisionBackend:
        return self._collision_backend

    @property
    def fk_backend(self) -> FkBackend:
        return self._fk_backend

    @property
    def convex_meshes(self) -> bool:
        """Newton バッチ干渉判定で `UsdGeom.Mesh` を単一凸包へ変換するか (`True`=凸包 / `False`=生三角形メッシュ)。"""
        return self._convex_meshes

    @convex_meshes.setter
    def convex_meshes(self, value: bool) -> None:
        flag = bool(value)
        if flag == self._convex_meshes:
            return
        self._convex_meshes = flag
        if self._newton_scene is not None:
            self._newton_scene.close()
            self._newton_scene = None
            self._newton_scene_collider_version = -1

    @property
    def newton_scene(self) -> BatchedNewtonScene:
        """全 N 台を multi-world で扱う `BatchedNewtonScene` (初回アクセス時に遅延生成する)。"""
        self._check_fresh()
        cur_cver = self._robots[0].collider_version
        if self._newton_scene is not None and cur_cver != self._newton_scene_collider_version:
            self._newton_scene.close()
            self._newton_scene = None
            if self._gpu_fabric_writer is not None:
                self._gpu_fabric_writer.close()
                self._gpu_fabric_writer = None
        if self._newton_scene is None:
            self._newton_scene = BatchedNewtonScene(
                self,
                device=self._newton_device,
                environment_roots=self._environment_roots,
                convex_meshes=self._convex_meshes,
                use_cuda_graph=self._use_cuda_graph,
            )
            self._newton_scene_collider_version = cur_cver
        return self._newton_scene

    def reload_colliders(self) -> None:
        """全ロボットのコライダーキャッシュと `BatchedNewtonScene` を破棄し、次の判定で最新 USD コライダーを読み直す。"""
        self._check_fresh()
        for robot in self._robots:
            robot.reload_colliders()
        if self._newton_scene is not None:
            self._newton_scene.close()
            self._newton_scene = None
        if self._gpu_fabric_writer is not None:
            self._gpu_fabric_writer.close()
            self._gpu_fabric_writer = None
        self._newton_scene_collider_version = -1

    def _use_newton_fk(self) -> bool:
        """現在の `fk_backend`・台数・CUDA 可否から、FK に `BatchedNewtonScene` を使うか判定する。"""
        if self._fk_backend == "newton":
            return True
        if self._fk_backend == "auto" and len(self._robots) >= _AUTO_FK_MIN_BATCH:
            return wp.is_cuda_available()
        return False

    def _unit_scale(self, cols: np.ndarray, unit: AngleUnit | None) -> np.ndarray | None:
        unit = self._angle_unit if unit is None else check_angle_unit(unit)
        return None if unit == "rad" else np.where(self._revolute[cols], np.pi / 180.0, 1.0)

    def _on_layer_changed(self, applied: bool) -> None:
        if applied:
            self._solver.refresh()
            self._repush_frames = 3
            self._push()

    def _on_update(self, _event) -> None:
        """USD 書き込み直後の数フレーム間、USD -> Fabric 同期による姿勢の巻き戻りを防ぐ再送コールバック。

        なぜ GPU Fabric パスを優先分岐するか:
            `use_gpu_fabric_writer=True` のときに `GeometricFabricWriter` へフォールバックすると、
            `GeometricGpuFabricWriter` が意図的に除去した `omni:fabric:localMatrix` が再生成されてしまい、
            以後の `omni:fabric:worldMatrix` 直接書き込みが `IFabricHierarchy` に上書きされるため。
        """
        if self._repush_frames <= 0 or not self._layer.is_applied() or self._write_target == "usd":
            return
        self._repush_frames -= 1
        if self._use_gpu_fabric_writer and self._use_newton_fk():
            gpu_fab = self._gpu_fabric()
            if gpu_fab is not None:
                try:
                    ns = self.newton_scene
                    gpu_fab.write_body_q(ns.newton_state.body_q, int(ns.newton_model.body_count))
                    return
                except Exception:
                    self._gpu_fabric_writer = None
        fabric = self._fabric()
        if fabric is not None:
            fabric.write(self._solver, self._world[:, self._prim_link_indices].reshape(-1, 4, 4))

    # ==========================================================================
    # ファクトリメソッド
    # ==========================================================================

    @classmethod
    def from_articulations(
        cls,
        stage: Usd.Stage,
        sources: Sequence[UsdArticulationSource | str],
        *,
        layer_options: GeometricLayerOptions | None = None,
        write_target: WriteTarget | None = None,
        apply_layer: bool = True,
        collision_backend: CollisionBackend | None = None,
        fk_backend: FkBackend | None = None,
        newton_device: str | None = None,
        convex_meshes: bool | None = None,
        auto_components: bool | None = None,
        environment_roots: Sequence[str] | None = None,
        use_cuda_graph: bool = True,
        use_gpu_fabric_writer: bool = False,
    ) -> "GeometricRobotView":
        """ステージ上の同じ構造のロボット複数台から `GeometricRobotView` を構築する。

        個々の `GeometricRobot` を `write_target="usd"`, `apply_layer=False`, `fk_backend="numpy"` で
        生成してから共有 `GeometricLayer` を 1 回だけ `apply()` する理由:
            - ロボット数ぶんの Sublayer や個別 Fabric ライター・個別 Newton シーンを無駄に作らず、
              `GeometricRobotView` が全台分の Sublayer とバッチ書き込みを一元管理するため。
        """
        layer = GeometricLayer(stage, layer_options)
        robots = [
            GeometricRobot.from_articulation(
                stage,
                s,
                layer=layer,
                write_target="usd",
                apply_layer=False,
                collision_backend=collision_backend,
                fk_backend="numpy",
                convex_meshes=convex_meshes,
                auto_components=auto_components,
            )
            for s in sources
        ]
        if apply_layer:
            layer.apply()
        return cls(
            robots,
            write_target=write_target,
            collision_backend=collision_backend,
            fk_backend=fk_backend,
            newton_device=newton_device,
            convex_meshes=convex_meshes,
            environment_roots=environment_roots,
            use_cuda_graph=use_cuda_graph,
            use_gpu_fabric_writer=use_gpu_fabric_writer,
        )

    # ==========================================================================
    # 名前・個数・インデックス照会
    # ==========================================================================

    def __len__(self) -> int:
        return len(self._robots)

    @property
    def robots(self) -> list[GeometricRobot]:
        return list(self._robots)

    @property
    def layer(self) -> GeometricLayer:
        return self._layer

    @property
    def num_dofs(self) -> int:
        return self._model.num_dofs

    @property
    def dof_names(self) -> list[str]:
        return self._model.dof_names

    @property
    def dof_types(self) -> list[str]:
        return list(self._model.dof_types)

    @property
    def num_joints(self) -> int:
        return self._model.num_joints

    @property
    def joint_names(self) -> list[str]:
        return self._model.joint_names

    @property
    def num_links(self) -> int:
        return self._model.num_links

    @property
    def link_names(self) -> list[str]:
        return self._model.link_names

    @property
    def dof_paths(self) -> list[list[str | None]]:
        return [r.dof_paths for r in self._robots]

    @property
    def joint_paths(self) -> list[list[str | None]]:
        return [r.joint_paths for r in self._robots]

    @property
    def link_paths(self) -> list[list[str | None]]:
        return [r.link_paths for r in self._robots]

    def get_dof_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_dof_indices(names), dtype=wp.int32, device="cpu")

    def get_joint_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_joint_indices(names), dtype=wp.int32, device="cpu")

    def get_link_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_link_indices(names), dtype=wp.int32, device="cpu")

    # ==========================================================================
    # 関節値の一括読み書き
    # ==========================================================================

    def _rows_cols(self, indices, dof_indices) -> tuple[np.ndarray, np.ndarray]:
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        cols = _as_indices(None if dof_indices is None else _np(dof_indices).astype(int), self.num_dofs)
        return rows, cols

    def set_dof_positions(self, positions, *, indices=None, dof_indices=None, unit: AngleUnit | None = None) -> None:
        """全台 (または `indices` で指定した台) の関節値を一括設定し、FK と書き込みを実行する。

        `positions` は `(len(rows), len(cols))` または 1 行 `(len(cols),)` (全台へブロードキャスト)。
        可動範囲外の値は切り詰められ、その後 `model.resolve_mimic(self._q)` で従動軸が参照軸から計算される。
        """
        self._check_fresh()
        rows, cols = self._rows_cols(indices, dof_indices)
        values = np.broadcast_to(_np(positions).reshape(-1, len(cols)), (len(rows), len(cols)))
        scale = self._unit_scale(cols, unit)
        if scale is not None:
            values = values * scale
        self._q[np.ix_(rows, cols)] = np.clip(values, self._lower[np.ix_(rows, cols)], self._upper[np.ix_(rows, cols)])
        # mimic の従動軸をこの場で解決しておくため、下流の `eval_fk_batch` / `check_collisions_batch` は
        # `resolve_mimic=False` を渡して二重計算を省ける。
        self._q[:] = self._model.resolve_mimic(self._q)
        self._push()

    def get_dof_positions(self, *, indices=None, dof_indices=None, unit: AngleUnit | None = None) -> wp.array:
        self._check_fresh()
        rows, cols = self._rows_cols(indices, dof_indices)
        q = self._q[np.ix_(rows, cols)]
        scale = self._unit_scale(cols, unit)
        return wp.array((q if scale is None else q / scale).astype(np.float32), dtype=wp.float32, device="cpu")

    def set_dof_position_targets(self, positions, *, indices=None, dof_indices=None, unit: AngleUnit | None = None) -> None:
        """`Articulation` 互換の別名。Drive を持たないため `set_dof_positions` と同義。"""
        self.set_dof_positions(positions, indices=indices, dof_indices=dof_indices, unit=unit)

    def get_dof_position_targets(self, *, indices=None, dof_indices=None, unit: AngleUnit | None = None) -> wp.array:
        return self.get_dof_positions(indices=indices, dof_indices=dof_indices, unit=unit)

    def get_dof_limits(self, *, indices=None, dof_indices=None, unit: AngleUnit | None = None) -> tuple[wp.array, wp.array]:
        self._check_fresh()
        rows, cols = self._rows_cols(indices, dof_indices)
        scale = self._unit_scale(cols, unit)
        factor = 1.0 if scale is None else scale
        lower = wp.array((self._lower[np.ix_(rows, cols)] / factor).astype(np.float32), dtype=wp.float32, device="cpu")
        upper = wp.array((self._upper[np.ix_(rows, cols)] / factor).astype(np.float32), dtype=wp.float32, device="cpu")
        return lower, upper

    # ==========================================================================
    # 姿勢の一括読み書き
    # ==========================================================================

    def get_world_poses(self, *, indices=None) -> tuple[wp.array, wp.array]:
        """各ロボットの根リンクのワールド姿勢 `(位置 (N, 3), 四元数 (N, 4) = (w, x, y, z))` を返す。"""
        self._check_fresh()
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        base = self._base[rows]
        return (
            wp.array(base[:, :3, 3].astype(np.float32), dtype=wp.float32, device="cpu"),
            wp.array(tf.matrix_to_quat(base[:, :3, :3]).astype(np.float32), dtype=wp.float32, device="cpu"),
        )

    def set_world_poses(self, positions=None, orientations=None, *, indices=None) -> None:
        """各ロボットの根リンクのワールド姿勢を更新し、FK と書き込みを実行する。`orientations` は `(w, x, y, z)`。"""
        self._check_fresh()
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        if positions is not None:
            self._base[rows, :3, 3] = np.broadcast_to(_np(positions).reshape(-1, 3), (len(rows), 3))
        if orientations is not None:
            self._base[rows, :3, :3] = tf.quat_to_matrix(np.broadcast_to(_np(orientations).reshape(-1, 4), (len(rows), 4)))
        self._push()

    def get_link_world_poses(self, *, indices=None, link_indices=None) -> tuple[wp.array, wp.array]:
        """各リンクのワールド姿勢 `(位置 (N, L, 3), 四元数 (N, L, 4) = (w, x, y, z))`。`Articulation` には無い拡張メソッド。"""
        self._check_fresh()
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        cols = _as_indices(None if link_indices is None else _np(link_indices).astype(int), self.num_links)
        world = self._world[np.ix_(rows, cols)]
        return (
            wp.array(world[..., :3, 3].astype(np.float32), dtype=wp.float32, device="cpu"),
            wp.array(tf.matrix_to_quat(world[..., :3, :3]).astype(np.float32), dtype=wp.float32, device="cpu"),
        )

    def get_link_world_matrices(self) -> np.ndarray:
        """全台・全リンクのワールド姿勢 `(N, num_links, 4, 4)` (列ベクトル形式・スケール抜き・倍精度 NumPy)。"""
        self._check_fresh()
        return self._world.copy()

    # ==========================================================================
    # バッチ干渉チェック・クリアランス (Newton multi-world GPU)
    # ==========================================================================

    def sync_environment(self, *, use_fabric: bool | str = "auto") -> None:
        """共有環境障害物 (`world=-1`) の姿勢を `BatchedNewtonScene` へ同期する。

        `use_fabric="auto"` の挙動は `BatchedNewtonScene.sync_environment` と同一
        (Play=ON 中の有効な `RigidBodyAPI` 障害物のみ Fabric、静的障害物・Stop 中は USD `XformCache`)。
        """
        self.newton_scene.sync_environment(use_fabric=use_fabric)  # type: ignore[arg-type]

    def check_collisions_batch(
        self,
        scope: CollisionScope = "all",
        *,
        clearance: float = 0.0,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
    ) -> list[list[GeometricCollisionPair]]:
        """全 N 台分の干渉を Newton GPU バッチ (multi-world + 共有環境) で 1 回にまとめて判定する (0 フレーム遅延)。

        `self._q` は `set_dof_positions` で mimic 解決済みのため `resolve_mimic=False` で渡す。
        """
        self._check_fresh()
        return self.newton_scene.check_collisions_batch(
            self._q,
            self._base,
            scope=scope,
            clearance=clearance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
            sync_env=sync_env,
            resolve_mimic=False,
        )

    def compute_clearances_batch(
        self,
        scope: CollisionScope = "all",
        *,
        max_distance: float = 0.05,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
    ) -> list[list[GeometricCollisionPair]]:
        """全 N 台分について `max_distance` [m] 以内の組と符号付き距離 (`separation`) を近い順で返す。"""
        self._check_fresh()
        return self.newton_scene.compute_clearances_batch(
            self._q,
            self._base,
            scope=scope,
            max_distance=max_distance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
            sync_env=sync_env,
            resolve_mimic=False,
        )

    # ==========================================================================
    # 書き込み・フラッシュ・終了処理
    # ==========================================================================

    def _gpu_fabric(self) -> GeometricGpuFabricWriter | None:
        if not self._use_gpu_fabric_writer or self._gpu_fabric_failed or not self._prim_paths:
            return None
        if self._gpu_fabric_writer is None:
            try:
                ns = self.newton_scene
                self._gpu_fabric_writer = GeometricGpuFabricWriter(
                    self._robots[0].stage,
                    self._prim_paths,
                    ns.prim_body_indices,
                    world_scales=self._solver.world_scales,
                    device=ns.device,
                )
            except Exception:
                self._gpu_fabric_failed = True
        return self._gpu_fabric_writer

    def _push(self) -> None:
        """全台の FK を評価し、各ロボットのキャッシュを揃えたうえで `write_target` へ一括書き込みする。"""
        use_newton = self._use_newton_fk()
        if use_newton:
            ns = self.newton_scene
            self._world = ns.eval_fk_batch(self._q, self._base, resolve_mimic=False, return_matrices=True)
            assert self._world is not None
        else:
            self._world = self._base[:, None] @ self._model.forward_kinematics(self._q, resolve_mimic=False)
        # 各ロボットが保持する状態を View のスライスへ差し替える (代入のみで再計算・個別書き込みはしない)
        for i, robot in enumerate(self._robots):
            robot._q, robot._base, robot._world = self._q[i], self._base[i], self._world[i]
            robot._usd_dirty = False
        if not self._layer.is_applied() or not self._prim_paths:
            return
        world = self._world[:, self._prim_link_indices].reshape(-1, 4, 4)
        gpu_fab = self._gpu_fabric() if (use_newton and self._write_target in ("fabric", "both")) else None
        fabric = None if gpu_fab is not None else (self._fabric() if self._write_target in ("fabric", "both") else None)
        if self._write_target in ("usd", "both") or (gpu_fab is None and fabric is None):
            self._usd_writer.write(self._solver, world)
            self._usd_dirty = False
            if gpu_fab is not None or fabric is not None:
                self._repush_frames = 2
        else:
            self._usd_dirty = True
        if gpu_fab is not None:
            ns = self.newton_scene
            gpu_fab.write_body_q(ns.newton_state.body_q, int(ns.newton_model.body_count))
        elif fabric is not None:
            fabric.write(self._solver, world)

    def _fabric(self) -> GeometricFabricWriter | None:
        if self._fabric_writer is None:
            try:
                self._fabric_writer = GeometricFabricWriter(self._robots[0].stage, self._prim_paths)
            except Exception:
                self._write_target = "usd"
        return self._fabric_writer

    def flush_to_usd(self) -> bool:
        """Fabric にのみ書いてある全台の姿勢を USD Sublayer へ一括書き出しする。

        `collision_backend="newton"` の `check_collisions_batch` では不要。
        PhysX バックエンド (`collision_backend="physx"`) や USD を直接読む外部処理の前に呼ぶ。
        """
        if self._stale_reason is not None or not self._usd_dirty or not self._layer.is_applied():
            return False
        self._usd_writer.write(self._solver, self._world[:, self._prim_link_indices].reshape(-1, 4, 4))
        self._usd_dirty = False
        self._repush_frames = 2
        return True

    def close(self, *, remove_layer: bool = True) -> None:
        """購読・GPU ライター・Newton バッチシーン・各ロボットを解放する。

        `remove_layer=True` (既定) なら共有 `GeometricLayer` をステージから外して元の構成へ戻す。
        """
        self._update_subscription = None
        self._layer.remove_listener(self._on_layer_changed)
        if self._gpu_fabric_writer is not None:
            self._gpu_fabric_writer.close()
            self._gpu_fabric_writer = None
        if self._newton_scene is not None:
            self._newton_scene.close()
            self._newton_scene = None
        for robot in self._robots:
            robot.remove_structure_listener(self._on_structure_changed)
            robot.close(remove_layer=False)
        if remove_layer:
            self._layer.remove()
