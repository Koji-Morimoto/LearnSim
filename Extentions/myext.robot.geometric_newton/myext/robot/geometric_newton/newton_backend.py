"""Newton 1.6.1 バックエンド: 自前コリジョン空間 (`GeometricScene`) とバッチ GPU 空間 (`BatchedNewtonScene`)。

PhysX の `PhysicsScene` やダイナミクスソルバー (`solver.step`) を一切使わず、Newton 内部の
    `newton.ModelBuilder` / `newton.eval_fk` / `newton.CollisionPipeline` / `newton.eval_rigid_contact_kinematics`
だけを使って順運動学・干渉チェック・符号付き距離 (めり込み量 / クリアランス) を 0 フレーム遅延で評価する。

なぜ `builder.add_usd()` ではなく `GeometricRobotModel` + USD コライダー抽出から直接 `ModelBuilder` を組むのか:
    1. `body0` / `body1` が逆転した USD 関節: `GeometricRobotModel` 側で親・子フレーム反転と軸反転を正規化済みのため、
       そのまま `add_joint_revolute` / `add_joint_prismatic` / `add_joint_fixed` へ渡せば 100% 一致する。
    2. 既に `GeometricLayer` が適用されたステージ: `GeometricLayer` は無名 Sublayer 上で `ArticulationRootAPI` と
       `Joint` を無効化しているため、`builder.add_usd()` では関節構造を読み取れない。
    3. 非 USD ソース (`from_dh` / `from_mdh` / `from_screw` / `from_urdf` / `attach()`):
       `UsdPhysics.Joint` を一切持たない簡易 prim 群でも、`GeometricRobotModel` を単一の正本 (Single Source of Truth)
       として同一パイプラインで FK と干渉チェックを実行できる。

座標・四元数・行列の規約 (本モジュール全体で統一):
    - `GeometricRobot` / `GeometricRobotModel`:
      4x4 行列は**列ベクトル形式** (`v' = M @ v`、並進は第 3 列 `M[:3, 3]`)、四元数は **`(w, x, y, z)`**。
    - Warp / Newton (`wp.transform`, `state.body_q`, `model.joint_q`, `model.shape_transform`):
      7 要素 `[px, py, pz, qx, qy, qz, qw]` で、四元数は **`(x, y, z, w)`**。
      相互変換は `_mat4_to_wp_transform` と `_wp_body_q_to_mat4` に集約している。
    - USD `Gf.Matrix4d` / Fabric `get_world_xform`:
      **行ベクトル形式** (並進が第 3 行) のため、読み取り時に `_np(...)` または `.T` で列ベクトル形式へ揃える。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Literal, Sequence, get_args

import numpy as np
from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from .collision import CollisionScope, GeometricCollisionPair, GeometricLinkSet, PairFilter, PairKind, find_colliders
from .layer import is_authoring
from .model import GeometricCollisionFilter, GeometricRobotModel
from .model import transforms as tf
from .usd_reader import _np, split_scale

if TYPE_CHECKING:
    from .robot import GeometricRobot
    from .robot_view import GeometricRobotView

__all__ = [
    "CollisionBackend",
    "FkBackend",
    "resolve_newton_device",
    "GeometricScene",
    "BatchedNewtonScene",
]

# ==============================================================================
# 定数・座標変換・デバイス解決ヘルパー
# ==============================================================================

CollisionBackend = Literal["newton", "physx"]
FkBackend = Literal["auto", "numpy", "newton"]

# `fk_backend="auto"` の切り替えしきい値。
# 少数 DOF・少数台では Warp 配列のホスト転送オーバーヘッドより NumPy のほうが速く、
# 大規模バッチ (`GeometricRobotView` で 16 台以上 + CUDA) では Newton GPU バッチが優位になる。
_AUTO_FK_MIN_DOFS = 32
_AUTO_FK_MIN_BATCH = 16

# コライダーの有効状態や形状寸法に直結する USD 属性名一覧。
# `xformOp:transform:geometric` などの姿勢属性を含めないことで、通常の姿勢更新では
# Newton モデルの再構築 (`finalize`) を発火させず、コライダー属性変更時のみ確実に再構築する。
_COLLIDER_ATTRS = frozenset(
    (
        "physics:collisionEnabled",
        "physics:approximation",
        "radius",
        "size",
        "height",
        "axis",
        "points",
        "faceVertexCounts",
        "faceVertexIndices",
    )
)


def _import_newton():
    """`newton` / `newton.usd` / `warp` を遅延 import する (純 NumPy 機能のみの import 時間を抑えるため)。"""
    import newton
    import newton.usd
    import warp as wp

    return newton, wp


def resolve_newton_device(device: str = "auto") -> str:
    """`'auto'` なら CUDA が使えるときは `'cuda:0'`、使えないときは `'cpu'` を返す。"""
    _, wp = _import_newton()
    if device == "auto":
        return "cuda:0" if wp.is_cuda_available() else "cpu"
    return device


def _stage_up_axis(stage: Usd.Stage):
    """USD ステージの Up 軸メタデータ (`Y` / `Z`) を `newton.Axis` へ変換する。"""
    newton, _ = _import_newton()
    token = UsdGeom.GetStageUpAxis(stage)
    return newton.Axis.Y if token == UsdGeom.Tokens.y else newton.Axis.Z


def _mat4_to_wp_transform(m: np.ndarray):
    """列ベクトル形式 4x4 剛体変換行列 `m` を `wp.transform` へ変換する。

    `tf.matrix_to_quat` は `(w, x, y, z)` を返すが、`wp.quat` のコンストラクタは `(x, y, z, w)` 順であるため
    ここで並べ替えて渡す。
    """
    _, wp = _import_newton()
    p = m[:3, 3]
    w, x, y, z = tf.matrix_to_quat(m[:3, :3])
    return wp.transform(
        wp.vec3(float(p[0]), float(p[1]), float(p[2])),
        wp.quat(float(x), float(y), float(z), float(w)),
    )


def _wp_body_q_to_mat4(body_q_np: np.ndarray) -> np.ndarray:
    """Newton `body_q (..., 7)` (`[px, py, pz, qx, qy, qz, qw]`) を列ベクトル形式 4x4 行列 `(..., 4, 4)` へ変換する。

    Warp の `(qx, qy, qz, qw)` を `GeometricRobot` 側の `(w, x, y, z)` へ並べ替えてから回転行列を復元する。
    """
    pos = body_q_np[..., :3]
    q_xyzw = body_q_np[..., 3:7]
    q_wxyz = np.concatenate([q_xyzw[..., 3:4], q_xyzw[..., 0:3]], axis=-1)
    rot = tf.quat_to_matrix(q_wxyz)
    out = np.zeros(body_q_np.shape[:-1] + (4, 4), dtype=np.float64)
    out[..., :3, :3] = rot
    out[..., :3, 3] = pos
    out[..., 3, 3] = 1.0
    return out


def _axis_rot_to_z(axis_token: str | None) -> np.ndarray:
    """Newton の Z 軸基準の `Capsule` / `Cylinder` / `Cone` を USD の `axis="X"|"Y"|"Z"` へ合わせる 3x3 回転行列。

    Newton のプリミティブ形状は常にローカル Z 軸が長手方向であるため、USD 側が `X` 軸なら Y 軸周り `+90°`、
    `Y` 軸なら X 軸周り `-90°` の補正回転をシェイプ姿勢の右側から掛ける。
    """
    tok = (axis_token or "Z").upper()
    if tok == "X":
        return tf.rotation_about_axis((0.0, 1.0, 0.0), math.pi / 2.0)
    if tok == "Y":
        return tf.rotation_about_axis((1.0, 0.0, 0.0), -math.pi / 2.0)
    return np.eye(3)


# ==============================================================================
# USD コライダー抽出・ステージ探索・Fabric 読取ヘルパー
# ==============================================================================


def _add_usd_collider_shape(
    builder,
    body_idx: int,
    stage: Usd.Stage,
    collider_path: str,
    body_world_rigid: np.ndarray,
    shape_cfg,
    xform_cache: UsdGeom.XformCache,
    *,
    convex_meshes: bool = True,
    vis_out: dict[int, tuple] | None = None,
) -> tuple[int | None, np.ndarray]:
    """USD の `UsdGeom.Gprim` コライダーを `newton.ModelBuilder` へ登録し、`(shape_id, axis_rot_3x3)` を返す。

    設計上のポイント:
        - `rel_mat = tf.invert(body_world_rigid) @ col_rigid` により、コライダーのワールド姿勢を
          所有リンク剛体 (`body_idx`) のローカル座標系へ変換して登録する (環境障害物は `body_idx=-1`, `body_world_rigid=I`)。
        - プリミティブ形状 (`Sphere` / `Cube` / `Capsule` / `Cylinder` / `Cone`) はワールドスケール
          `(sx, sy, sz)` を寸法パラメータへ焼き込み、`Mesh` は `scale=wp.vec3(sx, sy, sz)` として渡す。
        - `newton.usd.get_mesh(..., apply_stage_units=False)` とする理由:
          `GeometricRobot` はステージ単位そのままで FK・姿勢を扱うため、メッシュ頂点に二重の単位変換が入るのを防ぐ。
        - `convex_meshes=True` (既定) のときは `compute_convex_hull()` で単一凸包 (`GeoType.CONVEX_MESH`) へ変換して
          高速な GJK/MPR 狭域判定に乗せる。`convex_meshes=False` のときは元の三角形メッシュそのまま (`GeoType.MESH`) で登録する。
        - `vis_out` を渡すと、`shape_id -> 形状仕様タプル` を記録し、Debug モード時の `DebugDraw`
          ワイヤーフレーム可視化で「Newton が実際に判定に使っている形状」を遅延生成できるようにする
          (通常実行時は辺配列を生成しないためメモリ・時間コストはほぼゼロ)。
    """
    newton, wp = _import_newton()
    prim = stage.GetPrimAtPath(collider_path)
    if not prim or not prim.IsValid():
        return None, np.eye(3)

    col_world = _np(xform_cache.GetLocalToWorldTransform(prim))
    col_rigid, col_scale = split_scale(col_world)
    rel_mat = tf.invert(body_world_rigid) @ col_rigid
    sx, sy, sz = [abs(float(v)) for v in col_scale]
    axis_rot = np.eye(3)

    if prim.IsA(UsdGeom.Sphere):
        r = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0) * max(sx, sy, sz)
        sid = builder.add_shape_sphere(
            body_idx, xform=_mat4_to_wp_transform(rel_mat), radius=r, cfg=shape_cfg, label=collider_path
        )
        if vis_out is not None:
            vis_out[sid] = ("sphere", (r,))
        return sid, axis_rot

    if prim.IsA(UsdGeom.Cube):
        size = float(UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0)
        hx, hy, hz = 0.5 * size * sx, 0.5 * size * sy, 0.5 * size * sz
        sid = builder.add_shape_box(
            body_idx,
            xform=_mat4_to_wp_transform(rel_mat),
            hx=hx,
            hy=hy,
            hz=hz,
            cfg=shape_cfg,
            label=collider_path,
        )
        if vis_out is not None:
            vis_out[sid] = ("box", (hx, hy, hz))
        return sid, axis_rot

    if prim.IsA(UsdGeom.Capsule):
        cap = UsdGeom.Capsule(prim)
        axis_tok = cap.GetAxisAttr().Get() or "Z"
        r = float(cap.GetRadiusAttr().Get() or 0.5)
        h = float(cap.GetHeightAttr().Get() or 1.0)
        r_s, h_s = (max(sy, sz), sx) if axis_tok == "X" else ((max(sx, sz), sy) if axis_tok == "Y" else (max(sx, sy), sz))
        axis_rot = _axis_rot_to_z(axis_tok)
        m_adj = rel_mat.copy()
        m_adj[:3, :3] = m_adj[:3, :3] @ axis_rot
        rr, hh = r * r_s, 0.5 * h * h_s
        sid = builder.add_shape_capsule(
            body_idx,
            xform=_mat4_to_wp_transform(m_adj),
            radius=rr,
            half_height=hh,
            cfg=shape_cfg,
            label=collider_path,
        )
        if vis_out is not None:
            vis_out[sid] = ("capsule", (rr, hh))
        return sid, axis_rot

    if prim.IsA(UsdGeom.Cylinder):
        cyl = UsdGeom.Cylinder(prim)
        axis_tok = cyl.GetAxisAttr().Get() or "Z"
        r = float(cyl.GetRadiusAttr().Get() or 0.5)
        h = float(cyl.GetHeightAttr().Get() or 1.0)
        r_s, h_s = (max(sy, sz), sx) if axis_tok == "X" else ((max(sx, sz), sy) if axis_tok == "Y" else (max(sx, sy), sz))
        axis_rot = _axis_rot_to_z(axis_tok)
        m_adj = rel_mat.copy()
        m_adj[:3, :3] = m_adj[:3, :3] @ axis_rot
        rr, hh = r * r_s, 0.5 * h * h_s
        sid = builder.add_shape_cylinder(
            body_idx,
            xform=_mat4_to_wp_transform(m_adj),
            radius=rr,
            half_height=hh,
            cfg=shape_cfg,
            label=collider_path,
        )
        if vis_out is not None:
            vis_out[sid] = ("cylinder", (rr, hh))
        return sid, axis_rot

    if prim.IsA(UsdGeom.Cone):
        cone = UsdGeom.Cone(prim)
        axis_tok = cone.GetAxisAttr().Get() or "Z"
        r = float(cone.GetRadiusAttr().Get() or 0.5)
        h = float(cone.GetHeightAttr().Get() or 1.0)
        r_s, h_s = (max(sy, sz), sx) if axis_tok == "X" else ((max(sx, sz), sy) if axis_tok == "Y" else (max(sx, sy), sz))
        axis_rot = _axis_rot_to_z(axis_tok)
        m_adj = rel_mat.copy()
        m_adj[:3, :3] = m_adj[:3, :3] @ axis_rot
        rr, hh = r * r_s, 0.5 * h * h_s
        sid = builder.add_shape_cone(
            body_idx,
            xform=_mat4_to_wp_transform(m_adj),
            radius=rr,
            half_height=hh,
            cfg=shape_cfg,
            label=collider_path,
        )
        if vis_out is not None:
            vis_out[sid] = ("cone", (rr, hh))
        return sid, axis_rot

    if prim.IsA(UsdGeom.Mesh):
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts:
            return None, axis_rot
        mesh = newton.usd.get_mesh(
            prim,
            compute_inertia=False,
            load_visual_materials=False,
            apply_stage_units=False,
        )
        if convex_meshes:
            mesh = mesh.compute_convex_hull(replace=False)
            sid = builder.add_shape_convex_hull(
                body_idx,
                xform=_mat4_to_wp_transform(rel_mat),
                mesh=mesh,
                scale=wp.vec3(sx, sy, sz),
                cfg=shape_cfg,
                label=collider_path,
            )
            if vis_out is not None:
                vis_out[sid] = ("mesh", (mesh.vertices, mesh.indices, (sx, sy, sz)))
            return sid, axis_rot
        sid = builder.add_shape_mesh(
            body_idx,
            xform=_mat4_to_wp_transform(rel_mat),
            mesh=mesh,
            scale=wp.vec3(sx, sy, sz),
            cfg=shape_cfg,
            label=collider_path,
        )
        if vis_out is not None:
            vis_out[sid] = ("mesh", (mesh.vertices, mesh.indices, (sx, sy, sz)))
        return sid, axis_rot

    return None, axis_rot


def _discover_stage_colliders(
    stage: Usd.Stage,
    roots: Sequence[str],
    exclude_prefixes: Sequence[str],
) -> list[str]:
    """`roots` 配下にある有効な `UsdPhysics.CollisionAPI` 付き `Gprim` のパスを収集する (`exclude_prefixes` 配下は除外)。"""
    excludes = tuple(p.rstrip("/") for p in exclude_prefixes if p)

    def _is_excluded(path: str) -> bool:
        return any(path == ex or path.startswith(ex + "/") for ex in excludes)

    found: list[str] = []
    seen: set[str] = set()
    for root_path in roots:
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            continue
        it = iter(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
        for prim in it:
            p_str = str(prim.GetPath())
            if _is_excluded(p_str):
                it.PruneChildren()
                continue
            if not prim.IsA(UsdGeom.Gprim):
                continue
            owner = prim
            while owner and owner.IsValid() and not owner.HasAPI(UsdPhysics.CollisionAPI):
                if owner == root and not root.HasAPI(UsdPhysics.CollisionAPI):
                    owner = None
                    break
                parent = owner.GetParent()
                if not parent or parent.IsPseudoRoot():
                    owner = None
                    break
                owner = parent
            if not owner or not owner.HasAPI(UsdPhysics.CollisionAPI):
                continue
            enabled = UsdPhysics.CollisionAPI(owner).GetCollisionEnabledAttr().Get()
            if enabled is not None and not enabled:
                continue
            if p_str not in seen:
                seen.add(p_str)
                found.append(p_str)
    return found


def _try_fabric_hierarchy(stage: Usd.Stage):
    """Fabric の `(IFabricHierarchy, usdrt)` を返す。Kit / Fabric が存在しない純 Python 環境では `(None, None)`。"""
    try:
        import usdrt
        from pxr import UsdUtils

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            return None, None
        rt_stage = usdrt.Usd.Stage.Attach(stage_id)
        hier = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            rt_stage.GetFabricId(), rt_stage.GetStageIdAsStageId()
        )
        return hier, usdrt
    except Exception:
        return None, None


def _is_timeline_playing() -> bool:
    """Kit のタイムラインが停止中でない (`Play=ON` または `Pause`) なら `True`。Kit 外では `False`。"""
    try:
        import omni.timeline

        return not omni.timeline.get_timeline_interface().is_stopped()
    except Exception:
        return False


def _has_active_rigid_body(prim: Usd.Prim) -> bool:
    """`prim` またはその祖先に有効な `UsdPhysics.RigidBodyAPI` があるかを判定する。

    `sync_environment(use_fabric="auto")` が、Play=ON 中に PhysX が Fabric 側のみを更新する
    動的剛体障害物と、ユーザースクリプトが USD 側 (`xformOp`) を直接書き換える静的障害物を見分けるために使う。
    """
    cur = prim
    while cur and cur.IsValid() and not cur.IsPseudoRoot():
        if cur.HasAPI(UsdPhysics.RigidBodyAPI):
            enabled = UsdPhysics.RigidBodyAPI(cur).GetRigidBodyEnabledAttr().Get()
            return True if enabled is None else bool(enabled)
        cur = cur.GetParent()
    return False


# ==============================================================================
# 内部管理レコードとヘッドレス用プロキシ
# ==============================================================================


@dataclass
class _RobotRecord:
    """`GeometricScene` 内でロボット 1 台分のインデックス対応を保持する管理データ。"""

    key: int
    robot: Any  # GeometricRobot または _ModelProxy
    structure_version: int
    collider_version: int = 0
    link_idx_to_body: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    dof_to_newton_q: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    root_q_start: int = 0
    articulation_id: int = 0


class _ModelProxy:
    """`GeometricRobot` を作らずに `GeometricRobotModel` 単体で `GeometricScene` を使うための軽量プロキシ。"""

    def __init__(
        self,
        stage: Usd.Stage,
        model: GeometricRobotModel,
        link_prim_paths: dict[str, str],
        *,
        name: str | None = None,
        collision_filter: GeometricCollisionFilter | None = None,
        base_matrix: np.ndarray | None = None,
    ) -> None:
        self.stage = stage
        self.model = model
        self.name = name or model.name
        self.link_path_map = dict(link_prim_paths)
        all_paths = list(self.link_path_map.values())
        self.collider_paths = {ln: find_colliders(stage, lp, all_paths) for ln, lp in self.link_path_map.items()}
        self.collision_filter = (
            collision_filter if collision_filter is not None else GeometricCollisionFilter.from_model(model)
        )
        self._base = np.eye(4) if base_matrix is None else np.array(base_matrix, dtype=float)
        self._q = model.resolve_mimic(np.zeros(model.num_dofs, dtype=float))
        self.structure_version = 0
        self.collider_version = 0
        self.root_prim_paths: list[str] = []

    def reload_colliders(self) -> dict[str, list[str]]:
        """USD から各リンクのコライダー一覧を読み直す。"""
        all_paths = list(self.link_path_map.values())
        self.collider_paths = {ln: find_colliders(self.stage, lp, all_paths) for ln, lp in self.link_path_map.items()}
        self.collider_version += 1
        return self.collider_paths

    def get_dof_positions(self, *, unit=None) -> np.ndarray:
        return self._q.copy()

    def set_pose(self, q: np.ndarray | None = None, base_matrix: np.ndarray | None = None) -> None:
        if q is not None:
            self._q = self.model.resolve_mimic(np.asarray(q, dtype=float))
        if base_matrix is not None:
            self._base = np.array(base_matrix, dtype=float)

    def _default_scope(self) -> str:
        return "all"

    @property
    def link_set(self) -> GeometricLinkSet:
        return GeometricLinkSet(self, self.model.link_names, self.name)  # type: ignore[arg-type]


@dataclass
class _EnvShapeEntry:
    """`body=-1` の環境コライダー 1 形状の追跡情報 (`sync_environment` / `set_obstacle_pose` 用)。"""

    prim_path: str
    shape_id: int
    axis_rot: np.ndarray
    manual_override: bool = False


# ==============================================================================
# GeometricScene: 単体・複数ロボット共有の自前コリジョン空間
# ==============================================================================


class GeometricScene:
    """Newton 1.6.1 を使った自前の順運動学・コリジョン空間(PhysX PhysicsScene 非依存)。

        scene = GeometricScene(stage)
        scene.add_robot(robot)
        scene.add_environment("/World/Obstacles")     # 省略時はステージ上の環境コライダーを自動検出
        pairs = scene.check_collisions(robot)         # 0 フレーム遅延(USD flush 不要)
        near  = scene.compute_clearances(robot, max_distance=0.05)

    robot.scene から自動生成されたインスタンスを使うことも、複数ロボットで 1 つの GeometricScene を
    共有してロボット同士の干渉(check_between)を 0 フレーム遅延で調べることもできる。
    """

    def __init__(
        self,
        stage: Usd.Stage,
        *,
        device: str = "auto",
        broad_phase: Literal["auto", "explicit", "nxn", "sap"] = "auto",
        convex_meshes: bool = True,
        bake_adjacent_filter: bool = False,
        auto_discover_environment: bool = True,
    ) -> None:
        self._stage = stage
        self._device_spec = device
        self._device: str | None = None
        self._broad_phase_spec = broad_phase
        self._convex_meshes = convex_meshes
        self._bake_adjacent_filter = bake_adjacent_filter
        self._auto_discover = auto_discover_environment

        self._robots: dict[int, _RobotRecord] = {}
        self._structure_callbacks: dict[int, Callable[[list[str], list[str]], None]] = {}
        self._explicit_env_roots: list[str] = []
        self._env_exclude_paths: list[str] = []
        self._use_explicit_env = False

        # Finalized Newton objects
        self._dirty = True
        self._env_resync_dirty = False
        self._colliders_dirty = False
        self._model = None
        self._state = None
        self._pipeline = None
        self._contacts = None
        self._dist_buf = None
        self._p0_buf = None
        self._p1_buf = None
        self._joint_q_np: np.ndarray | None = None
        self._joint_q_wp = None
        self._joint_qd_wp = None
        self._shape_to_owner: dict[int, tuple[str, Any, str, str]] = {}
        self._shape_vis_info: dict[int, tuple] = {}
        self._wireframe_cache: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._env_entries: list[_EnvShapeEntry] = []
        self._env_by_path: dict[str, _EnvShapeEntry] = {}
        self._current_gap: float = 0.0
        self._closed = False

        self._notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)

    # ==========================================================================
    # 状態・プロパティ
    # ==========================================================================

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def convex_meshes(self) -> bool:
        """`UsdGeom.Mesh` を単一凸包へ変換して判定するか (`True`=凸包 / `False`=元の三角形メッシュ)。"""
        return self._convex_meshes

    @convex_meshes.setter
    def convex_meshes(self, value: bool) -> None:
        flag = bool(value)
        if flag == self._convex_meshes:
            return
        self._convex_meshes = flag
        self.mark_dirty()

    @property
    def device(self) -> str:
        if self._device is None:
            self._device = resolve_newton_device(self._device_spec)
        return self._device

    @property
    def newton_model(self):
        self._ensure_finalized()
        return self._model

    @property
    def newton_state(self):
        self._ensure_finalized()
        return self._state

    @property
    def robots(self) -> list[Any]:
        return [rec.robot for rec in self._robots.values()]

    @property
    def environment_collider_paths(self) -> list[str]:
        self._ensure_finalized()
        return [e.prim_path for e in self._env_entries]

    # ==========================================================================
    # ロボット・環境コライダーの登録
    # ==========================================================================

    def add_robot(self, robot: "GeometricRobot") -> None:
        """GeometricRobot をこの空間に登録する(登録済みなら何もしない)。"""
        key = id(robot)
        if key in self._robots:
            return
        ver = getattr(robot, "structure_version", 0)
        cver = getattr(robot, "collider_version", 0)
        self._robots[key] = _RobotRecord(key=key, robot=robot, structure_version=ver, collider_version=cver)
        if hasattr(robot, "add_structure_listener"):
            cb = lambda _old, _new, _k=key: self._on_robot_structure_changed(_k)  # noqa: E731
            robot.add_structure_listener(cb)
            self._structure_callbacks[key] = cb
        self._dirty = True

    def add_model(
        self,
        model: GeometricRobotModel,
        link_prim_paths: dict[str, str],
        *,
        name: str | None = None,
        collision_filter: GeometricCollisionFilter | None = None,
        base_matrix: np.ndarray | None = None,
    ) -> _ModelProxy:
        """GeometricRobot を作らずに GeometricRobotModel を直接登録する(pure テストやヘッドレス計算向け)。"""
        proxy = _ModelProxy(
            self._stage,
            model,
            link_prim_paths,
            name=name,
            collision_filter=collision_filter,
            base_matrix=base_matrix,
        )
        self.add_robot(proxy)  # type: ignore[arg-type]
        return proxy

    def remove_robot(self, robot: Any) -> bool:
        key = id(robot)
        rec = self._robots.pop(key, None)
        if rec is None:
            return False
        cb = self._structure_callbacks.pop(key, None)
        if cb is not None and hasattr(robot, "remove_structure_listener"):
            robot.remove_structure_listener(cb)
        self._dirty = True
        return True

    def add_environment(
        self,
        prim_paths_or_root: str | Sequence[str],
        *,
        exclude_robots: bool = True,
        exclude_prim_paths: Sequence[str] = (),
    ) -> int:
        """環境コライダーの探索ルート(またはコライダー prim パス)を明示登録する。

        1 回でも add_environment() を呼ぶと、自動検出モードから明示登録モードへ切り替わる。
        戻り値は現在登録されている環境ルート数(コライダー数は finalize 後に environment_collider_paths で取得)。
        """
        paths = [prim_paths_or_root] if isinstance(prim_paths_or_root, str) else list(prim_paths_or_root)
        self._use_explicit_env = True
        self._exclude_robots_on_env = exclude_robots
        for p in paths:
            p_clean = p.rstrip("/") or "/"
            if p_clean not in self._explicit_env_roots:
                self._explicit_env_roots.append(p_clean)
        for ex in exclude_prim_paths:
            ex_clean = ex.rstrip("/")
            if ex_clean and ex_clean not in self._env_exclude_paths:
                self._env_exclude_paths.append(ex_clean)
        self._dirty = True
        return len(self._explicit_env_roots)

    def clear_environment(self) -> None:
        """明示登録された環境コライダーを消す。"""
        self._explicit_env_roots.clear()
        self._env_exclude_paths.clear()
        self._use_explicit_env = False
        self._dirty = True

    def mark_dirty(self) -> None:
        """次のクエリ時に Newton モデルを再構築させる (コライダー形状の追加後など)。"""
        self._dirty = True

    def invalidate(self) -> None:
        """全登録ロボットのコライダーキャッシュと Newton モデルを破棄し、次のクエリ時に再構築させる。

        カスタムコライダー Sublayer の挿抜・Mute/Unmute や環境コライダーの追加・削除の後に、
        キャッシュ残存を確実に防ぎたいときに呼ぶ。
        """
        for rec in self._robots.values():
            robot = rec.robot
            if isinstance(robot, _ModelProxy):
                robot.reload_colliders()
            elif hasattr(robot, "_colliders"):
                robot._colliders = None
        self._dirty = True
        self._env_resync_dirty = True
        self._colliders_dirty = False

    def _on_robot_structure_changed(self, _key: int) -> None:
        self._dirty = True

    def _on_objects_changed(self, notice, _sender) -> None:
        if self._closed or is_authoring():
            return
        if notice.GetResyncedPaths():
            self._env_resync_dirty = True
            self._colliders_dirty = True
            self._dirty = True
            return
        for path in notice.GetChangedInfoOnlyPaths():
            if path.IsPropertyPath() and path.name in _COLLIDER_ATTRS:
                self._colliders_dirty = True
                self._dirty = True
                return

    # ==========================================================================
    # Newton モデル構築 (finalize)
    # ==========================================================================

    def _robot_exclude_prefixes(self) -> list[str]:
        prefixes: list[str] = []
        for rec in self._robots.values():
            r = rec.robot
            prefixes.extend(r.link_path_map.values())
            for rp in getattr(r, "root_prim_paths", ()):
                if rp not in prefixes:
                    prefixes.append(rp)
        return prefixes

    def finalize(self) -> None:
        """登録済みロボットと環境コライダーから Newton `Model` / `State` / `CollisionPipeline` を構築する。

        設計上の重要ポイント:
            - `builder.rigid_gap = 0.0` と `ShapeConfig(margin=0.0, gap=0.0)` で構築する理由:
              Newton は既定で正の `rigid_gap` (接触検出マージン) を持つ。ここで 0 に揃えておけば、
              通常の干渉チェック (`clearance=0.0`) では正確にめり込み (`separation <= 0`) のみを検出し、
              クリアランス計測 (`max_distance > 0`) のときだけ実行時に `self._model.shape_gap` を
              `0.5 * max_distance + 1e-4` へ切り替えることで、モデル再構築なしに両用途を両立できる。
            - `order = list(model._c["order"])` (BFS トポロジカル順) でリンクと関節を登録する理由:
              `newton.eval_fk` は親から子へ向かうトポロジカル順の関節木を前提とする。一方
              `GeometricRobotModel.dof_names` はユーザー指定や USD 読取順で任意の並びになり得るため、
              `finalize()` 後に `model.joint_q_start` から `rec.dof_to_newton_q` 置換テーブルを生成する。
            - 根リンクへ `builder.add_joint_free(parent=-1, child=root_body)` を入れる理由:
              根リンクのワールド姿勢を `joint_q` の 7 座標 `[px, py, pz, qx, qy, qz, qw]` として与えられるため、
              `robot.set_world_poses()` でロボット全体を動かしても Newton モデルを再構築せずに FK へ反映できる。
            - `lock_inertia=True` と単位慣性テンソルを渡す理由:
              本クラスはダイナミクス積分を行わず `eval_fk` と `CollisionPipeline` だけを使うため、
              ダミー質量・慣性で Newton のゼロ慣性バリデーションエラーを回避する。
            - `include_static_kinematic_pairs=True` を渡す理由:
              全ロボット剛体は FK 駆動 (運動学的) であり、環境障害物は `body=-1` の静的シェイプのため、
              この設定を付けないとロボットと静的環境障害物のペアがブロードフェーズで除外される。
        """
        newton, wp = _import_newton()
        dev = self.device

        builder = newton.ModelBuilder(up_axis=_stage_up_axis(self._stage), gravity=wp.vec3(0.0, 0.0, 0.0))
        builder.rigid_gap = 0.0
        shape_cfg = newton.ModelBuilder.ShapeConfig(
            margin=0.0,
            gap=0.0,
            collision_filter_parent=self._bake_adjacent_filter,
        )
        unit_inertia = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        self._shape_to_owner = {}
        self._shape_vis_info = {}
        self._wireframe_cache = {}
        self._env_entries = []
        self._env_by_path = {}

        pending_robots: list[tuple[_RobotRecord, int, dict[int, int]]] = []

        for art_idx, rec in enumerate(self._robots.values()):
            robot = rec.robot
            if hasattr(robot, "_ensure_current"):
                robot._ensure_current()
            model: GeometricRobotModel = robot.model
            rec.structure_version = getattr(robot, "structure_version", 0)
            rec.collider_version = getattr(robot, "collider_version", 0)
            rec.articulation_id = art_idx

            # BFS トポロジカル順でリンク剛体と関節を登録する (親が必ず子より先になる)
            order = list(model._c["order"])
            parent_joint = model._c["parent_joint"]

            link_idx_to_body = np.full(model.num_links, -1, dtype=int)
            link_world_at_build: dict[int, np.ndarray] = {}
            for li in order:
                lname = model.links[li].name
                lpath = robot.link_path_map.get(lname)
                if lpath:
                    prim = self._stage.GetPrimAtPath(lpath)
                    link_world_at_build[li] = (
                        split_scale(_np(xc.GetLocalToWorldTransform(prim)))[0] if prim and prim.IsValid() else np.eye(4)
                    )
                else:
                    link_world_at_build[li] = np.eye(4)

            for li in order:
                lname = model.links[li].name
                bid = builder.add_link(
                    xform=_mat4_to_wp_transform(link_world_at_build[li]),
                    mass=1.0,
                    inertia=unit_inertia,
                    lock_inertia=True,
                    label=f"{robot.name}/{lname}",
                )
                link_idx_to_body[li] = bid
                for col_path in robot.collider_paths.get(lname, ()):
                    sid, _ = _add_usd_collider_shape(
                        builder,
                        bid,
                        self._stage,
                        col_path,
                        link_world_at_build[li],
                        shape_cfg,
                        xc,
                        convex_meshes=self._convex_meshes,
                        vis_out=self._shape_vis_info,
                    )
                    if sid is not None:
                        self._shape_to_owner[sid] = ("robot", robot, lname, col_path)

            # 根リンクに FREE 関節 (7 座標: [px, py, pz, qx, qy, qz, qw]) を追加し、
            # `robot.set_world_poses()` による動的ベース移動をモデル再構築なしで `joint_q` へ反映する。
            root_body = int(link_idx_to_body[order[0]])
            root_jid = builder.add_joint_free(
                parent=-1,
                child=root_body,
                label=f"{robot.name}/root_free",
                collision_filter_parent=False,
            )
            art_joints = [root_jid]
            joint_idx_to_newton_joint: dict[int, int] = {}

            for li in order[1:]:
                j_idx = parent_joint[li]
                js = model.joints[j_idx]
                pb = int(link_idx_to_body[model.get_link_index(js.parent_link)])
                cb = int(link_idx_to_body[li])
                px = _mat4_to_wp_transform(js.parent_frame)
                cx = _mat4_to_wp_transform(js.child_frame)
                if js.joint_type == "revolute":
                    nj = builder.add_joint_revolute(
                        parent=pb,
                        child=cb,
                        parent_xform=px,
                        child_xform=cx,
                        axis=wp.vec3(*js.axis),
                        label=f"{robot.name}/{js.name}",
                        collision_filter_parent=self._bake_adjacent_filter,
                    )
                elif js.joint_type == "prismatic":
                    nj = builder.add_joint_prismatic(
                        parent=pb,
                        child=cb,
                        parent_xform=px,
                        child_xform=cx,
                        axis=wp.vec3(*js.axis),
                        label=f"{robot.name}/{js.name}",
                        collision_filter_parent=self._bake_adjacent_filter,
                    )
                else:
                    nj = builder.add_joint_fixed(
                        parent=pb,
                        child=cb,
                        parent_xform=px,
                        child_xform=cx,
                        label=f"{robot.name}/{js.name}",
                        collision_filter_parent=self._bake_adjacent_filter,
                    )
                art_joints.append(nj)
                joint_idx_to_newton_joint[j_idx] = nj

            builder.add_articulation(art_joints, label=robot.name)
            rec.link_idx_to_body = link_idx_to_body
            pending_robots.append((rec, root_jid, joint_idx_to_newton_joint))

        # 環境コライダーを `body=-1` の静的シェイプとして追加する。
        # 実行中の姿勢変更は `model.shape_transform` をその場更新するため、モデル再構築は不要。
        env_paths: list[str] = []
        if self._use_explicit_env:
            excludes = list(self._env_exclude_paths)
            if getattr(self, "_exclude_robots_on_env", True):
                excludes.extend(self._robot_exclude_prefixes())
            env_paths = _discover_stage_colliders(self._stage, self._explicit_env_roots, excludes)
        elif self._auto_discover:
            roots = ["/World"] if self._stage.GetPrimAtPath("/World").IsValid() else ["/"]
            env_paths = _discover_stage_colliders(self._stage, roots, self._robot_exclude_prefixes())

        for col_path in env_paths:
            sid, axis_rot = _add_usd_collider_shape(
                builder,
                -1,
                self._stage,
                col_path,
                np.eye(4),
                shape_cfg,
                xc,
                convex_meshes=self._convex_meshes,
                vis_out=self._shape_vis_info,
            )
            if sid is not None:
                entry = _EnvShapeEntry(prim_path=col_path, shape_id=sid, axis_rot=axis_rot)
                self._env_entries.append(entry)
                self._env_by_path[col_path] = entry
                self._shape_to_owner[sid] = ("env", None, col_path, col_path)

        self._model = builder.finalize(device=dev)
        self._model.shape_gap.zero_()
        self._current_gap = 0.0
        self._state = self._model.state()

        bp = self._broad_phase_spec
        if bp == "auto":
            bp = "explicit" if int(self._model.shape_count) <= 512 else "sap"
        self._pipeline = newton.CollisionPipeline(
            self._model,
            broad_phase=bp,
            include_static_kinematic_pairs=True,
            verify_buffers=False,
        )
        self._contacts = self._pipeline.contacts()
        cmax = int(self._contacts.rigid_contact_max)
        self._dist_buf = wp.zeros(cmax, dtype=wp.float32, device=dev)
        self._p0_buf = wp.zeros(cmax, dtype=wp.vec3, device=dev)
        self._p1_buf = wp.zeros(cmax, dtype=wp.vec3, device=dev)

        # `GeometricRobotModel` の DOF インデックス -> Newton の `joint_q` 配列インデックス置換テーブルを作成
        q_starts = self._model.joint_q_start.numpy()
        for rec, root_jid, j2nj in pending_robots:
            model = rec.robot.model
            dof_of_joint = model._c["dof_of_joint"]
            rec.root_q_start = int(q_starts[root_jid])
            d2q = np.full(model.num_dofs, -1, dtype=int)
            for j_idx, d_idx in dof_of_joint.items():
                d2q[d_idx] = int(q_starts[j2nj[j_idx]])
            rec.dof_to_newton_q = d2q

        self._joint_q_np = self._model.joint_q.numpy().astype(np.float32)
        self._joint_q_wp = wp.array(self._joint_q_np, dtype=wp.float32, device=dev)
        self._joint_qd_wp = wp.zeros(self._model.joint_dof_count, dtype=wp.float32, device=dev)
        self._dirty = False
        self._env_resync_dirty = False
        self._colliders_dirty = False

    def _ensure_finalized(self) -> None:
        if self._closed:
            raise RuntimeError("GeometricScene はすでに close() されている")
        for rec in self._robots.values():
            if hasattr(rec.robot, "_ensure_current"):
                rec.robot._ensure_current()
            if getattr(rec.robot, "structure_version", 0) != rec.structure_version:
                self._dirty = True
            if getattr(rec.robot, "collider_version", 0) != rec.collider_version:
                self._dirty = True
        if self._colliders_dirty:
            for rec in self._robots.values():
                robot = rec.robot
                if isinstance(robot, _ModelProxy):
                    robot.reload_colliders()
                elif hasattr(robot, "_colliders"):
                    robot._colliders = None
            self._colliders_dirty = False
            self._dirty = True
        if self._env_resync_dirty and (self._auto_discover or self._use_explicit_env):
            self._dirty = True
        if self._dirty or self._model is None:
            self.finalize()

    # ==========================================================================
    # 姿勢同期と順運動学 (newton.eval_fk)
    # ==========================================================================

    def _write_robot_q_to_buffer(
        self,
        rec: _RobotRecord,
        q: np.ndarray | None = None,
        base_matrix: np.ndarray | None = None,
        *,
        resolve_mimic: bool = True,
    ) -> None:
        """1 台分のベース姿勢と関節値を CPU ステージング配列 `self._joint_q_np` へ書き込む。

        なぜ `resolve_mimic=True` の前処理が必要か:
            `ModelBuilder` には各 DOF 関節を独立な 1-DOF 関節として登録しており、USD の
            `PhysxMimicJointAPI` 制約は Newton 側へ持たせていない。そのため `newton.eval_fk` を呼ぶ前に
            `model.resolve_mimic(q)` で従動軸の値を解決しておく必要がある
            (`GeometricRobot.set_dof_positions` 経由で解決済みの場合は `resolve_mimic=False` で重複計算を省く)。
        """
        assert self._joint_q_np is not None
        robot = rec.robot
        base = robot._base if base_matrix is None else base_matrix
        q_val = robot._q if q is None else np.asarray(q, dtype=float)
        if resolve_mimic:
            q_val = robot.model.resolve_mimic(q_val)
        p = base[:3, 3]
        # `tf.matrix_to_quat` は `(w, x, y, z)`、Newton の FREE 関節 `joint_q` は `(x, y, z, w)`
        w, x, y, z = tf.matrix_to_quat(base[:3, :3])
        r0 = rec.root_q_start
        self._joint_q_np[r0 : r0 + 3] = p
        self._joint_q_np[r0 + 3 : r0 + 7] = (x, y, z, w)
        if len(rec.dof_to_newton_q):
            self._joint_q_np[rec.dof_to_newton_q] = q_val.astype(np.float32)

    def sync_robots(self) -> None:
        """全登録ロボットの現在の関節値・ベース姿勢を Newton へ転送し、`newton.eval_fk` を実行する。"""
        self._ensure_finalized()
        if not self._robots:
            return
        newton, wp = _import_newton()
        for rec in self._robots.values():
            self._write_robot_q_to_buffer(rec, resolve_mimic=False)
        wp.copy(self._joint_q_wp, wp.array(self._joint_q_np, dtype=wp.float32, device="cpu"))
        newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)

    def eval_fk(
        self,
        robot: Any,
        q: np.ndarray | None = None,
        *,
        base_matrix: np.ndarray | None = None,
        resolve_mimic: bool = True,
    ) -> np.ndarray:
        """指定ロボットについて `newton.eval_fk` を実行し、全リンクのワールド行列 `(L, 4, 4)` (列ベクトル形式) を返す。"""
        if id(robot) not in self._robots:
            self.add_robot(robot)
        self._ensure_finalized()
        newton, wp = _import_newton()
        rec = self._robots[id(robot)]
        self._write_robot_q_to_buffer(rec, q=q, base_matrix=base_matrix, resolve_mimic=resolve_mimic)
        wp.copy(self._joint_q_wp, wp.array(self._joint_q_np, dtype=wp.float32, device="cpu"))
        newton.eval_fk(self._model, self._joint_q_wp, self._joint_qd_wp, self._state)
        body_q_np = self._state.body_q.numpy()
        return _wp_body_q_to_mat4(body_q_np[rec.link_idx_to_body])

    def get_robot_body_indices(self, robot: Any) -> np.ndarray:
        """`GeometricRobotModel.links` の並びに対応する Newton body インデックス配列を返す。"""
        if id(robot) not in self._robots:
            self.add_robot(robot)
        self._ensure_finalized()
        return self._robots[id(robot)].link_idx_to_body.copy()

    # ==========================================================================
    # 環境障害物の動的同期 (モデル再構築なし・0 フレーム遅延)
    # ==========================================================================

    def sync_environment(
        self,
        *,
        use_fabric: bool | Literal["auto"] = "auto",
        reset_manual_overrides: bool = True,
    ) -> None:
        """環境コライダーの最新ワールド姿勢を USD / Fabric から読み、`model.shape_transform` をその場更新する。

        なぜ `use_fabric="auto"` (既定) で対象 prim ごとに読み元を切り替えるのか:
            - `Play=ON` 中に PhysX が落下・移動させている動的剛体障害物 (有効な `UsdPhysics.RigidBodyAPI` を持つ) は、
              最新姿勢が Fabric にのみ反映され USD には書き戻されない -> Fabric から読む必要がある。
            - 一方、`RigidBodyAPI` を持たない静的障害物 (ユーザースクリプトが USD の `xformOp:translate` 等を書き換え、
              `app.update()` を挟まずにすぐ `check_collisions()` を呼ぶケース) は、Fabric に古い値が残ったままになる ->
              USD `XformCache` から読む必要がある。
            - `Stop` 中は PhysX が動いておらず USD が正本 -> USD `XformCache` から読む。
            この切り分けにより、動的障害物・静的障害物の両方に対して 0 フレーム遅延で最新姿勢を取り込める。
        """
        self._ensure_finalized()
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
        changed = False
        for entry in self._env_entries:
            if entry.manual_override:
                if reset_manual_overrides:
                    entry.manual_override = False
                else:
                    continue
            prim = self._stage.GetPrimAtPath(entry.prim_path)
            if not prim or not prim.IsValid():
                self._dirty = True
                return
            read_fabric = (use_fabric is True) or (use_fabric == "auto" and playing and _has_active_rigid_body(prim))
            world_m = None
            if read_fabric and hier is not None and usdrt is not None:
                try:
                    rt_p = usdrt.Sdf.Path(entry.prim_path)
                    # Fabric の `get_world_xform` は行ベクトル形式のため `.T` で列ベクトル形式へ揃える
                    fmat = np.array(hier.get_world_xform(rt_p), dtype=float).reshape(4, 4).T
                    if np.all(np.isfinite(fmat)) and np.linalg.norm(fmat[:3, :3]) > 1e-12:
                        world_m = fmat
                except Exception:
                    world_m = None
            if world_m is None:
                world_m = _np(xc.GetLocalToWorldTransform(prim))
            rigid, _ = split_scale(world_m)
            # USD の X/Y 軸カプセル等に対する軸補正 `entry.axis_rot` を掛け直してから書き戻す
            rot = rigid[:3, :3] @ entry.axis_rot
            p = rigid[:3, 3]
            w, x, y, z = tf.matrix_to_quat(rot)
            row = np.array([p[0], p[1], p[2], x, y, z, w], dtype=np.float32)
            st_np[entry.shape_id] = row
            changed = True
        if changed:
            wp.copy(self._model.shape_transform, wp.array(st_np, dtype=wp.transform, device="cpu"))

    def set_obstacle_pose(
        self,
        prim_path: str,
        position: Sequence[float],
        orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """環境コライダー `prim_path` (またはその親 Xform 配下) の姿勢を `model.shape_transform` へ直接書き込む。

        `orientation` は `(w, x, y, z)`。USD や Fabric を書き換えずに高速に障害物を移動して干渉判定したいときに使う。
        親 `Xform` パスを渡した場合は、配下の各子コライダーについて USD 上の相対剛体変換
        `rel = invert(target_world) @ child_world` を求め、`obs_mat @ rel` でローカルオフセット (並進・回転) を保持する。
        設定されたエントリには `manual_override = True` が立ち、自動 `sync_environment(reset_manual_overrides=False)`
        で上書きされない。USD/Fabric へ戻したいときは明示的に `sync_environment()` を呼ぶ。
        """
        self._ensure_finalized()
        _, wp = _import_newton()
        target = prim_path.rstrip("/")
        matched = [e for e in self._env_entries if e.prim_path == target or e.prim_path.startswith(target + "/")]
        if not matched:
            raise KeyError(f"環境コライダー {prim_path!r} は GeometricScene に登録されていない")
        pos = np.asarray(position, dtype=float).reshape(3)
        rot_base = tf.quat_to_matrix(np.asarray(orientation, dtype=float).reshape(4))
        obs_mat = tf.make_transform(rot_base, pos)
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        target_prim = self._stage.GetPrimAtPath(target)
        target_inv = (
            tf.invert(split_scale(_np(xc.GetLocalToWorldTransform(target_prim)))[0])
            if target_prim and target_prim.IsValid()
            else np.eye(4)
        )
        st_np = self._model.shape_transform.numpy()
        for entry in matched:
            if entry.prim_path == target:
                child_mat = obs_mat
            else:
                cprim = self._stage.GetPrimAtPath(entry.prim_path)
                if cprim and cprim.IsValid():
                    child_world = split_scale(_np(xc.GetLocalToWorldTransform(cprim)))[0]
                    rel = target_inv @ child_world
                    child_mat = obs_mat @ rel
                else:
                    child_mat = obs_mat
            rot = child_mat[:3, :3] @ entry.axis_rot
            p_c = child_mat[:3, 3]
            w, x, y, z = tf.matrix_to_quat(rot)
            st_np[entry.shape_id] = np.array([p_c[0], p_c[1], p_c[2], x, y, z, w], dtype=np.float32)
            entry.manual_override = True
        wp.copy(self._model.shape_transform, wp.array(st_np, dtype=wp.transform, device="cpu"))

    # ==========================================================================
    # 干渉チェックとクリアランス計測 (CollisionPipeline + eval_rigid_contact_kinematics)
    # ==========================================================================

    def _set_gap(self, max_distance: float) -> None:
        """クエリ距離 `max_distance` に応じて `self._model.shape_gap` を実行時に切り替える。

        Newton のペア検出距離は `shape_gap[a] + shape_gap[b]` で決まる。
        そのため `max_distance <= 0` では `0.0` (めり込みのみ)、`max_distance > 0` では
        各シェイプに `0.5 * max_distance + 1e-4` を設定して合計 `max_distance + 2e-4` 以内のペアを
        確実に狭域判定へ通し、直後の `sep <= threshold` で正確な境界切り出しを行う。
        """
        target = 0.0 if max_distance <= 0.0 else (0.5 * float(max_distance) + 1e-4)
        if abs(target - self._current_gap) > 1e-9:
            if target == 0.0:
                self._model.shape_gap.zero_()
            else:
                self._model.shape_gap.fill_(target)
            self._current_gap = target

    def _run_collide(
        self,
        *,
        max_distance: float = 0.0,
        sync_robots: bool = True,
        sync_env: bool = True,
    ) -> list[tuple[int, int, float, np.ndarray, np.ndarray, np.ndarray]]:
        """`CollisionPipeline.collide` + `eval_rigid_contact_kinematics` を実行し、条件を満たす生コンタクト列を返す。

        各要素: `(shape0, shape1, separation, surf_point0_world, surf_point1_world, normal_0_to_1)`。
        """
        self._ensure_finalized()
        if sync_robots:
            self.sync_robots()
        if sync_env and self._env_entries:
            # `set_obstacle_pose()` で手動設定した障害物姿勢を勝手に消さないよう `reset_manual_overrides=False`
            self.sync_environment(reset_manual_overrides=False)

        newton, _ = _import_newton()
        self._set_gap(max_distance)
        self._pipeline.collide(self._state, self._contacts)
        # Newton の `rigid_contact_count` はバッファ上限 `rigid_contact_max` を超えてもアトミック加算され続ける。
        # クランプせずにスライスすると範囲外参照や破壊データ読み取りにつながるため、必ず `min(..., rigid_contact_max)` で制限する。
        n = min(int(self._contacts.rigid_contact_count.numpy()[0]), int(self._contacts.rigid_contact_max))
        if n == 0:
            return []
        newton.eval_rigid_contact_kinematics(
            self._model,
            self._state,
            self._contacts,
            out_distance=self._dist_buf,
            out_point0_world=self._p0_buf,
            out_point1_world=self._p1_buf,
        )
        s0 = self._contacts.rigid_contact_shape0.numpy()[:n]
        s1 = self._contacts.rigid_contact_shape1.numpy()[:n]
        dist = self._dist_buf.numpy()[:n]
        p0_core = self._p0_buf.numpy()[:n]
        p1_core = self._p1_buf.numpy()[:n]
        normals = self._contacts.rigid_contact_normal.numpy()[:n]
        m0 = self._contacts.rigid_contact_margin0.numpy()[:n]
        m1 = self._contacts.rigid_contact_margin1.numpy()[:n]

        threshold = float(max_distance)
        out = []
        for i in range(n):
            a, b = int(s0[i]), int(s1[i])
            if a < 0 or b < 0:
                continue
            sep = float(dist[i])
            if sep <= threshold:
                nrm = normals[i]
                # `eval_rigid_contact_kinematics` が返す `p0_core` / `p1_core` はマージン差し引き後のコア点。
                # 実際のコライダー表面座標を得るため、法線方向に各形状のマージン (`m0`, `m1`) を復元する。
                pt0 = p0_core[i] + float(m0[i]) * nrm
                pt1 = p1_core[i] - float(m1[i]) * nrm
                out.append((a, b, sep, pt0, pt1, nrm))
        return out

    @staticmethod
    def _make_robot_pair(
        robot_a,
        link_a: str,
        robot_b,
        link_b: str,
        sep: float,
        pt_a: np.ndarray,
        pt_b: np.ndarray,
        normal: np.ndarray,
    ) -> GeometricCollisionPair:
        comp_a = robot_a.model.component_of_link(link_a)
        comp_b = robot_b.model.component_of_link(link_b)
        if robot_a is not robot_b:
            kind: PairKind = "between_robots"
        elif comp_a == comp_b:
            kind = "within_component"
        else:
            kind = "between_components"
        return GeometricCollisionPair(
            link_a=link_a,
            link_b=link_b,
            path_a=robot_a.link_path_map.get(link_a, link_a),
            path_b=robot_b.link_path_map.get(link_b, link_b),
            kind=kind,
            component_a=comp_a,
            component_b=comp_b,
            robot_a=robot_a.name,
            robot_b=robot_b.name,
            separation=sep,
            point_a=(float(pt_a[0]), float(pt_a[1]), float(pt_a[2])),
            point_b=(float(pt_b[0]), float(pt_b[1]), float(pt_b[2])),
            normal=(float(normal[0]), float(normal[1]), float(normal[2])),
        )

    @staticmethod
    def _make_env_pair(
        robot,
        link_a: str,
        env_path: str,
        sep: float,
        pt_a: np.ndarray,
        pt_b: np.ndarray,
        normal: np.ndarray,
    ) -> GeometricCollisionPair:
        return GeometricCollisionPair(
            link_a=link_a,
            link_b=None,
            path_a=robot.link_path_map.get(link_a, link_a),
            path_b=env_path,
            kind="environment",
            component_a=robot.model.component_of_link(link_a),
            component_b=None,
            robot_a=robot.name,
            robot_b=None,
            separation=sep,
            point_a=(float(pt_a[0]), float(pt_a[1]), float(pt_a[2])),
            point_b=(float(pt_b[0]), float(pt_b[1]), float(pt_b[2])),
            normal=(float(normal[0]), float(normal[1]), float(normal[2])),
        )

    def _filter_within(
        self,
        raw_hits,
        link_set: GeometricLinkSet,
        *,
        collision_filter: GeometricCollisionFilter | None = None,
        extra_filter: PairFilter | None = None,
    ) -> list[GeometricCollisionPair]:
        robot = link_set.robot
        flt = robot.collision_filter if collision_filter is None else collision_filter
        members = set(link_set.link_names)
        best: dict[frozenset[str], GeometricCollisionPair] = {}
        for sa, sb, sep, pt0, pt1, nrm in raw_hits:
            oa, ob = self._shape_to_owner.get(sa), self._shape_to_owner.get(sb)
            if oa is None or ob is None or oa[0] != "robot" or ob[0] != "robot":
                continue
            if oa[1] is not robot or ob[1] is not robot:
                continue
            la, lb = oa[2], ob[2]
            if la == lb or la not in members or lb not in members:
                continue
            if flt.is_ignored(la, lb) or (extra_filter and (extra_filter(la, lb) or extra_filter(lb, la))):
                continue
            if la > lb:
                la, lb = lb, la
                pt_a, pt_b, n_ab = pt1, pt0, -nrm
            else:
                pt_a, pt_b, n_ab = pt0, pt1, nrm
            key = frozenset((la, lb))
            prev = best.get(key)
            if prev is None or (prev.separation is not None and sep < prev.separation):
                best[key] = self._make_robot_pair(robot, la, robot, lb, sep, pt_a, pt_b, n_ab)
        return sorted(best.values(), key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))

    def _filter_between(
        self,
        raw_hits,
        set_a: GeometricLinkSet,
        set_b: GeometricLinkSet,
        *,
        extra_filter: PairFilter | None = None,
    ) -> list[GeometricCollisionPair]:
        ra, rb = set_a.robot, set_b.robot
        same_robot = ra is rb
        ma, mb = set(set_a.link_names), set(set_b.link_names)
        best: dict[tuple[str, str], GeometricCollisionPair] = {}
        for sa, sb, sep, pt0, pt1, nrm in raw_hits:
            oa, ob = self._shape_to_owner.get(sa), self._shape_to_owner.get(sb)
            if oa is None or ob is None or oa[0] != "robot" or ob[0] != "robot":
                continue
            if oa[1] is ra and ob[1] is rb and oa[2] in ma and ob[2] in mb:
                la, lb, pt_a, pt_b, n_ab = oa[2], ob[2], pt0, pt1, nrm
            elif ob[1] is ra and oa[1] is rb and ob[2] in ma and oa[2] in mb:
                la, lb, pt_a, pt_b, n_ab = ob[2], oa[2], pt1, pt0, -nrm
            else:
                continue
            if same_robot and (la == lb or ra.collision_filter.is_ignored(la, lb)):
                continue
            if extra_filter and (extra_filter(la, lb) or extra_filter(lb, la)):
                continue
            key = (la, lb)
            prev = best.get(key)
            if prev is None or (prev.separation is not None and sep < prev.separation):
                best[key] = self._make_robot_pair(ra, la, rb, lb, sep, pt_a, pt_b, n_ab)
        return sorted(best.values(), key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))

    def _filter_environment(
        self,
        raw_hits,
        link_set: GeometricLinkSet,
        *,
        exclude: Sequence[GeometricLinkSet] = (),
        ignore_prim_paths: Sequence[str] = (),
    ) -> list[GeometricCollisionPair]:
        robot = link_set.robot
        excluded_robots = {id(robot)} | {id(s.robot) for s in exclude}
        members = set(link_set.link_names)
        ignore = tuple(p.rstrip("/") for p in ignore_prim_paths)
        best: dict[tuple[str, str], GeometricCollisionPair] = {}
        for sa, sb, sep, pt0, pt1, nrm in raw_hits:
            oa, ob = self._shape_to_owner.get(sa), self._shape_to_owner.get(sb)
            if oa is None or ob is None:
                continue
            if oa[0] == "robot" and oa[1] is robot and oa[2] in members:
                if ob[0] == "robot" and id(ob[1]) in excluded_robots:
                    continue
                lname, env_path = oa[2], ob[3]
                pt_a, pt_b, n_ab = pt0, pt1, nrm
            elif ob[0] == "robot" and ob[1] is robot and ob[2] in members:
                if oa[0] == "robot" and id(oa[1]) in excluded_robots:
                    continue
                lname, env_path = ob[2], oa[3]
                pt_a, pt_b, n_ab = pt1, pt0, -nrm
            else:
                continue
            if any(env_path == p or env_path.startswith(p + "/") for p in ignore):
                continue
            key = (lname, env_path)
            prev = best.get(key)
            if prev is None or (prev.separation is not None and sep < prev.separation):
                best[key] = self._make_env_pair(robot, lname, env_path, sep, pt_a, pt_b, n_ab)
        return sorted(best.values(), key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))

    def check_within(
        self,
        link_set: GeometricLinkSet,
        *,
        clearance: float = 0.0,
        collision_filter: GeometricCollisionFilter | None = None,
        extra_filter: PairFilter | None = None,
        sync_env: bool = False,
    ) -> list[GeometricCollisionPair]:
        if id(link_set.robot) not in self._robots:
            self.add_robot(link_set.robot)
        raw = self._run_collide(max_distance=clearance, sync_robots=True, sync_env=sync_env)
        return self._filter_within(raw, link_set, collision_filter=collision_filter, extra_filter=extra_filter)

    def check_between(
        self,
        set_a: GeometricLinkSet,
        set_b: GeometricLinkSet,
        *,
        clearance: float = 0.0,
        extra_filter: PairFilter | None = None,
        sync_env: bool = False,
    ) -> list[GeometricCollisionPair]:
        if id(set_a.robot) not in self._robots:
            self.add_robot(set_a.robot)
        if id(set_b.robot) not in self._robots:
            self.add_robot(set_b.robot)
        raw = self._run_collide(max_distance=clearance, sync_robots=True, sync_env=sync_env)
        return self._filter_between(raw, set_a, set_b, extra_filter=extra_filter)

    def check_environment(
        self,
        link_set: GeometricLinkSet,
        *,
        clearance: float = 0.0,
        exclude: Sequence[GeometricLinkSet] = (),
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
    ) -> list[GeometricCollisionPair]:
        if id(link_set.robot) not in self._robots:
            self.add_robot(link_set.robot)
        raw = self._run_collide(max_distance=clearance, sync_robots=True, sync_env=sync_env)
        return self._filter_environment(raw, link_set, exclude=exclude, ignore_prim_paths=ignore_prim_paths)

    def touching_pairs(self, link_set: GeometricLinkSet, *, clearance: float = 0.0) -> list[GeometricCollisionPair]:
        """`collision_filter` を空にして現在重なっている組をすべて返す (診断用; `ignore_touching_pairs` は既存フィルタ適用済みの `check_within` を使う)。"""
        return self.check_within(link_set, clearance=clearance, collision_filter=GeometricCollisionFilter())

    def check_collisions(
        self,
        robot: Any | None = None,
        *,
        link_set: GeometricLinkSet | None = None,
        scope: CollisionScope | None = None,
        clearance: float = 0.0,
        collision_filter: GeometricCollisionFilter | None = None,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
    ) -> list[GeometricCollisionPair]:
        """ロボット(または link_set)の干渉を 1 回の collide パスでまとめて調べる(0 フレーム遅延)。"""
        if link_set is None:
            if robot is None:
                if len(self._robots) == 1:
                    robot = next(iter(self._robots.values())).robot
                else:
                    raise ValueError("ロボットが複数登録されているときは robot= または link_set= を指定する")
            link_set = robot.link_set
        target_robot = link_set.robot
        if id(target_robot) not in self._robots:
            self.add_robot(target_robot)

        resolved_scope = target_robot._default_scope() if scope is None else scope
        if resolved_scope not in get_args(CollisionScope):
            raise ValueError(f"scope は {get_args(CollisionScope)} のどれか: {resolved_scope!r}")

        need_env = resolved_scope in ("environment", "all")
        raw = self._run_collide(max_distance=clearance, sync_robots=True, sync_env=sync_env and need_env)
        out: list[GeometricCollisionPair] = []
        if resolved_scope in ("within", "all"):
            out.extend(self._filter_within(raw, link_set, collision_filter=collision_filter, extra_filter=extra_filter))
        if need_env:
            out.extend(self._filter_environment(raw, link_set, ignore_prim_paths=ignore_prim_paths))
        return out

    def compute_clearances(
        self,
        robot: Any | None = None,
        *,
        link_set: GeometricLinkSet | None = None,
        scope: CollisionScope | None = None,
        max_distance: float = 0.05,
        collision_filter: GeometricCollisionFilter | None = None,
        extra_filter: PairFilter | None = None,
        ignore_prim_paths: Sequence[str] = (),
        sync_env: bool = True,
    ) -> list[GeometricCollisionPair]:
        """max_distance [m] 以内にある組と符号付き距離(separation)を近い順(負=めり込みが先頭)で返す。"""
        if max_distance < 0.0:
            raise ValueError(f"max_distance は 0 以上: {max_distance}")
        pairs = self.check_collisions(
            robot=robot,
            link_set=link_set,
            scope=scope,
            clearance=max_distance,
            collision_filter=collision_filter,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
            sync_env=sync_env,
        )
        return sorted(pairs, key=lambda p: (float("inf") if p.separation is None else p.separation, p.kind, p.link_a))

    # ==========================================================================
    # デバッグ専用: Newton 内部コライダー形状・接触点の DebugDraw 可視化
    # ==========================================================================
    # 配色規約 (PhysX 標準の緑 / ピンク / 黄色と衝突しない組み合わせ):
    #   - 通常コライダー (ワイヤーフレーム): シアン (0.0, 0.9, 1.0, 1.0)
    #   - 干渉中コライダー (ワイヤーフレーム): 赤    (1.0, 0.15, 0.15, 1.0)
    #   - 接触点マーカー & めり込み線分    : オレンジ (1.0, 0.55, 0.0, 1.0)

    _COLOR_NORMAL = (0.0, 0.9, 1.0, 1.0)
    _COLOR_COLLIDING = (1.0, 0.15, 0.15, 1.0)
    _COLOR_CONTACT = (1.0, 0.55, 0.0, 1.0)

    @staticmethod
    def _build_local_wireframe(vis_spec: tuple, *, max_mesh_edges: int = 24000) -> tuple[np.ndarray, np.ndarray]:
        """Newton 登録済み形状仕様から形状ローカル座標系の線分 `(starts (E, 3), ends (E, 3))` を生成する。"""
        kind, params = vis_spec
        if kind == "box":
            hx, hy, hz = params
            corners = np.array(
                [
                    [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
                    [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
                ],
                dtype=np.float32,
            )
            edges = np.array(
                [
                    [0, 1], [1, 2], [2, 3], [3, 0],
                    [4, 5], [5, 6], [6, 7], [7, 4],
                    [0, 4], [1, 5], [2, 6], [3, 7],
                ],
                dtype=int,
            )
            return corners[edges[:, 0]], corners[edges[:, 1]]

        if kind == "sphere":
            (r,) = params
            n = 20
            ang = np.linspace(0.0, 2.0 * np.pi, n + 1, dtype=np.float32)
            c, s = r * np.cos(ang), r * np.sin(ang)
            z = np.zeros_like(c)
            rings = [
                np.stack([c, s, z], axis=-1),
                np.stack([c, z, s], axis=-1),
                np.stack([z, c, s], axis=-1),
            ]
            starts = np.concatenate([ring[:-1] for ring in rings], axis=0)
            ends = np.concatenate([ring[1:] for ring in rings], axis=0)
            return starts, ends

        if kind == "capsule":
            r, hh = params
            n = 16
            ang = np.linspace(0.0, 2.0 * np.pi, n + 1, dtype=np.float32)
            c, s = r * np.cos(ang), r * np.sin(ang)
            top = np.stack([c, s, np.full_like(c, hh)], axis=-1)
            bot = np.stack([c, s, np.full_like(c, -hh)], axis=-1)
            # 上下リング + 4 本の側稜 + XZ/YZ 半円キャップ
            semi = np.linspace(0.0, np.pi, n // 2 + 1, dtype=np.float32)
            sc, ss = r * np.cos(semi), r * np.sin(semi)
            z0 = np.zeros_like(sc)
            cap_xz_top = np.stack([sc, z0, hh + ss], axis=-1)
            cap_yz_top = np.stack([z0, sc, hh + ss], axis=-1)
            cap_xz_bot = np.stack([sc, z0, -hh - ss], axis=-1)
            cap_yz_bot = np.stack([z0, sc, -hh - ss], axis=-1)
            rib_s = np.array([[r, 0, -hh], [-r, 0, -hh], [0, r, -hh], [0, -r, -hh]], dtype=np.float32)
            rib_e = np.array([[r, 0, hh], [-r, 0, hh], [0, r, hh], [0, -r, hh]], dtype=np.float32)
            curves = [top, bot, cap_xz_top, cap_yz_top, cap_xz_bot, cap_yz_bot]
            starts = np.concatenate([*[cv[:-1] for cv in curves], rib_s], axis=0)
            ends = np.concatenate([*[cv[1:] for cv in curves], rib_e], axis=0)
            return starts, ends

        if kind == "cylinder":
            r, hh = params
            n = 16
            ang = np.linspace(0.0, 2.0 * np.pi, n + 1, dtype=np.float32)
            c, s = r * np.cos(ang), r * np.sin(ang)
            top = np.stack([c, s, np.full_like(c, hh)], axis=-1)
            bot = np.stack([c, s, np.full_like(c, -hh)], axis=-1)
            rib_s = np.array([[r, 0, -hh], [-r, 0, -hh], [0, r, -hh], [0, -r, -hh]], dtype=np.float32)
            rib_e = np.array([[r, 0, hh], [-r, 0, hh], [0, r, hh], [0, -r, hh]], dtype=np.float32)
            starts = np.concatenate([top[:-1], bot[:-1], rib_s], axis=0)
            ends = np.concatenate([top[1:], bot[1:], rib_e], axis=0)
            return starts, ends

        if kind == "cone":
            r, hh = params
            n = 16
            ang = np.linspace(0.0, 2.0 * np.pi, n + 1, dtype=np.float32)
            c, s = r * np.cos(ang), r * np.sin(ang)
            bot = np.stack([c, s, np.full_like(c, -hh)], axis=-1)
            apex = np.array([0.0, 0.0, hh], dtype=np.float32)
            rib_s = np.array([[r, 0, -hh], [-r, 0, -hh], [0, r, -hh], [0, -r, -hh]], dtype=np.float32)
            rib_e = np.broadcast_to(apex, rib_s.shape).copy()
            starts = np.concatenate([bot[:-1], rib_s], axis=0)
            ends = np.concatenate([bot[1:], rib_e], axis=0)
            return starts, ends

        if kind == "mesh":
            verts_raw, indices_raw, scale = params
            verts = np.asarray(verts_raw, dtype=np.float32).reshape(-1, 3) * np.asarray(scale, dtype=np.float32)
            tris = np.asarray(indices_raw, dtype=np.int32).reshape(-1, 3)
            if len(verts) == 0 or len(tris) == 0:
                empty = np.zeros((0, 3), dtype=np.float32)
                return empty, empty
            # 全三角形の 3 辺を抽出し、無向辺 (min, max) で重複除去して描画本数を約半分に抑える
            e01 = tris[:, [0, 1]]
            e12 = tris[:, [1, 2]]
            e20 = tris[:, [2, 0]]
            edges = np.concatenate([e01, e12, e20], axis=0)
            edges = np.sort(edges, axis=1)
            edges = np.unique(edges, axis=0)
            if len(edges) > max_mesh_edges:
                # 巨大生メッシュで Viewport が固まらないよう等間隔間引き
                step = max(1, len(edges) // max_mesh_edges)
                edges = edges[::step]
            return verts[edges[:, 0]], verts[edges[:, 1]]

        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty

    @staticmethod
    def _get_debug_draw_interface():
        """`omni.debugdraw` インターフェースを取得する。Kit 外や拡張無効時は `None` を返す。"""
        try:
            from isaacsim.util.debug_draw import _debug_draw

            return _debug_draw.acquire_debug_draw_interface()
        except Exception:
            pass
        try:
            import omni.debugdraw

            return omni.debugdraw.get_debug_draw_interface()
        except Exception:
            return None

    def clear_debug_draw(self) -> None:
        """`DebugDraw` で描画したコライダー線分と接触点マーカーを消去する。"""
        dd = self._get_debug_draw_interface()
        if dd is None:
            return
        dd.clear_lines()
        dd.clear_points()

    def draw_colliders(
        self,
        pairs: Sequence[GeometricCollisionPair] | None = None,
        *,
        robot: Any | None = None,
        include_environment: bool = True,
        clear: bool = True,
        sync_robots: bool = True,
        sync_env: bool = True,
        line_width: float = 1.5,
        point_size: float = 9.0,
        contact_cross_radius: float = 0.015,
    ) -> int:
        """Newton 内部の実際のコライダー形状と接触点を `DebugDraw` (RTX オーバーレイ) で可視化する。

        特徴:
            - `write_target="fabric"` のまま USD へ一切書き戻さず、0 フレーム遅延で最新形状を描画する。
            - 追加の衝突判定は行わない。`pairs` に直近の `check_collisions()` 結果を渡すと、
              干渉中リンク/環境コライダーを赤、通常コライダーをシアン、接触点とめり込み線分をオレンジで描く。
            - 戻り値は描画した線分総数 (Kit 外等で `DebugDraw` が利用不可の場合は 0)。
        """
        self._ensure_finalized()
        if sync_robots:
            self.sync_robots()
        if sync_env and include_environment and self._env_entries:
            self.sync_environment(reset_manual_overrides=False)

        dd = self._get_debug_draw_interface()
        if dd is None:
            return 0
        if clear:
            dd.clear_lines()
            dd.clear_points()

        # 干渉中リンク名 (ロボット別) と環境コライダーパスを集合化
        colliding_robot_links: set[tuple[int, str]] = set()
        colliding_env_paths: set[str] = set()
        contact_points: list[tuple[float, float, float]] = []
        contact_seg_s: list[tuple[float, float, float]] = []
        contact_seg_e: list[tuple[float, float, float]] = []
        r_cross = float(contact_cross_radius)

        if pairs:
            robots_by_name = {rec.robot.name: rec.key for rec in self._robots.values()}
            for pair in pairs:
                # 貫通 (`separation <= 0` または `None`) の組を干渉中として赤表示する
                is_hit = pair.separation is None or pair.separation <= 0.0
                if is_hit:
                    ra_key = robots_by_name.get(pair.robot_a)
                    if ra_key is not None:
                        colliding_robot_links.add((ra_key, pair.link_a))
                    if pair.link_b is not None:
                        rb_key = robots_by_name.get(pair.robot_b or pair.robot_a)
                        if rb_key is not None:
                            colliding_robot_links.add((rb_key, pair.link_b))
                    else:
                        colliding_env_paths.add(pair.path_b)
                if pair.point_a is not None:
                    pa = tuple(float(v) for v in pair.point_a)
                    contact_points.append(pa)
                    # 離れた視点からでも接触点を見失わないようオレンジの 3 軸小クロスを重ねる
                    for dx, dy, dz in ((r_cross, 0.0, 0.0), (0.0, r_cross, 0.0), (0.0, 0.0, r_cross)):
                        contact_seg_s.append((pa[0] - dx, pa[1] - dy, pa[2] - dz))
                        contact_seg_e.append((pa[0] + dx, pa[1] + dy, pa[2] + dz))
                if pair.point_b is not None:
                    pb = tuple(float(v) for v in pair.point_b)
                    contact_points.append(pb)
                if pair.point_a is not None and pair.point_b is not None:
                    contact_seg_s.append(tuple(float(v) for v in pair.point_a))
                    contact_seg_e.append(tuple(float(v) for v in pair.point_b))

        body_q_np = self._state.body_q.numpy()  # (B, 7) [px, py, pz, qx, qy, qz, qw]
        shape_body_np = self._model.shape_body.numpy()
        shape_xf_np = self._model.shape_transform.numpy()

        def _wp_xf_to_rt(row: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            p = row[:3].astype(np.float32)
            q_wxyz = np.array([row[6], row[3], row[4], row[5]], dtype=float)
            rot = tf.quat_to_matrix(q_wxyz).astype(np.float32)
            return rot, p

        # 全シェイプのワールド線分を通常 (シアン) / 干渉中 (赤) に仕分けて一括描画する
        s_all: list[tuple[float, float, float]] = []
        e_all: list[tuple[float, float, float]] = []
        c_all: list[tuple[float, float, float, float]] = []
        w_all: list[float] = []

        for sid, owner in self._shape_to_owner.items():
            kind_owner, owner_robot, link_or_path, _col_path = owner
            if kind_owner == "env" and not include_environment:
                continue
            if kind_owner == "robot" and robot is not None and owner_robot is not robot:
                continue
            vis_spec = self._shape_vis_info.get(sid)
            if vis_spec is None:
                continue
            cached = self._wireframe_cache.get(sid)
            if cached is None:
                cached = self._build_local_wireframe(vis_spec)
                self._wireframe_cache[sid] = cached
            loc_s, loc_e = cached
            if len(loc_s) == 0:
                continue

            sr, sp = _wp_xf_to_rt(shape_xf_np[sid])
            bid = int(shape_body_np[sid])
            if bid >= 0:
                br, bp = _wp_xf_to_rt(body_q_np[bid])
                wr = br @ sr
                wp_pos = br @ sp + bp
            else:
                wr, wp_pos = sr, sp

            ws = loc_s @ wr.T + wp_pos
            we = loc_e @ wr.T + wp_pos
            is_colliding = (
                (kind_owner == "robot" and (id(owner_robot), link_or_path) in colliding_robot_links)
                or (kind_owner == "env" and link_or_path in colliding_env_paths)
            )
            col = self._COLOR_COLLIDING if is_colliding else self._COLOR_NORMAL
            lw = float(line_width) * (1.6 if is_colliding else 1.0)
            n_seg = len(ws)
            s_all.extend(map(tuple, ws.tolist()))
            e_all.extend(map(tuple, we.tolist()))
            c_all.extend([col] * n_seg)
            w_all.extend([lw] * n_seg)

        if contact_seg_s:
            n_c = len(contact_seg_s)
            s_all.extend(contact_seg_s)
            e_all.extend(contact_seg_e)
            c_all.extend([self._COLOR_CONTACT] * n_c)
            w_all.extend([float(line_width) * 2.0] * n_c)

        if s_all:
            dd.draw_lines(s_all, e_all, c_all, w_all)
        if contact_points:
            dd.draw_points(contact_points, [self._COLOR_CONTACT] * len(contact_points), [float(point_size)] * len(contact_points))
        return len(s_all)

    def close(self) -> None:
        if self._closed:
            return
        for rec in list(self._robots.values()):
            cb = self._structure_callbacks.pop(rec.key, None)
            if cb is not None and hasattr(rec.robot, "remove_structure_listener"):
                rec.robot.remove_structure_listener(cb)
        self._robots.clear()
        if self._notice is not None:
            self._notice.Revoke()
            self._notice = None
        self._model = None
        self._state = None
        self._pipeline = None
        self._contacts = None
        self._closed = True


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
        self._current_gap = 0.0

        # テンプレート 1 台分を `GeometricRobotModel` + USD コライダーから組む (BFS トポロジカル順 + 根 FREE 関節)
        tmpl = newton.ModelBuilder(up_axis=_stage_up_axis(self._stage), gravity=wp.vec3(0.0, 0.0, 0.0))
        tmpl.rigid_gap = 0.0
        shape_cfg = newton.ModelBuilder.ShapeConfig(
            margin=0.0, gap=0.0, collision_filter_parent=bake_adjacent_filter
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
            for cp in first.collider_paths.get(ln, ()):
                sid, _ = _add_usd_collider_shape(
                    tmpl, bid, self._stage, cp, link_world[li], shape_cfg, xc, convex_meshes=convex_meshes
                )
                if sid is not None:
                    local_shape_link.append(ln)

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
                    label=js.name, collision_filter_parent=bake_adjacent_filter,
                )
            elif js.joint_type == "prismatic":
                nj = tmpl.add_joint_prismatic(
                    parent=pb, child=cb, parent_xform=px, child_xform=cx, axis=wp.vec3(*js.axis),
                    label=js.name, collision_filter_parent=bake_adjacent_filter,
                )
            else:
                nj = tmpl.add_joint_fixed(
                    parent=pb, child=cb, parent_xform=px, child_xform=cx,
                    label=js.name, collision_filter_parent=bake_adjacent_filter,
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
