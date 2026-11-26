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

from .types import CollisionScope, GeometricCollisionPair, GeometricLinkSet, PairFilter, PairKind, find_colliders
from ..usd.layer import is_authoring
from ..model import GeometricCollisionFilter, GeometricRobotModel
from ..model import transforms as tf
from ..usd.reader import _np, split_scale

if TYPE_CHECKING:
    from ..runtime.robot import GeometricRobot
    from ..runtime.robot_view import GeometricRobotView

__all__ = [
    "CollisionBackend",
    "FkBackend",
    "resolve_newton_device",
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


