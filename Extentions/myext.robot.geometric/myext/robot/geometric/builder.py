"""内部モデルから、簡易形状のロボットの prim を組む(DH・スクリュー・URDF から作るとき用)。

    <prim_path>                      Xform(ロボットの入れ物)
      <リンク名>                     Xform(全リンクが兄弟。translate / orient は関節値 0 の姿勢)
        visuals/bone_*               リンクの骨(関節から関節へのカプセル)
        visuals/axis_<関節名>        子リンク側に付く、関節の軸を表す円柱
        collisions/bone_*            干渉判定用のカプセル(CollisionAPI のみ。剛体は付けない = 静的コライダー)

Articulation も Joint も作らない。見た目と干渉判定は別の prim なので、干渉判定の形だけ差し替えられる。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .model import GeometricRobotModel
from .model import transforms as tf

__all__ = ["GeometricVisualStyle", "build_robot_prims", "safe_prim_name"]


@dataclass(frozen=True)
class GeometricVisualStyle:
    """簡易形状の寸法と色。長さの単位はモデルと同じ(ステージの単位)。"""

    link_radius: float = 0.025
    joint_radius: float = 0.035
    joint_length: float = 0.09
    link_color: tuple[float, float, float] = (0.75, 0.78, 0.82)
    revolute_color: tuple[float, float, float] = (0.95, 0.55, 0.10)
    prismatic_color: tuple[float, float, float] = (0.15, 0.55, 0.95)
    collisions: bool = True  # 干渉判定用の形状を作るか
    collision_radius: float | None = None  # None なら link_radius と同じ


def safe_prim_name(name: str) -> str:
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return out if out and not out[0].isdigit() else "_" + out


def _quat_from_z(direction: np.ndarray) -> Gf.Quatf:
    """+Z を direction に向ける回転。"""
    z = direction / np.linalg.norm(direction)
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    rot = np.stack([x, np.cross(z, x), z], axis=1)
    w, qx, qy, qz = tf.matrix_to_quat(rot)
    return Gf.Quatf(float(w), float(qx), float(qy), float(qz))


def _segment(stage: Usd.Stage, path: str, kind: str, a: np.ndarray, b: np.ndarray, radius: float, color, collider: bool) -> None:
    """a から b への棒(kind = "Capsule" か "Cylinder")。軸は Z で、Xform の代わりに自分の xformOp で置く。"""
    length = float(np.linalg.norm(b - a))
    geom = (UsdGeom.Capsule if kind == "Capsule" else UsdGeom.Cylinder).Define(stage, path)
    geom.CreateAxisAttr("Z")
    geom.CreateHeightAttr(length)
    geom.CreateRadiusAttr(radius)
    half = length / 2 + (radius if kind == "Capsule" else 0.0)
    geom.CreateExtentAttr([(-radius, -radius, -half), (radius, radius, half)])
    geom.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2).tolist()))
    geom.AddOrientOp().Set(_quat_from_z(b - a) if length > 1e-9 else Gf.Quatf(1.0))
    if collider:
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        geom.CreatePurposeAttr(UsdGeom.Tokens.guide)  # ふだんは描画しない
    else:
        geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def build_robot_prims(
    stage: Usd.Stage,
    model: GeometricRobotModel,
    prim_path: str,
    *,
    style: GeometricVisualStyle | None = None,
    base_transform: np.ndarray | None = None,
) -> dict[str, str]:
    """model の全リンクの prim を prim_path の下に作り、{リンク名: prim パス} を返す。いまの編集先のレイヤーに書く。

    base_transform はワールドで見た根リンクの姿勢(4x4)。prim_path の Xform は原点に置き、リンクを直接その姿勢に置く。
    """
    style = style or GeometricVisualStyle()
    if stage.GetPrimAtPath(prim_path):
        raise ValueError(f"prim がすでにある: {prim_path}")
    UsdGeom.Xform.Define(stage, prim_path)
    base = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
    poses = base @ model.forward_kinematics()
    col_radius = style.collision_radius if style.collision_radius is not None else style.link_radius
    paths: dict[str, str] = {}
    used: set[str] = set()
    for i, link in enumerate(model.links):
        name = safe_prim_name(link.name)
        while name in used:
            name += "_"
        used.add(name)
        path = f"{prim_path}/{name}"
        paths[link.name] = path
        xform = UsdGeom.Xform.Define(stage, path)
        w, x, y, z = tf.matrix_to_quat(poses[i, :3, :3])
        xform.AddTranslateOp().Set(Gf.Vec3d(*poses[i, :3, 3].tolist()))
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w), float(x), float(y), float(z)))
        xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
        UsdGeom.Scope.Define(stage, path + "/visuals")
        if style.collisions:
            UsdGeom.Scope.Define(stage, path + "/collisions")

        # 骨: 入ってくる関節の位置 → リンク原点 → 出ていく各関節の位置
        parent_joint = model.parent_joint_of(link.name)
        origin = np.zeros(3)
        segments: list[tuple[np.ndarray, np.ndarray]] = []
        if parent_joint is not None:
            segments.append((parent_joint.child_frame[:3, 3], origin))
        for joint in model.child_joints_of(link.name):
            segments.append((origin, joint.parent_frame[:3, 3]))
        n = 0
        for a, b in segments:
            if np.linalg.norm(b - a) < 1e-6:
                continue
            _segment(stage, f"{path}/visuals/bone_{n}", "Capsule", a, b, style.link_radius, style.link_color, False)
            if style.collisions:
                _segment(stage, f"{path}/collisions/bone_{n}", "Capsule", a, b, col_radius, None, True)
            n += 1
        if n == 0 and style.collisions:
            # 長さの無いリンクにも、位置が分かる小さな形を置く(干渉判定には入れない)
            sphere = UsdGeom.Sphere.Define(stage, f"{path}/visuals/origin")
            sphere.CreateRadiusAttr(style.link_radius)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(*style.link_color)])
        if parent_joint is not None and parent_joint.is_dof:
            center = parent_joint.child_frame[:3, 3]
            axis = parent_joint.child_frame[:3, :3] @ np.array(parent_joint.axis)
            half = axis * style.joint_length / 2
            color = style.revolute_color if parent_joint.joint_type == "revolute" else style.prismatic_color
            _segment(stage, f"{path}/visuals/axis_{safe_prim_name(parent_joint.name)}", "Cylinder", center - half, center + half, style.joint_radius, color, False)
    return paths
