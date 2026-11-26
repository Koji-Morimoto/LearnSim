"""内部モデル (`GeometricRobotModel`) から簡易形状のロボット prim 群を USD ステージへ構築するモジュール。

DH / 修正 DH / スクリュー軸 / URDF など「既存の USD Articulation を持たないソース」から
`GeometricRobot` を生成するときに `build_robot_prims` が呼ばれる。

生成する prim 階層:
    <prim_path>                      Xform (ロボット全体の入れ物。常に原点に置く)
      <リンク名>                     Xform (全リンクを兄弟として並列配置。translate / orient は関節値 0 の姿勢)
        visuals/bone_*               表示用の骨形状 (`UsdGeom.Capsule`、関節間を結ぶ)
        visuals/axis_<関節名>        子リンク側に付く関節軸マーカー (`UsdGeom.Cylinder`)
        collisions/bone_*            干渉判定用コライダー (`UsdGeom.Capsule` + `UsdPhysics.CollisionAPI`、`purpose="guide"`)

設計上の意図 (なぜ `ArticulationRootAPI` / `RigidBodyAPI` / `UsdPhysics.Joint` を付けないか):
    1. 幾何モードでは順運動学 (`GeometricRobotModel` または `newton.eval_fk`) が全リンクのワールド姿勢を直接計算するため、
       PhysX の剛体・関節ソルバーを一切起動させる必要が無い (`Play=ON` でも PhysX に姿勢を上書きされない)。
    2. 全リンクを `<prim_path>` 直下の兄弟 Xform として並列配置すると、各リンクの親行列が常に `<prim_path>` の
       単位行列になり、書き込み時のローカル行列計算 (`LocalPoseSolver`) と `GeometricGpuFabricWriter` が単純かつ高速になる。
    3. 表示用 (`visuals/`) と干渉判定用 (`collisions/`) を別 prim に分離し、コライダーに `purpose="guide"` を付けることで、
       通常ビューポートの表示を汚さずに、コライダー形状だけ独立して差し替えたり `collision_radius` を太らせたりできる。
    4. `GeometricScene` (Newton バックエンド) は `GeometricRobotModel` と `UsdPhysics.CollisionAPI` から直接構築するため、
       `ArticulationRootAPI` や `RigidBodyAPI` が無いこの階層でも USD Articulation と同一のコード経路で干渉・クリアランス判定が動く。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from ..model import GeometricRobotModel
from ..model import transforms as tf

__all__ = ["GeometricVisualStyle", "build_robot_prims", "safe_prim_name"]


# ==============================================================================
# スタイル定義とユーティリティ
# ==============================================================================


@dataclass(frozen=True)
class GeometricVisualStyle:
    """簡易形状の寸法・色・コライダー生成設定。長さの単位はモデルと同じ (ステージの単位系)。"""

    link_radius: float = 0.025
    joint_radius: float = 0.035
    joint_length: float = 0.09
    link_color: tuple[float, float, float] = (0.75, 0.78, 0.82)
    revolute_color: tuple[float, float, float] = (0.95, 0.55, 0.10)
    prismatic_color: tuple[float, float, float] = (0.15, 0.55, 0.95)
    collisions: bool = True  # `collisions/bone_*` (`UsdPhysics.CollisionAPI` 付き) を生成するか
    collision_radius: float | None = None  # `None` のときは `link_radius` をそのまま使う


def safe_prim_name(name: str) -> str:
    """USD の prim 名に使える識別子 (英数字と `_` のみ、先頭数字不可) へ変換する。"""
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return out if out and not out[0].isdigit() else "_" + out


# ==============================================================================
# ジオメトリ補助関数 (Z 軸基準カプセル・円柱の配置)
# ==============================================================================


def _quat_from_z(direction: np.ndarray) -> Gf.Quatf:
    """ローカル `+Z` 軸を単位ベクトル `direction` に一致させる右手系回転を `Gf.Quatf(w, x, y, z)` で返す。"""
    z = direction / np.linalg.norm(direction)
    # `direction` が X 軸にほぼ平行なときの特異点 (外積ゼロ) を避けるため補助ベクトルを切り替える
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    rot = np.stack([x, np.cross(z, x), z], axis=1)
    w, qx, qy, qz = tf.matrix_to_quat(rot)
    return Gf.Quatf(float(w), float(qx), float(qy), float(qz))


def _segment(stage: Usd.Stage, path: str, kind: str, a: np.ndarray, b: np.ndarray, radius: float, color, collider: bool) -> None:
    """リンクローカル座標系の 2 点 `a` -> `b` を結ぶ棒形状 (`Capsule` または `Cylinder`) を作成する。

    設計上の意図:
        - `axis="Z"` で作成し、`orient` で `b - a` 方向へ向けることで、USD / PhysX / Newton の
          既定 Z 軸表現と一致させ、追加の軸変換オーバーヘッドを避ける。
        - `collider=True` では `UsdPhysics.CollisionAPI` のみ付与し `RigidBodyAPI` は付けない (静的コライダー)。
          また `purpose="guide"` を設定し、通常レンダリングに表示されない干渉判定専用形状として扱う。
    """
    length = float(np.linalg.norm(b - a))
    geom = (UsdGeom.Capsule if kind == "Capsule" else UsdGeom.Cylinder).Define(stage, path)
    geom.CreateAxisAttr("Z")
    geom.CreateHeightAttr(length)
    geom.CreateRadiusAttr(radius)
    # カプセルは両端の半球半径 `radius` を含めたバウンディングボックスを `extent` に記録する
    half = length / 2 + (radius if kind == "Capsule" else 0.0)
    geom.CreateExtentAttr([(-radius, -radius, -half), (radius, radius, half)])
    geom.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2).tolist()))
    geom.AddOrientOp().Set(_quat_from_z(b - a) if length > 1e-9 else Gf.Quatf(1.0))
    if collider:
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        geom.CreatePurposeAttr(UsdGeom.Tokens.guide)  # ふだんのビューポート描画では非表示にする
    else:
        geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])


# ==============================================================================
# ロボット prim 一括生成
# ==============================================================================


def build_robot_prims(
    stage: Usd.Stage,
    model: GeometricRobotModel,
    prim_path: str,
    *,
    style: GeometricVisualStyle | None = None,
    base_transform: np.ndarray | None = None,
) -> dict[str, str]:
    """`model` の全リンクに対応する USD prim を `prim_path` の下に生成し、`{リンク名: prim パス}` を返す。

    現在のステージ編集ターゲット (`stage.GetEditTarget()`) に書き込む。
    `base_transform` はワールド座標系で見た根リンクの 4x4 同次変換行列 (列ベクトル形式)。
    `prim_path` 自体は原点に固定し、各リンク Xform に `base_transform @ model.forward_kinematics()` の
    ワールド姿勢を直接設定することで、全リンクが親行列 = 単位行列の兄弟階層になる。
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
        # `attach()` で `/` を含むリンク名などを USD 識別子へ正規化し、重複時は `_` で一意化する
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

        # 骨線分: 親関節の `child_frame` 位置 -> リンク原点 -> 各子関節の `parent_frame` 位置
        parent_joint = model.parent_joint_of(link.name)
        origin = np.zeros(3)
        segments: list[tuple[np.ndarray, np.ndarray]] = []
        if parent_joint is not None:
            segments.append((parent_joint.child_frame[:3, 3], origin))
        for joint in model.child_joints_of(link.name):
            segments.append((origin, joint.parent_frame[:3, 3]))
        n = 0
        for a, b in segments:
            # 長さほぼ 0 の退化線分 (原点一致の関節など) はカプセル化すると法線が不定になるためスキップする
            if np.linalg.norm(b - a) < 1e-6:
                continue
            _segment(stage, f"{path}/visuals/bone_{n}", "Capsule", a, b, style.link_radius, style.link_color, False)
            if style.collisions:
                _segment(stage, f"{path}/collisions/bone_{n}", "Capsule", a, b, col_radius, None, True)
            n += 1
        if n == 0 and style.collisions:
            # 長さを持たないダミーリンク・手先リンクにも位置が視認できる小球を置く (誤干渉を避けるため非コライダー)
            sphere = UsdGeom.Sphere.Define(stage, f"{path}/visuals/origin")
            sphere.CreateRadiusAttr(style.link_radius)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(*style.link_color)])
        if parent_joint is not None and parent_joint.is_dof:
            # 関節軸の向きと種類 (回転=橙 / 直動=青) を確認できるよう子リンク側に円柱マーカーを置く
            center = parent_joint.child_frame[:3, 3]
            axis = parent_joint.child_frame[:3, :3] @ np.array(parent_joint.axis)
            half = axis * style.joint_length / 2
            color = style.revolute_color if parent_joint.joint_type == "revolute" else style.prismatic_color
            _segment(stage, f"{path}/visuals/axis_{safe_prim_name(parent_joint.name)}", "Cylinder", center - half, center + half, style.joint_radius, color, False)
    return paths
