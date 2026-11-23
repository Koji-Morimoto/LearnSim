"""テストと計測用の、4 本アームのロボットの USD を作る(Articulation 付き)。

    base ─(固定)─ ワールド
    base ─ lift(直動 Z) ─ column ─ boom_yaw(回転 Z) ─ boom
    boom ─ アーム i (i = 0..3)
        セットアップ 4 軸: s1 回転 Z / s2 直動 Z / s3 回転 Z / s4 回転 Z
        本体 4 軸        : m1 回転 Z / m2 回転 Y / m3 直動 X / m4 回転 X
        器具(VariantSet "Instrument"、アームごとの prim arm{i}_tool に付く)
            wrist   : t_roll 回転 X / t_pitch 回転 Y / t_yaw 回転 Z / jaw_a 回転 Z / jaw_b 回転 Z   (5 軸)
            stapler : t_roll 回転 X / t_pitch 回転 Y / jaw 回転 Z                                   (3 軸)
            camera  : 先端は固定                                                                   (0 軸)

全部 wrist なら DOF = 2 + 4 × (8 + 5) = 54、リンク = 3 + 4 × (8 + 1 + 5) = 59。
リンクの prim は、ロボットの prim の直下に兄弟で並ぶ(器具のリンクだけは arm{i}_tool の下)。
各リンクにはカプセルのコライダー(子 prim "collision")を 1 つ付ける。
"""

from __future__ import annotations

import math

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ARMS = 4
INSTRUMENTS: dict[str, int] = {"wrist": 5, "stapler": 3, "camera": 0}


def _body(stage: Usd.Stage, path: str, position: tuple[float, float, float], radius: float = 0.03, half: float = 0.04) -> None:
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(0.5)
    capsule = UsdGeom.Capsule.Define(stage, path + "/collision")
    capsule.CreateRadiusAttr(radius)
    capsule.CreateHeightAttr(half * 2)
    capsule.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())


def _joint(stage, path, kind, body0, body1, offset, axis="Z", lower=None, upper=None) -> None:
    """body0 の原点から offset の位置に関節(姿勢は回転なし)。body1 の原点が関節の位置にある。"""
    cls = {"revolute": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint, "fixed": UsdPhysics.FixedJoint}[kind]
    joint = cls.Define(stage, path)
    if body0:
        joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*offset))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
    if kind != "fixed":
        joint.CreateAxisAttr(axis)
        default = (-170.0, 170.0) if kind == "revolute" else (-0.2, 0.2)
        joint.CreateLowerLimitAttr(default[0] if lower is None else lower)
        joint.CreateUpperLimitAttr(default[1] if upper is None else upper)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def write_multi_arm_asset(path: str, default_instrument: str = "wrist") -> str:
    """4 本アームのロボットを、ファイル path(.usda)に書く。defaultPrim は /Robot。path を返す。"""
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    R = "/Robot"
    UsdGeom.Scope.Define(stage, f"{R}/joints")

    pos = {"base": (0.0, 0.0, 0.1)}
    _body(stage, f"{R}/base", pos["base"], radius=0.15)
    _joint(stage, f"{R}/joints/root", "fixed", None, f"{R}/base", pos["base"])
    pos["column"] = _add(pos["base"], (0.0, 0.0, 0.6))
    _body(stage, f"{R}/column", pos["column"], radius=0.08)
    _joint(stage, f"{R}/joints/lift", "prismatic", f"{R}/base", f"{R}/column", (0.0, 0.0, 0.6), "Z", -0.1, 0.3)
    pos["boom"] = _add(pos["column"], (0.0, 0.0, 0.9))
    _body(stage, f"{R}/boom", pos["boom"], radius=0.08)
    _joint(stage, f"{R}/joints/boom_yaw", "revolute", f"{R}/column", f"{R}/boom", (0.0, 0.0, 0.9), "Z")

    arm_steps = [  # (名前, 種類, 軸, 親からのずれ)
        ("s1", "revolute", "Z", None),
        ("s2", "prismatic", "Z", (0.0, 0.0, -0.15)),
        ("s3", "revolute", "Z", (0.25, 0.0, 0.0)),
        ("s4", "revolute", "Z", (0.25, 0.0, 0.0)),
        ("m1", "revolute", "Z", (0.1, 0.0, -0.1)),
        ("m2", "revolute", "Y", (0.0, 0.0, -0.15)),
        ("m3", "prismatic", "X", (0.2, 0.0, 0.0)),
        ("m4", "revolute", "X", (0.15, 0.0, 0.0)),
    ]
    for i in range(ARMS):
        angle = math.pi / 4 + i * math.pi / 2
        mount = (0.2 * math.cos(angle), 0.2 * math.sin(angle), 0.0)
        parent = f"{R}/boom"
        parent_pos = pos["boom"]
        for name, kind, axis, offset in arm_steps:
            offset = mount if offset is None else (offset[0] * math.cos(angle) - offset[1] * math.sin(angle), offset[0] * math.sin(angle) + offset[1] * math.cos(angle), offset[2])
            body = f"{R}/arm{i}_{name}"
            p = _add(parent_pos, offset)
            _body(stage, body, p, radius=0.03)
            _joint(stage, f"{R}/joints/arm{i}_{name}", kind, parent, body, offset, axis)
            parent, parent_pos = body, p
        # 器具: arm{i}_tool の VariantSet で中身を切り替える
        tool = UsdGeom.Xform.Define(stage, f"{R}/arm{i}_tool").GetPrim()
        vset = tool.GetVariantSets().AddVariantSet("Instrument")
        for variant in INSTRUMENTS:
            vset.AddVariant(variant)
            vset.SetVariantSelection(variant)
            with vset.GetVariantEditContext():
                _instrument(stage, f"{R}/arm{i}_tool", variant, parent, parent_pos, angle)
        vset.SetVariantSelection(default_instrument)
    stage.GetRootLayer().Save()
    return path


def _instrument(stage, tool, variant, parent, parent_pos, angle) -> None:
    def rot(o):
        return (o[0] * math.cos(angle) - o[1] * math.sin(angle), o[0] * math.sin(angle) + o[1] * math.cos(angle), o[2])

    UsdGeom.Scope.Define(stage, f"{tool}/joints")
    shaft_off = rot((0.12, 0.0, 0.0))
    shaft_pos = _add(parent_pos, shaft_off)
    _body(stage, f"{tool}/shaft", shaft_pos, radius=0.012, half=0.06)
    _joint(stage, f"{tool}/joints/mount", "fixed", parent, f"{tool}/shaft", shaft_off)
    if variant == "camera":
        return
    steps = {
        "wrist": [("t_roll", "X", (0.12, 0, 0)), ("t_pitch", "Y", (0.02, 0, 0)), ("t_yaw", "Z", (0.02, 0, 0)), ("jaw_a", "Z", (0.015, 0.005, 0)), ("jaw_b", "Z", (0.0, -0.01, 0))],
        "stapler": [("t_roll", "X", (0.12, 0, 0)), ("t_pitch", "Y", (0.03, 0, 0)), ("jaw", "Z", (0.02, 0, 0))],
    }[variant]
    p_path, p_pos = f"{tool}/shaft", shaft_pos
    for name, axis, offset in steps:
        o = rot(offset)
        body = f"{tool}/{name}_link"
        p = _add(p_pos, o)
        _body(stage, body, p, radius=0.006, half=0.01)
        _joint(stage, f"{tool}/joints/{name}", "revolute", p_path, body, o, axis, -90.0, 90.0)
        if not name.startswith("jaw"):  # 2 本目の顎は、1 本目と同じ親(t_yaw のリンク)にぶら下げる
            p_path, p_pos = body, p


def add_multi_arm_robot(stage: Usd.Stage, prim_path: str, asset_path: str, position=(0.0, 0.0, 0.0)) -> Usd.Prim:
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    UsdGeom.Xformable(prim).ClearXformOpOrder()
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
    return prim
