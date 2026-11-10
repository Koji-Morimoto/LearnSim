"""A01: Articulation で、Play ON / OFF × バックエンド指定 × 書き換え系メソッドを測る(M01 の Articulation 版)。

--physx-fabric / --gpu 付きで実行すると、その構成で同じ測定をする。

ロボット(重力 0、浮遊ベース):
    /World/Robot          Xform + ArticulationRootAPI(xformOp は translate / orient / scale。A-90 だけ xformOp なし)
    /World/Robot/root     立方体(剛体、質量 1)。z=1
    /World/Robot/child    立方体(剛体、質量 1)。root から +x に 0.3 m
    /World/Robot/child/joint  root-child の回転関節(Z 軸)。角度ドライブ(剛性 0、減衰 0)と JointStateAPI 付き

マトリクス:
    開始状態: stopped / playing
    メソッド × バックエンド:
        set_world_poses(positions=(1,2,3))          : 既定 / tensor / usd / usdrt / fabric
        set_dof_positions(0.5 rad)                  : 既定 / usd     (tensor 専用メソッド)
        set_dof_position_targets(0.3 rad)           : 既定 / tensor / usd
        set_link_masses([1, 5])                     : 既定 / usd
手順は M01 と同じ(stopped: 呼ぶ → update → Play → 10 update → Stop、playing: Play → 呼ぶ → update → Stop → 再 Play)。

観測(ラッパーを通さない):
    USD    : Robot(Xform)のワールド位置、root リンクのワールド位置、関節角 state:angular:physics:position [deg]、
             目標角 drive:angular:physics:targetPosition [deg]、child の physics:mass
    Fabric : Robot と root リンクのワールド位置(IFabricHierarchy)、同名属性の Fabric 上の値
    tensor : 検証用 ArticulationView の root 位置、関節角 [rad]、目標角 [rad]、リンク質量

追加:
    A-90  Robot(Xform)に xformOp が無いと、usd バックエンドの set_world_poses はどうなるか
    A-91  simstate バックエンド。use_backend("simstate") と設定 /isaacsim/articulation/simStateMode="mirror" で
          set_dof_position_targets を呼ぶ。SimStateStorage を提供する isaacsim.mega.sim_state はこのインストールに無い
"""

from __future__ import annotations

import bplib

app = bplib.start()

import math
from typing import Any, Callable

from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

rec = bplib.Recorder("a01_articulation")

ROBOT: str = "/World/Robot"
ROOT: str = f"{ROBOT}/root"
CHILD: str = f"{ROBOT}/child"
JOINT: str = f"{CHILD}/joint"

TARGET_POS: list[float] = [1.0, 2.0, 3.0]  # set_world_poses で書く root 位置 [m]
DOF_POS: float = 0.5  # set_dof_positions で書く関節角 [rad](= 28.65 deg)
DOF_TARGET: float = 0.3  # set_dof_position_targets で書く目標角 [rad](= 17.19 deg)
LINK_MASSES: list[float] = [1.0, 5.0]  # set_link_masses で書くリンク質量 [kg](root, child)

OPS: dict[str, tuple[Callable[[Articulation], None], list[str | None]]] = {
    "set_world_poses": (lambda a: a.set_world_poses(positions=[TARGET_POS]), [None, "tensor", "usd", "usdrt", "fabric"]),
    "set_dof_positions": (lambda a: a.set_dof_positions([[DOF_POS]]), [None, "usd"]),
    "set_dof_position_targets": (lambda a: a.set_dof_position_targets([[DOF_TARGET]]), [None, "tensor", "usd"]),
    "set_link_masses": (lambda a: a.set_link_masses([LINK_MASSES]), [None, "usd"]),
}


def build(root_xform_ops: bool = True) -> Usd.Stage:
    """重力 0 のステージに 2 リンクの浮遊ベース Articulation を作る。

    Args:
        root_xform_ops: Robot(Xform)に translate / orient / scale の xformOp を付けるか。
    """
    stage = bplib.new_stage(gravity=0.0)
    robot = UsdGeom.Xform.Define(stage, ROBOT)
    if root_xform_ops:
        robot.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        robot.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        robot.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    for path, x in [(ROOT, 0.0), (CHILD, 0.3)]:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(0.2)
        cube.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 1.0))
        cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        cube.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
    joint = UsdPhysics.RevoluteJoint.Define(stage, JOINT)
    joint.CreateBody0Rel().SetTargets([ROOT])
    joint.CreateBody1Rel().SetTargets([CHILD])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.15, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(-0.15, 0.0, 0.0))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular").CreatePositionAttr(0.0)
    bplib.update()
    return stage


# 検証用 ArticulationView のキャッシュ(作成元の simulation view, view)
_view_cache: list[Any] = [None, None]


def art_view() -> Any | None:
    """検証用の ArticulationView。simulation view が無効(Stop 中)なら None。"""
    from isaacsim.core.simulation_manager import SimulationManager

    sv = SimulationManager._physics_sim_view__warp
    if sv is None or not sv.is_valid:
        return None
    if _view_cache[0] is not sv:
        _view_cache[0], _view_cache[1] = sv, sv.create_articulation_view(ROBOT)
    return _view_cache[1]


def _usd_attr(stage: Usd.Stage, path: str, name: str) -> Any:
    attr = stage.GetPrimAtPath(path).GetAttribute(name)
    return bplib._r(attr.Get()) if attr and attr.HasAuthoredValue() else None


def _fabric_attr(path: str, name: str) -> Any:
    attr = bplib._rt_stage().GetPrimAtPath(path).GetAttribute(name)
    return bplib._r(attr.Get()) if attr and attr.HasValue() else None


def snap(stage: Usd.Stage) -> dict[str, Any]:
    """全層の値。角度は USD / Fabric が deg、tensor が rad(比較しやすいよう tensor は deg も併記)。"""
    def usd_world(path: str) -> Any:
        return bplib._r(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())

    out: dict[str, Any] = {
        "timeline": bplib.timeline_state(),
        "usd": {
            "robot_xform": usd_world(ROBOT),
            "root": usd_world(ROOT),
            "joint_deg": _usd_attr(stage, JOINT, "state:angular:physics:position"),
            "target_deg": _usd_attr(stage, JOINT, "drive:angular:physics:targetPosition"),
            "child_mass": _usd_attr(stage, CHILD, "physics:mass"),
        },
    }
    try:
        out["fabric"] = {
            "robot_xform": bplib.fabric_state(ROBOT).get("world"),
            "root": bplib.fabric_state(ROOT).get("world"),
            "joint_deg": _fabric_attr(JOINT, "state:angular:physics:position"),
            "target_deg": _fabric_attr(JOINT, "drive:angular:physics:targetPosition"),
            "child_mass": _fabric_attr(CHILD, "physics:mass"),
        }
    except Exception as e:
        out["fabric"] = {"error": f"{type(e).__name__}: {e}"}
    v = art_view()
    if v is None:
        out["tensor"] = {"valid": False}
    else:
        dof = float(v.get_dof_positions().numpy()[0, 0])
        tgt = float(v.get_dof_position_targets().numpy()[0, 0])
        out["tensor"] = {
            "valid": True,
            "root": bplib._r(v.get_root_transforms().numpy()[0, :3]),
            "joint_rad": round(dof, 4),
            "joint_deg": round(math.degrees(dof), 2),
            "target_rad": round(tgt, 4),
            "target_deg": round(math.degrees(tgt), 2),
            "link_masses": bplib._r(v.get_masses().numpy()[0]),
        }
    return out


def run_case(start: str, op: str, backend: str | None) -> dict[str, Any]:
    stage = build()
    art = Articulation(ROBOT)
    fn = OPS[op][0]
    res: dict[str, Any] = {}
    if start == "playing":
        bplib.play(3)
    res["0_before"] = snap(stage)
    res["call"] = bplib.call_with_backend([art], lambda: fn(art), backend)
    res["1_after_call"] = snap(stage)
    bplib.update()
    res["2_after_update1"] = snap(stage)
    if start == "stopped":
        bplib.play(1)
        res["3_toggled_play_update1"] = snap(stage)
        bplib.update(9)
        res["4_play_update10"] = snap(stage)
        bplib.stop(1)
        res["5_stopped_again"] = snap(stage)
    else:
        bplib.stop(1)
        res["3_toggled_stop"] = snap(stage)
        bplib.play(1)
        res["4_replay_update1"] = snap(stage)
        bplib.update(9)
        res["5_replay_update10"] = snap(stage)
    return res


@bplib.guarded(rec, "A-00")
def env() -> None:
    rec.add("A-00", "バージョンと設定", **bplib.environment())


def make_case(start: str, op: str, backend: str | None) -> Callable[[], None]:
    test_id = f"A/{start}/{op}/{backend or '(既定)'}"

    @bplib.guarded(rec, test_id)
    def case() -> None:
        rec.add(test_id, f"開始={start} メソッド={op} use_backend={backend}", **run_case(start, op, backend))

    return case


@bplib.guarded(rec, "A-90")
def no_root_xform_ops() -> None:
    """Robot(Xform)に xformOp が無いステージで、Stop 中に set_world_poses(既定 = Stop 中は usd)を呼ぶ。
    reset_xform_op_properties=True でラッパーを作った場合とも比べる。
    """
    out: dict[str, Any] = {}
    for reset in [False, True]:
        stage = build(root_xform_ops=False)
        art = Articulation(ROBOT, reset_xform_op_properties=reset)
        call = bplib.call_with_backend([art], lambda: art.set_world_poses(positions=[TARGET_POS]), None)
        bplib.update()
        ops = [op.GetOpName() for op in UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT)).GetOrderedXformOps()]
        out[f"reset_xform_op_properties={reset}"] = {"call": call, "robot_xform_ops_after": ops, "after_update1": snap(stage)}
    rec.add("A-90", "Robot に xformOp が無いときの set_world_poses(Stop 中)", **out)


@bplib.guarded(rec, "A-91")
def simstate_backend() -> None:
    import carb.settings
    import isaacsim.core.experimental.utils.backend as backend_utils
    from isaacsim.core.experimental.prims import RigidPrim

    settings = carb.settings.get_settings()
    key = "/isaacsim/articulation/simStateMode"
    out: dict[str, Any] = {}
    for variant in ["use_backend('simstate')", "simStateMode=mirror"]:
        stage = build()
        art = Articulation(ROBOT)
        bplib.play(3)
        old = settings.get(key)
        modes: list[str] = []  # 呼び出しの内側(use_backend の中)で読んだ simstate モード

        def call_fn() -> None:
            modes.append(str(backend_utils.get_simstate_mode()))
            art.set_dof_position_targets([[DOF_TARGET]])

        try:
            if variant == "simStateMode=mirror":
                settings.set_string(key, "mirror")
                call = bplib.call_with_backend([art], call_fn, None)
            else:
                call = bplib.call_with_backend([art], call_fn, "simstate")
            mode = modes[0] if modes else None
        finally:
            settings.set_string(key, old or "disabled")
        bplib.update(1)
        out[variant] = {"call": call, "simstate_mode_during_call": mode, "after_update1": snap(stage)}
        bplib.cleanup()
    # simstate に対応しないクラス(RigidPrim)で use_backend("simstate") を指定した場合
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    bplib.update()
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    out["RigidPrim.set_masses under use_backend('simstate')"] = bplib.call_with_backend([rp], lambda: rp.set_masses([5.0]), "simstate")
    rec.add("A-91", "simstate バックエンド(Play 中に目標角 0.3 rad を書く)", **out)


env()
no_root_xform_ops()
simstate_backend()
for _start in ["stopped", "playing"]:
    for _op, (_fn, _backends) in OPS.items():
        for _b in _backends:
            make_case(_start, _op, _b)()
bplib.finish(rec)
