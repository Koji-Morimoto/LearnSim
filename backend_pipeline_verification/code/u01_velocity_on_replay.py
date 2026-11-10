"""U01: physx_fabric 構成で「Play 中に USD に速度を書く → Stop → 再 Play」すると速度 0 で始まる原因を切り分ける。

M01 の M/playing/set_velocities/usd で見つかった現象:
    USD の physics:velocity は 0.6 のまま残っているのに、再 Play 後の PhysX の速度は 0。
    Stop 直後の Fabric の速度は 0.6、再 Play の 1 update 後は 0。

仮説(omni.physx.fabric のプラグイン文字列 "FabricManager::resume:rigidBodyInitialization:createFabricAttributes" から):
    H1  Play 開始(resume)時に omni.physx.fabric が Fabric の速度を書き直し、
        それを omni.physx の Fabric change tracking が「ユーザーの変更」として PhysX に押し込む。
    H2  PhysX が Play 開始時に USD の速度をそもそも読んでいない(USD 属性の変更通知の扱いの問題)。

ラッパーは使わず、pxr / usdrt で直接書いて条件を 1 つずつ変える(毎回ステージを作り直す)。
各ケースで、最初の物理ステップ直前(pre-step コールバック)の Fabric の値と、Play 後の各層の値を記録する。
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any, Callable

import carb.settings
from omni.physx import get_physx_interface
from pxr import Gf, Usd

rec = bplib.Recorder("u01_velocity_on_replay")

V_USD: tuple[float, float, float] = (0.0, 0.6, 0.0)  # USD に書く速度 [m/s]
V_USD2: tuple[float, float, float] = (0.0, 0.5, 0.0)  # Stop 後に書き直す速度 [m/s]
V_FABRIC: tuple[float, float, float] = (0.0, 0.3, 0.0)  # Fabric に直接書く速度 [m/s]


def usd_set_velocity(stage: Usd.Stage, v: tuple[float, float, float]) -> None:
    """pxr で physics:velocity を直接書く。"""
    from pxr import UsdPhysics

    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bplib.BODY)).CreateVelocityAttr().Set(Gf.Vec3f(*v))


def fabric_set_velocity(v: tuple[float, float, float]) -> None:
    """usdrt で Fabric 上の physics:velocity を直接書く。"""
    import usdrt

    prim = bplib._rt_stage().GetPrimAtPath(bplib.BODY)
    attr = prim.GetAttribute("physics:velocity")
    if not attr:
        attr = prim.CreateAttribute("physics:velocity", usdrt.Sdf.ValueTypeNames.Vector3f, True)
    attr.Set(usdrt.Gf.Vec3f(*v))


class PreStepProbe:
    """物理ステップの直前ごとに、Fabric 上の速度と位置を記録する(最初の max_steps ステップ分)。"""

    def __init__(self, max_steps: int = 2) -> None:
        self.values: list[dict[str, Any]] = []
        self._max = max_steps
        self._sub = get_physx_interface().subscribe_physics_on_step_events(self._on, True, 0)

    def _on(self, dt: float) -> None:
        if len(self.values) < self._max:
            fs = bplib.fabric_state(bplib.BODY)
            self.values.append({"fabric_velocity": fs.get("velocity"), "fabric_local": fs.get("local")})

    def close(self) -> list[dict[str, Any]]:
        self._sub = None
        return self.values


def replay_and_observe(stage: Usd.Stage, res: dict[str, Any], prefix: str) -> None:
    """Play して、pre-step の Fabric 値、Play 直後(update 前)、1 update 後、10 update 後を記録する。"""
    probe = PreStepProbe()
    bplib.timeline().play()
    res[f"{prefix}_a_after_play_call"] = bplib.fabric_state(bplib.BODY)  # update 前(まだ物理は始まっていない)
    bplib.update(1)
    res[f"{prefix}_b_update1"] = bplib.snapshot(stage)
    bplib.update(9)
    res[f"{prefix}_c_update10"] = bplib.snapshot(stage)
    res[f"{prefix}_prestep_fabric"] = probe.close()


def fresh() -> Usd.Stage:
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    bplib.update()
    return stage


Case = Callable[[], dict[str, Any]]


def case_a_fresh_no_observe() -> dict[str, Any]:
    """一度も Play していないステージで、Stop 中に USD に速度を書き、Fabric を読まずに Play。"""
    stage = fresh()
    usd_set_velocity(stage, V_USD)
    res: dict[str, Any] = {}
    replay_and_observe(stage, res, "play")
    return res


def case_a_fresh_observe() -> dict[str, Any]:
    """同上だが、Play 前に Fabric の速度を読んでおく(Fabric に physics:velocity が載る)。"""
    stage = fresh()
    usd_set_velocity(stage, V_USD)
    bplib.update()
    res: dict[str, Any] = {"before_play": bplib.snapshot(stage)}
    replay_and_observe(stage, res, "play")
    return res


def case_b_after_cycle() -> dict[str, Any]:
    """何も書かずに Play → Stop を 1 回経てから、Stop 中に USD に速度を書いて Play。"""
    stage = fresh()
    bplib.play(3)
    bplib.stop(1)
    usd_set_velocity(stage, V_USD)
    bplib.update()
    res: dict[str, Any] = {"before_play": bplib.snapshot(stage)}
    replay_and_observe(stage, res, "play")
    return res


def _mystery_until_stop(stage: Usd.Stage, res: dict[str, Any]) -> None:
    """M01 と同じ手順: Play(3 update)→ Play 中に USD に速度 → 1 update → Stop。"""
    bplib.play(3)
    usd_set_velocity(stage, V_USD)
    bplib.update(1)
    res["1_play_after_write"] = bplib.snapshot(stage)
    bplib.stop(1)
    res["2_stopped"] = bplib.snapshot(stage)


def case_c_mystery() -> dict[str, Any]:
    """M01 の現象をラッパーなしで再現する。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    replay_and_observe(stage, res, "replay")
    return res


def case_c_fabric_write() -> dict[str, Any]:
    """C の Stop 後、再 Play 前に Fabric の速度を 0.3 に直接書く。再 Play の速度が 0.3 なら、PhysX は Fabric から速度を受け取っている。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    fabric_set_velocity(V_FABRIC)
    res["3_after_fabric_write"] = bplib.fabric_state(bplib.BODY)
    replay_and_observe(stage, res, "replay")
    return res


def case_c_pause_tracking() -> dict[str, Any]:
    """C の再 Play の最初の 2 update の間だけ Fabric change tracking を止める。速度 0.6 で動けば、0 を押し込んだのは change tracking。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    sim = bplib.phys_sim()
    sim.pause_change_tracking(True)
    probe = PreStepProbe()
    bplib.timeline().play()
    bplib.update(2)
    res["replay_b_update2_paused"] = bplib.snapshot(stage)
    sim.pause_change_tracking(False)
    bplib.update(8)
    res["replay_c_update10"] = bplib.snapshot(stage)
    res["replay_prestep_fabric"] = probe.close()
    return res


def case_c_no_fabric_velocity() -> dict[str, Any]:
    """C を /physics/fabricUpdateVelocities = false(omni.physx.fabric が速度を Fabric に書かない)で行う。"""
    s = carb.settings.get_settings()
    key = "/physics/fabricUpdateVelocities"
    old = s.get(key)
    s.set(key, False)
    try:
        stage = fresh()
        res: dict[str, Any] = {"setting_before": old}
        _mystery_until_stop(stage, res)
        replay_and_observe(stage, res, "replay")
    finally:
        s.set(key, old)
    return res


def case_c_reauthor() -> dict[str, Any]:
    """C の Stop 後、USD の速度を 0.5 に書き直してから再 Play。値の変更通知があれば USD の値が使われるか。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    usd_set_velocity(stage, V_USD2)
    bplib.update()
    res["3_after_reauthor"] = bplib.snapshot(stage)
    replay_and_observe(stage, res, "replay")
    return res


def case_c_fabric_vel_before_replay_same_as_usd() -> dict[str, Any]:
    """B と C の違いを詰める: 何も書かずに Play→Stop した後、Fabric の速度を 0.6 にしておき、Stop 中に USD にも 0.6 を書いて Play。"""
    stage = fresh()
    bplib.play(3)
    bplib.stop(1)
    fabric_set_velocity(V_USD)
    usd_set_velocity(stage, V_USD)
    bplib.update()
    res: dict[str, Any] = {"before_play": bplib.snapshot(stage)}
    replay_and_observe(stage, res, "play")
    return res


def case_d_preauthored() -> dict[str, Any]:
    """速度属性を最初の Play 前に 0 で作っておき、U-C と同じ手順。属性が Play 中に「新規作成」されることが条件かを見る。"""
    stage = fresh()
    usd_set_velocity(stage, (0.0, 0.0, 0.0))
    bplib.update()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    replay_and_observe(stage, res, "replay")
    return res


def case_d_third_play() -> dict[str, Any]:
    """U-C の後、もう一度 Stop → Play(3 回目)。一度きりの現象か、ずっと続くか。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    replay_and_observe(stage, res, "replay")
    bplib.stop(1)
    res["3_stopped_again"] = bplib.snapshot(stage)
    replay_and_observe(stage, res, "third")
    return res


def case_d_reopen() -> dict[str, Any]:
    """U-C の Stop 後、ステージをファイルに保存して開き直してから Play。メモリ上の状態が原因かを見る。"""
    import os
    import tempfile

    import omni.usd

    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    path = os.path.join(tempfile.gettempdir(), "bp_u01_reopen.usda")
    stage.GetRootLayer().Export(path)
    omni.usd.get_context().open_stage(path)
    bplib.update(2)
    stage = omni.usd.get_context().get_stage()
    res["3_reopened"] = bplib.snapshot(stage)
    replay_and_observe(stage, res, "replay")
    return res


def case_d_no_sleep() -> dict[str, Any]:
    """U-C を、剛体がスリープしない設定(physxRigidBody:sleepThreshold = 0)で行う。PhysX のスリープで速度が 0 にされている可能性を見る。"""
    from pxr import PhysxSchema

    stage = fresh()
    PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(bplib.BODY)).CreateSleepThresholdAttr(0.0)
    bplib.update()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    replay_and_observe(stage, res, "replay")
    return res


def case_d_angular() -> dict[str, Any]:
    """角速度(physics:angularVelocity)でも同じことが起きるか。Play 中に USD に 90 deg/s を書く → Stop → 再 Play。"""
    from omni.physx import get_physx_interface
    from pxr import UsdPhysics

    stage = fresh()
    bplib.play(3)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bplib.BODY)).CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 90.0))
    bplib.update(1)
    res: dict[str, Any] = {"1_tensor_angvel_in_play": bplib._r(bplib.tensor_view(bplib.BODY).get_velocities().numpy()[0, 3:])}
    bplib.stop(1)
    attr = stage.GetPrimAtPath(bplib.BODY).GetAttribute("physics:angularVelocity")
    res["2_usd_angvel_after_stop"] = bplib._r(attr.Get()) if attr and attr.HasAuthoredValue() else None
    bplib.play(1)
    bplib.update(9)
    res["3_tensor_angvel_replay"] = bplib._r(bplib.tensor_view(bplib.BODY).get_velocities().numpy()[0, 3:])
    t = get_physx_interface().get_rigidbody_transformation(bplib.BODY)
    res["3_physx_rotation_replay"] = bplib._r(t["rotation"])
    return res


def case_e_recreate_while_stopped() -> dict[str, Any]:
    """U-C の Stop 後、速度属性を一度削除して Stop 中に作り直してから再 Play。「Stop 中に作られた属性」なら読まれるか。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    stage.GetPrimAtPath(bplib.BODY).RemoveProperty("physics:velocity")
    bplib.update()
    usd_set_velocity(stage, V_USD2)
    bplib.update()
    res["3_after_recreate"] = bplib.snapshot(stage)
    replay_and_observe(stage, res, "replay")
    return res


def case_e_no_reset_on_stop() -> dict[str, Any]:
    """/physics/resetOnStop = false で、Play 中に USD に速度 0.6 → Stop → 再 Play。
    既定構成では PhysX 自身が Play 中に physics:velocity を USD に作るので、両構成で同じ条件(Play 中に作られた属性)になる。
    """
    s = carb.settings.get_settings()
    key = "/physics/resetOnStop"
    old = s.get(key)
    s.set(key, False)
    try:
        stage = fresh()
        res: dict[str, Any] = {"setting_before": old}
        _mystery_until_stop(stage, res)
        replay_and_observe(stage, res, "replay")
    finally:
        s.set(key, old)
    return res


def _set_physx_fabric(enabled: bool) -> None:
    """omni.physx.fabric を実行中に有効/無効にする(SimulationManager.enable_fabric と同じ)。"""
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.enable_fabric(enabled)
    bplib.update()


def case_f_disable_before_replay() -> dict[str, Any]:
    """U-C の Stop 後、再 Play 前に omni.physx.fabric を無効にする。0 にならなければ、0 を書くのは omni.physx.fabric の Play 開始処理。"""
    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    _set_physx_fabric(False)
    try:
        replay_and_observe(stage, res, "replay")
    finally:
        _set_physx_fabric(bplib.PHYSX_FABRIC)
    return res


def case_f_enable_only_on_replay() -> dict[str, Any]:
    """最初の Play(USD に速度を書く回)は omni.physx.fabric 無効、再 Play 前に有効にする。記録が作られるのが omni.physx.fabric 側かを見る。"""
    _set_physx_fabric(False)
    try:
        stage = fresh()
        res: dict[str, Any] = {}
        _mystery_until_stop(stage, res)
        # 無効で走らせたので resetOnStop で速度属性は消えている。条件を揃えるため Stop 中に作り直す
        usd_set_velocity(stage, V_USD)
        bplib.update()
    finally:
        _set_physx_fabric(True)
    res["3_before_replay"] = bplib.snapshot(stage)
    replay_and_observe(stage, res, "replay")
    _set_physx_fabric(bplib.PHYSX_FABRIC)
    return res


def case_f_parsed_velocity() -> dict[str, Any]:
    """U-C の再 Play で、物理ステップを 1 回も進めずに PhysX へ USD を読み込ませ、その時点の PhysX の速度を読む。
    0.6 なら「読み込みは正しく、その後(最初のステップまで)に 0 にされる」、0 なら「読み込みの時点で 0」。
    """
    import omni.physics.tensors as pt
    from omni.physx import get_physx_interface

    stage = fresh()
    res: dict[str, Any] = {}
    _mystery_until_stop(stage, res)
    bplib.timeline().play()
    get_physx_interface().force_load_physics_from_usd()
    # numpy フロントエンドは GPU 物理では使えないので warp フロントエンドで作る(CPU / GPU 共通)
    sv = pt.create_simulation_view("warp", bplib._stage_id(stage))
    view = sv.create_rigid_body_view(bplib.BODY)
    try:
        res["replay_parsed_velocity_before_any_step"] = bplib._r(view.get_velocities().numpy()[0, :3])
    except Exception as e:  # GPU 物理では最初のステップ前に状態を読めない
        res["replay_parsed_velocity_before_any_step"] = f"{type(e).__name__}: {e}"
    bplib.update(1)
    res["replay_velocity_after_update1"] = bplib._r(view.get_velocities().numpy()[0, :3]) if sv.is_valid else "view invalidated"
    res["replay_b_update1"] = bplib.snapshot(stage)
    return res


def _trace_after_play_write(preauthor: bool, pause_tracking: bool = False) -> dict[str, Any]:
    """Play 中に USD に速度 0.6 を書き、その後 6 update の間、update ごとに tensor の速度と位置を記録する。

    Args:
        preauthor: Play 前に速度属性を 0 で作っておくか。
        pause_tracking: 書く前から最後まで Fabric change tracking(Fabric → PhysX)を止めておくか。
    """
    stage = fresh()
    if preauthor:
        usd_set_velocity(stage, (0.0, 0.0, 0.0))
        bplib.update()
    bplib.play(3)
    sim = bplib.phys_sim()
    if pause_tracking:
        sim.pause_change_tracking(True)
    usd_set_velocity(stage, V_USD)
    res: dict[str, Any] = {"0_after_write_no_update": bplib.tensor_state(bplib.BODY)}
    probe = PreStepProbe(max_steps=4)  # 書いた後の最初の 4 ステップ直前の Fabric の速度
    for i in range(1, 7):
        bplib.update(1)
        s = bplib.snapshot(stage)
        res[f"update{i}"] = {"tensor": s["tensor"], "usd_velocity": s["usd"]["velocity"], "fabric_velocity": s["fabric"].get("velocity")}
    res["prestep_fabric"] = probe.close()
    if pause_tracking:
        sim.pause_change_tracking(False)
    return res


def case_g_trace_absent() -> dict[str, Any]:
    """速度属性が無い状態から、Play 中に USD に速度を書く(M01 の playing/usd と同じ条件)。1 ステップで消えないか。"""
    return _trace_after_play_write(preauthor=False)


def case_g_trace_preauthored() -> dict[str, Any]:
    """Play 前に速度属性を 0 で作っておき、Play 中に USD に速度を書く。"""
    return _trace_after_play_write(preauthor=True)


def case_g_trace_absent_tracking_paused() -> dict[str, Any]:
    """U-G1 を Fabric change tracking を止めた状態で行う。速度が残れば、2 ステップ目に 0 を押し込むのは Fabric → PhysX の経路。"""
    return _trace_after_play_write(preauthor=False, pause_tracking=True)


CASES: list[tuple[str, str, Case]] = [
    ("U-A1", "未 Play のステージ、Stop 中に USD 速度 0.6、Fabric を読まずに Play", case_a_fresh_no_observe),
    ("U-A2", "未 Play のステージ、Stop 中に USD 速度 0.6、Fabric を読んでから Play", case_a_fresh_observe),
    ("U-B", "Play→Stop を 1 回経た後、Stop 中に USD 速度 0.6 → Play", case_b_after_cycle),
    ("U-C", "現象の再現: Play 中に USD 速度 0.6 → Stop → 再 Play", case_c_mystery),
    ("U-C1", "U-C の再 Play 前に Fabric の速度を 0.3 に直接書く", case_c_fabric_write),
    ("U-C2", "U-C の再 Play 最初の 2 update だけ Fabric change tracking を停止", case_c_pause_tracking),
    ("U-C3", "U-C を /physics/fabricUpdateVelocities=false で", case_c_no_fabric_velocity),
    ("U-C4", "U-C の Stop 後に USD 速度を 0.5 に書き直してから再 Play", case_c_reauthor),
    ("U-C5", "Play→Stop 後、Fabric と USD の両方に速度 0.6 を書いて Play", case_c_fabric_vel_before_replay_same_as_usd),
    ("U-D1", "速度属性を最初の Play 前に 0 で作っておいて U-C", case_d_preauthored),
    ("U-D2", "U-C の後さらに Stop → 3 回目の Play", case_d_third_play),
    ("U-D3", "U-C の Stop 後、保存して開き直してから Play", case_d_reopen),
    ("U-D4", "U-C をスリープしない剛体(sleepThreshold=0)で", case_d_no_sleep),
    ("U-D5", "角速度で U-C と同じ手順", case_d_angular),
    ("U-E1", "U-C の Stop 後、速度属性を削除して Stop 中に 0.5 で作り直してから再 Play", case_e_recreate_while_stopped),
    ("U-E2", "resetOnStop=false で U-C", case_e_no_reset_on_stop),
    ("U-F1", "U-C の再 Play 前に omni.physx.fabric を無効化", case_f_disable_before_replay),
    ("U-F2", "最初の Play は omni.physx.fabric 無効、再 Play だけ有効", case_f_enable_only_on_replay),
    ("U-F3", "U-C の再 Play で、ステップ前に PhysX が読み込んだ速度", case_f_parsed_velocity),
    ("U-G1", "属性なしから Play 中に USD 速度 0.6 → update ごとの速度", case_g_trace_absent),
    ("U-G2", "Play 前に属性を 0 で作成 → Play 中に USD 速度 0.6 → update ごとの速度", case_g_trace_preauthored),
    ("U-G3", "U-G1 を Fabric change tracking 停止中に", case_g_trace_absent_tracking_paused),
]


@bplib.guarded(rec, "U-00")
def env() -> None:
    rec.add("U-00", "バージョンと設定", **bplib.environment())


def make(test_id: str, desc: str, fn: Case) -> Callable[[], None]:
    @bplib.guarded(rec, test_id)
    def run() -> None:
        rec.add(test_id, desc, **fn())

    return run


env()
for _id, _desc, _fn in CASES:
    make(_id, _desc, _fn)()
bplib.finish(rec)
