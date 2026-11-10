# time_lab.py - headless measurements for 03_時間と物理ステップ
# usage: python.bat time_lab.py <experience.kit or ""> <out.jsonl>
import json
import os
import sys
import time

EXP = sys.argv[1] if len(sys.argv) > 1 else ""
OUT = sys.argv[2] if len(sys.argv) > 2 else "time_lab.jsonl"

from isaacsim import SimulationApp

simapp = SimulationApp({"headless": True}, experience=EXP)

import carb
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager as SM
from omni.kit.loop import _loop as kit_loop
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

settings = carb.settings.get_settings()
kapp = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
loop = kit_loop.acquire_loop_interface()
physx_iface = omni.physx.get_physx_interface()
fout = open(OUT, "w", encoding="utf-8")


def emit(kind, **kw):
    kw["kind"] = kind
    fout.write(json.dumps(kw, ensure_ascii=False) + "\n")
    fout.flush()


SET_KEYS = [
    "/app/player/useFixedTimeStepping",
    "/app/player/useFastMode",
    "/app/runLoops/main/rateLimitEnabled",
    "/app/runLoops/main/rateLimitFrequency",
    "/app/runLoops/main/manualModeEnabled",
    "/app/runLoops/main/syncToPresent",
    "/persistent/simulation/minFrameRate",
    "/app/player/playSimulations",
    "/physics/updateToUsd",
    "/physics/fabricEnabled",
]
emit("fingerprint", exp=EXP, settings={k: settings.get(k) for k in SET_KEYS},
     manual_step_size=loop.get_manual_step_size(), manual_mode=loop.get_manual_mode(),
     fabric_enabled=SM.is_fabric_enabled(), engine=SM.get_active_physics_engine())

# ---------------------------------------------------------------- counters
class C:
    pre = 0
    post = 0
    frames = 0
    load_s = 0.0


def on_pre(dt, ctx):
    C.pre += 1


def on_post(dt, ctx):
    C.post += 1


def on_frame(e):
    C.frames += 1


_frame_id = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=on_frame)


def on_update(e):
    if C.load_s > 0:
        time.sleep(C.load_s)


_upd_sub = kapp.get_update_event_stream().create_subscription_to_pop(on_update, name="time_lab_load")

_rt_stage = {"id": None, "stage": None}


def read_ext_time():
    try:
        from usdrt import Usd as RtUsd

        sid = omni.usd.get_context().get_stage_id()
        if _rt_stage["id"] != sid:
            _rt_stage["stage"] = RtUsd.Stage.Attach(sid)
            _rt_stage["id"] = sid
        p = _rt_stage["stage"].GetPrimAtPath("/ExternalSimulationTime")
        if not p or not p.IsValid():
            return "noprim"
        a = p.GetAttribute("omni:time")
        return a.Get() if a and a.IsValid() else "noattr"
    except Exception as ex:  # noqa
        return f"err:{ex}"


def cube_z_physx():
    tr = physx_iface.get_rigidbody_transformation("/World/Cube")
    if not tr or not tr.get("ret_val", False):
        return None
    return float(tr["position"][2])


def cube_z_usd():
    st = omni.usd.get_context().get_stage()
    prim = st.GetPrimAtPath("/World/Cube")
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    return float(m.ExtractTranslation()[2])


def cube_z_fabric():
    try:
        from usdrt import Usd as RtUsd, Rt

        st = _rt_stage["stage"] or RtUsd.Stage.Attach(omni.usd.get_context().get_stage_id())
        p = st.GetPrimAtPath("/World/Cube")
        xf = Rt.Xformable(p)
        if xf.HasWorldXform():
            return float(xf.GetWorldPositionAttr().Get()[2])
        a = p.GetAttribute("omni:fabric:worldMatrix")
        if a and a.IsValid():
            v = a.Get()
            return float(v[3][2])
        return "none"
    except Exception as ex:  # noqa
        return f"err:{ex}"


def sample():
    tau = tl.get_current_time()
    tau0 = tl.get_start_time()
    t = SM.get_simulation_time()
    h = SM.get_physics_dt()
    return dict(
        tau=tau, tau0=tau0, t=t, n=SM.get_num_physics_steps(), h=h, lag=(tau - tau0) - t,
        mono=SM._simulation_manager_interface.get_simulation_time_monotonic(),
        ext=read_ext_time(), playing=tl.is_playing(), sm_sim=SM.is_simulating(), sm_paused=SM.is_paused(),
        upd=kapp.get_update_number(),
    )


def updates(k, tag, extra=None):
    for i in range(k):
        C.pre = C.post = C.frames = 0
        tau_b = tl.get_current_time()
        w0 = time.perf_counter()
        simapp.update()
        w1 = time.perf_counter()
        row = sample()
        row.update(tag=tag, i=i, N=C.post, Npre=C.pre, frames=C.frames, d_tau=row["tau"] - tau_b,
                   wall_ms=(w1 - w0) * 1000.0)
        if extra:
            row.update(extra())
        emit("upd", **row)


def new_scene(S, Ctps):
    omni.usd.get_context().new_stage()
    for _ in range(3):
        simapp.update()
    st = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.SetTimeCodesPerSecond(Ctps)
    st.SetStartTimeCode(0)
    st.SetEndTimeCode(1e6)
    scene = UsdPhysics.Scene.Define(st, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(int(S))
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr("MBP")
    UsdGeom.Xform.Define(st, "/World")
    cube = UsdGeom.Cube.Define(st, "/World/Cube")
    cube.CreateSizeAttr(0.1)
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0, 0, 5.0))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
    tl.set_time_codes_per_second(Ctps)
    tl.set_start_time(0)
    tl.set_end_time(1e6)
    for _ in range(2):
        simapp.update()
    SM.register_callback(on_pre, event=SimulationEvent.PHYSICS_PRE_STEP)
    SM.register_callback(on_post, event=SimulationEvent.PHYSICS_POST_STEP)
    emit("scene", S=S, C=Ctps, tcps_tl=tl.get_time_codes_per_seconds(), h=SM.get_physics_dt(),
         loop_dt=RenderingManager.get_dt())


def play_and_log(k, tag, extra=None):
    C.pre = C.post = 0
    b = sample()
    tl.play()
    a = sample()
    emit("play_call", tag=tag, before=b, after_play_call=a, pre_in_call=C.pre, post_in_call=C.post)
    updates(k, tag, extra)


def stop():
    tl.stop()
    simapp.update()
    simapp.update()


def phase(name):
    emit("phase", name=name)
    print("PHASE", name, flush=True)


# ================================================================ E1 step sequences
for S, Ctps in [(60, 60), (120, 60), (100, 30), (30, 60)]:
    phase(f"E1_seq_S{S}_C{Ctps}")
    new_scene(S, Ctps)
    play_and_log(60, f"seq_S{S}_C{Ctps}")
    stop()
    emit("after_stop", tag=f"seq_S{S}_C{Ctps}", **sample(), view=str(SM.get_physics_sim_view()))

# ================================================================ E2 pause + manual step (S60 C60)
phase("E2_pause_step")
new_scene(60, 60)
play_and_log(120, "E2_play")
tl.pause()
updates(3, "E2_paused", extra=lambda: dict(z_physx=cube_z_physx()))
view = SM.get_physics_sim_view()
emit("paused_view", view=str(view), valid=getattr(view, "is_valid", None))
b = sample()
zb = dict(physx=cube_z_physx(), usd=cube_z_usd(), fabric=cube_z_fabric())
C.pre = C.post = C.frames = 0
upd_b = kapp.get_update_number()
print("before:", SM.get_num_physics_steps(), SM.get_simulation_time())
SM.step(steps=3)
print("after :", SM.get_num_physics_steps(), SM.get_simulation_time())
a = sample()
za = dict(physx=cube_z_physx(), usd=cube_z_usd(), fabric=cube_z_fabric())
emit("manual_step", before=b, after=a, z_before=zb, z_after=za, pre=C.pre, post=C.post, frames=C.frames,
     upd_before=upd_b, upd_after=kapp.get_update_number())
updates(3, "E2_paused_after_step", extra=lambda: dict(z_physx=cube_z_physx(), z_usd=cube_z_usd(), z_fabric=cube_z_fabric()))
tl.play()
updates(30, "E2_resumed")
# second manual step while PLAYING
b = sample()
SM.step(steps=3)
emit("manual_step_playing", before=b, after=sample())
updates(10, "E2_after_step_playing")
stop()
emit("after_stop", tag="E2", **sample(), view=str(SM.get_physics_sim_view()))
# play again after stop: does tau restart? mono continue?
play_and_log(5, "E2_replay")
stop()

# ================================================================ E3 remainder after manual step (S100 C30)
for with_step in (False, True):
    tag = f"E3_step{int(with_step)}"
    phase(tag)
    new_scene(100, 30)
    play_and_log(5, tag + "_play")
    tl.pause()
    updates(2, tag + "_paused")
    if with_step:
        SM.step(steps=3)
    tl.play()
    updates(9, tag + "_resumed")
    stop()

# ================================================================ E4 playSimulations=false
phase("E4_playSimulations")
new_scene(60, 60)
play_and_log(10, "E4_on")
settings.set_bool("/app/player/playSimulations", False)
updates(10, "E4_off")
settings.set_bool("/app/player/playSimulations", True)
updates(10, "E4_on_again")
stop()

# ================================================================ E5 minFrameRate truncation
phase("E5_minFrameRate")
old_mfr = settings.get("/persistent/simulation/minFrameRate")
new_scene(60, 10)
settings.set_int("/persistent/simulation/minFrameRate", 30)
play_and_log(15, "E5_mfr30")
stop()
settings.set_int("/persistent/simulation/minFrameRate", 0)
new_scene(60, 10)
play_and_log(15, "E5_mfr0")
stop()
settings.set_int("/persistent/simulation/minFrameRate", old_mfr if old_mfr is not None else 30)

# ================================================================ E6 rate limit / load / determinism
phase("E6_rate")
old_rl = settings.get("/app/runLoops/main/rateLimitEnabled")
old_rf = settings.get("/app/runLoops/main/rateLimitFrequency")
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_float("/app/runLoops/main/rateLimitFrequency", 60.0)
for load_ms in (0, 5, 30):
    new_scene(60, 60)
    C.load_s = load_ms / 1000.0
    play_and_log(90, f"E6_load{load_ms}", extra=lambda: dict(z_physx=cube_z_physx()))
    C.load_s = 0.0
    stop()
# rate limit off, no load
settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
new_scene(60, 60)
play_and_log(60, "E6_nolimit", extra=lambda: dict(z_physx=cube_z_physx()))
stop()

# thinning: C=30 with loop at 60 Hz, useFastMode false/true
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
for fast in (False, True):
    settings.set_bool("/app/player/useFastMode", fast)
    new_scene(100, 30)
    play_and_log(30, f"E6_thin_fast{int(fast)}")
    stop()
settings.set_bool("/app/player/useFastMode", False)

# ================================================================ E7 timeline advance modes
phase("E7_modes")
old_fixed = settings.get("/app/player/useFixedTimeStepping")
old_manual = settings.get("/app/runLoops/main/manualModeEnabled")
for fixed, manual, load_ms in [(True, True, 0), (True, False, 0), (False, True, 0), (False, True, 30),
                                (False, False, 0), (False, False, 30)]:
    settings.set_bool("/app/player/useFixedTimeStepping", fixed)
    settings.set_bool("/app/runLoops/main/manualModeEnabled", manual)
    new_scene(60, 30)
    C.load_s = load_ms / 1000.0
    play_and_log(20, f"E7_fixed{int(fixed)}_manual{int(manual)}_load{load_ms}",
                 extra=lambda: dict(manual_step=loop.get_manual_step_size(), manual_mode=loop.get_manual_mode()))
    C.load_s = 0.0
    stop()
settings.set_bool("/app/player/useFixedTimeStepping", bool(old_fixed))
settings.set_bool("/app/runLoops/main/manualModeEnabled", bool(old_manual))
settings.set_bool("/app/runLoops/main/rateLimitEnabled", bool(old_rl))
if old_rf is not None:
    settings.set_float("/app/runLoops/main/rateLimitFrequency", float(old_rf))

# ================================================================ E8 long run drift (S100 C30)
phase("E8_long")
new_scene(100, 30)
play_and_log(600, "E8_long")
stop()

phase("done")
fout.close()
simapp.close()
