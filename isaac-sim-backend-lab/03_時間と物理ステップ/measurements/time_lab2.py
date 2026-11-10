# time_lab2.py - follow-up measurements (minFrameRate=0, remainder reset, real-time mode, pacing, start time)
import os
import sys

sys.argv = [sys.argv[0], sys.argv[1], sys.argv[2]]
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "time_lab.py")
code = open(SRC, encoding="utf-8").read()
# reuse helpers only: cut before the experiment section
code = code.split("# ================================================================ E1")[0]
exec(compile(code, SRC, "exec"))

MFR = "/persistent/simulation/minFrameRate"

# ---- A: does new_stage reset minFrameRate?
phase("A_mfr_reset")
settings.set_int(MFR, 0)
emit("mfr", when="after set 0", v=settings.get(MFR))
new_scene(60, 60)
emit("mfr", when="after new_scene", v=settings.get(MFR))
stop()

# ---- B: S100 C30 with minFrameRate 0, fast mode on/off
for fast in (True, False):
    settings.set_bool("/app/player/useFastMode", fast)
    tag = f"B_S100C30_mfr0_fast{int(fast)}"
    phase(tag)
    new_scene(100, 30)
    settings.set_int(MFR, 0)
    emit("mfr", when=tag, v=settings.get(MFR))
    play_and_log(30, tag)
    stop()

# ---- C: S60 C10 minFrameRate 0 vs 30 (fast mode on, no thinning)
settings.set_bool("/app/player/useFastMode", True)
for mfr in (0, 30):
    tag = f"C_S60C10_mfr{mfr}"
    phase(tag)
    new_scene(60, 10)
    settings.set_int(MFR, mfr)
    emit("mfr", when=tag, v=settings.get(MFR))
    play_and_log(12, tag)
    stop()

# ---- D: remainder after manual step (S100 C30, mfr 0, fast on)
for with_step in (False, True):
    tag = f"D_step{int(with_step)}"
    phase(tag)
    new_scene(100, 30)
    settings.set_int(MFR, 0)
    play_and_log(5, tag + "_play")
    tl.pause()
    updates(2, tag + "_paused")
    if with_step:
        SM.step(steps=3)
    tl.play()
    updates(9, tag + "_resumed")
    stop()
settings.set_bool("/app/player/useFastMode", False)

# ---- E: real-time timeline (no fixed stepping, no manual mode at all)
phase("E_realtime")
THR = "/exts/isaacsim.core.throttling/enable_manualmode"
emit("thr", v=settings.get(THR))
old_thr = settings.get(THR)
settings.set_bool(THR, False)
settings.set_bool("/app/player/useFixedTimeStepping", False)
settings.set_bool("/app/runLoops/main/manualModeEnabled", False)
loop.set_manual_mode(False)
for load_ms in (0, 30):
    new_scene(60, 60)
    settings.set_int(MFR, 0)
    C.load_s = load_ms / 1000.0
    play_and_log(20, f"E_rt_load{load_ms}", extra=lambda: dict(manual_mode=loop.get_manual_mode()))
    C.load_s = 0.0
    stop()
settings.set_bool("/app/player/useFixedTimeStepping", True)
settings.set_bool(THR, bool(old_thr))

# ---- F: pacing
phase("F_pacing")
for rl, f, sync in [(True, 30.0, True), (False, 60.0, False), (True, 60.0, False)]:
    settings.set_bool("/app/runLoops/main/rateLimitEnabled", rl)
    settings.set_float("/app/runLoops/main/rateLimitFrequency", f)
    settings.set_bool("/app/runLoops/main/syncToPresent", sync)
    new_scene(60, 60)
    play_and_log(40, f"F_rl{int(rl)}_f{int(f)}_sync{int(sync)}")
    stop()
settings.set_bool("/app/runLoops/main/syncToPresent", True)
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_float("/app/runLoops/main/rateLimitFrequency", 120.0)

# ---- G: start time != 0
phase("G_start")
new_scene(60, 60)
tl.set_start_time(1.0)
tl.set_current_time(1.0)
simapp.update()
play_and_log(10, "G_play")
tl.stop()
simapp.update()
emit("after_stop", tag="G", **sample())
phase("done")
fout.close()
simapp.close()
