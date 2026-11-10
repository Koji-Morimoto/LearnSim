# time_lab3.py - what paces the loop: rate limit vs timeline target framerate
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "time_lab.py")
code = open(SRC, encoding="utf-8").read().split("# ================================================================ E1")[0]
exec(compile(code, SRC, "exec"))

emit("tl", target=tl.get_target_framerate())
RL = "/app/runLoops/main/rateLimitEnabled"
RF = "/app/runLoops/main/rateLimitFrequency"
FIX = "/app/player/useFixedTimeStepping"
THR = "/exts/isaacsim.core.throttling/enable_manualmode"
settings.set_bool(THR, False)
cases = [
    # fixed, rl, rf, target
    (False, True, 30.0, 60),
    (False, True, 120.0, 60),
    (False, False, 120.0, 60),
    (True, False, 120.0, 30),
    (True, True, 120.0, 120),
    (True, True, 30.0, 60),
]
for fixed, rl, rf, target in cases:
    tag = f"H_fixed{int(fixed)}_rl{int(rl)}_rf{int(rf)}_tgt{target}"
    phase(tag)
    settings.set_bool(FIX, fixed)
    settings.set_bool(RL, rl)
    settings.set_float(RF, rf)
    settings.set_bool("/app/runLoops/main/manualModeEnabled", False)
    loop.set_manual_mode(False)
    new_scene(60, 60)
    tl.set_target_framerate(target)
    emit("cfg", tag=tag, target=tl.get_target_framerate(), rl=settings.get(RL), rf=settings.get(RF), fixed=settings.get(FIX))
    play_and_log(40, tag)
    stop()
phase("done")
fout.close()
simapp.close()
