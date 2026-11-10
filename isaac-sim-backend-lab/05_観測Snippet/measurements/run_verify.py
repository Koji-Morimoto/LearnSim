# run_verify.py - 追加の確認（USDへの書き戻しを4つとも止めた場合の費用、Pause 中の asyncRendering、pose_freshness の none）
import builtins
import os
import statistics
import sys
import time

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

phase("V1_async_in_pause")
stage = new_stage()
add_cube(stage)
set_state("Play", settle_updates=5)
emit("async", state="Play", value=settings.get("/app/asyncRendering"))
set_state("Pause", settle_updates=3)
emit("async", state="Pause 3回後", value=settings.get("/app/asyncRendering"))
run_updates(12)
emit("async", state="Pause 15回後", value=settings.get("/app/asyncRendering"))
set_state("Stop", settle_updates=15)
emit("async", state="Stop 15回後", value=settings.get("/app/asyncRendering"))

phase("V2_writeback_none_all")
settings.set_bool("/app/player/useFixedTimeStepping", False)
settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
settings.set_bool("/app/runLoops/main/manualModeEnabled", True)
for output_mode in ("usd", "none_all", "fabric"):
    stage = new_stage()
    for index in range(1000):
        add_cube(stage, f"/World/Cubes/c_{index}", position=(0.3 * (index % 32), 0.3 * (index // 32), 5.0))
    SimulationManager.enable_fabric(output_mode == "fabric")
    if output_mode == "none_all":
        for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateForceSensorsToUsd"):
            settings.set_bool(key, False)
    set_state("Play", settle_updates=20)
    durations = run_updates(120)
    emit("writeback", cube_count=1000, output_mode=output_mode, update_ms_median=statistics.median(durations) * 1000)
    set_state("Stop")
SimulationManager.enable_fabric(False)
settings.set_bool("/app/player/useFixedTimeStepping", True)
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_bool("/app/runLoops/main/manualModeEnabled", False)

phase("V3_pose_freshness_none")
stage = new_stage()
add_cube(stage, position=(0.0, 0.0, 1.0))
with captured_stdout("pose_freshness none(4つとも止める) Stop"):
    run_snippet("pose_freshness_probe", {"OUTPUT_MODE: OutputMode = None": "OUTPUT_MODE: OutputMode = 'none'"})
    run_updates(12)
set_state("Play", settle_updates=20)
with captured_stdout("pose_freshness none(4つとも止める) Play"):
    run_snippet("pose_freshness_probe")
    run_updates(12)
set_state("Stop")
RenderingManager.deregister_callback(builtins._pose_probe_id)
SimulationManager.enable_fabric(False)
close()
