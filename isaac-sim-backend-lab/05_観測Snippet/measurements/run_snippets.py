# run_snippets.py - 状態ごとの設定の変化と、各 Snippet を Stop / Play / Pause で実行した出力を記録する
import os
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import isaacsim.core.experimental.utils.backend as backend_utils
from isaacsim.core.experimental.prims import Articulation, RigidPrim

WATCHED_KEYS = [
    "/app/runLoops/main/rateLimitEnabled",
    "/app/runLoops/main/rateLimitFrequency",
    "/app/runLoops/main/manualModeEnabled",
    "/app/player/useFixedTimeStepping",
    "/app/player/useFastMode",
    "/app/asyncRendering",
    "/app/asyncRenderingLowLatency",
    "/exts/isaacsim.core.throttling/enable_async",
    "/exts/isaacsim.core.throttling/enable_manualmode",
    "/persistent/simulation/minFrameRate",
    "/physics/updateToUsd",
    "/app/player/playSimulations",
    "/physics/cudaDevice",
    "/physics/suppressReadback",
]


def snapshot(label):
    """状態に関係する設定と値をまとめて記録する。"""
    emit(
        "snapshot",
        label=label,
        state=state_name(),
        settings={key: settings.get(key) for key in WATCHED_KEYS},
        loop_manual_mode=loop_runner.get_manual_mode(),
        target_framerate=timeline.get_target_framerate(),
        rendering_dt=RenderingManager.get_dt(),
        update_wall_ms=sum(run_updates(30)[10:]) / 20 * 1000,
    )


# ---------------------------------------------------------------- A1 状態ごとの設定の変化
phase("A1_settings_by_state")
stage = new_stage()
add_cube(stage)
snapshot("Stop(起動直後)")
for target in ("Play", "Pause", "Play", "Stop"):
    set_state(target)
    snapshot(target)

# 目標フレームレートと timeCodesPerSecond を変えたとき、Play 中の rateLimitFrequency がどうなるか
for time_codes, target_framerate, preset_frequency in [(60, 30, None), (60, 120, None), (30, 60, None), (60, 60, 200.0)]:
    stage = new_stage(time_codes_per_second=time_codes)
    add_cube(stage)
    timeline.set_target_framerate(target_framerate)
    if preset_frequency is not None:
        settings.set_float("/app/runLoops/main/rateLimitFrequency", preset_frequency)
    run_updates(3)
    snapshot(f"C={time_codes} target={target_framerate} preset={preset_frequency} Stop")
    set_state("Play")
    snapshot(f"C={time_codes} target={target_framerate} preset={preset_frequency} Play")
    set_state("Pause")
    snapshot(f"C={time_codes} target={target_framerate} preset={preset_frequency} Pause")
    set_state("Stop")
    snapshot(f"C={time_codes} target={target_framerate} preset={preset_frequency} Stop(後)")
timeline.set_target_framerate(60)
settings.set_float("/app/runLoops/main/rateLimitFrequency", 120.0)

# ---------------------------------------------------------------- A2 env_fingerprint
phase("A2_env_fingerprint")
stage = new_stage()
add_cube(stage)
for target in ("Stop", "Play", "Pause"):
    set_state(target)
    with captured_stdout(f"env_fingerprint {target}"):
        run_snippet("env_fingerprint")
set_state("Stop")
# 副作用の確認：Physics Scene のないステージで is_gpu_dynamics_enabled を呼ぶと /PhysicsScene が作られるか
omni.usd.get_context().new_stage()
run_updates(3)
stage = omni.usd.get_context().get_stage()
before = [str(prim.GetPath()) for prim in stage.Traverse()]
with captured_stdout("env_fingerprint no-physics-scene"):
    run_snippet("env_fingerprint")
after_snippet = [str(prim.GetPath()) for prim in stage.Traverse()]
try:
    SimulationManager.is_gpu_dynamics_enabled()
except Exception as error:
    emit("note", text=f"is_gpu_dynamics_enabled raised {type(error).__name__}: {error}")
after_direct = [str(prim.GetPath()) for prim in stage.Traverse()]
emit("side_effect", before=before, after_snippet=after_snippet, after_direct_call=after_direct)

# ---------------------------------------------------------------- A3 step_probe
phase("A3_step_probe")
stage = new_stage()
add_cube(stage)
with captured_stdout("step_probe Stopで実行→Play"):
    run_snippet("step_probe")
    set_state("Play", settle_updates=10)
with captured_stdout("step_probe Play中に実行"):
    run_snippet("step_probe")
    run_updates(10)
set_state("Pause")
with captured_stdout("step_probe Pause中に予約→Pauseのまま5回更新"):
    import builtins
    builtins.schedule_after(2, lambda: print("fired in pause test"), "pause-test")
    run_updates(5)
with captured_stdout("step_probe Pause中の予約→Play再開"):
    set_state("Play", settle_updates=5)
set_state("Stop")
with captured_stdout("step_probe Stop後に再実行→Play"):
    run_snippet("step_probe")
    set_state("Play", settle_updates=8)
set_state("Stop")
with captured_stdout("step_probe Stop後に再実行せずPlay"):
    set_state("Play", settle_updates=8)
set_state("Stop")
for callback_id in getattr(builtins, "_step_probe_ids", []):
    SimulationManager.deregister_callback(callback_id)
builtins._step_probe_ids = []

# ---------------------------------------------------------------- A4 clock_probe
phase("A4_clock_probe")
stage = new_stage()
add_cube(stage)
with captured_stdout("clock_probe Stopで実行→Play"):
    run_snippet("clock_probe")
    set_state("Play", settle_updates=35)
with captured_stdout("clock_probe Play中に実行"):
    run_snippet("clock_probe")
    run_updates(35)
set_state("Pause")
with captured_stdout("clock_probe Pause中に実行"):
    run_snippet("clock_probe")
    run_updates(35)
set_state("Stop")
SimulationManager.deregister_callback(builtins._clock_probe["step_id"])
RenderingManager.deregister_callback(builtins._clock_probe["frame_id"])
builtins._clock_probe = None

# ---------------------------------------------------------------- A5 pose_freshness_probe（物理の出力先3通り × 3状態）
phase("A5_pose_freshness")
for output_mode in ("usd", "none", "fabric"):
    stage = new_stage()
    add_cube(stage, position=(0.0, 0.0, 1.0))
    replacements = {"OUTPUT_MODE: OutputMode = None": f"OUTPUT_MODE: OutputMode = {output_mode!r}"}
    with captured_stdout(f"pose_freshness {output_mode} Stop"):
        run_snippet("pose_freshness_probe", replacements)
        run_updates(12)
    set_state("Play", settle_updates=20)
    with captured_stdout(f"pose_freshness {output_mode} Play"):
        run_snippet("pose_freshness_probe")
        run_updates(12)
    set_state("Pause")
    with captured_stdout(f"pose_freshness {output_mode} Pause"):
        run_snippet("pose_freshness_probe")
        run_updates(12)
    set_state("Stop")
    with captured_stdout(f"pose_freshness {output_mode} Stop(Play後)"):
        run_snippet("pose_freshness_probe")
        run_updates(12)
RenderingManager.deregister_callback(builtins._pose_probe_id)
builtins._pose_probe_id = None
SimulationManager.enable_fabric(False)

# ---------------------------------------------------------------- A6 backend_spy
phase("A6_backend_spy")
stage = new_stage()
add_cube(stage)
add_articulation(stage)
run_updates(3)
with captured_stdout("backend_spy ON"):
    run_snippet("backend_spy")
wrappers_made_in_stop = {}
for target in ("Stop", "Play", "Pause", "Stop"):
    set_state(target)
    with captured_stdout(f"backend_spy {target} 新しく作ったラッパー"):
        robot = Articulation("/World/robot")
        print("dof_positions:", robot.get_dof_positions().numpy().tolist())
        cube = RigidPrim("/World/Cube")
        print("cube z:", cube.get_world_poses()[0].numpy().tolist())
        if not wrappers_made_in_stop:
            wrappers_made_in_stop = {"robot": robot, "cube": cube}
    with captured_stdout(f"backend_spy {target} Stop中に作ったラッパー"):
        print("dof_positions:", wrappers_made_in_stop["robot"].get_dof_positions().numpy().tolist())
        print("cube z:", wrappers_made_in_stop["cube"].get_world_poses()[0].numpy().tolist())
with captured_stdout("backend_spy OFF"):
    run_snippet("backend_spy", {"SPY_ENABLED: typing.Final[bool] = True": "SPY_ENABLED: typing.Final[bool] = False"})

# ---------------------------------------------------------------- A7 strict_backend
phase("A7_strict_backend")
stage = new_stage()
add_cube(stage)
for target in ("Stop", "Play", "Pause", "Stop"):
    set_state(target)
    with captured_stdout(f"strict_backend {target}"):
        run_snippet("strict_backend")

close()
