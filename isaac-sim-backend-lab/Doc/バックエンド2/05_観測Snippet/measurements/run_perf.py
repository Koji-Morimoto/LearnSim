# run_perf.py - 重い処理の候補を測り、軽くする方法を適用した後の値と比べる
import builtins
import os
import statistics
import sys
import time

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import isaacsim.core.experimental.utils.backend as backend_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationEvent

run_snippet("bench_probe")
measure = builtins.measure


def record(label, samples, **extra):
    """measure の生データから中央値と最小値をマイクロ秒で記録する。"""
    emit("perf", label=label, median_us=statistics.median(samples) * 1e6, min_us=min(samples) * 1e6, **extra)


def add_cubes(stage, cube_count):
    """cube_count 個の剛体の立方体を格子状に置き、パスのリストを返す。"""
    side = int(np.ceil(np.sqrt(cube_count)))
    paths = []
    for index in range(cube_count):
        path = f"/World/Cubes/c_{index}"
        add_cube(stage, path, position=(0.3 * (index % side), 0.3 * (index // side), 5.0))
        paths.append(path)
    return paths


# ---------------------------------------------------------------- P0 ログの設定
phase("P0_log_settings")
emit("log_settings", values={key: settings.get(key) for key in ["/log/level", "/log/outputStreamLevel", "/log/fileLogLevel", "/log/file", "/log/enabled", "/log/flushStandardStreamOutput", "/exts/omni.kit.window.script_editor/maxLogLines"]})

# ---------------------------------------------------------------- P1 print とログ出力の費用
phase("P1_print")
with captured_stdout("P1 print"):
    record("print 1行", measure("print 1行", lambda: print("timeline.is_playing:", timeline.is_playing()), repeat_count=200))
    lines = [f"line {index}" for index in range(100)]
    record("print 100行を1回にまとめる", measure("print 100行まとめ", lambda: print("\n".join(lines)), repeat_count=50), lines_per_call=100)
    record("文字列を作るだけ", measure("format only", lambda: f"timeline.is_playing: {timeline.is_playing()}", repeat_count=200))
record("carb.log_info（既定のログレベルでは出力されない）", measure("carb.log_info", lambda: carb.log_info("lab perf info"), repeat_count=200))
record("carb.log_warn（出力される）", measure("carb.log_warn", lambda: carb.log_warn("lab perf warn"), repeat_count=200))
# Script Editor と同じ経路（python scripting の execute_string）で print した場合
emit("script_editor", enabled=kit_app.get_extension_manager().is_extension_enabled("omni.kit.window.script_editor"))
script_code = (
    "import builtins, time, statistics\n"
    "samples = []\n"
    "for i in range(200):\n"
    "    start = time.perf_counter()\n"
    "    print('script editor path line', i)\n"
    "    samples.append(time.perf_counter() - start)\n"
    "builtins._lab_script_print = statistics.median(samples) * 1e6\n"
)
kit_app.get_python_scripting().execute_string(script_code, "lab-perf", False)
run_updates(2)
emit("perf", label="print 1行（execute_string 経由。Script Editor と同じ実行経路）", median_us=getattr(builtins, "_lab_script_print", None))

# ---------------------------------------------------------------- P2 ラッパーを毎回作る費用
phase("P2_wrapper_creation")
stage = new_stage()
add_cube(stage)
run_updates(2)
xform_once = XformPrim("/World/Cube")
with captured_stdout("P2"):
    record("XformPrim を毎回作って読む", measure("create+read", lambda: XformPrim("/World/Cube").get_world_poses(), repeat_count=50))
    record("作っておいた XformPrim で読む", measure("reuse+read", lambda: xform_once.get_world_poses(), repeat_count=200))

# ---------------------------------------------------------------- P3 プリム数 N と backend ごとの費用
phase("P3_scaling")
for cube_count in (1, 10, 100, 1000):
    stage = new_stage()
    paths = add_cubes(stage, cube_count)
    run_updates(3)
    xform = XformPrim(paths)
    rigid = RigidPrim(paths)
    with captured_stdout(f"P3 N={cube_count}"):
        for state_label in ("Stop", "Play"):
            set_state(state_label, settle_updates=5)
            for backend in ("usd", "usdrt", "fabric"):
                def read_with(backend_name=backend):
                    with backend_utils.use_backend(backend_name):
                        xform.get_world_poses()
                repeat = 20 if cube_count >= 1000 and backend != "fabric" else 50
                record(f"XformPrim.get_world_poses {backend}", measure(f"N={cube_count} {state_label} xform {backend}", read_with, repeat_count=repeat), cube_count=cube_count, state=state_label)
            record("RigidPrim.get_world_poses 既定", measure(f"N={cube_count} {state_label} rigid default", lambda: rigid.get_world_poses(), repeat_count=50), cube_count=cube_count, state=state_label)
    set_state("Stop")

# ---------------------------------------------------------------- P4 1個ずつのループと、まとめて1回の呼び出し
phase("P4_loop_vs_batch")
stage = new_stage()
paths = add_cubes(stage, 100)
run_updates(3)
set_state("Play", settle_updates=5)
rigid = RigidPrim(paths)
targets = np.zeros((100, 3), dtype=np.float32)
targets[:, 2] = 5.0


def set_one_by_one():
    for index in range(100):
        rigid.set_world_poses(positions=targets[index : index + 1], indices=[index])


with captured_stdout("P4"):
    record("set_world_poses を1個ずつ100回", measure("loop 100", set_one_by_one, repeat_count=10), cube_count=100)
    record("set_world_poses を100個まとめて1回", measure("batch 100", lambda: rigid.set_world_poses(positions=targets), repeat_count=50), cube_count=100)
set_state("Stop")

# ---------------------------------------------------------------- P5 物理の結果を USD に書き戻す費用（メインループの待ちを外して測る）
phase("P5_writeback")
settings.set_bool("/app/player/useFixedTimeStepping", False)
settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
settings.set_bool("/app/runLoops/main/manualModeEnabled", True)
step_time = {"start": 0.0, "total": 0.0, "count": 0}


def on_pre(dt, context):
    step_time["start"] = time.perf_counter()


def on_post(dt, context):
    step_time["total"] += time.perf_counter() - step_time["start"]
    step_time["count"] += 1


for cube_count in (100, 1000):
    for output_mode in ("usd", "none", "fabric"):
        stage = new_stage()
        add_cubes(stage, cube_count)
        if output_mode == "usd":
            SimulationManager.enable_fabric(False)
        elif output_mode == "fabric":
            SimulationManager.enable_fabric(True)
        else:
            SimulationManager.enable_fabric(False)
            settings.set_bool("/physics/updateToUsd", False)
        pre_id = SimulationManager.register_callback(on_pre, event=SimulationEvent.PHYSICS_PRE_STEP)
        post_id = SimulationManager.register_callback(on_post, event=SimulationEvent.PHYSICS_POST_STEP)
        set_state("Play", settle_updates=20)
        step_time.update(total=0.0, count=0)
        durations = run_updates(120)
        emit(
            "writeback",
            cube_count=cube_count,
            output_mode=output_mode,
            update_ms_median=statistics.median(durations) * 1000,
            update_ms_mean=statistics.mean(durations) * 1000,
            physics_step_ms=step_time["total"] / max(step_time["count"], 1) * 1000,
            steps=step_time["count"],
        )
        set_state("Stop")
        SimulationManager.deregister_callback(pre_id)
        SimulationManager.deregister_callback(post_id)
SimulationManager.enable_fabric(False)

# ---------------------------------------------------------------- P6 Kit のログ出力の量と、ログレベルを上げたときの差
phase("P6_log_volume")
log_path = settings.get("/log/file")
for log_level in ("default", "error"):
    stage = new_stage()
    add_cubes(stage, 100)
    if log_level == "error":
        settings.set_string("/log/level", "error")
        settings.set_string("/log/outputStreamLevel", "error")
        settings.set_string("/log/fileLogLevel", "error")
    set_state("Play", settle_updates=20)
    size_before = os.path.getsize(log_path) if log_path and os.path.exists(log_path) else None
    durations = run_updates(300)
    size_after = os.path.getsize(log_path) if log_path and os.path.exists(log_path) else None
    emit("log_volume", log_level=log_level, log_path=log_path, bytes_during_300_updates=(size_after - size_before) if size_before is not None else None, update_ms_median=statistics.median(durations) * 1000)
    set_state("Stop")
settings.set_string("/log/level", "Warn")
settings.set_bool("/app/player/useFixedTimeStepping", True)
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_bool("/app/runLoops/main/manualModeEnabled", False)

# ---------------------------------------------------------------- P7 GPU の配列と同期・CPU への写し
phase("P7_gpu")
settings.set_bool("/physics/suppressReadback", True)
settings.set_int("/physics/cudaDevice", 0)
stage = new_stage(gpu=True)
paths = add_cubes(stage, 1000)
run_updates(3)
set_state("Play", settle_updates=20)
rigid = RigidPrim(paths)
positions, _ = rigid.get_world_poses()
emit("gpu", device=str(SimulationManager.get_physics_sim_device()), array_device=str(positions.device))
with captured_stdout("P7"):
    record("GPU: get_world_poses sync なし", measure("gpu read nosync", lambda: rigid.get_world_poses(), repeat_count=100), cube_count=1000)
    record("GPU: get_world_poses sync あり", measure("gpu read sync", lambda: rigid.get_world_poses(), repeat_count=100, sync=True), cube_count=1000)
    record("GPU: get_world_poses して .numpy() で CPU に写す", measure("gpu read numpy", lambda: rigid.get_world_poses()[0].numpy(), repeat_count=100), cube_count=1000)
    record("GPU: 写すのを先頭1個だけにする", measure("gpu read first", lambda: rigid.get_world_poses(indices=[0])[0].numpy(), repeat_count=100), cube_count=1000)
set_state("Stop")
settings.set_bool("/physics/suppressReadback", False)

close()
