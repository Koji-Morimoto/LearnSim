# run_note_blocks.py - ノートに載せたコードブロックを、Script Editor と同じ実行経路（execute_string）で順に実行し、エラーが出ないかを確かめる
import builtins
import os
import re
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import carb.eventdispatcher

NOTE_DIR = os.path.join(LAB_ROOT, "05_観測Snippet")
outputs = {"stdout": [], "stderr": []}
dispatcher = carb.eventdispatcher.get_eventdispatcher()
subscriptions = [
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDOUT_IMMEDIATE, on_event=lambda event: outputs["stdout"].append(event.get("text")), observer_name="lab stdout"),
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDERR_IMMEDIATE, on_event=lambda event: outputs["stderr"].append(event.get("text")), observer_name="lab stderr"),
]


def note_blocks(note_name):
    with open(os.path.join(NOTE_DIR, note_name), encoding="utf-8") as note_file:
        return re.findall(r"```python\n(.*?)\n```", note_file.read(), re.S)


def run_code(label, code, updates_after=1):
    outputs["stdout"].clear()
    outputs["stderr"].clear()
    kit_app.get_python_scripting().execute_string(code, label, False)
    run_updates(updates_after)
    emit("block", label=label, stdout="".join(outputs["stdout"])[-3000:], stderr="".join(outputs["stderr"])[-3000:], ok=not outputs["stderr"])
    print(("OK  " if not outputs["stderr"] else "ERR ") + label, flush=True)


def run_block(note_name, index, updates_after=1):
    run_code(f"{note_name}[{index}]", note_blocks(note_name)[index], updates_after)


def run_snippet_file(snippet_name):
    with open(os.path.join(LAB_ROOT, "snippets", snippet_name + ".py"), encoding="utf-8") as source_file:
        run_code(f"snippet {snippet_name}", source_file.read())


def restore_defaults():
    SimulationManager.enable_fabric(False)
    settings.set_bool("/app/player/playSimulations", True)
    settings.set_bool("/app/player/useFastMode", False)
    settings.set_bool("/app/player/useFixedTimeStepping", True)
    settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
    settings.set_int("/persistent/simulation/minFrameRate", 30)
    settings.set_bool("/physics/suppressReadback", False)
    timeline.set_target_framerate(60)


phase("S0_namespace")
run_code("namespace set", "lab_shared_value = 41")
run_code("namespace read", "print('shared:', lab_shared_value + 1)")

phase("N01")
stage = new_stage(); add_cube(stage)
run_block("01_env_fingerprint.md", 1)
restore_defaults()

phase("N02")
stage = new_stage(); add_cube(stage)
run_snippet_file("step_probe")
run_block("02_step_probe.md", 3, updates_after=60)
set_state("Stop")

phase("N03")
stage = new_stage(); add_cube(stage)
run_block("03_clock_probe.md", 1)
run_snippet_file("clock_probe")
run_block("03_clock_probe.md", 3, updates_after=120)
set_state("Stop"); restore_defaults()

phase("N04")
stage = new_stage(); add_articulation(stage)
set_state("Play", settle_updates=5)
run_snippet_file("bench_probe")
run_block("04_bench_probe.md", 1)
set_state("Stop")

phase("N05")
omni.usd.get_context().new_stage(); run_updates(3)
UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World")
run_block("05_pose_freshness_probe.md", 2)
run_block("05_pose_freshness_probe.md", 3, updates_after=250)
set_state("Stop")
run_block("05_pose_freshness_probe.md", 0)
restore_defaults()

phase("N06")
omni.usd.get_context().new_stage(); run_updates(3)
UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World")
run_block("06_backend_spy.md", 1)
run_snippet_file("backend_spy")
run_block("06_backend_spy.md", 2)
run_block("06_backend_spy.md", 3, updates_after=20)
set_state("Stop")
run_code("spy off", open(os.path.join(LAB_ROOT, "snippets", "backend_spy.py"), encoding="utf-8").read().replace("SPY_ENABLED: typing.Final[bool] = True", "SPY_ENABLED: typing.Final[bool] = False"))

phase("N07")
stage = new_stage(); add_cube(stage)
run_block("07_strict_backend.md", 1, updates_after=20)
set_state("Stop")

phase("N09")
stage = new_stage(); add_cube(stage)
run_block("08_重い処理と軽くする方法.md", 1)
run_block("08_重い処理と軽くする方法.md", 2)
set_state("Play", settle_updates=5)
run_block("08_重い処理と軽くする方法.md", 3, updates_after=320)
set_state("Stop")
run_block("08_重い処理と軽くする方法.md", 4)
settings.set_string("/log/level", "Info"); settings.set_string("/log/outputStreamLevel", "Warning"); settings.set_string("/log/fileLogLevel", "Info")
run_block("08_重い処理と軽くする方法.md", 5)
run_block("08_重い処理と軽くする方法.md", 6)
set_state("Play", settle_updates=5)
run_block("08_重い処理と軽くする方法.md", 7)
run_block("08_重い処理と軽くする方法.md", 8)
set_state("Stop")
run_block("08_重い処理と軽くする方法.md", 9, updates_after=200)
run_block("08_重い処理と軽くする方法.md", 10)
run_block("08_重い処理と軽くする方法.md", 9, updates_after=200)
set_state("Stop"); restore_defaults()
run_block("08_重い処理と軽くする方法.md", 12)
set_state("Play", settle_updates=20)
run_block("08_重い処理と軽くする方法.md", 11)
set_state("Stop"); restore_defaults()
close()
