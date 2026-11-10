# run_note_blocks_09_10.py - 09_supported_backends.md と 10_lab_parts.md のコードブロックを、Script Editor と同じ実行経路（execute_string）で順に実行して確かめる
import os
import re

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())
import carb.eventdispatcher

NOTE_DIR = os.path.join(LAB_ROOT, "05_観測Snippet")
texts = {"stdout": [], "stderr": []}
dispatcher = carb.eventdispatcher.get_eventdispatcher()
subscriptions = [
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDOUT_IMMEDIATE, on_event=lambda event: texts["stdout"].append(event.get("text")), observer_name="lab stdout"),
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDERR_IMMEDIATE, on_event=lambda event: texts["stderr"].append(event.get("text")), observer_name="lab stderr"),
]


def note_blocks(note_name):
    with open(os.path.join(NOTE_DIR, note_name), encoding="utf-8") as note_file:
        return re.findall(r"```python\n(.*?)\n```", note_file.read(), re.S)


def run_block(note_name, index, updates_after=1):
    label = f"{note_name}[{index}]"
    texts["stdout"].clear(); texts["stderr"].clear()
    kit_app.get_python_scripting().execute_string(note_blocks(note_name)[index], label, False)
    run_updates(updates_after)
    emit("block", label=label, stdout="".join(texts["stdout"])[-4000:], stderr="".join(texts["stderr"])[-3000:], ok=not texts["stderr"])
    print(("OK  " if not texts["stderr"] else "ERR ") + label, flush=True)


phase("N09")
omni.usd.get_context().new_stage(); run_updates(3)
run_block("09_supported_backends.md", 0)
run_block("09_supported_backends.md", 1)

phase("N10_play")
omni.usd.get_context().new_stage(); run_updates(3)
UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World")
run_block("10_lab_parts.md", 0)
run_block("10_lab_parts.md", 1, updates_after=40)

phase("N10_stop_overlap")
omni.usd.get_context().new_stage(); run_updates(3)
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
run_block("10_lab_parts.md", 2, updates_after=3)
set_state("Stop")
close()
