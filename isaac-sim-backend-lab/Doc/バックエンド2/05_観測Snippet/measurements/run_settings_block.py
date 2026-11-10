# run_settings_block.py - 04_設定/01_設定一覧.md のコードを Script Editor と同じ実行経路で3つの状態で実行する
import os
import re
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())
import carb.eventdispatcher

texts = {"stdout": [], "stderr": []}
dispatcher = carb.eventdispatcher.get_eventdispatcher()
subscriptions = [
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDOUT_IMMEDIATE, on_event=lambda event: texts["stdout"].append(event.get("text")), observer_name="lab stdout"),
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDERR_IMMEDIATE, on_event=lambda event: texts["stderr"].append(event.get("text")), observer_name="lab stderr"),
]
with open(os.path.join(LAB_ROOT, "04_設定", "01_設定一覧.md"), encoding="utf-8") as note_file:
    code = re.findall(r"```python\n(.*?)\n```", note_file.read(), re.S)[0]
omni.usd.get_context().new_stage(); run_updates(3)
texts["stdout"].clear(); texts["stderr"].clear()
kit_app.get_python_scripting().execute_string(code, "settings no scene", False); run_updates(1)
emit("settings_block", state="Stop(Physics Sceneなし)", stdout="".join(texts["stdout"]), stderr="".join(texts["stderr"]), prims=[str(p.GetPath()) for p in omni.usd.get_context().get_stage().Traverse()])
stage = new_stage(); add_cube(stage)
for target in ("Stop", "Play", "Pause"):
    set_state(target, settle_updates=15)
    texts["stdout"].clear(); texts["stderr"].clear()
    kit_app.get_python_scripting().execute_string(code, f"settings {target}", False); run_updates(1)
    emit("settings_block", state=target, stdout="".join(texts["stdout"]), stderr="".join(texts["stderr"]))
set_state("Stop")
close()
