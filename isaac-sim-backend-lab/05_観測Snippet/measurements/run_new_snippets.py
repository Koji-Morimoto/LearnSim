# run_new_snippets.py - supported_backends.py と lab_parts.py を Script Editor と同じ実行経路で実行して確かめる
import os
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())
import carb.eventdispatcher

texts = {"stdout": [], "stderr": []}
dispatcher = carb.eventdispatcher.get_eventdispatcher()
subscriptions = [
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDOUT_IMMEDIATE, on_event=lambda event: texts["stdout"].append(event.get("text")), observer_name="lab stdout"),
    dispatcher.observe_event(event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDERR_IMMEDIATE, on_event=lambda event: texts["stderr"].append(event.get("text")), observer_name="lab stderr"),
]


def run_code(label, code, updates_after=1):
    texts["stdout"].clear(); texts["stderr"].clear()
    kit_app.get_python_scripting().execute_string(code, label, False)
    run_updates(updates_after)
    emit("block", label=label, stdout="".join(texts["stdout"]), stderr="".join(texts["stderr"]))
    print(("OK  " if not texts["stderr"] else "ERR ") + label, flush=True)


def snippet(name):
    return open(os.path.join(LAB_ROOT, "snippets", name + ".py"), encoding="utf-8").read()


omni.usd.get_context().new_stage(); run_updates(3)
UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World")
run_code("supported_backends", snippet("supported_backends"))
run_code("supported one", "show_supported_backends(__import__('isaacsim.core.experimental.prims', fromlist=['RigidPrim']).RigidPrim, 'get_world_poses')")
run_code("lab_parts", snippet("lab_parts"))
run_code("parts scene", "lab.add_minimal_articulation(); paths = lab.add_cube_grid(9, kinematic=False); print(len(paths), lab.state_name(), lab.physics_output_summary())")
run_code("parts timing", "print('us', round(lab.time_per_call_us(lambda: None), 2))")
run_code("parts async", """
async def steps():
    await lab.switch_state('Play', settle_updates=5)
    print('state', lab.state_name())
    from isaacsim.core.experimental.prims import RigidPrim, Articulation
    rigid = RigidPrim(paths)
    print('tensor z', lab.read_with_backend(rigid, 'tensor')[0].numpy()[0][2])
    robot = Articulation('/World/robot')
    print('targets', robot.get_dof_position_targets().numpy().tolist())
    await lab.switch_state('Stop', settle_updates=2)
    print('stop targets', robot.get_dof_position_targets().numpy().tolist())
    print('state', lab.state_name())
lab.run_async(steps())
""", updates_after=20)
run_code("parts output", "lab.set_physics_output('none'); print(lab.physics_output_summary()); lab.set_physics_output('usd'); print(lab.physics_output_summary())")
run_code("parts stop physics", """
lab.start_physics_without_play()
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
r = RigidPrim(paths)
z0 = lab.read_with_backend(r, 'tensor')[0].numpy()[0][2]
SimulationManager.step(steps=10)
print('stop physics', lab.state_name(), z0, lab.read_with_backend(r, 'tensor')[0].numpy()[0][2])
""")
run_code("parts find", "lab.find_setting_usages('/app/useFabricSceneDelegate', max_hits=12)")
run_code("parts find2", "lab.find_setting_usages('/exts/omni.kit.window.script_editor/maxLogLines', max_hits=8)")
run_code("parts snippet", "lab.run_snippet('strict_backend', {'RUN_DEMO: typing.Final[bool] = True': 'RUN_DEMO: typing.Final[bool] = False'})")
set_state("Stop")
close()
