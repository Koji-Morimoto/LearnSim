# run_kinematic_timing.py - キネマティックな剛体 1000 個を Stop のまま動かす2つの方法の所要時間を、lab_parts の部品で測る
import os
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())
exec(open(os.path.join(LAB_ROOT, "snippets", "lab_parts.py"), encoding="utf-8").read())
import builtins
import omni.physx
from isaacsim.core.experimental.prims import RigidPrim

lab = builtins.lab
omni.usd.get_context().new_stage(); run_updates(3)
UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World")
SimulationManager.set_physics_dt(1.0 / 60.0)
paths = lab.add_cube_grid(1000, root_path="/World/M", kinematic=True)
run_updates(2)
lab.start_physics_without_play()
cubes = RigidPrim(paths)
positions = cubes.get_world_poses()[0].numpy().copy()

def move_with_set_world_poses():
    positions[:, 0] += 0.001
    cubes.set_world_poses(positions=positions)

def move_with_kinematic_targets_and_step():
    positions[:, 0] += 0.001
    lab.set_kinematic_targets(cubes, positions=positions)
    SimulationManager.step(steps=1)

def step_only():
    SimulationManager.step(steps=1)

query = omni.physx.get_physx_scene_query_interface()
def overlap_once():
    query.overlap_box(carb.Float3(0.06, 0.06, 0.06), carb.Float3(*[float(v) for v in positions[0]]), carb.Float4(0, 0, 0, 1), lambda hit: True, False)

for label, action in [("set_world_poses だけ", move_with_set_world_poses), ("set_kinematic_targets + step(1)", move_with_kinematic_targets_and_step), ("step(1) だけ", step_only), ("overlap_box 1回", overlap_once)]:
    emit("timing", label=label, median_us=lab.time_per_call_us(action, repeat_count=50), device=str(SimulationManager.get_physics_sim_device()))
close()
