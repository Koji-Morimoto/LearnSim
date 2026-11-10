# run_kinematic_overlap_debug.py - キネマティックな剛体を tensor で置き直したとき、シーンクエリに新しい位置がいつ反映されるかを調べる
import os
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())
exec(open(os.path.join(LAB_ROOT, "snippets", "lab_parts.py"), encoding="utf-8").read())
import builtins
import numpy as np
import omni.physx
from isaacsim.core.experimental.prims import RigidPrim

lab = builtins.lab
query = omni.physx.get_physx_scene_query_interface()

def hits_at(center, half=0.06):
    found = []
    query.overlap_box(carb.Float3(half, half, half), carb.Float3(*[float(v) for v in center]), carb.Float4(0, 0, 0, 1), lambda hit: found.append(hit.rigid_body) or True, False)
    return found

for kinematic in (True, False):
    phase(f"kinematic={kinematic}")
    omni.usd.get_context().new_stage(); run_updates(3)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    SimulationManager.set_physics_dt(1.0 / 60.0)
    UsdPhysics.Scene.Get(stage, "/PhysicsScene").GetGravityMagnitudeAttr().Set(0.0) if UsdPhysics.Scene.Get(stage, "/PhysicsScene") else None
    paths = lab.add_cube_grid(4, root_path="/World/K", kinematic=kinematic)
    run_updates(2)
    lab.start_physics_without_play()
    cubes = RigidPrim(paths)
    positions = cubes.get_world_poses()[0].numpy().copy()
    original_c1 = positions[1].copy()
    emit("probe", step="準備直後", pos=positions.tolist(), at_c0=hits_at(positions[0]), at_c1=hits_at(original_c1))
    positions[1] = positions[0] + np.array([0.05, 0.0, 0.0], dtype=np.float32)
    cubes.set_world_poses(positions=positions)
    emit("probe", step="set 直後", tensor_c1=cubes.get_world_poses()[0].numpy()[1].tolist(), at_c0=hits_at(positions[0]), at_old_c1=hits_at(original_c1))
    for step_index in range(1, 4):
        SimulationManager.step(steps=1)
        emit("probe", step=f"step {step_index} 回目の後", tensor_c1=cubes.get_world_poses()[0].numpy()[1].tolist(), at_c0=hits_at(positions[0]), at_old_c1=hits_at(original_c1))
    run_updates(3)
    emit("probe", step="メインループ3回の後", at_c0=hits_at(positions[0]), at_old_c1=hits_at(original_c1))
close()
