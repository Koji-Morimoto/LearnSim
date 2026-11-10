# run_kinematic_overlap_fix.py - キネマティックな剛体を新しい位置に置いたまま重なりを調べる2つの方法（set 直後にクエリ／キネマティックの目標を設定して step）を確かめる
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


import warp as wp
for method in ("A: set 直後にクエリ（step しない）", "B: set_kinematic_targets してから step"):
    phase(method)
    omni.usd.get_context().new_stage(); run_updates(3)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    SimulationManager.set_physics_dt(1.0 / 60.0)
    paths = lab.add_cube_grid(4, root_path="/World/K", kinematic=True)
    run_updates(2)
    lab.start_physics_without_play()
    cubes = RigidPrim(paths)
    positions = cubes.get_world_poses()[0].numpy().copy()
    original_c1 = positions[1].copy()
    positions[1] = positions[0] + np.array([0.05, 0.0, 0.0], dtype=np.float32)
    if method.startswith("A"):
        cubes.set_world_poses(positions=positions)
        emit("probe", step="set 直後", at_c0=hits_at(positions[0]), at_old_c1=hits_at(original_c1))
    else:
        view = cubes._physics_rigid_body_view
        transforms = view.get_transforms()
        data = transforms.numpy().copy() if hasattr(transforms, "numpy") else np.array(transforms)
        data[:, :3] = positions
        indices = wp.array(np.arange(len(paths), dtype=np.int32), device=str(SimulationManager.get_physics_sim_device()))
        view.set_kinematic_targets(wp.array(data, dtype=wp.float32, device=str(SimulationManager.get_physics_sim_device())), indices)
        emit("probe", step="set_kinematic_targets 直後（step 前）", tensor_c1=cubes.get_world_poses()[0].numpy()[1].tolist(), at_c0=hits_at(positions[0]))
    for step_index in range(1, 3):
        SimulationManager.step(steps=1)
        emit("probe", step=f"step {step_index} 回目の後", tensor_c1=cubes.get_world_poses()[0].numpy()[1].tolist(), at_c0=hits_at(positions[0]), at_old_c1=hits_at(original_c1))
    # 1000 個で時間を測る
    omni.usd.get_context().new_stage(); run_updates(3)
    stage = omni.usd.get_context().get_stage(); UsdGeom.Xform.Define(stage, "/World")
    SimulationManager.set_physics_dt(1.0 / 60.0)
    many = lab.add_cube_grid(1000, root_path="/World/M", kinematic=True)
    run_updates(2); lab.start_physics_without_play()
    rigid_all = RigidPrim(many)
    pose = rigid_all.get_world_poses()[0].numpy().copy()
    if method.startswith("A"):
        def move():
            pose[:, 0] += 0.001
            rigid_all.set_world_poses(positions=pose)
    else:
        view_all = rigid_all._physics_rigid_body_view
        device = str(SimulationManager.get_physics_sim_device())
        full = view_all.get_transforms().numpy().copy()
        all_indices = wp.array(np.arange(len(many), dtype=np.int32), device=device)
        def move():
            full[:, 0] += 0.001
            view_all.set_kinematic_targets(wp.array(full, dtype=wp.float32, device=device), all_indices)
            SimulationManager.step(steps=1)
    emit("timing", method=method, median_us=lab.time_per_call_us(move, repeat_count=50))
close()

