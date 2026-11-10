# run_stop_physics.py - Stop のまま物理を初期化して tensor の view と手動ステップを使えるか、干渉の検出、GPU、設定の途中変更を確かめる
import os
import statistics
import sys
import time

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import carb.eventdispatcher
import isaacsim.core.experimental.utils.backend as backend_utils
import numpy as np
import omni.physx
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from pxr import Gf, PhysxSchema, UsdPhysics

query = omni.physx.get_physx_scene_query_interface()


def overlap_hits(center, half_extent=0.06):
    hits = []
    def report(hit):
        hits.append(hit.rigid_body)
        return True
    query.overlap_box(carb.Float3(half_extent, half_extent, half_extent), carb.Float3(*center), carb.Float4(0, 0, 0, 1), report, False)
    return hits


# ---------------------------------------------------------------- S1 Stop のまま initialize_physics
phase("S1_stop_initialize")
stage = new_stage()
add_cube(stage, "/World/A", position=(0.0, 0.0, 1.0))
add_cube(stage, "/World/B", position=(0.05, 0.0, 1.0))  # A と重なる位置
add_cube(stage, "/World/C", position=(1.0, 0.0, 1.0))  # 離れた位置
run_updates(2)
emit("stop_init", step="initialize の前", overlap_A=overlap_hits((0.0, 0.0, 1.0)), view=SimulationManager.get_physics_simulation_view() is not None)
SimulationManager.initialize_physics()
emit("stop_init", step="initialize の直後", stopped=timeline.is_stopped(), view=SimulationManager.get_physics_simulation_view() is not None, steps=SimulationManager.get_num_physics_steps(), overlap_A=overlap_hits((0.0, 0.0, 1.0)))
rigid = RigidPrim(["/World/A", "/World/B", "/World/C"])
try:
    with backend_utils.use_backend("tensor", raise_on_unsupported=True, raise_on_fallback=True):
        z0 = rigid.get_world_poses()[0].numpy()[:, 2].tolist()
    emit("stop_init", step="tensor で読めた", z=z0)
except Exception as error:
    emit("stop_init", step="tensor で読めない", error=f"{type(error).__name__}: {error}")
run_updates(10)
emit("stop_init", step="Stop のまま10回更新した後", stopped=timeline.is_stopped(), steps=SimulationManager.get_num_physics_steps(), z=rigid.get_world_poses()[0].numpy()[:, 2].tolist())
SimulationManager.step(steps=10)
emit("stop_init", step="step(10) の後", stopped=timeline.is_stopped(), steps=SimulationManager.get_num_physics_steps(), z=rigid.get_world_poses()[0].numpy()[:, 2].tolist(), usd_A=float(UsdGeom.Xformable(stage.GetPrimAtPath("/World/A")).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]))
targets = np.array([[0.0, 0.0, 3.0], [2.0, 0.0, 3.0], [4.0, 0.0, 3.0]], dtype=np.float32)
rigid.set_world_poses(positions=targets)
emit("stop_init", step="tensor で set した直後", z=rigid.get_world_poses()[0].numpy()[:, 2].tolist(), overlap_at_new_A=overlap_hits((0.0, 0.0, 3.0)))
SimulationManager.step(steps=1)
emit("stop_init", step="set の後に step(1)", z=rigid.get_world_poses()[0].numpy()[:, 2].tolist(), overlap_at_new_A=overlap_hits((0.0, 0.0, 3.0)))
# Play / Stop が正常に動くか
set_state("Play", settle_updates=5)
emit("stop_init", step="その後 Play", playing=timeline.is_playing(), steps=SimulationManager.get_num_physics_steps(), view=SimulationManager.get_physics_simulation_view() is not None)
set_state("Stop")
emit("stop_init", step="その後 Stop", stopped=timeline.is_stopped(), view=SimulationManager.get_physics_simulation_view() is not None, steps=SimulationManager.get_num_physics_steps())
set_state("Play", settle_updates=5)
emit("stop_init", step="もう一度 Play", playing=timeline.is_playing(), steps=SimulationManager.get_num_physics_steps(), view=SimulationManager.get_physics_simulation_view() is not None)
set_state("Stop")

# ---------------------------------------------------------------- S2 重力なし・干渉だけの用途：Stop のまま 1000 個を tensor で動かして干渉を数える
phase("S2_interference_only")
stage = new_stage()
UsdPhysics.Scene.Get(stage, "/physicsScene").GetGravityMagnitudeAttr().Set(0.0)
paths = []
for index in range(1000):
    path = f"/World/Cubes/c_{index}"
    cube = add_cube(stage, path, position=(0.3 * (index % 32), 0.3 * (index // 32), 1.0))
    UsdPhysics.RigidBodyAPI(cube.GetPrim()).CreateKinematicEnabledAttr(True)  # キネマティック（力で動かず、指定した姿勢に置くだけ）
    paths.append(path)
run_updates(2)
SimulationManager.initialize_physics()
rigid_all = RigidPrim(paths)
positions = rigid_all.get_world_poses()[0].numpy().copy()
def move_and_step():
    positions[:, 0] += 0.001
    rigid_all.set_world_poses(positions=positions)
    SimulationManager.step(steps=1)
samples = []
for _ in range(50):
    start = time.perf_counter(); move_and_step(); samples.append(time.perf_counter() - start)
emit("interference", step="1000個を tensor で置き直して step(1)", median_ms=statistics.median(samples) * 1000, device=str(SimulationManager.get_physics_sim_device()))
query_samples = []
for _ in range(50):
    start = time.perf_counter(); hits = overlap_hits(tuple(positions[0])); query_samples.append(time.perf_counter() - start)
emit("interference", step="overlap_box 1回", median_us=statistics.median(query_samples) * 1e6, hits=hits)
set_state("Stop")

# ---------------------------------------------------------------- S3 GPU の構成で Stop のまま initialize_physics
phase("S3_gpu_stop")
settings.set_bool("/physics/suppressReadback", True)
settings.set_int("/physics/cudaDevice", 0)
stage = new_stage(gpu=True)
add_cube(stage, "/World/A", position=(0.0, 0.0, 1.0))
run_updates(2)
emit("gpu_stop", step="initialize の前", device=str(SimulationManager.get_physics_sim_device()))
SimulationManager.initialize_physics()
rigid = RigidPrim("/World/A")
positions = rigid.get_world_poses()[0]
SimulationManager.step(steps=10)
emit("gpu_stop", step="initialize と step(10) の後", stopped=timeline.is_stopped(), device=str(SimulationManager.get_physics_sim_device()), array_device=str(positions.device), z=float(rigid.get_world_poses()[0].numpy()[0][2]))
set_state("Play", settle_updates=3); set_state("Stop")
settings.set_bool("/physics/suppressReadback", False)

# ---------------------------------------------------------------- S4 Play 中に /physics/updateToUsd を変えると、その場で効くか
phase("S4_live_setting")
stage = new_stage()
add_cube(stage, "/World/A", position=(0.0, 0.0, 5.0))
set_state("Play", settle_updates=10)
usd_z = lambda: float(UsdGeom.Xformable(stage.GetPrimAtPath("/World/A")).ComputeLocalToWorldTransform(0).ExtractTranslation()[2])
before = usd_z()
settings.set_bool("/physics/updateToUsd", False)
run_updates(10)
frozen = usd_z()
run_updates(10)
emit("live_setting", key="/physics/updateToUsd", before=before, after_10=frozen, after_20=usd_z(), physx=float(omni.physx.get_physx_interface().get_rigidbody_transformation("/World/A")["position"][2]))
settings.set_bool("/physics/updateToUsd", True)
run_updates(5)
emit("live_setting", key="/physics/updateToUsd を true に戻した後", usd=usd_z())
set_state("Stop")
close()
