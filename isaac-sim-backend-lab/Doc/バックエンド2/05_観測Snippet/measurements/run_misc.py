# run_misc.py - 各ノートの未確定事項のうち、Snippet 以外で確かめるもの
import asyncio
import os
import sys
import threading

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.impl.backend as backend_impl
import numpy as np
import omni.physx
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.core.simulation_manager import SimulationEvent
from pxr import Usd, UsdUtils

physx_interface = omni.physx.get_physx_interface()


def physx_z(path):
    transform = physx_interface.get_rigidbody_transformation(path)
    return float(transform["position"][2]) if transform and transform.get("ret_val") else None


# ---------------------------------------------------------------- C1 view の getter が返す配列を書き換えても物理は変わらないか
phase("C1_getter_copy")
stage = new_stage()
add_cube(stage, position=(0.0, 0.0, 5.0))
set_state("Play", settle_updates=5)
cube = RigidPrim("/World/Cube")
positions, _ = cube.get_world_poses()
emit("getter", step="read", z=float(positions.numpy()[0][2]), device=str(positions.device), physx=physx_z("/World/Cube"))
positions_np = positions.numpy()
positions_np[0][2] = 100.0  # numpy は CPU の warp 配列とメモリを共有するので、warp 配列そのものを書き換える
emit("getter", step="after_write", z_in_array=float(positions.numpy()[0][2]), physx=physx_z("/World/Cube"))
positions_again, _ = cube.get_world_poses()
emit("getter", step="read_again", z=float(positions_again.numpy()[0][2]), physx=physx_z("/World/Cube"), same_object=positions_again is positions)
run_updates(1)
positions_after_update, _ = cube.get_world_poses()
emit("getter", step="after_update", z=float(positions_after_update.numpy()[0][2]), physx=physx_z("/World/Cube"))
# omni.physics.tensors の view を直接使う
raw_view = cube._physics_rigid_body_view
transforms = raw_view.get_transforms()
emit("getter", step="raw_view_read", type=str(type(transforms)), z=float(transforms.numpy()[0][2]) if hasattr(transforms, "numpy") else None)
try:
    transforms_np = transforms.numpy()
    transforms_np[0][2] = 200.0
except Exception as error:
    emit("getter", step="raw_view_write_error", error=f"{type(error).__name__}: {error}")
emit("getter", step="raw_view_after_write", reread=float(raw_view.get_transforms().numpy()[0][2]), physx=physx_z("/World/Cube"))
set_state("Stop")

# ---------------------------------------------------------------- C2 use_backend とコールバック
phase("C2_use_backend_callbacks")
stage = new_stage()
add_cube(stage)
records = []
main_thread_id = threading.get_ident()


def on_post_step(dt, context):
    records.append(
        {
            "context_backend": getattr(backend_impl._context, "backend", None),
            "selected": backend_utils.get_current_backend(["tensor", "usd"]),
            "same_thread": threading.get_ident() == main_thread_id,
        }
    )


with backend_utils.use_backend("usd"):
    callback_id = SimulationManager.register_callback(on_post_step, event=SimulationEvent.PHYSICS_POST_STEP)
set_state("Play", settle_updates=3)
emit("callback", case="with の中で登録し、with の外の更新で呼ばれた", records=records[:3])
records.clear()
with backend_utils.use_backend("usd"):
    SimulationManager.step(steps=2)  # with の中で物理ステップを直接進めると、コールバックも with の中で呼ばれる
emit("callback", case="with の中で SimulationManager.step を呼んだ", records=records[:3])
records.clear()
with backend_utils.use_backend("usd"):
    run_updates(2)  # with の中でメインループを更新した場合
emit("callback", case="with の中で更新した（Script Editor では起きない形）", records=records[:3])
SimulationManager.deregister_callback(callback_id)
set_state("Stop")

# ---------------------------------------------------------------- C3 /app/useFabricSceneDelegate を起動後に変えた場合
phase("C3_fsd_cached")
emit("fsd", module_value=backend_impl._fsd_enabled, setting=settings.get("/app/useFabricSceneDelegate"))
settings.set_bool("/app/useFabricSceneDelegate", False)
try:
    with backend_utils.use_backend("usdrt"):
        pass
    emit("fsd", result="設定を false にしても use_backend('usdrt') は例外にならなかった", module_value=backend_impl._fsd_enabled)
except Exception as error:
    emit("fsd", result=f"{type(error).__name__}: {error}")
settings.set_bool("/app/useFabricSceneDelegate", True)

# ---------------------------------------------------------------- C4 simstate
phase("C4_simstate")
try:
    import isaacsim.mega.sim_state.client  # noqa: F401

    emit("simstate", client="import できた")
except Exception as error:
    emit("simstate", client=f"import できない: {type(error).__name__}: {error}")
stage = new_stage()
add_articulation(stage)
set_state("Play", settle_updates=5)
robot = Articulation("/World/robot")
for mode, target_value in (("disabled", 0.3), ("mirror", 0.5), ("exclusive", 0.8)):
    settings.set_string("/isaacsim/articulation/simStateMode", mode)
    with captured_stdout(f"simstate {mode}"):
        robot.set_dof_position_targets([[target_value]])
    run_updates(30)
    emit(
        "simstate",
        mode=mode,
        written_target=target_value,
        read_target=robot.get_dof_position_targets().numpy().tolist(),
        dof_position=robot.get_dof_positions().numpy().tolist(),
    )
settings.set_string("/isaacsim/articulation/simStateMode", "disabled")
with backend_utils.use_backend("simstate"):
    robot.set_dof_position_targets([[1.0]])
run_updates(30)
emit("simstate", mode="use_backend('simstate')", written_target=1.0, read_target=robot.get_dof_position_targets().numpy().tolist())
set_state("Stop")

# ---------------------------------------------------------------- C5 コードから Play / Pause / Stop する
phase("C5_timeline_from_code")
stage = new_stage()
add_cube(stage)
timeline.play()
emit("timeline_code", step="play() の直後", playing=timeline.is_playing(), simulating=SimulationManager.is_simulating(), steps=SimulationManager.get_num_physics_steps(), view=SimulationManager.get_physics_simulation_view() is not None)
timeline.commit()
emit("timeline_code", step="commit() の直後", playing=timeline.is_playing(), simulating=SimulationManager.is_simulating(), steps=SimulationManager.get_num_physics_steps(), view=SimulationManager.get_physics_simulation_view() is not None)
run_updates(1)
emit("timeline_code", step="1回更新した後", playing=timeline.is_playing(), steps=SimulationManager.get_num_physics_steps())
timeline.pause()
emit("timeline_code", step="pause() の直後", playing=timeline.is_playing(), stopped=timeline.is_stopped())
run_updates(1)
emit("timeline_code", step="pause() 後に1回更新", playing=timeline.is_playing(), stopped=timeline.is_stopped())
timeline.stop()
run_updates(1)
emit("timeline_code", step="stop() 後に1回更新", stopped=timeline.is_stopped(), steps=SimulationManager.get_num_physics_steps())

sequence_log = []


async def run_sequence():
    app = omni.kit.app.get_app()
    timeline.play()
    for _ in range(30):
        await app.next_update_async()
    sequence_log.append(("after 30 updates of Play", state_name(), SimulationManager.get_num_physics_steps()))
    timeline.pause()
    await app.next_update_async()
    sequence_log.append(("after pause", state_name(), SimulationManager.get_num_physics_steps()))
    for _ in range(10):
        await app.next_update_async()
    sequence_log.append(("after 10 updates of Pause", state_name(), SimulationManager.get_num_physics_steps()))
    timeline.stop()
    await app.next_update_async()
    sequence_log.append(("after stop", state_name(), SimulationManager.get_num_physics_steps()))


task = asyncio.ensure_future(run_sequence())
for _ in range(100):
    simapp.update()
    if task.done():
        break
emit("timeline_code", step="async の手順", log=sequence_log, done=task.done(), error=str(task.exception()) if task.done() and task.exception() else None)

# ---------------------------------------------------------------- C6 carb.log_warn は Python の標準出力を通るか
phase("C6_log_warn")
with captured_stdout("carb.log_warn") as buffer:
    carb.log_warn("lab-test: carb.log_warn から出した文字列")
    print("lab-test: print から出した文字列")
emit("log_warn", captured=buffer.getvalue())

# ---------------------------------------------------------------- C7 1つのプロセスで2つのステージを同時に物理シミュレーションできるか
phase("C7_multi_stage")
stage = new_stage()
add_cube(stage, position=(0.0, 0.0, 10.0))
set_state("Play", settle_updates=5)
main_before = physx_z("/World/Cube")
second_stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(second_stage, UsdGeom.Tokens.z)
second_scene = UsdPhysics.Scene.Define(second_stage, "/physicsScene")
second_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
second_scene.CreateGravityMagnitudeAttr(9.81)
UsdGeom.Xform.Define(second_stage, "/World")
add_cube(second_stage, "/World/Other", position=(0.0, 0.0, 10.0))
second_stage_id = UsdUtils.StageCache.Get().Insert(second_stage).ToLongInt()
simulation_interface = omni.physx.get_physx_simulation_interface()
attached = simulation_interface.attach_stage(second_stage_id)
run_updates(10)
emit(
    "multi_stage",
    attached=attached,
    main_cube_before=main_before,
    main_cube_after=physx_z("/World/Cube"),
    main_cube_usd=float(UsdGeom.Xformable(stage.GetPrimAtPath("/World/Cube")).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]),
    second_cube_physx=physx_z("/World/Other"),
)
for _ in range(5):
    simulation_interface.simulate(1.0 / 60.0, 0.0)
    simulation_interface.fetch_results()
emit(
    "multi_stage",
    step="2つ目のステージを simulate で5回進めた後",
    second_cube_usd=float(UsdGeom.Xformable(second_stage.GetPrimAtPath("/World/Other")).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]),
    main_cube_physx=physx_z("/World/Cube"),
)
simulation_interface.detach_stage()
run_updates(5)
emit("multi_stage", step="detach 後", main_cube_physx=physx_z("/World/Cube"), playing=timeline.is_playing())
set_state("Stop")

close()
