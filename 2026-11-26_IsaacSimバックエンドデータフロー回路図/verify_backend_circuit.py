r"""Isaac Sim 6.2.0 バックエンドデータフロー回路図 実測検証スクリプト.

AGENTS.md の SimulationApp 起動ルールに従い tools/myext/sim_app_launcher.py 経由で起動し、
回路図の各経路（USD / Fabric / PhysX / Tensor API / Notice / Render Gate / TimeSample）を
実機で測定して JSON に出力する。

実行:
    tools\myext\run_sim.bat verify_backend_circuit.py
"""

from __future__ import annotations

import importlib
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, r"C:/isaacsim-6.2.0/tools/myext")
from sim_app_launcher import create_simulation_app

simulation_app = create_simulation_app({"headless": True})

import carb
import carb.settings
import omni.kit.app
import omni.physics.tensors
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdPhysics
from usdrt import Rt as UsdRtGeom
from usdrt import Sdf as UsdRtSdf
from usdrt import Usd as UsdRt
from usdrt import hierarchy as usdrt_hierarchy

from isaacsim.core.api import SimulationContext
from isaacsim.core.simulation_manager import SimulationManager


class NoticeCounter:
    """pxr.Usd.Notice.ObjectsChanged の発火回数を記録するリスナー."""

    def __init__(self, stage: Usd.Stage, path_prefix: str = "/World/Cube") -> None:
        self.path_prefix = path_prefix
        self.total_events = 0
        self.target_events = 0
        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_notice, stage)

    def _on_notice(self, notice: Any, sender: Any) -> None:
        self.total_events += 1
        for p in list(notice.GetChangedInfoOnlyPaths()) + list(notice.GetResyncedPaths()):
            if str(p).startswith(self.path_prefix):
                self.target_events += 1
                break

    def reset(self) -> None:
        self.total_events = 0
        self.target_events = 0

    def revoke(self) -> None:
        if self._listener is not None:
            self._listener.Revoke()
            self._listener = None


def pump(n: int = 3) -> None:
    for _ in range(n):
        simulation_app.update()


def setup_scene(enable_gpu: bool = False) -> tuple[Usd.Stage, UsdRt.Stage, Any, str]:
    omni.timeline.get_timeline_interface().stop()
    pump(2)
    omni.usd.get_context().new_stage()
    pump(2)

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    if enable_gpu:
        physx_scene.CreateBroadphaseTypeAttr("GPU")
        physx_scene.CreateEnableGPUDynamicsAttr(True)
    else:
        physx_scene.CreateBroadphaseTypeAttr("MBP")
        physx_scene.CreateEnableGPUDynamicsAttr(False)

    cube_path = "/World/Cube"
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(0.5)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 5.0))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    pump(3)
    stage_id = omni.usd.get_context().get_stage_id()
    usdrt_stage = UsdRt.Stage.Attach(stage_id)
    usdrt_stage.SynchronizeToFabric()
    fabric_id = usdrt_stage.GetFabricId()
    stage_id_obj = usdrt_stage.GetStageIdAsStageId()
    hier = usdrt_hierarchy.IFabricHierarchy().get_fabric_hierarchy(fabric_id, stage_id_obj)
    return stage, usdrt_stage, hier, cube_path


def get_usd_z(stage: Usd.Stage, path: str) -> float:
    prim = stage.GetPrimAtPath(path)
    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return round(float(mat.ExtractTranslation()[2]), 5)


def get_fabric_z_raw(hier: Any, path: str) -> float:
    """SynchronizeToFabric を暗黙誘発せずに Fabric 階層から直接 worldMatrix を読む."""
    hier.update_world_xforms()
    mat = hier.get_world_xform(UsdRtSdf.Path(path))
    return round(float(mat.ExtractTranslation()[2]), 5)


def get_fabric_z_attr(usdrt_stage: UsdRt.Stage, path: str) -> float:
    prim = usdrt_stage.GetPrimAtPath(path)
    xf = UsdRtGeom.Xformable(prim)
    attr = xf.GetFabricHierarchyWorldMatrixAttr()
    mat = attr.Get()
    return round(float(mat.ExtractTranslation()[2]), 5)


def get_tensor_z(path: str, use_gpu: bool = False) -> tuple[float | None, str | None]:
    try:
        sim_view = SimulationManager.get_physics_sim_view()
        if sim_view is None:
            backend = "torch" if use_gpu else "numpy"
            sim_view = omni.physics.tensors.create_simulation_view(backend)
        rb_view = sim_view.create_rigid_body_view(path)
        tf = rb_view.get_transforms()
        if hasattr(tf, "cpu"):
            tf = tf.cpu().numpy()
        elif hasattr(tf, "numpy"):
            tf = tf.numpy()
        return round(float(tf[0][2]), 5), None
    except Exception as e:
        return None, repr(e)


def main() -> None:
    settings = carb.settings.get_settings()
    timeline = omni.timeline.get_timeline_interface()
    results: dict[str, Any] = {
        "env": {
            "app_name": settings.get("/app/name"),
            "app_version": settings.get("/app/version"),
            "useFabricSceneDelegate": bool(settings.get_as_bool("/app/useFabricSceneDelegate")),
            "resetOnStop_default": bool(settings.get_as_bool("/physics/resetOnStop")),
        },
        "tests": {},
    }

    # ------------------------------------------------------------------
    # T1: 既定モード (PLAY, updateToUsd=True, omni.physx.fabric OFF, CPU)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(False)
    settings.set_bool("/physics/updateToUsd", True)
    settings.set_bool("/physics/suppressReadback", False)
    settings.set_bool("/physics/resetOnStop", True)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)

    nc = NoticeCounter(stage, cube_path)
    z_usd_0 = get_usd_z(stage, cube_path)
    z_fab_0 = get_fabric_z_raw(hier, cube_path)

    timeline.play()
    nc.reset()
    pump(25)
    t1_notice_play = nc.target_events
    z_usd_1 = get_usd_z(stage, cube_path)
    z_fab_1 = get_fabric_z_raw(hier, cube_path)
    z_ten_1, z_ten_1_err = get_tensor_z(cube_path, use_gpu=False)
    timeline.stop()
    pump(3)
    nc.revoke()

    results["tests"]["T1_default_usd_play"] = {
        "settings": {"updateToUsd": True, "omni_physx_fabric": False, "suppressReadback": False},
        "usd_z_before": z_usd_0,
        "usd_z_after": z_usd_1,
        "fabric_z_before": z_fab_0,
        "fabric_z_after": z_fab_1,
        "tensor_z_after": z_ten_1,
        "tensor_err": z_ten_1_err,
        "usd_notice_events_25_steps": t1_notice_play,
        "usd_moved": abs(z_usd_1 - z_usd_0) > 0.05,
        "fabric_moved": abs(z_fab_1 - z_fab_0) > 0.05,
    }

    # ------------------------------------------------------------------
    # T2: Fabric モード (SimulationManager.enable_fabric(True) -> updateToUsd=False)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(True)
    settings.set_bool("/physics/fabricEnabled", True)
    settings.set_bool("/physics/suppressReadback", False)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)

    nc = NoticeCounter(stage, cube_path)
    z_usd_0 = get_usd_z(stage, cube_path)
    z_fab_0 = get_fabric_z_raw(hier, cube_path)

    timeline.play()
    pump(2)
    nc.reset()
    pump(25)
    t2_notice_play = nc.target_events
    z_usd_1 = get_usd_z(stage, cube_path)
    z_fab_1 = get_fabric_z_raw(hier, cube_path)
    z_ten_1, z_ten_2_err = get_tensor_z(cube_path, use_gpu=False)

    # 手動 save_to_usd() ワンショット確認
    omni_physxfabric = importlib.import_module("omni.physxfabric")

    nc.reset()
    omni_physxfabric.get_physx_fabric_interface().save_to_usd()
    t2_notice_save = nc.target_events
    z_usd_after_save = get_usd_z(stage, cube_path)

    timeline.stop()
    pump(3)
    nc.revoke()

    results["tests"]["T2_fabric_mode_play"] = {
        "settings": {
            "updateToUsd": bool(settings.get_as_bool("/physics/updateToUsd")),
            "fabricEnabled": bool(settings.get_as_bool("/physics/fabricEnabled")),
            "omni_physx_fabric": SimulationManager.is_fabric_enabled(),
        },
        "usd_z_before": z_usd_0,
        "usd_z_after": z_usd_1,
        "fabric_z_before": z_fab_0,
        "fabric_z_after": z_fab_1,
        "tensor_z_after": z_ten_1,
        "usd_notice_events_25_steps": t2_notice_play,
        "usd_z_after_save_to_usd": z_usd_after_save,
        "usd_notice_events_on_save_to_usd": t2_notice_save,
        "usd_frozen_during_play": abs(z_usd_1 - z_usd_0) < 1e-4,
        "fabric_moved": abs(z_fab_1 - z_fab_0) > 0.05,
        "save_to_usd_synced": abs(z_usd_after_save - z_fab_1) < 0.05,
    }

    # ------------------------------------------------------------------
    # T3: 描画断線 (updateToUsd=False かつ fabricEnabled=False)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(False)
    settings.set_bool("/physics/updateToUsd", False)
    settings.set_bool("/physics/fabricEnabled", False)
    settings.set_bool("/physics/suppressReadback", False)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)

    z_usd_0 = get_usd_z(stage, cube_path)
    z_fab_0 = get_fabric_z_raw(hier, cube_path)
    timeline.play()
    pump(25)
    z_usd_1 = get_usd_z(stage, cube_path)
    z_fab_1 = get_fabric_z_raw(hier, cube_path)
    z_ten_1, z_ten_3_err = get_tensor_z(cube_path, use_gpu=False)
    timeline.stop()
    pump(3)

    results["tests"]["T3_isolated_physx_play"] = {
        "settings": {"updateToUsd": False, "fabricEnabled": False},
        "usd_z_before": z_usd_0,
        "usd_z_after": z_usd_1,
        "fabric_z_before": z_fab_0,
        "fabric_z_after": z_fab_1,
        "tensor_z_after": z_ten_1,
        "tensor_err": z_ten_3_err,
        "usd_frozen": abs(z_usd_1 - z_usd_0) < 1e-4,
        "fabric_frozen": abs(z_fab_1 - z_fab_0) < 1e-4,
        "tensor_moved": z_ten_1 is not None and abs(z_ten_1 - z_usd_0) > 0.05,
    }

    # ------------------------------------------------------------------
    # T4: GPU Dynamics + suppressReadback=True 時に updateToUsd=True はどうなるか
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(False)
    settings.set_bool("/physics/updateToUsd", True)
    settings.set_bool("/physics/fabricEnabled", False)
    settings.set_bool("/physics/suppressReadback", True)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=True)

    nc = NoticeCounter(stage, cube_path)
    z_usd_0 = get_usd_z(stage, cube_path)
    z_fab_0 = get_fabric_z_raw(hier, cube_path)
    timeline.play()
    pump(2)
    nc.reset()
    pump(25)
    t4_notice_play = nc.target_events
    z_usd_1 = get_usd_z(stage, cube_path)
    z_fab_1 = get_fabric_z_raw(hier, cube_path)
    z_ten_gpu, z_ten_4_err = get_tensor_z(cube_path, use_gpu=True)
    timeline.stop()
    pump(3)
    nc.revoke()
    settings.set_bool("/physics/suppressReadback", False)

    results["tests"]["T4_gpu_suppress_readback_vs_updateToUsd"] = {
        "settings": {"gpuDynamics": True, "suppressReadback": True, "updateToUsd": True, "fabricEnabled": False},
        "usd_z_before": z_usd_0,
        "usd_z_after": z_usd_1,
        "fabric_z_after": z_fab_1,
        "tensor_gpu_z_after": z_ten_gpu,
        "tensor_err": z_ten_4_err,
        "usd_notice_events_25_steps": t4_notice_play,
        "readback_blocked_usd_write": abs(z_usd_1 - z_usd_0) < 1e-4,
        "tensor_gpu_moved": z_ten_gpu is not None and abs(z_ten_gpu - z_usd_0) > 0.05,
    }

    # ------------------------------------------------------------------
    # T5: Timeline PAUSE 保持 & STOP 時の resetOnStop (True / False)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(False)
    settings.set_bool("/physics/updateToUsd", True)
    settings.set_bool("/physics/suppressReadback", False)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)

    timeline.play()
    pump(20)
    timeline.pause()
    pump(2)
    z_at_pause = get_usd_z(stage, cube_path)
    pump(12)
    z_after_pause_12 = get_usd_z(stage, cube_path)

    settings.set_bool("/physics/resetOnStop", False)
    timeline.stop()
    pump(4)
    z_stop_no_reset = get_usd_z(stage, cube_path)

    # もう一回初期から落として resetOnStop=True を確認
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)
    settings.set_bool("/physics/resetOnStop", True)
    timeline.play()
    pump(20)
    z_before_stop_reset = get_usd_z(stage, cube_path)
    timeline.stop()
    pump(4)
    z_stop_with_reset = get_usd_z(stage, cube_path)

    results["tests"]["T5_timeline_pause_and_resetOnStop"] = {
        "z_at_pause": z_at_pause,
        "z_after_pause_12_updates": z_after_pause_12,
        "pause_held_pose": abs(z_after_pause_12 - z_at_pause) < 1e-4 and z_at_pause < 4.9,
        "z_after_stop_resetOnStop_False": z_stop_no_reset,
        "stop_kept_pose_when_reset_false": abs(z_stop_no_reset - z_at_pause) < 1e-4,
        "z_before_stop_resetOnStop_True": z_before_stop_reset,
        "z_after_stop_resetOnStop_True": z_stop_with_reset,
        "stop_restored_initial_when_reset_true": abs(z_stop_with_reset - 5.0) < 1e-4,
    }

    # ------------------------------------------------------------------
    # T6: USD -> Fabric 遅延バッチ同期 & 静止中アイドル Notice = 0 の実測
    # ------------------------------------------------------------------
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)
    nc = NoticeCounter(stage, cube_path)

    # 編集せずに 10 フレーム回して Notice = 0 を確認
    nc.reset()
    pump(10)
    idle_notices = nc.target_events

    # USD を編集し、SynchronizeToFabric() 呼び出し「前」と「後」で Fabric 値を比較
    nc.reset()
    cube_prim = stage.GetPrimAtPath(cube_path)
    cube_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(1.0, 2.0, 9.0))
    edit_notices = nc.target_events
    fab_z_immediately_after_usd_edit = get_fabric_z_raw(hier, cube_path)

    usdrt_stage.SynchronizeToFabric()
    fab_z_after_explicit_sync = get_fabric_z_raw(hier, cube_path)
    nc.revoke()

    results["tests"]["T6_usd_to_fabric_delayed_sync"] = {
        "idle_10_frames_usd_notices": idle_notices,
        "usd_notices_on_single_edit": edit_notices,
        "fabric_z_immediately_after_usd_edit_before_sync": fab_z_immediately_after_usd_edit,
        "fabric_z_after_SynchronizeToFabric": fab_z_after_explicit_sync,
        "delayed_sync_confirmed": abs(fab_z_immediately_after_usd_edit - 5.0) < 1e-4
        and abs(fab_z_after_explicit_sync - 9.0) < 1e-4,
    }

    # ------------------------------------------------------------------
    # T7: Render Gate (SimulationContext.step(render=False) vs render=True)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(True)
    settings.set_bool("/physics/fabricEnabled", True)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=False)
    sim_ctx = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
    sim_ctx.initialize_physics()
    sim_ctx.play()

    t0 = time.perf_counter()
    for _ in range(30):
        sim_ctx.step(render=False)
    dt_no_render = time.perf_counter() - t0
    z_ten_no_render, z_ten_7a_err = get_tensor_z(cube_path, use_gpu=False)
    z_fab_no_render = get_fabric_z_raw(hier, cube_path)

    t1 = time.perf_counter()
    for _ in range(30):
        sim_ctx.step(render=True)
    dt_with_render = time.perf_counter() - t1
    z_ten_with_render, z_ten_7b_err = get_tensor_z(cube_path, use_gpu=False)
    z_fab_with_render = get_fabric_z_raw(hier, cube_path)
    sim_ctx.stop()
    sim_ctx.clear_instance()

    results["tests"]["T7_render_gate_step"] = {
        "steps": 30,
        "elapsed_ms_render_False": round(dt_no_render * 1000.0, 2),
        "elapsed_ms_render_True": round(dt_with_render * 1000.0, 2),
        "tensor_z_after_30_render_False": z_ten_no_render,
        "fabric_z_after_30_render_False": z_fab_no_render,
        "tensor_z_after_60_render_True": z_ten_with_render,
        "fabric_z_after_60_render_True": z_fab_with_render,
        "physics_advanced_without_render": z_ten_no_render is not None and z_ten_no_render < 4.5,
    }

    # ------------------------------------------------------------------
    # T8: Time-Sampled 属性（USD未編集でも Timeline 時刻変化で Fabric 更新）
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(False)
    stage, usdrt_stage, hier, _ = setup_scene(enable_gpu=False)
    anim_path = "/World/AnimCube"
    anim_cube = UsdGeom.Cube.Define(stage, anim_path)
    tr_op = UsdGeom.Xformable(anim_cube.GetPrim()).AddTranslateOp()
    tcps = stage.GetTimeCodesPerSecond()
    tr_op.Set(Gf.Vec3d(0.0, 0.0, 1.0), time=Usd.TimeCode(0.0))
    tr_op.Set(Gf.Vec3d(0.0, 0.0, 11.0), time=Usd.TimeCode(tcps * 1.0))
    timeline.set_current_time(0.0)
    pump(3)

    nc = NoticeCounter(stage, anim_path)
    z_anim_t0 = get_fabric_z_attr(usdrt_stage, anim_path)
    nc.reset()
    timeline.set_current_time(0.5)
    timeline.commit()
    pump(3)
    z_anim_t05 = get_fabric_z_attr(usdrt_stage, anim_path)
    anim_notices = nc.target_events
    nc.revoke()

    results["tests"]["T8_timesampled_usd_to_fabric"] = {
        "fabric_z_at_t0": z_anim_t0,
        "fabric_z_at_t0_5s": z_anim_t05,
        "usd_notices_during_time_scrub": anim_notices,
        "timesample_updated_fabric_without_usd_edit": anim_notices == 0 and abs(z_anim_t05 - 6.0) < 0.2,
    }

    # ------------------------------------------------------------------
    # T9: GPU Dynamics + suppressReadback=True + omni.physx.fabric + fabricUseGPUInterop=True
    #     (GPU Physics -> Fabric GPU 直結で USD 書き戻しゼロのまま Fabric が追従することを確認)
    # ------------------------------------------------------------------
    SimulationManager.enable_fabric(True)
    settings.set_bool("/physics/fabricEnabled", True)
    settings.set_bool("/physics/fabricUseGPUInterop", True)
    settings.set_bool("physics/fabricUseGPUInterop", True)
    settings.set_bool("/physics/suppressReadback", True)
    stage, usdrt_stage, hier, cube_path = setup_scene(enable_gpu=True)

    nc = NoticeCounter(stage, cube_path)
    z_usd_0 = get_usd_z(stage, cube_path)
    timeline.play()
    pump(2)
    nc.reset()
    pump(25)
    t9_notice_play = nc.target_events
    z_usd_1 = get_usd_z(stage, cube_path)
    z_fab_1 = get_fabric_z_raw(hier, cube_path)
    z_ten_gpu, z_ten_9_err = get_tensor_z(cube_path, use_gpu=True)
    timeline.stop()
    pump(3)
    nc.revoke()
    settings.set_bool("/physics/suppressReadback", False)

    results["tests"]["T9_gpu_fabric_interop_with_suppress_readback"] = {
        "settings": {
            "gpuDynamics": True,
            "suppressReadback": True,
            "fabricEnabled": True,
            "fabricUseGPUInterop": True,
            "updateToUsd": bool(settings.get_as_bool("/physics/updateToUsd")),
        },
        "usd_z_before": z_usd_0,
        "usd_z_after": z_usd_1,
        "fabric_z_after": z_fab_1,
        "tensor_gpu_z_after": z_ten_gpu,
        "tensor_err": z_ten_9_err,
        "usd_notice_events_25_steps": t9_notice_play,
        "usd_frozen": abs(z_usd_1 - z_usd_0) < 1e-4,
        "fabric_moved_under_suppress_readback": abs(z_fab_1 - z_usd_0) > 0.05,
        "tensor_gpu_moved": z_ten_gpu is not None and abs(z_ten_gpu - z_usd_0) > 0.05,
    }

    out_path = Path(r"c:\isaacsim-6.2.0\verify_backend_circuit_results.json")
    out_path.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print("=== VERIFY_BACKEND_CIRCUIT_RESULTS ===")
    print(json.dumps(results, indent=2, ensure_ascii=False))

    simulation_app.close()


if __name__ == "__main__":
    main()
