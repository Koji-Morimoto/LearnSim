"""Demo 01: Play=ON × UR10e × Fabric のみ駆動による 0 フレーム遅延 Newton 干渉判定。

実行方法（どちらも事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
     （開いているステージは新規ステージに置き換わります）
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_01_playon_ur10e_zero_lag.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_01_playon_ur10e_zero_lag.py --headless --frames 120

見どころ:
  - タイムライン Play=ON のまま、オレンジの DynamicBox は PhysX 重力で落下して床にバウンド
  - UR10e は GeometricLayer により PhysX に上書きされず、Fabric のみ（USD フラッシュなし）で毎フレーム駆動
  - Newton GeometricScene が Frame 0（0 フレーム遅延）で自己干渉・環境干渉・めり込み量・クリアランスを出力
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from pathlib import Path

# インストールフォルダ（別パスに置いた場合はここだけ書き換えてください）
ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"
DEFAULT_SCRIPT_EDITOR_FRAMES = 300


def _is_kit_running() -> bool:
    try:
        import omni.kit.app

        return omni.kit.app.get_app() is not None
    except Exception:
        return False


def _enable_extension() -> None:
    import omni.kit.app

    ext_root = (ISAAC_ROOT / "extsUser" / "Extentions").as_posix()
    mgr = omni.kit.app.get_app().get_extension_manager()
    if not mgr.is_extension_enabled("myext.robot.geometric_newton"):
        mgr.add_path(ext_root)
        mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)


def _setup_stage(stage):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    # Play=ON で動的剛体と共存することを示す PhysX Scene
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(850.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 25.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(6.0, 6.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.26, 0.28, 0.33)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    # 静的障害物（UR10e の手先が周期的に接近・接触する）
    obs = UsdGeom.Cube.Define(stage, "/World/Obstacle")
    obs.CreateSizeAttr(0.24)
    obs.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.25, 0.48))
    obs.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.55, 0.92)])
    UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

    # 共存する PhysX 動的剛体（Play=ON で重力に従って落下）
    dyn = UsdGeom.Cube.Define(stage, "/World/DynamicBox")
    dyn.CreateSizeAttr(0.18)
    dyn.AddTranslateOp().Set(Gf.Vec3d(-0.85, -0.55, 1.40))
    dyn.CreateDisplayColorAttr([Gf.Vec3f(0.96, 0.60, 0.14)])
    UsdPhysics.RigidBodyAPI.Apply(dyn.GetPrim())
    UsdPhysics.CollisionAPI.Apply(dyn.GetPrim())
    UsdPhysics.MassAPI.Apply(dyn.GetPrim()).CreateMassAttr(1.0)

    # ローカル UR10e アセットを参照
    if not UR10E_USD.exists():
        raise FileNotFoundError(f"UR10e アセットが見つかりません: {UR10E_USD}")
    robot_prim = stage.DefinePrim("/World/UR10e", "Xform")
    robot_prim.GetReferences().AddReference(UR10E_USD.as_posix())

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.3, -1.9, 1.5), Gf.Vec3d(0.1, 0.0, 0.5), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(24.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass
    return obs


class DemoRunner:
    def __init__(self, stage, obs_prim) -> None:
        import numpy as np
        from myext.robot.geometric_newton import GeometricRobot

        self.np = np
        self.obs_color = obs_prim.GetDisplayColorAttr()
        self.robot = GeometricRobot.from_articulation(
            stage,
            "/World/UR10e",
            write_target="fabric",
            collision_backend="newton",
        )
        q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
        self.robot.set_dof_positions(q_safe)
        self.robot.flush_to_usd()
        self.robot.compute_clearances("all", max_distance=0.12)  # 初回 finalize / カーネルコンパイルをループ外で完了
        self.step_ms: list[float] = []

    def step(self, frame: int) -> None:
        from pxr import Gf

        np = self.np
        t = frame / 60.0
        q = np.array([
            0.75 * np.sin(0.8 * t),
            -0.75 + 0.45 * np.cos(0.7 * t),
            1.45 + 1.15 * np.sin(0.9 * t),
            -1.2,
            -0.8,
            0.0,
        ])
        t0 = time.perf_counter()
        self.robot.set_dof_positions(q)  # Fabric のみ書き込み（USD flush なし）
        # 1 パスで貫通（separation <= 0）と近接（0 < separation <= 0.12）を同時に取得（2 重判定を回避）
        pairs = self.robot.compute_clearances("all", max_distance=0.12)
        hits = [p for p in pairs if p.separation is not None and p.separation <= 0.0]
        near = [p for p in pairs if p.separation is not None and p.separation > 0.0]
        t1 = time.perf_counter()
        self.step_ms.append((t1 - t0) * 1e3)

        if any(p.kind == "environment" for p in hits):
            self.obs_color.Set([Gf.Vec3f(0.95, 0.18, 0.15)])
        elif any(p.kind == "environment" for p in near):
            self.obs_color.Set([Gf.Vec3f(0.98, 0.82, 0.15)])
        else:
            self.obs_color.Set([Gf.Vec3f(0.20, 0.55, 0.92)])

        if frame % 30 == 0:
            if hits:
                depths = [p.separation * 1e3 for p in hits if p.separation is not None]
                print(
                    f"[demo_01] frame={frame:4d} 干渉={len(hits)}組 "
                    f"(最小sep={min(depths):.1f}mm, USD未書込={self.robot._usd_dirty}) / {self.step_ms[-1]:.2f}ms",
                    flush=True,
                )
            elif near and near[0].separation is not None:
                print(
                    f"[demo_01] frame={frame:4d} 接近 {near[0].names} "
                    f"(距離={near[0].separation * 1e3:.1f}mm) / {self.step_ms[-1]:.2f}ms",
                    flush=True,
                )
            else:
                print(f"[demo_01] frame={frame:4d} 安全 / {self.step_ms[-1]:.2f}ms", flush=True)

    def close(self) -> None:
        med = float(self.np.median(self.step_ms)) if self.step_ms else 0.0
        print(f"[demo_01] 終了 ({len(self.step_ms)} frames, 中央値={med:.2f}ms/frame)", flush=True)
        self.robot.close()


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.timeline
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    obs = _setup_stage(stage)
    await app.next_update_async()
    await app.next_update_async()

    runner = DemoRunner(stage, obs)
    await app.next_update_async()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for frame in range(frames):
        runner.step(frame)
        await app.next_update_async()
    tl.stop()
    await app.next_update_async()
    runner.close()


def _run_standalone() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="0 ならウィンドウを閉じるまで実行")
    args, _ = ap.parse_known_args()

    sys.path.insert(0, str(ISAAC_ROOT / "tools" / "myext"))
    from sim_app_launcher import create_simulation_app

    sim_app = create_simulation_app(
        {"headless": args.headless, "width": 1280, "height": 720},
        extra_extensions=("myext.robot.geometric_newton",),
    )

    import omni.timeline
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    sim_app.update()
    stage = ctx.get_stage()
    obs = _setup_stage(stage)
    for _ in range(3):
        sim_app.update()

    runner = DemoRunner(stage, obs)
    sim_app.update()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    frame = 0
    while sim_app.is_running() and (not args.frames or frame < args.frames):
        runner.step(frame)
        sim_app.update()
        frame += 1
    tl.stop()
    sim_app.update()
    runner.close()
    sim_app.close()


if _is_kit_running():
    task = asyncio.ensure_future(_run_script_editor(DEFAULT_SCRIPT_EDITOR_FRAMES))
else:
    _run_standalone()
