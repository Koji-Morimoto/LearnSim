"""Demo 02: PhysicsScene なしの自前 GeometricScene × 移動障害物同期（sync_environment / set_obstacle_pose）とクリアランス可視化。

実行方法（事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_02_geometric_scene_moving_env.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_02_geometric_scene_moving_env.py --headless --frames 120

見どころ:
  - ステージに UsdPhysics.Scene を一切作成しない（PhysX 不要）
  - 障害物ボックスを左右に往復移動させ、GeometricScene.sync_environment() でモデル再構築なし・0 フレーム遅延で同期
  - クリアランス（12 cm 以内で黄色）・干渉（めり込みで赤色）をリアルタイム色変化とログで確認
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from pathlib import Path

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
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    # 意図的に UsdPhysics.Scene は定義しない（GeometricScene 単体で動作する確認）

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 20.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(5.0, 5.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.26, 0.28, 0.33)])

    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    obs = UsdGeom.Cube.Define(stage, "/World/Obstacles/MovingBox")
    obs.CreateSizeAttr(0.22)
    obs_op = obs.AddTranslateOp()
    obs_op.Set(Gf.Vec3d(0.75, 0.0, 0.55))
    obs.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.55, 0.92)])
    UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

    if not UR10E_USD.exists():
        raise FileNotFoundError(f"UR10e アセットが見つかりません: {UR10E_USD}")
    robot_prim = stage.DefinePrim("/World/UR10e", "Xform")
    robot_prim.GetReferences().AddReference(UR10E_USD.as_posix())

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.1, -1.7, 1.3), Gf.Vec3d(0.25, 0.0, 0.50), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(24.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass
    return obs, obs_op


class DemoRunner:
    def __init__(self, stage, obs_prim, obs_op) -> None:
        import numpy as np
        from myext.robot.geometric_newton import GeometricRobot, GeometricScene

        self.np = np
        self.obs_op = obs_op
        self.obs_color = obs_prim.GetDisplayColorAttr()
        self.scene = GeometricScene(stage)
        self.robot = GeometricRobot.from_articulation(
            stage,
            "/World/UR10e",
            scene=self.scene,
            write_target="fabric",
            collision_backend="newton",
        )
        self.scene.add_environment("/World/Obstacles", exclude_robots=True)
        # 手先を前方へ伸ばした固定姿勢にする
        self.robot.set_dof_positions(np.array([0.0, -0.55, 0.90, -0.35, 1.57, 0.0]))
        self.tip_pos = self.robot.links["wrist_3_link"].get_world_pose()[0]
        print(f"[demo_02] PhysicsScene なしで初期化完了 / wrist_3_link={np.round(self.tip_pos, 3).tolist()}", flush=True)

    def step(self, frame: int) -> None:
        from pxr import Gf

        np = self.np
        t = frame / 60.0
        # 障害物を手先の前後へ往復移動（USD xform 更新 -> sync_environment でモデル再構築なし同期）
        offset_y = 0.42 * np.sin(1.4 * t)
        pos = self.tip_pos + np.array([0.0, offset_y, 0.0])
        self.obs_op.Set(Gf.Vec3d(*pos.tolist()))
        self.scene.sync_environment()

        # 1 パスで干渉（separation <= 0）と近接クリアランス（0 < separation <= 0.14）を同時に取得
        pairs = self.robot.compute_clearances("environment", max_distance=0.14)
        hits = [p for p in pairs if p.separation is not None and p.separation <= 0.0]
        near = [p for p in pairs if p.separation is not None and p.separation > 0.0]

        if hits:
            self.obs_color.Set([Gf.Vec3f(0.95, 0.16, 0.14)])
            if frame % 20 == 0:
                min_sep = min(p.separation for p in hits if p.separation is not None)
                print(f"[demo_02] frame={frame:4d} 干渉! sep={min_sep * 1e3:.1f} mm / pairs={[p.names for p in hits]}", flush=True)
        elif near and near[0].separation is not None:
            self.obs_color.Set([Gf.Vec3f(0.98, 0.80, 0.15)])
            if frame % 20 == 0:
                print(f"[demo_02] frame={frame:4d} 接近中 clearance={near[0].separation * 1e3:.1f} mm ({near[0].names})", flush=True)
        else:
            self.obs_color.Set([Gf.Vec3f(0.20, 0.55, 0.92)])
            if frame % 30 == 0:
                print(f"[demo_02] frame={frame:4d} 安全（140 mm 超）", flush=True)

        # 後半で set_obstacle_pose（USD を触らない超高速ダイレクト更新）も 1 回検証
        if frame == 60:
            self.scene.set_obstacle_pose("/World/Obstacles/MovingBox", position=self.tip_pos.tolist())
            direct_hits = self.robot.check_collisions("environment")
            print(f"[demo_02] set_obstacle_pose 直接移動チェック -> 検出数={len(direct_hits)}", flush=True)
            self.scene.sync_environment(reset_manual_overrides=True)

    def close(self) -> None:
        self.robot.close()
        self.scene.close()
        print("[demo_02] 終了", flush=True)


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    obs, obs_op = _setup_stage(stage)
    await app.next_update_async()
    await app.next_update_async()

    runner = DemoRunner(stage, obs, obs_op)
    for frame in range(frames):
        runner.step(frame)
        await app.next_update_async()
    runner.close()


def _run_standalone() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--frames", type=int, default=0)
    args, _ = ap.parse_known_args()

    sys.path.insert(0, str(ISAAC_ROOT / "tools" / "myext"))
    from sim_app_launcher import create_simulation_app

    sim_app = create_simulation_app(
        {"headless": args.headless, "width": 1280, "height": 720},
        extra_extensions=("myext.robot.geometric_newton",),
    )

    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    sim_app.update()
    stage = ctx.get_stage()
    obs, obs_op = _setup_stage(stage)
    for _ in range(3):
        sim_app.update()

    runner = DemoRunner(stage, obs, obs_op)
    frame = 0
    while sim_app.is_running() and (not args.frames or frame < args.frames):
        runner.step(frame)
        sim_app.update()
        frame += 1
    runner.close()
    sim_app.close()


if _is_kit_running():
    task = asyncio.ensure_future(_run_script_editor(DEFAULT_SCRIPT_EDITOR_FRAMES))
else:
    _run_standalone()
