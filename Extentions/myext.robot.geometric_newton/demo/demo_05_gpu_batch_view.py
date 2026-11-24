"""Demo 05: GeometricRobotView × 8 台並列 GPU バッチ（Newton FK + CUDA Graph + GPU 直接 Fabric 書込 + バッチ干渉判定）。

実行方法（事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_05_gpu_batch_view.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_05_gpu_batch_view.py --headless --frames 120

見どころ:
  - 8 台の UR10e を 4x2 グリッドに自動配置し、GeometricRobotView 1 つで一括制御
  - fk_backend="newton" + use_gpu_fabric_writer=True + use_cuda_graph=True で GPU 完結の高速更新
  - 偶数番ロボットは安全姿勢、奇数番ロボットは自己干渉姿勢へ交互に動かし、check_collisions_batch / compute_clearances_batch を一括計測
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"
NUM_ROBOTS = 8
DEFAULT_SCRIPT_EDITOR_FRAMES = 240


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


def _setup_stage(stage) -> list[str]:
    from pxr import Gf, UsdGeom, UsdLux

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 20.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(2.4, 1.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(10.0, 7.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.27, 0.32)])

    if not UR10E_USD.exists():
        raise FileNotFoundError(f"UR10e アセットが見つかりません: {UR10E_USD}")

    paths: list[str] = []
    for i in range(NUM_ROBOTS):
        p = f"/World/UR_{i}"
        prim = stage.DefinePrim(p, "Xform")
        prim.GetReferences().AddReference(UR10E_USD.as_posix())
        row, col = divmod(i, 4)
        UsdGeom.Xformable(prim).ClearXformOpOrder()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(col * 1.6, row * 1.8, 0.0))
        paths.append(p)

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.4, -3.8, 3.2), Gf.Vec3d(2.4, 0.9, 0.4), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(24.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass
    return paths


class DemoRunner:
    def __init__(self, stage, paths: list[str]) -> None:
        import numpy as np
        from myext.robot.geometric_newton import GeometricRobotView

        self.np = np
        self.view = GeometricRobotView.from_articulations(
            stage,
            paths,
            write_target="fabric",
            collision_backend="newton",
            fk_backend="newton",
            use_cuda_graph=True,
            use_gpu_fabric_writer=True,
        )
        self.q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
        self.q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])
        self.fk_ms: list[float] = []
        self.col_ms: list[float] = []
        print(f"[demo_05] {len(self.view)} 台の UR10e を GPU バッチ View で初期化完了 (device={self.view.newton_scene.device})", flush=True)

    def step(self, frame: int) -> None:
        np = self.np
        t = frame / 60.0
        q_batch = np.tile(self.q_safe, (len(self.view), 1))
        for i in range(len(self.view)):
            if i % 2 == 0:
                q_batch[i, 0] = 0.8 * np.sin(0.9 * t + i * 0.5)
                q_batch[i, 2] = 0.6 * np.sin(1.2 * t + i * 0.3)
            else:
                alpha = 0.5 * (1.0 - np.cos(1.2 * t + i * 0.4))
                q_batch[i] = (1.0 - alpha) * self.q_safe + alpha * self.q_self

        t0 = time.perf_counter()
        self.view.set_dof_positions(q_batch)
        t1 = time.perf_counter()
        hits = self.view.check_collisions_batch("within")
        t2 = time.perf_counter()
        self.fk_ms.append((t1 - t0) * 1e3)
        self.col_ms.append((t2 - t1) * 1e3)

        if frame % 30 == 0:
            counts = [len(h) for h in hits]
            cl = self.view.compute_clearances_batch("within", max_distance=0.08)
            cl_counts = [len(c) for c in cl]
            print(
                f"[demo_05] frame={frame:4d} 干渉数={counts} 近接数={cl_counts} / "
                f"FK+GPU_Fabric={self.fk_ms[-1]:.2f}ms, BatchCollide={self.col_ms[-1]:.2f}ms",
                flush=True,
            )

    def close(self) -> None:
        if self.fk_ms:
            print(
                f"[demo_05] 終了 ({len(self.fk_ms)} frames, "
                f"FK+GPU_Fabric中央値={float(self.np.median(self.fk_ms)):.2f}ms, "
                f"BatchCollide中央値={float(self.np.median(self.col_ms)):.2f}ms)",
                flush=True,
            )
        self.view.close()


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    paths = _setup_stage(stage)
    await app.next_update_async()
    await app.next_update_async()

    runner = DemoRunner(stage, paths)
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
    paths = _setup_stage(stage)
    for _ in range(3):
        sim_app.update()

    runner = DemoRunner(stage, paths)
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
