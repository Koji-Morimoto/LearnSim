"""Demo 04: 外部 USD 不要 — DH / MDH / Screw ロボット生成・attach() 双腕結合・Jacobian IK 追従デモ。

実行方法（事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_04_dh_mdh_screw_ik_dual_arm.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_04_dh_mdh_screw_ik_dual_arm.py --headless --frames 120

見どころ:
  - 外部 USD アセット不要で、標準 DH・修正 DH・スクリュー軸からロボットを構築
  - 台車モデルへ左右 2 本の 6 軸 MDH アームを attach() で結合し、左アームは 6 軸 IK、右アームはアクティブ軸マスク付き IK で追従
  - 中央の箱に干渉したリンクを赤くハイライト（Newton GeometricScene・めり込み量ログ付き）
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
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

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 20.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(6.0, 6.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.26, 0.28, 0.33)])

    # 障害物ボックス（左アームの軌道上）
    obs = UsdGeom.Cube.Define(stage, "/World/Obstacle")
    obs.CreateSizeAttr(1.0)
    obs.AddTranslateOp().Set(Gf.Vec3d(0.42, 0.30, 0.58))
    obs.AddScaleOp().Set(Gf.Vec3d(0.14, 0.14, 0.42))
    obs.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.52, 0.90)])
    UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

    # 目標球（左アーム用・右アーム用）
    t_left = UsdGeom.Sphere.Define(stage, "/World/TargetLeft")
    t_left.CreateRadiusAttr(0.035)
    t_left_op = t_left.AddTranslateOp()
    t_left_op.Set(Gf.Vec3d(0.45, 0.25, 0.55))
    t_left.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.85, 0.10)])

    t_right = UsdGeom.Sphere.Define(stage, "/World/TargetRight")
    t_right.CreateRadiusAttr(0.035)
    t_right_op = t_right.AddTranslateOp()
    t_right_op.Set(Gf.Vec3d(0.45, -0.25, 0.55))
    t_right.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.92, 0.55)])

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.3, -1.5, 1.5), Gf.Vec3d(0.2, 0.0, 0.45), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(24.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass
    return t_left_op, t_right_op


class DemoRunner:
    def __init__(self, stage, t_left_op, t_right_op) -> None:
        import numpy as np
        from myext.robot.geometric_newton import (
            DHParameters,
            DHRow,
            GeometricLinkSpec,
            GeometricRobot,
            GeometricRobotModel,
            IKSolverConfig,
            MDHParameters,
            MDHRow,
            ScrewAxisRow,
            ScrewParameters,
        )
        from myext.robot.geometric_newton import transforms as tf

        self.np = np
        self.stage = stage
        self.t_left_op = t_left_op
        self.t_right_op = t_right_op

        pi = np.pi
        lim = dict(lower_limit=-pi, upper_limit=pi)
        mdh_arm = MDHParameters(
            rows=[
                MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.35, **lim),
                MDHRow(alpha=-pi / 2, a=0.04, theta=-pi / 2, d=0.0, **lim),
                MDHRow(alpha=0.0, a=0.38, theta=0.0, d=0.0, **lim),
                MDHRow(alpha=-pi / 2, a=0.03, theta=0.0, d=0.36, **lim),
                MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, **lim),
                MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.09, **lim),
            ],
            tool=tf.translation(z=0.07),
            name="mdh6",
        ).to_model()

        # 台車 + 左右 2 本の MDH アームを attach() で結合
        cart = GeometricRobotModel("cart", (GeometricLinkSpec("body"),), ())
        dual_model = (
            cart.attach(mdh_arm, "body", tf.translation(y=0.28), name="left")
                .attach(mdh_arm, "body", tf.translation(y=-0.28), name="right")
        )
        self.dual = GeometricRobot.from_model(stage, dual_model, "/World/DualArm")
        # 複数姿勢サンプリングで全サンプル共通に重なる組だけを除外（姿勢依存の自己干渉は残す）
        self.dual.ignore_touching_pairs(samples=8)

        # 参照展示: 標準 DH ロボットとスクリュー軸ロボットも隣に並べて生成
        dh_params = DHParameters(
            rows=[
                DHRow(theta=0.0, d=0.30, a=0.0, alpha=pi / 2, **lim),
                DHRow(theta=0.0, d=0.0, a=0.32, alpha=0.0, **lim),
                DHRow(theta=0.0, d=0.0, a=0.28, alpha=0.0, **lim),
            ],
            name="dh3",
        )
        self.dh_robot = GeometricRobot.from_dh(stage, dh_params, "/World/DH_Arm", base_transform=tf.translation(x=-0.9, y=0.5))

        screw_params = ScrewParameters(
            rows=[
                ScrewAxisRow.through(point=(0.0, 0.0, 0.25), direction=(0.0, 0.0, 1.0), **lim),
                ScrewAxisRow.through(point=(0.0, 0.0, 0.25), direction=(0.0, 1.0, 0.0), **lim),
                ScrewAxisRow.through(point=(0.30, 0.0, 0.25), direction=(0.0, 1.0, 0.0), **lim),
            ],
            tool_home=tf.translation(x=0.55, z=0.25),
            name="screw3",
        )
        self.screw_robot = GeometricRobot.from_screw(stage, screw_params, "/World/Screw_Arm", base_transform=tf.translation(x=-0.9, y=-0.5))

        # 左アーム: 全 6 軸で IK、右アーム: 最初の 3 軸だけをアクティブにした IK
        cfg = IKSolverConfig(orientation_mode="none", num_restarts=2)
        self.ik_left = self.dual.create_ik_solver("jacobian", "left/link0", "left/tool", cfg)
        self.ik_right = self.dual.create_ik_solver("jacobian", "right/link0", "right/tool", cfg)
        self.ik_right.set_active_dofs([True, True, True, False, False, False])

        self.dual.get_component("left").set_dof_positions([0.0, 0.3, 0.5, 0.0, 0.4, 0.0])
        self.dual.get_component("right").set_dof_positions([0.0, 0.3, 0.5, 0.0, 0.4, 0.0])
        self.red_links: set[str] = set()
        print(f"[demo_04] 生成完了: DualArm ({self.dual.num_dofs} DOF), DH ({self.dh_robot.num_dofs} DOF), Screw ({self.screw_robot.num_dofs} DOF)", flush=True)

    def _paint(self, hit_links: set[str]) -> None:
        from pxr import Gf, UsdGeom

        if hit_links == self.red_links:
            return
        red, gray = Gf.Vec3f(0.95, 0.15, 0.12), Gf.Vec3f(0.75, 0.78, 0.82)
        with self.dual.layer.edit_context():
            for link in hit_links ^ self.red_links:
                col = red if link in hit_links else gray
                vis = self.stage.GetPrimAtPath(self.dual.link_path_map[link] + "/visuals")
                if vis and vis.IsValid():
                    for prim in vis.GetChildren():
                        if prim.GetName().startswith("bone"):
                            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([col])
        self.red_links = set(hit_links)

    def step(self, frame: int) -> None:
        from pxr import Gf

        np = self.np
        t = frame / 60.0
        tgt_l = [0.45 + 0.08 * np.cos(0.9 * t), 0.28 + 0.22 * np.sin(1.1 * t), 0.52 + 0.16 * np.sin(0.7 * t)]
        tgt_r = [0.46 + 0.08 * np.sin(0.9 * t), -0.28 + 0.18 * np.cos(1.0 * t), 0.50 + 0.14 * np.cos(0.7 * t)]
        self.t_left_op.Set(Gf.Vec3d(*tgt_l))
        self.t_right_op.Set(Gf.Vec3d(*tgt_r))

        res_l = self.dual.solve_ik(self.ik_left, tgt_l)
        res_r = self.dual.solve_ik(self.ik_right, tgt_r)
        self.dh_robot.set_dof_positions([0.8 * np.sin(t), 0.4 * np.cos(t), 0.5 * np.sin(1.3 * t)])
        self.screw_robot.set_dof_positions([0.8 * np.cos(t), 0.4 * np.sin(t), 0.5 * np.cos(1.3 * t)])

        pairs = self.dual.check_collisions("all")
        hit_links = {p.link_a for p in pairs} | {p.link_b for p in pairs if p.link_b}
        self._paint(hit_links)

        if frame % 30 == 0:
            depths = [p.separation * 1e3 for p in pairs if p.separation is not None]
            d_str = f"min_sep={min(depths):.1f}mm" if depths else "干渉なし"
            print(
                f"[demo_04] frame={frame:4d} IK_L={res_l.success}({res_l.position_error * 1e3:.2f}mm) "
                f"IK_R(3軸)={res_r.success}({res_r.position_error * 1e3:.2f}mm) / {d_str}",
                flush=True,
            )

    def close(self) -> None:
        self.dual.close()
        self.dh_robot.close()
        self.screw_robot.close()
        print("[demo_04] 終了", flush=True)


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    ops = _setup_stage(stage)
    await app.next_update_async()

    runner = DemoRunner(stage, *ops)
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
    ops = _setup_stage(stage)
    sim_app.update()

    runner = DemoRunner(stage, *ops)
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
