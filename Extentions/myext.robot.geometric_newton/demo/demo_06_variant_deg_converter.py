"""Demo 06: VariantSet グリッパ切替 × 度単位（angle_unit="deg"）× CommandConverter 連動デモ。

実行方法（事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_06_variant_deg_converter.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_06_variant_deg_converter.py --headless --frames 180

見どころ:
  - UR10e の Gripper VariantSet を Robotiq_2F_85 -> None -> Robotiq_2F_140 へ実行中に自動切替
  - 構造変化に応じて DOF 数（12 -> 6 -> 14）、GeometricLayer、Newton GeometricScene がすべて自動再構築
  - angle_unit="deg"（度単位）と、Variant 条件付き（when=）LinearCommandConverter による指令変換・逆変換を確認
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"
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


def _setup_stage(stage) -> None:
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
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(5.0, 5.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.26, 0.28, 0.33)])

    if not UR10E_USD.exists():
        raise FileNotFoundError(f"UR10e アセットが見つかりません: {UR10E_USD}")
    prim = stage.DefinePrim("/World/UR10e", "Xform")
    prim.GetReferences().AddReference(UR10E_USD.as_posix())
    prim.GetVariantSet("Gripper").SetVariantSelection("Robotiq_2F_85")

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.8, -1.5, 1.2), Gf.Vec3d(0.0, 0.0, 0.55), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(26.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass


class DemoRunner:
    def __init__(self, stage) -> None:
        import numpy as np
        from myext.robot.geometric_newton import GeometricRobot, LinearCommandConverter

        self.np = np
        self.robot = GeometricRobot.from_articulation(
            stage,
            "/World/UR10e",
            angle_unit="deg",
            collision_backend="newton",
        )
        self.robot.add_structure_listener(
            lambda old_dofs, new_dofs: print(
                f"[demo_06] 構造再構築: DOF {len(old_dofs)} -> {len(new_dofs)} / 変換={self.robot.active_command_converters()}",
                flush=True,
            )
        )
        # グリッパ開度コマンド [0..1] を finger_joint [deg] へ変換（Variant 条件付き）
        conv_85 = LinearCommandConverter(["grip_ratio"], ["finger_joint"], [[40.0]], output_angle_unit="deg")
        conv_140 = LinearCommandConverter(["grip_ratio"], ["finger_joint"], [[35.0]], output_angle_unit="deg")
        self.robot.set_command_converter("grip_2f85", conv_85, when={("", "Gripper"): "Robotiq_2F_85"})
        self.robot.set_command_converter("grip_2f140", conv_140, when={("", "Gripper"): "Robotiq_2F_140"})
        print(f"[demo_06] Variant 選択肢: {self.robot.get_variant_choices()}", flush=True)

    def step(self, frame: int) -> None:
        np = self.np
        # 60 フレームごとにグリッパ Variant を循環切替
        schedule = {0: "Robotiq_2F_85", 60: "None", 120: "Robotiq_2F_140", 180: "Robotiq_2F_85"}
        if frame in schedule:
            target_variant = schedule[frame]
            cur = self.robot.get_variants().get("/World/UR10e", {}).get("Gripper")
            if cur != target_variant:
                self.robot.set_variant("Gripper", target_variant)
                print(f"[demo_06] frame={frame:4d} Gripper -> {target_variant} (DOF={self.robot.num_dofs})", flush=True)

        t = frame / 60.0
        cmd: dict[str, float] = {
            "shoulder_pan_joint": float(35.0 * np.sin(1.0 * t)),
            "shoulder_lift_joint": float(-75.0 + 18.0 * np.cos(0.9 * t)),
            "elbow_joint": float(65.0 + 25.0 * np.sin(1.2 * t)),
            "wrist_1_joint": -80.0,
            "wrist_2_joint": -90.0,
        }
        if self.robot.active_command_converters():
            cmd["grip_ratio"] = float(0.5 + 0.5 * np.sin(2.5 * t))
        self.robot.apply_command(cmd)  # angle_unit="deg" で解釈 + Converter 適用

        if frame % 30 == 0:
            hits = self.robot.check_collisions("within")
            print(
                f"[demo_06] frame={frame:4d} variant={self.robot.get_variants()['/World/UR10e']['Gripper']} "
                f"pan={self.robot.joints['shoulder_pan_joint'].get_position():.1f}deg "
                f"read_command={self.robot.read_command()} 自己干渉={len(hits)}",
                flush=True,
            )

    def close(self) -> None:
        self.robot.close()
        print("[demo_06] 終了", flush=True)


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    _setup_stage(stage)
    await app.next_update_async()
    await app.next_update_async()

    runner = DemoRunner(stage)
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
    _setup_stage(stage)
    for _ in range(3):
        sim_app.update()

    runner = DemoRunner(stage)
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
