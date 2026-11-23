"""Play=ON × Newton GeometricScene (0 フレーム遅延) の単独デモ。

実行:
    C:\\isaacsim-6.2.0\\python.bat demo\\demo_playon_newton.py --no-ros-env
    C:\\isaacsim-6.2.0\\python.bat demo\\demo_playon_newton.py --headless --frames 120 --no-ros-env

見せること:
    1. タイムラインを Play=ON にし、周囲の PhysX 動的剛体(落下する箱)が普通に物理シミュレーションされている状態
    2. GeometricLayer により PhysX の上書きを防ぎつつ、UR10e を Fabric のみで毎フレーム駆動
    3. USD への flush なしで、Newton の GeometricScene が 0 フレーム遅延で自己干渉・環境干渉・
       めり込み量(separation <= 0)・クリアランス(0 < separation <= max_distance)を毎フレーム算出
"""

from __future__ import annotations

import argparse
import os
import time

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--frames", type=int, default=0, help="0 ならウィンドウを閉じるまで続ける")
args, _ = parser.parse_known_args()

EXT_FOLDER = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "extra_args": ["--ext-folder", EXT_FOLDER]})

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("myext.robot.geometric_newton", True)

from myext.robot.geometric_newton import GeometricRobot

ISAAC_DIR = os.environ.get("ISAAC_PATH") or os.path.dirname(os.path.dirname(EXT_FOLDER))
UR10E_USD = os.path.join(ISAAC_DIR, "exts", "isaacsim.asset.transformer.rules", "data", "tests", "ur10e", "ur10e.usd")


def log(msg: str) -> None:
    print(f"[demo_playon_newton] {msg}", flush=True)


def build_stage():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    # Play=ON 中の共存を示すための PhysX シーン + 落下する動的箱
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
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.30, 0.35)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    # 静的障害物(ロボットの手先が周期的に接近・接触する)
    obs = UsdGeom.Cube.Define(stage, "/World/Obstacle")
    obs.CreateSizeAttr(0.25)
    obs.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.25, 0.45))
    obs.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.55, 0.90)])
    UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

    # 共存する PhysX 動的剛体(Play=ON で重力に従って落ち、床で止まる)
    dyn = UsdGeom.Cube.Define(stage, "/World/DynamicBox")
    dyn.CreateSizeAttr(0.18)
    dyn.AddTranslateOp().Set(Gf.Vec3d(-0.9, -0.6, 1.2))
    dyn.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.65, 0.15)])
    UsdPhysics.RigidBodyAPI.Apply(dyn.GetPrim())
    UsdPhysics.CollisionAPI.Apply(dyn.GetPrim())
    UsdPhysics.MassAPI.Apply(dyn.GetPrim()).CreateMassAttr(1.0)

    # UR10e
    robot_prim = stage.DefinePrim("/World/UR10e", "Xform")
    robot_prim.GetReferences().AddReference(UR10E_USD)
    app.update()
    return stage


def main() -> None:
    if not os.path.exists(UR10E_USD):
        log(f"UR10e アセットが見つからないため終了: {UR10E_USD}")
        return
    stage = build_stage()
    robot = GeometricRobot.from_articulation(
        stage,
        "/World/UR10e",
        write_target="fabric",
        collision_backend="newton",
    )
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    robot.set_dof_positions(q_safe)
    robot.flush_to_usd()
    app.update()

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    log("Play=ON 開始: DynamicBox は PhysX で落下しつつ、UR10e は Fabric + Newton 0 フレーム遅延判定で動作")

    frame = 0
    step_ms: list[float] = []
    while app.is_running() and (not args.frames or frame < args.frames):
        t = frame / 60.0
        q = np.array(
            [
                0.7 * np.sin(0.8 * t),
                -0.8 + 0.45 * np.cos(0.7 * t),
                1.4 + 1.2 * np.sin(0.9 * t),
                -1.2,
                -0.8,
                0.0,
            ]
        )
        t0 = time.perf_counter()
        robot.set_dof_positions(q)  # Fabric のみに書き込み(USD flush なし)
        hits = robot.check_collisions("all")  # Newton 0 フレーム遅延判定
        near = [] if hits else robot.compute_clearances("all", max_distance=0.10)
        t1 = time.perf_counter()
        step_ms.append((t1 - t0) * 1e3)

        if frame % 30 == 0:
            if hits:
                depths = [p.separation * 1e3 for p in hits if p.separation is not None]
                log(f"frame {frame:4d}: 干渉 {len(hits)} 組 (最小 sep={min(depths):.1f} mm) / 処理 {step_ms[-1]:.2f} ms")
            elif near and near[0].separation is not None:
                log(f"frame {frame:4d}: 干渉なし / 最接近 {near[0].names} ({near[0].separation * 1e3:.1f} mm) / 処理 {step_ms[-1]:.2f} ms")
            else:
                log(f"frame {frame:4d}: 干渉なし / 処理 {step_ms[-1]:.2f} ms")

        app.update()
        frame += 1

    log(f"終了 ({frame} frames, 中央値 {float(np.median(step_ms)):.2f} ms / frame)")
    tl.stop()
    app.update()
    robot.close()


main()
app.close()
