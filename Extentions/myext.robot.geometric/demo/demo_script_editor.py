"""起動中の Isaac Sim の Script Editor に貼って実行する短いデモ。

注意: 開いているステージを破棄して、新しいステージを作る。

やること:
    1. 修正 DH パラメータから 6 軸アームを組む(Articulation も剛体も使わない)
    2. 手先を円に沿って動かす目標を IK で解き、関節値を書く(Play しない)
    3. 途中にある箱との干渉を、関節値を書いた直後に調べて表示する
    4. 最後に close() で Sublayer を外す
"""

import asyncio

import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("myext.robot.geometric", True)

from myext.robot.geometric import GeometricRobot, IKSolverConfig, MDHParameters, MDHRow
from myext.robot.geometric import transforms as tf

FRAMES = 240


def arm_parameters() -> MDHParameters:
    pi = np.pi
    limits = dict(lower_limit=-pi, upper_limit=pi)
    return MDHParameters(
        rows=[
            MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.40, **limits),
            MDHRow(alpha=-pi / 2, a=0.05, theta=-pi / 2, d=0.0, **limits),
            MDHRow(alpha=0.0, a=0.45, theta=0.0, d=0.0, **limits),
            MDHRow(alpha=-pi / 2, a=0.04, theta=0.0, d=0.42, **limits),
            MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, **limits),
            MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.10, **limits),
        ],
        tool=tf.translation(z=0.08),
    )


async def main():
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    await context.new_stage_async()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")  # 干渉チェック(シーンクエリ)に必要
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateEnableGPUDynamicsAttr(False)

    box = UsdGeom.Cube.Define(stage, "/World/Obstacle")  # 静的コライダー
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.33, 0.62))
    box.AddScaleOp().Set(Gf.Vec3d(0.16, 0.16, 0.5))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    robot = GeometricRobot.from_mdh(stage, arm_parameters(), "/World/Arm")  # Sublayer が付き、幾何モードになる
    robot.ignore_touching_pairs()  # 簡易形状で常に重なる組を、無視する組に足す
    solver = robot.create_ik_solver("jacobian", "link0", "tool", IKSolverConfig(orientation_mode="none", num_restarts=2))
    robot.set_dof_positions([0.0, 0.4, 0.6, 0.0, 0.5, 0.0])
    await app.next_update_async()

    hit_frames = []
    for k in range(FRAMES):
        t = k / 60.0
        target = [0.52 + 0.10 * np.cos(0.7 * t), 0.42 * np.sin(0.9 * t), 0.55 + 0.22 * np.sin(0.5 * t)]
        robot.solve_ik(solver, target)  # ワールド座標の目標。成功したら関節値に反映する
        pairs = robot.check_collisions()  # Play していなくても、いまの関節値で判定される
        if pairs:
            hit_frames.append(k)
            if len(hit_frames) == 1 or hit_frames[-2] != k - 1:
                print(f"frame {k}: 干渉 {[p.names for p in pairs]}")
        await app.next_update_async()

    tool_position, _ = robot.links["tool"].get_world_pose()
    print(f"終了。干渉していたフレーム数 {len(hit_frames)} / {FRAMES}、最後の手先の位置 {np.round(tool_position, 3).tolist()}")
    result = {"hit_frames": hit_frames, "dof_positions": robot.get_dof_positions().tolist(), "tool_position": tool_position.tolist()}
    robot.close()  # Sublayer を外す(アームは組んだときの姿勢に戻る)
    return result


task = asyncio.ensure_future(main())
