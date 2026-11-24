"""起動中の Isaac Sim の Script Editor に貼って実行する短いデモ(Newton バックエンド版)。

注意: 開いているステージを破棄して、新しいステージを作る。

やること:
    1. 修正 DH パラメータから 6 軸アームを組む(Articulation も剛体も UsdPhysics.Scene も使わない)
    2. 手先を円に沿って動かす目標を IK で解き、関節値を Fabric に書く(Play しない)
    3. 途中にある箱との干渉と、めり込み量・クリアランスを Newton の GeometricScene で
       USD フラッシュなし・0 フレーム遅延で調べて表示する
    4. 最後に close() で Sublayer と GeometricScene を片付ける
"""

import asyncio

import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("myext.robot.geometric_newton", True)

from myext.robot.geometric_newton import GeometricRobot, IKSolverConfig, MDHParameters, MDHRow
from myext.robot.geometric_newton import transforms as tf

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
    # Newton バックエンド(既定)は自前の GeometricScene を持つため UsdPhysics.Scene は不要

    box = UsdGeom.Cube.Define(stage, "/World/Obstacle")  # 静的コライダー
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.33, 0.62))
    box.AddScaleOp().Set(Gf.Vec3d(0.16, 0.16, 0.5))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    robot = GeometricRobot.from_mdh(stage, arm_parameters(), "/World/Arm")  # collision_backend="newton" が既定
    robot.ignore_touching_pairs()  # 現在の初期姿勢で重なっている非隣接組を無視フィルタへ登録する
    solver = robot.create_ik_solver("jacobian", "link0", "tool", IKSolverConfig(orientation_mode="none", num_restarts=2))
    robot.set_dof_positions([0.0, 0.4, 0.6, 0.0, 0.5, 0.0])
    await app.next_update_async()

    hit_frames = []
    min_depth_mm = 0.0
    for k in range(FRAMES):
        t = k / 60.0
        target = [0.52 + 0.10 * np.cos(0.7 * t), 0.42 * np.sin(0.9 * t), 0.55 + 0.22 * np.sin(0.5 * t)]
        robot.solve_ik(solver, target)  # ワールド座標の目標。成功したら Fabric へ反映(USD 書き出し不要)
        pairs = robot.check_collisions()  # Newton GeometricScene による 0 フレーム遅延判定
        if pairs:
            hit_frames.append(k)
            depths = [p.separation * 1e3 for p in pairs if p.separation is not None]
            if depths:
                min_depth_mm = min(min_depth_mm, min(depths))
            if len(hit_frames) == 1 or hit_frames[-2] != k - 1:
                depth_str = f" / 最大めり込み {min(depths):.1f} mm" if depths else ""
                print(f"frame {k}: 干渉 {[p.names for p in pairs]}{depth_str}")
        elif k % 60 == 0:
            near = robot.compute_clearances("environment", max_distance=0.10)
            if near and near[0].separation is not None:
                print(f"frame {k}: 接近中 {near[0].names} (距離 {near[0].separation * 1e3:.1f} mm)")
        await app.next_update_async()

    tool_position, _ = robot.links["tool"].get_world_pose()
    print(
        f"終了。干渉フレーム数 {len(hit_frames)} / {FRAMES}、"
        f"最大めり込み {min_depth_mm:.1f} mm、最後の手先位置 {np.round(tool_position, 3).tolist()}"
    )
    result = {
        "hit_frames": hit_frames,
        "min_depth_mm": round(min_depth_mm, 3),
        "dof_positions": robot.get_dof_positions().tolist(),
        "tool_position": tool_position.tolist(),
    }
    robot.close()  # Sublayer と GeometricScene を片付ける
    return result


task = asyncio.ensure_future(main())
