"""S3: Play しないまま(Stop 中)、kinematic 剛体を動かして overlap_shape で調べる。

Script Editor にそのまま貼って実行できる。開いているステージは破棄して新しいステージを作る。
注意: update_simulation() はシーン全体の物理を 1 ステップ進める。動的剛体があると PhysX 上で動くので、
      動的剛体が無いシーンで使う。
"""

import asyncio

import omni.kit.app
import omni.usd
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

MOVER = "/World/Mover"
TARGET = "/World/Target"
WALL = "/World/Wall"


def make_box(stage, path, x, kinematic):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.5))
    cube = UsdGeom.Cube.Define(stage, path + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim()).CreateKinematicEnabledAttr(True)


def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    make_box(stage, MOVER, 0.05, kinematic=True)
    make_box(stage, TARGET, 0.5, kinematic=True)
    make_box(stage, WALL, 1.0, kinematic=False)


def overlapping(collider_path):
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True

    id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(collider_path))
    get_physx_scene_query_interface().overlap_shape(id0, id1, report, False)
    return sorted(h for h in hits if h != collider_path)


async def main():
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    await context.new_stage_async()
    stage = context.get_stage()
    build_scene(stage)
    await app.next_update_async()

    physx = get_physx_interface()
    physx.force_load_physics_from_usd()  # ← Stop 中は PhysX にシーンが無いので、USD から読み込ませる

    results = []
    for k in range(11):
        x = round(0.05 + 0.1 * k, 2)
        stage.GetPrimAtPath(MOVER).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, 0.5))  # USD に書く(Fabric だけでは届かない)
        physx.update_simulation(1.0 / 60.0, 0.0)  # ← 1 ステップ進めると、kinematic の位置が PhysX に入る
        hits = overlapping(MOVER + "/collider")
        # Target を問い合わせ側にして、Mover が PhysX 上で動いていることも確かめる
        seen_from_target = MOVER + "/collider" in overlapping(TARGET + "/collider")
        results.append((x, hits, seen_from_target))
        print(f"x={x:.2f}  重なっている相手: {hits}  Target 側から見て Mover と重なる: {seen_from_target}")

    print("[DONE] S3")
    return results


task = asyncio.ensure_future(main())
