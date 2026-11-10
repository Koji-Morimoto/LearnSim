"""S1: kinematic 剛体を Play 中に動かし、overlap_shape で「いま重なっている相手」を調べる。

Script Editor にそのまま貼って実行できる。開いているステージは破棄して新しいステージを作る。
シーン: Mover(kinematic、動かす) / Target(kinematic、動かさない) / Wall(静的コライダー)。一辺 0.2 m の立方体。
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
import usdrt
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

USE_FABRIC = False  # True: Mover の位置を Fabric だけに書く / False: USD に書く

MOVER = "/World/Mover"
TARGET = "/World/Target"
WALL = "/World/Wall"


def make_box(stage, path, x, kinematic):
    """一辺 0.2 m の箱。親 Xform が剛体、子の Cube がコライダー。kinematic=False なら静的コライダー。"""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.5))
    cube = UsdGeom.Cube.Define(stage, path + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        body = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        body.CreateKinematicEnabledAttr(True)  # ← kinematic 剛体にする設定


def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")  # 重力は既定(9.81)のまま。kinematic は落ちない
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)  # CPU 物理。GPU 構成では Play 中の USD / Fabric 書き込みが届かないことがある
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    # overlap だけなら reportKinematic*Pairs と ContactReportAPI は不要
    make_box(stage, MOVER, 0.05, kinematic=True)
    make_box(stage, TARGET, 0.5, kinematic=True)
    make_box(stage, WALL, 1.0, kinematic=False)


def move_usd(stage, path, x):
    stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, 0.5))


def move_fabric(path, x):
    rt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(rt_stage.GetFabricId(), rt_stage.GetStageIdAsStageId())
    matrix = hierarchy.get_local_xform(usdrt.Sdf.Path(path))
    matrix.SetTranslateOnly(usdrt.Gf.Vec3d(x, 0.0, 0.5))
    hierarchy.set_local_xform(usdrt.Sdf.Path(path), matrix)


def overlapping(collider_path):
    """collider_path の形状と重なっているコライダーのパス一覧(自分自身は除く)。"""
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True  # True を返すと次のヒットも受け取る

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

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(3):  # 最初の update で PhysX にシーンが作られる
        await app.next_update_async()

    results = []
    for k in range(11):
        x = round(0.05 + 0.1 * k, 2)
        if USE_FABRIC:
            move_fabric(MOVER, x)
        else:
            move_usd(stage, MOVER, x)
        await app.next_update_async()  # ← Play 中は、書いてから update 1 回後に PhysX 上の位置が変わる
        hits = overlapping(MOVER + "/collider")
        # Target を問い合わせ側にして、Mover が PhysX 上でも動いていることを確かめる
        seen_from_target = MOVER + "/collider" in overlapping(TARGET + "/collider")
        results.append((x, hits, seen_from_target))
        print(f"x={x:.2f}  重なっている相手: {hits}  Target 側から見て Mover と重なる: {seen_from_target}")

    timeline.stop()
    await app.next_update_async()
    print("[DONE] S1")
    return results


task = asyncio.ensure_future(main())
