"""USD書き戻しON（updateToUsd=True）のまま omni.physx.fabric 拡張を有効にしたとき、PhysX の結果が USD に書き戻されるかを実測する。

ソース omni.physx InternalActor.cpp では updateUSD = updateToUsd && ... && !fabricEnabled となっていて、
Fabric 拡張が有効（fabricEnabled=True）だと USD に書かれない可能性があるため確かめる。

落下する動的剛体を 30 フレーム再生し、USD の位置が動いたか、立方体について USD の変更通知が何回来たかを
次の 2 条件で比べる:
- Fabric 拡張 無効 ＋ updateToUsd=True
- Fabric 拡張 有効 ＋ updateToUsd=True

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_writeback_with_fabric_ext.py
"""

import asyncio
import os

from isaacsim import SimulationApp

_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
app = SimulationApp({"headless": True}, experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"))

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics  # noqa: E402

CUBE = "/World/Cube"


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def run(fabric_ext: bool):
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.physx.fabric", fabric_ext)
    s = carb.settings.get_settings()
    omni.usd.get_context().new_stage()
    await frames(5)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    cube = UsdGeom.Cube.Define(stage, CUBE)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    s.set_bool("/physics/updateToUsd", True)
    await frames(10)

    hits = {"n": 0}

    def on_changed(notice, sender):
        for p in list(notice.GetChangedInfoOnlyPaths()) + list(notice.GetResyncedPaths()):
            if str(p).startswith(CUBE):
                hits["n"] += 1

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_changed, stage)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(CUBE))
    z0 = xf.ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(30)
    z1 = xf.ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    listener.Revoke()
    tl.stop()
    await frames(5)
    return fabric_ext, s.get("/physics/fabricEnabled"), s.get("/physics/updateToUsd"), z0, z1, hits["n"]


async def main():
    rows = [await run(False), await run(True)]
    print("\n===== RESULT =====")
    for ext, fe, utu, z0, z1, n in rows:
        moved = "動いた（書き戻された）" if abs(z1 - z0) > 1e-6 else "動かない（書き戻されない）"
        print(f"Fabric 拡張 {'有効' if ext else '無効'}（fabricEnabled={fe}）, updateToUsd={utu}: USD の位置 {moved}, 変更通知 {n} 回")
    print("===== END =====")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
