"""静的 Collider と Kinematic 剛体を再生中に動かしたとき、USD・Fabric・PhysX・Collider 表示がどうなるかを実測する。

用語:
- 静的 Collider: CollisionAPI だけを持ち RigidBodyAPI を持たない prim。物理では動かない。
- Kinematic 剛体: RigidBodyAPI の kinematicEnabled=True。重力では動かず、xform を書き換えるとその位置へ動く。

条件（種類 × 出力先 × 動かし方）:
- 出力先: usd / fabric_cpu（physics_output_mode.set_output_mode）
- 動かし方: "usd"    = USD の translate を毎フレーム書き換える（ギズモ操作やスクリプトと同じ経路）
            "fabric" = Fabric の omni:fabric:worldMatrix を毎フレーム直接書き換える
再生後 20 フレームかけて z を 2.0 → 1.0 へ動かし、5 フレーム待ってから次を記録する（再生中・一時停止中・停止後）:
- USD の z（合成済みステージから計算）、Fabric の z（omni:fabric:worldMatrix）
- PhysX の z: 真上 (0,0,10) から真下へのレイキャストが当たった高さ − 立方体の半分（0.25）。PhysX の当たり判定の位置。
- 再生中に立方体のパスについて USD の変更通知が来た回数（自分で USD に書いた回数も併記）
- Viewport の画像（Collider 表示 = All。緑の線）

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_static_kinematic.py
画像は環境変数 SHOT_DIR（英数字だけのパス）に出す。
"""

import asyncio
import os
import sys
from typing import Literal

from isaacsim import SimulationApp

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
app = SimulationApp(
    {"headless": True, "hide_ui": False, "width": 960, "height": 540},
    experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"),
)
sys.path.insert(0, _HERE)

import carb  # noqa: E402
import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx  # noqa: E402
import omni.physx.bindings._physx as pxb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import physics_output_mode as pom  # noqa: E402

BodyType = Literal["static", "kinematic"]
MoveMethod = Literal["usd", "fabric"]

OUT = os.environ.get("SHOT_DIR", os.path.join(os.environ.get("TEMP", "C:/Temp"), "fabric_only_shots"))
os.makedirs(OUT, exist_ok=True)
CUBE = "/World/Cube"
HALF = 0.25
Z_START, Z_END, MOVE_FRAMES = 2.0, 1.0, 20


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def stage():
    return omni.usd.get_context().get_stage()


def rt_stage():
    return usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())


def usd_z():
    return UsdGeom.Xformable(stage().GetPrimAtPath(CUBE)).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]


def fabric_z():
    attr = rt_stage().GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    return float(attr.Get()[3][2]) if attr and attr.HasValue() else None


def physx_z():
    hit = omni.physx.get_physx_scene_query_interface().raycast_closest(
        carb.Float3(0.0, 0.0, 10.0), carb.Float3(0.0, 0.0, -1.0), 100.0
    )
    if not hit or not hit.get("hit"):
        return None
    return float(hit["position"][2]) - HALF


def move_usd(z):
    UsdGeom.Xformable(stage().GetPrimAtPath(CUBE)).GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, z))


def move_fabric(z):
    attr = rt_stage().GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    m = usdrt.Gf.Matrix4d(1.0)
    m.SetTranslateOnly(usdrt.Gf.Vec3d(0.0, 0.0, z))
    attr.Set(m)


async def shot(name):
    path = os.path.join(OUT, name)
    await capture_viewport_to_file(get_active_viewport(), path).wait_for_result()
    return path


async def build_stage(body: BodyType):
    omni.usd.get_context().new_stage()
    await frames(5)
    s = stage()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    scene = UsdPhysics.Scene.Define(s, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    UsdLux.DistantLight.Define(s, "/World/Light").CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(s, "/World/Dome").CreateIntensityAttr(500)
    cube = UsdGeom.Cube.Define(s, CUBE)
    cube.CreateSizeAttr(HALF * 2)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, Z_START))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if body == "kinematic":
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateKinematicEnabledAttr(True)
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[4.0, 4.0, 1.6], target=[0.0, 0.0, 1.4])
    carb.settings.get_settings().set_int(pxb.SETTING_DISPLAY_COLLIDERS, 2)  # 2 = All
    await frames(20)


async def run(body: BodyType, mode: pom.OutputMode, method: MoveMethod):
    await build_stage(body)
    pom.set_output_mode(mode)
    await frames(10)
    tag = f"{body}_{mode}_{method}"
    rows = [("再生前", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_0_before.png"))]

    hits = {"n": 0}

    def on_changed(notice, sender):
        for p in list(notice.GetChangedInfoOnlyPaths()) + list(notice.GetResyncedPaths()):
            if str(p).startswith(CUBE):
                hits["n"] += 1

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_changed, stage())
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(5)
    my_usd_writes = 0
    for i in range(1, MOVE_FRAMES + 1):
        z = Z_START + (Z_END - Z_START) * i / MOVE_FRAMES
        if method == "usd":
            move_usd(z)
            my_usd_writes += 1
        else:
            move_fabric(z)
        await frames(1)
    await frames(5)
    rows.append(("再生中", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_1_playing.png")))
    notices = hits["n"]
    tl.pause()
    await frames(10)
    rows.append(("一時停止中", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_2_paused.png")))
    listener.Revoke()
    tl.stop()
    await frames(10)
    rows.append(("停止後", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_3_stopped.png")))
    return tag, notices, my_usd_writes, rows


def fmt(v):
    return "None" if v is None else f"{v:.3f}"


async def main():
    results = []
    for body in ("static", "kinematic"):
        for method in ("usd", "fabric"):
            for mode in ("usd", "fabric_cpu"):
                results.append(await run(body, mode, method))
    print("\n===== RESULT =====")
    for tag, notices, writes, rows in results:
        print(f"[{tag}] 再生中の USD 変更通知 {notices} 回（うち自分で USD に書いた回数 {writes}）")
        for label, uz, fz, pz, path in rows:
            print(f"  {label:<6} USD z={fmt(uz)}  Fabric z={fmt(fz)}  PhysX z={fmt(pz)}")
    print("===== END =====")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
