"""GPU 物理のシーンで、Show By Type の Collider 表示が USD と Fabric のどちらの位置を見るかを実測する。

USD書き戻しON（mode "usd"）と USD書き戻しOFF（Fabric GPU、mode "fabric_gpu"）の両方を同じ手順で撮る。
Fabric GPU は GPU 物理のシーンで使うため、物理シーンは enableGPUDynamics=True にする。
種類: dynamic（動的剛体）/ static（静的 Collider）/ kinematic（Kinematic 剛体）

各種類で 2 回（毎回ステージを作り直す）:
- Play OFF: 停止中に omni:fabric:worldMatrix の z を 2.0 → 1.0 に書き、30 フレーム後に撮る。
- Play ON: 再生して、動的剛体はそのまま落とす。静的と Kinematic は Fabric の z を 20 フレームかけて 2.0 → 1.0 に書く。
  再生中に撮り、一時停止して 10 フレーム後に撮る。
記録: USD の z（合成済みステージから計算）、Fabric の z（omni:fabric:worldMatrix）。
Collider 表示 = All（線の色: 動的=緑、静的=マゼンタ、Kinematic=黄）。灰色の立方体は Fabric を読む描画。

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_fabric_gpu_collider.py
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

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx.bindings._physx as pxb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import physics_output_mode as pom  # noqa: E402

BodyType = Literal["dynamic", "static", "kinematic"]

OUT = os.environ.get("SHOT_DIR", os.path.join(os.environ.get("TEMP", "C:/Temp"), "fabric_only_shots"))
os.makedirs(OUT, exist_ok=True)
CUBE = "/World/Cube"
HALF = 0.25
Z_START, Z_FABRIC = 2.0, 1.0


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


def write_fabric(z):
    attr = rt_stage().GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    m = usdrt.Gf.Matrix4d(1.0)
    m.SetTranslateOnly(usdrt.Gf.Vec3d(0.0, 0.0, z))
    attr.Set(m)


async def shot(name):
    path = os.path.join(OUT, name)
    await capture_viewport_to_file(get_active_viewport(), path).wait_for_result()
    return path


async def build_stage(body: BodyType, mode: pom.OutputMode):
    omni.usd.get_context().new_stage()
    await frames(5)
    s = stage()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    scene = UsdPhysics.Scene.Define(s, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(True)
    physx_scene.CreateBroadphaseTypeAttr("GPU")
    UsdLux.DistantLight.Define(s, "/World/Light").CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(s, "/World/Dome").CreateIntensityAttr(500)
    cube = UsdGeom.Cube.Define(s, CUBE)
    cube.CreateSizeAttr(HALF * 2)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, Z_START))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if body != "static":
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateKinematicEnabledAttr(body == "kinematic")
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[4.0, 4.0, 1.6], target=[0.0, 0.0, 1.4])
    carb.settings.get_settings().set_int(pxb.SETTING_DISPLAY_COLLIDERS, 2)  # 2 = All
    await frames(20)
    pom.set_output_mode(mode)
    await frames(10)


async def run_play_off(body: BodyType, mode: pom.OutputMode):
    await build_stage(body, mode)
    rows = [("Play OFF・書く前", usd_z(), fabric_z(), await shot(f"{mode}_{body}_off_0_before.png"))]
    write_fabric(Z_FABRIC)
    await frames(30)
    rows.append(("Play OFF・Fabric に書いた後", usd_z(), fabric_z(), await shot(f"{mode}_{body}_off_1_written.png")))
    return rows


async def run_play_on(body: BodyType, mode: pom.OutputMode):
    await build_stage(body, mode)
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(5)
    for i in range(1, 21):
        if body != "dynamic":
            write_fabric(Z_START + (Z_FABRIC - Z_START) * i / 20)
        await frames(1)
    await frames(5)
    rows = [("Play ON・再生中", usd_z(), fabric_z(), await shot(f"{mode}_{body}_on_1_playing.png"))]
    tl.pause()
    await frames(10)
    rows.append(("Play ON・一時停止中", usd_z(), fabric_z(), await shot(f"{mode}_{body}_on_2_paused.png")))
    tl.stop()
    await frames(10)
    return rows


def fmt(v):
    return "None" if v is None else f"{v:.3f}"


async def main():
    s = carb.settings.get_settings()
    initial_colliders = s.get(pxb.SETTING_DISPLAY_COLLIDERS)
    results = []
    for mode in ("usd", "fabric_gpu"):
        for body in ("dynamic", "static", "kinematic"):
            results.append((mode, body, await run_play_off(body, mode) + await run_play_on(body, mode)))
    print("\n===== RESULT =====")
    for mode, body, rows in results:
        print(f"[{mode} {body}]")
        for label, uz, fz, path in rows:
            print(f"  {label} USD z={fmt(uz)}  Fabric z={fmt(fz)}  画像={os.path.basename(path)}")
    print("===== END =====")
    s.set(pxb.SETTING_DISPLAY_COLLIDERS, initial_colliders)  # 保存される設定なので初期値に戻す
    await frames(5)


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
