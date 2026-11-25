"""Fabric のみ出力のとき、USD への書き込みと Collider 表示（デバッグ表示）がどうなるかを実測する。

落下する立方体（Collider 表示 = All）を、出力先 usd / fabric_cpu で 20 ステップ再生し、
再生中・一時停止中・停止後のそれぞれで次を記録する。
- USD の z（合成済みステージから計算）と Fabric の z（omni:fabric:worldMatrix）
- 再生中に立方体のパスについて USD の変更通知（Usd.Notice.ObjectsChanged）が来た回数
- Viewport の画像（Collider の線が立方体についてくるか、再生前の位置に残るか）

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_collider_display.py
画像は環境変数 SHOT_DIR（英数字だけのパス）に出す。
"""

import asyncio
import os
import sys

from isaacsim import SimulationApp

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
app = SimulationApp(
    {"headless": True, "hide_ui": False, "width": 1280, "height": 720},
    experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"),
)
sys.path.insert(0, _HERE)

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx.bindings._physx as pxb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import physics_output_mode as pom  # noqa: E402

OUT = os.environ.get("SHOT_DIR", os.path.join(os.environ.get("TEMP", "C:/Temp"), "fabric_only_shots"))
os.makedirs(OUT, exist_ok=True)
CUBE = "/World/Cube"
STEPS = 20


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def usd_z():
    stage = omni.usd.get_context().get_stage()
    return UsdGeom.Xformable(stage.GetPrimAtPath(CUBE)).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]


def fabric_z():
    import usdrt

    rt = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    attr = rt.GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    return float(attr.Get()[3][2]) if attr and attr.HasValue() else None


async def shot(name):
    path = os.path.join(OUT, name)
    cap = capture_viewport_to_file(get_active_viewport(), path)
    await cap.wait_for_result()
    return path


async def build_stage():
    omni.usd.get_context().new_stage()
    await frames(5)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    cube = UsdGeom.Cube.Define(stage, CUBE)
    cube.CreateSizeAttr(0.5)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    from pxr import UsdLux

    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(500)
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[4.0, 4.0, 1.5], target=[0.0, 0.0, 1.0])
    carb.settings.get_settings().set_int(pxb.SETTING_DISPLAY_COLLIDERS, 2)  # 2 = All
    await frames(20)


async def run(mode):
    await build_stage()
    pom.set_output_mode(mode)
    await frames(10)
    rows = []
    rows.append(("再生前", usd_z(), fabric_z(), await shot(f"{mode}_0_before.png")))

    hits = {"n": 0}

    def on_changed(notice, sender):
        for p in list(notice.GetChangedInfoOnlyPaths()) + list(notice.GetResyncedPaths()):
            if str(p).startswith(CUBE):
                hits["n"] += 1

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_changed, omni.usd.get_context().get_stage())
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(STEPS)
    playing_hits = hits["n"]
    rows.append(("再生中", usd_z(), fabric_z(), await shot(f"{mode}_1_playing.png")))
    tl.pause()
    await frames(10)
    rows.append(("一時停止中", usd_z(), fabric_z(), await shot(f"{mode}_2_paused.png")))
    listener.Revoke()
    tl.stop()
    await frames(10)
    rows.append(("停止後", usd_z(), fabric_z(), await shot(f"{mode}_3_stopped.png")))
    return mode, playing_hits, rows


async def main():
    results = [await run("usd"), await run("fabric_cpu")]
    print("\n===== RESULT =====")
    for mode, n, rows in results:
        print(f"[{mode}] 再生中に立方体の USD 変更通知が来た回数: {n}")
        for label, uz, fz, path in rows:
            fz_s = "None" if fz is None else f"{fz:.3f}"
            print(f"  {label:<6} USD z={uz:.3f}  Fabric z={fz_s}  画像={os.path.basename(path)}")
    print("===== END =====")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
