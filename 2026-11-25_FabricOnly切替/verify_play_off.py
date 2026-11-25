"""Play OFF（タイムライン停止中）に Fabric へ直接位置を書いたとき、描画・Collider 表示・USD・PhysX がどうなるかを実測する。

種類: dynamic（動的剛体）/ static（静的 Collider）/ kinematic（Kinematic 剛体）
出力先: usd / fabric_cpu（physics_output_mode.set_output_mode）

手順（各条件）:
1. 停止中に omni:fabric:worldMatrix の z を 2.0 → 1.0 に 1 回書く。30 フレーム待って記録・撮影（停止中）。
   この間に立方体のパスについて USD の変更通知が来た回数も数える。
2. Play を押し、5 フレーム後に記録・撮影（Play 直後）。PhysX がどちらの位置から始めたかが分かる。
3. Stop して 10 フレーム後に記録・撮影（Stop 後）。

記録: USD の z / Fabric の z / PhysX の z（真上からのレイキャストの当たり − 0.25。PhysX が無い停止中は None）
画像: Collider 表示 = All（線の色: 動的=緑、静的=マゼンタ、Kinematic=黄）

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_play_off.py
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


def physx_z():
    hit = omni.physx.get_physx_scene_query_interface().raycast_closest(
        carb.Float3(0.0, 0.0, 10.0), carb.Float3(0.0, 0.0, -1.0), 100.0
    )
    if not hit or not hit.get("hit"):
        return None
    return float(hit["position"][2]) - HALF


def write_fabric(z):
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
    if body != "static":
        rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rb.CreateKinematicEnabledAttr(body == "kinematic")
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[4.0, 4.0, 1.6], target=[0.0, 0.0, 1.4])
    carb.settings.get_settings().set_int(pxb.SETTING_DISPLAY_COLLIDERS, 2)  # 2 = All
    await frames(20)


async def run(body: BodyType, mode: pom.OutputMode):
    await build_stage(body)
    pom.set_output_mode(mode)
    await frames(10)
    tag = f"{body}_{mode}"
    rows = [("停止中・書く前", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_0_before.png"))]

    hits = {"n": 0}

    def on_changed(notice, sender):
        for p in list(notice.GetChangedInfoOnlyPaths()) + list(notice.GetResyncedPaths()):
            if str(p).startswith(CUBE):
                hits["n"] += 1

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_changed, stage())
    write_fabric(Z_FABRIC)
    await frames(30)
    notices = hits["n"]
    listener.Revoke()
    rows.append(("停止中・書いた後", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_1_stopped_written.png")))

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(5)  # 1 フレーム後だと描画が 1 フレーム遅れて画像と数値が合わないため 5 フレーム待つ
    rows.append(("Play直後", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_2_play_start.png")))
    tl.stop()
    await frames(10)
    rows.append(("Stop後", usd_z(), fabric_z(), physx_z(), await shot(f"{tag}_3_stopped.png")))
    return tag, notices, rows


def fmt(v):
    return "None" if v is None else f"{v:.3f}"


async def main():
    results = []
    for body in ("dynamic", "static", "kinematic"):
        for mode in ("usd", "fabric_cpu"):
            results.append(await run(body, mode))
    print("\n===== RESULT =====")
    for tag, notices, rows in results:
        print(f"[{tag}] 停止中に Fabric へ書いた後 30 フレームの USD 変更通知 {notices} 回")
        for label, uz, fz, pz, path in rows:
            print(f"  {label} USD z={fmt(uz)}  Fabric z={fmt(fz)}  PhysX z={fmt(pz)}")
    print("===== END =====")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
