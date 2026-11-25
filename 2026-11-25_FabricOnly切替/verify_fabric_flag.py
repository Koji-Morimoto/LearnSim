"""Show By Type の Collider 表示が USD と Fabric のどちらの位置を見るかを、USD書き戻しと fabricEnabled の組み合わせごとに実測する。

条件（すべて同じ手順・同じ形式で撮る）:
- "writeback_on_flag_off"            : USD書き戻しON  ＋ Fabric 拡張 無効（fabricEnabled=False）。GUI の USD ボタンと同じ
- "writeback_off_flag_on"            : USD書き戻しOFF ＋ Fabric 拡張 有効（fabricEnabled=True）。GUI の Fabric CPU ボタンと同じ
- "writeback_off_flag_on_fabric_gpu" : 上と同じで GPU 物理のシーン＋fabricUseGPUInterop=True。GUI の Fabric GPU ボタンと同じ
- "writeback_off_flag_off"           : USD書き戻しOFF ＋ Fabric 拡張 無効（fabricEnabled=False）
- "writeback_on_flag_on"             : USD書き戻しON  ＋ Fabric 拡張 有効（fabricEnabled=True）
fabricEnabled は omni.physx.fabric 拡張を有効にすると True、無効にすると False になる（拡張自身が設定する）。

手順:
- Play OFF: 停止中に Fabric にだけ位置を書いて撮る。
- Play ON: 動的剛体はそのまま落とす。静的 Collider と Kinematic は Fabric にだけ位置を書いて動かす。再生中と一時停止中に撮る。

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_fabric_flag.py
画像は環境変数 SHOT_DIR（英数字だけのパス）に出す。画像にするのは compose_collider_writeback.py。
"""

import asyncio
import os
from typing import Literal, get_args

from isaacsim import SimulationApp

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
app = SimulationApp(
    {"headless": True, "hide_ui": False, "width": 960, "height": 540},
    experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"),
)

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx.bindings._physx as pxb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

BodyType = Literal["dynamic", "static", "kinematic"]
Config = Literal[
    "writeback_on_flag_off",
    "writeback_off_flag_on",
    "writeback_off_flag_on_fabric_gpu",
    "writeback_off_flag_off",
    "writeback_on_flag_on",
]

OUT = os.environ.get("SHOT_DIR", os.path.join(os.environ.get("TEMP", "C:/Temp"), "fabric_only_shots"))
os.makedirs(OUT, exist_ok=True)
CUBE = "/World/Cube"
HALF = 0.25
Z_START, Z_FABRIC = 2.0, 1.0
FABRIC_EXT = "omni.physx.fabric"


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def settings():
    return carb.settings.get_settings()


def ext_manager():
    return omni.kit.app.get_app().get_extension_manager()


def apply_config(config: Config):
    """Fabric 拡張の有効/無効を先に決め（これで fabricEnabled が自動で変わる）、そのあと updateToUsd を書く。"""
    use_fabric_ext = "_flag_on" in config
    ext_manager().set_extension_enabled_immediate(FABRIC_EXT, use_fabric_ext)
    flag_after_ext = settings().get("/physics/fabricEnabled")
    writeback = config.startswith("writeback_on")
    settings().set_bool("/physics/updateToUsd", writeback)
    settings().set_bool("/physics/updateVelocitiesToUsd", writeback)
    settings().set_bool("/physics/fabricUseGPUInterop", config.endswith("_fabric_gpu"))
    return flag_after_ext


def rt_stage():
    return usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())


def write_fabric(z):
    attr = rt_stage().GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    m = usdrt.Gf.Matrix4d(1.0)
    m.SetTranslateOnly(usdrt.Gf.Vec3d(0.0, 0.0, z))
    attr.Set(m)


Z_VALUES = {}  # 画像名 -> {"mesh_z", "collider_z", "usd_z", "fabric_z"}。最後に OUT/z_values.json に保存する


def usd_z():
    s = omni.usd.get_context().get_stage()
    return UsdGeom.Xformable(s.GetPrimAtPath(CUBE)).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]


def fabric_z():
    attr = rt_stage().GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    return float(attr.Get()[3][2]) if attr and attr.HasValue() else None


async def shot(name, playing=False):
    """撮影し、Mesh（描画 = Fabric を読む）と Collider 表示の z を記録する。
    Collider 表示の z は omni.physx.ui の条件（Play 中かつ fabricEnabled=True なら Fabric、それ以外は USD）で選ぶ。
    """
    await capture_viewport_to_file(get_active_viewport(), os.path.join(OUT, name)).wait_for_result()
    uz, fz = usd_z(), fabric_z()
    collider_reads_fabric = playing and bool(settings().get("/physics/fabricEnabled"))
    Z_VALUES[name] = {"mesh_z": fz, "collider_z": fz if collider_reads_fabric else uz, "usd_z": uz, "fabric_z": fz}


async def build_stage(body: BodyType, config: Config):
    omni.usd.get_context().new_stage()
    await frames(5)
    s = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    scene = UsdPhysics.Scene.Define(s, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    if config.endswith("_fabric_gpu"):
        from pxr import PhysxSchema

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
    settings().set_int(pxb.SETTING_DISPLAY_COLLIDERS, 2)  # 2 = All
    await frames(20)
    flag = apply_config(config)
    await frames(10)
    return flag


async def run(body: BodyType, config: Config):
    flag = await build_stage(body, config)
    await shot(f"{config}_{body}_off_0_before.png")
    write_fabric(Z_FABRIC)
    await frames(30)
    await shot(f"{config}_{body}_off_1_written.png")

    await build_stage(body, config)
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await frames(5)
    for i in range(1, 21):
        if body != "dynamic":
            write_fabric(Z_START + (Z_FABRIC - Z_START) * i / 20)
        await frames(1)
    await frames(5)
    await shot(f"{config}_{body}_on_1_playing.png", playing=True)
    tl.pause()
    await frames(10)
    await shot(f"{config}_{body}_on_2_paused.png", playing=True)  # Pause でも Play 中扱い（Stop まで）
    tl.stop()
    await frames(10)
    state = (
        flag,
        settings().get("/physics/fabricEnabled"),
        settings().get("/physics/updateToUsd"),
        ext_manager().is_extension_enabled(FABRIC_EXT),
    )
    return state


async def main():
    initial_colliders = settings().get(pxb.SETTING_DISPLAY_COLLIDERS)
    print("起動直後: omni.physx.fabric 有効 =", ext_manager().is_extension_enabled(FABRIC_EXT),
          " fabricEnabled =", settings().get("/physics/fabricEnabled"))
    lines = []
    for config in get_args(Config):
        for body in get_args(BodyType):
            flag, fe, utu, ext = await run(body, config)
            lines.append(f"[{config} {body}] 拡張切替直後の fabricEnabled={flag}  終了時 fabricEnabled={fe} updateToUsd={utu} 拡張有効={ext}")
    print("\n===== RESULT =====")
    print("\n".join(lines))
    print("===== END =====")
    import json

    with open(os.path.join(OUT, "z_values.json"), "w", encoding="utf-8") as f:
        json.dump(Z_VALUES, f, ensure_ascii=False, indent=1)
    settings().set(pxb.SETTING_DISPLAY_COLLIDERS, initial_colliders)  # 保存される設定なので初期値に戻す
    await frames(5)


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
app.close()
