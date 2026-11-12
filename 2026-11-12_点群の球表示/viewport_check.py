"""ウィンドウありの Isaac Sim(GUI のビューポート)で、同じ設定が効くかを確かめる。

確かめること:
    1. 選択枠(selection outline)の仕組みで点群に縁取りが付くか(ビューポートでのみ写るはず)
    2. 裏返した殻の縁取り(add_outline_shell)と DLAA がビューポートでも効くか
実行: C:\\isaacsim-6.2.0\\python.bat viewport_check.py --no-ros-env   (ウィンドウが開く。最小化しないこと)
出力: media/viewport_<id>.png(アプリ画面全体)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, r"C:/isaacsim-6.2.0/skills/isaac-sim-visual-explainer/scripts")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": False, "width": 1280, "height": 800})

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402
from explainer import capture_app_window  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import pointcloud_spheres as pcs  # noqa: E402

OUT = os.path.join(HERE, "media")
TMP = os.path.join(os.environ.get("TEMP", "C:/Temp"), "pc_viewport")  # Kit の画面撮影は日本語を含むパスに保存できない
os.makedirs(TMP, exist_ok=True)


def grab(name):
    """アプリ画面全体を ASCII のパスに撮ってから media に移す。"""
    import shutil

    tmp = os.path.join(TMP, name)
    if os.path.exists(tmp):
        os.remove(tmp)
    ok = capture_app_window(app, tmp)
    if ok:
        shutil.move(tmp, os.path.join(OUT, name))
    return ok
S = carb.settings.get_settings()
pts, cols = pcs.demo_cloud(1500)
R = 0.008

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(350)
dome.CreateColorAttr(Gf.Vec3f(0.55, 0.57, 0.6))
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(1200)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 20))
pcs.add_points_prim(stage, "/World/Points", pts, cols, radius=R)

cam = UsdGeom.Camera.Define(stage, "/World/Cam")
view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, -0.55, 0.55), Gf.Vec3d(0, 0, 0.25), Gf.Vec3d(0, 0, 1))
UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(view.GetInverse())
cam.CreateFocalLengthAttr(24.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
get_active_viewport().camera_path = "/World/Cam"


def settle(n=120):
    for _ in range(n):
        app.update()


settle(200)
print("viewport aa/op =", S.get("/rtx/post/aa/op"), "execMode =", S.get("/rtx/post/dlss/execMode"))
ok = grab("viewport_1_default.png")
print("1 default", ok)

pcs.set_crisp_render("DLAA")
gid = pcs.add_selection_outline(["/World/Points"], color=(1.0, 0.5, 0.0, 1.0), width=3)
settle()
print("2 selection outline gid", gid, grab("viewport_2_selection_outline.png"))

omni.usd.get_context().set_selection_group(0, "/World/Points")  # 選択枠を外す
pcs.add_outline_shell(stage, "/World/Points", pts, radius=R, thickness=0.25)
settle()
print("3 shell", grab("viewport_3_dlaa_shell.png"))
app.close()
