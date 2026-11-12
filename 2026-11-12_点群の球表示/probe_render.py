"""点群の描き方 × アンチエイリアスの設定を変えて撮り比べる(下調べ用)。

実行: set PYTHONPATH= & set PYTHONUTF8=1 & C:\\isaacsim-6.2.0\\python.bat probe_render.py --no-ros-env
出力: media/probe/<variant>.png(撮った絵そのまま)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, r"C:/isaacsim-6.2.0/skills/isaac-sim-visual-explainer/scripts")

from explainer import Explainer  # noqa: E402

OUT = os.path.join(HERE, "media", "probe")
ex = Explainer(out_dir=OUT, name="probe", render_size=(960, 720))
app = ex.start_app()

import carb.settings  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import pointcloud_spheres as pcs  # noqa: E402

S = carb.settings.get_settings()
print("default aa/op =", S.get("/rtx/post/aa/op"), " dlss execMode =", S.get("/rtx/post/dlss/execMode"))

pts, cols = pcs.demo_cloud(1500)
R = 0.008


def new_scene(kind: str):
    omni.usd.get_context().new_stage()
    for _ in range(5):
        app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(600)
    dome.CreateColorAttr(Gf.Vec3f(0.55, 0.57, 0.6))  # 背景も灰色になる
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 20))
    if kind == "points":
        pcs.add_points_prim(stage, "/World/Cloud", pts, cols, radius=R)
    elif kind == "inst":
        pcs.add_sphere_cloud(stage, "/World/Cloud", pts, cols, radius=R)
    elif kind == "inst_flat":
        pcs.add_sphere_cloud(stage, "/World/Cloud", pts, cols, radius=R, flat=True)
    elif kind == "inst_shell":
        pcs.add_sphere_cloud(stage, "/World/Cloud", pts, cols, radius=R)
        pcs.add_outline_shell(stage, "/World/Cloud", pts, radius=R)
    ex.add_camera(eye=(0.0, -0.55, 0.55), target=(0.0, 0.0, 0.25))
    for _ in range(30):
        app.update()


def shot(name: str, aa: int, exec_mode: int = 0, frames: int = 24):
    S.set_int("/rtx/post/aa/op", aa)
    S.set_int("/rtx/post/dlss/execMode", exec_mode)
    img = ex.capture(settle=frames)  # DLSS / DLAA は時間方向に積み上げるので何枚か描いてから使う
    Image.fromarray(img).save(os.path.join(OUT, name + ".png"))
    print("saved", name, img.shape, img.mean())


for kind in ["points", "inst", "inst_flat", "inst_shell"]:
    new_scene(kind)
    shot(f"{kind}_dlss_perf", 3, 0)
    shot(f"{kind}_dlss_quality", 3, 2)
    shot(f"{kind}_dlaa", 4)

app.close()
