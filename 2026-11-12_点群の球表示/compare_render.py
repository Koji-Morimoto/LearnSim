"""点群の球表示: 描き方 × 設定を変えて撮り比べ、ふちの拡大図とシャープさの数値を出す。

実行(PowerShell なら先に $env:PYTHONPATH=""; $env:PYTHONUTF8="1"):
    C:\\isaacsim-6.2.0\\python.bat compare_render.py --no-ros-env
出力: media/
    full_<id>.png      撮った絵そのまま(960x720)
    compare_grid.png   各設定のふちを 3 倍に拡大して並べた比較図(見出し・数値つき)
    shadow_compare.png 影のあり/なし(点群ごと・描画全体)の比較
    before_after.png   既定(DLSS Performance) と おすすめ設定 の全体比較
    metrics.txt        各設定のシャープさの数値
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, r"C:/isaacsim-6.2.0/skills/isaac-sim-visual-explainer/scripts")

from explainer import Explainer  # noqa: E402

OUT = os.path.join(HERE, "media")
ex = Explainer(out_dir=OUT, name="pointcloud", render_size=(960, 720))
app = ex.start_app()

import carb.settings  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import pointcloud_spheres as pcs  # noqa: E402

S = carb.settings.get_settings()
DEFAULTS = {k: S.get(k) for k in ["/rtx/post/aa/op", "/rtx/post/dlss/execMode", "/rtx/shadows/enabled", "/rtx/ambientOcclusion/enabled"]}
print("defaults:", DEFAULTS, "rendermode:", S.get("/rtx/rendermode"))

# ----------------------------------------------------------------------------- ステージ
pts, cols = pcs.demo_cloud(1500)
R = 0.008
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(350)
dome.CreateColorAttr(Gf.Vec3f(0.55, 0.57, 0.6))  # 背景(灰色)
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(1200)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 20))

pcs.add_points_prim(stage, "/World/Points", pts, cols, radius=R)
pcs.add_sphere_cloud(stage, "/World/Inst", pts, cols, radius=R)
pcs.add_sphere_cloud(stage, "/World/InstFlat", pts, cols, radius=R, flat=True)
pcs.add_outline_shell(stage, "/World/Points", pts, radius=R, thickness=0.25)  # -> /World/Points_Outline
# 影の比較用: 地面と、影を落とし続ける立方体(点群だけ影を消したことが分かるように)
ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
ground.CreatePointsAttr([(-0.6, -0.4, 0.1), (0.6, -0.4, 0.1), (0.6, 0.6, 0.1), (-0.6, 0.6, 0.1)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
ground.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.8, 0.8)])
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(0.04)
cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.3, 0.3)])
UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.33, -0.06, 0.16))
UsdGeom.Xformable(cube.GetPrim()).AddScaleOp().Set(Gf.Vec3f(1, 1, 3))  # 柱にして影を長くする
ALL = ["/World/Points", "/World/Inst", "/World/InstFlat", "/World/Points_Outline", "/World/Ground", "/World/Cube"]

ex.add_camera(eye=(0.0, -0.55, 0.55), target=(0.0, 0.0, 0.25))
for _ in range(30):
    app.update()


def show(paths):
    for p in ALL:
        img = UsdGeom.Imageable(stage.GetPrimAtPath(p))
        (img.MakeVisible if p in paths else img.MakeInvisible)()


def set_render(aa=3, exec_mode=0, flat_look=False):
    S.set_int("/rtx/post/aa/op", aa)
    S.set_int("/rtx/post/dlss/execMode", exec_mode)
    pcs.set_shadows(stage, not flat_look)
    pcs.set_ambient_occlusion(not flat_look)


# (id, 見出し, 説明, 表示する prim, aa, execMode, flat_look)
CASES = [
    ("A", "既定: DLSS Performance", "Isaac Sim の既定。半分の解像度で描いて拡大", ["/World/Points"], 3, 0, False),
    ("B", "DLSS Quality", "拡大率を下げる(約 2/3 の解像度)", ["/World/Points"], 3, 2, False),
    ("C", "DLAA", "拡大なし。元の解像度で描く", ["/World/Points"], 4, 0, False),
    ("D", "DLAA + 影/AO なし", "重なった球の境目がかえって薄くなる", ["/World/Points"], 4, 0, True),
    ("E", "DLAA + 黒い縁取り", "裏返した殻で球ごとに輪郭線", ["/World/Points", "/World/Points_Outline"], 4, 0, False),
    ("F", "DLAA + べた塗り + 縁取り", "自己発光の材質で陰影なし(図の見た目)", ["/World/InstFlat", "/World/Points_Outline"], 4, 0, True),
    ("G", "PointInstancer + DLAA", "本物の球メッシュ。C と同じ見た目か確認", ["/World/Inst"], 4, 0, False),
]

CROP = (440, 300, 600, 420)  # 拡大して見る範囲 (x0, y0, x1, y1)
ZOOM = 3


def sharpness(img: np.ndarray) -> float:
    """ふちのシャープさ: 輝度の勾配の大きい方から 1% の平均。大きいほど境目が急(くっきり)。"""
    y = img[..., :3].astype(np.float32) @ np.array([0.299, 0.587, 0.114], np.float32)
    gx = np.abs(np.diff(y, axis=1))[:-1, :]
    gy = np.abs(np.diff(y, axis=0))[:, :-1]
    g = np.hypot(gx, gy).ravel()
    k = max(1, g.size // 100)
    return float(np.sort(g)[-k:].mean())


shots = {}
lines = []
for cid, title, desc, paths, aa, em, flat in CASES:
    show(paths)
    set_render(aa, em, flat)
    img = ex.capture(settle=30)  # DLSS / DLAA は数フレームかけて積み上げるので 30 回描いてから使う
    Image.fromarray(img).save(os.path.join(OUT, f"full_{cid}.png"))
    crop = img[CROP[1]:CROP[3], CROP[0]:CROP[2]]
    sh = sharpness(crop)
    shots[cid] = (img, sh)
    lines.append(f"{cid}\t{title}\tsharpness={sh:.1f}\tmean={img.mean():.1f}")
    print(lines[-1])

# 画面の選択枠(selection outline)が replicator のカメラ画像に写るかを確かめる
show(["/World/Points"])
set_render(4, 0, False)
pcs.add_selection_outline(["/World/Points"], color=(1.0, 0.5, 0.0, 1.0), width=3)
img_sel = ex.capture(settle=30)
Image.fromarray(img_sel).save(os.path.join(OUT, "full_selection_outline.png"))
diff = float(np.abs(img_sel.astype(np.int32) - shots["C"][0].astype(np.int32)).mean())
orange = int(((img_sel[..., 0] > 200) & (img_sel[..., 1] > 90) & (img_sel[..., 1] < 160) & (img_sel[..., 2] < 60)).sum())
lines.append(f"selection_outline\tmean_abs_diff_vs_C={diff:.2f}\torange_pixels={orange}")
print(lines[-1])

# ----------------------------------------------------------------------------- 影のあり/なし
GROUND_BOX = (60, 560, 900, 720)  # 地面に影が落ちる辺り(明るさを測る)
SHADOW_CASES = [
    ("S1", "影あり(既定)", "点群も立方体も影を落とす", ["/World/Points", "/World/Ground", "/World/Cube"], True, True),
    ("S2", "点群だけ影なし", "set_cast_shadows(stage, 点群, False)。立方体の影は残る", ["/World/Points", "/World/Ground", "/World/Cube"], False, True),
    ("S3", "描画全体で影なし", "set_shadows(stage, False)。立方体の影も消える", ["/World/Points", "/World/Ground", "/World/Cube"], True, False),
    ("S3x", "(確認)光源の ShadowAPI だけ切る", "inputs:shadow:enable=False と /rtx/shadows/enabled=False", ["/World/Points", "/World/Ground", "/World/Cube"], True, "light"),
    ("S4", "PointInstancer で点群だけ影なし", "add_sphere_cloud(..., cast_shadows=False) と同じ", ["/World/Inst", "/World/Ground", "/World/Cube"], False, True),
]
shadow_shots = {}
for sid, title, desc, paths, cloud_cast, global_on in SHADOW_CASES:
    show(paths)
    set_render(4, 0, False)
    pcs.set_cast_shadows(stage, "/World/Points", cloud_cast)  # 点群ごとの設定を先に。全体の設定(下)はその後で上書きする
    pcs.set_cast_shadows(stage, "/World/Inst", cloud_cast)
    if global_on == "light":  # 光源側だけで切る(RealTimePathTracing で効くかの確認)
        S.set_bool("/rtx/shadows/enabled", False)
        for lp in ["/World/Sun", "/World/Dome"]:
            UsdLux.ShadowAPI.Apply(stage.GetPrimAtPath(lp)).CreateShadowEnableAttr().Set(False)
        print("  light shadow:enable =", [stage.GetPrimAtPath(lp).GetAttribute("inputs:shadow:enable").Get() for lp in ["/World/Sun", "/World/Dome"]])
    elif not global_on:
        pcs.set_shadows(stage, False)
    img = ex.capture(settle=30)
    Image.fromarray(img).save(os.path.join(OUT, f"full_{sid}.png"))
    g = img[GROUND_BOX[1]:GROUND_BOX[3], GROUND_BOX[0]:GROUND_BOX[2]].mean()
    shadow_shots[sid] = (img, g)
    lines.append(f"{sid}	{title}	ground_mean={g:.1f}")
    print(lines[-1])
pcs.set_cast_shadows(stage, "/World/Points", True)
pcs.set_cast_shadows(stage, "/World/Inst", True)
pcs.set_shadows(stage, True)

with open(os.path.join(OUT, "metrics.txt"), "w", encoding="utf-8") as f:
    f.write("defaults: " + repr(DEFAULTS) + "\n" + "\n".join(lines) + "\n")

# ----------------------------------------------------------------------------- 比較図
FONT_T, FONT_S = ex.font(26), ex.font(18)
cw, ch = (CROP[2] - CROP[0]) * ZOOM, (CROP[3] - CROP[1]) * ZOOM
cols_n = 4
rows_n = (len(CASES) + cols_n - 1) // cols_n
pad, head = 16, 92
grid = Image.new("RGB", (pad + cols_n * (cw + pad), 70 + rows_n * (ch + head + pad)), (24, 26, 32))
d = ImageDraw.Draw(grid)
d.text((pad, 18), f"球のふちを {ZOOM} 倍に拡大(最近傍。にじみ = 色が段階的に変わる画素の帯)", font=FONT_T, fill=(235, 235, 235))
for i, (cid, title, desc, *_rest) in enumerate(CASES):
    img, sh = shots[cid]
    x = pad + (i % cols_n) * (cw + pad)
    y = 70 + (i // cols_n) * (ch + head + pad)
    d.text((x, y), f"{cid}. {title}", font=FONT_T, fill=(255, 196, 64))
    d.text((x, y + 34), desc, font=FONT_S, fill=(200, 205, 215))
    d.text((x, y + 60), f"シャープさ {sh:.0f}", font=FONT_S, fill=(160, 166, 178))
    crop = Image.fromarray(img[CROP[1]:CROP[3], CROP[0]:CROP[2]]).resize((cw, ch), Image.NEAREST)
    grid.paste(crop, (x, y + head))
grid.save(os.path.join(OUT, "compare_grid.png"))

# 既定 vs おすすめ(全体)
ba = Image.new("RGB", (960 * 2 + 3 * pad, 720 + 80), (24, 26, 32))
d = ImageDraw.Draw(ba)
for j, (cid, label) in enumerate([("A", "A. 既定(DLSS Performance)"), ("E", "E. DLAA + 黒い縁取り")]):
    ba.paste(Image.fromarray(shots[cid][0]), (pad + j * (960 + pad), 64))
    d.text((pad + j * (960 + pad), 18), label, font=FONT_T, fill=(255, 196, 64))
    d.rectangle([pad + j * (960 + pad) + CROP[0], 64 + CROP[1], pad + j * (960 + pad) + CROP[2], 64 + CROP[3]], outline=(255, 80, 80), width=2)
ba.save(os.path.join(OUT, "before_after.png"))

# 影の比較図(2x2、各 640x480 に縮小)
tw, th = 640, 480
sc = Image.new("RGB", (pad + 2 * (tw + pad), 70 + 2 * (th + 70 + pad)), (24, 26, 32))
d = ImageDraw.Draw(sc)
d.text((pad, 18), "影のあり/なし(点群ごと と 描画全体)", font=FONT_T, fill=(235, 235, 235))
for i, (sid, title, desc, *_r) in enumerate([c for c in SHADOW_CASES if c[0] != "S3x"]):
    x = pad + (i % 2) * (tw + pad)
    y = 70 + (i // 2) * (th + 70 + pad)
    d.text((x, y), f"{sid}. {title}", font=FONT_T, fill=(255, 196, 64))
    d.text((x, y + 34), desc, font=FONT_S, fill=(200, 205, 215))
    sc.paste(Image.fromarray(shadow_shots[sid][0]).resize((tw, th), Image.LANCZOS), (x, y + 70))
sc.save(os.path.join(OUT, "shadow_compare.png"))

# 設定を元に戻す
for k, v in DEFAULTS.items():
    if isinstance(v, bool):
        S.set_bool(k, v)
    elif v is not None:
        S.set_int(k, v)
app.close()
