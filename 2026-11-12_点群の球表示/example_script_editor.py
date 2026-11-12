"""Isaac Sim の Script Editor(Window > Script Editor)に貼って実行する例。

作成日: 2026-11-12
開いているステージに点群を球で置き、ふちをくっきりさせる。下の「設定」を書き換えて使う。
自分の点群は POINTS / COLORS を (N, 3) の numpy 配列に置き換える(例: np.load("cloud.npy"))。
"""

import importlib
import sys

import numpy as np
import omni.usd

sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\2026-11-12_点群の球表示")
import pointcloud_spheres as pcs  # noqa: E402

importlib.reload(pcs)  # 関数を書き換えたときに読み直す

# ----------------------------------------------------------------------------- 設定
PATH = "/World/PointCloud"
RADIUS = 0.008  # 球の半径 [m]
USE_INSTANCER = False  # False: UsdGeom.Points(軽い・色は点ごと) / True: PointInstancer(本物の球メッシュ・色は丸める)
FLAT = False  # True: べた塗り(陰影なし。USE_INSTANCER=True のときだけ)
OUTLINE = True  # 球ごとの黒い縁取り
OUTLINE_THICKNESS = 0.2  # 縁取りの太さ(半径に対する比)
CAST_SHADOWS = True  # この点群が影を落とすか(後から pcs.set_cast_shadows で切り替え可)
CRISP = True  # DLAA にする(にじみ対策の本命)
POINTS, COLORS = pcs.demo_cloud(1500)  # ← 自分の点群に置き換える

# ----------------------------------------------------------------------------- 実行
stage = omni.usd.get_context().get_stage()
for p in (PATH, PATH + "_Outline"):  # 作り直すときは前のものを消す
    if stage.GetPrimAtPath(p):
        stage.RemovePrim(p)

if USE_INSTANCER:
    pcs.add_sphere_cloud(stage, PATH, POINTS, COLORS, radius=RADIUS, flat=FLAT, cast_shadows=CAST_SHADOWS)
else:
    pcs.add_points_prim(stage, PATH, POINTS, COLORS, radius=RADIUS, cast_shadows=CAST_SHADOWS)
if OUTLINE:
    pcs.add_outline_shell(stage, PATH, POINTS, radius=RADIUS, thickness=OUTLINE_THICKNESS)
if CRISP:
    pcs.set_crisp_render("DLAA")

# 影の切り替え(後からこの行だけ実行してもよい):
# pcs.set_cast_shadows(stage, PATH, False)   # この点群だけ影なし
# pcs.set_shadows(stage, False)              # ステージ全体で影なし(True で戻す)
