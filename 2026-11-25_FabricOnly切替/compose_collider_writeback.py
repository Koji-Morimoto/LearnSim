"""verify_fabric_flag.py の撮影画像から、条件ごとの見比べ画像を 1 枚ずつ（同じ形式で）作る。

行 = 動的剛体 / 静的 Collider / Kinematic、列 = Play OFF・書く前 / Play OFF・Fabric に書いた後 / Play ON・再生中 / Play ON・一時停止中。

実行: python.bat compose_collider_writeback.py <撮影画像のフォルダ> <出力フォルダ>
"""

import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

shot_dir, out_dir = sys.argv[1], sys.argv[2]
FONT = "C:/Windows/Fonts/meiryo.ttc"
title_font = ImageFont.truetype(FONT, 26)
row_font = ImageFont.truetype(FONT, 22)
col_font = ImageFont.truetype(FONT, 19)

MODES = {
    "writeback_on_flag_off": (
        "USD書き戻しON ＋ Fabric 拡張 無効（fabricEnabled=False）",
        "collider_writeback_on_flag_off.png",
    ),
    "writeback_off_flag_on": (
        "USD書き戻しOFF ＋ Fabric 拡張 有効（fabricEnabled=True）",
        "collider_writeback_off_flag_on.png",
    ),
    "writeback_off_flag_on_fabric_gpu": (
        "USD書き戻しOFF ＋ Fabric 拡張 有効（fabricEnabled=True）／ Fabric GPU",
        "collider_writeback_off_flag_on_fabric_gpu.png",
    ),
    "writeback_off_flag_off": (
        "USD書き戻しOFF ＋ Fabric 拡張 無効（fabricEnabled=False）",
        "collider_writeback_off_flag_off.png",
    ),
    "writeback_on_flag_on": (
        "USD書き戻しON の設定 ＋ Fabric 拡張 有効（fabricEnabled=True）→ 実際は USD に書き戻されない",
        "collider_writeback_on_flag_on.png",
    ),
}
COLS = [
    ("off_0_before", "Play OFF・書く前"),
    ("off_1_written", "Play OFF・Fabric にだけ位置を書いた後"),
    ("on_1_playing", "Play ON・再生中"),
    ("on_2_paused", "Play ON・一時停止中"),
]
ROWS = [("dynamic", "動的剛体（線: 緑）"), ("static", "静的 Collider（線: マゼンタ）"), ("kinematic", "Kinematic（線: 黄）")]
LINE_COLOR = {"dynamic": (0, 230, 0), "static": (255, 80, 255), "kinematic": (255, 230, 0)}
MESH_COLOR = (190, 190, 190)
z_font = ImageFont.truetype(FONT, 20)
with open(os.path.join(shot_dir, "z_values.json"), encoding="utf-8") as f:
    Z = json.load(f)
NOTE = "線 = Show By Type の Collider 表示　灰色の立方体 = Mesh（Fabric を読む描画）　各画像の下: Mesh z（灰）／ Collider z（線の色）"

W, H, ROW_LABEL, TOP = 400, 225, 32, 112
for mode, (title, filename) in MODES.items():
    if not os.path.exists(os.path.join(shot_dir, f"{mode}_dynamic_off_0_before.png")):
        continue  # このフォルダに撮影画像が無い組み合わせは飛ばす
    img = Image.new("RGB", (W * len(COLS), TOP + (H + ROW_LABEL) * len(ROWS)), (30, 30, 30))
    d = ImageDraw.Draw(img)
    d.text((10, 8), title, font=title_font, fill=(255, 220, 120))
    d.text((10, 44), NOTE, font=col_font, fill=(220, 220, 220))
    for c, (_, label) in enumerate(COLS):
        d.text((c * W + W // 2, TOP - 14), label, font=col_font, fill=(255, 255, 255), anchor="mm")
    for r, (body, row_label) in enumerate(ROWS):
        y0 = TOP + r * (H + ROW_LABEL)
        d.text((8, y0 + 4), row_label, font=row_font, fill=(255, 255, 255))
        for c, (key, _) in enumerate(COLS):
            im = Image.open(os.path.join(shot_dir, f"{mode}_{body}_{key}.png")).convert("RGB").resize((W, H))
            img.paste(im, (c * W, y0 + ROW_LABEL))
            z = Z[f"{mode}_{body}_{key}.png"]
            ybar = y0 + ROW_LABEL + H - 28
            d.rectangle((c * W, ybar, c * W + W, ybar + 28), fill=(20, 20, 20))
            mesh_txt = f"Mesh z={z['mesh_z']:.2f}"
            d.text((c * W + 8, ybar + 3), mesh_txt, font=z_font, fill=MESH_COLOR)
            x2 = c * W + 8 + d.textlength(mesh_txt, font=z_font) + 24
            d.text((x2, ybar + 3), f"Collider z={z['collider_z']:.2f}", font=z_font, fill=LINE_COLOR[body])
    path = os.path.join(out_dir, filename)
    img.save(path)
    print(path, img.size)
