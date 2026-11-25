import sys
from PIL import Image, ImageDraw, ImageFont
sd = sys.argv[1]; out = sys.argv[2]
font = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 26)
small = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 22)
cols = [("0_before", "再生前"), ("1_playing", "再生中"), ("2_paused", "一時停止中")]
rows = [("usd", "USD に書き戻す（既定）", ["USD z=2.00", "USD z=1.31", "USD z=1.11"]),
        ("fabric_cpu", "Fabric のみ（USD 書き戻し OFF）", ["USD z=2.00", "USD z=2.00（動かない）", "USD z=2.00（動かない）"])]
W, H = 640, 360
top, left, lab = 50, 0, 40
img = Image.new("RGB", (W * 3, top + (H + lab) * 2), (30, 30, 30))
d = ImageDraw.Draw(img)
for c, (_, t) in enumerate(cols):
    d.text((c * W + W // 2, 25), t, font=font, fill=(255, 255, 255), anchor="mm")
for r, (mode, title, zs) in enumerate(rows):
    y0 = top + r * (H + lab)
    d.text((10, y0 + 6), title, font=font, fill=(255, 220, 120))
    for c, (key, _) in enumerate(cols):
        im = Image.open(f"{sd}/{mode}_{key}.png").convert("RGB").resize((W, H))
        img.paste(im, (c * W, y0 + lab))
        d.text((c * W + 10, y0 + lab + H - 34), zs[c] + "  緑の線 = Collider 表示", font=small, fill=(255, 255, 255))
img.save(out)
print(out, img.size)
