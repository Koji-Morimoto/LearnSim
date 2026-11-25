import sys, re
from PIL import Image, ImageDraw, ImageFont
sd, out, log = sys.argv[1], sys.argv[2], sys.argv[3]
font = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 22)
small = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 17)
# ログから数値を読む
vals = {}
cur = None
for line in open(log, encoding="utf-8", errors="replace"):
    m = re.match(r"\[(\S+)\] ", line)
    if m:
        cur = m.group(1); vals[cur] = {}
        continue
    m = re.match(r"\s+(\S+)\s+USD z=(\S+)\s+Fabric z=(\S+)\s+PhysX z=(\S+)", line)
    if m and cur:
        vals[cur][m.group(1)] = m.group(2, 3, 4)
cols = [("1_playing", "再生中"), ("2_paused", "一時停止中"), ("3_stopped", "停止後")]
names = {"static": "静的 Collider", "kinematic": "Kinematic", "usd": "USD", "fabric_cpu": "Fabric CPU"}
rows = []
for body in ("static", "kinematic"):
    for method in ("usd", "fabric"):
        for mode in ("usd", "fabric_cpu"):
            rows.append((f"{body}_{mode}_{method}",
                         f"{names[body]} ／ 出力先 {names[mode]} ／ 動かし方: {'USD の xform に書く' if method=='usd' else 'Fabric の worldMatrix に書く'}"))
W, H, lab, top = 480, 270, 32, 40
img = Image.new("RGB", (W * 3, top + (H + lab) * len(rows)), (30, 30, 30))
d = ImageDraw.Draw(img)
for c, (_, t) in enumerate(cols):
    d.text((c * W + W // 2, 20), t, font=font, fill=(255, 255, 255), anchor="mm")
for r, (tag, title) in enumerate(rows):
    y0 = top + r * (H + lab)
    d.text((8, y0 + 4), title, font=font, fill=(255, 220, 120))
    for c, (key, label) in enumerate(cols):
        im = Image.open(f"{sd}/{tag}_{key}.png").convert("RGB").resize((W, H))
        img.paste(im, (c * W, y0 + lab))
        u, f, p = vals[tag][label]
        d.rectangle((c * W, y0 + lab + H - 26, c * W + W, y0 + lab + H), fill=(0, 0, 0))
        d.text((c * W + 6, y0 + lab + H - 24), f"USD z={u}  Fabric z={f}  PhysX z={p}", font=small, fill=(255, 255, 255))
img.save(out)
print(out, img.size)
