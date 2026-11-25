import sys, re
from PIL import Image, ImageDraw, ImageFont
sd, out, log = sys.argv[1], sys.argv[2], sys.argv[3]
font = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 22)
small = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 15)
vals, cur = {}, None
for line in open(log, encoding="utf-8", errors="replace"):
    m = re.match(r"\[(\S+)\] ", line)
    if m:
        cur = m.group(1); vals[cur] = {}; continue
    m = re.match(r"\s+(\S+) USD z=(\S+)\s+Fabric z=(\S+)\s+PhysX z=(\S+)", line)
    if m and cur:
        vals[cur][m.group(1)] = m.group(2, 3, 4)
cols = [("0_before", "停止中・書く前"), ("1_stopped_written", "停止中・Fabric に z=1.0 を書いた後"), ("2_play_start", "Play直後"), ("3_stopped", "Stop後")]
names = {"dynamic": "動的剛体（緑）", "static": "静的 Collider（マゼンタ）", "kinematic": "Kinematic（黄）", "usd": "USD", "fabric_cpu": "Fabric CPU"}
rows = [(f"{b}_{m}", f"{names[b]} ／ 出力先 {names[m]}") for b in ("dynamic", "static", "kinematic") for m in ("usd", "fabric_cpu")]
W, H, lab, top = 400, 225, 32, 40
img = Image.new("RGB", (W * 4, top + (H + lab) * len(rows)), (30, 30, 30))
d = ImageDraw.Draw(img)
for c, (_, t) in enumerate(cols):
    d.text((c * W + W // 2, 20), t, font=ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 19), fill=(255, 255, 255), anchor="mm")
for r, (tag, title) in enumerate(rows):
    y0 = top + r * (H + lab)
    d.text((8, y0 + 4), title, font=font, fill=(255, 220, 120))
    for c, (key, label) in enumerate(cols):
        im = Image.open(f"{sd}/{tag}_{key}.png").convert("RGB").resize((W, H))
        img.paste(im, (c * W, y0 + lab))
        lab_key = {"0_before": "停止中・書く前", "1_stopped_written": "停止中・書いた後", "2_play_start": "Play直後", "3_stopped": "Stop後"}[key]
        u, f, p = vals[tag][lab_key]
        d.rectangle((c * W, y0 + lab + H - 22, c * W + W, y0 + lab + H), fill=(0, 0, 0))
        d.text((c * W + 5, y0 + lab + H - 21), f"USD z={u}  Fabric z={f}  PhysX z={p}", font=small, fill=(255, 255, 255))
img.save(out)
print(out, img.size)
