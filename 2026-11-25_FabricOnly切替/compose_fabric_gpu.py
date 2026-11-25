import sys, re
from PIL import Image, ImageDraw, ImageFont
sd, out, log = sys.argv[1], sys.argv[2], sys.argv[3]
font = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 22)
hfont = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 19)
small = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 16)
vals, cur = {}, None
for line in open(log, encoding="utf-8", errors="replace"):
    m = re.match(r"\[(dynamic|static|kinematic)\]", line)
    if m:
        cur = m.group(1); vals[cur] = []; continue
    m = re.match(r"\s+(.+?) USD z=(\S+)\s+Fabric z=(\S+)\s+画像=(\S+)", line)
    if m and cur:
        vals[cur].append(m.groups())
cols = ["Play OFF・書く前", "Play OFF・Fabric に z=1.0 を書いた後", "Play ON・再生中", "Play ON・一時停止中"]
names = {"dynamic": "動的剛体（緑）", "static": "静的 Collider（マゼンタ）", "kinematic": "Kinematic（黄）"}
W, H, lab, top = 400, 225, 32, 40
img = Image.new("RGB", (W * 4, top + (H + lab) * 3), (30, 30, 30))
d = ImageDraw.Draw(img)
for c, t in enumerate(cols):
    d.text((c * W + W // 2, 20), t, font=hfont, fill=(255, 255, 255), anchor="mm")
for r, body in enumerate(("dynamic", "static", "kinematic")):
    y0 = top + r * (H + lab)
    d.text((8, y0 + 4), names[body] + " ／ USD書き戻しOFF（Fabric GPU）", font=font, fill=(255, 220, 120))
    for c, (label, u, f, fn) in enumerate(vals[body]):
        im = Image.open(f"{sd}/{fn}").convert("RGB").resize((W, H))
        img.paste(im, (c * W, y0 + lab))
        d.rectangle((c * W, y0 + lab + H - 22, c * W + W, y0 + lab + H), fill=(0, 0, 0))
        d.text((c * W + 5, y0 + lab + H - 21), f"USD z={u}  Fabric z={f}", font=small, fill=(255, 255, 255))
img.save(out)
print(out, img.size)
