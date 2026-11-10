# make_time_states.py - 図4（time-states.svg / .png）を実測値から生成する
# 実測: 2026-11-09、Isaac Sim 6.2.0 ヘッドレス（isaacsim.exp.full.kit）、Steps per Second 30、timeCodesPerSecond 60
# 使い方: py make_time_states.py  （cairosvg が必要）
import pathlib

import cairosvg

HERE = pathlib.Path(__file__).parent
FONT = "Noto Sans JP, sans-serif"  # Windows に入っている CJK フォント（Noto Sans CJK JP は未導入）
BG, FG, BLUE, GREEN, RED, YEL, GREY = "#1e1e2e", "#cdd6f4", "#89b4fa", "#a6e3a1", "#f38ba8", "#f9e2af", "#45475a"

# 更新番号 k ごとの (timeline 時刻 τ, シミュレーション時刻 t)。k=0 は Play を押す前。
# k=1..6 は Play、k=7..8 は Pause、k=8 と k=9 の間で step(3)、k=9..16 は Play 再開（実測値）
TAU = [0.0, 0.01667, 0.03333, 0.05, 0.06667, 0.08333, 0.1, 0.1, 0.1,
       0.11667, 0.13333, 0.15, 0.16667, 0.18333, 0.2, 0.21667, 0.23333]
SIM = [0.0, 0.06667, 0.1, 0.1, 0.13333, 0.13333, 0.16667, 0.16667, 0.16667,
       0.26667, 0.3, 0.3, 0.33333, 0.33333, 0.36667, 0.36667, 0.4]

X0, DX, Y0, YS = 130, 52, 480, 820.0  # x 原点・間隔、y 原点・1秒あたりの px


def px(k):
    return X0 + DX * k


def py(v):
    return Y0 - YS * v


out = []
a = out.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="1160" height="660" viewBox="0 0 1160 660">')
a(f'<defs><marker id="arrow-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
  f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{RED}"/></marker>'
  f'<marker id="arrow-fg" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
  f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{FG}"/></marker></defs>')
a(f'<rect class="background" x="0" y="0" width="1160" height="660" fill="{BG}"/>')
a(f'<text x="40" y="42" font-family="{FONT}" font-size="20" fill="{FG}" font-weight="bold">'
  f'図4　Play・Pause・手動ステップでの2つの時計（実測。Steps per Second 30、timeCodesPerSecond 60）</text>')
a(f'<text x="40" y="78" font-family="{FONT}" font-size="14" fill="{FG}">'
  f'時刻 [秒]（青丸は timeline 時刻、緑のひし形はシミュレーション時刻）</text>')
# Pause 帯
pl, pr = px(6.5), px(8.85)
a(f'<rect class="container" x="{pl}" y="100" width="{pr - pl:.1f}" height="380" fill="{GREY}" stroke="{GREY}"/>')
a(f'<text x="{(pl + pr) / 2:.1f}" y="124" font-family="{FONT}" font-size="15" fill="{FG}" text-anchor="middle">Pause</text>')
a(f'<text x="{px(3.5):.1f}" y="124" font-family="{FONT}" font-size="15" fill="{FG}" text-anchor="middle">Play</text>')
a(f'<text x="{px(12.5):.1f}" y="124" font-family="{FONT}" font-size="15" fill="{FG}" text-anchor="middle">Play（再開）</text>')
# 軸
a(f'<line x1="{X0}" y1="{Y0}" x2="{px(16) + 30}" y2="{Y0}" stroke="{FG}" stroke-width="2"/>')
a(f'<line x1="{X0}" y1="{Y0}" x2="{X0}" y2="100" stroke="{FG}" stroke-width="2"/>')
for k in range(17):
    a(f'<line x1="{px(k)}" y1="{Y0}" x2="{px(k)}" y2="{Y0 + 8}" stroke="{FG}" stroke-width="2"/>')
    a(f'<text x="{px(k)}" y="{Y0 + 26}" font-family="{FONT}" font-size="13" fill="{FG}" text-anchor="middle">{k}</text>')
for i in range(9):
    v = 0.05 * i
    a(f'<line x1="{X0 - 8}" y1="{py(v):.1f}" x2="{X0}" y2="{py(v):.1f}" stroke="{FG}" stroke-width="2"/>')
    a(f'<text x="{X0 - 14}" y="{py(v) + 5:.1f}" font-family="{FONT}" font-size="13" fill="{FG}" text-anchor="end">{v:.2f}</text>')
a(f'<text x="{px(8):.1f}" y="{Y0 + 52}" font-family="{FONT}" font-size="14" fill="{FG}" text-anchor="middle">'
  f'メインループの更新の回数（0 は Play を押す前）</text>')
# データ点
for k in range(17):
    x = px(k)
    a(f'<circle cx="{x}" cy="{py(TAU[k]):.1f}" r="6" fill="{BLUE}"/>')
    yy = py(SIM[k])
    a(f'<path d="M{x},{yy - 9:.1f} L{x + 9},{yy:.1f} L{x},{yy + 9:.1f} L{x - 9},{yy:.1f} z" fill="none" stroke="{GREEN}" stroke-width="2.5"/>')
# 手動ステップの矢印
ax = px(8.45)
a(f'<line x1="{ax:.1f}" y1="{py(SIM[8]) - 4:.1f}" x2="{ax:.1f}" y2="{py(SIM[9]) + 2:.1f}" stroke="{RED}" stroke-width="3" marker-end="url(#arrow-red)"/>')
a(f'<text x="{ax - 16:.1f}" y="{py(0.25):.1f}" font-family="{FONT}" font-size="14" fill="{RED}" text-anchor="end">step(3)：</text>')
a(f'<text x="{ax - 16:.1f}" y="{py(0.25) + 20:.1f}" font-family="{FONT}" font-size="14" fill="{RED}" text-anchor="end">シミュレーション時刻だけ</text>')
a(f'<text x="{ax - 16:.1f}" y="{py(0.25) + 40:.1f}" font-family="{FONT}" font-size="14" fill="{RED}" text-anchor="end">0.1 秒進む</text>')
# 差の寸法線（k=16）
gx = px(16) + 22
a(f'<line x1="{gx}" y1="{py(TAU[16]) - 2:.1f}" x2="{gx}" y2="{py(SIM[16]) + 2:.1f}" stroke="{FG}" stroke-width="1.5" '
  f'marker-start="url(#arrow-fg)" marker-end="url(#arrow-fg)"/>')
a(f'<text x="{gx + 10}" y="{py((TAU[16] + SIM[16]) / 2) - 4:.1f}" font-family="{FONT}" font-size="14" fill="{FG}">差 0.167 秒</text>')
a(f'<text x="{gx + 10}" y="{py((TAU[16] + SIM[16]) / 2) + 16:.1f}" font-family="{FONT}" font-size="14" fill="{FG}">（縮まない）</text>')
# 左上の注記（ウォームアップ）
for i, line in enumerate(["Play 開始時のウォームアップで", "物理ステップが2回進むため、",
                          "シミュレーション時刻は最初から", "timeline 時刻より 0.05〜0.067 秒先"]):
    a(f'<text x="{X0 + 20}" y="{160 + 20 * i}" font-family="{FONT}" font-size="14" fill="{GREEN}">{line}</text>')
# 凡例
ly = 572
a(f'<circle cx="60" cy="{ly - 5}" r="6" fill="{BLUE}"/>')
a(f'<text x="76" y="{ly}" font-family="{FONT}" font-size="14" fill="{FG}">timeline 時刻</text>')
a(f'<path d="M220,{ly - 14} L229,{ly - 5} L220,{ly + 4} L211,{ly - 5} z" fill="none" stroke="{GREEN}" stroke-width="2.5"/>')
a(f'<text x="238" y="{ly}" font-family="{FONT}" font-size="14" fill="{FG}">シミュレーション時刻</text>')
a(f'<rect class="container" x="420" y="{ly - 15}" width="22" height="20" fill="{GREY}" stroke="{GREY}"/>')
a(f'<text x="450" y="{ly}" font-family="{FONT}" font-size="14" fill="{FG}">Pause 中（どちらの時計も止まる）</text>')
a(f'<line x1="720" y1="{ly - 5}" x2="770" y2="{ly - 5}" stroke="{RED}" stroke-width="3" marker-end="url(#arrow-red)"/>')
a(f'<text x="780" y="{ly}" font-family="{FONT}" font-size="14" fill="{FG}">手動ステップ</text>')
a(f'<text x="40" y="620" font-family="{FONT}" font-size="15" fill="{YEL}" font-weight="bold">'
  f'→ シミュレーション時刻は Play 中ずっと timeline 時刻より先にある。手動ステップの分だけ差が広がり、Stop まで戻らない</text>')
a('</svg>')

svg = "\n".join(out)
(HERE / "time-states.svg").write_text(svg, encoding="utf-8")
(HERE / "time-states.html").write_text(
    '<!DOCTYPE html>\n<html lang="ja"><head><meta charset="utf-8"><title>time-states</title>'
    f'<style>html,body{{margin:0;padding:0;background:{BG}}}</style></head>\n<body>\n{svg}\n</body></html>\n',
    encoding="utf-8")
cairosvg.svg2png(bytestring=svg.encode("utf-8"), write_to=str(HERE / "time-states.png"), scale=2)
print("written")
