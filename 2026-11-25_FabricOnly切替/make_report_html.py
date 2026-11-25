"""検証結果のまとめ HTML（概要.html）を作る。screenshots/ の画像を HTML の中に埋め込む（HTML 1 つで見られる）。

実行: python make_report_html.py    （標準ライブラリだけで動く）
"""

import base64
import html
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SHOTS = os.path.join(HERE, "screenshots")
OUT = os.path.join(HERE, "概要.html")

IMAGES = [
    ("collider_writeback_on_flag_off.png", "USD書き戻しON ＋ Fabric 拡張 無効", "GUI の USD ボタンと同じ。物理の結果は USD に書かれ、そこから Fabric に入る。"),
    ("collider_writeback_off_flag_on.png", "USD書き戻しOFF ＋ Fabric 拡張 有効", "GUI の Fabric CPU ボタンと同じ。物理の結果は Fabric にだけ書かれる。"),
    ("collider_writeback_off_flag_on_fabric_gpu.png", "USD書き戻しOFF ＋ Fabric 拡張 有効（Fabric GPU）", "GUI の Fabric GPU ボタンと同じ。GPU 物理のシーン。"),
    ("collider_writeback_off_flag_off.png", "USD書き戻しOFF ＋ Fabric 拡張 無効", "物理の結果がどこにも書かれないので、Mesh も Collider も落下しない。"),
    ("collider_writeback_on_flag_on.png", "USD書き戻しON の設定 ＋ Fabric 拡張 有効", "Fabric 拡張が有効だと、USD書き戻しON の設定でも USD には書き戻されない。"),
]


def img_tag(filename):
    with open(os.path.join(SHOTS, filename), "rb") as f:
        data = base64.b64encode(f.read()).decode("ascii")
    return f'<img src="data:image/png;base64,{data}" alt="{html.escape(filename)}">'


def main():
    figures = "\n".join(
        f"""<figure>
  <h3>{html.escape(title)}</h3>
  <p class="cap">{html.escape(caption)}</p>
  {img_tag(name)}
  <p class="file">{html.escape(name)}</p>
</figure>"""
        for name, title, caption in IMAGES
    )
    page = f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<title>USD書き戻しOFF と Show By Type の Collider 表示</title>
<style>
  body {{ font-family: "Meiryo", "Yu Gothic", sans-serif; max-width: 1680px; margin: 24px auto; padding: 0 16px;
         line-height: 1.6; color: #222; background: #fafafa; }}
  h1 {{ font-size: 1.6em; border-bottom: 2px solid #444; padding-bottom: 4px; }}
  h2 {{ font-size: 1.25em; margin-top: 1.8em; border-left: 6px solid #3a7; padding-left: 8px; }}
  h3 {{ margin: 0 0 4px 0; font-size: 1.1em; }}
  table {{ border-collapse: collapse; margin: 8px 0; }}
  th, td {{ border: 1px solid #999; padding: 6px 12px; text-align: left; }}
  th {{ background: #e8e8e8; }}
  .fabric {{ color: #0a6; font-weight: bold; }}
  .usd {{ color: #36c; font-weight: bold; }}
  .none {{ color: #c33; font-weight: bold; }}
  figure {{ margin: 24px 0; padding: 12px; background: #fff; border: 1px solid #ccc; }}
  figure img {{ max-width: 100%; height: auto; display: block; }}
  .cap {{ margin: 0 0 8px 0; }}
  .file {{ margin: 4px 0 0 0; color: #777; font-size: 0.85em; }}
  code {{ background: #eee; padding: 1px 4px; }}
</style>
</head>
<body>
<h1>USD書き戻しOFF と Show By Type の Collider 表示</h1>
<p>2026-11-25〜26　Isaac Sim Full 6.2.0 で実測。</p>
<p>Viewport の Display &gt; Show By Type &gt; Physics &gt; Colliders の表示（以下 Collider 表示）が、
USD と Fabric のどちらの位置を見ているかを調べた。</p>

<h2>結論</h2>
<table>
  <tr><th>条件</th><th>Collider 表示が見るもの</th></tr>
  <tr><td>Play OFF</td><td class="usd">USD</td></tr>
  <tr><td>Play ON、Fabric 拡張 無効（fabricEnabled=False）</td><td class="usd">USD</td></tr>
  <tr><td>Play ON、Fabric 拡張 有効（fabricEnabled=True）</td><td class="fabric">Fabric</td></tr>
</table>
<ul>
  <li>Play ON = Play を押してから Stop するまで。一時停止中も含む。</li>
  <li>動的剛体・静的 Collider・Kinematic のどれでも同じ。</li>
  <li>つまり Collider 表示は「物理の結果が書かれている側」を読む。</li>
</ul>

<h2>fabricEnabled とは</h2>
<ul>
  <li><code>/physics/fabricEnabled</code> は「PhysX の結果を Fabric に書き出すかどうか」のスイッチ。</li>
  <li>omni.physx.fabric 拡張を有効にすると True、無効にすると False になる（拡張自身が設定する）。</li>
  <li>True のときは、USD書き戻しON の設定（<code>/physics/updateToUsd=True</code>）でも USD には書き戻されない。</li>
  <li>Collider 表示（omni.physx.ui 拡張）はこの値を見て、Play 中は Fabric から読むかを決めている。</li>
</ul>

<h2>Mesh（描画）に物理の結果が届く経路</h2>
<p>描画は Fabric を読む。物理の結果が Fabric に入る経路は 2 本ある。</p>
<table>
  <tr><th>Fabric 拡張</th><th>USD書き戻し</th><th>経路</th><th>Mesh</th></tr>
  <tr><td>有効</td><td>（設定に関係なく書き戻されない）</td><td>PhysX → Fabric</td><td class="fabric">動く</td></tr>
  <tr><td>無効</td><td>ON</td><td>PhysX → USD → Fabric</td><td class="fabric">動く</td></tr>
  <tr><td>無効</td><td>OFF</td><td>なし</td><td class="none">動かない（落下もしない）</td></tr>
</table>

<h2>GUI での切り替え</h2>
<p>メインメニュー <b>Window &gt; Simulation Output Settings</b> の 3 つのボタン。</p>
<table>
  <tr><th>ボタン</th><th>USD書き戻し</th><th>Fabric 拡張</th></tr>
  <tr><td>USD</td><td>ON</td><td>無効</td></tr>
  <tr><td>Fabric CPU</td><td>OFF</td><td>有効</td></tr>
  <tr><td>Fabric GPU</td><td>OFF</td><td>有効（GPU で Fabric へ渡す）</td></tr>
</table>

<h2>画像</h2>
<p>どれも同じ手順で撮った。列は Play OFF・書く前 / Play OFF・Fabric にだけ位置を書いた後 / Play ON・再生中 / Play ON・一時停止中。
静的 Collider と Kinematic は、Play ON 中も Fabric にだけ位置を書いて動かしている。
各画像の下の数値は Mesh の z（灰色の文字）と Collider 表示の z（線と同じ色の文字）。</p>
{figures}

<h2>ファイル</h2>
<ul>
  <li><code>physics_output_mode.py</code> — スクリプトで USD書き戻し ON / OFF を切り替える（<code>set_output_mode("usd" | "fabric_cpu" | "fabric_gpu")</code>）。</li>
  <li><code>verify_fabric_flag.py</code> → <code>compose_collider_writeback.py</code> — 上の画像を撮って作る。</li>
  <li><code>verify_writeback_with_fabric_ext.py</code> — Fabric 拡張が有効だと USD に書き戻されないことの確認。</li>
  <li><code>make_report_html.py</code> — この HTML を作る。</li>
  <li>その他の <code>verify_*.py</code> — 途中で行った検証。細かい数値は実行すると表示される。</li>
</ul>
</body>
</html>
"""
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(page)
    print(OUT, f"{os.path.getsize(OUT) / 1024:.0f} KB")


if __name__ == "__main__":
    main()
