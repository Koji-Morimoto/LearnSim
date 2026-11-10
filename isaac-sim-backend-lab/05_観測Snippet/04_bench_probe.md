---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, performance]
---

# bench_probe

ファイル：[snippets/bench_probe.py](../snippets/bench_probe.py)

## このノートで使う用語

| 用語      | 意味                                                                                                                                                                                             |
| ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 状態      | timeline の状態。Stop / Play / Pause の3つがある                                                                                                                                                        |
| バックエンド  | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある                                                                                                               |
| ラッパー    | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| view    | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口                                                                                                                          |
| 子の view | simulation view から、プリムのパスを指定して作る view。ラッパーは tensor で読み書きするためにこれを内部に持つ                                                                                                                          |
| メタ情報    | Snippet の解説に書く4項目（有効な状態・使うバックエンド・書き込みが届く先・他のバックエンドから見えるタイミング）                                                                                                                                  |
| 基準値     | 何もしない関数を同じ方法で測った所要時間。計測の仕組みそのものにかかる時間で、測った値からこの分は差し引いて読む                                                                                                                                       |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

API の1回の呼び出しにかかる時間を測る共通関数 `measure` を提供する。`measure` は bench_probe.py の中で定義し、`builtins` に登録するので、Script Editor の別のタブからも import せずに呼べる。あわせて、計測の仕組みそのものにかかる時間を、基準値として表示する。

どういう処理が重いか、その処理をなくす方法、なくした後に軽くなったことの確かめ方は、派生ノート [重い処理と軽くする方法](08_重い処理と軽くする方法.md) にまとめた。

## 計測値がずれる原因と対策

| 原因                                                                    | 対策                                                                                                                                                                                                                                                                                                     |
| --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 初回の呼び出しには、view の作成などの初期化コストが含まれる                                      | 最初の数回（`warmup_count`）を計測から捨てる                                                                                                                                                                                                                                                                          |
| GPU 上の Warp 配列に対する処理は、非同期に実行されることがある。完了を待たずに時刻を取ると、処理を依頼するまでの時間しか測れない | 計測区間の最後に `warp.synchronize()` を入れる（`sync=True`）。物理を GPU で動かして 1000 個の剛体の `isaacsim.core.experimental.prims.RigidPrim.get_world_poses()` を測ったところ、sync なしで 240 µs、sync ありで 255 µs だった（2026-11-09、ヘッドレスで実測）。この API は、戻った時点で GPU 側の処理がほぼ終わっているので、差は小さい。ただし、API によっては非同期のまま戻るので、GPU を使う計測では sync=True にしておく |
| 計測そのものにもオーバーヘッドがある                                                    | 何もしない関数を同じ方法で測り、基準値として並べる                                                                                                                                                                                                                                                                              |

1回の物理ステップにかかる時間は [step_probe](02_step_probe.md)、1回の更新全体の内訳は Tracy で見る。

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | 計測する API による |
| 使うバックエンド | 計測する API による |
| 書き込みが届く先 | なし |
| 他のバックエンドから見えるタイミング | 該当なし |

## コード

[snippets/bench_probe.py](../snippets/bench_probe.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/bench_probe.py（全文）
```python
# bench_probe.py — APIの呼び出し時間を計る共通関数と、計測方法そのものの基準値
# 解説: 05_観測Snippet/04_bench_probe.md
"""API の呼び出し時間を計る共通関数 measure と、計測方法そのものの基準値を提供する。

Isaac Sim の Script Editor で実行する。実行後は、別のタブから measure(...) を import なしで呼べる。

Note:
    呼べる状態: Stop／Play／Pause のどれでも実行できる。計る対象の API が呼べる状態は、対象ごとに異なる。
    backend: measure 自体は backend を選ばない。backend を指定して計る場合は、target の中で use_backend を使う。
"""

from __future__ import annotations

import builtins  # 別のScript Editorタブからも measure を呼べるように、組み込み名前空間を使う
import statistics  # 中央値を計算する標準ライブラリを読み込む
import time  # 高分解能タイマー perf_counter を使うために読み込む
import typing

import warp as wp  # GPU処理の完了待ち（synchronize）に使うWarpを読み込む


def measure(
    label: str,
    target: typing.Callable[[], object],
    repeat_count: int = 100,
    warmup_count: int = 5,
    sync: bool = False,
) -> list[float]:
    """target を warmup_count 回呼んで捨ててから、repeat_count 回呼んで1回ごとの所要時間を計る。

    結果の中央値と最小値をマイクロ秒で1行に表示し、生データを返す。

    Args:
        label: 表示する計測の名前。
        target: 計る対象の処理。引数を取らない関数。戻り値は使わないので object とする。
        repeat_count: 計測する回数。1 以上。
        warmup_count: 計測の前に空打ちする回数。0 以上。初回の初期化コストを計測から外すために使う。
        sync: True なら、各回の終了時刻を取る前に warp.synchronize で GPU 処理の完了を待つ。

    Returns:
        1回ごとの所要時間 [s] のリスト。要素数は repeat_count。
    """
    for _ in range(warmup_count):  # 初回の初期化コストを計測から外すため、先に空打ちする
        target()  # 空打ちの結果は使わずに捨てる
    if sync:  # GPUを使う計測では
        wp.synchronize()  # 空打ちで発行したGPU処理が終わるまで待ってから計測を始める
    samples: list[float] = []  # 1回ごとの所要時間（秒）を貯めるリストを用意する
    for _ in range(repeat_count):  # repeat_count回くり返して計る
        start = time.perf_counter()  # 開始時刻を取る
        target()  # 計測対象を1回呼ぶ
        if sync:  # GPU処理の完了まで含めて計る場合は
            wp.synchronize()  # GPU処理の完了を待ってから終了時刻を取る
        samples.append(time.perf_counter() - start)  # 経過時間をリストに記録する
    median_us = statistics.median(samples) * 1e6  # 中央値を秒からマイクロ秒に直す
    min_us = min(samples) * 1e6  # 最小値を秒からマイクロ秒に直す
    print(f"{label:<28} median={median_us:9.1f}us min={min_us:9.1f}us repeat={repeat_count} sync={sync}")  # 結果を1行で表示する
    return samples  # 分布を後で見られるように生データを返す


builtins.measure = measure  # 別のタブから measure(...) と呼べるように登録する
measure("baseline: 空の関数", lambda: None)  # 計測方法そのもののオーバーヘッドを基準値として出す
measure("baseline: 空の関数+sync", lambda: None, sync=True)  # synchronize自体のコストを基準値として出す
```

## 観測手順と期待される出力

1. 上の「コード」の節の bench_probe.py（全文）を Script Editor に貼って実行する。bench_probe.py は、最後の2行で「何もしない関数」を2通りの方法で測り、その結果を基準値として表示する。そのため、実行すると次の2行が出る。

```text
baseline: 空の関数               median=      0.2us min=      0.1us repeat=100 sync=False
baseline: 空の関数+sync          median=      1.9us min=      1.7us repeat=100 sync=True
```

   上は 2026-11-09 にヘッドレスで実行したときの出力である。
   1行目は、`measure` で「何もしない関数」を測った時間、つまり計測の仕組みそのものにかかる時間である。
   2行目は、それに `warp.synchronize()` を加えた時間である。数値は環境によって変わる。
2. 以後、API を測った値は、この2行と並べて読む。基準値と同じ桁の差は、計測誤差の範囲とみなす。
3. 使い方の例として、別のタブで次のコードを実行する。`measure` は bench_probe.py が `builtins` に登録した関数なので、import せずに呼べる。

Script Editor に貼って実行するコード（ファイルではない。bench_probe.py を実行した後に実行する）
```python
# bench_probe.py の measure で、Articulation.get_dof_positions の呼び出し時間を測る（Play 中に実行する）
from __future__ import annotations  # 型ヒントを実行時に評価させない
import typing
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー

ROBOT_PATH: typing.Final[str] = "/World/robot"  # Articulation のルートのパス（自分のシーンに合わせて書き換える）
robot: Articulation = Articulation(ROBOT_PATH)  # ラッパーを作る
measure("get_dof_positions", lambda: robot.get_dof_positions(), sync=True)  # GPU処理の完了まで含めて100回測る
```

4. [複数プリム対応とPythonループの有無](../01_用語と概念/05_複数プリム対応とPythonループの有無.md) の「usd と usdrt はプリムの数 N に比例して時間が伸び、fabric と tensor はほとんど伸びない」を、N を変えて確かめる。2026-11-09 にヘッドレスで測った `get_world_poses` の中央値は次のとおりである（µs）。

| N    | XformPrim usd | XformPrim usdrt | XformPrim fabric | RigidPrim 既定（Play 中は tensor） |
| ---- | ------------- | --------------- | ---------------- | ---------------------------- |
| 1    | 100           | 97              | 208              | 119                          |
| 10   | 337           | 325             | 208              | 114                          |
| 100  | 2770          | 2425            | 224              | 117                          |
| 1000 | 24745         | 22512           | 237              | 150                          |

usd と usdrt は、プリム1個あたり約 22 µs ずつ伸びた。fabric と tensor は、N が 1000 になってもほとんど伸びなかった。ただし N = 1 では、fabric（208 µs）が usd（100 µs）より遅かった。fabric の経路は、呼び出すたびに Fabric の階層の更新と selection の更新を行うので、N が小さいときはその固定費のほうが大きい。N = 10 では逆転し、fabric のほうが速かった。

## 期待と違ったときに疑う箇所

- tensor の計測値が sync なしで極端に小さい場合、GPU 処理の完了を待っていない可能性がある。sync=True と比べる。
- 1回目の計測値だけが極端に大きい場合、初期化のコストが入っている。`warmup_count` を増やす。

## 補足：print の費用

### 検証コード

```python
def show_raw(label: str, value: object) -> None:
    print(f"{label}: {value}")

measure("show_raw", lambda: show_raw("timeline.is_playing", timeline.is_playing))
measure("print only", lambda: print("timeline.is_playing:", timeline.is_playing))
measure("format only", lambda: f"timeline.is_playing: {timeline.is_playing}")
measure("is_playing only", lambda: timeline.is_playing())
```

### 実行結果

圧倒的に`print`が重い。10回実行すれば1msだからかなりのコスト感。

```
show_raw                     median=     73.2us, min=     71.2us, n=100, sync=False
print only                   median=    122.3us, min=    119.1us, n=100, sync=False
format only                  median=      0.8us, min=      0.7us, n=100, sync=False
is_playing only              median=      0.5us, min=      0.4us, n=100, sync=False
```

### 考察

1. **Script Editor の print が重い理由。** Script Editor で実行したコードの print は、標準出力のイベントとして Script Editor のログ欄に送られる。ログ欄（`omni.kit.window.script_editor` の `log_view.py` の `LogModel`）は、1行受け取るたびにその行を一覧に足し、表示用のツリーに「中身が変わった」と通知して作り直させる。ソースのコメントにも、行数が多いと作り直しに時間がかかると書かれている（ログ欄は最大 1000 行を保ち、2000 行を超えると古い行を捨てる。設定 `/exts/omni.kit.window.script_editor/maxLogLines`）。同じ print を、Script Editor のウィンドウを表示しないヘッドレスで端末に出すと 0.9 µs、Script Editor と同じ実行経路（`omni.kit.app.get_app().get_python_scripting().execute_string()`）で実行すると 13.3 µs だった（2026-11-09 に実測）。GUI の 122 µs との差は、ログ欄の表示を作り直す時間である。
2. **`show_raw` と `print only` の差は、引数の数では説明できない。** Script Editor と同じ実行経路で、print 1回あたりにログ欄へ送られるイベントの数を数えると、引数が1つでも2つでも1件だった。費用も、引数1つで 13.3 µs、引数2つで 13.7 µs と、ほとんど変わらなかった（ヘッドレスで実測）。GUI での 73 µs と 122 µs の差の原因を切り分けるには、[重い処理と軽くする方法](08_重い処理と軽くする方法.md) の「1. Script Editor の print」の確認コードを、ログ欄を Clear した直後と、ログ欄に 1000 行ほど溜まった後の2回実行して比べる。
3. **`timeline.is_playing` と `timeline.is_playing()` の違い。** `format only` と `show_raw` は、`is_playing` をメソッドとして呼ばずに、メソッドそのものを文字列にしている（`<bound method ...>` という文字列になる）。値を表示したい場合は `timeline.is_playing()` と書く。
4. **軽くする方法。** 行をリストに貯めて最後に1回の print でまとめて出す、ログ欄を Clear してから実行する、などがある。それぞれの効果を確かめる手順と結果の記入欄は、[重い処理と軽くする方法](08_重い処理と軽くする方法.md) の「1. Script Editor の print」にある。

## 関連ノート

- [重い処理と軽くする方法](08_重い処理と軽くする方法.md)
- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [clock_probe](03_clock_probe.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [pose_freshness_probe](05_pose_freshness_probe.md) |
