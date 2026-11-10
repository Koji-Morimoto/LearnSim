---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, backend, strict]
---

# strict_backend

ファイル：[snippets/strict_backend.py](../snippets/strict_backend.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| 子の view | simulation view から、プリムのパスを指定して作る view。ラッパーは tensor で読み書きするためにこれを内部に持つ |
| フォールバック | 選ばれたバックエンドが使えないために、別のバックエンドに差し替わること |
| strict | `isaacsim.core.experimental.utils.backend.use_backend()` の2つのフラグ（raise_on_unsupported と raise_on_fallback）を True に固定したもの。指定どおりに処理されなければ例外になる |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

2つの raise フラグを True に固定した `isaacsim.core.experimental.utils.backend.use_backend()`（strict）を定義し、別のタブからも `with strict("tensor"):` と書けるようにする。あわせて、2種類の例外が出る条件をデモで観測する。strict の考え方は [バックエンド選択の仕組み（「strict による仮説検証」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md) にまとめた。

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | strict の定義はどの状態でもよい。デモは Stop・Play・Pause のそれぞれで実行する |
| 使うバックエンド | デモでは tensor を指定する |
| 書き込みが届く先 | なし（姿勢を読むだけ） |
| 他のバックエンドから見えるタイミング | 該当なし |

## コード

[snippets/strict_backend.py](../snippets/strict_backend.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/strict_backend.py（全文）
```python
# strict_backend.py — 2つのraiseフラグをTrueに固定したuse_backend（strict）を定義し、2種類の例外が出る条件をデモで観測する
# 解説: 05_観測Snippet/07_strict_backend.md
"""2つの raise フラグを True に固定した use_backend（strict）を定義し、2種類の例外が出る条件をデモで観測する。

Isaac Sim の Script Editor で実行する。実行後は、別のタブから with strict("tensor"): と import なしで書ける。

Note:
    呼べる状態: strict の定義は Stop／Play／Pause のどれでもできる。デモの結果は状態で変わる
        （Stop 中と Play 中の両方で実行して比べる）。
    backend: デモでは tensor を指定する。
"""

from __future__ import annotations

import builtins  # 別のScript Editorタブからも strict を使えるように、組み込み名前空間を使う
import functools  # 引数を固定した関数を作るために使う
import typing

import isaacsim.core.experimental.utils.backend as backend_utils  # use_backendを提供するモジュール
from isaacsim.core.experimental.prims import RigidPrim, XformPrim  # デモで使う2種類のラッパー

# 指定どおりに処理されなければ例外にするuse_backend。型は functools.partial の戻り値からエディタが推論するので、注釈を書かない
strict = functools.partial(backend_utils.use_backend, raise_on_unsupported=True, raise_on_fallback=True)
builtins.strict = strict  # 別のタブから with strict("tensor"): と書けるように登録する
print('strict を登録した。使い方: with strict("tensor"): ...')  # 登録できたことを表示する

RUN_DEMO: typing.Final[bool] = True  # Trueならデモを実行する。strictの定義だけが必要なときはFalseにする
PRIM_PATH: typing.Final[str] = "/World/Cube"  # デモで使う剛体のパス（自分のシーンに合わせて書き換える）


def try_call(label: str, action: typing.Callable[[], None]) -> None:
    """action を呼び、成功したか、どの例外が出たかを1行で表示する。

    デモの2つの例を続けて実行するために、例外をこの関数の中で受け止める。

    Args:
        label: 表示する例の名前。
        action: 実行する処理。引数を取らず、戻り値を使わない関数。
    """
    try:  # 呼び出しを試みる
        action()  # 対象の処理を実行する
        print(f"[OK ] {label}")  # 例外が出なかったことを表示する
    except Exception as error:  # 例外が出た場合は
        print(f"[ERR] {label}: {type(error).__name__}: {error}")  # 例外の種類とメッセージを表示する


def demo_unsupported() -> None:
    """非対応の backend（XformPrim.get_world_poses に tensor）を指定する例。

    Raises:
        RuntimeError: 候補に tensor がないため、Unsupported backend の例外が出る（Stop／Play／Pause のすべてで実測）。

    Note:
        呼べる状態: Stop／Play／Pause のどれでも同じ例外になる（実測）。
        backend: tensor を指定する。
    """
    xform = XformPrim(PRIM_PATH)  # 姿勢取得の候補が usd→usdrt→fabric のラッパーを作る
    with strict("tensor"):  # 仮説：「XformPrimの姿勢取得はtensorで処理できる」
        xform.get_world_poses()  # 候補にtensorがないので Unsupported backend の例外になる


def demo_fallback() -> None:
    """対応しているが、状態によっては使えない backend（RigidPrim.get_world_poses に tensor）を指定する例。

    Raises:
        RuntimeError: Stop 中は子の view が無効なので、フォールバックの例外が出る（実測）。

    Note:
        呼べる状態: Stop 中は例外、Play 中と Pause 中は成功する（実測）。
        backend: tensor を指定する。
    """
    rigid = RigidPrim(PRIM_PATH)  # 姿勢取得の候補が tensor→usd→usdrt→fabric のラッパーを作る
    with strict("tensor"):  # 仮説：「今の状態ではtensorが使える」
        rigid.get_world_poses()  # Stop中は子のviewが無効なので例外、Play中とPause中は成功する


if RUN_DEMO:  # デモを実行する場合
    try_call("XformPrim.get_world_poses を tensor で", demo_unsupported)  # 非対応の例を実行する
    try_call("RigidPrim.get_world_poses を tensor で", demo_fallback)  # 状態に依存する例を実行する
```

## 観測手順と期待される出力

準備：`/World/Cube` に Rigid Body と Collider を付けた Cube を置く（パスが違う場合は `PRIM_PATH` を書き換える）。[pose_freshness_probe](05_pose_freshness_probe.md) の「観測手順」にある準備のコードでも作れる。

ここでの「strict_backend を実行する」は、上の「コード」の節の strict_backend.py（全文）を Script Editor に貼って実行することを指す。Stop・Play・Pause の各状態で1回ずつ strict_backend を実行する。1回の実行で、登録の行に続いて2行が表示される。表の行は、2026-11-09 にヘッドレスで実測した出力である。

| 手順 | 状態 | 表示された行 | 意味 |
|---|---|---|---|
| 1 | Stop | `[ERR] XformPrim.get_world_poses を tensor で: RuntimeError: Unsupported backend: 'tensor'. Supported backends: 'usd', 'usdrt', 'fabric'` | `isaacsim.core.experimental.prims.XformPrim.get_world_poses()` は、tensor に対応していない |
| 2 | Stop | `[ERR] RigidPrim.get_world_poses を tensor で: RuntimeError: Physics tensor entity is not valid. Fallback set to 'usd' backend.` | `isaacsim.core.experimental.prims.RigidPrim.get_world_poses()` は tensor に対応しているが、Stop 中は子の view が無効である |
| 3 | Play | 手順1と同じ行 | 対応していないことは、状態によって変わらない |
| 4 | Play | `[OK ] RigidPrim.get_world_poses を tensor で` | Play 中は子の view が有効なので、tensor で処理される |
| 5 | Pause | 手順1と同じ行と、手順4と同じ行 | Pause 中も子の view は有効なままである |

`Supported backends:` の後には、候補リストの中身が、引用符で囲んでカンマで区切った形（`'usd', 'usdrt', 'fabric'`）で表示される。

strict の定義だけが必要な場合は、`RUN_DEMO = False` にして実行する。

### 手順をコードで自動実行する

Play / Pause / Stop のボタンは、`omni.timeline` のメソッドで代わりに押せる。`timeline.play()` などは呼んだ時点では状態を変えず、次のメインループの更新で反映される。続けて結果を見るには、`async` の関数の中で `await omni.kit.app.get_app().next_update_async()` を使って更新を待つ。

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# Stop → Play → Pause → Stop の各状態で strict_backend.py を実行する
import asyncio  # async の関数を Kit のイベントループで走らせるために使う

import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う

SNIPPET_PATH = r"C:\isaacsim-6.2.0\CodeByClaude\isaac-sim-backend-lab\snippets\strict_backend.py"  # strict_backend.py の場所


def run_demo(state_label: str) -> None:
    """見出しを出してから strict_backend.py を実行する。"""
    print(f"===== {state_label} =====")  # どの状態の出力かを示す見出し
    exec(open(SNIPPET_PATH, encoding="utf-8").read(), {"__name__": "__main__"})  # Script Editor で実行したのと同じように実行する


async def run_all() -> None:
    """状態を切り替えながら run_demo を呼ぶ。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    run_demo("Stop")  # 手順1・2
    timeline.play()  # Play（次の更新で反映される）
    for _ in range(5):  # 物理の準備ができるまで待つ
        await app.next_update_async()  # 更新を1回待つ
    run_demo("Play")  # 手順3・4
    timeline.pause()  # Pause
    await app.next_update_async()  # Pause が反映されるまで1回待つ
    run_demo("Pause")  # 手順5
    timeline.stop()  # Stop に戻す


asyncio.ensure_future(run_all())  # Kit のイベントループで run_all を走らせる（この行はすぐに戻る）
```

## 期待と違ったときに疑う箇所

- 手順4で例外が出る場合、デモは実行のたびにラッパーを作り直しているので、子の view が作られない原因は simulation view の側にある。[env_fingerprint](01_env_fingerprint.md) で `tensor simulation view` を確かめる。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [backend_spy](06_backend_spy.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [重い処理と軽くする方法](08_重い処理と軽くする方法.md) |
