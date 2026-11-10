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
