# backend_spy.py — Core Experimental APIが呼び出しごとに選んだバックエンドと、フォールバックの発生を表示する観測用コード
# 解説: 05_観測Snippet/06_backend_spy.md
"""Core Experimental API が呼び出しごとに選んだバックエンドと、フォールバックの発生を表示する。

Isaac Sim の Script Editor で実行する。SPY_ENABLED を False にして再実行すると、元の関数に戻す。

Note:
    呼べる状態: Stop／Play／Pause のどれでも実行できる。[FALLBACK] の表示が出るのは、
        tensor の子の view が無効な Stop 中に、候補リストの先頭の tensor が選ばれたときである（実測）。
        Pause 中は view が有効なままなので出ない。候補リストを持たない tensor 専用のメソッド
        （get_dof_positions など）は、Stop 中は [BACKEND] 行を出さずに AssertionError になる。
    backend: backend を選ばない。prim ラッパーが行う選択を横から表示するだけである。
"""

from __future__ import annotations

import builtins  # 元の関数を、Script Editorの再実行をまたいで保持するために使う
import sys  # 呼び出し元のメソッド名を調べるために使う
import typing

import isaacsim.core.experimental.utils.backend as backend_utils  # prims側が呼び出しに使う公開モジュール
import isaacsim.core.experimental.utils.impl.backend as backend_impl  # 指定値(_context)が実際に置かれている内部モジュール
from isaacsim.core.experimental.prims import Articulation, RigidPrim  # tensorの有効性チェックを持つクラス

# tensor の有効性チェックを差し替えるクラスの名前。これ以外の文字列はエディタが警告する
SpiedClassName: typing.TypeAlias = typing.Literal["Articulation", "RigidPrim"]
# tensor の有効性チェックの型。(self, backend, fallback_backend=...) を受け取り、使う backend 名を返す
CheckForTensorBackend: typing.TypeAlias = typing.Callable[..., str]


class OriginalFunctions(typing.TypedDict):
    """差し替える前の元の関数。辞書のキーと値の型をエディタに伝えるための定義。"""

    get_current_backend: typing.Callable[..., str]  # 候補から1つ選ぶ関数
    Articulation: CheckForTensorBackend  # Articulationのtensor有効性チェック
    RigidPrim: CheckForTensorBackend  # RigidPrimのtensor有効性チェック


SPY_ENABLED: typing.Final[bool] = True  # Trueでスパイを有効にする。Falseにして再実行すると元に戻す

if not hasattr(builtins, "_backend_spy_orig"):  # 初回だけ元の関数を保存する（包みが二重にならないようにするため）
    builtins._backend_spy_orig = {  # 元の関数の保存先
        "get_current_backend": backend_utils.get_current_backend,  # 候補から1つ選ぶ関数
        "Articulation": Articulation._check_for_tensor_backend,  # Articulationのtensor有効性チェック
        "RigidPrim": RigidPrim._check_for_tensor_backend,  # RigidPrimのtensor有効性チェック
    }
orig: OriginalFunctions = builtins._backend_spy_orig  # 保存しておいた元の関数を短い名前で参照する


def spy_get_current_backend(supported_backends: list[str], **kwargs: bool | None) -> str:
    """get_current_backend の代わりに呼ばれ、選択の結果を表示してから元の関数と同じ値を返す。

    Args:
        supported_backends: 呼び出し元のメソッドが対応する backend の候補。先頭ほど優先される。
        **kwargs: 元の関数へそのまま渡す引数（raise_on_unsupported）。

    Returns:
        元の get_current_backend が選んだ backend の名前。
    """
    result = orig["get_current_backend"](supported_backends, **kwargs)  # 実際の選択は元の関数に任せる
    caller = sys._getframe(1).f_code.co_name  # この関数を呼んだメソッドの名前を取る
    # use_backend で指定された値は内部モジュールのスレッドローカル変数にある。型は公開されていないので object とする
    requested: object = getattr(backend_impl._context, "backend", None)  # use_backendで指定された値を読む（未指定ならNone）
    print(f"[BACKEND] {caller}: 候補={supported_backends} 指定={requested} → 選択={result}")  # 選択の結果を表示する
    return result  # 元の関数と同じ値を返す


def make_spy_check(class_name: SpiedClassName) -> CheckForTensorBackend:
    """指定したクラスについて、tensor 有効性チェックの包みを作る。

    Args:
        class_name: 包みを作るクラスの名前。

    Returns:
        元のチェックを呼び、判定の前後で backend が変わったらフォールバックとして表示する関数。
    """
    original = orig[class_name]  # そのクラスの元のチェック関数を取り出す

    def spy_check(self: Articulation | RigidPrim, backend: str, **kwargs: str) -> str:
        """元のチェックの代わりに呼ばれ、フォールバックが起きたら表示する。

        Args:
            self: チェックを呼んだラッパーのインスタンス。
            backend: 呼び出し元のメソッドが選んだ backend の名前。
            **kwargs: 元のチェックへそのまま渡す引数（fallback_backend）。

        Returns:
            元のチェックが返した backend の名前。
        """
        result = original(self, backend, **kwargs)  # 判定は元のチェックに任せる（例外もそのまま呼び出し元へ伝わる）
        if result != backend:  # 判定の前後でバックエンドが変わっていれば、フォールバックが起きている
            caller = sys._getframe(1).f_code.co_name  # 呼び出し元のメソッド名を取る
            print(f"[FALLBACK] {class_name}.{caller}: {backend} → {result}（tensorの子のviewが無効）")  # フォールバックを表示する
        return result  # 元のチェックと同じ値を返す

    return spy_check  # 作った包みを返す


if SPY_ENABLED:  # スパイを有効にする場合
    backend_utils.get_current_backend = spy_get_current_backend  # 公開モジュールの関数を差し替える
    Articulation._check_for_tensor_backend = make_spy_check("Articulation")  # Articulationのチェックを差し替える
    RigidPrim._check_for_tensor_backend = make_spy_check("RigidPrim")  # RigidPrimのチェックを差し替える
    print("backend spy: ON")  # 有効になったことを表示する
else:  # スパイを無効にする場合
    backend_utils.get_current_backend = orig["get_current_backend"]  # 元の関数に戻す
    Articulation._check_for_tensor_backend = orig["Articulation"]  # 元のチェックに戻す
    RigidPrim._check_for_tensor_backend = orig["RigidPrim"]  # 元のチェックに戻す
    print("backend spy: OFF")  # 無効になったことを表示する
