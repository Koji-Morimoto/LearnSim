# supported_backends.py — Core Experimental API のメソッドが対応するバックエンドを、メソッドのソースを読んで一覧表示する
# 解説: 05_観測Snippet/09_supported_backends.md
"""Core Experimental API のメソッドが対応するバックエンド（候補リスト）を、ソースを解析して表示する。

スパイのようにメソッドを実際に呼ぶ必要はなく、状態（Stop / Play / Pause）にも左右されない。
実行すると show_supported_backends を builtins に登録し、例として Articulation・RigidPrim・XformPrim の一覧を表示する。

Note:
    呼べる状態: Stop／Play／Pause のどれでも同じ結果になる（ソースを読むだけで、ステージには触らない）。
    backend: メソッドを呼ばないので、backend の選択は起きない。
"""

from __future__ import annotations

import ast
import builtins  # 別のタブからも show_supported_backends を呼べるようにするために使う
import inspect  # メソッドのソースを取り出すために使う
import textwrap  # クラスの中で字下げされたソースを、解析できる形に戻すために使う
import typing

from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim  # 例として表示する3つのラッパー


class BackendInfo(typing.TypedDict):
    """1つのメソッドについて、ソースから分かったこと。"""

    candidate_lists: list[list[str]]  # get_current_backend に渡している候補リスト（メソッドの中に出てくる順）
    tensor_only: bool  # 候補リストを持たず、子の view が有効かを assert してから tensor で処理するか
    delegates_to: list[str]  # 処理を渡している、別クラスの同じ名前のメソッド（例: XformPrim.get_world_poses）
    kind: str  # "method" / "property" / "staticmethod"


def _source_function(cls: type, name: str) -> tuple[typing.Callable[..., object] | None, str]:
    """クラスの属性 name から、ソースを読める関数と、その種類を取り出す。"""
    attribute = inspect.getattr_static(cls, name)  # 継承をたどってクラスの属性をそのまま取り出す（呼び出さない）
    if isinstance(attribute, property):  # プロパティなら
        return attribute.fget, "property"  # 値を返す関数を読む
    if isinstance(attribute, staticmethod):  # 静的メソッドなら
        return attribute.__func__, "staticmethod"  # 中の関数を読む
    if isinstance(attribute, classmethod):  # クラスメソッドなら
        return attribute.__func__, "classmethod"  # 中の関数を読む
    if inspect.isfunction(attribute):  # 普通のメソッドなら
        return attribute, "method"  # そのまま読む
    return None, "other"  # 関数でない属性は読まない


def analyze_method(cls: type, name: str) -> BackendInfo | None:
    """cls.name のソースを解析し、バックエンドに関する情報を返す。関数でなければ None を返す。

    Args:
        cls: ラッパーのクラス（Articulation など）。
        name: メソッドかプロパティの名前。

    Returns:
        候補リスト・tensor 専用かどうか・処理を渡す先・種類をまとめた辞書。
    """
    function, kind = _source_function(cls, name)  # ソースを読める関数を取り出す
    if function is None:  # 関数でなければ
        return None  # 解析しない
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))  # ソースを構文木にする
    info: BackendInfo = {"candidate_lists": [], "tensor_only": False, "delegates_to": [], "kind": kind}  # 結果の入れ物
    for node in ast.walk(tree):  # 構文木のすべての節を調べる
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):  # 「なにか.名前(...)」の呼び出しなら
            if node.func.attr == "get_current_backend" and node.args and isinstance(node.args[0], ast.List):  # 候補リストを渡す呼び出し
                info["candidate_lists"].append([element.value for element in node.args[0].elts if isinstance(element, ast.Constant)])  # 候補を記録する
            delegate = None  # 処理を渡す先（見つからなければ None）
            if isinstance(node.func.value, ast.Name) and node.func.value.id in ("XformPrim", "RigidPrim", "Articulation"):  # 別のラッパーのメソッドを直接呼んでいれば
                delegate = f"{node.func.value.id}.{node.func.attr}"  # 「クラス名.メソッド名」
            elif isinstance(node.func.value, ast.Call) and isinstance(node.func.value.func, ast.Name) and node.func.value.func.id == "super":  # super().名前(...) なら
                owner = next(c for c in cls.__mro__ if name in c.__dict__)  # このメソッドを定義しているクラス
                parent = next((c for c in owner.__mro__[1:] if node.func.attr in c.__dict__), None)  # super() で呼ばれる親クラス
                delegate = f"{parent.__name__}.{node.func.attr}" if parent else None  # 「親クラス名.メソッド名」
            if delegate and node.func.attr != name:  # 名前の違うメソッド（補助の関数など）は、処理を渡す先として数えない
                delegate = None
            if delegate and delegate not in info["delegates_to"]:  # 同じものを二度記録しない
                info["delegates_to"].append(delegate)  # 処理を渡す先として記録する
        if isinstance(node, ast.Assert) and "is_physics_tensor_entity_valid" in ast.unparse(node.test):  # 子の view が有効かを assert していれば
            info["tensor_only"] = not info["candidate_lists"]  # 候補リストがなければ tensor 専用（あとで候補が見つかれば下で直す）
    if info["candidate_lists"]:  # 候補リストがあれば
        info["tensor_only"] = False  # tensor 専用ではない
    return info  # 結果を返す


def describe(info: BackendInfo) -> str:
    """analyze_method の結果を1行の説明にする。"""
    if info["tensor_only"]:  # tensor 専用なら
        return "tensor 専用（get_current_backend を呼ばない。Stop 中は AssertionError）"
    if not info["candidate_lists"]:  # 候補リストもなければ
        return "バックエンドを選ばない"
    text = " / ".join(str(candidates) for candidates in info["candidate_lists"])  # 候補リストを並べる
    if info["delegates_to"]:  # 処理を渡す先があれば
        text += f"（tensor 以外は {', '.join(info['delegates_to'])} に渡し、そこでもう一度選ぶ）"
    return text  # 説明を返す


def show_supported_backends(target: type | object, method_name: str | None = None) -> None:
    """ラッパーのクラスかインスタンスについて、メソッドごとの対応バックエンドを表示する。

    Args:
        target: Articulation などのクラス、またはそのインスタンス。
        method_name: 1つのメソッドだけを見る場合はその名前。None なら公開されているすべてのメソッドとプロパティを表示する。
    """
    cls = target if isinstance(target, type) else type(target)  # インスタンスを渡された場合はクラスにする
    names = [method_name] if method_name else sorted(name for name in dir(cls) if not name.startswith("_"))  # 調べる名前の一覧
    for name in names:  # 名前ごとに
        info = analyze_method(cls, name)  # ソースを解析する
        if info is None:  # 関数でなければ
            continue  # 表示しない
        label = f"{cls.__name__}.{name}" + ("（プロパティ）" if info["kind"] == "property" else "")  # 表示名
        print(f"{label:<62} {describe(info)}")  # 1行で表示する


builtins.show_supported_backends = show_supported_backends  # 別のタブから show_supported_backends(...) と呼べるように登録する
builtins.analyze_method = analyze_method  # 結果を値として受け取りたい場合のために登録する

for wrapper_class in (Articulation, RigidPrim, XformPrim):  # 例として3つのラッパーを表示する
    print(f"===== {wrapper_class.__name__} =====")  # 見出し
    show_supported_backends(wrapper_class)  # すべてのメソッドを表示する
