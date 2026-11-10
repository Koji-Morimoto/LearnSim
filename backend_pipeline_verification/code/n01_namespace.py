"""N01: 名前空間(モジュールパス)を見れば、使われるバックエンドを類推できるか。

確かめること:
    N-01  静的解析: isaacsim.core.experimental.prims の各クラス・各メソッドについて
          docstring の "Backends:" 表記と、実装の get_current_backend([...]) に渡す候補リストを突き合わせる。
          既定のバックエンド(=候補リストの先頭)がクラスやメソッドでどう違うかを表にする。
    N-02  返り値の型: バックエンドを変えても、返り値の型(名前空間)は変わるか。
    N-03  実行時の追跡: 実装モジュールが参照する名前空間(pxr / usdrt / warp.fabricarray / omni.physics.tensors)を
          スパイで記録し、「実際にどの層に触ったか」をタイムラインの状態 × バックエンド指定 × メソッドごとに表にする。
    N-04  "backend" という語の別の意味: SimulationManager.get_backend() や create_simulation_view の backend 引数、
          非推奨 API(isaacsim.core.prims)の backend が何を指すか。
    N-05  use_backend のエラー・フォールバック規則と、スレッドローカルであることの確認。
"""

from __future__ import annotations

import bplib

app = bplib.start()

import ast
import inspect
import re
import threading
from typing import Any

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.physics.tensors
from isaacsim.core.experimental.prims import Articulation, DeformablePrim, GeomPrim, Prim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager

rec = bplib.Recorder("n01_namespace")

BACKENDS: list[str | None] = [None, "tensor", "usd", "usdrt", "fabric"]  # None = use_backend を使わない(既定)
CLASSES: list[type] = [Prim, XformPrim, GeomPrim, RigidPrim, Articulation, DeformablePrim]
_DOC_BACKENDS = re.compile(r"Backends:\s*((?::guilabel:`[^`]+`[,\s]*)+)")


def doc_backends(fn: Any) -> list[str] | None:
    """docstring の "Backends: :guilabel:`a`, :guilabel:`b`." から ["a", "b"] を取り出す。表記が無ければ None。"""
    m = _DOC_BACKENDS.search(inspect.getdoc(fn) or "")
    return re.findall(r"`([^`]+)`", m.group(1)) if m else None


def code_backend_calls(cls: type) -> dict[str, dict[str, Any]]:
    """クラスが「自分で定義している」メソッドの実装を AST で読み、バックエンド選択に関わる書き方を拾う。

    Returns:
        メソッド名 → {
            "lists": get_current_backend([...]) に渡している候補リスト(呼び出しごと),
            "tensor_fallback": _check_for_tensor_backend(...) を通しているか(tensor が無効なら usd に落とす処理),
            "fabric_to_usdrt": `if backend == "fabric": backend = "usdrt"` 型の読み替えがあるか,
        }
    """
    src = inspect.getsource(cls)
    tree = ast.parse(src)
    out: dict[str, dict[str, Any]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        lists: list[list[str]] = []
        tensor_fallback = False
        for sub in ast.walk(node):
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
                if sub.func.attr == "get_current_backend" and sub.args and isinstance(sub.args[0], ast.List):
                    lists.append([e.value for e in sub.args[0].elts if isinstance(e, ast.Constant)])
                if sub.func.attr == "_check_for_tensor_backend":
                    tensor_fallback = True
        body_src = ast.get_source_segment(src, node) or ""
        out[node.name] = {
            "lists": lists,
            "tensor_fallback": tensor_fallback,
            "fabric_to_usdrt": bool(re.search(r'if backend == "fabric":\s*\n\s*backend = "usdrt"', body_src)),
        }
    return out


@bplib.guarded(rec, "N-00")
def env() -> None:
    rec.add("N-00", "バージョンと設定", **bplib.environment())


@bplib.guarded(rec, "N-01")
def static_table() -> None:
    rows: list[dict[str, Any]] = []
    for cls in CLASSES:
        for name, fn in inspect.getmembers(cls, predicate=inspect.isfunction):
            if name.startswith("_"):
                continue
            owner = next(c for c in cls.__mro__ if name in vars(c))  # そのメソッドを定義しているクラス
            impl = code_backend_calls(owner).get(name, {"lists": [], "tensor_fallback": False, "fabric_to_usdrt": False})
            docs = doc_backends(fn)
            first = impl["lists"][0] if impl["lists"] else None
            rows.append(
                {
                    "class": cls.__name__,
                    "method": name,
                    "defined_in": owner.__name__,
                    "doc_backends": docs,
                    "code_candidates": first,  # 最初の get_current_backend 呼び出しの候補リスト
                    "default_backend": first[0] if first else (docs[0] if docs else None),
                    "tensor_fallback_to_usd": impl["tensor_fallback"],
                    "fabric_rewritten_to_usdrt": impl["fabric_to_usdrt"],
                    "doc_matches_code": (docs == first) if (docs is not None and first is not None) else None,
                }
            )
    # 同じメソッド名なのにクラスで既定が違うもの
    by_name: dict[str, dict[str, Any]] = {}
    for r in rows:
        by_name.setdefault(r["method"], {})[r["class"]] = r["default_backend"]
    differing = {m: d for m, d in by_name.items() if len({v for v in d.values() if v}) > 1}
    # docstring と実装の候補が食い違うもの(順序も含む)
    mismatch = [r for r in rows if r["doc_matches_code"] is False]
    # バックエンドの組み合わせの数(定義元クラスで重複を除く)
    combos: dict[str, list[str]] = {}
    seen: set[tuple[str, str]] = set()
    for r in rows:
        key = (r["defined_in"], r["method"])
        if key in seen:
            continue
        seen.add(key)
        combo = ",".join(r["doc_backends"] or ["(表記なし)"])
        combos.setdefault(combo, []).append(f"{r['defined_in']}.{r['method']}")
    rec.add(
        "N-01",
        "docstring の Backends 表記と実装の候補リスト",
        rows=rows,
        same_name_different_default=differing,
        doc_code_mismatch=[
            {k: r[k] for k in ["class", "method", "doc_backends", "code_candidates"]} for r in mismatch
        ],
        backend_combinations={k: {"count": len(v), "methods": v} for k, v in sorted(combos.items())},
    )


def build_scene() -> tuple[Any, RigidPrim, XformPrim]:
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    bplib.make_marker(stage)
    bplib.update()
    return stage, RigidPrim(bplib.BODY), XformPrim(bplib.MARKER)


@bplib.guarded(rec, "N-02")
def return_types() -> None:
    _, rp, xp = build_scene()
    bplib.play(2)
    out: dict[str, Any] = {}
    for b in BACKENDS:
        label = b or "(既定)"
        ctx = backend_utils.use_backend(b) if b else _null()
        with ctx:
            pos, ori = rp.get_world_poses()
            out[f"RigidPrim.get_world_poses/{label}"] = f"{type(pos).__module__}.{type(pos).__qualname__} device={pos.device}"
            vel, _ = rp.get_velocities()
            out[f"RigidPrim.get_velocities/{label}"] = f"{type(vel).__module__}.{type(vel).__qualname__} device={vel.device}"
        if b in (None, "usd", "usdrt", "fabric"):
            st = stage_utils.get_current_stage(backend=b)
            out[f"stage_utils.get_current_stage(backend={b!r})"] = f"{type(st).__module__}.{type(st).__qualname__}"
    rec.add("N-02", "バックエンドを変えたときの返り値の型(Play 中)", types=out)


class _null:
    """use_backend を使わない場合の何もしないコンテキストマネージャ。"""

    def __enter__(self) -> None:
        return None

    def __exit__(self, *exc: Any) -> None:
        return None


# N-03 で呼ぶメソッド: (ラベル, ラッパーの種類, 呼び出し)
CALLS: list[tuple[str, str, Any]] = [
    ("get_world_poses", "rigid", lambda rp, xp: rp.get_world_poses()),
    ("set_world_poses", "rigid", lambda rp, xp: rp.set_world_poses(positions=[[0.0, 0.0, 1.0]])),
    ("get_local_poses", "rigid", lambda rp, xp: rp.get_local_poses()),
    ("get_velocities", "rigid", lambda rp, xp: rp.get_velocities()),
    ("set_velocities", "rigid", lambda rp, xp: rp.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]])),
    ("get_masses", "rigid", lambda rp, xp: rp.get_masses()),
    ("set_masses", "rigid", lambda rp, xp: rp.set_masses([1.0])),
    ("Xform.get_world_poses", "xform", lambda rp, xp: xp.get_world_poses()),
    ("Xform.set_world_poses", "xform", lambda rp, xp: xp.set_world_poses(positions=[[0.0, 0.0, 0.0]])),
    ("Xform.set_local_poses", "xform", lambda rp, xp: xp.set_local_poses(translations=[[0.0, 0.0, 0.0]])),
]


@bplib.guarded(rec, "N-03")
def runtime_trace() -> None:
    table: dict[str, dict[str, Any]] = {}
    detail: dict[str, list[str]] = {}
    for state in ["stopped", "playing"]:
        _, rp, xp = build_scene()
        if state == "playing":
            bplib.play(2)
        for label, _kind, call in CALLS:
            for b in BACKENDS:
                key = f"{state}/{label}/{b or '(既定)'}"
                with bplib.LogCapture() as lc, bplib.spy_namespaces([rp]) as log:
                    try:
                        with backend_utils.use_backend(b) if b else _null():
                            call(rp, xp)
                        err = None
                    except Exception as e:
                        err = f"{type(e).__name__}: {e}"
                table[key] = {
                    "touched": bplib.classify(log),
                    "warnings": sorted(set(lc.messages)),
                    "error": err,
                }
                detail[key] = bplib.touched(log)
        bplib.cleanup()
    rec.add("N-03", "実際に触った名前空間(=実際に使われたバックエンド)", table=table, touched_symbols=detail)


@bplib.guarded(rec, "N-04")
def other_meanings() -> None:
    bplib.new_stage()
    sig = str(inspect.signature(omni.physics.tensors.create_simulation_view))
    # 非推奨 API(extsDeprecated/isaacsim.core.prims)で backend が何を指すかを、ソースから拾う
    dep = r"C:\isaacsim-6.2.0\extsDeprecated\isaacsim.core.prims\isaacsim\core\prims\impl\single_rigid_prim.py"
    with open(dep, encoding="utf-8") as f:
        dep_lines = [f"L{i}: {line.strip()}" for i, line in enumerate(f, 1) if "_backend = " in line or "_backend_utils = " in line]
    rec.add(
        "N-04",
        "「backend」という語の別の意味",
        simulation_manager_get_backend=SimulationManager.get_backend(),
        simulation_manager_active_engine=SimulationManager.get_active_physics_engine(),
        create_simulation_view_signature=sig,
        experimental_use_backend_annotation=str(inspect.signature(backend_utils.use_backend).parameters["backend"].annotation),
        deprecated_single_rigid_prim_backend_lines=dep_lines,
    )


@bplib.guarded(rec, "N-05")
def rules() -> None:
    out: dict[str, Any] = {}

    def attempt(label: str, fn: Any) -> None:
        with bplib.LogCapture() as lc:
            try:
                out[label] = {"result": fn(), "warnings": lc.messages}
            except Exception as e:
                out[label] = {"error": f"{type(e).__name__}: {e}", "warnings": lc.messages}

    def get_in(backend: str, supported: list[str], **kw: Any) -> str:
        with backend_utils.use_backend(backend, **kw):
            return backend_utils.get_current_backend(supported)

    attempt("既定(use_backend なし) supported=[tensor,usd]", lambda: backend_utils.get_current_backend(["tensor", "usd"]))
    attempt("use_backend(usdrt) supported=[tensor,usd]", lambda: get_in("usdrt", ["tensor", "usd"]))
    attempt("use_backend(usdrt, raise_on_unsupported) supported=[tensor,usd]", lambda: get_in("usdrt", ["tensor", "usd"], raise_on_unsupported=True))

    # 入れ子: 内側を抜けると外側の指定に戻るか
    def nested() -> list[str]:
        seen = []
        with backend_utils.use_backend("usd"):
            seen.append(backend_utils.get_current_backend(["tensor", "usd"]))
            with backend_utils.use_backend("tensor"):
                seen.append(backend_utils.get_current_backend(["tensor", "usd"]))
            seen.append(backend_utils.get_current_backend(["tensor", "usd"]))
        seen.append(backend_utils.get_current_backend(["tensor", "usd"]))
        return seen

    attempt("入れ子 usd > tensor > (抜ける) > (抜ける)", nested)

    # スレッドローカル: メインスレッドの use_backend は別スレッドから見えるか
    def thread_local() -> dict[str, str]:
        res: dict[str, str] = {}
        with backend_utils.use_backend("usd"):
            res["main"] = backend_utils.get_current_backend(["tensor", "usd"])
            t = threading.Thread(target=lambda: res.__setitem__("worker", backend_utils.get_current_backend(["tensor", "usd"])))
            t.start()
            t.join()
        return res

    attempt("スレッドローカル", thread_local)

    # FSD が無効な環境では usdrt / fabric を指定した瞬間に例外になる(_fsd_enabled は import 時に設定から読まれる)
    def fsd_off() -> str:
        # _fsd_enabled は公開モジュール(backend)ではなく実装モジュール(impl.backend)のグローバル変数
        import isaacsim.core.experimental.utils.impl.backend as impl_backend

        saved = impl_backend._fsd_enabled
        impl_backend._fsd_enabled = False  # FSD 無効の環境を模擬する
        try:
            with backend_utils.use_backend("fabric"):
                return "no error"
        finally:
            impl_backend._fsd_enabled = saved

    attempt("FSD 無効を模擬して use_backend(fabric)", fsd_off)

    # tensor が無効(Stop 中)のときの raise_on_fallback
    _, rp, _ = build_scene()

    def stopped_tensor(**kw: Any) -> Any:
        with backend_utils.use_backend("tensor", **kw):
            return bplib._r(rp.get_world_poses()[0].numpy()[0])

    attempt("Stop 中 use_backend(tensor) get_world_poses", stopped_tensor)
    attempt("Stop 中 use_backend(tensor, raise_on_fallback) get_world_poses", lambda: stopped_tensor(raise_on_fallback=True))
    attempt("Stop 中 既定 get_world_poses", lambda: bplib._r(rp.get_world_poses()[0].numpy()[0]))
    rec.add("N-05", "use_backend の規則", cases=out)


env()
static_table()
return_types()
runtime_trace()
other_meanings()
rules()
bplib.finish(rec)
