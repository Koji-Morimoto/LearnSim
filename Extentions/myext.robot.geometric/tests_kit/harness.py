"""Kit 側テストの土台。headless の Isaac Sim(描画なし)を別プロセスで起動し、テスト関数を順に走らせる。

    import harness
    app = harness.start()            # ここより後で pxr / omni / myext を import する

    @harness.test
    def test_something(): ...

    harness.finish()                 # 結果を表示し、失敗があれば終了コード 1 で終わる
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import traceback
from typing import Any, Callable

HERE = os.path.dirname(os.path.abspath(__file__))
UR10E_USD = r"C:\isaacsim-6.2.0\exts\isaacsim.asset.transformer.rules\data\tests\ur10e\ur10e.usd"
FRANKA_URDF = r"C:\isaacsim-6.2.0\exts\isaacsim.asset.importer.urdf\data\urdf\robots\franka_description\robots\panda_arm_hand.urdf"

_app: Any = None
_tests: list[Callable[[], None]] = []
_results: list[dict[str, Any]] = []
_notes: dict[str, Any] = {}


def start(timeout_s: float = 900.0) -> Any:
    global _app

    def die() -> None:
        print(f"[WATCHDOG] {timeout_s}s 経過のため終了", flush=True)
        os._exit(3)

    timer = threading.Timer(timeout_s, die)
    timer.daemon = True
    timer.start()
    from isaacsim import SimulationApp

    _app = SimulationApp({"headless": True}, experience=os.path.join(HERE, "geo_tests.kit"))
    return _app


def update(n: int = 1) -> None:
    for _ in range(n):
        _app.update()


def timeline():
    import omni.timeline

    return omni.timeline.get_timeline_interface()


def play(n: int = 2) -> None:
    timeline().play()
    update(n)


def stop(n: int = 2) -> None:
    timeline().stop()
    update(n)


def new_stage(gravity: float = 0.0):
    """Z-up、1 unit = 1 m の空ステージ。PhysicsScene は CPU 物理。"""
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    if not timeline().is_stopped():
        stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    update(2)
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr("MBP")
    update(2)
    return stage


def note(key: str, value: Any) -> None:
    """テストの中で測った値を結果ファイルに残す。"""
    _notes[key] = value
    print(f"[NOTE] {key}: {json.dumps(value, ensure_ascii=False, default=str)[:1200]}", flush=True)


def test(fn: Callable[[], None]) -> Callable[[], None]:
    _tests.append(fn)
    return fn


def finish() -> None:
    only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--only=")]
    for fn in _tests:
        if only and not any(o in fn.__name__ for o in only):
            continue
        t0 = time.perf_counter()
        try:
            fn()
            _results.append({"name": fn.__name__, "ok": True, "seconds": round(time.perf_counter() - t0, 2)})
            print(f"[PASS] {fn.__name__} ({time.perf_counter() - t0:.1f}s)", flush=True)
        except Exception:
            tb = traceback.format_exc()
            _results.append({"name": fn.__name__, "ok": False, "seconds": round(time.perf_counter() - t0, 2), "traceback": tb})
            print(f"[FAIL] {fn.__name__}\n{tb}", flush=True)
        try:
            if not timeline().is_stopped():
                stop()
        except Exception:
            pass
    failed = [r["name"] for r in _results if not r["ok"]]
    with open(os.path.join(HERE, "last_run.json"), "w", encoding="utf-8") as f:
        json.dump({"results": _results, "notes": _notes}, f, ensure_ascii=False, indent=1, default=str)
    print(f"[SUMMARY] {len(_results) - len(failed)} passed, {len(failed)} failed {failed}", flush=True)
    sys.stdout.flush()
    os._exit(1 if failed else 0)
