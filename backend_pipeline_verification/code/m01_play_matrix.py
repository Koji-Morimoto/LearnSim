"""M01: Play ON / OFF の状態で、バックエンドを指定して RigidPrim の書き換え系メソッドを呼ぶとどうなるか。
その後 ON / OFF を反転させると何が起きるか。

--physx-fabric / --gpu 付きで実行すると、その構成(omni.physx.fabric 有効 / GPU 物理)で同じ測定をする。

マトリクス:
    開始状態   : stopped(Play OFF) / playing(Play ON)
    メソッド   : set_world_poses(positions=(1,2,3)) / set_velocities(linear=(0,0.6,0)) / set_masses(5.0)
    バックエンド: 指定なし(既定) / tensor / usd / usdrt / fabric   (use_backend で指定)

1 ケースの手順(毎回ステージを作り直す。重力 0、剛体は z=1 に静止、質量 1):
    stopped 開始: 呼ぶ → update 1 回 → Play(update 1 回)→ さらに update 9 回 → Stop
    playing 開始: Play(update 3 回)→ 呼ぶ → update 1 回 → Stop → もう一度 Play(update 1 回)→ さらに update 9 回
各段階で USD / Fabric / PhysX / tensor の全層を観測する(bplib.snapshot)。
呼び出しの間は、ラッパーが触った名前空間(=実際に使われたバックエンド)と carb の警告も記録する。

追加:
    M-90  tensor 専用メソッド(apply_forces)は use_backend("usd") を指定しても無視されるか、Stop 中はどうなるか
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any, Callable

import isaacsim.core.experimental.utils.backend as backend_utils
from isaacsim.core.experimental.prims import RigidPrim
from pxr import Usd

rec = bplib.Recorder("m01_play_matrix")

TARGET_POS: list[float] = [1.0, 2.0, 3.0]  # set_world_poses で書く位置 [m]
TARGET_VEL: list[float] = [0.0, 0.6, 0.0]  # set_velocities で書く並進速度 [m/s]。10 ステップ(1/6 s)で y が 0.1 m 進む
TARGET_MASS: float = 5.0  # set_masses で書く質量 [kg]

BACKENDS: list[str | None] = [None, "tensor", "usd", "usdrt", "fabric"]  # None = use_backend を使わない
OPS: dict[str, Callable[[RigidPrim], None]] = {
    "set_world_poses": lambda rp: rp.set_world_poses(positions=[TARGET_POS]),
    "set_velocities": lambda rp: rp.set_velocities(linear_velocities=[TARGET_VEL]),
    "set_masses": lambda rp: rp.set_masses([TARGET_MASS]),
}


def scene() -> tuple[Usd.Stage, RigidPrim]:
    """重力 0、剛体 1 個のステージと、その RigidPrim ラッパー(Stop 中に作る)を返す。"""
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    bplib.update()
    return stage, RigidPrim(bplib.BODY)


def call_with_backend(rp: RigidPrim, op: str, backend: str | None) -> dict[str, Any]:
    """use_backend(backend) の中で op を呼び、触った層・警告・例外を返す。"""
    return bplib.call_with_backend([rp], lambda: OPS[op](rp), backend)


def run_case(start: str, op: str, backend: str | None) -> dict[str, Any]:
    stage, rp = scene()
    snap: Callable[[], dict[str, Any]] = lambda: bplib.snapshot(stage)
    res: dict[str, Any] = {}
    if start == "playing":
        bplib.play(3)
    res["0_before"] = snap()
    res["call"] = call_with_backend(rp, op, backend)
    res["1_after_call"] = snap()
    res["1_readback"] = bplib.rigid_readback(rp)
    bplib.update()
    res["2_after_update1"] = snap()
    if start == "stopped":
        bplib.play(1)
        res["3_toggled_play_update1"] = snap()
        bplib.update(9)
        res["4_play_update10"] = snap()
        res["4_readback"] = bplib.rigid_readback(rp)
        bplib.stop(1)
        res["5_stopped_again"] = snap()
    else:
        bplib.stop(1)
        res["3_toggled_stop"] = snap()
        res["3_readback"] = bplib.rigid_readback(rp)
        bplib.play(1)
        res["4_replay_update1"] = snap()
        bplib.update(9)
        res["5_replay_update10"] = snap()
    return res


@bplib.guarded(rec, "M-00")
def env() -> None:
    rec.add("M-00", "バージョンと設定", **bplib.environment())


def make_case(start: str, op: str, backend: str | None) -> Callable[[], None]:
    test_id = f"M/{start}/{op}/{backend or '(既定)'}"

    @bplib.guarded(rec, test_id)
    def case() -> None:
        rec.add(test_id, f"開始={start} メソッド={op} use_backend={backend}", **run_case(start, op, backend))

    return case


@bplib.guarded(rec, "M-90")
def tensor_only_method() -> None:
    out: dict[str, Any] = {}
    for start in ["stopped", "playing"]:
        _, rp = scene()
        if start == "playing":
            bplib.play(3)
        with bplib.LogCapture() as lc, bplib.spy_namespaces([rp]) as log:
            try:
                with backend_utils.use_backend("usd"):
                    rp.apply_forces([[0.0, 0.0, 1.0]])
                err = None
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
        out[start] = {"touched": bplib.classify(log), "warnings": lc.messages, "error": err}
        bplib.cleanup()
    rec.add("M-90", "tensor 専用の apply_forces を use_backend('usd') の中で呼ぶ", **out)


env()
for _start in ["stopped", "playing"]:
    for _op in OPS:
        for _b in BACKENDS:
            make_case(_start, _op, _b)()
tensor_only_method()
bplib.finish(rec)
