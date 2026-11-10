"""Q01: Pause(一時停止)中に、バックエンドを指定して RigidPrim の書き換え系メソッドを呼ぶとどうなるか。
その後 Pause → Play(再開)、Pause → Stop と反転させると何が起きるか。

--physx-fabric / --gpu 付きで実行すると、その構成で同じ測定をする。

マトリクス(M01 と同じ値):
    メソッド   : set_world_poses(positions=(1,2,3)) / set_velocities(linear=(0,0.6,0)) / set_masses(5.0)
    バックエンド: 指定なし(既定) / tensor / usd / usdrt / fabric

1 ケースの手順(毎回ステージを作り直す。重力 0、剛体は z=1 に静止、質量 1):
    Play(update 3 回)→ Pause(update 1 回)→ 呼ぶ → Pause のまま update 3 回 → 再開(update 1 回)→ さらに update 9 回 → Stop
各段階で USD / Fabric / PhysX / tensor の全層を観測する(bplib.snapshot)。

追加:
    Q-90  Pause 中に書いて、再開せずに Stop したら残るか(既定 / usd の 2 通り × 3 メソッド)
    Q-91  Pause 中の update で物理が進まないこと、tensor ビューが有効なままであることの確認
    Q-92  Pause → 再開の後も、PhysX の結果が Fabric(描画が読む層)に出続けるか(Pause 中に何も書かない / tensor で書く)
          さらにその後 Stop → Play で元に戻るか
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any, Callable

from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd

rec = bplib.Recorder("q01_pause")

TARGET_POS: list[float] = [1.0, 2.0, 3.0]  # set_world_poses で書く位置 [m]
TARGET_VEL: list[float] = [0.0, 0.6, 0.0]  # set_velocities で書く並進速度 [m/s]
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


def pause(n_updates: int = 1) -> None:
    """タイムラインを一時停止して n_updates 回 update する。"""
    bplib.timeline().pause()
    bplib.update(n_updates)


def run_case(op: str, backend: str | None) -> dict[str, Any]:
    stage, rp = scene()
    snap: Callable[[], dict[str, Any]] = lambda: bplib.snapshot(stage)
    res: dict[str, Any] = {}
    bplib.play(3)
    pause(1)
    res["0_paused"] = snap()
    res["call"] = bplib.call_with_backend([rp], lambda: OPS[op](rp), backend)
    res["1_after_call"] = snap()
    res["1_readback"] = bplib.rigid_readback(rp)
    bplib.update(3)
    res["2_paused_update3"] = snap()
    bplib.play(1)
    res["3_resumed_update1"] = snap()
    bplib.update(9)
    res["4_resumed_update10"] = snap()
    bplib.stop(1)
    res["5_stopped"] = snap()
    res["5_readback"] = bplib.rigid_readback(rp)
    return res


@bplib.guarded(rec, "Q-00")
def env() -> None:
    rec.add("Q-00", "バージョンと設定", **bplib.environment())


def make_case(op: str, backend: str | None) -> Callable[[], None]:
    test_id = f"Q/{op}/{backend or '(既定)'}"

    @bplib.guarded(rec, test_id)
    def case() -> None:
        rec.add(test_id, f"Pause 中に {op} use_backend={backend}", **run_case(op, backend))

    return case


@bplib.guarded(rec, "Q-90")
def pause_then_stop() -> None:
    """Pause 中に書いて、再開せずに Stop。その後もう一度 Play して最初の値を見る。"""
    out: dict[str, Any] = {}
    for op in OPS:
        for backend in [None, "usd"]:
            stage, rp = scene()
            bplib.play(3)
            pause(1)
            call = bplib.call_with_backend([rp], lambda: OPS[op](rp), backend)
            bplib.stop(1)
            stopped = bplib.snapshot(stage)
            bplib.play(1)
            replay = bplib.snapshot(stage)
            bplib.stop(1)
            out[f"{op}/{backend or '(既定)'}"] = {"call": call, "stopped": stopped, "replay_update1": replay}
    rec.add("Q-90", "Pause 中に書いて、再開せずに Stop → 再 Play", **out)


@bplib.guarded(rec, "Q-91")
def pause_does_not_step() -> None:
    """速度 0.6 で動いている剛体を Pause し、update を回しても位置・時刻が進まないか。tensor ビューは有効なままか。"""
    stage, rp = scene()
    bplib.play(3)
    rp.set_velocities(linear_velocities=[TARGET_VEL])
    bplib.update(3)
    moving = bplib.snapshot(stage)
    pause(1)
    paused1 = bplib.snapshot(stage)
    bplib.update(10)
    paused11 = bplib.snapshot(stage)
    rec.add(
        "Q-91",
        "Pause 中の update で物理は進むか",
        moving=moving,
        paused_update1=paused1,
        paused_update11=paused11,
        wrapper_tensor_valid_while_paused=rp.is_physics_tensor_entity_valid(),
        timeline_state=bplib.timeline_state(),
    )


def _follow_record(stage: Usd.Stage) -> dict[str, Any]:
    """tensor と Fabric の y 位置を記録する。

    Fabric は CPU 側(usdrt の読み出し)と cuda:0 側(warp fabricarray。描画が見る側)の両方を読む。
    """
    s = bplib.snapshot(stage)
    dev = bplib.fabric_device_state(bplib.BODY, "cuda:0")
    return {
        "tensor_y": (s["tensor"].get("position") or [None] * 3)[1],
        "fabric_y": (s["fabric"].get("local") or [None] * 3)[1],
        "fabric_cuda_world_y": dev["world"][1] if isinstance(dev.get("world"), list) else dev,
        "fabric_cuda_local_y": dev["local"][1] if isinstance(dev.get("local"), list) else dev,
    }


def _fabric_follow_trace(stage: Usd.Stage, n: int, manual_fabric_update: bool = False) -> list[dict[str, Any]]:
    """n 回 update し、update ごとに _follow_record で記録する。

    Args:
        manual_fabric_update: True なら update のたびに omni.physx.fabric の update(time, dt) を手で呼んでから記録する。
    """
    out: list[dict[str, Any]] = []
    for _ in range(n):
        bplib.update(1)
        if manual_fabric_update:
            from omni.physxfabric import get_physx_fabric_interface

            get_physx_fabric_interface().update(bplib.timeline().get_current_time(), 1.0 / 60.0)
        out.append(_follow_record(stage))
    return out


Q92_VARIANTS: list[str] = [
    "Pause中は何もしない",
    "Pause中にtensorで速度を書く",
    "GPU interop 有効(/physics/fabricUseGPUInterop=true。既定は false)",
    "再開後に毎回 physx_fabric.update() を手で呼ぶ",
    "Pause 中に update を回さずに再開",
]


@bplib.guarded(rec, "Q-92")
def fabric_after_resume() -> None:
    """速度 0.6 で動かした剛体を Pause → 再開し、Fabric が PhysX に追従し続けるかを update ごとに見る。"""
    import carb.settings

    settings = carb.settings.get_settings()
    interop_key = "/physics/fabricUseGPUInterop"
    out: dict[str, Any] = {}
    for variant in Q92_VARIANTS:
        if variant.startswith("再開後に毎回") and not SimulationManager.is_fabric_enabled():
            out[variant] = "omni.physx.fabric が無効な構成では対象外"
            continue
        old_interop = settings.get(interop_key)
        if variant.startswith("GPU interop"):
            settings.set_bool(interop_key, True)
        try:
            stage, rp = scene()
            bplib.play(3)
            rp.set_velocities(linear_velocities=[TARGET_VEL])
            res: dict[str, Any] = {"before_pause": _fabric_follow_trace(stage, 3)}
            pause(0 if variant.startswith("Pause 中に update を回さず") else 1)
            if variant == "Pause中にtensorで速度を書く":
                rp.set_velocities(linear_velocities=[[0.0, 1.2, 0.0]])
            bplib.play(0)
            res["after_resume"] = _fabric_follow_trace(stage, 5, manual_fabric_update=variant.startswith("再開後に毎回"))
            bplib.stop(1)
            bplib.play(1)  # 1 update して RigidPrim の tensor ビューを作り直させてから書く
            rp.set_velocities(linear_velocities=[TARGET_VEL])
            res["after_stop_and_replay"] = _fabric_follow_trace(stage, 3)
            res[interop_key] = settings.get(interop_key)
            out[variant] = res
            bplib.cleanup()
        finally:
            if old_interop is not None:
                settings.set_bool(interop_key, bool(old_interop))
    rec.add("Q-92", "Pause → 再開の後、Fabric は PhysX に追従し続けるか", **out)


env()
for _op in OPS:
    for _b in BACKENDS:
        make_case(_op, _b)()
pause_then_stop()
pause_does_not_step()
fabric_after_resume()
bplib.finish(rec)
