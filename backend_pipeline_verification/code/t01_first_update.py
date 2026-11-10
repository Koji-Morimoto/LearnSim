"""T01: Play 直後の最初の update で、物理は何ステップ進むか。

U01 / M01 の追跡で、速度 0.6 m/s の剛体が Play 後の最初の update だけ 0.03 m(3 ステップ分)進み、
以降は 1 update で 0.01 m(1 ステップ)だった。物理ステップのイベントを数えて、その内訳を確かめる。

SimulationManager.initialize_physics() は Play 時に USD を読み込んだあと simulate(dt, 0) を 1 回呼ぶ
(simulation_manager.py の initialize_physics の最後)。これがステップとして数えられるかも見る。
--physx-fabric / --gpu 付きで実行すると、その構成で同じ測定をする。
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any

from omni.physx import get_physx_interface
from pxr import Gf, UsdPhysics

rec = bplib.Recorder("t01_first_update")


class StepCounter:
    """物理ステップの後イベントを数え、各ステップの dt を記録する。"""

    def __init__(self) -> None:
        self.dts: list[float] = []
        self._sub = get_physx_interface().subscribe_physics_step_events(lambda dt: self.dts.append(round(dt, 5)))

    def take(self) -> list[float]:
        out, self.dts = self.dts, []
        return out


@bplib.guarded(rec, "T-01")
def first_updates() -> None:
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bplib.BODY)).CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.6, 0.0))
    bplib.update()
    counter = StepCounter()
    out: dict[str, Any] = {"physics_dt": None}
    bplib.timeline().play()
    out["after_play_call_no_update"] = {"steps_dt": counter.take()}
    for i in range(1, 5):
        bplib.update(1)
        s = bplib.snapshot(stage)
        out[f"update{i}"] = {
            "steps_dt": counter.take(),
            "time": s["time"],
            "y": (s["tensor"].get("position") or [None] * 3)[1],
        }
    from isaacsim.core.simulation_manager import SimulationManager

    out["physics_dt"] = SimulationManager.get_physics_dt()
    rec.add("T-01", "Play 後の各 update で物理ステップが何回走るか(速度 0.6 m/s、1 ステップ 0.01 m)", **out)


@bplib.guarded(rec, "T-00")
def env() -> None:
    rec.add("T-00", "バージョンと設定", **bplib.environment())


env()
first_updates()
bplib.finish(rec)
