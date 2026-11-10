"""X01: GitHub のソース(NVIDIA-Omniverse/PhysX タグ 110.3-omni-and-physx-5.9.1 の omni/ 以下)を読んで立てた仮説を、実測で確かめる。

ソースは ../../_external/PhysX-110.3/omni/ovruntime/source/ に置いた(手元の omni.physx は 110.4.4 なので完全一致ではない)。

X-A  GPU 構成の「Pause 中に update を回して再開すると Fabric が止まる」(Q-92)
     ソース上の仮説:
       omni.physx.fabric/plugins/FabricManager.cpp
         pause()  : PhysX の ePaused イベントで呼ばれ、mDirectGpuHelper.clear() で「GPU から Fabric に書き出す剛体の一覧」を空にする
         resume() : eResumed で呼ばれ、initializeRigidBodyBatched() で剛体を登録し直すはずだが、
                    mInitialTransformation(Play 開始時の姿勢の記録。stop() でしか消えない)に載っている剛体は
                    「初期化済み」として飛ばす → registerRigidBody() が呼ばれず、一覧は空のまま
       Pause 中に update を回さないと ePaused 自体が出ず、clear() されない、という予想
     確かめること: PhysX のシミュレーションイベント(PAUSED / RESUMED)が、Pause 中に update を回した場合 / 回さない場合に届くか

X-B  physx_fabric 構成の「Play 中に新規作成した physics:velocity が 1 ステップで 0 になる」(U-G1)
     ソース上の手がかり:
       omni.physx.fabric は Play 開始時に Fabric の physics:velocity を float3(役割なし)で作る(FabricManager.cpp mFloat3Type)
       USD の physics:velocity は vector3f(役割=ベクトル)
       omni.physx の change tracking は Fabric 起点の変更を Fabric の値で読む(PhysXTools.h getValue)
     確かめること: Fabric の physics:velocity の型名と値が、USD で作った前後と各ステップでどう変わるか
                   (属性を Play 前から作っておく場合 / 作らない場合)
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any

from omni.physx import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent
from pxr import Gf, UsdPhysics

rec = bplib.Recorder("x01_source_checks")


class SimEventLog:
    """PhysX のシミュレーションイベント(v2 ストリーム)を受け取った順に記録する。"""

    NAMES: dict[int, str] = {int(SimulationEvent.RESUMED): "RESUMED", int(SimulationEvent.PAUSED): "PAUSED", int(SimulationEvent.STOPPED): "STOPPED"}

    def __init__(self) -> None:
        self.events: list[str] = []
        stream = get_physx_interface().get_simulation_event_stream_v2()
        self._sub = stream.create_subscription_to_pop(lambda e: self.events.append(self.NAMES.get(int(e.type), str(e.type))))

    def take(self) -> list[str]:
        out, self.events = self.events, []
        return out


def fabric_velocity(path: str = bplib.BODY) -> dict[str, Any]:
    """Fabric 上の physics:velocity の型名と値。"""
    attr = bplib._rt_stage().GetPrimAtPath(path).GetAttribute("physics:velocity")
    if not attr or not attr.HasValue():
        return {"type": None, "value": None}
    return {"type": str(attr.GetTypeName()), "value": bplib._r(attr.Get())}


@bplib.guarded(rec, "X-00")
def env() -> None:
    rec.add("X-00", "バージョンと設定", **bplib.environment())


@bplib.guarded(rec, "X-A")
def pause_events() -> None:
    from isaacsim.core.experimental.prims import RigidPrim

    out: dict[str, Any] = {}
    for variant, paused_updates in [("Pause 中に update 1 回", 1), ("Pause 中に update なし", 0)]:
        stage = bplib.new_stage(gravity=0.0)
        bplib.make_body(stage)
        bplib.update()
        rp = RigidPrim(bplib.BODY)
        log = SimEventLog()
        bplib.play(3)
        rp.set_velocities(linear_velocities=[[0.0, 0.6, 0.0]])
        bplib.update(2)
        res: dict[str, Any] = {"play": log.take()}
        bplib.timeline().pause()
        bplib.update(paused_updates)
        res["pause"] = log.take()
        bplib.timeline().play()
        bplib.update(3)
        res["resume_and_3_updates"] = log.take()
        s = bplib.snapshot(stage)
        res["after_resume"] = {"tensor_y": s["tensor"].get("position"), "fabric_y": s["fabric"].get("local")}
        bplib.stop(1)
        res["stop"] = log.take()
        out[variant] = res
    rec.add("X-A", "Pause → 再開で PhysX の PAUSED / RESUMED イベントが届くか", **out)


@bplib.guarded(rec, "X-B")
def velocity_attr_type() -> None:
    out: dict[str, Any] = {}
    for variant, preauthor in [("属性なしから Play 中に作る", False), ("Play 前に属性を作っておく", True)]:
        stage = bplib.new_stage(gravity=0.0)
        bplib.make_body(stage)
        api = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bplib.BODY))
        if preauthor:
            api.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        bplib.update()
        res: dict[str, Any] = {"0_stopped": fabric_velocity()}
        bplib.play(3)
        res["1_playing"] = fabric_velocity()
        api.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.6, 0.0))
        res["2_after_usd_write"] = fabric_velocity()
        for i in range(1, 4):
            bplib.update(1)
            res[f"3_update{i}"] = {"fabric": fabric_velocity(), "tensor_vy": (bplib.tensor_state(bplib.BODY).get("velocity") or [None] * 3)[1]}
        bplib.stop(1)
        res["4_stopped"] = fabric_velocity()
        bplib.play(1)
        res["5_replay_update1"] = {"fabric": fabric_velocity(), "tensor_vy": (bplib.tensor_state(bplib.BODY).get("velocity") or [None] * 3)[1]}
        out[variant] = res
        bplib.cleanup()
    rec.add("X-B", "Fabric の physics:velocity の型名と値の変化", **out)


env()
pause_events()
velocity_attr_type()
bplib.finish(rec)
