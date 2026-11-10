"""G01: GPU 物理で「Play 中に USD / Fabric に書いた値が PhysX に届かない」のは、何が原因か。

M01 / P01 を --gpu で実行して分かったこと:
    Play 中の USD への位置・速度の書き込みも、Fabric への位置の書き込みも、PhysX(tensor で読む値)に届かない。
    質量の USD 書き込みは届く。tensor の書き込みは届く。

GPU 構成(SimulationManager.set_device("cuda:0"))は 3 つを同時に変える:
    1. PhysicsScene の GPU dynamics / GPU broadphase
    2. omni.physx.fabric を有効にする
    3. /physics/suppressReadback = true
これを 1 つずつ切り替えて、どれが効いているかを調べる。

注意: SimulationManager.get_device() は PhysicsScene ではなく /physics/suppressReadback を見て "cuda" / "cpu" を返す
(suppressReadback=false なら GPU シーンでも "cpu")。そこで PhysX が本当に GPU で解いているかは、
IPhysxStatistics の gpu_mem_heap(GPU メモリ使用量。CPU 実行なら 0)で別に確かめる。

各構成 × 各書き込みで、毎回ステージを作り直す(重力 0、剛体は z=1 に静止、質量 1):
    Play(update 3 回)→ 書く → update(位置・質量は 1 回、速度は 5 回)→ tensor で読む → Stop
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any, Callable

import carb.settings
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd, UsdPhysics

rec = bplib.Recorder("g01_gpu_factors")

SUPPRESS_READBACK: str = "/physics/suppressReadback"

# (名前, PhysicsScene を GPU にするか, omni.physx.fabric を有効にするか, suppressReadback)
CONFIGS: list[tuple[str, bool, bool, bool]] = [
    ("シーンCPU / fabric無 / readback有 (=既定構成)", False, False, False),
    ("シーンCPU / fabric有 / readback有 (=physx_fabric構成)", False, True, False),
    ("シーンCPU / fabric有 / readback抑止", False, True, True),
    ("シーンGPU / fabric有 / readback抑止 (=GPU構成)", True, True, True),
    ("シーンGPU / fabric有 / readback有", True, True, False),
    ("シーンGPU / fabric無 / readback有", True, False, False),
    ("シーンGPU / fabric無 / readback抑止", True, False, True),
]


def gpu_mem_heap(stage: Usd.Stage) -> int:
    """Play 中の PhysicsScene の GPU メモリ使用量 [byte]。PhysX が CPU で解いていれば 0。"""
    from omni.physx import get_physx_statistics_interface
    from omni.physx.bindings._physx import PhysicsSceneStats
    from pxr import PhysicsSchemaTools

    stats = PhysicsSceneStats()
    ok = get_physx_statistics_interface().get_physx_scene_statistics(
        bplib._stage_id(stage), PhysicsSchemaTools.sdfPathToInt("/World/physicsScene"), stats
    )
    return int(stats.gpu_mem_heap) if ok else -1


def apply_config(gpu: bool, fabric: bool, suppress: bool) -> None:
    """シミュレーションデバイス・omni.physx.fabric・suppressReadback をこの順で設定する(set_device が後 2 つも変えるため)。"""
    SimulationManager.set_device("cuda:0" if gpu else "cpu")
    SimulationManager.enable_fabric(fabric)
    carb.settings.get_settings().set_bool(SUPPRESS_READBACK, suppress)
    bplib.update()


def write_usd_translate(stage: Usd.Stage) -> None:
    bplib.usd_set_translate(stage, bplib.BODY, (0.0, 0.0, 2.0))


def write_fabric_translate(stage: Usd.Stage) -> None:
    bplib.fabric_set_translate(bplib.BODY, (0.0, 0.0, 3.0))


def write_usd_velocity(stage: Usd.Stage) -> None:
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bplib.BODY)).CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.6, 0.0))


def write_usd_mass(stage: Usd.Stage) -> None:
    UsdPhysics.MassAPI(stage.GetPrimAtPath(bplib.BODY)).GetMassAttr().Set(5.0)


# (名前, 書き込み関数, 書いた後の update 回数, PhysX に届いたと判定する関数(最後の update 後の tensor 値で判定))
WRITES: list[tuple[str, Callable[[Usd.Stage], None], int, Callable[[dict[str, Any]], bool]]] = [
    ("USD 位置 z=2", write_usd_translate, 1, lambda t: abs(t["position"][2] - 2.0) < 1e-3),
    ("Fabric 位置 z=3", write_fabric_translate, 1, lambda t: abs(t["position"][2] - 3.0) < 1e-3),
    ("USD 速度 y=0.6", write_usd_velocity, 5, lambda t: abs(t["velocity"][1] - 0.6) < 1e-3),
    ("USD 質量 5", write_usd_mass, 1, lambda t: abs(t["mass"] - 5.0) < 1e-3),
]


@bplib.guarded(rec, "G-00")
def env() -> None:
    rec.add("G-00", "バージョンと設定(起動時)", **bplib.environment())


def make(cfg: tuple[str, bool, bool, bool]) -> Callable[[], None]:
    name, gpu, fabric, suppress = cfg

    @bplib.guarded(rec, f"G/{name}")
    def run() -> None:
        out: dict[str, Any] = {}
        apply_config(gpu, fabric, suppress)
        out["settings"] = {
            "SimulationManager.get_device()": str(SimulationManager.get_device()),
            "fabric": SimulationManager.is_fabric_enabled(),
            SUPPRESS_READBACK: carb.settings.get_settings().get(SUPPRESS_READBACK),
        }
        for wname, write, n, reached in WRITES:
            stage = bplib.new_stage(gravity=0.0, gpu=gpu)
            bplib.make_body(stage)
            bplib.update()
            bplib.play(3)
            out.setdefault("physx_gpu_mem_heap_bytes", gpu_mem_heap(stage))
            write(stage)
            trace: list[dict[str, Any]] = []  # update ごとの tensor 値
            for _ in range(n):
                bplib.update(1)
                trace.append(bplib.tensor_state(bplib.BODY))
            t = trace[-1]
            out[wname] = {"reached_physx": reached(t), "tensor": t, "tensor_trace": trace, "snapshot": bplib.snapshot(stage)}
            bplib.stop(1)
        rec.add(f"G/{name}", "Play 中の USD / Fabric 書き込みが PhysX に届くか", **out)

    return run


env()
for _cfg in CONFIGS:
    make(_cfg)()
apply_config(False, False, False)
bplib.finish(rec)
