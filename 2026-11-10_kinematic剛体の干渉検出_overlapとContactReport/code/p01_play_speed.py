"""P01: kinematic 剛体だけのシーンで、Play 中の update 1 回にかかる時間を測る(headless、描画なし、CPU 物理)。

N 個の箱(一辺 0.2 m)を y 方向に 0.25 m 間隔で並べる(すき間 5 cm)。偶数番の箱を 1 フレームおきに +0.06 m 動かすと
上隣の箱に 1 cm めり込み、次のフレームで戻す。つまり毎フレーム N/2 個が動き、約 N/2 組の接触が始まるか終わる。

測る構成:
    report=True   ContactReportAPI + 報告フラグあり(Contact Report を使う構成)
    report=False  どちらも無し(overlap だけを使う構成)
測る項目(中央値、ms):
    stop_idle           Stop 中、何もしない update
    play_idle           Play 中、何も動かさない update
    play_usd            Play 中、N/2 個を USD(Sublayer の Sdf を ChangeBlock)で動かす: 書き込み + update
    play_fabric         Play 中、N/2 個を Fabric 一括(XformPrim.set_local_poses)で動かす: 書き込み + update
    overlap_all         Play 中、全コライダーに overlap_shape を 1 回ずつ
ループの周波数制限(rateLimit)は外して測る。
"""

from __future__ import annotations

import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code")
import bplib

bplib.RESULT_DIR = os.path.join(os.path.dirname(HERE), "results")
app = bplib.start(render=False)

from typing import Any

import carb
import carb.settings
import numpy as np
import warp as wp
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

rec = bplib.Recorder("p01_play_speed")
settings = carb.settings.get_settings()
for key in ("/app/runLoops/main/rateLimitEnabled", "/app/runLoops/present/rateLimitEnabled"):
    settings.set(key, False)
SPACING = 0.25
SHIFT = 0.06


def path(i: int) -> str:
    return f"/World/Robot/L{i}"


def build(n: int, report: bool) -> Usd.Stage:
    stage = bplib.new_stage(gravity=9.81)
    UsdGeom.Xform.Define(stage, "/World/Robot")
    if report:
        api = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/physicsScene"))
        api.CreateReportKinematicKinematicPairsAttr(True)
        api.CreateReportKinematicStaticPairsAttr(True)
    for i in range(n):
        xf = UsdGeom.Xform.Define(stage, path(i))
        xf.AddTranslateOp().Set(Gf.Vec3d(0, SPACING * i, 0.5))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        xf.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
        cube = UsdGeom.Cube.Define(stage, path(i) + "/collider")
        cube.CreateSizeAttr(0.2)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim()).CreateKinematicEnabledAttr(True)
        if report:
            PhysxSchema.PhysxContactReportAPI.Apply(xf.GetPrim()).CreateThresholdAttr(0.0)
    bplib.update(2)
    return stage


def med_ms(samples: list[float]) -> float:
    return round(statistics.median(samples) * 1e3, 3)


class StepTimer:
    """物理ステップの直前・直後のコールバックで、ステップ 1 回の所要時間を測る。"""

    def __init__(self) -> None:
        from omni.physx import get_physx_interface

        self.durations: list[float] = []
        self._t = 0.0
        px = get_physx_interface()
        self._pre = px.subscribe_physics_on_step_events(self._on_pre, True, 0)
        self._post = px.subscribe_physics_on_step_events(self._on_post, False, 0)

    def _on_pre(self, dt: float) -> None:
        self._t = time.perf_counter()

    def _on_post(self, dt: float) -> None:
        self.durations.append(time.perf_counter() - self._t)

    def take(self) -> list[float]:
        d, self.durations = self.durations, []
        return d


STEP: StepTimer | None = None


def timed_updates(n_updates: int, before: Any = None) -> dict[str, Any]:
    """書き込み・update(壁時計)・update 中のメインスレッド CPU 時間・物理ステップ 1 回、それぞれの中央値 [ms]。

    Play 中の update は 1/60 s に揃えて待つので、壁時計は 16.7 ms になる。実際に使った時間は CPU 時間と物理ステップで見る。
    """
    w, u, c = [], [], []
    if STEP is not None:
        STEP.take()
    for k in range(n_updates):
        t0 = time.perf_counter()
        if before is not None:
            before(k)
        t1 = time.perf_counter()
        c0 = time.thread_time()
        bplib.update(1)
        c1 = time.thread_time()
        t2 = time.perf_counter()
        w.append(t1 - t0)
        u.append(t2 - t1)
        c.append(c1 - c0)
    steps = STEP.take() if STEP is not None else []
    return {
        "write_ms": med_ms(w),
        "update_wall_ms": med_ms(u),
        "update_main_thread_cpu_ms": med_ms(c),
        "physics_step_ms": med_ms(steps) if steps else None,
        "physics_steps_per_update": round(len(steps) / n_updates, 2),
    }


def measure(n: int, report: bool) -> dict[str, Any]:
    from isaacsim.core.experimental.prims import XformPrim
    import isaacsim.core.experimental.utils.backend as backend_utils

    stage = build(n, report)
    movers = list(range(0, n, 2))
    layer = Sdf.Layer.CreateAnonymous("geometric")
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    with Usd.EditContext(stage, layer):
        for i in movers:
            stage.GetPrimAtPath(path(i)).GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, SPACING * i, 0.5))
    specs = [layer.GetAttributeAtPath(Sdf.Path(path(i)).AppendProperty("xformOp:translate")) for i in movers]
    xp = XformPrim([path(i) for i in movers])
    bplib.update(3)

    counts = {"headers": 0}
    sub = None
    if report:
        def on_contact(headers: Any, data: Any) -> None:
            counts["headers"] += len(headers)

        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)

    def write_usd(k: int) -> None:
        dy = SHIFT if k % 2 == 0 else 0.0
        with Sdf.ChangeBlock():
            for s, i in zip(specs, movers):
                s.default = Gf.Vec3d(0, SPACING * i + dy, 0.5)

    pos = []
    for dy in (SHIFT, 0.0):
        a = np.zeros((len(movers), 3), dtype=np.float32)
        a[:, 1] = SPACING * np.array(movers) + dy
        a[:, 2] = 0.5
        pos.append(wp.array(a, dtype=wp.float32, device="cpu"))

    def write_fabric(k: int) -> None:
        with backend_utils.use_backend("fabric"):
            xp.set_local_poses(translations=pos[k % 2])

    cols = [path(i) + "/collider" for i in range(n)]
    ids = [PhysicsSchemaTools.encodeSdfPath(Sdf.Path(c)) for c in cols]
    sq = get_physx_scene_query_interface()

    def overlap_all() -> int:
        total = 0
        for id0, id1 in ids:
            hits: list[str] = []
            sq.overlap_shape(id0, id1, lambda h: hits.append(h.collision) or True, False)
            total += len(hits) - 1
        return total

    global STEP
    out: dict[str, Any] = {"movers": len(movers)}
    STEP = None
    out["stop_idle"] = timed_updates(40)
    STEP = StepTimer()
    bplib.play(5)
    out["play_idle"] = timed_updates(60)

    counts["headers"] = 0
    out["play_usd"] = timed_updates(60, write_usd)
    out["play_usd_contact_headers_per_update"] = round(counts["headers"] / 60, 1)
    write_usd(0)  # めり込んだ状態にして 2 update 待ち、overlap が見えるか確かめる
    bplib.update(2)
    out["play_usd_overlapping_pairs_x2"] = overlap_all()
    write_usd(1)
    bplib.update(2)

    counts["headers"] = 0
    out["play_fabric"] = timed_updates(60, write_fabric)
    out["play_fabric_contact_headers_per_update"] = round(counts["headers"] / 60, 1)
    write_fabric(0)
    bplib.update(2)
    out["play_fabric_overlapping_pairs_x2"] = overlap_all()

    samples = []
    for _ in range(15):
        t0 = time.perf_counter()
        overlap_all()
        samples.append(time.perf_counter() - t0)
    out["overlap_all_ms"] = med_ms(samples)
    sub = None
    STEP = None
    bplib.stop(2)
    return out


@bplib.guarded(rec, "P-00")
def p00() -> None:
    rec.add("P-00", "環境", **bplib.environment())


@bplib.guarded(rec, "P-01")
def p01() -> None:
    for n in (10, 30, 100, 300, 1000):
        for report in (True, False):
            rec.add(f"P-01 N={n} report={report}", "Play 中の update 時間 [ms]", **measure(n, report))


p00()
p01()
bplib.finish(rec)
