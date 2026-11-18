"""Kit の更新を進める部品と、使い捨てのステージ。パイプラインは headless の Isaac Sim(SimulationApp)から呼ぶ前提。"""

from __future__ import annotations

import asyncio
import contextlib

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


def update(n: int = 1) -> None:
    app = omni.kit.app.get_app()
    for _ in range(n):
        app.update()


def run_async(coro) -> object:
    task = asyncio.ensure_future(coro)
    app = omni.kit.app.get_app()
    while not task.done():
        app.update()
    return task.result()


def timeline():
    return omni.timeline.get_timeline_interface()


def stop() -> None:
    if not timeline().is_stopped():
        timeline().stop()
        update(2)


def new_probe_stage(gravity: float = 9.81, up_axis: str = "Z", meters_per_unit: float = 1.0) -> Usd.Stage:
    """USD コンテキストに空のステージを作る(前のステージは閉じる)。物理は CPU。"""
    stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    update(2)
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z if up_axis == "Z" else UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1) if up_axis == "Z" else Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr("MBP")
    return stage


@contextlib.contextmanager
def capture_warnings():
    """carb の警告とエラーを集める。"""
    logs: list[str] = []

    def on_log(source, level, filename, lineno, msg):  # noqa: ANN001
        if level >= carb.logging.LEVEL_WARN:
            logs.append(f"{source}: {msg}".strip())

    sub = carb.logging.acquire_logging().add_logger(on_log)
    try:
        yield logs
    finally:
        carb.logging.acquire_logging().remove_logger(sub)
