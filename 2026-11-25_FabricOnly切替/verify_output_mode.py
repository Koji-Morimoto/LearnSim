"""physics_output_mode.set_output_mode() の効果を測る（ヘッドレスの別プロセスで実行）。

落下する立方体を 30 ステップ再生し、USD の位置と Fabric の位置（omni:fabric:worldMatrix）の
z 座標が再生前から動いたか、USD の速度 physics:velocity が書かれたかを、出力先ごとに表で出す。

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_output_mode.py
"""

import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

import physics_output_mode as pom  # noqa: E402

CUBE = "/World/Cube"
START_Z = 2.0
STEPS = 30


def build_stage(gpu: bool) -> None:
    omni.usd.get_context().new_stage()
    app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(gpu)
    physx_scene.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    cube = UsdGeom.Cube.Define(stage, CUBE)
    cube.CreateSizeAttr(0.2)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, START_Z))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    for _ in range(5):
        app.update()


def usd_z() -> float:
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.Xformable(stage.GetPrimAtPath(CUBE)).ComputeLocalToWorldTransform(0)
    return m.ExtractTranslation()[2]


def fabric_z():
    import usdrt

    stage_id = omni.usd.get_context().get_stage_id()
    rt_stage = usdrt.Usd.Stage.Attach(stage_id)
    attr = rt_stage.GetPrimAtPath(CUBE).GetAttribute("omni:fabric:worldMatrix")
    if not attr or not attr.HasValue():
        return None
    m = attr.Get()
    return float(m[3][2])


def run(mode: pom.OutputMode, gpu: bool) -> dict:
    build_stage(gpu)
    pom.set_output_mode(mode)
    app.update()
    timeline = omni.timeline.get_timeline_interface()
    usd_before = usd_z()
    timeline.play()
    for _ in range(STEPS):
        app.update()
    usd_after = usd_z()
    fab_after = fabric_z()
    vel_attr = omni.usd.get_context().get_stage().GetPrimAtPath(CUBE).GetAttribute("physics:velocity")
    usd_vz = vel_attr.Get()[2] if vel_attr and vel_attr.Get() is not None else None
    timeline.pause()
    app.update()
    timeline.stop()
    for _ in range(3):
        app.update()
    usd_after_stop = usd_z()
    return {
        "mode": mode,
        "gpu": gpu,
        "usd_before": usd_before,
        "usd_after": usd_after,
        "fabric_after": fab_after,
        "usd_vz": usd_vz,
        "usd_after_stop": usd_after_stop,
    }


results = []
for mode, gpu in [
    ("usd", False),
    ("fabric_cpu", False),
    ("usd", False),  # Fabric 拡張を有効にしたまま USD に戻した場合
    ("fabric_gpu", True),
    ("usd", True),
]:
    results.append(run(mode, gpu))

print("\n===== RESULT =====")
print(f"{'mode':<11} {'GPU':<5} {'USD z 再生前':>12} {'USD z 再生後':>12} {'Fabric z 再生後':>15} {'USD z Stop後':>12} {'USD 速度z 再生後':>15}")
for r in results:
    fab = "None" if r["fabric_after"] is None else f"{r['fabric_after']:.4f}"
    vz = "None" if r["usd_vz"] is None else f"{r['usd_vz']:.4f}"
    print(
        f"{r['mode']:<11} {str(r['gpu']):<5} {r['usd_before']:>12.4f} {r['usd_after']:>12.4f} {fab:>15} {r['usd_after_stop']:>12.4f} {vz:>15}"
    )
print("===== END =====")
app.close()
