# 2026-11-18  やり方 A: SimulationApp(headless=True) のスクリプトをデバッガで起動して止める
# 使い方: このファイルを開いて、実行とデバッグから「A: SimulationApp (headless)」を選んで F5
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

# SimulationApp を作った後でないと import できないモジュール
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()
UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

# 床は置いていないので、箱は z=2 から落ち続ける（z の値が毎フレーム変わるのを見るため）
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 2.0))
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

import omni.timeline

omni.timeline.get_timeline_interface().play()

for frame in range(60):
    simulation_app.update()
    z = cube.GetPrim().GetAttribute("xformOp:translate").Get()[2]
    print(f"frame={frame:02d} z={z:.3f}")  # ← ここにブレークポイントを置く

simulation_app.close()
