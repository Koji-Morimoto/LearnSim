"""Red/Green 実測検証 03 (RED): collision_mode='static' では PhysX 空間にロボットコライダーが残ることを実測。

WinMerge で `03_collision_mode_none_vs_static_Green.py` と比較すると、差分は `COLLISION_MODE` のみです。

実行方法:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\tests_red_green\\03_collision_mode_none_vs_static_Red.py --headless
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

# === WinMerge 差分ポイント (Red="static" / Green="none") ===
COLLISION_MODE = "static"
# ===========================================================


def main() -> None:
    sys.path.insert(0, (ISAAC_ROOT / "tools/myext").as_posix())
    from sim_app_launcher import create_simulation_app

    app = create_simulation_app(
        {"headless": True},
        extra_extensions=("myext.robot.geometric_newton",),
    )
    try:
        import omni.usd
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from pxr import Gf, UsdGeom, UsdPhysics
        from myext.robot.geometric_newton import GeometricLayerOptions, GeometricRobot

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        prim = stage.DefinePrim("/World/Robot", "Xform")
        prim.GetReferences().AddReference(UR10E_USD.as_posix())
        for _ in range(4):
            app.update()

        robot = GeometricRobot.from_articulation(
            stage,
            "/World/Robot",
            layer_options=GeometricLayerOptions(collision_mode=COLLISION_MODE),
        )
        robot.flush_to_usd()
        for _ in range(3):
            app.update()

        # 1) PhysX へ USD をロードしてから大球 (半径 5m) を照会し、PhysX が保持しているロボットコライダー数を実測
        get_physx_interface().force_load_physics_from_usd()
        physx_robot_hits: list[str] = []

        def _on_hit(hit) -> bool:
            p = str(hit.collision)
            if p.startswith("/World/Robot"):
                physx_robot_hits.append(p)
            return True

        get_physx_scene_query_interface().overlap_sphere(5.0, (0.0, 0.0, 0.0), _on_hit, False)

        # 2) Newton 側からは同じロボットのコライダーが見え、障害物との干渉を 0 遅延検知できることを確認
        tip_pos = robot.links["wrist_3_link"].get_world_pose()[0]
        obs = UsdGeom.Cube.Define(stage, "/World/Obs")
        obs.CreateSizeAttr(0.15)
        obs.AddTranslateOp().Set(Gf.Vec3d(*tip_pos.tolist()))
        UsdPhysics.CollisionAPI.Apply(obs.GetPrim())
        newton_collider_count = sum(len(v) for v in robot.collider_paths.values())
        newton_env_hits = robot.check_collisions("environment")

        print("\n=== SUMMARY ===")
        print(f"COLLISION_MODE                  = {COLLISION_MODE!r}")
        print(f"PhysX active robot colliders    = {len(physx_robot_hits)}  ({'RED: PHYSX STILL HOLDS COLLIDERS' if physx_robot_hits else 'GREEN: ZERO PHYSX COLLIDERS'})")
        print(f"Newton visible robot colliders  = {newton_collider_count}")
        print(f"Newton environment hits         = {len(newton_env_hits)} ({[p.names for p in newton_env_hits[:3]]})")

        robot.close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
