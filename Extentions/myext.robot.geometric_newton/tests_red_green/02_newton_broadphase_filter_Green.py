"""Red/Green 実測検証 02 (GREEN): Newton Broad Phase へ collision_filter を登録した場合の実測。

WinMerge で `02_newton_broadphase_filter_Red.py` と比較すると、差分は `BAKE_FILTER_IN_BROADPHASE` のみです。

実行方法:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\tests_red_green\\02_newton_broadphase_filter_Green.py --headless
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

# === WinMerge 差分ポイント (Red=False / Green=True) ===
BAKE_FILTER_IN_BROADPHASE = True
# ======================================================


def main() -> None:
    sys.path.insert(0, (ISAAC_ROOT / "tools/myext").as_posix())
    from sim_app_launcher import create_simulation_app

    app = create_simulation_app(
        {"headless": True},
        extra_extensions=("myext.robot.geometric_newton",),
    )
    try:
        import numpy as np
        import omni.usd
        from pxr import UsdGeom
        from myext.robot.geometric_newton import GeometricRobot, GeometricScene

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        prim = stage.DefinePrim("/World/Robot", "Xform")
        prim.GetReferences().AddReference(UR10E_USD.as_posix())
        prim.GetVariantSets().GetVariantSet("Gripper").SetVariantSelection("Robotiq_2F_85")
        for _ in range(4):
            app.update()

        scene = GeometricScene(
            stage,
            bake_adjacent_filter=BAKE_FILTER_IN_BROADPHASE,
            bake_collision_filter=BAKE_FILTER_IN_BROADPHASE,
        )
        robot = GeometricRobot.from_articulation(stage, "/World/Robot", scene=scene, angle_unit="deg")
        robot.ignore_touching_pairs()

        # ウォームアップ
        for _ in range(10):
            robot.check_collisions("within")

        baked_pairs = int(scene.newton_model.shape_collision_filter_pairs_array().shape[0])
        broad_phase_candidates = int(scene._pipeline.shape_pairs_filtered.shape[0])
        raw_narrow_contacts = int(scene._contacts.rigid_contact_count.numpy()[0])

        iters = 200
        q = np.zeros(robot.num_dofs)
        t0 = time.perf_counter()
        for k in range(iters):
            q[0] = float(k % 30)
            robot.set_dof_positions(q, unit="deg")
            hits = robot.check_collisions("within")
        ms_per_iter = (time.perf_counter() - t0) * 1000.0 / iters

        print("\n=== SUMMARY ===")
        print(f"BAKE_FILTER_IN_BROADPHASE = {BAKE_FILTER_IN_BROADPHASE}")
        print(f"robot.collision_filter.pairs   = {len(robot.collision_filter.pairs)}")
        print(f"Newton baked filter pairs      = {baked_pairs}")
        print(f"Broad Phase candidate pairs    = {broad_phase_candidates}")
        print(f"Narrow Phase raw contact count = {raw_narrow_contacts}  ({'RED: WASTED NARROW PHASE' if raw_narrow_contacts > 0 else 'GREEN: ZERO WASTED CONTACTS'})")
        print(f"Reported user hits             = {len(hits)}")
        print(f"Avg set_dof + check_collisions = {ms_per_iter:.3f} ms/iter ({iters} iters)")

        robot.close()
        scene.close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
