"""Red/Green 実測検証 04 (GREEN): 毎フレーム write_target='fabric' で更新した場合の負荷と USD 書込回数を実測。

WinMerge で `04_write_target_usd_vs_fabric_Red.py` と比較すると、差分は `WRITE_TARGET` のみです。

実行方法:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\tests_red_green\\04_write_target_usd_vs_fabric_Green.py --headless
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

# === WinMerge 差分ポイント (Red="usd" / Green="fabric") ===
WRITE_TARGET = "fabric"
# ==========================================================


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
        from myext.robot.geometric_newton import GeometricRobot

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

        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target=WRITE_TARGET, angle_unit="deg")
        for _ in range(4):
            app.update()

        usd_writes = 0
        orig_usd_write = robot._usd_writer.write

        def _counted_usd_write(*a, **kw):
            nonlocal usd_writes
            usd_writes += 1
            return orig_usd_write(*a, **kw)

        robot._usd_writer.write = _counted_usd_write  # type: ignore[method-assign]

        # ウォームアップ
        q = np.zeros(robot.num_dofs)
        for _ in range(10):
            robot.set_dof_positions(q, unit="deg")
            app.update()
        usd_writes = 0

        frames = 120
        t_set = 0.0
        t_upd = 0.0
        for f in range(frames):
            q[0] = float(f % 90)
            q[1] = -30.0 + 0.2 * f
            t0 = time.perf_counter()
            robot.set_dof_positions(q, unit="deg")
            t1 = time.perf_counter()
            app.update()
            t2 = time.perf_counter()
            t_set += (t1 - t0) * 1000.0
            t_upd += (t2 - t1) * 1000.0

        print("\n=== SUMMARY ===")
        print(f"WRITE_TARGET             = {WRITE_TARGET!r}")
        print(f"USD write calls / {frames}f   = {usd_writes}")
        print(f"Avg set_dof_positions    = {t_set / frames:.3f} ms/frame")
        print(f"Avg app.update()         = {t_upd / frames:.3f} ms/frame")
        print(f"Total (set + update)     = {(t_set + t_upd) / frames:.3f} ms/frame")

        robot.close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
