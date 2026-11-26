"""Red/Green 実測検証 01 (RED): _repush_frames ガード無効時に 1 フレーム戻りのちらつきが実測で起きるか確認。

WinMerge で `01_fabric_repush_flicker_Green.py` と比較すると、差分は `ENABLE_REPUSH_GUARD` のみです。

実行方法:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\tests_red_green\\01_fabric_repush_flicker_Red.py --headless
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

# === WinMerge 差分ポイント (Red=False / Green=True) ===
ENABLE_REPUSH_GUARD = False
# ======================================================


def _setup_stage():
    import omni.usd
    from pxr import UsdGeom

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    prim = stage.DefinePrim("/World/Robot", "Xform")
    prim.GetReferences().AddReference(UR10E_USD.as_posix())
    return stage


def _read_fabric_tip_pos(stage, link_path: str):
    import numpy as np
    from myext.robot.geometric_newton.usd.writer import GeometricFabricWriter

    reader = GeometricFabricWriter(stage, [link_path])
    m = reader.read_world()[0]
    reader.close()
    return np.array(m[:3, 3], dtype=float)


def _measure_frames(app, stage, robot, tip_link: str, expected_pos, label: str, frames: int = 4) -> float:
    import numpy as np

    tip_path = robot.link_path_map[tip_link]
    max_err = 0.0
    print(f"\n--- [{label}] ENABLE_REPUSH_GUARD={ENABLE_REPUSH_GUARD} ---")
    for f in range(frames + 1):
        if f > 0:
            app.update()
        fab_pos = _read_fabric_tip_pos(stage, tip_path)
        err = float(np.linalg.norm(fab_pos - expected_pos))
        max_err = max(max_err, err)
        tag = "FLICKER!" if err > 1e-3 else "OK"
        print(
            f"  frame +{f}: fabric_tip={np.round(fab_pos, 4).tolist()} "
            f"expected={np.round(expected_pos, 4).tolist()} err={err:.6f} m [{tag}] "
            f"(pending_repush={robot.pending_render_sync_frames})"
        )
    return max_err


def main() -> None:
    sys.path.insert(0, (ISAAC_ROOT / "tools/myext").as_posix())
    from sim_app_launcher import create_simulation_app

    app = create_simulation_app(
        {"headless": True},
        extra_extensions=("myext.robot.geometric_newton",),
    )
    try:
        import numpy as np
        from myext.robot.geometric_newton import GeometricRobot

        stage = _setup_stage()
        for _ in range(4):
            app.update()

        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="fabric", angle_unit="deg")
        for _ in range(4):
            app.update()

        tip_link = "wrist_3_link"
        q_old = np.array([0.0, -60.0, 60.0, 0.0, 0.0, 0.0])
        q_new = np.array([90.0, -30.0, 30.0, 45.0, -45.0, 0.0])

        # ------------------------------------------------------------------
        # Case A: flush_to_usd() で USD へ q_old を書いた直後に Fabric だけで q_new へ移動
        # ------------------------------------------------------------------
        robot.set_dof_positions(q_old, unit="deg")
        robot.flush_to_usd()
        if not ENABLE_REPUSH_GUARD:
            robot._repush_frames = 0
            robot._usd_sync_pending = False
        robot.set_dof_positions(q_new, unit="deg")  # Fabric のみへ書き込み
        expected_a = robot.links[tip_link].get_world_pose()[0].copy()
        err_a = _measure_frames(app, stage, robot, tip_link, expected_a, "Case A: flush_to_usd -> immediate Fabric move")

        # ------------------------------------------------------------------
        # Case B: set_variant() で構造再構築した直後に Fabric だけで別姿勢へ移動
        # ------------------------------------------------------------------
        robot.set_dof_positions(q_old, unit="deg")
        app.update()
        robot.set_variant("Gripper", "Robotiq_2F_85")
        if not ENABLE_REPUSH_GUARD:
            robot._repush_frames = 0
            robot._usd_sync_pending = False
        q_new_v = np.zeros(robot.num_dofs)
        q_new_v[:6] = q_new
        robot.set_dof_positions(q_new_v, unit="deg")  # Fabric のみへ書き込み
        expected_b = robot.links[tip_link].get_world_pose()[0].copy()
        err_b = _measure_frames(app, stage, robot, tip_link, expected_b, "Case B: set_variant -> immediate Fabric move")

        # ------------------------------------------------------------------
        # Case C: layer.apply() 直後に Fabric だけで別姿勢へ移動
        # ------------------------------------------------------------------
        robot.close()
        app.update()
        robot2 = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="fabric", angle_unit="deg")
        if not ENABLE_REPUSH_GUARD:
            robot2._repush_frames = 0
            robot2._usd_sync_pending = False
        q_new_c = np.zeros(robot2.num_dofs)
        q_new_c[:6] = np.array([-80.0, -40.0, 50.0, 20.0, 30.0, 0.0])
        robot2.set_dof_positions(q_new_c, unit="deg")
        expected_c = robot2.links[tip_link].get_world_pose()[0].copy()
        err_c = _measure_frames(app, stage, robot2, tip_link, expected_c, "Case C: layer.apply -> immediate Fabric move")
        robot2.close()

        print("\n=== SUMMARY ===")
        print(f"ENABLE_REPUSH_GUARD = {ENABLE_REPUSH_GUARD}")
        print(f"Case A max_err = {err_a:.6f} m  ({'RED: FLICKER REPRODUCED' if err_a > 1e-3 else 'GREEN: NO FLICKER'})")
        print(f"Case B max_err = {err_b:.6f} m  ({'RED: FLICKER REPRODUCED' if err_b > 1e-3 else 'GREEN: NO FLICKER'})")
        print(f"Case C max_err = {err_c:.6f} m  ({'RED: FLICKER REPRODUCED' if err_c > 1e-3 else 'GREEN: NO FLICKER'})")
    finally:
        app.close()


if __name__ == "__main__":
    main()
