"""Red/Green 実測検証 05 (RED): 54 DOF 4 アームロボットで batch_update() なしに Component 毎セットした場合の速度差を実測。

WinMerge で `05_batch_update_components_Green.py` と比較すると、差分は `USE_BATCH_UPDATE` のみです。

実行方法:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\tests_red_green\\05_batch_update_components_Red.py --headless
"""

from __future__ import annotations

import os
import sys
import tempfile
import time
from contextlib import nullcontext
from pathlib import Path

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
EXT_ROOT = ISAAC_ROOT / "extsUser/Extentions/myext.robot.geometric_newton"

# === WinMerge 差分ポイント (Red=False / Green=True) ===
USE_BATCH_UPDATE = False
# ======================================================


def main() -> None:
    sys.path.insert(0, (ISAAC_ROOT / "tools/myext").as_posix())
    from sim_app_launcher import create_simulation_app

    app = create_simulation_app(
        {"headless": True},
        extra_extensions=("myext.robot.geometric_newton",),
    )
    try:
        sys.path.insert(0, (EXT_ROOT / "tests_kit").as_posix())
        import numpy as np
        import omni.usd
        from pxr import UsdGeom
        from multi_arm_robot import ARMS, add_multi_arm_robot, write_multi_arm_asset
        from myext.robot.geometric_newton import GeometricRobot, UsdArticulationSource

        tmp_dir = tempfile.mkdtemp()
        usd_path = write_multi_arm_asset(os.path.join(tmp_dir, "multi_arm.usda"))

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        add_multi_arm_robot(stage, "/World/Robot", usd_path, (0.0, 0.0, 0.0))
        for _ in range(4):
            app.update()

        comp_spec = {
            "cart": ["/World/Robot/base", "/World/Robot/column", "/World/Robot/boom"],
            **{f"arm{i}": f"/World/Robot/arm{i}_*" for i in range(ARMS)},
        }
        robot = GeometricRobot.from_articulation(
            stage,
            UsdArticulationSource("/World/Robot", components=comp_spec),
            write_target="fabric",
            fk_backend="numpy",
        )
        for _ in range(4):
            app.update()

        comp_names = ["cart", "arm0", "arm1", "arm2", "arm3"]
        comps = [robot.get_component(c) for c in comp_names]
        # 各部位用の固定長配列と、Variant 異軸数対応 Binder (各アーム用) を事前生成
        rng = np.random.default_rng(42)
        q_all = rng.uniform(-0.1, 0.1, size=robot.num_dofs)
        q_comps = [rng.uniform(-0.1, 0.1, size=c.num_dofs) for c in comps]
        q_dict = {n: float(v) for n, v in zip(robot.dof_names, q_all)}
        binders = [robot.bind_dofs(c.dof_names, ignore_unknown=True) for c in comps]

        print(f"num_dofs = {robot.num_dofs}, num_links = {robot.num_links}")
        print(f"robot.dof_names[:12] (BFS 順で並列アームが交互に並ぶ) = {robot.dof_names[:12]}")
        print(f"arm0.dof_names (Component ならアーム単位で整列)        = {comps[1].dof_names}")

        # _push 呼び出し回数カウンタ
        push_calls = 0
        orig_push = robot._push

        def _counted_push():
            nonlocal push_calls
            push_calls += 1
            return orig_push()

        robot._push = _counted_push  # type: ignore[method-assign]

        # ウォームアップ
        for _ in range(20):
            robot.set_dof_positions(q_all)

        iters = 300

        # 1) 全体 1 本のフラット配列を一括渡し (最速ベースライン)
        push_calls = 0
        t0 = time.perf_counter()
        for k in range(iters):
            q_all[0] = 0.0001 * k
            robot.set_dof_positions(q_all)
        ms_flat = (time.perf_counter() - t0) * 1000.0 / iters
        pushes_flat = push_calls // iters

        # 2) Component ごとにバラバラにセット (Red: batch_update なし / Green: with robot.batch_update():)
        push_calls = 0
        t0 = time.perf_counter()
        for k in range(iters):
            q_comps[0][0] = 0.0001 * k
            ctx_mgr = robot.batch_update() if USE_BATCH_UPDATE else nullcontext()
            with ctx_mgr:
                for c, qc in zip(comps, q_comps):
                    c.set_dof_positions(qc)
        ms_comp = (time.perf_counter() - t0) * 1000.0 / iters
        pushes_comp = push_calls // iters

        # 3) Variant 異軸数対応 GeometricDofBinder x 5 部位 (batch_update 有無は同じフラグ)
        push_calls = 0
        t0 = time.perf_counter()
        for k in range(iters):
            q_comps[0][0] = 0.0001 * k
            ctx_mgr = robot.batch_update() if USE_BATCH_UPDATE else nullcontext()
            with ctx_mgr:
                for b, qc in zip(binders, q_comps):
                    b.set_positions(qc)
        ms_binder = (time.perf_counter() - t0) * 1000.0 / iters
        pushes_binder = push_calls // iters

        # 4) 参考: 全 54 軸を毎回辞書で渡す
        push_calls = 0
        t0 = time.perf_counter()
        for k in range(iters):
            q_dict[robot.dof_names[0]] = 0.0001 * k
            robot.set_dof_positions(q_dict)
        ms_dict = (time.perf_counter() - t0) * 1000.0 / iters

        print("\n=== SUMMARY ===")
        print(f"USE_BATCH_UPDATE = {USE_BATCH_UPDATE}")
        print(f"1) Flat array (54 DOF 1回)       : {ms_flat:.4f} ms/frame (pushes/frame={pushes_flat})")
        print(f"2) 5 Components set_dof_positions: {ms_comp:.4f} ms/frame (pushes/frame={pushes_comp}, diff={ms_comp - ms_flat:+.4f} ms, {ms_comp / ms_flat:.2f}x)")
        print(f"3) 5 GeometricDofBinders         : {ms_binder:.4f} ms/frame (pushes/frame={pushes_binder}, diff={ms_binder - ms_flat:+.4f} ms, {ms_binder / ms_flat:.2f}x)")
        print(f"4) Reference: 54-key dict        : {ms_dict:.4f} ms/frame (diff={ms_dict - ms_flat:+.4f} ms, {ms_dict / ms_flat:.2f}x)")

        robot.close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
