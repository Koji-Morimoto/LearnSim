"""4 本アームのロボット(最大 54 軸)で、作る・作り直す・毎フレームの処理の時間を測る(headless、描画なし、CPU 物理)。

実行:  C:\\isaacsim-6.2.0\\python.bat tests_kit\\bench_multi_arm.py --no-ros-env [--label 名前]
結果:  tests_kit/bench_<label>.json(中央値、ms)
"""

from __future__ import annotations

import json
import os
import statistics
import sys
import tempfile
import time

import harness

app = harness.start()

import numpy as np
from pxr import Usd

import myext.robot.geometric_newton as geo
from multi_arm_robot import add_multi_arm_robot, write_multi_arm_asset
from myext.robot.geometric_newton import GeometricRobot, read_articulation

LABEL = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--label=")), "current")
ROBOT = "/World/Robot"
results: dict = {"label": LABEL}


def med_ms(fn, repeat=30, warmup=3) -> float:
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn()
        samples.append(time.perf_counter() - t0)
    return round(statistics.median(samples) * 1e3, 3)


def once_ms(fn) -> tuple[float, object]:
    t0 = time.perf_counter()
    value = fn()
    return round((time.perf_counter() - t0) * 1e3, 2), value


tmp = tempfile.mkdtemp()
asset = write_multi_arm_asset(os.path.join(tmp, "multi_arm.usda"))
stage = harness.new_stage()
prim = add_multi_arm_robot(stage, ROBOT, asset)
harness.update(3)

# ---- 作る
t, (model, info) = once_ms(lambda: read_articulation(stage, ROBOT))
results["read_articulation_ms"] = t
results["dofs"] = model.num_dofs
results["links"] = model.num_links
results["read_articulation_ms_median"] = med_ms(lambda: read_articulation(stage, ROBOT), repeat=10, warmup=1)
from myext.robot.geometric_newton.model import GeometricRobotModel

results["model_build_ms"] = med_ms(lambda: GeometricRobotModel(model.name, model.links, model.joints, model.root_link, model.components), repeat=20)
t, robot = once_ms(lambda: GeometricRobot.from_articulation(stage, ROBOT, write_target="fabric"))
results["from_articulation_total_ms(初回、Sublayer を付けるまで)"] = t
harness.update(3)

# ---- 毎フレーム
q = np.zeros(robot.num_dofs)
rng = np.random.default_rng(0)
qs = [robot.model.resolve_mimic(rng.uniform(-0.3, 0.3, size=robot.num_dofs)) for _ in range(64)]
k = iter(range(10**9))
results["forward_kinematics_ms"] = med_ms(lambda: robot.model.forward_kinematics(qs[next(k) % 64]), repeat=200)
batch = np.array(qs)
results["forward_kinematics_batch64_ms"] = med_ms(lambda: robot.model.forward_kinematics(batch), repeat=30)
results["jacobian_ms"] = med_ms(lambda: robot.model.jacobian(qs[next(k) % 64]), repeat=100)
results["set_dof_positions_fabric_ms"] = med_ms(lambda: robot.set_dof_positions(qs[next(k) % 64]), repeat=200)
results["flush_to_usd_ms"] = med_ms(lambda: (robot.set_dof_positions(qs[next(k) % 64]), robot.flush_to_usd()), repeat=50)
robot.write_target = "usd"
results["set_dof_positions_usd_ms"] = med_ms(lambda: robot.set_dof_positions(qs[next(k) % 64]), repeat=100)
robot.write_target = "fabric"
robot.set_dof_positions(np.zeros(robot.num_dofs))
t, _ = once_ms(lambda: robot.check_collisions("within"))
results["check_collisions_within_first_ms(PhysX 読み込み込み)"] = t
results["check_collisions_within_ms"] = med_ms(lambda: robot.check_collisions("within"), repeat=10)
results["collider_count"] = sum(len(v) for v in robot.collider_paths.values())

# ---- 器具を切り替えて作り直す
switches = [("arm0_tool", "camera"), ("arm1_tool", "stapler"), ("arm0_tool", "wrist"), ("arm1_tool", "wrist")]
rebuild = []
for tool, variant in switches:
    before = robot.num_dofs if not getattr(robot, "_closed", False) else None
    if hasattr(robot, "set_variant"):  # 変更後の実装: 切り替えと作り直しを 1 回で
        t, _ = once_ms(lambda: robot.set_variant("Instrument", variant, prim=f"{ROBOT}/{tool}"))
        t_check, _ = once_ms(lambda: robot.check_collisions("within"))
        rebuild.append({"switch": f"{tool}={variant}", "dofs": [before, robot.num_dofs], "rebuild_ms": t, "next_check_collisions_ms": t_check, "breakdown_ms": getattr(robot, "last_rebuild_timings", None)})
    else:  # 変更前: 自動では作り直さないので、閉じて作り直すしかない
        def recreate():
            global robot
            robot.close()
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                stage.GetPrimAtPath(f"{ROBOT}/{tool}").GetVariantSet("Instrument").SetVariantSelection(variant)
            robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="fabric")

        t, _ = once_ms(recreate)
        t_check, _ = once_ms(lambda: robot.check_collisions("within"))
        rebuild.append({"switch": f"{tool}={variant}", "dofs": [before, robot.num_dofs], "recreate_ms": t, "next_check_collisions_ms": t_check})
    harness.update(2)
results["variant_switch"] = rebuild

path = os.path.join(harness.HERE, f"bench_{LABEL}.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(results, f, ensure_ascii=False, indent=1)
print("[BENCH] " + json.dumps(results, ensure_ascii=False), flush=True)
os._exit(0)
