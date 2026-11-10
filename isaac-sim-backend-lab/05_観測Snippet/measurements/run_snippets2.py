# run_snippets2.py - backend_spy・strict_backend と、Stop をまたいだコールバックの生存を記録する（run_snippets.py の続き）
import builtins
import os
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

from isaacsim.core.experimental.prims import Articulation, RigidPrim


def try_print(label, action):
    """action を呼び、結果か例外を1行で表示する。"""
    try:
        print(f"{label}: {action()}")
    except Exception as error:
        print(f"{label}: ERR {type(error).__name__}: {str(error)[:120]}")


# ---------------------------------------------------------------- A3b Stop をまたいでもコールバックが生きているか
phase("A3b_step_probe_across_stop")
stage = new_stage()
add_cube(stage)
with captured_stdout("step_probe 登録→Play→Stop"):
    run_snippet("step_probe")
    set_state("Play", settle_updates=10)
    set_state("Stop")
with captured_stdout("step_probe Stop中に再実行せず予約→Play"):
    builtins.schedule_after(5, lambda: print("fired after stop"), "across-stop")
    set_state("Play", settle_updates=10)
set_state("Stop")
for callback_id in builtins._step_probe_ids:
    SimulationManager.deregister_callback(callback_id)
builtins._step_probe_ids = []

# ---------------------------------------------------------------- A6 backend_spy
phase("A6_backend_spy")
stage = new_stage()
add_cube(stage)
add_articulation(stage)
run_updates(3)
with captured_stdout("backend_spy ON"):
    run_snippet("backend_spy")
made_in_stop = {}
for target in ("Stop", "Play", "Pause", "Stop"):
    set_state(target)
    for origin in ("新しく作ったラッパー", "Stop中に作ったラッパー"):
        if origin == "新しく作ったラッパー" or not made_in_stop:
            robot = Articulation("/World/robot")
            cube = RigidPrim("/World/Cube")
            if not made_in_stop:
                made_in_stop = {"robot": robot, "cube": cube}
        else:
            robot = made_in_stop["robot"]
            cube = made_in_stop["cube"]
        with captured_stdout(f"backend_spy {target} {origin}"):
            try_print("get_dof_positions", lambda: robot.get_dof_positions().numpy().tolist())
            try_print("get_dof_position_targets", lambda: robot.get_dof_position_targets().numpy().tolist())
            try_print("get_dof_limits", lambda: robot.get_dof_limits().numpy().tolist())
            try_print("RigidPrim.get_world_poses z", lambda: cube.get_world_poses()[0].numpy()[0][2].item())
with captured_stdout("backend_spy OFF"):
    run_snippet("backend_spy", {"SPY_ENABLED: typing.Final[bool] = True": "SPY_ENABLED: typing.Final[bool] = False"})

# ---------------------------------------------------------------- A7 strict_backend
phase("A7_strict_backend")
stage = new_stage()
add_cube(stage)
for target in ("Stop", "Play", "Pause", "Stop"):
    set_state(target)
    with captured_stdout(f"strict_backend {target}"):
        run_snippet("strict_backend")

close()
