# step_probe.py — 物理ステップの前後にログを出し、「指定したステップ数の後に実行」を予約・検証する
# 解説: 05_観測Snippet/02_step_probe.md
"""物理ステップの前後にログを出し、「指定したステップ数の後に実行」を予約・検証する。

Isaac Sim の Script Editor で実行する。再実行すると、前回登録したコールバックを解除してから登録し直す。

Note:
    呼べる状態: Stop／Play／Pause のどれでも登録できる。コールバックは物理ステップが実行されたとき
        （Play 中、または SimulationManager.step を呼んだとき）にだけ呼ばれる。
        Play 開始時のウォームアップで実行される2回の物理ステップでは、コールバックは呼ばれない
        （累計の物理ステップ数には数えられる）。
    backend: Core Experimental API の prim ラッパーを使わないので、backend の選択は関係しない。
"""

from __future__ import annotations

import builtins  # Script Editorの再実行をまたいで値を保持するために組み込み名前空間を使う
import typing

from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # ステップへの割り込みと計数のAPIを読み込む

if typing.TYPE_CHECKING:
    import omni.physics.core  # 型ヒントにだけ使う。実行時には import しない

# 予約1件の内容。(実行予定の累計ステップ数, 予約した時点の累計ステップ数, 実行する処理, 表示名)
PendingItem: typing.TypeAlias = tuple[int, int, typing.Callable[[], None], str]


class ProbeState(typing.TypedDict):
    """観測の途中経過。辞書のキーと値の型をエディタに伝えるための定義。"""

    logged_steps: int  # ログを出したステップ数
    pending: list[PendingItem]  # 予約中の処理


for callback_id in getattr(builtins, "_step_probe_ids", []):  # 前回の実行で登録したコールバックIDを取り出す（初回は空）
    print(f"deregister callback registered before run time. uid={callback_id}")  # 解除するIDを表示する
    SimulationManager.deregister_callback(callback_id)  # 二重登録を防ぐため、前回の登録を解除する
builtins._step_probe_ids = []  # 今回登録するIDを入れるリストを作り直す

LOG_STEPS: typing.Final[int] = 6  # ログを出すステップ数の上限（毎ステップ出すとログが埋まるため）
probe_state: ProbeState = {"logged_steps": 0, "pending": []}  # ログを出したステップ数と、予約中の処理を保持する


def schedule_after(steps_from_now: int, action: typing.Callable[[], None], label: str) -> None:
    """今の累計ステップ数から steps_from_now ステップ後の物理ステップ直後（POST）に action を実行するよう予約する。

    Args:
        steps_from_now: 今から何ステップ後に実行するか。1 以上の整数。
        action: 実行する処理。引数を取らず、戻り値を使わない関数。
        label: ログに表示する予約の名前。

    Note:
        呼べる状態: Stop／Play／Pause のどれでも予約できる。予約した処理が実行されるのは、
            物理ステップが進んで POST のコールバックが呼ばれたときだけである。
            Stop 中に予約すると、Play 開始時のウォームアップの2ステップでは POST が呼ばれないため、
            予約したステップ数より遅れて実行される。
        backend: 関係しない。
    """
    steps_now = SimulationManager.get_num_physics_steps()  # 予約した時点の累計ステップ数を取る
    probe_state["pending"].append((steps_now + steps_from_now, steps_now, action, label))  # 実行予定・予約時・処理・名前を記録する
    print(f"[SCHEDULE] {label}: 予約時 num_steps={steps_now} → 実行予定 num_steps={steps_now + steps_from_now}")  # 予約の内容を表示する


def on_pre_step(dt: float, context: omni.physics.core.PhysicsStepContext) -> None:
    """物理ステップの直前に呼ばれ、ステップ直前の累計ステップ数・シミュレーション時刻・刻み幅を表示する。

    Args:
        dt: これから実行する物理ステップの刻み幅 [s]。
        context: 物理ステップの情報。この関数では使わない。
    """
    if probe_state["logged_steps"] < LOG_STEPS:  # 上限に達するまでだけログを出す
        print(f"[PRE ] num_steps={SimulationManager.get_num_physics_steps()} sim_time={SimulationManager.get_simulation_time():.5f} dt={dt:.5f}")  # ステップ直前の計数・時刻・刻み幅を出す


def on_post_step(dt: float, context: omni.physics.core.PhysicsStepContext) -> None:
    """物理ステップの直後に呼ばれ、ログを出し、実行予定に達した予約を実行する。

    Args:
        dt: 実行し終えた物理ステップの刻み幅 [s]。
        context: 物理ステップの情報。この関数では使わない。
    """
    steps_now = SimulationManager.get_num_physics_steps()  # ステップ直後の累計ステップ数を取る
    if probe_state["logged_steps"] < LOG_STEPS:  # 上限に達するまでだけログを出す
        print(f"[POST] num_steps={steps_now} sim_time={SimulationManager.get_simulation_time():.5f} dt={dt:.5f}")  # ステップ直後の計数・時刻・刻み幅を出す
        probe_state["logged_steps"] += 1  # ログを出したステップ数を1つ増やす
    for item in list(probe_state["pending"]):  # 予約中の処理を順に調べる（ループ中に削除するのでコピーを回す）
        target_steps, scheduled_steps, action, label = item  # 予約の内容を取り出す
        if steps_now >= target_steps:  # 実行予定の累計ステップ数に達していれば
            print(f"[RUN ] {label}: num_steps={steps_now}（予約時から {steps_now - scheduled_steps} ステップ後）")  # 実際に何ステップ後だったかを出す
            action()  # 予約された処理を実行する
            probe_state["pending"].remove(item)  # 実行し終えた予約を消す


builtins._step_probe_ids.append(SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP))  # ステップ直前への割り込みを登録する
builtins._step_probe_ids.append(SimulationManager.register_callback(on_post_step, event=SimulationEvent.PHYSICS_POST_STEP))  # ステップ直後への割り込みを登録する
builtins.schedule_after = schedule_after  # 別のScript Editorタブからも予約できるように登録する
print("registered callback ids:", builtins._step_probe_ids)  # 登録できたことを表示する

# 今から2ステップ後に print を実行するよう予約する
schedule_after(2, lambda: print("fired"), "test")  # 予約時と実行時のステップ数が [SCHEDULE] / [RUN ] 行に出る
