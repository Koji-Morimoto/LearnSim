# clock_probe.py — 描画フレームごとに、timelineの進み・物理ステップ数・lag（timelineが開始時刻から進んだ量からシミュレーション時刻を引いた値）を表示し、
#                  lag が「刻み幅の −2 倍以上、−1 倍未満」に収まるかを判定する（導出: 03_時間と物理ステップ/01_時間と物理ステップ.md）
# 解説: 05_観測Snippet/03_clock_probe.md
"""描画フレームごとに時計の値を表示し、lag = (τ − τ0) − t が −2h ≤ lag < −h を満たすかを判定する。

τ は timeline 時刻、τ0 は timeline の開始時刻、t はシミュレーション時刻、h は物理の刻み幅である。
lag は「timeline が開始時刻から進んだ量 (τ − τ0)」から「シミュレーション時刻 t」を引いた値である。
Play 開始時のウォームアップで物理ステップが2回進み、t にも数えられるため、Play 中の lag は −2h 以上 −h 未満になる。

Isaac Sim の Script Editor で実行し、Play する。再実行すると、前回登録したコールバックを解除してから登録し直す。

Note:
    呼べる状態: Stop／Play／Pause のどれでも登録できる。lag の判定に意味があるのは Play 中である。
    backend: Core Experimental API の prim ラッパーを使わないので、backend の選択は関係しない。
"""

from __future__ import annotations

import builtins  # 再実行をまたいで登録IDを保持するために使う
import time  # 実時間（wall clock）の計測に使う
import typing

import omni.timeline  # timeline時刻を読むために使う
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager  # 描画フレームのイベントとloop dtの取得に使う
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # 物理ステップのイベントと時刻・計数の取得に使う

if typing.TYPE_CHECKING:
    import carb.eventdispatcher  # 型ヒントにだけ使う。実行時には import しない
    import omni.physics.core  # 型ヒントにだけ使う。実行時には import しない


class ProbeState(typing.TypedDict):
    """観測の途中経過。辞書のキーと値の型をエディタに伝えるための定義。"""

    frame: int  # 表示したフレーム数
    steps_in_frame: int  # 前回の表示から今回までに実行された物理ステップ数
    wall_prev: float  # 前回の表示の実時刻 [s]（time.perf_counter の値）
    tau_prev: float  # 前回の表示のtimeline時刻 [s]


previous_ids = getattr(builtins, "_clock_probe", None)  # 前回の実行で登録したIDを取り出す（なければNone）
if previous_ids:  # 前回の登録が残っていれば
    SimulationManager.deregister_callback(previous_ids["step_id"])  # 物理ステップのコールバックを解除する
    RenderingManager.deregister_callback(previous_ids["frame_id"])  # 描画フレームのコールバックを解除する

FRAMES: typing.Final[int] = 30  # 何フレーム分表示するか
EPS: typing.Final[float] = 1e-6  # 浮動小数点の丸めを許す幅（秒）
timeline: omni.timeline.Timeline = omni.timeline.get_timeline_interface()  # timelineのインスタンスを取得する
probe_state: ProbeState = {  # 観測の状態を保持する辞書
    "frame": 0,  # 表示したフレーム数
    "steps_in_frame": 0,  # 前回の表示から今回までに実行された物理ステップ数
    "wall_prev": time.perf_counter(),  # 前回の表示の実時刻
    "tau_prev": timeline.get_current_time(),  # 前回の表示のtimeline時刻
}


def on_post_step(dt: float, context: omni.physics.core.PhysicsStepContext) -> None:
    """物理ステップの直後に呼ばれ、前回の表示から実行された物理ステップ数を1つ数える。

    Args:
        dt: 実行し終えた物理ステップの刻み幅 [s]。この関数では使わない。
        context: 物理ステップの情報。この関数では使わない。
    """
    probe_state["steps_in_frame"] += 1  # 物理ステップ数を1つ数える


def on_new_frame(event: carb.eventdispatcher.Event) -> None:
    """描画フレームが1つできるたびに呼ばれ、時計の値と lag の判定結果を1行で表示する。

    lag は、timeline が開始時刻から進んだ量 (τ − τ0) からシミュレーション時刻 t を引いた値である。
    判定では、lag が刻み幅 h の −2 倍以上、−1 倍未満（−2h ≤ lag < −h）に収まっているかを調べる。

    Args:
        event: 描画フレームのイベント。この関数では使わない。
    """
    if probe_state["frame"] >= FRAMES:  # 表示の上限に達したら
        return  # 何もしない
    now = time.perf_counter()  # 今の実時刻を取る
    tau = timeline.get_current_time()  # timeline時刻 τ を読む
    tau0 = timeline.get_start_time()  # timelineの開始時刻 τ0 を読む
    sim_time = SimulationManager.get_simulation_time()  # シミュレーション時刻 t を読む
    step_dt = SimulationManager.get_physics_dt()  # 物理の刻み幅 h を読む
    lag = (tau - tau0) - sim_time  # timelineが開始時刻から進んだ量から、シミュレーション時刻を引く
    in_range = (-2 * step_dt - EPS <= lag) and (lag < -step_dt + EPS)  # lag が −2h ≤ lag < −h に入っているかを判定する
    print(  # 1フレーム分の観測結果を1行で表示する
        f"frame={probe_state['frame']:3d} "  # フレーム番号
        f"wall_dt={(now - probe_state['wall_prev']) * 1000:7.2f}ms "  # 前回の表示からの実時間（更新の実時間 Δw の実測値）
        f"d_tau={tau - probe_state['tau_prev']:.5f} "  # 前回の表示からのtimelineの進み（Δτ）
        f"N={probe_state['steps_in_frame']} "  # 前回の表示からの物理ステップ数（N）
        f"n={SimulationManager.get_num_physics_steps()} "  # 累計の物理ステップ数（n。ウォームアップの2回を含む）
        f"tau-tau0={tau - tau0:.5f} t={sim_time:.5f} lag={lag:+.5f} h={step_dt:.5f} in_range={in_range}"  # lagの計算に使った各値と判定結果
    )
    probe_state["wall_prev"] = now  # 次の表示の基準実時刻を更新する
    probe_state["tau_prev"] = tau  # 次の表示の基準timeline時刻を更新する
    probe_state["steps_in_frame"] = 0  # 物理ステップ数の数え直しを始める
    probe_state["frame"] += 1  # 表示したフレーム数を増やす


builtins._clock_probe = {  # 今回の登録IDを保存する
    "step_id": SimulationManager.register_callback(on_post_step, event=SimulationEvent.PHYSICS_POST_STEP),  # 物理ステップ直後への割り込みを登録する
    "frame_id": RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=on_new_frame),  # 描画フレームへの割り込みを登録する
}
print(f"physics dt h = {SimulationManager.get_physics_dt()}  loop dt(設定値) = {RenderingManager.get_dt()}")  # 2つの刻み幅を最初に表示する
