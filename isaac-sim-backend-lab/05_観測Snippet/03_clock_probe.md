---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, timeline, physics-step]
---

# clock_probe

ファイル：[snippets/clock_probe.py](../snippets/clock_probe.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| メタ情報 | Snippet の解説に書く4項目（有効な状態・使うバックエンド・書き込みが届く先・他のバックエンドから見えるタイミング） |
| メインループ | Kit アプリケーションが終了まで繰り返し実行する処理の流れ |
| 更新 | メインループの1回分の反復。`omni.kit.app.get_app().update()` の1回にあたる |
| timeline 時刻 | `omni.timeline` が管理する時刻 |
| 物理ステップ | 物理エンジンが物理の状態を刻み幅 h 秒だけ進める1回分の計算 |
| 刻み幅 h | 物理ステップ1回で進める時間（秒）。Physics Scene の Steps per Second の逆数 |
| シミュレーション時刻 | Play を開始してから実行された物理ステップの数（ウォームアップの2回を含む）に、刻み幅 h を掛けた時刻。`isaacsim.core.simulation_manager.SimulationManager.get_simulation_time()` で読める |
| ウォームアップ | Play を開始したときに `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` が実行する2回の物理ステップ |
| lag | timeline が開始時刻から進んだ量 (τ − τ0) から、シミュレーション時刻 t を引いた値 |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

描画フレームごとに、更新の実時間、timeline の進み、物理ステップ数、lag を1行で表示する。あわせて、lag が「刻み幅 h の −2 倍以上、−1 倍未満」（−2h ≤ lag < −h）に収まっているかを判定する。

lag が負になるのは、Play 開始時のウォームアップで物理ステップが2回進み、その分もシミュレーション時刻に数えられるからである。この範囲の導出は [時間と物理ステップ（「timeline 時刻とシミュレーション時刻のずれ」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md) にある。

## 出力の列と、ノートの記号の対応

| 列 | 意味 | ノートの記号 |
|---|---|---|
| `wall_dt` | 前回の表示からの実時間（ミリ秒） | $\Delta w$ |
| `d_tau` | 前回の表示からの timeline の進み（秒） | $\Delta\tau$ |
| `N` | 前回の表示からの物理ステップ数 | $N$ |
| `n` | 累計の物理ステップ数（ウォームアップの2回を含む） | $w + n$ |
| `tau-tau0` | timeline が開始時刻から進んだ量 | $\tau - \tau_0$ |
| `t` | シミュレーション時刻 | $t$ |
| `lag` | `tau-tau0` から `t` を引いた値 | $(\tau - \tau_0) - t$ |
| `h` | 物理の刻み幅 | $h$ |
| `in_range` | `lag` が −2h 以上、−h 未満なら True | $-2h \le (\tau - \tau_0) - t < -h$ |

描画フレームのイベント（NEW_FRAME）は、平均すると更新1回につき1回出る。ただし、1回の更新だけを見ると、0 回のことも 2 回のこともある（下の「考察」）。

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | Play（Stop 中と Pause 中は物理ステップが実行されないので、`N` は 0 になる。Stop 中は lag が 0 なので `in_range` は False になる） |
| 使うバックエンド | なし（時計と計数を読むだけ） |
| 書き込みが届く先 | なし |
| 他のバックエンドから見えるタイミング | 該当なし |

## コード

[snippets/clock_probe.py](../snippets/clock_probe.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/clock_probe.py（全文）
```python
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
```

## 観測手順と期待される出力

ここでの「clock_probe を実行する」は、上の「コード」の節の clock_probe.py（全文）を Script Editor に貼って実行することを指す。実行すると、描画フレームごとに30行が出て止まる。

S は Physics Scene の Steps per Second、C はステージの timeCodesPerSecond である。
どちらも Stop 中に変えてから clock_probe を実行し、Play する。変え方は下の「S と C をコードで変える」にある。
期待される `N` の並びは、2026-11-09 にヘッドレスで実測した値である（GUI と同じ設定 `isaacsim.exp.full.kit`）。

| 手順 | 設定 | 実測した `N` の並び | `in_range` |
|---|---|---|---|
| 1 | 既定（S = 60、C = 60） | 1, 1, 1, … | 常に True（lag は −0.0333 で一定） |
| 2 | S = 120、C = 60 | 2, 2, 2, … | 常に True（lag は −0.0167 で一定） |
| 3 | S = 100、C = 30、minFrameRate 30（既定） | 3, 0, 3, 0, … | 最初は True だが、lag が 0.0033 ずつ増えて False になる |
| 3' | S = 100、C = 30、minFrameRate 0 | 3, 0, 3, 0, 4, 0, … | 常に True |
| 4 | S = 30、C = 60 | 0, 1, 0, 1, … | 常に True |

手順3・3' で 0 が1つおきに入るのは、メインループが1秒に60回まわるのに timeline は1秒に30回しか進まないからである（間引き）。
手順3で4回目が出ないのは、minFrameRate 30 では1回の更新の物理ステップが「100 ÷ 30 = 3.33 → 3 回」までに打ち切られるからである。
詳しくは [時間と物理ステップ](../03_時間と物理ステップ/01_時間と物理ステップ.md) の「数値例」と「このずれの範囲が成り立たない場面」の節にある。

### S と C をコードで変える

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# Steps per Second（S）と timeCodesPerSecond（C）と minFrameRate を変える
import carb.settings  # minFrameRate の設定を変えるために使う
import omni.timeline  # timeline の timeCodesPerSecond を変えるために使う
import omni.usd  # ステージの timeCodesPerSecond を変えるために使う
from isaacsim.core.simulation_manager import SimulationManager  # 物理の刻み幅を変えるために使う

STEPS_PER_SECOND = 100  # S：物理ステップを1秒あたり何回行うか
TIME_CODES_PER_SECOND = 30  # C：timeline を1秒あたり何コマ進めるか
MIN_FRAME_RATE = 0  # 1回の更新の物理ステップ数の上限を決める値。0 なら上限なし（既定は 30）

SimulationManager.set_physics_dt(1.0 / STEPS_PER_SECOND)  # 刻み幅 h = 1/S にする（Physics Scene の timeStepsPerSecond を書き換える）
omni.usd.get_context().get_stage().SetTimeCodesPerSecond(TIME_CODES_PER_SECOND)  # ステージの timeCodesPerSecond を変える
omni.timeline.get_timeline_interface().set_time_codes_per_second(TIME_CODES_PER_SECOND)  # timeline にも同じ値を設定する
carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", MIN_FRAME_RATE)  # minFrameRate を変える（ステージを開き直すと 30 に戻る）
print("h =", SimulationManager.get_physics_dt())  # 刻み幅が変わったことを確かめる
```

### 追加の観測

5. Play 直後の数フレームで、`n` が 2 から始まっていること（ウォームアップの2回）を見る。実測では、Stop 中に実行して Play すると、Play 後の最初の行で `N=2 n=4` になった（1回の更新で物理ステップが2回進み、累計はウォームアップの2回を足した4）。
6. Pause 中に次のコードを実行してから Play を再開し、`lag` が物理ステップ3回分（−3h）ずれることを見る。実測では、S = 60、C = 60 で lag が −0.0333 から −0.0833 になり、再開後もそのまま変わらなかった。

Script Editor に貼って実行するコード（ファイルではない）
```python
# Pause 中に物理ステップを3回だけ手動で進める
from isaacsim.core.simulation_manager import SimulationManager  # 手動ステップの API を持つクラス

SimulationManager.step(steps=3)  # 物理ステップを3回実行する
```

7. Stop の前後で `tau-tau0` を見る。実測では、Stop すると timeline 時刻は開始時刻に戻り、`tau-tau0`・`t`・`n` はすべて 0 になった。

### 手順をコードで自動実行する

Play / Pause / Stop のボタンは、`omni.timeline` のメソッドで代わりに押せる。`timeline.play()` などは呼んだ時点では状態を変えず、次のメインループの更新で反映される。続けて結果を見るには、`async` の関数の中で `await omni.kit.app.get_app().next_update_async()` を使って更新を待つ。

Script Editor に貼って実行するコード（ファイルではない。先に clock_probe を Stop 中に実行しておく）
```python
# Play → 40回更新 → Pause → 手動ステップ3回 → Play → 40回更新 → Stop を、ボタンを押さずに行う
import asyncio  # async の関数を Kit のイベントループで走らせるために使う

import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う
from isaacsim.core.simulation_manager import SimulationManager  # 手動ステップに使う


async def run_steps() -> None:
    """clock_probe の追加の観測5〜7をまとめて行う。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    timeline.play()  # Play（次の更新で反映される）
    for _ in range(40):  # 40回分の更新を待つ
        await app.next_update_async()  # 更新を1回待つ
    timeline.pause()  # Pause
    await app.next_update_async()  # Pause が反映されるまで1回待つ
    SimulationManager.step(steps=3)  # 手動ステップを3回行う
    exec(open(r"C:\isaacsim-6.2.0\CodeByClaude\isaac-sim-backend-lab\snippets\clock_probe.py", encoding="utf-8").read(), {})  # 表示を30行分やり直すために clock_probe を再実行する
    timeline.play()  # 再開
    for _ in range(40):  # 40回分の更新を待つ
        await app.next_update_async()  # 更新を1回待つ
    timeline.stop()  # Stop


asyncio.ensure_future(run_steps())  # Kit のイベントループで run_steps を走らせる（この行はすぐに戻る）
```

## 期待と違ったときに疑う箇所

- `d_tau` が timeCodesPerSecond の逆数の秒数にならない場合は、`/app/player/useFixedTimeStepping` と `/app/player/useFastMode` の値を確かめる。固定刻み（`useFixedTimeStepping = true`）で、メインループの回数が timeCodesPerSecond より多い場合は、`d_tau` が 0 の行が混ざる（間引き。`useFastMode = false` のとき）。
- `N` の並びが表と違う場合は、`/persistent/simulation/minFrameRate` の値を確かめる。Steps per Second ÷ minFrameRate（切り捨て）を超える回数は打ち切られる。ステージを開き直すと 30 に戻る。
- `in_range` が False になる場合は、minFrameRate による打ち切り、手動ステップ（`isaacsim.core.simulation_manager.SimulationManager.step()`）、`/app/player/playSimulations = false` のどれかが起きていないかを確かめる。どれも、lag をずらしたまま戻さない。

## 結果

2026-11-09 に GUI で実行した結果である。この結果の `in_range` は、lag を「0 以上、刻み幅未満」で判定するコードで出したものである（今のコードの判定とは範囲が違う。下の考察1〜3）。
下の「考察」に、ソースとヘッドレスでの追加の実測で分かったことを書いた。

```
# PlayOFF
physics dt h = 0.016666666666666666  loop dt(設定値) = 0.008333333333333333
frame=  0 wall_dt=   1.76ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  1 wall_dt=   8.80ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  2 wall_dt=   9.06ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  3 wall_dt=  17.34ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  4 wall_dt=   9.62ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  5 wall_dt=   0.31ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  6 wall_dt=  19.81ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  7 wall_dt=   0.35ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  8 wall_dt=  19.69ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame=  9 wall_dt=   0.40ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 10 wall_dt=  19.11ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 11 wall_dt=   0.36ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 12 wall_dt=  19.67ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 13 wall_dt=   0.38ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 14 wall_dt=  19.44ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 15 wall_dt=   0.35ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 16 wall_dt=  19.67ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 17 wall_dt=   0.34ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 18 wall_dt=  19.70ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 19 wall_dt=   0.36ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 20 wall_dt=  19.67ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 21 wall_dt=   0.34ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 22 wall_dt=  19.66ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 23 wall_dt=   0.39ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 24 wall_dt=  19.65ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 25 wall_dt=   0.34ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 26 wall_dt=  19.18ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 27 wall_dt=   0.37ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 28 wall_dt=  20.16ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True
frame= 29 wall_dt=   0.36ms d_tau=0.00000 N=0 n=0 tau-tau0=0.00000 t=0.00000 lag=+0.00000 h=0.01667 in_range=True

# PlayON
physics dt h = 0.016666666666666666  loop dt(設定値) = 0.016666666666666666
frame=  0 wall_dt=   8.28ms d_tau=0.00000 N=0 n=172 tau-tau0=2.83333 t=2.86667 lag=-0.03333 h=0.01667 in_range=False
frame=  1 wall_dt=  16.66ms d_tau=0.01667 N=1 n=173 tau-tau0=2.85000 t=2.88333 lag=-0.03333 h=0.01667 in_range=False
frame=  2 wall_dt=  16.86ms d_tau=0.01667 N=1 n=174 tau-tau0=2.86667 t=2.90000 lag=-0.03333 h=0.01667 in_range=False
frame=  3 wall_dt=  16.54ms d_tau=0.01667 N=1 n=175 tau-tau0=2.88333 t=2.91667 lag=-0.03333 h=0.01667 in_range=False
frame=  4 wall_dt=  16.59ms d_tau=0.01667 N=1 n=176 tau-tau0=2.90000 t=2.93333 lag=-0.03333 h=0.01667 in_range=False
frame=  5 wall_dt=  16.67ms d_tau=0.01667 N=1 n=177 tau-tau0=2.91667 t=2.95000 lag=-0.03333 h=0.01667 in_range=False
frame=  6 wall_dt=  16.69ms d_tau=0.01667 N=1 n=178 tau-tau0=2.93333 t=2.96667 lag=-0.03333 h=0.01667 in_range=False
frame=  7 wall_dt=  16.65ms d_tau=0.01667 N=1 n=179 tau-tau0=2.95000 t=2.98333 lag=-0.03333 h=0.01667 in_range=False
frame=  8 wall_dt=  16.67ms d_tau=0.01667 N=1 n=180 tau-tau0=2.96667 t=3.00000 lag=-0.03333 h=0.01667 in_range=False
frame=  9 wall_dt=  16.87ms d_tau=0.01667 N=1 n=181 tau-tau0=2.98333 t=3.01667 lag=-0.03333 h=0.01667 in_range=False
frame= 10 wall_dt=  16.56ms d_tau=0.01667 N=1 n=182 tau-tau0=3.00000 t=3.03333 lag=-0.03333 h=0.01667 in_range=False
frame= 11 wall_dt=  16.62ms d_tau=0.01667 N=1 n=183 tau-tau0=3.01667 t=3.05000 lag=-0.03333 h=0.01667 in_range=False
frame= 12 wall_dt=  16.64ms d_tau=0.01667 N=1 n=184 tau-tau0=3.03333 t=3.06667 lag=-0.03333 h=0.01667 in_range=False
frame= 13 wall_dt=  16.80ms d_tau=0.01667 N=1 n=185 tau-tau0=3.05000 t=3.08333 lag=-0.03333 h=0.01667 in_range=False
frame= 14 wall_dt=  16.50ms d_tau=0.01667 N=1 n=186 tau-tau0=3.06667 t=3.10000 lag=-0.03333 h=0.01667 in_range=False
frame= 15 wall_dt=  16.70ms d_tau=0.01667 N=1 n=187 tau-tau0=3.08333 t=3.11667 lag=-0.03333 h=0.01667 in_range=False
frame= 16 wall_dt=  16.75ms d_tau=0.01667 N=1 n=188 tau-tau0=3.10000 t=3.13333 lag=-0.03333 h=0.01667 in_range=False
frame= 17 wall_dt=  16.56ms d_tau=0.01667 N=1 n=189 tau-tau0=3.11667 t=3.15000 lag=-0.03333 h=0.01667 in_range=False
frame= 18 wall_dt=  16.77ms d_tau=0.01667 N=1 n=190 tau-tau0=3.13333 t=3.16667 lag=-0.03333 h=0.01667 in_range=False
frame= 19 wall_dt=  16.65ms d_tau=0.01667 N=1 n=191 tau-tau0=3.15000 t=3.18333 lag=-0.03333 h=0.01667 in_range=False
frame= 20 wall_dt=  16.56ms d_tau=0.01667 N=1 n=192 tau-tau0=3.16667 t=3.20000 lag=-0.03333 h=0.01667 in_range=False
frame= 21 wall_dt=  16.72ms d_tau=0.01667 N=1 n=193 tau-tau0=3.18333 t=3.21667 lag=-0.03333 h=0.01667 in_range=False
frame= 22 wall_dt=  16.71ms d_tau=0.01667 N=1 n=194 tau-tau0=3.20000 t=3.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 23 wall_dt=  16.75ms d_tau=0.01667 N=1 n=195 tau-tau0=3.21667 t=3.25000 lag=-0.03333 h=0.01667 in_range=False
frame= 24 wall_dt=  16.56ms d_tau=0.01667 N=1 n=196 tau-tau0=3.23333 t=3.26667 lag=-0.03333 h=0.01667 in_range=False
frame= 25 wall_dt=  16.65ms d_tau=0.01667 N=1 n=197 tau-tau0=3.25000 t=3.28333 lag=-0.03333 h=0.01667 in_range=False
frame= 26 wall_dt=  16.90ms d_tau=0.01667 N=1 n=198 tau-tau0=3.26667 t=3.30000 lag=-0.03333 h=0.01667 in_range=False
frame= 27 wall_dt=  16.69ms d_tau=0.01667 N=1 n=199 tau-tau0=3.28333 t=3.31667 lag=-0.03333 h=0.01667 in_range=False
frame= 28 wall_dt=  16.62ms d_tau=0.01667 N=1 n=200 tau-tau0=3.30000 t=3.33333 lag=-0.03333 h=0.01667 in_range=False
frame= 29 wall_dt=  16.61ms d_tau=0.01667 N=1 n=201 tau-tau0=3.31667 t=3.35000 lag=-0.03333 h=0.01667 in_range=False

# Pause
physics dt h = 0.016666666666666666  loop dt(設定値) = 0.008333333333333333
frame=  0 wall_dt=   1.13ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  1 wall_dt=   0.29ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  2 wall_dt=   9.12ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  3 wall_dt=   9.45ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  4 wall_dt=  18.07ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  5 wall_dt=   0.39ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  6 wall_dt=  20.05ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  7 wall_dt=   0.36ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  8 wall_dt=  19.16ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame=  9 wall_dt=   0.37ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 10 wall_dt=  19.64ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 11 wall_dt=   0.38ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 12 wall_dt=  19.16ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 13 wall_dt=   0.35ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 14 wall_dt=  20.15ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 15 wall_dt=   0.38ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 16 wall_dt=  19.65ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 17 wall_dt=   0.37ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 18 wall_dt=  19.68ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 19 wall_dt=   0.38ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 20 wall_dt=  19.62ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 21 wall_dt=   0.38ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 22 wall_dt=  19.61ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 23 wall_dt=   0.37ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 24 wall_dt=  19.18ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 25 wall_dt=   0.37ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 26 wall_dt=  20.14ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 27 wall_dt=   0.35ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 28 wall_dt=  19.18ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
frame= 29 wall_dt=   0.39ms d_tau=0.00000 N=0 n=314 tau-tau0=5.20000 t=5.23333 lag=-0.03333 h=0.01667 in_range=False
```

### 考察

1. **Play 中に `in_range` が毎回 False だった理由。** Play 中の lag は毎回 −0.03333 秒（= −2h）だった。このときの判定は「0 以上、刻み幅未満」だったので、毎回 False になった。lag が −2h になるのは、Play 開始時のウォームアップで物理ステップが2回進み、その2回もシミュレーション時刻と累計ステップ数に数えられるからである（[step_probe](02_step_probe.md) の考察3）。実際、`n=172` のとき `t = 172 × h = 2.86667` で、`tau-tau0 = 170 × h = 2.83333` と、ちょうど2ステップ分ずれている。正しい範囲は −2h 以上 −h 未満で、今のコードではヘッドレスでの実測で Play 中は毎回 True になった。
2. **Pause 中も lag が −0.03333 のまま変わらない理由。** Pause 中は timeline 時刻も物理ステップも止まるので、lag も Pause した時点の値のまま変わらない。今のコードでは、Pause 中も True になる。
3. **Stop 中の lag が 0 で、判定が True だった理由。** Stop 中は timeline 時刻が開始時刻に戻り、シミュレーション時刻も 0 に戻るので、lag は 0 になる。このときの判定の範囲には 0 が入っていたので True になった。今のコードでは False になる。Stop 中の判定には意味がない。
4. **Stop 中と Pause 中に `wall_dt` が約 19.7 ms と約 0.35 ms を交互にくり返す理由。** Stop 中と Pause 中は、描画フレームのイベントが2回ずつまとまって届いている。2つを足すと約 20 ms で、2フレームで約 20 ms、つまり1フレームあたり約 10 ms になる。これは、Stop 中と Pause 中のメインループの上限周波数 120（1回の更新が約 8.3 ms 以上）とおおむね合う。まとまって届くのは、拡張機能 isaacsim.core.throttling が、Stop 中と Pause 中に描画を別スレッドで行う設定（`/app/asyncRendering`）を有効にするからである。この拡張は、Play を押すと `/app/asyncRendering` を false にし、Pause・Stop の後は10回の更新を待ってから true に戻す（`isaacsim/core/throttling/extension.py`。ヘッドレスでの実測で、Play 中は false、Pause・Stop の10回後は true だった）。描画が別スレッドで進むと、NEW_FRAME のイベントはメインループの更新に合わせて溜まった分がまとめて届く。Play 中は描画がメインループの中で行われるので、`wall_dt` は 16.7 ms で揃う。
5. **Play 中の `wall_dt` が 16.7 ms で、Stop 中・Pause 中が約 8.3〜10 ms になる理由。** `loop dt(設定値)` が、Play 中は 0.01667（1秒に60回）、Stop 中と Pause 中は 0.00833（1秒に120回）になっている。これは `/app/runLoops/main/rateLimitFrequency` の値（60 と 120）の逆数である。Play を押すと、timeline がこの設定を目標フレームレート（既定 60）に書き換え、Pause・Stop で元の 120 に戻す（ヘッドレスでの実測。Stop 中に 200 にしておくと、Play 中は 60、Pause・Stop の後は 200 に戻った）。詳しくは [env_fingerprint](01_env_fingerprint.md) の考察にある。
6. **Play 直後の最初の行で `d_tau=0` になる理由。** clock_probe を実行した直後の最初のフレームは、前回の表示の基準（`tau_prev`）を実行した時点の値で初期化しているので、進みが 0 になる。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [step_probe](02_step_probe.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [bench_probe](04_bench_probe.md) |
