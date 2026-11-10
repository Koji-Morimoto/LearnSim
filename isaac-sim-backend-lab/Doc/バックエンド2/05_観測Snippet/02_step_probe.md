---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, physics-step]
---

# step_probe

ファイル：[snippets/step_probe.py](../snippets/step_probe.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| simulation view | ステージと物理エンジンの組ごとに1つ作られる view の入口。Play で作られ、Stop で無効になる |
| メタ情報 | Snippet の解説に書く4項目（有効な状態・使うバックエンド・書き込みが届く先・他のバックエンドから見えるタイミング） |
| 物理ステップ | 物理エンジンが物理の状態を刻み幅 h 秒だけ進める1回分の計算 |
| 刻み幅 h | 物理ステップ1回で進める時間（秒）。Physics Scene の Steps per Second の逆数 |
| 累計ステップ数 | `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` が返す、Play を開始してから実行された物理ステップの数。Play 開始時のウォームアップの2回を含み、Stop で 0 に戻る |
| ウォームアップ | Play を開始したときに `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` が実行する2回の物理ステップ |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

物理ステップの直前と直後にログを出す。あわせて、「今の累計ステップ数から、指定したステップ数の後に処理を実行する」予約の仕組み（`schedule_after`）を提供し、実際に何ステップ後に実行されたかを記録する。

## 確かめるノート

- [時間と物理ステップ（「全体像：メインループの1回の更新で起きること」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)
- [時間と物理ステップ（「手動ステップ」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)

## 使っている API（手元のソースで確認した）

| API | 内容 |
|---|---|
| `isaacsim.core.simulation_manager.SimulationManager.register_callback(fn, event=isaacsim.core.simulation_manager.SimulationEvent.PHYSICS_PRE_STEP)` | 物理ステップの直前に fn を呼ぶ。fn は `(dt, context)` を受け取る |
| `isaacsim.core.simulation_manager.SimulationManager.register_callback(fn, event=isaacsim.core.simulation_manager.SimulationEvent.PHYSICS_POST_STEP)` | 物理ステップの直後に fn を呼ぶ |
| `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` | 累計ステップ数 |
| `isaacsim.core.simulation_manager.SimulationManager.deregister_callback(uid)` | 登録を解除する |

PRE / POST のコールバックは、tensor の simulation view ができた後にだけ呼ばれる。
`isaacsim.core.simulation_manager.SimulationManager.register_callback()` は、登録された関数を次の条件付きの関数で包んでから、物理ステップのイベントに登録している。

simulation_manager.py（isaacsim.core.simulation_manager, v6.2.0）抜粋。日本語のコメントを加えた。抜粋なので、このままでは実行できない。
```python
# simulation_manager.py（v6.2.0）抜粋：register_callback が PRE / POST のコールバックを包む部分
def on_event(step_dt: float, context: Any) -> Any:  # 物理ステップのイベントから呼ばれる関数
    return callback(step_dt, context) if cls._simulation_view_created else None  # view ができるまでは、登録された関数を呼ばない
```

`cls._simulation_view_created` が True になるのは、ウォームアップの2回目の物理ステップが終わった後である（`isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` の最後で True にする）。一方、累計ステップ数は C++ 側（`isaacsim.core.simulation_manager` の `PluginInterface.cpp` の `onPhysicsStep`）が物理ステップのイベントを直接受けて数えている。そのため、**ウォームアップの2回は累計ステップ数に数えられるが、PRE / POST のコールバックは呼ばれない**。この差が、下の「結果」で予約が1ステップ遅れて実行される原因である。

## メタ情報

| 項目                 | 内容                                                  |
| ------------------ | --------------------------------------------------- |
| 有効な状態              | Play（Stop 中や Pause 中に登録してもよい。ログが出るのは物理ステップが進んだときだけ） |
| 使うバックエンド           | なし（物理ステップのイベントと計数を読むだけ）                             |
| 書き込みが届く先           | なし                                                  |
| 他のバックエンドから見えるタイミング | 該当なし                                                |

## コード

[snippets/step_probe.py](../snippets/step_probe.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/step_probe.py（全文）
```python
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
```

## 観測手順と期待される出力

ここでの「step_probe を実行する」は、上の「コード」の節の step_probe.py（全文）を Script Editor に貼って実行することを指す。step_probe は、実行の最後に「今から2ステップ後に `fired` を表示する」予約を1つ入れる。

| 手順  | 操作                                                           | 期待される出力（2026-11-09 の実測で確かめた）                                                                                         |
| --- | ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------- |
| 1   | Stop 中に step_probe を実行する                                     | `registered callback ids: [...]` と `[SCHEDULE] test: 予約時 num_steps=0 → 実行予定 num_steps=2` だけが出る。PRE / POST のログは出ない    |
| 2   | Play を押す                                                     | 最初の `[PRE ]` が `num_steps=2` から始まる（ウォームアップの2回が数えられている）。<br>`[RUN ] test` は「予約時から 3 ステップ後」に出る。PRE / POST が6ステップ分出て止まる |
| 3   | Play 中に step_probe をもう一度実行する                                 | 前回の登録を解除した ID が `deregister callback ...` 行に出る。<br>`[RUN ] test` は「予約時から 2 ステップ後」に出る                                 |
| 4   | Pause を押し、別のタブで下のコードを実行する                                    | `[SCHEDULE] pause-test` だけが出て、Pause している間は `[RUN ]` が出ない                                                             |
| 5   | Play を押して再開する                                                | `[RUN ] pause-test` が「予約時から 2 ステップ後」に出る                                                                              |
| 6   | Stop を押す。step_probe は再実行せずに、別のタブで下のコードの数字を 5 にして実行し、Play を押す | `[RUN ]` が「予約時から 5 ステップ後」に出る。<br>Stop をまたいでもコールバックは登録されたまま動く                                                         |

手順4と6で実行するコード（Script Editor に貼って実行するコード。ファイルではない）
```python
# 今から2ステップ後に print を実行するよう予約する。schedule_after は step_probe.py が builtins に登録した関数なので、import せずに呼べる
schedule_after(2, lambda: print("fired in pause test"), "pause-test")  # 予約時と実行時の累計ステップ数が [SCHEDULE] / [RUN ] 行に出る
```

手順6で「予約時から 5 ステップ後」になり、手順2のように1ステップ遅れないのは、予約したときの累計ステップ数（0）から5ステップ後の「5」が、ウォームアップの2回（累計 2）より後だからである。

### 手順をコードで自動実行する

Play / Pause / Stop のボタンは、`omni.timeline` のメソッドで代わりに押せる。`timeline.play()` などは呼んだ時点では状態を変えず、次のメインループの更新で反映される（実測）。続けて結果を見るには、更新を待つ必要がある。Script Editor では、次のように `async` の関数を作り、`asyncio.ensure_future()` で走らせる。`await omni.kit.app.get_app().next_update_async()` が、更新1回分を待つ。

Script Editor に貼って実行するコード（ファイルではない。先に step_probe を Stop 中に実行しておく）
```python
# step_probe の手順2・4・5・6を、ボタンを押さずにコードで行う
import asyncio  # async の関数を Kit のイベントループで走らせるために使う

import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う


async def run_steps() -> None:
    """Play → Pause 中に予約 → 再開 → Stop → 再生、の順に状態を切り替える。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    
    timeline.play()  # 手順2：Play（次の更新で反映される）
    for _ in range(10):  # 10回分の更新を待つ
        await app.next_update_async()  # 更新を1回待つ
        
    timeline.pause()  # 手順4：Pause
    await app.next_update_async()  # Pause が反映されるまで1回待つ
    
    schedule_after(2, lambda: print("fired in pause test"), "pause-test")  # Pause 中に予約する
    for _ in range(5):  # Pause のまま5回更新しても [RUN ] が出ないことを確かめる
        await app.next_update_async()  # 更新を1回待つ
        
    timeline.play()  # 手順5：再開
    for _ in range(5):  # 予約が実行されるまで待つ
        await app.next_update_async()  # 更新を1回待つ
    timeline.stop()  # 手順6：Stop
    await app.next_update_async()  # Stop が反映されるまで1回待つ
    
    schedule_after(5, lambda: print("fired after stop"), "across-stop")  # 再実行せずに予約する
    
    timeline.play()  # もう一度 Play
    for _ in range(10):  # 予約が実行されるまで待つ
        await app.next_update_async()  # 更新を1回待つ
    timeline.stop()  # 最後に Stop に戻す


asyncio.ensure_future(run_steps())  # Kit のイベントループで run_steps を走らせる（この行はすぐに戻る）
```

`timeline.commit()` を呼ぶと、次の更新を待たずに、その場で状態の切り替えが反映される。Play の場合は、ウォームアップの2回の物理ステップと view の作成も `commit()` の中で終わる（実測：`play()` の直後は `is_playing()` が False、`commit()` の直後は True で、累計ステップ数は 2 だった）。

## 期待と違ったときに疑う箇所

- Play してもログが出ない場合、tensor の simulation view が作られていない可能性がある。[env_fingerprint](01_env_fingerprint.md) で `tensor simulation view` を確かめる。
- 6ステップ分のログを出し終えると、それ以降は PRE / POST のログを出さない（`LOG_STEPS`）。ログを見直したいときは step_probe を再実行する。予約（`[SCHEDULE]` / `[RUN ]`）の表示には、この上限はかからない。

## 結果

2026-11-09 に GUI で実行した結果である。下の「考察」に、ソースとヘッドレスでの追加の実測で分かったことを書いた。

- `probe["logged"] += 1` しているのは `on_post_step` のみでからpre/postのどちらも `LOG_STEPS: typing.Final[int] = 6 ` 回実行されている
- `sim_time` が`post`→`pre`の時に全く進んでいないのはまだ物理ステップが実行される前だから。
- PlayOFF状態から実行すると2ステップ後に予約していても3ステップ後に実行された。warmup時の物理ステップではイベント発火しないようになってる？

```
# PlayOFF状態でコード実行→PlayON
deregister callback registered before run time. uid=14
deregister callback registered before run time. uid=15
registered callback ids: [16, 17]

[SCHEDULE] test: 予約時 num_steps=0 → 実行予定 num_steps=2

[PRE ] num_steps=2 sim_time=0.03333 dt=0.01667
[POST] num_steps=3 sim_time=0.05000 dt=0.01667

[RUN ] test: num_steps=3（予約時から 3 ステップ後）
fired

[PRE ] num_steps=3 sim_time=0.05000 dt=0.01667
[POST] num_steps=4 sim_time=0.06667 dt=0.01667
[PRE ] num_steps=4 sim_time=0.06667 dt=0.01667
[POST] num_steps=5 sim_time=0.08333 dt=0.01667
[PRE ] num_steps=5 sim_time=0.08333 dt=0.01667
[POST] num_steps=6 sim_time=0.10000 dt=0.01667
[PRE ] num_steps=6 sim_time=0.10000 dt=0.01667
[POST] num_steps=7 sim_time=0.11667 dt=0.01667
[PRE ] num_steps=7 sim_time=0.11667 dt=0.01667
[POST] num_steps=8 sim_time=0.13333 dt=0.01667

# PlayON中に実行
deregister callback registered before run time. uid=16
deregister callback registered before run time. uid=17
registered callback ids: [18, 19]

[SCHEDULE] test: 予約時 num_steps=518 → 実行予定 num_steps=520

[PRE ] num_steps=518 sim_time=8.63333 dt=0.01667
[POST] num_steps=519 sim_time=8.65000 dt=0.01667
[PRE ] num_steps=519 sim_time=8.65000 dt=0.01667
[POST] num_steps=520 sim_time=8.66667 dt=0.01667

[RUN ] test: num_steps=520（予約時から 2 ステップ後）
fired

[PRE ] num_steps=520 sim_time=8.66667 dt=0.01667
[POST] num_steps=521 sim_time=8.68333 dt=0.01667
[PRE ] num_steps=521 sim_time=8.68333 dt=0.01667
[POST] num_steps=522 sim_time=8.70000 dt=0.01667
[PRE ] num_steps=522 sim_time=8.70000 dt=0.01667
[POST] num_steps=523 sim_time=8.71667 dt=0.01667
[PRE ] num_steps=523 sim_time=8.71667 dt=0.01667
[POST] num_steps=524 sim_time=8.73333 dt=0.01667

# PlayON中に実行
deregister callback registered before run time. uid=18
deregister callback registered before run time. uid=19
registered callback ids: [20, 21]

[SCHEDULE] test: 予約時 num_steps=993 → 実行予定 num_steps=995

[PRE ] num_steps=993 sim_time=16.55000 dt=0.01667
[POST] num_steps=994 sim_time=16.56667 dt=0.01667
[PRE ] num_steps=994 sim_time=16.56667 dt=0.01667
[POST] num_steps=995 sim_time=16.58333 dt=0.01667

[RUN ] test: num_steps=995（予約時から 2 ステップ後）
fired

[PRE ] num_steps=995 sim_time=16.58333 dt=0.01667
[POST] num_steps=996 sim_time=16.60000 dt=0.01667
[PRE ] num_steps=996 sim_time=16.60000 dt=0.01667
[POST] num_steps=997 sim_time=16.61667 dt=0.01667
[PRE ] num_steps=997 sim_time=16.61667 dt=0.01667
[POST] num_steps=998 sim_time=16.63333 dt=0.01667
[PRE ] num_steps=998 sim_time=16.63333 dt=0.01667
[POST] num_steps=999 sim_time=16.65000 dt=0.01667

# PlayON→Pause中にコード実行後、再度PlayON
deregister callback registered before run time. uid=20
deregister callback registered before run time. uid=21
registered callback ids: [22, 23]

[SCHEDULE] test: 予約時 num_steps=8482 → 実行予定 num_steps=8484

[PRE ] num_steps=8482 sim_time=141.36667 dt=0.01667
[POST] num_steps=8483 sim_time=141.38333 dt=0.01667
[PRE ] num_steps=8483 sim_time=141.38333 dt=0.01667
[POST] num_steps=8484 sim_time=141.40000 dt=0.01667

[RUN ] test: num_steps=8484（予約時から 2 ステップ後）
fired

[PRE ] num_steps=8484 sim_time=141.40000 dt=0.01667
[POST] num_steps=8485 sim_time=141.41667 dt=0.01667
[PRE ] num_steps=8485 sim_time=141.41667 dt=0.01667
[POST] num_steps=8486 sim_time=141.43333 dt=0.01667
[PRE ] num_steps=8486 sim_time=141.43333 dt=0.01667
[POST] num_steps=8487 sim_time=141.45000 dt=0.01667
[PRE ] num_steps=8487 sim_time=141.45000 dt=0.01667
[POST] num_steps=8488 sim_time=141.46667 dt=0.01667
```

### PlayOFF状態の物理ステップ

```python
print(SimulationManager.get_num_physics_steps()) # 0を出力した。
```

### 考察

1. **PRE と POST がどちらも6回出る理由。** `on_pre_step` と `on_post_step` は、どちらも「ログを出したステップ数が `LOG_STEPS`（6）未満か」を調べてからログを出す。この数を1つ増やすのは `on_post_step` だけである。1回の物理ステップでは PRE → POST の順に呼ばれるので、6回目のステップの PRE の時点ではまだ数が5で、PRE も POST も出る。7回目のステップの PRE の時点で数が6になっているので、どちらも出なくなる。上のメモのとおりである（今の版のコードでは、この数の名前を `probe["logged"]` から `probe_state["logged_steps"]` に変えた）。
2. **POST → 次の PRE で `sim_time` が変わらない理由。** シミュレーション時刻は、物理ステップが1回終わるたびに書き換わる。POST は「ステップが終わった直後」、次の PRE は「次のステップを始める直前」なので、その間に物理ステップは実行されていない。上のメモのとおりである。PRE の `num_steps` と `sim_time` は「これから実行するステップの前の値」、POST は「実行し終えた後の値」と読む。
3. **Stop から実行すると予約が3ステップ後になる理由。** メモの「warmup 時の物理ステップではイベント発火しないようになってる？」は、半分正しい。物理ステップのイベントそのものは、ウォームアップの2回でも発生している。累計ステップ数を数える C++ 側の処理はこのイベントを受けて数えるので、最初の PRE の時点で `num_steps=2` になっている。一方、step_probe の PRE / POST は `isaacsim.core.simulation_manager.SimulationManager.register_callback()` 経由で登録しているので、上の「使っている API」の節のとおり、view ができるまで（ウォームアップの2回目が終わるまで）は呼ばれない。予約の実行予定は `num_steps=2` だが、その時点では `on_post_step` が呼ばれないので、最初に呼ばれた `num_steps=3` の POST で実行された。ヘッドレスで同じ操作をしても、同じ出力になった。
4. **Play 中・Pause 中に予約した場合。** ウォームアップは Play を開始したときにしか起きないので、予約したとおり2ステップ後に実行される（上の2つ目以降の出力）。Pause 中に予約した場合も、Pause の間は実行されず、再開してから2ステップ後に実行された（ヘッドレスで確認した。手順4・5）。
5. **Stop をまたいだ場合。** Stop しても、`isaacsim.core.simulation_manager.SimulationManager.register_callback()` で登録したコールバックは解除されない（解除されるのは、ステージを開き直したときと閉じたとき）。step_probe を再実行せずに Stop → Play しても、予約は実行された（ヘッドレスで確認した。手順6）。
6. **Stop 中の累計ステップ数が 0 になる理由。** Stop すると、C++ 側の `onStop` が累計ステップ数を 0 に戻す。最後のメモのとおりである。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [env_fingerprint](01_env_fingerprint.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [clock_probe](03_clock_probe.md) |
