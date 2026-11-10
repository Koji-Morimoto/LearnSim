---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, settings]
---

# env_fingerprint

ファイル：[snippets/env_fingerprint.py](../snippets/env_fingerprint.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| simulation view | ステージと物理エンジンの組ごとに1つ作られる view の入口。Play で作られ、Stop で無効になる |
| 環境指紋 | `snippets/env_fingerprint.py` が出力する、設定値・拡張の有効状態・現在の状態の一覧 |
| メタ情報 | Snippet の解説に書く4項目（有効な状態・使うバックエンド・書き込みが届く先・他のバックエンドから見えるタイミング） |
| 上限周波数 | メインループが1秒あたりに行う更新の回数の上限。`/app/runLoops/main/rateLimitFrequency` |
| 目標フレームレート | timeline が持つ値。Play 中のメインループの回数を決める。`omni.timeline.get_timeline_interface().get_target_framerate()` |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

バックエンドの挙動と時間の進み方を左右する設定値、拡張の有効状態、現在の状態を一覧表示する。この出力（環境指紋）を、各実験の観測結果と一緒に記録する。

## 確かめるノート

- [設定一覧](../04_設定/01_設定一覧.md)
- [時間と物理ステップ（「Play・Pause・Stop で時計がどう動くか」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | Stop / Play / Pause のすべて |
| 使うバックエンド | なし（設定と状態を読むだけ） |
| 書き込みが届く先 | なし（ステージにも書かない。下の「注意」） |
| 他のバックエンドから見えるタイミング | 該当なし |

## 注意：Physics Scene がないステージでの副作用

`isaacsim.core.simulation_manager.SimulationManager.get_physics_dt()` と `isaacsim.core.simulation_manager.SimulationManager.is_gpu_dynamics_enabled()` は、ステージに Physics Scene がないと、`/PhysicsScene` を新しく作ってから値を返す（ソース：どちらも内部で `_ensure_physics_scene_prim()` を呼ぶ）。env_fingerprint は、先に `isaacsim.core.simulation_manager.SimulationManager.get_physics_scenes()` で Physics Scene の有無を調べ、ないときはこの2つを呼ばないので、ステージを変えない（ヘッドレスで、Physics Scene のないステージで実行して確かめた）。

## コード

[snippets/env_fingerprint.py](../snippets/env_fingerprint.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/env_fingerprint.py（全文）
```python
# env_fingerprint.py — バックエンドの挙動と時間の進み方を左右する設定値・拡張・状態を一覧表示する（各実験の先頭で実行する）
# 解説: 05_観測Snippet/01_env_fingerprint.md
"""バックエンドの挙動と時間の進み方を左右する設定値・拡張・状態を一覧表示する。

Isaac Sim の Script Editor で実行する。表示した値は、同じ日に取った他の観測結果と並べて記録する。

Note:
    呼べる状態: Stop／Play／Pause のどれでも実行できる。物理シーンがない状態では、
        一部の項目が「取得失敗」と表示される。
    backend: Core Experimental API の prim ラッパーを使わないので、backend の選択は関係しない。
"""

from __future__ import annotations

import typing

import carb.settings  # Kitの設定ストアを読むモジュールを読み込む
import omni.kit.app  # 拡張マネージャを取得するモジュールを読み込む
import omni.timeline  # timelineの状態を読むモジュールを読み込む
import omni.usd  # 現在のステージを取得するモジュールを読み込む
from isaacsim.core.rendering_manager import RenderingManager  # メインループの刻みを読むクラスを読み込む
from isaacsim.core.simulation_manager import SimulationManager  # 物理エンジン・デバイス・ステップの情報を読むクラスを読み込む

if typing.TYPE_CHECKING:
    import omni.ext  # 型ヒントにだけ使う。実行時には import しない
    from pxr import Usd  # 型ヒントにだけ使う。実行時には import しない

settings: carb.settings.ISettings = carb.settings.get_settings()  # 設定ストアのインスタンスを取得する

BACKEND_KEYS: typing.Final[list[str]] = [  # バックエンドの挙動を左右する設定キーの一覧
    "/app/useFabricSceneDelegate",  # FSD：usdrt/fabricバックエンドの前提になる
    "/physics/updateToUsd",  # 物理の計算結果（姿勢）をUSDへ書き戻すか
    "/physics/updateVelocitiesToUsd",  # 速度をUSDへ書き戻すか
    "/physics/updateParticlesToUsd",  # パーティクルの結果をUSDへ書き戻すか
    "/physics/updateForceSensorsToUsd",  # 力覚センサーの値をUSDへ書き戻すか
    "/persistent/physics/resetOnStop",  # Stop時にPlay前の状態へ戻すか
    "/exts/isaacsim.core.simulation_manager/enable_default_callbacks",  # Play/Stop時にviewを作成・無効化するか
    "/exts/isaacsim.core.simulation_manager/default_engine",  # 既定の物理エンジン
    "/physics/cudaDevice",  # 物理に使うGPUの番号
    "/physics/suppressReadback",  # GPUからCPUへの読み戻しを抑制するか
    "/isaacsim/articulation/simStateMode",  # simstateバックエンドの動作モード
]
TIME_KEYS: typing.Final[list[str]] = [  # 時間の進み方を左右する設定キーの一覧
    "/app/player/playSimulations",  # 更新のたびに物理ステップを実行するか
    "/app/player/useFixedTimeStepping",  # timelineを固定刻み(1/C)で進めるか
    "/app/player/useFastMode",  # 固定刻みで、更新が速すぎるときも更新ごとにtimelineを進めるか
    "/app/runLoops/main/rateLimitEnabled",  # メインループの更新回数に上限をかけるか
    "/app/runLoops/main/rateLimitFrequency",  # 上限の値 F
    "/app/runLoops/main/manualModeEnabled",  # メインループを固定の刻みで回すか
    "/persistent/simulation/minFrameRate",  # 1回の更新で実行する物理ステップ数の上限を決める値
    "/app/asyncRendering",  # 描画をメインループと別スレッドで行うか（isaacsim.core.throttling が Stop/Pause で true、Play で false にする）
    "/exts/isaacsim.core.throttling/enable_async",  # isaacsim.core.throttling が状態に応じて asyncRendering を切り替えるか
    "/exts/isaacsim.core.throttling/enable_manualmode",  # isaacsim.core.throttling が状態に応じて manual mode を切り替えるか
]
CHECKED_EXTENSIONS: typing.Final[list[str]] = [  # 有効かどうかを確かめる拡張の一覧
    "omni.physx.fabric",  # fabric を使う挙動の前提になる拡張
    "omni.physx.tensors",  # tensor バックエンドの前提になる拡張
    "isaacsim.physics.newton",  # 物理エンジン Newton の拡張
    "isaacsim.core.experimental.prims",  # Core Experimental API の prim ラッパーの拡張
]

print("=== settings: backend ===")  # バックエンド関係の見出しを出す
for key in BACKEND_KEYS:  # 各キーについて
    print(f"{key} = {settings.get(key)!r}")  # 値を表示する（キーが存在しなければNoneと出る）
print("=== settings: time ===")  # 時間関係の見出しを出す
for key in TIME_KEYS:  # 各キーについて
    print(f"{key} = {settings.get(key)!r}")  # 値を表示する（キーが存在しなければNoneと出る）

timeline: omni.timeline.Timeline = omni.timeline.get_timeline_interface()  # timelineのインスタンスを取得する
has_physics_scene: bool = bool(SimulationManager.get_physics_scenes())  # Physics Scene があるか（ないと get_physics_dt などが /PhysicsScene を作ってしまうため、先に調べる）
ext_manager: omni.ext.ExtensionManager = omni.kit.app.get_app().get_extension_manager()  # 拡張マネージャを取得する
print("=== extensions ===")  # 拡張の見出しを出す
for ext_name in CHECKED_EXTENSIONS:  # 確認する拡張の一覧
    print(f"{ext_name}: enabled={ext_manager.is_extension_enabled(ext_name)}")  # 拡張が有効かどうかを表示する


def show(label: str, getter: typing.Callable[[], object]) -> None:
    """1項目の値を取得して表示する。取得に失敗しても例外を外へ出さない。

    1項目の取得が失敗しても残りの項目を表示し続けるために、例外をこの関数の中で受け止める。

    Args:
        label: 表示する項目名。
        getter: 引数なしで呼ぶと値を返す関数。戻り値の型は項目ごとに異なるので object とする。
    """
    try:  # 取得を試みる
        print(f"{label}: {getter()!r}")  # 取得できた値を表示する
    except Exception as error:  # 物理シーンがない等で失敗した場合は
        print(f"{label}: 取得失敗 ({type(error).__name__}: {error})")  # 失敗したことと理由を表示する


stage: Usd.Stage = omni.usd.get_context().get_stage()  # 現在のステージを取得する
print("=== time ===")  # 時間の見出しを出す
show("stage timeCodesPerSecond (C)", stage.GetTimeCodesPerSecond)  # ステージのtimeCodesPerSecond
show("RenderingManager.get_dt (loop dt, 設定値)", RenderingManager.get_dt)  # メインループの刻みの設定値
show("timeline target framerate", timeline.get_target_framerate)  # timelineの目標フレームレート（Play中のメインループの回数を決める）
if has_physics_scene:  # Physics Scene がある場合だけ読む
    show("physics dt (h)", SimulationManager.get_physics_dt)  # 物理の刻み幅
else:  # Physics Scene がない場合は
    print("physics dt (h): Physics Scene がないため表示しない")  # 読まなかったことを表示する
show("timeline start time (tau_0)", timeline.get_start_time)  # timelineの開始時刻
show("timeline current time (tau)", timeline.get_current_time)  # timelineの現在時刻
show("simulation_time (t)", SimulationManager.get_simulation_time)  # シミュレーション時刻
show("num_physics_steps (n)", SimulationManager.get_num_physics_steps)  # 累計の物理ステップ数
print("=== state ===")  # 状態の見出しを出す
show("timeline.is_playing", timeline.is_playing)  # timelineがPlay中か
show("timeline.is_stopped", timeline.is_stopped)  # timelineがStop中か
show("SimulationManager.is_simulating", SimulationManager.is_simulating)  # シミュレーションが進行中か
show("SimulationManager.is_paused", SimulationManager.is_paused)  # シミュレーションがPause中か
show("active engine", SimulationManager.get_active_physics_engine)  # 実行中の物理エンジン
show("available engines", SimulationManager.get_available_physics_engines)  # 登録されている物理エンジンと有効状態
show("fabric enabled (omni.physx.fabric)", SimulationManager.is_fabric_enabled)  # omni.physx.fabricが有効か
show("physics device", SimulationManager.get_physics_sim_device)  # tensor のデータが置かれるデバイス（/physics/suppressReadback が true のときだけ cuda）
if has_physics_scene:  # Physics Scene がある場合だけ読む
    show("gpu dynamics", SimulationManager.is_gpu_dynamics_enabled)  # Physics Scene の GPU dynamics 属性（物理の計算を GPU で行うか）
else:  # Physics Scene がない場合は
    print("gpu dynamics: Physics Scene がないため表示しない")  # 読まなかったことを表示する
show("tensor simulation view", SimulationManager.get_physics_simulation_view)  # tensorのsimulation view（作られていなければNone）
```

## 観測手順と期待される出力

ここでの「env_fingerprint を実行する」は、上の「コード」の節の env_fingerprint.py（全文）を Script Editor に貼って実行することを指す。

1. Stop・Play・Pause の3つの状態で、それぞれ env_fingerprint を実行する。
2. 3回分の出力を並べて比べる（WinMerge などで3つを並べると、変わった行がすぐ分かる）。
3. 状態によって変わる行と、その値は次のとおりである（2026-11-09 に GUI とヘッドレスの両方で確かめた）。

| 行 | Stop | Play | Pause |
|---|---|---|---|
| `/app/runLoops/main/rateLimitFrequency` | 120 | 60（目標フレームレートの値） | 120 |
| `/app/runLoops/main/manualModeEnabled` | False | True | False |
| `/app/asyncRendering` | True（Stop の10回後から） | False | True（Pause の10回後から） |
| `RenderingManager.get_dt` | 0.00833（= 1/120） | 0.01667（= 1/60） | 0.00833 |
| `timeline current time`、`simulation_time`、`num_physics_steps` | 0（開始時刻に戻る） | 増え続ける | Pause した時点の値のまま |
| `SimulationManager.is_simulating` / `is_paused` | False / False | True / False | True / True |
| `tensor simulation view` | None | view のオブジェクト | Play 中と同じオブジェクト |

### 結果を左右する設定の変え方

env_fingerprint に出る設定のうち、観測結果を大きく変えるものは、次のコードで変えられる。
どれも Script Editor で実行すれば、その場で効く。
変えた値は Isaac Sim を再起動すると元に戻る（`/persistent/` で始まる設定だけは、次回の起動に持ち越される）。

Script Editor に貼って実行するコード（ファイルではない。必要な行だけを実行する）
```python
# 観測結果を大きく変える設定の変え方
import carb.settings  # Kit の設定を読み書きするモジュール
import omni.timeline  # timeline の目標フレームレートを変えるために使う
from isaacsim.core.simulation_manager import SimulationManager  # 物理の出力先を切り替えるクラス

settings = carb.settings.get_settings()  # 設定ストアを取得する

# 物理の計算結果の書き出し先（Play 中に XformPrim や usdrt / fabric で読める値が変わる）
SimulationManager.enable_fabric(True)  # Fabric へ書き、USD へは書かない（/physics/updateToUsd など4つを false にし、omni.physx.fabric を有効にする）
SimulationManager.enable_fabric(False)  # USD へ書く既定の構成に戻す（4つを true にし、omni.physx.fabric を無効にする）

# 1回の更新で実行する物理ステップの回数の上限（Steps per Second ÷ この値）。0 で上限なし。ステージを開き直すと 30 に戻る
settings.set_int("/persistent/simulation/minFrameRate", 0)  # 上限をなくす

# 固定刻みで、メインループの回数が timeCodesPerSecond より多いときも、毎回 timeline を進めるか（true にすると間引かない）
settings.set_bool("/app/player/useFastMode", True)  # 間引かない（シミュレーションが実時間より速く進むことがある）

# Play 中のメインループの回数（timeCodesPerSecond の倍数に丸められる）
omni.timeline.get_timeline_interface().set_target_framerate(120)  # Play 中に1秒あたり120回更新する

# Play 中に物理ステップを実行するか
settings.set_bool("/app/player/playSimulations", False)  # 物理を止めて timeline だけを進める（true に戻しても、進まなかった分は取り戻さない）
```

## 期待と違ったときに疑う箇所

- Play 中に `tensor simulation view` が None の場合、`/exts/isaacsim.core.simulation_manager/enable_default_callbacks` と `/app/player/playSimulations` の値を確かめる。`isaacsim.core.simulation_manager.SimulationManager._on_play()` は、`playSimulations` が false だと view を作らない（ソース）。
- `physics dt (h)` と `gpu dynamics` が「Physics Scene がないため表示しない」になる場合、ステージに Physics Scene がない。Play すると `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` が `/PhysicsScene` を作る。

## 結果

2026-11-09 に GUI で実行した結果である。下の「考察」に、ソースとヘッドレスでの追加の実測で分かったことを書いた。

- 以下の項目のBold部分はPlayON→OFF→ONで変わる。Pause→ONでは変化しない。
  tensor simulation view: <omni.physics.tensors.api.SimulationView object at **0x0000022B6DBD01A0**>
- PlayOFF/Pauseすると描画が高速になるのは `/app/runLoops/main/rateLimitFrequency = 120` の設定が効いている。
  PlayONの時のみ60でそれ以外が120になっているため。

![[env_fingerprint実行結果比較.png|1290]]
### Play OFF
```
Play OFF
=== settings: backend ===
/app/useFabricSceneDelegate = True
/physics/updateToUsd = True
/physics/updateVelocitiesToUsd = True
/physics/updateParticlesToUsd = True
/physics/updateForceSensorsToUsd = None
/persistent/physics/resetOnStop = True
/exts/isaacsim.core.simulation_manager/enable_default_callbacks = True
/exts/isaacsim.core.simulation_manager/default_engine = 'physx'
/physics/cudaDevice = -1
/physics/suppressReadback = False
/isaacsim/articulation/simStateMode = None
=== settings: time ===
/app/player/playSimulations = True
/app/player/useFixedTimeStepping = True
/app/player/useFastMode = False
/app/runLoops/main/rateLimitEnabled = True
/app/runLoops/main/rateLimitFrequency = 120
/app/runLoops/main/manualModeEnabled = False
/persistent/simulation/minFrameRate = 30
=== extensions ===
omni.physx.fabric: enabled=False
omni.physx.tensors: enabled=True
isaacsim.physics.newton: enabled=False
isaacsim.core.experimental.prims: enabled=True
=== time ===
stage timeCodesPerSecond (C): 60.0
RenderingManager.get_dt (loop dt, 設定値): 0.008333333333333333
physics dt (h): 0.016666666666666666
timeline start time (tau_0): 0.0
timeline current time (tau): 0.0
simulation_time (t): 0.0
num_physics_steps (n): 0
=== state ===
timeline.is_playing: False
timeline.is_stopped: True
SimulationManager.is_simulating: False
SimulationManager.is_paused: False
active engine: 'physx'
available engines: [('physx', True)]
fabric enabled (omni.physx.fabric): False
physics device: 'cpu'
gpu dynamics: True
tensor simulation view: None

```

### PlayON
```
Play ON
=== settings: backend ===
/app/useFabricSceneDelegate = True
/physics/updateToUsd = True
/physics/updateVelocitiesToUsd = True
/physics/updateParticlesToUsd = True
/physics/updateForceSensorsToUsd = None
/persistent/physics/resetOnStop = True
/exts/isaacsim.core.simulation_manager/enable_default_callbacks = True
/exts/isaacsim.core.simulation_manager/default_engine = 'physx'
/physics/cudaDevice = -1
/physics/suppressReadback = False
/isaacsim/articulation/simStateMode = None
=== settings: time ===
/app/player/playSimulations = True
/app/player/useFixedTimeStepping = True
/app/player/useFastMode = False
/app/runLoops/main/rateLimitEnabled = True
/app/runLoops/main/rateLimitFrequency = 60.0
/app/runLoops/main/manualModeEnabled = True
/persistent/simulation/minFrameRate = 30
=== extensions ===
omni.physx.fabric: enabled=False
omni.physx.tensors: enabled=True
isaacsim.physics.newton: enabled=False
isaacsim.core.experimental.prims: enabled=True
=== time ===
stage timeCodesPerSecond (C): 60.0
RenderingManager.get_dt (loop dt, 設定値): 0.016666666666666666
physics dt (h): 0.016666666666666666
timeline start time (tau_0): 0.0
timeline current time (tau): 1.3333333333333335
simulation_time (t): 1.3666666666666667
num_physics_steps (n): 82
=== state ===
timeline.is_playing: True
timeline.is_stopped: False
SimulationManager.is_simulating: True
SimulationManager.is_paused: False
active engine: 'physx'
available engines: [('physx', True)]
fabric enabled (omni.physx.fabric): False
physics device: 'cpu'
gpu dynamics: True
tensor simulation view: <omni.physics.tensors.api.SimulationView object at 0x0000022B6DBD01A0>
```

### Pause
```
Pause
=== settings: backend ===
/app/useFabricSceneDelegate = True
/physics/updateToUsd = True
/physics/updateVelocitiesToUsd = True
/physics/updateParticlesToUsd = True
/physics/updateForceSensorsToUsd = None
/persistent/physics/resetOnStop = True
/exts/isaacsim.core.simulation_manager/enable_default_callbacks = True
/exts/isaacsim.core.simulation_manager/default_engine = 'physx'
/physics/cudaDevice = -1
/physics/suppressReadback = False
/isaacsim/articulation/simStateMode = None
=== settings: time ===
/app/player/playSimulations = True
/app/player/useFixedTimeStepping = True
/app/player/useFastMode = False
/app/runLoops/main/rateLimitEnabled = True
/app/runLoops/main/rateLimitFrequency = 120.0
/app/runLoops/main/manualModeEnabled = False
/persistent/simulation/minFrameRate = 30
=== extensions ===
omni.physx.fabric: enabled=False
omni.physx.tensors: enabled=True
isaacsim.physics.newton: enabled=False
isaacsim.core.experimental.prims: enabled=True
=== time ===
stage timeCodesPerSecond (C): 60.0
RenderingManager.get_dt (loop dt, 設定値): 0.008333333333333333
physics dt (h): 0.016666666666666666
timeline start time (tau_0): 0.0
timeline current time (tau): 34.63333333333265
simulation_time (t): 34.666666666666664
num_physics_steps (n): 2080
=== state ===
timeline.is_playing: False
timeline.is_stopped: False
SimulationManager.is_simulating: True
SimulationManager.is_paused: True
active engine: 'physx'
available engines: [('physx', True)]
fabric enabled (omni.physx.fabric): False
physics device: 'cpu'
gpu dynamics: True
tensor simulation view: <omni.physics.tensors.api.SimulationView object at 0x0000022B6DBD01A0>
```

### 考察

1. **view のアドレスが Stop → Play で変わり、Pause → Play で変わらない理由。** 上のメモのとおりである。Play を押すと、`isaacsim.core.simulation_manager.SimulationManager._on_play()` が、view がまだなければ `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` を呼び、simulation view を新しく作る。Stop すると `isaacsim.core.simulation_manager.SimulationManager.invalidate_physics()` が view を無効にして None にするので、次の Play では別のオブジェクトが作られる。Pause では view を消す処理が登録されていないので、再開しても同じオブジェクトのまま使われる（ソース）。
2. **Stop 中・Pause 中に更新が速くなる理由と、その設定を書き換えているもの。** メモのとおり、`/app/runLoops/main/rateLimitFrequency` が Stop 中・Pause 中は 120、Play 中は 60 になっている。この値は、メインループが1秒あたりに行う更新の回数の上限である（`omni.kit.loop-isaac` の `RunLoopRunner.cpp` が、1回の更新を「1 ÷ この値」秒より短く終えたら、残りを待つ）。既定の 120 は `omni.kit.loop-isaac` の `extension.toml` が設定している。Play 中に 60 になるのは、Play を押したときに timeline がこの設定を目標フレームレート（既定 60）に書き換え、Pause・Stop で元の値に戻すからである。Isaac Sim のソースにはこの書き換えはなく（Kit の timeline の中で行われる）、ヘッドレスで次のように確かめた。
   - Stop 中に `rateLimitFrequency` を 200 にしてから Play すると、Play 中は 60、Pause・Stop の後は 200 に戻った。
   - 目標フレームレートを 120 にすると、Play 中は 120 になった。30 にすると（timeCodesPerSecond は 60）、Play 中は 60 になった。timeline のヘッダ（`omni/timeline/ITimeline.h`）の説明どおり、Play 中の回数は timeCodesPerSecond の倍数に丸められる。
   - `RenderingManager.get_dt` は、`rateLimitFrequency` の逆数を返しているだけである（0.00833 = 1/120、0.01667 = 1/60）。
3. **Play 中だけ `manualModeEnabled` が True になる理由。** 拡張機能 isaacsim.core.throttling が、`/exts/isaacsim.core.throttling/enable_manualmode = true` のとき、Play でメインループの manual mode を有効にし、Pause・Stop で無効にする（`isaacsim/core/throttling/extension.py`）。manual mode が有効だと、メインループは更新ごとの時間の進みを固定の刻み（既定 1/60 秒）にする。ただし既定の固定刻み（`/app/player/useFixedTimeStepping = true`）では、timeline の進みは timeCodesPerSecond で決まるので、manual mode の有無で timeline の進みは変わらない（[時間と物理ステップ](../03_時間と物理ステップ/01_時間と物理ステップ.md) の「1回の更新で timeline 時刻が進む時間」の節）。
4. **`gpu dynamics: True` なのに `physics device: 'cpu'` になる理由。** 2つは別のものを表している。`gpu dynamics` は Physics Scene の `physxScene:enableGPUDynamics` 属性で、物理の計算を GPU で行うかを表す。`physics device` は、tensor のデータが置かれるデバイスである。`isaacsim.core.simulation_manager.SimulationManager.get_device()` は、`/physics/suppressReadback` が true（GPU から CPU へ結果を読み戻さない）で、かつ GPU を使う Physics Scene の場合にだけ cuda を返し、それ以外は cpu を返す（ソース）。GUI の既定は `/physics/suppressReadback = False` なので、物理を GPU で計算していても、結果は CPU に読み戻され、tensor のデータは CPU に置かれる。
5. **`omni.physx.fabric: enabled=False`、`/physics/updateToUsd = True` の意味。** GUI の既定では、物理の計算結果は USD に書き戻され、Fabric へは直接書かれない。それでも usdrt / fabric のバックエンドで最新の値が読めるのは、Fabric Scene Delegate（`/app/useFabricSceneDelegate = True`）が USD の変更を Fabric に写すからである（[pose_freshness_probe](05_pose_freshness_probe.md) の考察）。
6. **`/physics/cudaDevice = -1`。** GUI の既定は -1（GPU の番号を指定しない）だった。`/physics/suppressReadback` を true にして GPU のデータを使う構成にすると、`isaacsim.core.simulation_manager.SimulationManager.get_device()` は -1 のときに 0 に書き換え、警告を出す（ソース）。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [MOC-観測Snippet](00_MOC-観測Snippet.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [step_probe](02_step_probe.md) |
