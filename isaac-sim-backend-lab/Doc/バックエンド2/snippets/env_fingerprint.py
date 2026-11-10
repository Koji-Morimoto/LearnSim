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
