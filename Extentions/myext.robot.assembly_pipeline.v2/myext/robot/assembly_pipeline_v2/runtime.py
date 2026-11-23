"""Isaac Sim（Kit）ランタイムのフレーム進行・Timeline 制御・プローブ用ステージ管理を行うモジュール。

責務:
  1. Kit アプリケーションのフレーム更新（``step``）および非同期コルーチンの同期駆動（``run_async``）。
  2. Timeline の再生・停止制御（``play_and_step`` / ``stop``）。
  3. 動的プローブ用の使い捨てステージ生成（``new_probe_stage``）と ``carb`` 警告ログの収集（``capture_warnings``）。

主な公開 API:
  - ``app()`` / ``timeline()``: Kit アプリおよび Timeline インターフェース取得
  - ``step(n=1)`` / ``update(n=1)``: Kit メインループを ``n`` フレーム進める
  - ``run_async(coro)``: ``RobotAssembler.finish_assemble()`` 等の非同期処理を完了まで進める
  - ``play_and_step(frames)`` / ``stop()``: 物理シミュレーションの再生と安全な停止
  - ``new_probe_stage(...)``: CPU 物理の新しい空ステージを USD コンテキストに作成する
  - ``capture_warnings()``: コンテキスト内の ``LEVEL_WARN`` 以上の ``carb`` ログを収集する

パイプライン全体での位置づけ:
  ``probes.py``・``official.py``・``kinematics_check.py``・``pipeline.run_pipeline`` から呼ばれる
  低レイヤの実行基盤。なお ``pipeline.build_assembly``（非破壊ビルド）は ``new_probe_stage`` や
  Timeline 操作を呼ばない設計になっている。

手を入れやすい箇所:
  - プローブ時の物理シーン設定（広域衝突判定や GPU 物理の切り替え等）を変える場合は ``new_probe_stage`` を編集する。
"""

from __future__ import annotations

import asyncio
import contextlib

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


def app():
    """現在の Kit Application インスタンスを返す。副作用なし。"""
    return omni.kit.app.get_app()


def step(n: int = 1) -> None:
    """Kit の更新フレームを ``n`` 回進める。

    副作用: USD 変更通知・PhysX シミュレーション・非同期タスクが ``n`` ステップ進行する。
    """
    kit_app = app()
    for _ in range(n):
        kit_app.update()


def update(n: int = 1) -> None:
    """``step(n)`` の互換エイリアス。"""
    step(n)


def run_async(coro) -> object:
    """非同期コルーチンを Kit の ``update()`` ループで完了まで進めて戻り値を返す。

    Isaac Sim の ``RobotAssembler.finish_assemble()`` のように Kit のイベントループ上で
    進行するコルーチンを同期関数内から待機するために使う。

    Args:
        coro: 実行するコルーチン。

    Returns:
        コルーチンの返却値。

    副作用:
        完了まで ``app().update()`` を繰り返し呼ぶ。
    """
    task = asyncio.ensure_future(coro)
    kit_app = app()
    while not task.done():
        kit_app.update()
    return task.result()


def timeline():
    """グローバルな Timeline インターフェースを返す。副作用なし。"""
    return omni.timeline.get_timeline_interface()


def play_and_step(frames: int) -> None:
    """Timeline を Play し、指定フレーム数だけ進める。

    副作用: グローバル Timeline の再生状態を開始し、物理シミュレーションを進める。
    """
    timeline().play()
    step(frames)


def stop() -> None:
    """Timeline が停止していなければ Stop し、PhysX 状態の解放を待つため 2 フレーム進める。

    副作用: グローバル Timeline を停止状態に戻す。
    """
    if not timeline().is_stopped():
        timeline().stop()
        step(2)


def new_probe_stage(
    up_axis: str | float = "Z",
    mpu: float | str = 1.0,
    gravity: float = 9.81,
    *,
    meters_per_unit: float | None = None,
) -> Usd.Stage:
    """グローバル USD コンテキストに使い捨ての空ステージを作り、CPU 物理シーンを定義して返す。

    ``up_axis`` が数値で渡された場合は旧位置引数順 ``(gravity, up_axis, meters_per_unit)`` として
    解釈し、文字列の場合は現行引数順 ``(up_axis, mpu, gravity)`` として解釈する。

    CPU 物理（``EnableGPUDynamics=False`` / ``BroadphaseType="MBP"``）を使う理由:
    単体ロボットの短時間プローブでは GPU 初期化コストを避け、決定性と安定性を優先するため。

    Args:
        up_axis: ステージの上方向軸（``"Z"`` または ``"Y"``）。数値指定時は重力加速度として解釈。
        mpu: ``metersPerUnit``（既定 ``1.0``）。旧引数順では上方向軸文字列。
        gravity: 重力加速度の大きさ [m/s^2]（無重力プローブ時は ``0.0``）。
        meters_per_unit: ``metersPerUnit`` の明示キーワード指定（``mpu`` より優先）。

    Returns:
        新規作成された ``Usd.Stage``。

    副作用:
        Timeline を停止し、グローバル USD コンテキストで開いている既存ステージを閉じて破棄する。
    """
    if isinstance(up_axis, (int, float)):
        real_gravity = float(up_axis)
        real_up_axis = str(mpu) if isinstance(mpu, str) else "Z"
        real_mpu = float(meters_per_unit if meters_per_unit is not None else (gravity if gravity != 9.81 else 1.0))
    else:
        real_up_axis = str(up_axis)
        real_mpu = float(meters_per_unit if meters_per_unit is not None else mpu)
        real_gravity = float(gravity)

    stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    step(2)
    stage = ctx.get_stage()
    is_z = real_up_axis.upper() == "Z"
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z if is_z else UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, real_mpu)
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1) if is_z else Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(real_gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr("MBP")
    return stage


@contextlib.contextmanager
def capture_warnings():
    """``with`` ブロック実行中に発生した ``carb`` の警告・エラーログをリストに収集する。

    PhysX が Play 時に出力する閉ループ警告・ネスト Articulation 警告・無効ジョイント警告を
    捕捉し、規則評価（R07 / R09 等）の根拠として使う。

    Yields:
        ``list[str]``: ``LEVEL_WARN`` 以上のログ文字列が逐次追記されるリスト。

    副作用:
        ブロック中のみ ``carb.logging`` にコールバックを登録し、終了時に必ず解除する。
    """
    logs: list[str] = []

    def on_log(source, level, filename, lineno, msg):  # noqa: ANN001
        if level >= carb.logging.LEVEL_WARN:
            logs.append(f"{source}: {msg}".strip())

    sub = carb.logging.acquire_logging().add_logger(on_log)
    try:
        yield logs
    finally:
        carb.logging.acquire_logging().remove_logger(sub)


__all__ = [
    "app",
    "step",
    "update",
    "run_async",
    "timeline",
    "play_and_step",
    "stop",
    "new_probe_stage",
    "capture_warnings",
]
