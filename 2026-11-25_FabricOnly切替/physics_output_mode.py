"""物理シミュレーション結果の書き込み先（出力先）を切り替える。

出力先とは、PhysX が 1 ステップごとに計算した剛体の位置・姿勢・速度を書き込む場所を指す。
- "usd"        : USD ステージに書き込む（Isaac Sim の既定）。Property 欄や保存ファイルに値が残る。
- "fabric_cpu" : Fabric だけに書き込み、USD には書き戻さない（Fabric のみ）。
- "fabric_gpu" : "fabric_cpu" に加えて GPU 上で Fabric へ直接渡す。GPU 物理のシーン向け。

GUI ではメインメニュー Window > Simulation Output Settings で開く
"Simulation Output Settings" ウィンドウの USD / Fabric CPU / Fabric GPU ボタンと同じ操作をする。
ボタンとの違いは、速度・粒子の USD 書き戻し（updateVelocitiesToUsd / updateParticlesToUsd）も
あわせて止めること（ボタンは位置の書き戻しだけを止める）。

使い方（Script Editor に貼る、または python_server に送る）:
    set_output_mode("fabric_cpu")   # USD 書き戻し OFF
    set_output_mode("usd")          # 元に戻す
    print_output_mode()

注意:
- 再生中に切り替えず、Stop してから切り替える。
- updateToUsd などはステージごとの設定で、ステージを開き直すと既定値（USD 書き戻し ON）に戻る。
  そのときは再度 set_output_mode() を実行する。
- Stop すると USD 側は再生前の値のままなので、物体は再生前の位置に表示が戻る。
"""

from typing import Literal, get_args

import carb.settings
import omni.kit.app

OutputMode = Literal["usd", "fabric_cpu", "fabric_gpu"]

FABRIC_EXTENSION = "omni.physx.fabric"

# omni.physx の設定パス（omni.physx.bindings._physx の SETTING_* と同じ値）
UPDATE_TO_USD = "/physics/updateToUsd"
UPDATE_VELOCITIES_TO_USD = "/physics/updateVelocitiesToUsd"
UPDATE_PARTICLES_TO_USD = "/physics/updateParticlesToUsd"
# omni.physx.fabric の設定パス
FABRIC_ENABLED = "/physics/fabricEnabled"
FABRIC_UPDATE_TRANSFORMATIONS = "/physics/fabricUpdateTransformations"
FABRIC_UPDATE_VELOCITIES = "/physics/fabricUpdateVelocities"
FABRIC_UPDATE_POINTS = "/physics/fabricUpdatePoints"
FABRIC_USE_GPU_INTEROP = "/physics/fabricUseGPUInterop"

_ALL_KEYS = (
    UPDATE_TO_USD,
    UPDATE_VELOCITIES_TO_USD,
    UPDATE_PARTICLES_TO_USD,
    FABRIC_ENABLED,
    FABRIC_UPDATE_TRANSFORMATIONS,
    FABRIC_UPDATE_VELOCITIES,
    FABRIC_UPDATE_POINTS,
    FABRIC_USE_GPU_INTEROP,
)


def _usd_default(settings: carb.settings.ISettings, path: str) -> bool:
    """USD 書き戻し系の設定の既定値（/defaults に登録された値。無ければ True）。"""
    value = settings.get("/defaults" + path)
    return True if value is None else bool(value)


def set_output_mode(mode: OutputMode) -> None:
    """物理シミュレーション結果の出力先を mode に切り替える。"""
    if mode not in get_args(OutputMode):
        raise ValueError(f"mode は {get_args(OutputMode)} のどれか: {mode!r}")

    settings = carb.settings.get_settings()
    use_fabric = mode != "usd"

    if use_fabric:
        # Fabric 拡張を先に有効にする（無効のままだと Fabric に何も書かれない）
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.is_extension_enabled(FABRIC_EXTENSION):
            manager.set_extension_enabled_immediate(FABRIC_EXTENSION, True)

    settings.set_bool(UPDATE_TO_USD, not use_fabric)
    if use_fabric:
        settings.set_bool(UPDATE_VELOCITIES_TO_USD, False)
        settings.set_bool(UPDATE_PARTICLES_TO_USD, False)
    else:
        settings.set_bool(UPDATE_VELOCITIES_TO_USD, _usd_default(settings, UPDATE_VELOCITIES_TO_USD))
        settings.set_bool(UPDATE_PARTICLES_TO_USD, _usd_default(settings, UPDATE_PARTICLES_TO_USD))

    settings.set_bool(FABRIC_ENABLED, use_fabric)
    settings.set_bool(FABRIC_UPDATE_TRANSFORMATIONS, use_fabric)
    settings.set_bool(FABRIC_UPDATE_VELOCITIES, use_fabric)
    settings.set_bool(FABRIC_UPDATE_POINTS, use_fabric)
    settings.set_bool(FABRIC_USE_GPU_INTEROP, mode == "fabric_gpu")

    # Fabric 拡張は無効にしない。有効のままでも fabricEnabled=False なら USD 出力になる
    # （GUI の USD ボタンは拡張を無効にするが、他の機能が依存している場合に備えて残す）。
    print(f"[physics_output_mode] 出力先を {mode} にしました")
    print_output_mode()


def get_output_mode() -> dict:
    """現在の出力先に関わる設定値と Fabric 拡張の有効状態を返す。"""
    settings = carb.settings.get_settings()
    manager = omni.kit.app.get_app().get_extension_manager()
    state = {key: settings.get(key) for key in _ALL_KEYS}
    state[FABRIC_EXTENSION + " enabled"] = manager.is_extension_enabled(FABRIC_EXTENSION)
    return state


def print_output_mode() -> None:
    for key, value in get_output_mode().items():
        print(f"  {key} = {value}")


if __name__ == "__main__":
    # Script Editor でそのまま実行したときは USD 書き戻し OFF（Fabric のみ）にする
    set_output_mode("fabric_cpu")
