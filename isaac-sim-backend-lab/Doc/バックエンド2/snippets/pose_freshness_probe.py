# pose_freshness_probe.py — 同じ剛体の高さzを4つの経路で描画フレームごとに読み比べ、どの経路が物理の最新値を返すかを観測する
# 解説: 05_観測Snippet/05_pose_freshness_probe.md
"""同じ剛体の高さ z を4つの経路で描画フレームごとに読み比べ、どの経路が物理の最新値を返すかを観測する。

Isaac Sim の Script Editor で実行し、Play する。PRIM_PATH は自分のシーンの剛体に合わせて書き換える。
結果は「物理の計算結果をどこへ書き出すか」の設定で変わるので、実行時にその設定を表示する。
OUTPUT_MODE を変えると、その設定を切り替えてから観測できる。

Note:
    呼べる状態: 登録は Stop／Play／Pause のどれでもできる。比べる意味があるのは、剛体が動いている Play 中である。
    backend: XformPrim は既定の選択（usd）・usdrt・fabric、RigidPrim は既定の選択（Play 中と Pause 中は tensor、Stop 中は usd）で読む。
"""

from __future__ import annotations

import builtins  # 再実行をまたいで登録IDを保持するために使う
import typing

import carb.settings  # 物理の出力先の設定を読み書きするために使う
import isaacsim.core.experimental.utils.backend as backend_utils  # バックエンドを指定するuse_backendを使う
import omni.timeline  # 現在の状態（Stop / Play / Pause）を表示するために使う
from isaacsim.core.experimental.prims import RigidPrim, XformPrim  # 比べる2種類のラッパー
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager  # 描画フレームごとに呼ばれるイベントを使う
from isaacsim.core.simulation_manager import SimulationManager  # 物理ステップ数の表示と、物理の出力先の切り替えに使う

if typing.TYPE_CHECKING:
    import carb.eventdispatcher  # 型ヒントにだけ使う。実行時には import しない

# use_backend が受け付ける backend 名。これ以外の文字列はエディタが警告する
BackendName: typing.TypeAlias = typing.Literal["usd", "usdrt", "fabric", "tensor", "simstate"]
# 物理の計算結果の書き出し先。None は「今の設定のまま変えない」
OutputMode: typing.TypeAlias = typing.Literal["usd", "fabric", "none"] | None


class ProbeState(typing.TypedDict):
    """観測の途中経過。辞書のキーと値の型をエディタに伝えるための定義。"""

    frame: int  # 表示したフレーム数


PRIM_PATH: typing.Final[str] = "/World/Cube"  # 観測する剛体のパス（自分のシーンに合わせて書き換える）
FRAMES: typing.Final[int] = 10  # 何フレーム分表示するか
OUTPUT_MODE: OutputMode = None  # None: 変えない／"usd": USDへ書く（既定）／"fabric": Fabricへ書きUSDへは書かない／"none": どちらにも書かない

settings = carb.settings.get_settings()  # 設定ストアを取得する
if OUTPUT_MODE == "usd":  # USD へ書く構成にする場合
    SimulationManager.enable_fabric(False)  # omni.physx.fabric を無効にし、/physics/updateToUsd などを true にする
elif OUTPUT_MODE == "fabric":  # Fabric へ書く構成にする場合
    SimulationManager.enable_fabric(True)  # omni.physx.fabric を有効にし、/physics/updateToUsd などを false にする
elif OUTPUT_MODE == "none":  # どちらにも書かない構成にする場合
    SimulationManager.enable_fabric(False)  # omni.physx.fabric を無効にする
    for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateForceSensorsToUsd"):  # USD へ書き戻す4つの設定
        settings.set_bool(key, False)  # USD への書き戻しも止める

print(  # 結果を左右する設定と状態を、観測結果と一緒に残せるように表示する
    f"[SETTINGS] /physics/updateToUsd={settings.get('/physics/updateToUsd')} "  # 物理の姿勢を USD に書き戻すか
    f"omni.physx.fabric={SimulationManager.is_fabric_enabled()} "  # 物理の姿勢を Fabric に直接書くか
    f"/app/useFabricSceneDelegate={settings.get('/app/useFabricSceneDelegate')} "  # usdrt / fabric バックエンドの前提（FSD）
    f"state={'Play' if omni.timeline.get_timeline_interface().is_playing() else ('Stop' if omni.timeline.get_timeline_interface().is_stopped() else 'Pause')}"  # 実行した時点の状態
)

previous_id = getattr(builtins, "_pose_probe_id", None)  # 前回登録したコールバックIDを取り出す（初回はNone）
if previous_id is not None:  # 前回の登録が残っていれば
    RenderingManager.deregister_callback(previous_id)  # 二重登録を防ぐために解除する

xform = XformPrim(PRIM_PATH)  # 姿勢取得の候補が usd→usdrt→fabric の順のラッパー
rigid = RigidPrim(PRIM_PATH)  # 姿勢取得の候補が tensor→usd→usdrt→fabric の順のラッパー
probe_state: ProbeState = {"frame": 0}  # 表示したフレーム数


def read_z(prim: XformPrim | RigidPrim, backend: BackendName | None) -> str:
    """指定した backend で、1つ目のプリムの世界座標の z を読み、表示用の文字列にして返す。

    backend が使えない場合も観測を続けるために、例外をこの関数の中で受け止めて文字列で返す。

    Args:
        prim: 読む対象のラッパー。
        backend: 使う backend の名前。None なら既定の選択に任せる。

    Returns:
        z [m] を小数点以下4桁で表した文字列。読めなかった場合は「ERR(例外の種類)」。

    Note:
        呼べる状態: Stop／Play／Pause のどれでも呼べる。usd・usdrt・fabric は3つの状態のすべてで読める（実測）。
        backend: 引数 backend で指定する。指定した backend で処理されなければ例外にする。
    """
    try:  # バックエンドが使えない場合に備える
        if backend is None:  # バックエンドを指定しない場合（既定の選択に任せる）
            positions, _ = prim.get_world_poses()  # 既定のバックエンドで姿勢を読む
        else:  # バックエンドを指定する場合
            with backend_utils.use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):  # 指定どおりでなければ例外にする
                positions, _ = prim.get_world_poses()  # 指定したバックエンドで姿勢を読む
        return f"{positions.numpy()[0][2]:.4f}"  # 1つ目のプリムのzを文字列にして返す（positions の shape: (プリム数, 3)）
    except Exception as error:  # 使えなかった場合は
        return f"ERR({type(error).__name__})"  # 例外の種類を返す


def on_new_frame(event: carb.eventdispatcher.Event) -> None:
    """描画フレームごとに呼ばれ、4つの経路で読んだ z を1行で表示する。

    Args:
        event: 描画フレームのイベント。この関数では使わない。
    """
    if probe_state["frame"] >= FRAMES:  # 上限に達したら
        return  # 何もしない
    print(  # 1フレーム分の観測結果を1行で表示する
        f"frame={probe_state['frame']:2d} steps={SimulationManager.get_num_physics_steps():4d} "  # フレーム番号と累計ステップ数
        f"Xform既定={read_z(xform, None)} Xform_usdrt={read_z(xform, 'usdrt')} "  # XformPrimの既定(usd)とusdrt
        f"Xform_fabric={read_z(xform, 'fabric')} Rigid既定={read_z(rigid, None)}"  # XformPrimのfabricとRigidPrimの既定
    )
    probe_state["frame"] += 1  # 表示したフレーム数を増やす


builtins._pose_probe_id = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=on_new_frame)  # 描画フレームごとの割り込みを登録する
print("pose probe registered")  # 登録できたことを表示する
