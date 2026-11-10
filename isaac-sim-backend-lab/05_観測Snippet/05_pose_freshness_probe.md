---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, snippet, backend, fabric, usd]
---

# pose_freshness_probe

ファイル：[snippets/pose_freshness_probe.py](../snippets/pose_freshness_probe.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| 姿勢 | 位置と向きの組。スケールは含まない |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| 子の view | simulation view から、プリムのパスを指定して作る view。ラッパーは tensor で読み書きするためにこれを内部に持つ |
| フォールバック | 選ばれたバックエンドが使えないために、別のバックエンドに差し替わること |
| 物理の出力先 | 物理エンジンが計算した姿勢を、毎ステップどこへ書き出すか。USD・Fabric・どちらにも書かない、の3通りがある |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

落下する剛体の高さ（世界座標の z）を、次の4つの経路で描画フレームごとに読み比べる。値が更新ごとに変わる列が、物理の最新値を返す経路である。どの列が変わるかは「物理の出力先」の設定で決まるので、Snippet はその設定を最初の行（`[SETTINGS]`）に表示する。

| 列 | ラッパー | バックエンド |
|---|---|---|
| `Xform既定` | `isaacsim.core.experimental.prims.XformPrim` | 指定なし（usd が選ばれる） |
| `Xform_usdrt` | `isaacsim.core.experimental.prims.XformPrim` | usdrt |
| `Xform_fabric` | `isaacsim.core.experimental.prims.XformPrim` | fabric |
| `Rigid既定` | `isaacsim.core.experimental.prims.RigidPrim` | 指定なし（Play 中と Pause 中は tensor、Stop 中は usd にフォールバック） |

## 確かめるノート

- [バックエンド選択の仕組み（「Play 中に XformPrim が読む値と、物理の出力先」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | Play（Stop・Pause でも動くが、値は変わらない） |
| 使うバックエンド | usd / usdrt / fabric / tensor |
| 書き込みが届く先 | なし（読むだけ。`OUTPUT_MODE` を None 以外にした場合だけ、設定を書き換える） |
| 他のバックエンドから見えるタイミング | 物理の出力先で決まる（下の「結果の表」） |

## 物理の出力先の設定と、その変え方

物理の出力先は、次の設定で決まる。GUI の既定は「USD へ書く」である（env_fingerprint で確かめた）。

| 出力先 | 設定の値 | 切り替えるコード |
|---|---|---|
| USD へ書く（既定） | `/physics/updateToUsd` など4つが true、omni.physx.fabric 拡張は無効 | `isaacsim.core.simulation_manager.SimulationManager.enable_fabric(False)` |
| Fabric へ書く | 4つが false、omni.physx.fabric 拡張は有効 | `isaacsim.core.simulation_manager.SimulationManager.enable_fabric(True)` |
| どちらにも書かない | 4つが false、omni.physx.fabric 拡張は無効 | `enable_fabric(False)` の後に、4つの設定を false にする |

4つの設定は `/physics/updateToUsd`（姿勢）、`/physics/updateVelocitiesToUsd`（速度）、`/physics/updateParticlesToUsd`（パーティクル）、`/physics/updateForceSensorsToUsd`（力覚センサー）である。`isaacsim.core.simulation_manager.SimulationManager.enable_fabric()` は、omni.physx.fabric 拡張の有効・無効と、この4つの設定を、まとめて逆向きに切り替える（ソース）。

Snippet の `OUTPUT_MODE` を `"usd"`・`"fabric"`・`"none"` のどれかにして実行すると、上の表のとおりに設定を切り替えてから観測を始める。None のままなら設定を変えない。切り替えた設定は、Isaac Sim を再起動すると元に戻る。Script Editor で設定だけを切り替えたい場合は、次のコードを使う。

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# 物理の出力先を切り替える（必要な行だけを実行する）
import carb.settings  # Kit の設定を読み書きするモジュール
from isaacsim.core.simulation_manager import SimulationManager  # 物理の出力先を切り替えるクラス

SimulationManager.enable_fabric(False)  # USD へ書く（既定に戻す）
SimulationManager.enable_fabric(True)  # Fabric へ書き、USD へは書かない
for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateForceSensorsToUsd"):  # USD へ書き戻す4つの設定
    carb.settings.get_settings().set_bool(key, False)  # enable_fabric(False) の後にこれを実行すると「どちらにも書かない」になる
```

## コード

[snippets/pose_freshness_probe.py](../snippets/pose_freshness_probe.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/pose_freshness_probe.py（全文）
```python
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
```

## 観測手順と期待される出力

準備：`/World/Cube` に Rigid Body と Collider を付けた Cube を、空中（z が 1 m 程度）に置く。次のコードで作れる。

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# 落下させる剛体の Cube を /World/Cube に作る
import omni.usd  # 開いているステージを取得するために使う
from pxr import Gf, UsdGeom, UsdPhysics  # USD のプリムと物理の API

stage = omni.usd.get_context().get_stage()  # 開いているステージを取得する
cube = UsdGeom.Cube.Define(stage, "/World/Cube")  # Cube を作る
cube.CreateSizeAttr(0.1)  # 一辺 0.1 m にする
UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0.0, 0.0, 1.0))  # 高さ 1 m に置く
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())  # 剛体にする
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())  # コライダーを付ける
```

ここでの「pose_freshness_probe を実行する」は、上の「コード」の節の pose_freshness_probe.py（全文）を Script Editor に貼って実行することを指す。
実行すると、描画フレームごとに10行が出て止まる。

1. Stop 中に、`OUTPUT_MODE` を `"usd"` にして pose_freshness_probe を実行する。最初の行が `[SETTINGS] /physics/updateToUsd=True omni.physx.fabric=False ...` になることを確かめる。
2. Play して、もう一度 pose_freshness_probe を実行する（`OUTPUT_MODE` は None に戻してよい）。各列が落下中に変わるかを見る。
3. Pause して実行し、すべての列が止まることを見る。
4. Stop して、`OUTPUT_MODE` を `"none"`、`"fabric"` に変えて、1〜3をくり返す。

### 結果の表

2026-11-09 にヘッドレス（GUI と同じ `isaacsim.exp.full.kit`）で、3通りの出力先 × Stop / Play / Pause を実測した結果である。「変わる」は、Play 中に物理の最新値と同じ値が更新ごとに出たことを表す。

| 物理の出力先      | `Xform既定`（usd）         | `Xform_usdrt` | `Xform_fabric` | `Rigid既定`（tensor） |
| ----------- | ---------------------- | ------------- | -------------- | ----------------- |
| USD へ書く（既定） | 変わる                    | 変わる           | 変わる            | 変わる               |
| Fabric へ書く  | 変わらない（Play 前の 1.0 のまま） | 変わる           | 変わる            | 変わる               |
| どちらにも書かない   | 変わらない（Play 前の 1.0 のまま） | 変わらない         | 変わらない          | 変わる               |

- Pause 中は、どの出力先でも、すべての列が Pause した時点の値で止まった。`Rigid既定` は Pause 中も tensor で読めた。
- Stop 中は、どの出力先でも、すべての列が Play 前の値（1.0）になった。Stop すると物理の状態が Play 前に戻り（`/persistent/physics/resetOnStop = True`）、**`Rigid既定` は tensor が使えないので usd にフォールバックする。**
- usd・usdrt・fabric は、Stop・Play・Pause のどの状態でも例外にならなかった。`ERR(...)` が出たのは、観測の途中でステージを開き直して、ラッパーが受け持つプリムがなくなったときだけだった（`ERR(AssertionError)`）。
- ステージを作った直後の最初の1フレームだけ、`Xform_usdrt` が 0.0000 になった。次のフレームからは正しい値になった。usdrt は Fabric の階層が更新されるまで値を持たないためである。

### 手順をコードで自動実行する

Play / Pause / Stop のボタンは、`omni.timeline` のメソッドで代わりに押せる。`timeline.play()` などは呼んだ時点では状態を変えず、次のメインループの更新で反映される。続けて結果を見るには、`async` の関数の中で `await omni.kit.app.get_app().next_update_async()` を使って更新を待つ。

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# 3通りの出力先それぞれで、Play → 観測 → Pause → 観測 → Stop を行う
import asyncio  # async の関数を Kit のイベントループで走らせるために使う

import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う

SNIPPET_PATH = r"C:\isaacsim-6.2.0\CodeByClaude\isaac-sim-backend-lab\snippets\pose_freshness_probe.py"  # pose_freshness_probe.py の場所


def run_probe(output_mode: str | None) -> None:
    """OUTPUT_MODE を書き換えて pose_freshness_probe.py を実行する。"""
    source = open(SNIPPET_PATH, encoding="utf-8").read()  # Snippet のソースを読む
    source = source.replace("OUTPUT_MODE: OutputMode = None", f"OUTPUT_MODE: OutputMode = {output_mode!r}")  # 出力先を指定する
    exec(source, {"__name__": "__main__"})  # Script Editor で実行したのと同じように実行する


async def run_all() -> None:
    """出力先ごとに、Play 中と Pause 中の値を表示させる。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    for output_mode in ("usd", "none", "fabric"):  # 3通りの出力先
        run_probe(output_mode)  # Stop 中に出力先を切り替えて観測を始める
        timeline.play(
        )  # Play（次の更新で反映される）
        for _ in range(20):  # 剛体が落ち始めるまで待つ
            await app.next_update_async()  # 更新を1回待つ
        run_probe(None)  # Play 中の値を10フレーム分表示させる
        for _ in range(15):  # 表示が終わるまで待つ
            await app.next_update_async()  # 更新を1回待つ
        timeline.pause()  # Pause
        await app.next_update_async()  # Pause が反映されるまで1回待つ
        run_probe(None)  # Pause 中の値を表示させる
        for _ in range(15):  # 表示が終わるまで待つ
            await app.next_update_async()  # 更新を1回待つ
        timeline.stop()  # Stop
        for _ in range(3):  # Stop が反映されるまで待つ
            await app.next_update_async()  # 更新を1回待つ
    run_probe("usd")  # 最後に既定の出力先に戻す


asyncio.ensure_future(run_all())  # Kit のイベントループで run_all を走らせる（この行はすぐに戻る）
```

## 期待と違ったときに疑う箇所

- `Xform_usdrt` と `Xform_fabric` が `ERR(RuntimeError)` になる場合、Fabric Scene Delegate が無効である（`/app/useFabricSceneDelegate`）。この設定は、`isaacsim.core.experimental.utils.backend` モジュールが読み込まれたときに1回だけ読まれるので、起動後に変えても結果は変わらない（ソースと実測）。
  起動時のオプション `--/app/useFabricSceneDelegate=true` か kit ファイルで設定する。
- `Rigid既定` が Play 中に変わらない場合、tensor の子の view が無効で usd にフォールバックしている。[backend_spy](06_backend_spy.md) で確かめる。
- 全部の列が `ERR(AssertionError)` になる場合、ラッパーを作った後にステージを開き直している。pose_freshness_probe を実行し直して、ラッパーを作り直す。

## 結果

2026-11-09 に GUI で実行した結果である（設定の表示と `OUTPUT_MODE` を加える前のコードで実行したので、`[SETTINGS]` 行の代わりに env_fingerprint の出力を載せている）。下の「考察」に、ソースとヘッドレスでの追加の実測で分かったことを書いた。

```
# env_fingerprint
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
timeline current time (tau): 3.516666666666659
simulation_time (t): 3.55
num_physics_steps (n): 213
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
tensor simulation view: <omni.physics.tensors.api.SimulationView object at 0x000001E85327E480>

# PlayOFF
pose probe registered
Module isaacsim.core.experimental.prims.impl._fabric d50a5e6 load on device 'cpu' took 1.29 ms  (cached)
frame= 0 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 1 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 2 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 3 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 4 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 5 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 6 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 7 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 8 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000
frame= 9 steps=   0 Xform既定=0.0000 Xform_usdrt=0.0000 Xform_fabric=0.0000 Rigid既定=0.0000

# PlayON
pose probe registered
frame= 0 steps=  69 Xform既定=-6.5809 Xform_usdrt=-6.5809 Xform_fabric=-6.5809 Rigid既定=-6.5809
frame= 1 steps=  69 Xform既定=-6.5809 Xform_usdrt=-6.5809 Xform_fabric=-6.5809 Rigid既定=-6.5809
frame= 2 steps=  71 Xform既定=-6.9651 Xform_usdrt=-6.9651 Xform_fabric=-6.9651 Rigid既定=-6.9651
frame= 3 steps=  72 Xform既定=-7.1613 Xform_usdrt=-7.1613 Xform_fabric=-7.1613 Rigid既定=-7.1613
frame= 4 steps=  73 Xform既定=-7.3602 Xform_usdrt=-7.3602 Xform_fabric=-7.3602 Rigid既定=-7.3602
frame= 5 steps=  74 Xform既定=-7.5619 Xform_usdrt=-7.5619 Xform_fabric=-7.5619 Rigid既定=-7.5619
frame= 6 steps=  75 Xform既定=-7.7662 Xform_usdrt=-7.7662 Xform_fabric=-7.7662 Rigid既定=-7.7662
frame= 7 steps=  76 Xform既定=-7.9733 Xform_usdrt=-7.9733 Xform_fabric=-7.9733 Rigid既定=-7.9733
frame= 8 steps=  77 Xform既定=-8.1832 Xform_usdrt=-8.1832 Xform_fabric=-8.1832 Rigid既定=-8.1832
frame= 9 steps=  78 Xform既定=-8.3957 Xform_usdrt=-8.3957 Xform_fabric=-8.3957 Rigid既定=-8.3957

# Pause
pose probe registered
frame= 0 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 1 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 2 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 3 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 4 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 5 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 6 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 7 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 8 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
frame= 9 steps= 213 Xform既定=-62.1054 Xform_usdrt=-62.1054 Xform_fabric=-62.1054 Rigid既定=-62.1054
```

### 考察

1. **既定の構成では、4つの経路がすべて物理の最新値を返す。** 上の結果では、`omni.physx.fabric: enabled=False`、`/physics/updateToUsd = True` の構成で、Play 中の4つの列がすべて同じ値で変わった。物理の計算結果がまず USD に書かれ（`updateToUsd`）、USD に書かれた値は Fabric にも写されるので、usdrt / fabric で読んでも最新の値になる。
2. **USD へ書かない構成では、XformPrim の既定は古い値を返す。** ヘッドレスで出力先を変えた結果（上の「結果の表」）のとおり、USD に書かない構成（Fabric へ書く、どちらにも書かない）では、`Xform既定`（usd）は Play 前の値のまま変わらなかった。`isaacsim.core.experimental.prims.XformPrim` の姿勢のメソッドは、Play 中でも、何も指定しなければ usd を読むからである。物理で動く剛体の姿勢は、`isaacsim.core.experimental.prims.RigidPrim`（Play 中は tensor）で読めば、出力先に関係なく最新の値になる。
3. **Fabric へ書く構成では、usdrt / fabric は最新、usd は古い。** Isaac Lab の既定と同じ構成である。USD への書き戻しをしないぶん、更新が速い。1000個の剛体を落とした実測では、1回の更新が USD へ書く構成で 66 ms、Fabric へ書く構成で 8.4 ms、どちらにも書かない構成で 6.2 ms だった（[重い処理と軽くする方法](08_重い処理と軽くする方法.md)）。
4. **最初の2フレームで `steps` が同じ（69, 69）になる理由。** Snippet を実行した直後の更新で、描画フレームのイベントが2回続けて届いたためである。描画フレームのイベントは、平均すると更新1回につき1回だが、1回の更新だけを見ると 0 回や 2 回のことがある（[clock_probe](03_clock_probe.md) の考察4）。
5. **Pause 中にすべての列が同じ値で止まる。** Pause 中は物理ステップが実行されないので、どの経路も最後の物理ステップの結果を返す。上の結果のとおりである。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [bench_probe](04_bench_probe.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [backend_spy](06_backend_spy.md) |
