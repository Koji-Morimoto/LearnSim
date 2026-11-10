---
type: snippet-guide
created: 2026-11-08
updated: 2026-11-10
tags: [isaac-sim-6.2.0, snippet, backend, fallback]
---

# backend_spy

ファイル：[snippets/backend_spy.py](../snippets/backend_spy.py)

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| simulation view | ステージと物理エンジンの組ごとに1つ作られる view の入口。Play で作られ、Stop で無効になる |
| 子の view | simulation view から、プリムのパスを指定して作る view。ラッパーは tensor で読み書きするためにこれを内部に持つ |
| 候補リスト | メソッドが `isaacsim.core.experimental.utils.backend.get_current_backend()` に渡す、対応するバックエンドを優先順に並べたリスト |
| フォールバック | 選ばれたバックエンドが使えないために、別のバックエンドに差し替わること |
| スパイ | 元の関数を包み替えて、呼ばれるたびに中身を表示させる観測用のコード |
| tensor 専用のメソッド | 候補リストを持たず、tensor でしか処理しないメソッド。`isaacsim.core.experimental.prims.Articulation.get_dof_positions()` など |

ほかの用語の定義は [用語集](../01_用語と概念/01_用語集.md) にまとめている。

## 目的

Core Experimental API が、呼び出しごとにどのバックエンドを選んだかと、フォールバックが起きたかを表示する。何も指定していない既定の状態では、フォールバックは何の通知もなく起きるため、このスパイで目に見えるようにする。

## 仕組み

- `isaacsim.core.experimental.utils.backend.get_current_backend()` を差し替え、候補リスト・指定値・選ばれたバックエンドを `[BACKEND]` 行として表示する。
- `isaacsim.core.experimental.prims.Articulation._check_for_tensor_backend` と `isaacsim.core.experimental.prims.RigidPrim._check_for_tensor_backend` を差し替え、tensor から usd に差し替わったときに `[FALLBACK]` 行を表示する。
- 差し替えが成り立つ理由は [バックエンド選択の仕組み（「フォールバックの検知方法」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md) にまとめた。
- `isaacsim.core.experimental.utils.impl.backend._context` と `isaacsim.core.experimental.prims.Articulation._check_for_tensor_backend()` は、アンダースコアで始まる内部の名前である。Isaac Sim のバージョンが変わると、このスパイは動かなくなる可能性がある。

## スパイに映らないメソッド

次の2種類のメソッドは `isaacsim.core.experimental.utils.backend.get_current_backend()` を呼ばないので、スパイは `[BACKEND]` 行を出さない。

| 種類 | 例 | Stop 中に呼んだとき |
|---|---|---|
| tensor 専用のメソッド | `isaacsim.core.experimental.prims.Articulation` の `get_dof_positions`・`get_dof_velocities`・`set_dof_positions` など26個、`isaacsim.core.experimental.prims.RigidPrim` の `apply_forces`・`get_coms` など7個（6.2.0 のソースで数えた） | usd にフォールバックせず、`AssertionError: Instance's physics tensor entity is not valid. Play the simulation/timeline to re-initialize it` になる（実測） |
| プロパティ | `isaacsim.core.experimental.prims.Articulation.num_dofs`、`dof_names` など | バックエンドの選択をしない |

## メタ情報

| 項目 | 内容 |
|---|---|
| 有効な状態 | Stop / Play / Pause のすべて |
| 使うバックエンド | 観測対象の API が使うもの |
| 書き込みが届く先 | なし（表示だけ） |
| 他のバックエンドから見えるタイミング | 該当なし |

## コード

[snippets/backend_spy.py](../snippets/backend_spy.py) と同じ内容である。Script Editor にそのまま貼って実行できる。

snippets/backend_spy.py（全文）
```python
# backend_spy.py — Core Experimental APIが呼び出しごとに選んだバックエンドと、フォールバックの発生を表示する観測用コード
# 解説: 05_観測Snippet/06_backend_spy.md
"""Core Experimental API が呼び出しごとに選んだバックエンドと、フォールバックの発生を表示する。

Isaac Sim の Script Editor で実行する。SPY_ENABLED を False にして再実行すると、元の関数に戻す。

Note:
    呼べる状態: Stop／Play／Pause のどれでも実行できる。[FALLBACK] の表示が出るのは、
        tensor の子の view が無効な Stop 中に、候補リストの先頭の tensor が選ばれたときである（実測）。
        Pause 中は view が有効なままなので出ない。候補リストを持たない tensor 専用のメソッド
        （get_dof_positions など）は、Stop 中は [BACKEND] 行を出さずに AssertionError になる。
    backend: backend を選ばない。prim ラッパーが行う選択を横から表示するだけである。
"""

from __future__ import annotations

import builtins  # 元の関数を、Script Editorの再実行をまたいで保持するために使う
import sys  # 呼び出し元のメソッド名を調べるために使う
import typing

import isaacsim.core.experimental.utils.backend as backend_utils  # prims側が呼び出しに使う公開モジュール
import isaacsim.core.experimental.utils.impl.backend as backend_impl  # 指定値(_context)が実際に置かれている内部モジュール
from isaacsim.core.experimental.prims import Articulation, RigidPrim  # tensorの有効性チェックを持つクラス

# tensor の有効性チェックを差し替えるクラスの名前。これ以外の文字列はエディタが警告する
SpiedClassName: typing.TypeAlias = typing.Literal["Articulation", "RigidPrim"]
# tensor の有効性チェックの型。(self, backend, fallback_backend=...) を受け取り、使う backend 名を返す
CheckForTensorBackend: typing.TypeAlias = typing.Callable[..., str]


class OriginalFunctions(typing.TypedDict):
    """差し替える前の元の関数。辞書のキーと値の型をエディタに伝えるための定義。"""

    get_current_backend: typing.Callable[..., str]  # 候補から1つ選ぶ関数
    Articulation: CheckForTensorBackend  # Articulationのtensor有効性チェック
    RigidPrim: CheckForTensorBackend  # RigidPrimのtensor有効性チェック


SPY_ENABLED: typing.Final[bool] = True  # Trueでスパイを有効にする。Falseにして再実行すると元に戻す

if not hasattr(builtins, "_backend_spy_orig"):  # 初回だけ元の関数を保存する（包みが二重にならないようにするため）
    builtins._backend_spy_orig = {  # 元の関数の保存先
        "get_current_backend": backend_utils.get_current_backend,  # 候補から1つ選ぶ関数
        "Articulation": Articulation._check_for_tensor_backend,  # Articulationのtensor有効性チェック
        "RigidPrim": RigidPrim._check_for_tensor_backend,  # RigidPrimのtensor有効性チェック
    }
orig: OriginalFunctions = builtins._backend_spy_orig  # 保存しておいた元の関数を短い名前で参照する


def spy_get_current_backend(supported_backends: list[str], **kwargs: bool | None) -> str:
    """get_current_backend の代わりに呼ばれ、選択の結果を表示してから元の関数と同じ値を返す。

    Args:
        supported_backends: 呼び出し元のメソッドが対応する backend の候補。先頭ほど優先される。
        **kwargs: 元の関数へそのまま渡す引数（raise_on_unsupported）。

    Returns:
        元の get_current_backend が選んだ backend の名前。
    """
    result = orig["get_current_backend"](supported_backends, **kwargs)  # 実際の選択は元の関数に任せる
    caller = sys._getframe(1).f_code.co_name  # この関数を呼んだメソッドの名前を取る
    # use_backend で指定された値は内部モジュールのスレッドローカル変数にある。型は公開されていないので object とする
    requested: object = getattr(backend_impl._context, "backend", None)  # use_backendで指定された値を読む（未指定ならNone）
    print(f"[BACKEND] {caller}: 候補={supported_backends} 指定={requested} → 選択={result}")  # 選択の結果を表示する
    return result  # 元の関数と同じ値を返す


def make_spy_check(class_name: SpiedClassName) -> CheckForTensorBackend:
    """指定したクラスについて、tensor 有効性チェックの包みを作る。

    Args:
        class_name: 包みを作るクラスの名前。

    Returns:
        元のチェックを呼び、判定の前後で backend が変わったらフォールバックとして表示する関数。
    """
    original = orig[class_name]  # そのクラスの元のチェック関数を取り出す

    def spy_check(self: Articulation | RigidPrim, backend: str, **kwargs: str) -> str:
        """元のチェックの代わりに呼ばれ、フォールバックが起きたら表示する。

        Args:
            self: チェックを呼んだラッパーのインスタンス。
            backend: 呼び出し元のメソッドが選んだ backend の名前。
            **kwargs: 元のチェックへそのまま渡す引数（fallback_backend）。

        Returns:
            元のチェックが返した backend の名前。
        """
        result = original(self, backend, **kwargs)  # 判定は元のチェックに任せる（例外もそのまま呼び出し元へ伝わる）
        if result != backend:  # 判定の前後でバックエンドが変わっていれば、フォールバックが起きている
            caller = sys._getframe(1).f_code.co_name  # 呼び出し元のメソッド名を取る
            print(f"[FALLBACK] {class_name}.{caller}: {backend} → {result}（tensorの子のviewが無効）")  # フォールバックを表示する
        return result  # 元のチェックと同じ値を返す

    return spy_check  # 作った包みを返す


if SPY_ENABLED:  # スパイを有効にする場合
    backend_utils.get_current_backend = spy_get_current_backend  # 公開モジュールの関数を差し替える
    Articulation._check_for_tensor_backend = make_spy_check("Articulation")  # Articulationのチェックを差し替える
    RigidPrim._check_for_tensor_backend = make_spy_check("RigidPrim")  # RigidPrimのチェックを差し替える
    print("backend spy: ON")  # 有効になったことを表示する
else:  # スパイを無効にする場合
    backend_utils.get_current_backend = orig["get_current_backend"]  # 元の関数に戻す
    Articulation._check_for_tensor_backend = orig["Articulation"]  # 元のチェックに戻す
    RigidPrim._check_for_tensor_backend = orig["RigidPrim"]  # 元のチェックに戻す
    print("backend spy: OFF")  # 無効になったことを表示する
```

## 観測手順と期待される出力

### 準備

`/World/robot` に Articulation、`/World/Cube` に剛体の Cube を置く。自分のロボットを使う場合は、下の観測用コードの `ROBOT_PATH` を書き換える。試すだけなら、次のコードで「土台を世界に固定し、1つの回転関節で棒をつないだ」最小の Articulation と Cube を作れる。

Script Editor に貼って実行するコード（ファイルではない。Stop 中に実行する）
```python
# 最小の Articulation（/World/robot）と剛体の Cube（/World/Cube）を作る
import omni.usd  # 開いているステージを取得するために使う
from pxr import Gf, UsdGeom, UsdPhysics  # USD のプリムと物理の API

stage = omni.usd.get_context().get_stage()  # 開いているステージを取得する


def add_rigid_cube(path: str, position: tuple[float, float, float]) -> UsdGeom.Cube:
    """剛体とコライダーを付けた一辺 0.1 m の Cube を path に置く。"""
    cube = UsdGeom.Cube.Define(stage, path)  # Cube を作る
    cube.CreateSizeAttr(0.1)  # 一辺 0.1 m にする
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*position))  # 位置を決める
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())  # 剛体にする
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())  # コライダーを付ける
    return cube  # 作った Cube を返す


add_rigid_cube("/World/Cube", (0.0, 0.0, 1.0))  # 落下させる Cube
UsdGeom.Xform.Define(stage, "/World/robot")  # Articulation のルート
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/World/robot"))  # ルートに Articulation の印を付ける
base = add_rigid_cube("/World/robot/base", (2.0, 0.0, 1.0))  # 土台
link = add_rigid_cube("/World/robot/link", (2.0, 0.0, 1.3))  # 関節でつなぐ棒
fixed = UsdPhysics.FixedJoint.Define(stage, "/World/robot/fixed_joint")  # 土台を世界に固定する関節
fixed.CreateBody1Rel().SetTargets([base.GetPath()])  # body0 を空にすると世界につながる
fixed.CreateLocalPos0Attr(Gf.Vec3f(2.0, 0.0, 1.0))  # 世界側の固定位置
revolute = UsdPhysics.RevoluteJoint.Define(stage, "/World/robot/revolute_joint")  # 回転関節
revolute.CreateBody0Rel().SetTargets([base.GetPath()])  # 土台側
revolute.CreateBody1Rel().SetTargets([link.GetPath()])  # 棒側
revolute.CreateAxisAttr("Y")  # Y 軸まわりに回る
revolute.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0.15))  # 土台から見た関節の位置
revolute.CreateLocalPos1Attr(Gf.Vec3f(0, 0, -0.15))  # 棒から見た関節の位置
drive = UsdPhysics.DriveAPI.Apply(revolute.GetPrim(), "angular")  # 角度のドライブを付ける（Stop 中に DOF の種類を決めるのに使われる）
drive.CreateStiffnessAttr(1000.0)  # 目標角へ引き戻す強さ
drive.CreateDampingAttr(100.0)  # 速度を抑える強さ
```

関節に角度のドライブ（`drive:angular:...` の属性）を付けておくのは、Stop 中の Articulation が、DOF の種類（回転か直動か）を USD のドライブの属性から決めるからである。ドライブがないと DOF の種類が Invalid になり、Stop 中に `get_dof_position_targets` などを呼ぶと警告が出る（下の「自分の出力との違い」）。

### 手順

1. Stop 中に、上の「コード」の節の backend_spy.py（全文）を Script Editor で実行する。`backend spy: ON` と表示され、スパイが有効になる。
2. 別のタブで、次の観測用コードを実行する。

Script Editor に貼って実行するコード（ファイルではない。手順1でスパイを有効にした後に実行する）
```python
# スパイを有効にした状態で、Articulation と RigidPrim のメソッドを1回ずつ呼ぶ
from __future__ import annotations  # 型ヒントを実行時に評価させない

import typing

from isaacsim.core.experimental.prims import Articulation, RigidPrim  # 観測する2種類のラッパー

ROBOT_PATH: typing.Final[str] = "/World/robot"  # Articulation のルートのパス（自分のシーンに合わせて書き換える）
CUBE_PATH: typing.Final[str] = "/World/Cube"  # 剛体のパス（自分のシーンに合わせて書き換える）


def try_print(label: str, action: typing.Callable[[], object]) -> None:
    """action を呼び、結果か例外を1行で表示する。"""
    try:  # 例外が出ても続けるために受け止める
        print(f"{label}: {action()}")  # 結果を表示する
    except Exception as error:  # 例外が出た場合は
        print(f"{label}: ERR {type(error).__name__}: {error}")  # 例外の種類とメッセージを表示する


robot = Articulation(ROBOT_PATH)  # Articulation のラッパーを作る
cube = RigidPrim(CUBE_PATH)  # 剛体のラッパーを作る
try_print("get_dof_positions", lambda: robot.get_dof_positions().numpy().tolist())  # tensor 専用のメソッド
try_print("get_dof_position_targets", lambda: robot.get_dof_position_targets().numpy().tolist())  # 候補リスト ['tensor', 'usd'] のメソッド
try_print("RigidPrim.get_world_poses z", lambda: cube.get_world_poses()[0].numpy()[0][2].item())  # 候補リスト ['tensor', 'usd', 'usdrt', 'fabric'] のメソッド
```

3. Play してから、手順2の観測用コードをもう一度実行する。
4. Pause してから、もう一度実行する。
5. 観測が終わったら、backend_spy.py の `SPY_ENABLED` を False にして再実行し、元に戻す。

### 期待される出力（2026-11-10 にヘッドレスで、Script Editor と同じ実行経路で実行した出力）

手順2（Stop 中）
```text
[BACKEND] fetch_articulation_root_api_prim_paths: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
get_dof_positions: ERR AssertionError: Instance's physics tensor entity is not valid. Play the simulation/timeline to re-initialize it

[BACKEND] get_dof_position_targets: 候補=['tensor', 'usd'] 指定=None → 選択=tensor
[FALLBACK] Articulation.get_dof_position_targets: tensor → usd（tensorの子のviewが無効）
get_dof_position_targets: [[0.0]]

[BACKEND] get_world_poses: 候補=['tensor', 'usd', 'usdrt', 'fabric'] 指定=None → 選択=tensor
[FALLBACK] RigidPrim.get_world_poses: tensor → usd（tensorの子のviewが無効）
[BACKEND] get_world_poses: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
RigidPrim.get_world_poses z: 1.0
```

- 1行目は、`Articulation(ROBOT_PATH)` でラッパーを作ったときの行である。`isaacsim.core.experimental.prims.Articulation` は、作るときに `fetch_articulation_root_api_prim_paths` を呼んで、指定したパスの下から ArticulationRootAPI の付いたプリムを探す。この静的メソッドの候補リストは usd → usdrt → fabric なので、usd が選ばれる。
- `get_dof_positions` は tensor 専用のメソッドなので、`[BACKEND]` 行が出ず、usd にフォールバックせずに例外になる。
- `get_dof_position_targets` は、候補の先頭の tensor が選ばれた後、子の view が無効なので usd に差し替わる。
- `RigidPrim.get_world_poses` は、usd に差し替わった後、`isaacsim.core.experimental.prims.XformPrim` の実装に処理を渡し、そこでもう一度 usd → usdrt → fabric の候補から選び直す。そのため `[BACKEND]` 行が2行出る。

手順3（Play 中）と手順4（Pause 中）
```text
[BACKEND] fetch_articulation_root_api_prim_paths: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
get_dof_positions: [[0.0]]

[BACKEND] get_dof_position_targets: 候補=['tensor', 'usd'] 指定=None → 選択=tensor
get_dof_position_targets: [[0.0]]

[BACKEND] get_world_poses: 候補=['tensor', 'usd', 'usdrt', 'fabric'] 指定=None → 選択=tensor
RigidPrim.get_world_poses z: 0.9591249227523804
```

Play 中も Pause 中も `[FALLBACK]` は出ない。Pause では view が無効にならないからである。Stop 中に作ったラッパーをそのまま Play 中・Pause 中に使っても、新しく作ったラッパーと同じ出力になった。ラッパーは、Play で物理の準備ができたとき（PHYSICS_READY のイベント）に子の view を作り直すからである。

### 自分の出力

手順2(Stop中)
```
[BACKEND] fetch_articulation_root_api_prim_paths: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
get_dof_positions: ERR AssertionError: Instance's physics tensor entity is not valid. Play the simulation/timeline to re-initialize it
[BACKEND] get_dof_position_targets: 候補=['tensor', 'usd'] 指定=None → 選択=tensor
[FALLBACK] Articulation.get_dof_position_targets: tensor → usd（tensorの子のviewが無効）
2026-11-10T02:10:22Z [52,847,400ms] [Warning] [isaacsim.core.experimental.prims.impl.articulation] Invalid DOF type (DofType.Invalid) at index 0
get_dof_position_targets: [[0.0]]
[BACKEND] get_world_poses: 候補=['tensor', 'usd', 'usdrt', 'fabric'] 指定=None → 選択=tensor
[FALLBACK] RigidPrim.get_world_poses: tensor → usd（tensorの子のviewが無効）
[BACKEND] get_world_poses: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
RigidPrim.get_world_poses z: 0.0
```

手順3（Play 中）と手順4（Pause 中）
```
[BACKEND] fetch_articulation_root_api_prim_paths: 候補=['usd', 'usdrt', 'fabric'] 指定=None → 選択=usd
get_dof_positions: [[0.0]]
[BACKEND] get_dof_position_targets: 候補=['tensor', 'usd'] 指定=None → 選択=tensor
get_dof_position_targets: [[0.0]]
[BACKEND] get_world_poses: 候補=['tensor', 'usd', 'usdrt', 'fabric'] 指定=None → 選択=tensor
RigidPrim.get_world_poses z: -2182.02197265625
```

### 自分の出力との違い

上の「自分の出力」（GUI で実行した出力）と期待される出力を比べると、次の3点で違って見える。どれもスパイや手順の誤りではない。

1. **`fetch_articulation_root_api_prim_paths` の行。** 観測用コードの `Articulation(ROBOT_PATH)` でラッパーを作るときに出る行である。期待される出力の1行目と同じである。
2. **`Invalid DOF type (DofType.Invalid) at index 0` の警告。** 準備のコードで作った関節にドライブが付いていなかったために出た警告である。Stop 中は tensor が使えないので、`isaacsim.core.experimental.prims.Articulation` は DOF の情報を物理エンジンの性質の問い合わせ（`omni.physx` の property query）と USD から作る。このとき DOF の種類は、関節のプリムに `drive:angular:` か `drive:linear:` で始まる属性があるかで決まり、どちらもなければ Invalid になる（`articulation.py` の `_query_articulation_properties`）。`get_dof_position_targets` の usd の経路は、DOF ごとに `_get_drive_api_and_type` を呼び、種類が Invalid だとこの警告を出して、`drive:invalid:...` という名前のドライブの API を関節に付けてから値を読む（ステージに属性が増える）。Play 中は DOF の種類を tensor から読むので、警告は出ない。準備のコードは、角度のドライブを付けるように直した。
3. **`RigidPrim.get_world_poses z` の値。** Stop 中の値は、USD に書かれている `/World/Cube` の位置そのものである（tensor が使えないので usd で読む）。期待される出力の 1.0 は準備のコードで置いた高さで、自分の出力の 0.0 は、GUI のシーンの `/World/Cube` が高さ 0 にあったことを表す。Play 中の値（−2182）は、床のないシーンで Cube が落ち続けた結果で、Play を押してからの時間で決まる（自由落下では z = −4.9 t²。−2182 m は約 21 秒にあたる）。どちらも、物理エンジンの最新の値を tensor で読めていることを示している。

### 手順をコードで自動実行する

Play / Pause / Stop のボタンは、`omni.timeline` のメソッドで代わりに押せる。`timeline.play()` などは呼んだ時点では状態を変えず、次のメインループの更新で反映される。続けて結果を見るには、`async` の関数の中で `await omni.kit.app.get_app().next_update_async()` を使って更新を待つ。

Script Editor に貼って実行するコード（ファイルではない。手順1でスパイを有効にし、手順2の観測用コードを1回実行して try_print・robot・cube を作った後に実行する）
```python
# Stop → Play → Pause → Stop の各状態で、手順2と同じ3つの呼び出しを行う
import asyncio  # async の関数を Kit のイベントループで走らせるために使う

import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う


def observe(state_label: str) -> None:
    """今の状態で3つのメソッドを呼び、スパイの表示を出させる。"""
    print(f"===== {state_label} =====")  # どの状態の出力かを示す見出し
    try_print("get_dof_positions", lambda: robot.get_dof_positions().numpy().tolist())  # tensor 専用のメソッド
    try_print("get_dof_position_targets", lambda: robot.get_dof_position_targets().numpy().tolist())  # 候補リスト ['tensor', 'usd']
    try_print("RigidPrim.get_world_poses z", lambda: cube.get_world_poses()[0].numpy()[0][2].item())  # 候補リスト ['tensor', 'usd', 'usdrt', 'fabric']


async def run_all() -> None:
    """状態を切り替えながら observe を呼ぶ。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    observe("Stop")  # Stop 中
    timeline.play()  # Play（次の更新で反映される）
    for _ in range(5):  # 物理の準備ができるまで待つ
        await app.next_update_async()  # 更新を1回待つ
    observe("Play")  # Play 中
    timeline.pause()  # Pause
    await app.next_update_async()  # Pause が反映されるまで1回待つ
    observe("Pause")  # Pause 中
    timeline.stop()  # Stop
    await app.next_update_async()  # Stop が反映されるまで1回待つ
    observe("Stop（Play の後）")  # Stop 中


asyncio.ensure_future(run_all())  # Kit のイベントループで run_all を走らせる（この行はすぐに戻る）
```

## 期待と違ったときに疑う箇所

- `[BACKEND]` 行が1行も出ない場合、そのメソッドは `isaacsim.core.experimental.utils.backend.get_current_backend()` を経由していない（上の「スパイに映らないメソッド」）。
- Play 中も `[FALLBACK]` が出る場合、ラッパーの子の view が作られていない。[env_fingerprint](01_env_fingerprint.md) で simulation view の有無を確かめ、Play 中にラッパーを作り直す。

## 関連ノート

- [MOC-観測Snippet](00_MOC-観測Snippet.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [pose_freshness_probe](05_pose_freshness_probe.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [strict_backend](07_strict_backend.md) |
