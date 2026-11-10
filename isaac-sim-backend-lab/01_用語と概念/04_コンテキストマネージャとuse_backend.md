---
type: note
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, python, context-manager, backend]
source:
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.utils\isaacsim\core\experimental\utils\impl\backend.py
---

# コンテキストマネージャと isaacsim.core.experimental.utils.backend.use_backend

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| 候補リスト | メソッドが `isaacsim.core.experimental.utils.backend.get_current_backend()` に渡す、対応するバックエンドを優先順に並べたリスト |

ほかの用語の定義は [用語集](01_用語集.md) にまとめている。

## 一言要約

コンテキストマネージャは、Python の `with` 文に渡すオブジェクトである。`with` ブロックに入るときに状態を設定し、ブロックを抜けるときに元の状態へ戻す。例外でブロックを抜けた場合も、元の状態へ戻す。

## isaacsim.core.experimental.utils.backend.use_backend の動作（手元のソースで確認した）

1. ブロックに入るとき、`isaacsim.core.experimental.utils.backend.use_backend()` は現在の3つの値（バックエンド名、raise_on_unsupported、raise_on_fallback）を退避する。そのうえで、指定された値をスレッドローカル変数 `isaacsim.core.experimental.utils.impl.backend._context` に書き込む。
2. ブロックを抜けるとき、`isaacsim.core.experimental.utils.backend.use_backend()` は退避しておいた値を `isaacsim.core.experimental.utils.impl.backend._context` に書き戻す。

スレッドローカル変数とは、スレッドごとに別々の値を持つ変数である。

## 設定が効く範囲

- **範囲は「実行している間」で決まる。** ブロックの中から関数を呼び、その関数の奥で API が呼ばれても、設定は効く。
- **後で呼ばれるコールバックには効かない。** ブロックの中で登録した物理ステップやフレームのコールバックは、ブロックを抜けた後の更新で呼ばれるので、そのときには指定が外れている（2026-11-09 に実測：with の中で登録した物理ステップのコールバックの中で読むと、指定値は None で、候補 `['tensor', 'usd']` から tensor が選ばれた）。コールバックの中で効かせたい場合は、コールバックの中に `with` を書く。
- **ブロックの中で物理ステップを直接進めた場合は効く。** with の中で `isaacsim.core.simulation_manager.SimulationManager.step()` を呼ぶと、そのステップの PRE / POST のコールバックは、with の中で、同じスレッドから呼ばれる。そのため、コールバックの中でも指定が効いた（実測：指定値 usd、選ばれたのも usd）。
- **スレッドごとに独立している。** 別のスレッドで動く処理には効かない。
- **入れ子にできる。** 内側のブロックが優先され、内側を抜けると外側の設定に戻る。

## 具体例

Script Editor に貼って実行できるコード例（ファイルではない。`/World/Cube` というプリムがあるステージで実行する）
```python
# with ブロックの範囲を示す例
from __future__ import annotations  # 型ヒントを実行時に評価させない（下の wp は実行時には import しないため）

import typing

import isaacsim.core.experimental.utils.backend as backend_utils  # use_backend を提供するモジュール
from isaacsim.core.experimental.prims import XformPrim  # 姿勢取得の候補リストが usd→usdrt→fabric のラッパー

if typing.TYPE_CHECKING:
    import warp as wp  # 型ヒントにだけ使う。実行時には import しない

xform: XformPrim = XformPrim("/World/Cube")  # 観測対象のプリムを受け持つラッパーを作る
poses: tuple[wp.array, wp.array]  # 読み取った姿勢（位置, 向き）。下の4回の読み取りで上書きする

with backend_utils.use_backend("usdrt"):  # ここから usdrt を指定する
    poses = xform.get_world_poses()  # usdrt で処理される
    with backend_utils.use_backend("usd"):  # 入れ子：内側だけ usd を指定する
        poses = xform.get_world_poses()  # usd で処理される
    poses = xform.get_world_poses()  # 内側を抜けたので usdrt に戻る
poses = xform.get_world_poses()  # 外側も抜けたので指定なし（候補リストの先頭）に戻る
```

## 境界

- `isaacsim.core.experimental.utils.backend.use_backend("usdrt")` と `isaacsim.core.experimental.utils.backend.use_backend("fabric")` は、Fabric Scene Delegate が無効な場合 RuntimeError になる。無効かどうかは、モジュールを読み込んだ時点の `/app/useFabricSceneDelegate` の値で判定される。
- 設定値の既定を変える設定項目（carb の設定など）は存在しない。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー
from isaacsim.core.experimental.prims import RigidPrim  # 剛体のラッパー
from isaacsim.core.experimental.prims import XformPrim  # Xform のラッパー
import isaacsim.core.experimental.utils.backend as backend_utils  # use_backend / get_current_backend などを持つモジュール
import isaacsim.core.experimental.utils.impl.backend as backend_impl  # _context（use_backend の指定値）が置かれた内部モジュール
```

## 関連ノート

- [バックエンド選択の仕組み（「バックエンド選択の3段階」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)
- [バックエンド選択の仕組み（「strict による仮説検証」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [view とは何か](03_viewとは何か.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [複数プリム対応と Python ループの有無](05_複数プリム対応とPythonループの有無.md) |
