---
type: note
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, performance, backend, multi-prim]
source:
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.prims\isaacsim\core\experimental\prims\impl\xform_prim.py
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.prims\isaacsim\core\experimental\prims\impl\rigid_prim.py
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.prims\isaacsim\core\experimental\prims\impl\articulation.py
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/docs/overview/experimental.html
---

# 複数プリム対応と Python ループの有無

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| 姿勢 | 位置と向きの組。スケールは含まない |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| simulation view | ステージと物理エンジンの組ごとに1つ作られる view の入口。Play で作られ、Stop で無効になる |
| 子の view | simulation view から、プリムのパスを指定して作る view。ラッパーは tensor で読み書きするためにこれを内部に持つ |

ほかの用語の定義は [用語集](01_用語集.md) にまとめている。

## 一言要約

複数プリム対応とは、1つのラッパーが、正規表現のパスで選んだ N 個のプリムをまとめて受け持ち、データを (N, …) という形の配列でやり取りする仕組みである。呼び出しが1回で済むのは見た目の上だけで、内部で Python のループが回るかどうかはバックエンドによって決まる。

## 旧API との違い

旧Core API で複数のプリムを受け持てたのは、`isaacsim.core.prims.Articulation`、`isaacsim.core.prims.RigidPrim`、`isaacsim.core.prims.XFormPrim` など一部のクラスだけだった。Core Experimental API では、形状・メッシュ・ライト・マテリアルを含め、すべてのラッパーが複数のプリムを受け持つ[^exp]。

## 姿勢の取得で、内部に Python のループがあるか（手元のソースで確認した）

`isaacsim.core.experimental.prims.XformPrim` の `get_world_poses` の各経路は次のとおりである。

| バックエンド | 内部の処理 | Python のループ |
|---|---|---|
| usd | プリムごとに `pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform()` を呼ぶ | あり |
| usdrt | プリムごとに `usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(...).get_world_xform()` を呼ぶ | あり |
| fabric | Warp のカーネル（`warp.launch()`）で、N 個の変換行列を一度に分解する | なし |

`isaacsim.core.experimental.prims.RigidPrim` と `isaacsim.core.experimental.prims.Articulation` の `get_world_poses` で tensor が選ばれた場合は、子の view の `get_transforms()` または `get_root_transforms()` を1回呼んで、N 個分をまとめて受け取る。Python のループはない。tensor が選ばれなかった場合は、`isaacsim.core.experimental.prims.XformPrim` の実装に処理を渡し、上の表のどれかで処理される。

## fabric と usdrt で扱えるデータ

usdrt と fabric を候補に持つメソッドは、次の15個だけである。このほかに、静的メソッドの `isaacsim.core.experimental.prims.Articulation.fetch_articulation_root_api_prim_paths()` が1個ある。どのメソッドも、扱えるのは姿勢（位置と向き）とスケールだけである。

| クラス | メソッド |
|---|---|
| `isaacsim.core.experimental.prims.XformPrim` | `get_world_poses`、`set_world_poses`、`get_local_poses`、`set_local_poses`、`set_local_scales`、`get_world_scales`、`get_local_scales` |
| `isaacsim.core.experimental.prims.RigidPrim` | `get_world_poses`、`set_world_poses`、`get_local_poses`、`set_local_poses` |
| `isaacsim.core.experimental.prims.Articulation` | `get_world_poses`、`set_world_poses`、`get_local_poses`、`set_local_poses` |

関節角などの DOF 系のデータは、usd か tensor でしか扱えない。そのため、Play OFF の状態で関節角を扱う場合、6.2.0 の Core Experimental API には、Python のループを回さない経路がない。

## 例：DOF のリミットを取得する場合

`isaacsim.core.experimental.prims.Articulation.get_dof_limits` の usd 経路は、プリムごと・DOF ごとの2重の Python ループで、USD のジョイント属性を読む。tensor 経路は、子の view の `get_dof_limits()` を1回呼ぶ（[バックエンド選択の仕組み（「バックエンド選択の3段階」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md) に抜粋がある）。

## 速さについての注意

- Python のループがないことは、メソッドとバックエンドの組ごとにソースで確かめる必要がある。
- fabric 経路は、呼び出すたびに `usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(...).update_world_xforms()` と selection の更新を行う。そのため、N が小さいときは usd より遅い。2026-11-09 にヘッドレスで測った `isaacsim.core.experimental.prims.XformPrim.get_world_poses()` の中央値は、N = 1 で usd 100 µs・fabric 208 µs、N = 10 で usd 337 µs・fabric 208 µs、N = 1000 で usd 24745 µs・fabric 237 µs だった（Play 中）。fabric は N によらず約 200 µs の固定費がかかり、usd と usdrt はプリム1個あたり約 22 µs ずつ伸びる。N が数個を超えると fabric のほうが速い（[bench_probe](../05_観測Snippet/04_bench_probe.md) の手順4）。

## 観測方法

[bench_probe](../05_観測Snippet/04_bench_probe.md) の `measure` で、同じメソッドをバックエンドを変えて測る。プリムの数 N を変えて、所要時間が N に比例して伸びるかを比べる。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー
from isaacsim.core.experimental.prims import RigidPrim  # 剛体のラッパー
from isaacsim.core.experimental.prims import XformPrim  # Xform のラッパー
from isaacsim.core.prims import Articulation  # 旧Core API のクラス（6.0.0 で非推奨）
from isaacsim.core.prims import RigidPrim  # 旧Core API のクラス（6.0.0 で非推奨）
from isaacsim.core.prims import XFormPrim  # 旧Core API のクラス（6.0.0 で非推奨）
from pxr import UsdGeom  # USD のジオメトリ API
import usdrt.hierarchy  # Fabric の階層 API（IFabricHierarchy）
import omni.physics.tensors  # view を作るモジュール
import warp  # Warp（GPU 配列とカーネル）
```

## 関連ノート

- [バックエンド選択の仕組み（「候補リストとメソッドの対応表」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [コンテキストマネージャと isaacsim.core.experimental.utils.backend.use_backend](04_コンテキストマネージャとuse_backend.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [旧Core API と新API の対応](06_旧CoreAPIと新APIの対応.md) |

[^exp]: [Core Experimental API — Isaac Sim 6.0.1](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/docs/overview/experimental.html)
