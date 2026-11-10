---
type: note
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, view, omni-physics-tensors]
source:
  - C:\isaacsim-6.2.0\exts\isaacsim.core.simulation_manager\isaacsim\core\simulation_manager\impl\simulation_manager.py
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.prims\isaacsim\core\experimental\prims\impl\articulation.py
  - https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.physics.newton.tensors/docs/index.html
  - https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/concepts/physics-backends/physx.html
  - https://isaac-sim.github.io/IsaacLab/v2.1.1/source/refs/migration.html
---

# view とは何か

## 一言要約

view とは、`omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口である。

## USD のプリムとの違い

| | USD のプリム | view |
|---|---|---|
| 何を表すか | シーンの記述データ | 物理エンジンが実行中に持っている状態への参照 |
| いつ存在するか | Stop 中も存在し、ファイルに保存される | Play で物理が初期化された後に作られ、Stop で無効になる |
| 1回の読み書きで扱う数 | プリム1個 | 指定したパスに当てはまる全プリム |

## 階層

```
simulation view（ステージと物理エンジンの組ごとに1つ）
 ├─ articulation view（パスで選んだ Articulation の集まり）
 ├─ rigid body view（パスで選んだ剛体の集まり）
 └─ rigid contact view（接触の報告）
```

- **simulation view：** `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` が Play 時に作る。`isaacsim.core.simulation_manager.SimulationManager.get_physics_simulation_view()` で取得できる（Warp 用）。
- **子の view：** simulation view の `create_articulation_view(パス)` などで作る。articulation view は、DOF の位置・速度・リミット・剛性・減衰、ルートやリンクの姿勢と速度、質量などを、まとめて読み書きする手段を持つ[^newton]。

## 具体例

- Core Experimental API の `isaacsim.core.experimental.prims.Articulation` ラッパーは、PHYSICS_READY イベントを受けると `create_articulation_view(self.paths)` を呼び、子の view を `isaacsim.core.experimental.prims.Articulation._physics_articulation_view` に保存する。ラッパーを作った時点で simulation view がすでにあれば、その場で子の view を作る（手元のソースで確認した）。
- エラーメッセージに出る「physics tensor entity」は、この子の view を指す。
- Isaac Lab のドキュメントによると、view の getter はライブなポインタを返さない。返された配列を書き換えても、シミュレーションは変わらない[^lab]。Isaac Sim 6.2.0 単体でも同じだった（2026-11-09 にヘッドレスで実測）。Play 中に `isaacsim.core.experimental.prims.RigidPrim.get_world_poses()` が返した配列の z を 100 に書き換えても、物理エンジンの値は変わらず、もう一度呼ぶと元の値の入った別の配列が返った。子の view の `get_transforms()` が返す配列を直接書き換えても、同じだった。値を物理に反映するには、`set_world_poses()` などの set 系のメソッドを呼ぶ。

## view を複数作る場面

view はステージごとではなく、プリムの組ごとに作る。

- `isaacsim.core.experimental.prims.Articulation` のラッパーを3つ作れば、1つの simulation view から子の view が3つ作られる。
- 並列環境は、1つのステージの中に環境を複製し、`/World/envs/env_*/Object` のようなパスのパターンで、すべての環境を1つの view にまとめて扱う。
- 1つのプロセスで物理シミュレーションできるステージは、同時に1つだけである。omni.physx のステージをつなぐ関数（`omni.physx.get_physx_simulation_interface().attach_stage()`）は、前につないでいたステージを切り離してから新しいステージをつなぐ（`IPhysxSimulation.h` の説明「previous stage will be detached」）。2026-11-09 にヘッドレスで、Play 中のステージとは別のステージをつなぐと、元のステージの剛体は物理エンジンから外れ（姿勢が読めなくなり）、切り離すと元のステージに戻った（実測）。並列の環境は、1つのステージの中に複製して作る。

## 境界

- **Stop：** `isaacsim.core.simulation_manager.SimulationManager.invalidate_physics()` が simulation view を無効化し、None にする。ラッパーの `isaacsim.core.experimental.prims.Articulation.is_physics_tensor_entity_valid()` は False になる。
- **Pause：** view は無効化されない。
- **名前の衝突：** 旧Core API（Isaac Sim 4.2 まで）には、`ArticulationView` や `RigidPrimView` という名前のラッパークラスがあった。これらは 4.5 で `Articulation` や `RigidPrim` に名前が変わった[^mig]。これらのラッパークラスは、この資料でいう view（omni.physics.tensors のオブジェクト）とは別のものである。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.simulation_manager import SimulationManager  # シミュレーションの状態・物理ステップ・コールバックを扱うクラス
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー
import omni.physics.tensors  # view を作るモジュール
```

## 関連ノート

- [用語集](01_用語集.md)
- [旧CoreAPIと新APIの対応](06_旧CoreAPIと新APIの対応.md)
- [時間と物理ステップ（「Play・Pause・Stop で時計がどう動くか」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [backend という語の3つの意味](02_backendという語の3つの意味.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [コンテキストマネージャと isaacsim.core.experimental.utils.backend.use_backend](04_コンテキストマネージャとuse_backend.md) |

[^newton]: [isaacsim.physics.newton.tensors — Isaac Sim 6.2.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.physics.newton.tensors/docs/index.html)
[^lab]: [IsaacSim PhysX — Isaac Lab 3.0.0](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/concepts/physics-backends/physx.html)
[^mig]: [Migration Guide (Isaac Sim) — Isaac Lab v2.1.1](https://isaac-sim.github.io/IsaacLab/v2.1.1/source/refs/migration.html)
