---
type: note
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, migration, core-api]
source:
  - https://github.com/isaac-sim/IsaacSim/tree/v6.2.0/source/deprecated
  - https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/deprecated/isaacsim.core.prims/docs/index.html
  - https://isaac-sim.github.io/IsaacLab/v2.1.1/source/refs/migration.html
---

# 旧Core API と新API の対応

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| 状態 | timeline の状態。Stop / Play / Pause の3つがある |
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| 数値フレームワーク | 計算結果を入れる配列の種類。numpy / torch（PyTorch）/ warp がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |
| view | `omni.physics.tensors` が提供するオブジェクトで、実行中の物理エンジンの内部オブジェクト群をまとめて読み書きする窓口 |
| simulation view | ステージと物理エンジンの組ごとに1つ作られる view の入口。Play で作られ、Stop で無効になる |

ほかの用語の定義は [用語集](01_用語集.md) にまとめている。

## 一言要約

Core Experimental API が置き換えるのは、旧Core API というラッパーの層である。USD、Fabric、omni.physics.tensors、PhysX は、統合されずに別々のライブラリのまま残っている。

## レイヤー構成

図：レイヤー構成（テキストで描いた図で、コードではない）
```
 利用者のスクリプト
        │ 呼ぶ
        ▼
 ┌─ ラッパー層（Isaac Sim Core）─────────────────────────────┐
 │ 旧: isaacsim.core.api / .prims / .utils（6.2.0 で非推奨）    │
 │ 新: isaacsim.core.experimental.*                             │
 │     + isaacsim.core.simulation_manager / rendering_manager   │
 └──────────┬───────────────┬──────────────────┬──────────┘
            │ usd            │ usdrt / fabric      │ tensor
            ▼               ▼                     ▼
       USD（pxr）     USDRT / Fabric      omni.physics.tensors（view）
            │               ▲                     │ 直接読み書き
            │ Play 時に読込  │ 計算結果を書く         ▼
            └────────────→ 物理エンジン（PhysX / Newton）
                         （設定によっては USD にも結果を書き戻す）
```

## 旧Core API（v6.2.0 では `source/deprecated/` に置かれている）

| 拡張 | 主な中身 | 置き換え先 | 置き換え先の根拠 |
|---|---|---|---|
| `isaacsim.core.prims` | `isaacsim.core.prims.XFormPrim`、`isaacsim.core.prims.Articulation`、`isaacsim.core.prims.RigidPrim` など | `isaacsim.core.experimental.prims` | 公式の非推奨表記[^dep] |
| `isaacsim.core.utils` | 各種ユーティリティ | `isaacsim.core.experimental.utils` | 拡張の非推奨表記（`extsDeprecated/isaacsim.core.utils/config/extension.toml` の「Extension deprecated since Isaac Sim 6.0.0 in favor of the Core Experimental extension: isaacsim.core.experimental.utils」） |
| `isaacsim.core.api` | World、SimulationContext など | `isaacsim.core.experimental.*`（拡張の非推奨表記はここまでしか書いていない） | 拡張の非推奨表記（`extsDeprecated/isaacsim.core.api/config/extension.toml` の「in favor of the Core Experimental extensions: isaacsim.core.experimental.*」）。World と SimulationContext の移行先を名指しする記述は、非推奨表記・ソース・拡張の文書のどこにもない。シミュレーションの状態・物理ステップ・コールバックは `isaacsim.core.simulation_manager.SimulationManager`、メインループの刻みと描画のコールバックは `isaacsim.core.rendering_manager.RenderingManager` が扱う |

Core Experimental API で新たに加わった拡張は、`isaacsim.core.experimental.objects`（形状・ライト）、`isaacsim.core.experimental.materials`、`isaacsim.core.experimental.actuators`、`isaacsim.core.experimental.primdata` である。

## クラス名の変遷

| 層 | 4.2 まで | 4.5 〜 5.x | 6.x |
|---|---|---|---|
| 複数プリム用ラッパー | `ArticulationView`、`RigidPrimView`、`XFormPrimView` | `Articulation`、`RigidPrim`、`XFormPrim` | `isaacsim.core.experimental.prims` の `Articulation`、`RigidPrim`、`XformPrim` |
| 単一プリム用ラッパー | `Articulation`、`RigidPrim`、`XFormPrim` | `SingleArticulation`、`SingleRigidPrim`、`SingleXFormPrim` | なし（単一プリムも複数プリムの一種として扱う） |
| omni.physics.tensors の view | articulation view など | 同じ | 同じ |

4.5 で、複数プリム用ラッパーの名前から「View」が外れ、単一プリム用ラッパーの名前の先頭に「Single」が付いた[^mig]。

## 数値フレームワークの扱い

- 旧Core API は、numpy か torch の配列を返す。`isaacsim.core.simulation_manager.SimulationManager.set_backend()` は simulation_manager 1.8.0 で非推奨になった（手元のソースで確認した）。
- Core Experimental API は、旧API とは別の Warp 用 simulation view を使い、常に Warp の配列を返す。入力には Python の list や NumPy 配列も渡せる。
- 物理のデバイスが GPU の場合、旧API 用の view は numpy から torch に自動で切り替わり、警告が出る。torch がインストールされていなければ、旧API 用の view は作られない。どちらの場合も、Core Experimental API の動作には影響しない。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.simulation_manager import SimulationManager  # シミュレーションの状態・物理ステップ・コールバックを扱うクラス
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー
from isaacsim.core.experimental.prims import RigidPrim  # 剛体のラッパー
from isaacsim.core.experimental.prims import XformPrim  # Xform のラッパー
from isaacsim.core.prims import XFormPrim  # 旧Core API のクラス（6.0.0 で非推奨）
from isaacsim.core.prims import Articulation  # 旧Core API のクラス（6.0.0 で非推奨）
from isaacsim.core.prims import RigidPrim  # 旧Core API のクラス（6.0.0 で非推奨）
import omni.physics.tensors  # view を作るモジュール
```

## 関連ノート

- [CoreExperimentalAPIの目的](07_CoreExperimentalAPIの目的.md)
- [viewとは何か](03_viewとは何か.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [複数プリム対応と Python ループの有無](05_複数プリム対応とPythonループの有無.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [Core Experimental API の目的](07_CoreExperimentalAPIの目的.md) |

[^dep]: [isaacsim.core.prims（非推奨）— Isaac Sim 6.2.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/deprecated/isaacsim.core.prims/docs/index.html)
[^mig]: [Migration Guide (Isaac Sim) — Isaac Lab v2.1.1](https://isaac-sim.github.io/IsaacLab/v2.1.1/source/refs/migration.html)
