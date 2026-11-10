---
type: note
created: 2026-11-08
updated: 2026-11-08
tags: [isaac-sim-6.2.0, core-experimental]
source:
  - https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/docs/overview/experimental.html
  - https://docs.isaacsim.omniverse.nvidia.com/latest/physics/newton_physics.html
---

# Core Experimental API の目的

## このノートで使う用語

| 用語 | 意味 |
|---|---|
| バックエンド | Core Experimental API がデータを読み書きする経路。usd / usdrt / fabric / tensor / simstate がある |
| 物理エンジン | 力・拘束・接触を解いて次の状態を計算する本体。PhysX / Newton がある |
| ラッパー | Core Experimental API の `isaacsim.core.experimental.prims.Articulation`・`isaacsim.core.experimental.prims.RigidPrim`・`isaacsim.core.experimental.prims.XformPrim` などのクラス。1つのラッパーが1個以上のプリムを受け持つ |

ほかの用語の定義は [用語集](01_用語集.md) にまとめている。

## 一言要約

Core Experimental API は、旧Core API を書き直したラッパー層である。公式が挙げる目的は、依存の削減、他の機械学習フレームワークとの連携、保守性、全ラッパーの複数プリム対応、複数の物理エンジンへの対応の5つである。

## 公式が挙げている目的

| 目的 | 内容 |
|---|---|
| 依存の削減 | Core Experimental API が本流になった後は、PyTorch が依存から外れ、配布サイズが約 4 GB 減る[^exp]。現時点では PyTorch 2.7.0 が依存に含まれている |
| 他の機械学習フレームワークとの連携 | PyTorch 2.7.0 と JAX 0.6 以上が依存関係で衝突する例が挙げられている[^exp] |
| 保守性 | 実装を単純にし、保守しやすくする[^exp] |
| 全ラッパーの複数プリム対応 | [複数プリム対応とPythonループの有無](05_複数プリム対応とPythonループの有無.md) |
| 複数の物理エンジン対応 | PhysX と Newton のどちらでも動くコードを書くために、`isaacsim.core.experimental` を使うよう公式が強く推奨している[^newton] |

主な機能の1つとして、「フォールバック付きのバックエンド選択」も挙げられている[^exp]。

## 書かれていないこと

「バックエンドを明示的に指定しなければならないことが、利用者の参入障壁になっていた」という趣旨の記述は、2026-11-08 時点で、出典に挙げた公式ドキュメントの中には見つからなかった。

## 注意

公式は、このAPIが実験的なものであり、非推奨期間を設けずに変更されることがあると警告している[^exp]。旧Core API は将来非推奨になり、削除される予定である[^exp]。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.experimental.prims import Articulation  # Articulation のラッパー
from isaacsim.core.experimental.prims import RigidPrim  # 剛体のラッパー
from isaacsim.core.experimental.prims import XformPrim  # Xform のラッパー
```

## 関連ノート

- [旧CoreAPIと新APIの対応](06_旧CoreAPIと新APIの対応.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [旧Core API と新API の対応](06_旧CoreAPIと新APIの対応.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [バックエンド選択の仕組み](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md) |

[^exp]: [Core Experimental API — Isaac Sim 6.2.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/docs/overview/experimental.html)
[^newton]: [Newton Physics Backend — Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/physics/newton_physics.html)
