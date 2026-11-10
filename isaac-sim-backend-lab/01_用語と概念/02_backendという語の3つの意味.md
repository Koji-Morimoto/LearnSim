---
type: note
created: 2026-11-08
updated: 2026-11-08
tags: [isaac-sim-6.2.0, glossary, backend]
source:
  - C:\isaacsim-6.2.0\exts\isaacsim.core.simulation_manager\isaacsim\core\simulation_manager\impl\simulation_manager.py
  - C:\isaacsim-6.2.0\exts\isaacsim.core.experimental.utils\isaacsim\core\experimental\utils\impl\backend.py
---

# backend という語の3つの意味

## 一言要約

Isaac Sim 6.2.0 のソースでは、`backend` という英単語が3つの別々のものを指している。ソースを読むときは、どの意味で使われているかを毎回確かめる必要がある。

## 3つの意味

| 使われる場所 | 指すもの | 値 | このリポジトリでの呼び名 |
|---|---|---|---|
| `isaacsim.core.experimental.utils.backend.use_backend(backend)` | データを読み書きする経路 | usd / usdrt / fabric / tensor / simstate | バックエンド |
| `omni.physics.tensors.create_simulation_view(..., backend=...)` | 物理計算をする本体 | physx / newton | 物理エンジン |
| `isaacsim.core.simulation_manager.SimulationManager.get_backend()` / `isaacsim.core.simulation_manager.SimulationManager.set_backend()` | 結果を入れる配列の種類 | numpy / torch | 数値フレームワーク |

## 具体例（手元のソースで確認した）

- `isaacsim.core.simulation_manager.SimulationManager.initialize_physics()` は、`omni.physics.tensors.create_simulation_view("warp", stage_id=..., backend=cls._engine)` を呼ぶ。最初の引数 `"warp"` は数値フレームワーク、`backend=` に渡す `cls._engine` は物理エンジン名である。
- `isaacsim.core.simulation_manager.SimulationManager.set_backend()` は、simulation_manager 1.8.0 で非推奨になった。この値が影響するのは、旧Core API 用の simulation view だけである（[旧CoreAPIと新APIの対応](06_旧CoreAPIと新APIの対応.md)）。

## 境界

このリポジトリで単に「バックエンド」と書いた場合は、表の1行目（データを読み書きする経路）だけを指す。

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.simulation_manager import SimulationManager  # シミュレーションの状態・物理ステップ・コールバックを扱うクラス
import isaacsim.core.experimental.utils.backend as backend_utils  # use_backend / get_current_backend などを持つモジュール
import omni.physics.tensors  # view を作るモジュール
```

## 関連ノート

- [用語集](01_用語集.md)
- [MOC-用語と概念](00_MOC-用語と概念.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [用語集](01_用語集.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [view とは何か](03_viewとは何か.md) |
