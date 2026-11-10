---
type: moc
created: 2026-11-08
updated: 2026-11-08
tags: [isaac-sim-6.2.0, moc, glossary]
---

# MOC-用語と概念

他のカテゴリのノートを読むときに前提になる、用語と概念のカテゴリである。

## 読む順序

| 順 | ノート | 内容 |
|---|---|---|
| 1 | [用語集](01_用語集.md) | このリポジトリで使う用語の定義と、確度の表記 |
| 2 | [backendという語の3つの意味](02_backendという語の3つの意味.md) | バックエンド・物理エンジン・数値フレームワークの区別 |
| 3 | [viewとは何か](03_viewとは何か.md) | omni.physics.tensors の view と、旧APIのクラス名との違い |
| 4 | [コンテキストマネージャとuse_backend](04_コンテキストマネージャとuse_backend.md) | with ブロックの設定が効く範囲 |
| 5 | [複数プリム対応とPythonループの有無](05_複数プリム対応とPythonループの有無.md) | バックエンドごとの内部ループの有無と、fabric で扱えるデータ |
| 6 | [旧CoreAPIと新APIの対応](06_旧CoreAPIと新APIの対応.md) | レイヤー構成、置き換え先、クラス名の変遷 |
| 7 | [CoreExperimentalAPIの目的](07_CoreExperimentalAPIの目的.md) | 公式が挙げる目的 |

## 他のカテゴリ

- [README（ルート目次）](../README.md)

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
import omni.physics.tensors  # view を作るモジュール
```

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [Isaac Sim 6.2.0 バックエンド学習ラボ](../README.md) | [MOC-用語と概念](00_MOC-用語と概念.md) | [用語集](01_用語集.md) |
