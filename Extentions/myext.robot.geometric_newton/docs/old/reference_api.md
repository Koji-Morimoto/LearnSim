# API・設定・ファイル構成リファレンス（`myext.robot.geometric_newton`）

> **完全版の API リファレンスは自己完結 HTML 版へ移行しました。**
> すべての公開クラス・プロパティ・メソッドについて、引数・型・既定値・形状・単位・Mimic 規約・性能ティア・落とし穴・最小コード例・公式出典を記載し、インクリメンタル検索付きで分割整理しています。

## HTML API リファレンス（完全版）

| ページ | 主な収録内容 |
|---|---|
| [docs/index.html](index.html) | ドキュメント全体ポータル（アーキテクチャ解説・レシピ集・API 仕様書への入口） |
| [docs/api/index.html](api/index.html) | API 目次・やりたいこと逆引き表・ホットパス性能ガイド・`config/extension.toml` 全 11 設定キー・型エイリアス一覧 |
| [docs/api/robot_api.html](api/robot_api.html) | `GeometricRobot`（全 18 引数・ファクトリ・`independent_dof_indices` / `Mimic` 規約・`set_dof_positions`・`batch_update()`・`describe()`・`convex_meshes`・`debug_draw_colliders` / `draw_colliders()`・`VariantSet`・Collider Sublayer）・`GeometricRobotComponent`（`IsaacRobotAPI` 自動生成）・`GeometricJoint`・`GeometricLink`・`GeometricStructureChangedError` |
| [docs/api/collision_newton_api.html](api/collision_newton_api.html) | `resolve_newton_device`・`GeometricScene`・`convex_meshes=True` vs `False` 実測比較と水密メッシュ要件・`BatchedNewtonScene`・`GeometricCollisionPair`（全 13 フィールド）・`GeometricLinkSet`・`find_colliders`・`GeometricCollisionFilter`（全 16 メソッド）・PhysX バックエンド |
| [docs/api/view_batch_api.html](api/view_batch_api.html) | `GeometricRobotView`（`from_articulations`・`(N, D)` / `wp.array` バッチ操作・CUDA Graph・`use_gpu_fabric_writer`・構造変更時例外） |
| [docs/api/model_ik_command_api.html](api/model_ik_command_api.html) | `GeometricRobotModel`（全プロパティ・全 18 メソッド）・`UsdArticulationSource`（`auto_components=True`）・`UsdRobotInfo`・`DH` / `MDH` / `Screw` / `URDF`・`build_robot_prims`・`KinematicChain`・IK ソルバー群・`GeometricCommandConverter` 群 |
| [docs/api/layer_writer_internals.html](api/layer_writer_internals.html) | `GeometricLayer` / `GeometricLayerOptions` / `GeometricLayerReport` / `authoring()`・`LocalPoseSolver`・`GeometricFabricWriter` / `GeometricGpuFabricWriter` / `GeometricUsdWriter`・列ベクトル＆`(w, x, y, z)` 座標規約・`model.transforms` 全 13 関数 |

## 関連 HTML ドキュメント

- [docs/explanation_architecture.html](explanation_architecture.html) — アーキテクチャ解説（設計背景・図解・編集ガイド）
- [docs/howto_recipes.html](howto_recipes.html) — 目的別ハウツー・レシピ集（10 レシピ）
