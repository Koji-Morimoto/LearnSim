# MyExt Geometric Robot — Newton 統合版（`myext.robot.geometric_newton`）

対象環境：Isaac Sim 6.2.0 / Newton 1.6.1 / Warp 1.17.0

`myext.robot.geometric_newton` は、物理ソルバーや Drive を使わず、**関節値からリンクの Transform を計算して動かす**ロボット API 拡張機能です。既存の `myext.robot.geometric` とは独立した別拡張・別モジュール名（`myext.robot.geometric_newton`）として実装されており、同じ Isaac Sim 環境で共存できます。

## `myext.robot.geometric` に対する主な強化点

| 機能領域 | `myext.robot.geometric`（従来） | `myext.robot.geometric_newton`（本拡張） |
|---|---|---|
| 干渉判定エンジン（既定） | PhysX `overlap_shape`（`UsdPhysics.Scene` 必須） | Newton 1.6.1 自前コリジョン空間 `GeometricScene`（`UsdPhysics.Scene` 不要） |
| `Play=ON` × Fabric 駆動の遅延 | 1 フレーム遅延（BVH 置き去り）／`GeometricContactMonitor` は 2 update 遅延 | **0 フレーム遅延**（`flush_to_usd()` 不要で Frame 0 から全組を検出） |
| 取得できる干渉情報 | 重なりの有無（2 値） | 符号付き距離 `separation`（めり込み量・クリアランス）、接触点 `point_a` / `point_b`、法線 `normal` |
| 動的環境コライダーの追従 | USD 反映・再同期が必要 | `sync_environment()` / `set_obstacle_pose()` によるモデル再構築なし・0 フレーム遅延のその場更新 |
| 複数台並列評価 | NumPy バッチ順運動学 + CPU Fabric 書き込み | `BatchedNewtonScene`（multi-world + CUDA Graph）+ `GeometricGpuFabricWriter`（GPU 直接 Fabric 書き込み） |
| 後方互換性 | — | `collision_backend="physx"` / `fk_backend="numpy"` で従来挙動へ切替可能 |

## クイックスタート

```python
import omni.usd
from myext.robot.geometric_newton import GeometricRobot

stage = omni.usd.get_context().get_stage()
robot = GeometricRobot.from_articulation(stage, "/World/UR10e")   # collision_backend="newton", write_target="fabric"
robot.set_dof_positions([1.57, -0.5, 2.95, -1.57, -1.57, 0.0])    # Fabric のみに書き込み（USD 書き出し不要）

pairs = robot.check_collisions("all")                             # Frame 0 から 0 フレーム遅延で検出
for p in pairs:
    print(p.names, f"sep={p.separation * 1e3:.1f} mm")            # 負の値がめり込み量

near = robot.compute_clearances("all", max_distance=0.05)         # 5 cm 以内の近接ペアとクリアランス
robot.close()                                                     # Sublayer と GeometricScene を解放
```

### デモとテストの実行コマンド

| 用途 | コマンド |
|---|---|
| `Play=ON` × Newton デモ（単独起動） | `C:\isaacsim-6.2.0\python.bat demo\demo_playon_newton.py --no-ros-env`（`--headless --frames 120` 可） |
| IK + Articulation 切替デモ | `try_geometric_robot_newton.bat` または `C:\isaacsim-6.2.0\python.bat demo\demo_geometric_robot.py --no-ros-env` |
| 起動中の Isaac Sim（Script Editor） | `demo\demo_script_editor.py` を貼って実行（開いているステージは破棄されます） |
| 全テスト実行（pure 57 本 + Kit 34 本） | `powershell -ExecutionPolicy Bypass -File run_tests.ps1` |
| pure テストのみ実行 | `powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Pure` |
| Kit テスト（Newton 項目のみ） | `powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Kit -Only newton` |

## ドキュメント案内

目的に応じて 3 本の専用ドキュメントへ分かれています。

| ドキュメント | 類型 | 読む対象・内容 |
|---|---|---|
| [explanation_architecture.md](explanation_architecture.md) | 解説（Explanation） | コード編集者向け。モジュール依存関係、データフロー、座標系・四元数・行列の規約、設計判断の理由、機能別の編集箇所ガイド |
| [howto_recipes.md](howto_recipes.md) | ハウツー / レシピ（How-to） | 実装者向け。`Play=ON` 0 フレーム遅延駆動、`GeometricScene`、`GeometricRobotView` GPU バッチ、DH / MDH / URDF / `attach()`、IK、VariantSet、移行手順のコピペ可能なコード集 |
| [reference_api.md](reference_api.md) | リファレンス（Reference） | 全公開クラス・メソッドシグネチャ・引数・既定値・戻り値型、`config/extension.toml` 設定キー、20 モジュール構成の仕様一覧 |
