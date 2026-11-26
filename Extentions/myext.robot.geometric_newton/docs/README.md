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
| カスタムコライダー Sublayer 切替 | 手動再読込が必要 | `attach_collider_layer()` / `set_collider_layer_enabled()` / `detach_collider_layer()` と `Usd.Notice.ObjectsChanged` 自動検知で 0 フレーム遅延切替 |
| 複数台並列評価 | NumPy バッチ順運動学 + CPU Fabric 書き込み | `BatchedNewtonScene`（multi-world + CUDA Graph）+ `GeometricGpuFabricWriter`（GPU 直接 Fabric 書き込み） |
| 後方互換性 | — | `collision_backend="physx"` / `fk_backend="numpy"` で従来挙動へ切替可能 |

## クイックスタート

起動中の Isaac Sim 6.2.0 の Script Editor にそのまま貼り付けて動く最小例です（事前ステージ準備不要、先頭の `ISAAC_ROOT` のみ環境に合わせて変更してください）。

```python
import asyncio
from pathlib import Path
import omni.kit.app
import omni.usd
from pxr import UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot


async def quickstart():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.DefinePrim("/World/UR10e", "Xform").GetReferences().AddReference(UR10E_USD.as_posix())
    await app.next_update_async()

    robot = GeometricRobot.from_articulation(stage, "/World/UR10e")   # collision_backend="newton", write_target="fabric"
    robot.set_dof_positions([1.57, -0.5, 2.95, -1.57, -1.57, 0.0])    # Fabric のみに書き込み（USD 書き出し不要）

    pairs = robot.check_collisions("all")                             # Frame 0 から 0 フレーム遅延で検出
    for p in pairs:
        print(p.names, f"sep={p.separation * 1e3:.1f} mm")            # 負の値がめり込み量

    near = robot.compute_clearances("all", max_distance=0.05)         # 5 cm 以内の近接ペアとクリアランス
    print("近接組数:", len(near))
    robot.close()                                                     # Sublayer と GeometricScene を解放


asyncio.ensure_future(quickstart())
```

## Demo カタログ（事前準備不要・Script Editor / CLI 両対応）

`demo/demo_01_*.py`〜`demo/demo_06_*.py` は、すべて**ステージ自動構築・照明・カメラ・ローカル同梱アセット参照を完結させた自己完結デモ**です。

- **Script Editor 実行**：ファイル全体を Script Editor に貼り付けて実行（開いているステージは新規ステージに置き換わります）。
- **コマンドライン単独起動**：`C:\isaacsim-6.2.0\tools\myext\run_sim.bat demo\<ファイル名>`（`--headless --frames 120` オプション対応）。`tools\myext\sim_app_launcher.py` の `create_simulation_app()` 経由で HUD FPS 表示・必須拡張・UTF-8 コンソールを有効化して起動します。
- 別のフォルダに Isaac Sim 6.2.0 を配置している場合は、各デモ先頭の `ISAAC_ROOT`（または環境変数 `ISAAC_PATH`）を書き換えてください。

| デモファイル | テーマ | 主に体験できる API |
|---|---|---|
| `demo/demo_01_playon_ur10e_zero_lag.py` | `Play=ON` × UR10e × 0 フレーム遅延干渉判定 | `GeometricRobot.from_articulation`, `write_target="fabric"`, `compute_clearances`（1 パス貫通＋近接）, PhysX 動的剛体との共存 |
| `demo/demo_02_geometric_scene_moving_env.py` | `UsdPhysics.Scene` なしの動的環境・双腕干渉 | `GeometricScene`, `add_environment`, `sync_environment`, `set_obstacle_pose`, `check_between` |
| `demo/demo_03_collider_sublayer_toggle.py` | カスタムコライダー `.usda` Sublayer の実行時 ON/OFF | `attach_collider_layer`, `set_collider_layer_enabled`, `detach_collider_layer`, `collider_paths`, `collider_version` |
| `demo/demo_04_dh_mdh_screw_ik_dual_arm.py` | DH / 修正 DH / スクリュー軸 / URDF + 双腕 `attach()` + IK | `from_dh`, `from_mdh`, `from_screw`, `from_urdf`, `attach()`, `get_component`, `create_ik_solver`, `solve_ik`, `set_active_dofs` |
| `demo/demo_05_gpu_batch_view.py` | 16 台 UR10e の GPU バッチ FK・干渉・GPU 直接 Fabric | `GeometricRobotView.from_articulations`, `BatchedNewtonScene` (`use_cuda_graph=True`), `use_gpu_fabric_writer=True`, `check_collisions_batch`, `compute_clearances_batch` |
| `demo/demo_06_variant_deg_converter.py` | VariantSet 切替・角度単位 `"deg"`・`CommandConverter` | `set_variant`, `get_variant_choices`, `add_structure_listener`, `angle_unit="deg"`, `LinearCommandConverter(..., when=...)`, `apply_command`, `read_command` |

従来のデモおよびテスト実行コマンド一覧です。

| 用途 | コマンド |
|---|---|
| `Play=ON` × Newton 簡易デモ（単独起動） | `C:\isaacsim-6.2.0\tools\myext\run_sim.bat demo\demo_playon_newton.py`（`--headless --frames 120` 可） |
| IK + Articulation 切替デモ（従来互換） | `try_geometric_robot_newton.bat` または `C:\isaacsim-6.2.0\tools\myext\run_sim.bat demo\demo_geometric_robot.py` |
| Script Editor 簡易デモ（従来互換） | `demo\demo_script_editor.py` を Script Editor に貼って実行 |
| 全テスト実行（pure + Kit） | `powershell -ExecutionPolicy Bypass -File run_tests.ps1` |
| pure テストのみ実行 | `powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Pure` |
| Kit テスト（Newton 項目のみ） | `powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Kit -Only newton` |

## ドキュメント案内（HTML 自己完結スイート推奨）

ブラウザでそのまま開ける自己完結 HTML ドキュメント（全 9 ページ、公式ドキュメント出典・検索フィルタ・相互ハイライト図解付き）を推奨エントリポイントとして用意しています。

| ドキュメント | 類型 | 読む対象・内容 |
|---|---|---|
| [index.html](index.html) | ポータル | 全ドキュメントへの入口、クイックスタート、目的別ガイド、主要機能一覧 |
| [explanation_architecture.html](explanation_architecture.html) | 解説（Explanation） | コード編集者向け。モジュール依存関係、データフロー図、座標系・四元数・行列規約、`convex_meshes` / `Show by Type` / `VariantSet` の背景、機能別の編集箇所ガイド |
| [howto_recipes.html](howto_recipes.html) | ハウツー / レシピ（How-to） | 実装者向け。`Play=ON` 0 フレーム遅延駆動、`IsaacRobotAPI` 自動コンポーネント＋`batch_update()`、`convex_meshes=False` 凹メッシュ、DebugDraw コライダー可視化、`GeometricScene`、`GeometricRobotView` GPU バッチ、カスタムコライダー Sublayer 切替、DH/MDH/URDF/IK/VariantSet のレシピ集 |
| [api/index.html](api/index.html) | リファレンス（Reference） | API 目次・逆引き表・性能ティアガイド・`config/extension.toml` 全 11 設定キー・型エイリアス一覧。個別仕様書：[robot_api.html](api/robot_api.html) / [collision_newton_api.html](api/collision_newton_api.html) / [view_batch_api.html](api/view_batch_api.html) / [model_ik_command_api.html](api/model_ik_command_api.html) / [layer_writer_internals.html](api/layer_writer_internals.html) |

従来の Markdown 版（[old/explanation_architecture.md](old/explanation_architecture.md)・[old/howto_recipes.md](old/howto_recipes.md)・[old/reference_api.md](old/reference_api.md)）も参照用に残していますが、最新の完全仕様は HTML 版を参照してください。
