# `tests_kit/` テスト方針とテスト一覧

このフォルダには、`myext.robot.assembly_pipeline.v2` の動作を Isaac Sim 6.2.0 の Headless 環境（`create_simulation_app({"headless": True})`）で自動検証する 2 つのテストスイートが収められています。

---

## 1. テスト全体の方針（2 つのスイートの役割分担）

本拡張のテストは、目的と検証粒度の違いに応じて 2 つのスイートに分かれています。

| スイート | テスト数 | 役割と検証対象 | 結果ファイル |
|---|---|---|---|
| `tests_kit\run_tests.py` | 全 18 テスト | **パイプライン機能・規則・API・冪等性の統合テスト**。URDF パーサ、TOML 静的検査、規則 R01〜R14 の検知・修正・`skip`/`stop` 制御、再実行時の SHA-256 冪等性、非破壊ビルド `build_assembly`、`AssembledRobotDescriptor` / `discover_components`、複数 `VariantSet` の順序制御、外部幾何拡張への依存ゼロ、URDF+USD 混在結合を検証する。 | `tests_kit\last_run.json`（`run_tests.ps1` 経由時は `tests_kit\last_run.log` も生成） |
| `tests_kit\test_official_motion_match.py` | 全 6 テスト | **公式 Isaac Sim アセットとの定量的な PhysX 動作一致テスト**。Isaac Sim 公式ロボット（UR10e / Unitree G1 / NVIDIA Kaya / NVIDIA Carter v1 / Denso Cobotta Pro 900 / OpenArm Bimanual）を部品から本パイプラインで再構成し、公式アセットと同一の関節指令・重力・接地条件で PhysX シミュレーションを実行して、全リンクの位置・姿勢および全 DOF・mimic 追従が定量的に一致することを検証する。 | `tests_kit\last_official_motion_run.json` |

### 設計上の共通ルール

- **1 プロセス 1 `SimulationApp`**: テストごとに Isaac Sim を再起動せず、スイート冒頭で `C:\isaacsim-6.2.0\tools\myext\sim_app_launcher.py` の `create_simulation_app({"headless": True})` を 1 回だけ起動し、各テスト内で新規ステージを作成して分離する。
- **ウォッチドッグ保護**: 無限ハングを防ぐため 3600 秒のタイマーを設定し、超過時は `[WATCHDOG]` を出力して終了コード `3` で終了する。
- **確実な終了コード返却**: Kit が終了時に例外を握りつぶすことがあるため、テスト末尾で `[SUMMARY] passed <P> / failed <F>` と JSON 結果を書き出し、`os._exit(1 if failed else 0)` で終了する。

---

## 2. 実行方法

すべてのコマンドは拡張ルートフォルダから実行します。いずれも **Headless（GUI なし）で実行され、全テスト完了後に自動終了** します。

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline.v2
```

### 2-1. 統合テストスイート（`tests_kit\run_tests.py`：全 18 テスト）

PowerShell ラッパー `run_tests.ps1` を使う方法（推奨：`PYTHONPATH` の自動クリアと `[SUMMARY]` 行の厳密チェックを含む）:

- **出力先**: 全ログ `tests_kit\last_run.log`、構造化結果 `tests_kit\last_run.json`
- **成功の合図**: 端末に `[SUMMARY] passed 18 / failed 0` および `== exit=0` が表示されること

```powershell
# 全 18 テストを実行
powershell -ExecutionPolicy Bypass -File run_tests.ps1

# テスト関数名に "franka" を含むテストだけ実行
powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Only franka
```

`run_sim.bat` から直接実行する方法:

```powershell
$env:PYTHONPATH = ""
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\run_tests.py
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\run_tests.py --only=multi_arm
```

> **ログに関する補足**: `tests_kit\run_tests.py` では `run_pipeline` の公式 `RobotAssembler` 比較フェーズも実行されるため、ログファイル `tests_kit\last_run.log` 内に比較用の PhysX `closed articulation` エラーが記録されます。成否の判定は `[PASS]` / `[FAIL]` / `[SUMMARY]` と終了コードで行います（詳細は [docs/README.md](../docs/README.md) の「ログの読み方」節を参照）。

### 2-2. 公式アセット定量 PhysX 動作一致テスト（`tests_kit\test_official_motion_match.py`：全 6 テスト）

- **出力先**: 定量メトリクスと成否を `tests_kit\last_official_motion_run.json` に保存
- **成功の合図**: 最終行に `[SUMMARY] official_motion passed 6 / failed 0` が表示され、終了コード `0` で終了すること

```powershell
# 全 6 テストを実行
$env:PYTHONPATH = ""
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py

# 特定のロボットだけ実行（関数名の一部で絞り込み）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py --only=ur10e
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py --only=openarm
```

---

## 3. テスト一覧と合格基準

### 3-1. `tests_kit\run_tests.py`（全 18 テスト）

| # | テスト関数名 | 検証対象 | 主な合格基準 |
|---|---|---|---|
| 1 | `test_urdf_model_ghost_and_fk` | `urdf_model.py` のゴーストリンク判定と解析的 FK | `flange` / `gripper_mount` のゴースト判定、`fk_zero` / `fk` の誤差 `< 1e-9`、`fixed_descendants` 一致 |
| 2 | `test_spec_validation` | `spec.py` と `spec_check.py` の静的エラー検出 | `.usda` 重複・未定義参照・拡張子欠落・循環参照・誤リンク名・無効な `child_variant_selection` をすべて検出 |
| 3 | `test_join_under_link` | 既定のリンク配下結合（`arm` + `gripper`） | `status == "ok"`、`R06`/`R07`/`R09`/`R14` が `fixed`、`R10` が `upstream_fixed`、`R02`/`R08`/`R12` が `ok`、順運動学最大誤差 `< 1e-4 m` |
| 4 | `test_idempotent_rerun` | 2 回目実行のキャッシュと冪等性 | 2 部品の import スキップ、全 `.usd*` の SHA-256 完全一致、`_import/*_1` 未生成、`report_previous.json` 生成 |
| 5 | `test_join_under_top` | ルート prim 直下への結合（`place_child_under = "."`） | `status == "ok"`、`R02`/`R12` が `fixed`、`R09` が `ok` |
| 6 | `test_skip_shows_still_broken` | `fix = "skip"` による修正無効化 | `R02`/`R07`/`R12` がすべて `still_broken` となり `status == "problems"` を返す |
| 7 | `test_stop_before_join` | `on_violation = "stop"` による事前停止 | `PipelineStopped` 送出、`status == "stopped_before_join"`、結合ファイル未生成 |
| 8 | `test_ghost_root_child` | ルートリンクがゴーストリンクの子部品（`gripper_ghost.urdf`） | `status == "ok"`、`R01`/`R08` が `fixed`、順運動学最大誤差 `< 1e-4 m` |
| 9 | `test_mimic_damping` | mimic follower 関節の drive ゲイン修正（`R03`） | URDF 既定は `R03 = ok`、`joint_drive_damping = 5.0` 上書き時は `R03 = fixed` |
| 10 | `test_multi_arm` | `examples\multi_arm.toml`（2 段結合・4 本アーム・VariantSet） | `status == "ok"`、全 6 結合で `R07`/`R09` が `fixed`、単一 Articulation Root、13 DOF 全到達（誤差 `< 1e-3`）、順運動学 `< 1e-4 m`、4 アームの Variant 選択一致 |
| 11 | `test_multi_arm_rerun` | `examples\multi_arm.toml` の再実行冪等性 | 全 `.usd*` の SHA-256 完全一致かつ `status == "ok"` |
| 12 | `test_franka_example` | `examples\franka\franka.toml`（Franka Panda + Hand） | `status == "ok"`、`R09 = fixed`、`R04 = ok`、URDF マウント整列誤差 `< 1e-6 m` / `< 1e-3 deg`、順運動学 `< 1e-4 m` |
| 13 | `test_example_toml` | `arm_gripper.toml` / `multi_arm.toml` の TOML 読み込み | 相対パス解決、`file_order()` 依存順、`child_variant_selection` の読込一致 |
| 14 | `test_build_assembly_and_descriptor` | `build_assembly` 非破壊性・`AssembledRobotDescriptor`・`spawn_assembled_robot` | アクティブステージのセンチネル prim 無破壊、`arm4/end_effector`（非アクティブ Variant）を含む全構成要素取得、`to_articulation_source_dict()` のキー構造一致 |
| 15 | `test_multi_variant_and_non_variant_ordering` | 複数 `VariantSet` と非 Variant 結合の混在順序 | 結合エラー 0 件、`Slot1 = "ArmA"`・`Slot2 = "Grip"` の既定選択維持、全構成要素取得 |
| 16 | `test_zero_external_geometric_dependency` | 外部幾何拡張（`myext.robot.geometric*`）への依存ゼロ | `sys.modules` 未読込、`config\extension.toml` の `dependencies` に未記載 |
| 17 | `test_usd_to_usd_ur10e_pipeline` | `examples\official_assets\ur10e\ur10e.toml` の USD+USD フル検証と冪等性 | `status == "ok"`、DOF 名一致、順運動学 `< 1e-4 m`、2 回目実行の SHA-256 完全一致 |
| 18 | `test_mixed_urdf_and_usd_pipeline` | 親 URDF（`arm.urdf`）+ 子 USD（`Robotiq_2F_85.usda`）混在結合 | `status == "ok"`、DOF 名一致、順運動学 `< 1e-4 m`、構成要素 `arm_with_robotiq` / `gripper` 検出 |

### 3-2. `tests_kit\test_official_motion_match.py`（全 6 テスト）

各テストは公式アセットと本パイプライン生成 `.usda` を同一条件で PhysX 駆動し、DOF 数・剛体数・リンク位置誤差・リンク姿勢誤差・DOF 角度誤差を比較します。

| # | テスト関数名 | 対象ロボット | 入力 TOML vs 公式比較対象 | 主な検証条件と合格基準 |
|---|---|---|---|---|
| 1 | `test_1_ur10e_all_gripper_variants` | UR10e + Robotiq 2F-85 / 2F-140 | `examples\official_assets\ur10e\ur10e.toml` vs 公式 `ur10e.usd` | `Gripper` 全 3 選択肢（`None`: 6 DOF / 7 RB、`Robotiq_2F_85`: 12 DOF / 16 RB、`Robotiq_2F_140`: 14 DOF / 16 RB）で無重力 3 姿勢比較 + `Robotiq_2F_85` の重力 `9.81` 比較（位置 `< 0.005 m`、姿勢 `< 1.5 deg`、DOF `< 0.02 rad`） |
| 2 | `test_2_g1_variant_combinations` | Unitree G1 29-DOF | `examples\official_assets\g1\g1.toml` vs 公式 `g1.usda` | 4 通りの `VariantSet` 組み合わせ（`ThreeFinger`×2、`Inspire`×2、`None`+`Thor=Attached`、`ThreeFinger`+`Inspire`+`Thor=Attached`）。浮遊ベースのため `pelvis` 相対姿勢と DOF 角度、および `Thor` 複合バックパック（追加剛体・追加 DOF 0）を検証 |
| 3 | `test_3_kaya_kinematic_and_grounded_rolling` | NVIDIA Kaya | `examples\official_assets\kaya\kaya.toml` vs 公式 `kaya.urdf` | 3a: 無重力 33 DOF / 45 RB 比較（位置 `< 0.003 m`、姿勢 `< 1.0 deg`、DOF `< 0.02 rad`）。3b: 重力 `9.81` + `GroundPlane` 上での 3 輪ホロノミック並進・旋回走行軌跡の一致 |
| 4 | `test_4_carter_kinematic_and_grounded_driving` | NVIDIA Carter v1 | `examples\official_assets\carter\carter.toml` vs 公式 `carter.urdf` | 4a: 無重力 4 DOF / 7 RB 比較（位置 `< 0.003 m`、姿勢 `< 1.0 deg`、DOF `< 0.02 rad`、右輪 `Rz(pi)` 符号反転対応）。4b: 重力 `9.81` + `GroundPlane` 上での直進・差動旋回（後輪キャスター追従含む）軌跡の一致 |
| 5 | `test_5_cobotta_pro_900_and_rg6` | Denso Cobotta Pro 900 + OnRobot RG6 | `examples\official_assets\cobotta_pro_900\cobotta_pro_900.toml` vs 公式 `cobotta_pro_900.urdf` | 5a: `OnRobot_RG6` 装着時（12 DOF / 13+ RB、位置 `< 0.002 m`、姿勢 `< 0.5 deg`、DOF `< 0.01 rad`、5 mimic 追従誤差 `< 0.02 rad`）。5b: `Gripper = "None"` 時（6 DOF / 7 RB、同許容誤差） |
| 6 | `test_6_openarm_bimanual` | OpenArm Bimanual | `examples\official_assets\openarm\openarm_bimanual.toml` vs 公式 `openarm_bimanual.usd` | 6a: 無重力 22 DOF / 23 RB・3 姿勢（位置 `< 0.002 m`、姿勢 `< 0.5 deg`、DOF `< 0.01 rad`、両手グリッパ mimic 誤差 `< 0.002 m`）。6b: 重力 `9.81` 同一 3 姿勢（位置 `< 0.005 m`、姿勢 `< 1.5 deg`、DOF `< 0.02 rad`）。6c: `LeftArm` / `RightArm` 片腕 Variant（11 DOF / 12 RB） |

---

## 4. 新しいテストを追加する手順

### 4-1. ルール・パイプライン機能テストを `tests_kit\run_tests.py` に追加する

1. 必要であればテスト用 URDF を `examples\urdf\` に追加する（既存の `arm.urdf` / `gripper.urdf` / `one_join(...)` ヘルパを再利用できる場合はそのまま使う）。
2. `tests_kit\run_tests.py` に `@test` デコレータ付きで関数 `def test_<内容>():` を追加する。
3. 出力フォルダは必ず `os.path.join(OUT, "<テスト固有フォルダ名>")` に分離し、テスト開始前に必要なら `shutil.rmtree(..., ignore_errors=True)` で初期化する。
4. 規則の判定確認には既存の `verdicts(rep)` / `expect(v, (rule_id, target), want, rep)` / `problems(rep)` ヘルパを使う。
5. 追加したテストだけを絞り込み実行して確認し、最後に全テストを通して回帰が無いことを確認する。

```powershell
powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Only <追加した関数名の一部>
powershell -ExecutionPolicy Bypass -File run_tests.ps1
```

### 4-2. 新しいロボットの公式動作一致テストを `tests_kit\test_official_motion_match.py` に追加する

1. `examples\official_assets\<robot_name>\` に部品 URDF/USD（または薄ラッパ `.usda`）と `<robot_name>.toml` を配置し、まず CLI の `--check` と `--build-only` で問題なく結合できることを確認する。
2. `tests_kit\test_official_motion_match.py` の冒頭に公式比較対象アセットのパス定数を追加する。
3. `@test` デコレータ付きで `def test_<番号>_<robot_name>():` を追加し、既存の共通ヘルパ（`_build_official_example`, `_simulate_position_trajectory`, `_compare_trajectories`, 移動ロボットなら `_simulate_grounded_velocity`）を使って公式アセットと結合 `.usda` を同一指令で比較する。
   - 固定ベースロボットはワールド座標でのリンク位置・姿勢比較（無重力 + 重力 `9.81`）を行う。
   - 浮遊ベースロボットはルート剛体（例: `pelvis`, `base_link`, `chassis_link`）基準の相対姿勢比較、および必要なら `GroundPlane` 上の動的走行比較を行う。
4. `--only=<robot_name>` で単体確認した後、スイート全体を実行して全テスト合格と `tests_kit\last_official_motion_run.json` の更新を確認する。

```powershell
$env:PYTHONPATH = ""
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py --only=<robot_name>
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py
```
