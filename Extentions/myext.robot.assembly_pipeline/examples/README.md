# `examples/` サンプル仕様ガイド

このフォルダには、`myext.robot.assembly_pipeline` の動作をすぐ試せる TOML 仕様ファイルと、テスト・サンプル用の URDF / USD / メッシュが収められています。

実行モードやログの読み方の全体説明は [docs/README.md](../docs/README.md) を参照してください。

---

## 実行前の共通手順

すべてのコマンドは拡張ルートフォルダから `C:\isaacsim-6.2.0\tools\myext\run_sim.bat` 経由で実行します。

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline
```

各オプションの挙動は次のとおりです。

| オプション | 画面の挙動 | 出力物 | 成功の合図 |
|---|---|---|---|
| `--gui` | **Isaac Sim GUI を開き、ロボットを `/World/<stem>` に配置して画面を維持** | `output_dir` 配下に `.usda` 群 | `[GUI] 組み立て済みロボットを Isaac Sim GUI に配置しました` |
| `--build-only` | **Headless で `.usda` を生成して自動終了**（エラーログなし） | `output_dir` 配下に `.usda` 群 | `[BUILD] .usda の非破壊生成が完了しました`・終了コード `0` |
| `--check` | **Isaac Sim を起動せず数秒で自動終了** | ファイル出力なし | `[CHECK] 仕様に誤りは無い`・終了コード `0` |
| オプションなし | **Headless でフル検証 + レポート生成して自動終了** | `.usda` 群 + `report.md` / `report.json` | `[RESULT] ok: ...`・終了コード `0` |

---

## 1. 基本サンプル（`examples/` 直下および `examples/franka/`）

| TOML | 入力形式 | 出力先 (`output_dir`) | 最終結合ファイル | 特徴・学べること |
|---|---|---|---|---|
| `examples\arm_gripper.toml` | URDF + URDF | `examples\out\arm_gripper\` | `arm_with_two_finger.usda` | 最小構成の 1 段結合。ゴーストリンク `flange` への結合と mimic 二本指グリッパ |
| `examples\multi_arm.toml` | URDF + URDF | `examples\out\multi_arm\` | `robot.usda`（途中生成 `arm_unit.usda`） | 2 段結合。`arm_unit.usda` に `EndEffector` VariantSet（`TwoFinger` / `Rod` / `None`）を作り、`robot.usda` で 4 本のアームごとに別 Variant を選択 |
| `examples\franka\franka.toml` | URDF + URDF | `examples\out\franka\` | `panda_with_hand.usda` | Isaac Sim 同梱の Franka Panda アームと `panda_hand.urdf` の結合。`ros_package_paths` と `offset_rpy`（Z 軸 -45 度）の使用例 |

### 実行例（GUI で開く）

```powershell
# 最小構成（アーム + グリッパ）を GUI で開く
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\arm_gripper.toml --gui

# 4 本アームロボット全体を GUI で開く
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --gui

# 4 本アームの途中結合ファイル（arm_unit.usda のみ）を GUI で開く
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --gui --target arm_unit.usda

# Franka Panda + Panda Hand を GUI で開く
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\franka\franka.toml --gui
```

### 実行例（Headless で `.usda` のみ高速生成）

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --build-only
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\franka\franka.toml --build-only
```

---

## 2. 公式アセット再現サンプル（`examples/official_assets/`）

Isaac Sim 6.2.0 同梱または NVIDIA 公式公開のロボットアセットを、部品単位に分離した状態から本パイプラインで再構成するサンプル群です。これらは `tests_kit\test_official_motion_match.py` の公式アセット動作一致テストの入力としても使用されます。

| フォルダ / TOML | 入力形式 | 出力先 (`output_dir`) | 最終結合ファイル | VariantSet / 構成の特徴 |
|---|---|---|---|---|
| `examples\official_assets\ur10e\ur10e.toml` | USD + USD | `examples\out\official_assets\ur10e\` | `ur10e_assembled.usda` | UR10e ベース USD に Robotiq 2F-85 / 2F-140（閉ループ 4 節リンク）を `Gripper` VariantSet（`Robotiq_2F_85` / `Robotiq_2F_140` / `None`）で切替結合 |
| `examples\official_assets\g1\g1.toml` | USD + USD | `examples\out\official_assets\g1\` | `g1_assembled.usda` | Unitree G1 29-DOF 浮遊ベース本体に `left_hand` / `right_hand`（`ThreeFinger` / `Inspire` / `None`）および `Thor`（`Attached` / `None`、剛体なし複合バックパック）を結合 |
| `examples\official_assets\kaya\kaya.toml` | URDF + URDF | `examples\out\official_assets\kaya\` | `kaya.usda` | NVIDIA Kaya（`fix_base = false`）。同一の `kaya_wheel_unit.urdf` を 120 度間隔で 3 回結合（33 DOF / 45 剛体） |
| `examples\official_assets\carter\carter.toml` | URDF + URDF | `examples\out\official_assets\carter\` | `carter.usda` | NVIDIA Carter v1（`fix_base = false`）。同一の `carter_wheel.urdf` を左右対称（右輪 `Rz(pi)`）に 2 回結合し、後輪キャスター `carter_caster.urdf` を追加 |
| `examples\official_assets\cobotta_pro_900\cobotta_pro_900.toml` | URDF + URDF | `examples\out\official_assets\cobotta_pro_900\` | `cobotta_pro_900.usda` | Denso Cobotta Pro 900（6-DOF）に OnRobot RG6（5 mimic 関節）を `Gripper` VariantSet（`OnRobot_RG6` / `None`）で結合 |
| `examples\official_assets\openarm\openarm_bimanual.toml` | USD + USD | `examples\out\official_assets\openarm\` | `openarm_bimanual_assembled.usda` | OpenArm 双腕台座（`openarm_body_base.usda`）に同一の公式片腕 `openarm_unimanual.usd` を左右対称姿勢で 2 本結合（`LeftArm` / `RightArm` VariantSet：`Attached` / `None`） |

> **OpenArm サンプルの前提ファイル**: `examples\official_assets\openarm\openarm_bimanual.toml` は、`examples\official_assets\openarm\_s3_cache\Isaac\Robots\OpenArm\openarm_unimanual\openarm_unimanual.usd`（および `openarm_bimanual` の台座ベース `openarm_body_base.usda` 経由の参照先）を使用します。`_s3_cache\` は `.gitignore` 対象のため、ローカルに配置済みの環境で実行してください。

### 実行例（GUI で開く）

```powershell
# UR10e + Robotiq グリッパ（Property パネルで Gripper VariantSet を切り替え可能）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\ur10e\ur10e.toml --gui

# Unitree G1（left_hand / right_hand / Thor の 3 つの VariantSet を切り替え可能）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\g1\g1.toml --gui

# NVIDIA Kaya（3 輪ホロノミック移動ロボット）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\kaya\kaya.toml --gui

# NVIDIA Carter v1（差動二輪移動ロボット）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\carter\carter.toml --gui

# Denso Cobotta Pro 900 + OnRobot RG6
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\cobotta_pro_900\cobotta_pro_900.toml --gui

# OpenArm Bimanual（双腕ロボット）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\openarm\openarm_bimanual.toml --gui
```

### 実行例（Headless で `.usda` のみ高速生成）

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\ur10e\ur10e.toml --build-only
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\g1\g1.toml --build-only
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\official_assets\openarm\openarm_bimanual.toml --build-only
```

---

## 3. 補助アセットフォルダ

| フォルダ | 内容 |
|---|---|
| `examples\urdf\` | 基本サンプルおよび `tests_kit\run_tests.py` が使用する合成 URDF 群（`arm.urdf`, `arm_world.urdf`, `gripper.urdf`, `gripper_damp.urdf`, `gripper_ghost.urdf`, `multi_base.urdf`, `tool.urdf`） |
| `examples\meshes\` | `examples\urdf\` が参照する STL メッシュ群（`base.stl`, `link.stl`, `palm.stl`, `finger.stl`, `tool.stl`） |
| `examples\out\` | 各サンプル TOML を実行したときの既定の出力フォルダ（`.gitignore` 対象） |
