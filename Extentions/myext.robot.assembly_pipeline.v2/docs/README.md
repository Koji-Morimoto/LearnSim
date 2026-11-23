# myext.robot.assembly_pipeline.v2

Isaac Sim 6.2.0（URDF Importer 3.11.12 / Robot Assembler 3.3.9）向けの **URDF / USD ロボット結合パイプライン** です。

TOML 仕様ファイル 1 つから、URDF（`[[urdf_to_usda]]`）および既存 USD アセット（`[[usd_to_usda]]`）を正規化した `.usda` に変換し、それらをリンク単位で結合した **結合ファイル**（`[[join_file]]`）を冪等（何度実行しても同じ結果）に生成します。生成した結合ファイルは別の結合の親にも子にもでき、`VariantSet` によるエンドエフェクタやアームの切り替えに対応します。

---

## 目次

1. [すぐ試す（4 つの実行モード）](#すぐ試す4-つの実行モード)
2. [ログの読み方：意図的に出るエラーログと本当の異常の見分け方](#ログの読み方意図的に出るエラーログと本当の異常の見分け方)
3. [終了コード一覧](#終了コード一覧)
4. [同梱サンプル一覧](#同梱サンプル一覧)
5. [TOML 仕様ファイルの書き方](#toml-仕様ファイルの書き方)
6. [事前チェックと修正規則（R01〜R14）](#事前チェックと修正規則r01r14)
7. [Python API](#python-api)
8. [テストスイート](#テストスイート)
9. [ファイル構成](#ファイル構成)

---

## すぐ試す（4 つの実行モード）

すべてのコマンドは、拡張ルートフォルダをカレントディレクトリにして `C:\isaacsim-6.2.0\tools\myext\run_sim.bat` 経由で実行します。`run_sim.bat` はコンソールを UTF-8 に設定したうえで Isaac Sim の Python 環境を起動します。

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline.v2
```

### モード 1: `--gui`（Isaac Sim GUI を開いてロボットを確認・操作する）

組み立てたロボットを Isaac Sim の 3D ビューポートで実際に見て、Play・VariantSet 切り替え・関節操作を行いたい場合は、最初にこのコマンドを実行します。

- **画面の挙動**: **Isaac Sim GUI ウィンドウが開き、ロボットを配置した状態で開いたまま維持されます**（ウィンドウを閉じるか端末で `Ctrl+C` を押すまで終了しません）。
- **出力物と保存先**: TOML の `output_dir`（`examples\multi_arm.toml` では `examples\out\multi_arm\`）に各部品 `.usda` と結合ファイル（`arm_unit.usda`, `robot.usda`）を保存します。`report.md` / `report.json` は出力しません。
- **成功の合図**: 端末に `[GUI] 組み立て済みロボットを Isaac Sim GUI に配置しました` とスポーン先 prim パス（`/World/robot`）・コンポーネント一覧・VariantSet 一覧が表示され、ビューポートにロボットが表示されます。公式比較を実行しないため、PhysX の `closed articulation` エラーログは出力されません。
- **本当のエラー**: 終了コード `0` 以外で終了した場合、または `[SPEC ERROR]` / `[STOPPED]` / Python の Traceback が出力された場合。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --gui
```

途中の結合ファイル（例: アーム単体 `arm_unit.usda`）だけを配置して確認したい場合は、`--target <usdaファイル名>` を追加します。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --gui --target arm_unit.usda
```

### モード 2: `--check`（仕様の静的検査だけ行う）

TOML の記述ミス・存在しないリンク名・無効な `VariantSet` 選択を数秒で検査します。

- **画面の挙動**: **Isaac Sim を起動しません**。端末に仕様の読み取り結果（作る `.usda` 一覧と結合一覧）を表示して数秒で自動終了します。
- **出力物と保存先**: ファイルは一切生成・変更しません。
- **成功の合図**: 最終行に `[CHECK] 仕様に誤りは無い` と表示され、終了コード `0` で終了します。
- **本当のエラー**: `[SPEC ERROR]` が出力され、終了コード `4` で終了します。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --check
```

### モード 3: `--build-only`（Headless で `.usda` の生成だけ行う）

動的検証や公式比較を省き、`.usda` ファイルの生成だけを高速に行います。

- **画面の挙動**: **Headless（GUI なし）で起動し、`.usda` 生成が終わると自動終了します**。
- **出力物と保存先**: TOML の `output_dir`（`examples\multi_arm.toml` では `examples\out\multi_arm\`）に各部品 `.usda` と結合ファイルを保存します。`report.md` / `report.json` は出力しません。
- **成功の合図**: `[BUILD] .usda の非破壊生成が完了しました: <output_dir>` と生成ファイル一覧が表示され、終了コード `0` で終了します。公式比較を行わないため、意図的な PhysX エラーログは出力されません。
- **本当のエラー**: 終了コード `0` 以外、または `[SPEC ERROR]` / `[STOPPED]` / Traceback が出力された場合。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --build-only
```

### モード 4: オプションなし（Headless フル検証 + レポート出力）

`.usda` の生成に加えて、公式 `RobotAssembler` との比較（`compare_official = true` の場合）・PhysX 物理シミュレーション・`RobotPoser` 追従確認・解析的順運動学との突き合わせを自動実行し、検証レポートを書き出します。

- **画面の挙動**: **Headless（GUI なし）で起動し、全検証とレポート出力が終わると自動終了します**。ロボットを画面で操作したい場合はモード 1（`--gui`）を使用してください。
- **出力物と保存先**: TOML の `output_dir`（`examples\multi_arm.toml` では `examples\out\multi_arm\`）にすべての `.usda` と検証レポート `report.md` / `report.json` を保存します（2 回目以降の実行では前回結果を `report_previous.json` に退避します）。
- **成功の合図**: 最終行付近に `[RESULT] ok: <output_dir>\report.md` が表示され、終了コード `0` で終了します。
- **本当のエラー**: `[RESULT] problems: <output_dir>\report.md`（終了コード `1`）、`[STOPPED]`（終了コード `2`）、Traceback（終了コード `3`）、`[SPEC ERROR]`（終了コード `4`）。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml
```

---

## ログの読み方：意図的に出るエラーログと本当の異常の見分け方

モード 4（オプションなしの Headless フル検証）では、TOML の既定設定 `compare_official = true` が有効なときに **公式 `RobotAssembler` で同じ結合をそのまま実行し、公式ツールの既知不具合が再現されることを確認してから、本パイプラインの修正後出力で解消していることを証明する** という比較フェーズを実行します。

この比較フェーズの実行中には、公式 `RobotAssembler` が生成した不具合ありのステージを PhysX に読み込ませるため、端末ログに次のようなエラーメッセージが大量に出力されます。

```text
[Error] [omni.physx.plugin] RigidBody (/World/arm_unit/Geometry/base_link/link1/link2) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints: ...
```

### 意図的な比較用ログと本当の異常を見分ける 3 つの基準

1. **コンソールの境界バナーを確認する**
   - 次の開始バナーと終了バナーのあいだに出る PhysX / `closed articulation` エラーは、公式 `RobotAssembler` の不具合再現による **意図的な比較用ログ** です。
     - 開始: `[INFO] === 公式 RobotAssembler 比較フェーズ開始（この区間の PhysX / closed articulation エラーは公式不具合の意図的な再現ログです） ===`
     - 終了: `[INFO] === 公式 RobotAssembler 比較フェーズ終了（ここ以降のエラーは意図しない異常です） ===`
   - 終了バナー以降（本パイプラインが生成した `.usda` の検証中）に出た PhysX エラーは、意図しない異常として確認が必要です。
2. **最終結果行と終了コードを確認する**
   - 端末の末尾に `[RESULT] ok: <output_dir>\report.md` が出力され、終了コードが `0` であれば、本パイプラインが生成した `.usda` の物理・RobotPoser・順運動学検証はすべて正常に合格しています。
   - `[RESULT] problems: ...`（終了コード `1`）が出た場合は本当の検証失敗です。
3. **`<output_dir>\report.md` の判定列を確認する**
   - 規則 R01〜R14 の判定が `ok` / `fixed` / `upstream_fixed` / `not_measured` のみで、順運動学チェックにエラーが無ければ正常です。
   - `still_broken`（修正後も症状が残存）または `unexpected`（事前予測外の症状が発生）、あるいは順運動学の誤差超過・DOF 不一致が記録されている場合は本当の異常です。

> **補足**: `--gui` モードと `--build-only` モードでは公式比較フェーズを実行しないため、意図的な PhysX エラーログは一切出力されません。フル検証でも公式比較を省きたい場合は、TOML 先頭に `compare_official = false` を記述します。

---

## 終了コード一覧

`run_pipeline.py` の終了コードは全モード共通です。

| 終了コード | 意味 | 対処 |
|---|---|---|
| `0` | 正常終了 | 問題なし |
| `1` | 検証で問題あり（`[RESULT] problems`） | `<output_dir>\report.md` の `still_broken` / `unexpected` / 順運動学誤差を確認 |
| `2` | 事前チェックの規則で停止（`[STOPPED]`） | `on_violation = "stop"` を指定した規則に該当。`<output_dir>\report.md` を確認 |
| `3` | 実行時例外 | 端末の Python Traceback を確認 |
| `4` | TOML 仕様または CLI 引数の誤り（`[SPEC ERROR]`） | エラーメッセージに従い TOML またはコマンド引数を修正 |

> **補足**: `run_sim.bat`（内部の `python.bat`）は Python 側の非ゼロ終了コード（`1`〜`4`）を一律 `1` に丸めて返します。終了理由の区分は端末の `[RESULT]` / `[STOPPED]` / `[SPEC ERROR]` タグで確認できます。

---

## 同梱サンプル一覧

すべてのサンプルの詳しい構成・特徴・コピーして使える `--gui` / `--build-only` コマンドは [examples/README.md](../examples/README.md) にまとめています。

| TOML パス | 入力形式 | ロボット構成 | 主な確認要素 |
|---|---|---|---|
| `examples\arm_gripper.toml` | URDF + URDF | 2-DOF アーム + mimic 二本指グリッパ | 最小構成の 1 段結合・ゴーストリンク結合・mimic 追従 |
| `examples\multi_arm.toml` | URDF + URDF | ベース + 4 本アーム + `EndEffector`（`TwoFinger` / `Rod` / `None`） | 2 段結合・`VariantSet` 定義と子ごとの選択切替 |
| `examples\franka\franka.toml` | URDF + URDF | Franka Panda 7-DOF アーム + Panda Hand | Isaac Sim 同梱 URDF・ROS パッケージ解決・`offset_rpy` 回転結合 |
| `examples\official_assets\ur10e\ur10e.toml` | USD + USD | UR10e + Robotiq 2F-85 / 2F-140（`Gripper` VariantSet） | 既存 USD 同士の結合・閉ループ 4 節リンクグリッパ |
| `examples\official_assets\g1\g1.toml` | USD + USD | Unitree G1 29-DOF + `left_hand` / `right_hand` / `Thor` | 複数 `VariantSet`・浮遊ベース・剛体なし複合アタッチメント |
| `examples\official_assets\kaya\kaya.toml` | URDF + URDF | NVIDIA Kaya（`kaya_base` + 同一 `kaya_wheel_unit` × 3） | `fix_base = false`（浮遊ベース）・同一子部品の 3 重配置 |
| `examples\official_assets\carter\carter.toml` | URDF + URDF | NVIDIA Carter v1（シャーシ + 同一車輪 × 2 + 後輪キャスター） | `fix_base = false`・差動二輪の対称回転配置（`Rz(pi)`） |
| `examples\official_assets\cobotta_pro_900\cobotta_pro_900.toml` | URDF + URDF | Denso Cobotta Pro 900 + OnRobot RG6（`Gripper` VariantSet） | 6-DOF アーム + 5 mimic 関節グリッパ |
| `examples\official_assets\openarm\openarm_bimanual.toml` | USD + USD | OpenArm Bimanual（台座 + 同一 `openarm_unimanual.usd` × 2） | 同一片腕 USD の左右対称配置・`LeftArm` / `RightArm` VariantSet |

---

## TOML 仕様ファイルの書き方

仕様ファイルのパスはすべて TOML ファイルのあるフォルダからの相対パス（または絶対パス）で記述します。`.usda` 名はフォルダを含まないファイル名（例: `"arm.usda"`）で統一し、`output_dir` 配下に生成されます。

### トップレベル設定

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `output_dir` | 必須 | — | `.usda` および検証レポートの出力先フォルダ |
| `compare_official` | 任意 | `true` | フル検証時に公式 `RobotAssembler` との比較フェーズを実行するか |
| `check_kinematics` | 任意 | `true` | フル検証時に解析的順運動学と PhysX リンク姿勢の突き合わせを実行するか |

### `[[urdf_to_usda]]`（URDF から部品 `.usda` を生成）

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `urdf` | 必須 | — | 入力 URDF ファイルのパス |
| `usda` | 必須 | — | 出力する部品 `.usda` のファイル名（例: `"arm.usda"`） |
| `fix_base` | 任意 | `None`（Importer 既定） | `false` で浮遊ベース（移動ロボット・ヒューマノイド等）、`true` でワールド固定 |
| `ros_package_paths` | 任意 | `[]` | `package://` の解決先（`[{ name = "pkg", path = "..." }]`） |
| `physics_variant` | 任意 | `"auto"` | 使用する Physics variant（`"auto"` / `"physx"` / `"physics"` / `"mujoco"`） |
| `collision_from_visuals` | 任意 | `None` | `None` なら URDF に collision が無い場合のみ visual から生成 |
| `collision_type` | 任意 | `"Convex Hull"` | `collision_from_visuals` 有効時の近似形状 |
| `joint_drive_stiffness` | 任意 | `None` | URDF Importer の `override_joint_stiffness` |
| `joint_drive_damping` | 任意 | `None` | URDF Importer の `override_joint_damping` |

### `[[usd_to_usda]]`（既存 USD アセットから正規化部品 `.usda` を生成）

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `usd` | 必須 | — | 入力 USD ファイルのパスまたは URI（`"://"` を含むパス） |
| `usda` | 必須 | — | 出力する正規化 `.usda` のファイル名（例: `"ur10e_base.usda"`） |
| `prim_path` | 任意 | `None`（`defaultPrim`） | 参照する prim パス（`"/"` で始める） |
| `variant_selections` | 任意 | `{}` | 入力 USD 側で事前選択する VariantSet（例: `{ Physics = "PhysX" }`） |
| `physics_variant` | 任意 | `"auto"` | 使用する Physics variant 名 |

### `[[join_file]]` と `[[join_file.join]]`（結合ファイルの定義）

`[[join_file]]` で 1 つの結合ファイル（`usda`, `parent_usda`, 任意で `variant_defaults`）を定義し、配下の `[[join_file.join]]` で親リンクと子リンクの結合を列挙します。

| キー | 対象 | 必須 | 既定値 | 説明 |
|---|---|---|---|---|
| `usda` | `[[join_file]]` | 必須 | — | 出力する結合ファイルのファイル名（例: `"robot.usda"`） |
| `parent_usda` | `[[join_file]]` | 必須 | — | 親となる `.usda` のファイル名 |
| `variant_defaults` | `[[join_file]]` | 任意 | `{}` | VariantSet ごとの既定選択を明示指定（`"None"` 指定可） |
| `name` | `[[join_file.join]]` | 必須 | — | 結合名（結合ファイル内で一意・英数字と `_`） |
| `child_usda` | `[[join_file.join]]` | 必須 | — | 子となる `.usda` のファイル名 |
| `parent_link` | `[[join_file.join]]` | 必須 | — | 親側の結合先リンク名（多段結合では `"arm1/flange"` 形式も可） |
| `child_link` | `[[join_file.join]]` | 必須 | — | 子側の基準リンク名 |
| `place_child_under` | `[[join_file.join]]` | 任意 | `parent_link` の prim | 子を Reference 配置する親 prim（リンク名、`"."`、または `"/..."`） |
| `child_prim_name` | `[[join_file.join]]` | 任意 | `name` | 子を配置する prim 名（同一 VariantSet で揃えるとパスが一致） |
| `offset_xyz` | `[[join_file.join]]` | 任意 | `[0.0, 0.0, 0.0]` | `parent_link` 座標系での `child_link` 位置オフセット（m） |
| `offset_rpy` | `[[join_file.join]]` | 任意 | `[0.0, 0.0, 0.0]` | `parent_link` 座標系での `child_link` 姿勢オフセット（rad、固定軸 XYZ） |
| `variant_set` / `variant` | `[[join_file.join]]` | 任意（対で指定） | `None` | 結合を VariantSet の選択肢として登録（`"None"` 選択肢は自動追加） |
| `child_variant_selection` | `[[join_file.join]]` | 任意 | `{}` | 子の `.usda` が持つ VariantSet の選択（例: `{ EndEffector = "Rod" }`） |

### `[checks]` と `[probe]`（規則制御と測定条件）

- `[checks]`: `on_violation`（`"continue"` / `"stop"`）、`fix`（`"apply"` / `"skip"`）、および `[checks.rules.<R01..R14>]` による規則別の個別設定を指定できます。
- `[probe]`: `steps`, `drive_stiffness`, `drive_damping`, `revolute_target`, `prismatic_ratio`, `mimic_steps`, `angle_tol_deg`, `length_tol_m` 等の物理検証パラメータを指定できます。

---

## 事前チェックと修正規則（R01〜R14）

`run_pipeline` は規則 `R01`〜`R14` について「事前チェックによる予測」と「実測による答え合わせ」を行います。

| 規則 ID | 種別 | 検出内容・修正対象 |
|---|---|---|
| `R01` | URDF | ルートリンクがゴーストリンク（形状・慣性なし）のためワールド固定 joint 名が `root_joint` にならない問題 |
| `R02` | 結合 | 子のワールド固定 joint が無効化されず親の関節が動かない問題 |
| `R03` | URDF / USD | mimic follower 関節に非ゼロの drive ゲインが入り leader が目標角に届かない問題 |
| `R04` | URDF | 末端リンク prim の座標系がメッシュ原点へずれる問題 |
| `R05` | 結合 | `instanceable` な prim 配下への結合が失敗する問題 |
| `R06` | URDF | `Physics` VariantSet が未選択のまま出力され剛体・関節が存在しない問題 |
| `R07` | 結合 | 子の `ArticulationRootAPI`（Newton API 含む）が残り二重 Articulation になる問題 |
| `R08` | 結合 | 子の基準リンクに剛体が無く固定関節が張れない問題 |
| `R09` | 結合 | 親剛体配下への子配置で閉ループ（closed articulation）になる問題 |
| `R10` | URDF | 衝突メッシュが通常表示されてしまう問題 |
| `R11` | 結合 | 結合した親リンクと子リンク間に衝突除外（FilteredPairs）が無い問題 |
| `R12` | 結合 | `RobotPoser` のテレポートで子ロボットが置いて行かれる問題 |
| `R13` | URDF | 再インポート時に `_1` 等の別名フォルダが増殖する問題 |
| `R14` | 結合 | 再結合時に固定関節 prim が重複増加する問題 |

---

## Python API

Isaac Sim（`SimulationApp`）起動済みの Python スクリプトから直接呼び出せます。`SimulationApp` の起動には `C:\isaacsim-6.2.0\tools\myext\sim_app_launcher.py` の `create_simulation_app` を使用します。

```python
from myext.robot.assembly_pipeline_v2 import (
    AssembledRobotDescriptor,
    PipelineStopped,
    build_assembly,
    discover_components,
    inspect_assembled_robot,
    load_spec,
    run_pipeline,
    spawn_assembled_robot,
)

# 1. 非破壊ビルド（アクティブな USD Stage や Timeline をリセットせずに .usda を生成）
spec = load_spec("examples/multi_arm.toml")
built = build_assembly(spec)
desc: AssembledRobotDescriptor = built["descriptors"]["robot.usda"]
print(desc.prim_path)                     # "/robot"
print(desc.components)                    # 構成要素名 -> prim パス
print(desc.to_articulation_source_dict()) # {"root_prim_path": ..., "components": ...}

# 2. 組み立て済み .usda を任意ステージへ Reference 配置してディスクリプタを取得
spawn_desc = spawn_assembled_robot(stage, built["outputs"]["robot.usda"], "/World/robot")
all_comps = discover_components(stage, "/World/robot", include_inactive_variants=True)

# 3. 事前チェック・公式比較・動的物理プローブ・順運動学検証・レポート出力を含むフル実行
result = run_pipeline(spec)
print(result.report["status"], result.outputs["robot.usda"])
```

| API | 役割 |
|---|---|
| `load_spec(path)` | TOML 仕様を読み込み `PipelineSpec` を返す（Isaac Sim 不要） |
| `check_links(spec)` / `describe(spec)` | リンク・VariantSet の静的整合性検査と参照関係サマリ生成（Isaac Sim 不要） |
| `build_assembly(spec, check_idempotency=False)` | 既存ステージ・Timeline を壊さずに `.usda` 生成と `AssembledRobotDescriptor` 抽出を実行 |
| `run_pipeline(spec)` | `build_assembly` に加え、公式比較・動的物理プローブ・順運動学検証・`report.md`/`report.json` 出力を実行 |
| `spawn_assembled_robot(stage, usda_path, prim_path)` | 生成した `.usda` をステージに Reference 配置し `AssembledRobotDescriptor` を返す |
| `inspect_assembled_robot(stage, prim_path)` | ステージ上のロボット prim から構成要素・Articulation Root・リンク・関節・VariantSet を抽出 |
| `discover_components(stage, robot_prim_path, include_inactive_variants=True)` | `IsaacRobotAPI` と `customLayerData` から構成要素名 → prim パスの辞書を返す |
| `AssembledRobotDescriptor` | 組み立て済みロボットの中立記述子（`to_articulation_source_dict()` で外部 Articulation リーダーへ連携可能） |

---

## テストスイート

テスト方針・各テストの検証内容と合格基準・新しいテストの追加手順は [tests_kit/README.md](../tests_kit/README.md) にまとめています。

1. **パイプライン統合テスト（全 18 テスト・Headless 自動終了）**
   - 出力: `tests_kit\last_run.log`, `tests_kit\last_run.json`
   - 成功の合図: `[SUMMARY] passed 18 / failed 0` および `== exit=0`

```powershell
powershell -ExecutionPolicy Bypass -File run_tests.ps1
```

2. **公式アセット定量 PhysX 動作一致テスト（全 6 テスト・Headless 自動終了）**
   - 出力: `tests_kit\last_official_motion_run.json`
   - 成功の合図: `[SUMMARY] official_motion passed 6 / failed 0` および終了コード `0`

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py
```

---

## ファイル構成

```text
myext.robot.assembly_pipeline.v2/
├── config/extension.toml                      # Kit 拡張マニフェスト
├── docs/README.md                             # 本ドキュメント（使い方・仕様・API）
├── examples/
│   ├── README.md                              # 同梱サンプル TOML と実行手順ガイド
│   ├── arm_gripper.toml / multi_arm.toml      # 基本・多段 VariantSet サンプル
│   ├── franka/                                # Franka Panda サンプル
│   ├── official_assets/                       # Isaac Sim 公式アセット再現サンプル 6 種
│   │   ├── ur10e/                             # UR10e + Robotiq 2F-85 / 2F-140
│   │   ├── g1/                                # Unitree G1 29-DOF + Hands + Thor
│   │   ├── kaya/                              # NVIDIA Kaya (3 輪・浮遊ベース)
│   │   ├── carter/                            # NVIDIA Carter v1 (差動二輪・浮遊ベース)
│   │   ├── cobotta_pro_900/                   # Denso Cobotta Pro 900 + OnRobot RG6
│   │   └── openarm/                           # OpenArm Bimanual (双腕 USD)
│   ├── urdf/ / meshes/                        # テスト・サンプル用 URDF と STL メッシュ
├── myext/robot/assembly_pipeline_v2/
│   ├── __init__.py                            # 公開 API の再エクスポートと遅延読み込み
│   ├── extension.py                           # Kit 拡張エントリポイント (omni.ext.IExt)
│   ├── kinds.py                               # Literal 型定義と実行時バリデータ
│   ├── spec.py                                # TOML 仕様読み込みとデータクラス定義
│   ├── spec_check.py                          # リンク・VariantSet の静的検査と仕様サマリ生成
│   ├── urdf_model.py                          # URDF パーサ・ゴーストリンク判定・解析的 FK
│   ├── usd_util.py                            # USD 行列変換・剛体/関節探索・API 判定
│   ├── sources.py                             # 多段結合の階層マップ (Owner / UsdaInfo) と複合 FK
│   ├── descriptor.py                          # AssembledRobotDescriptor / discover_components / spawn
│   ├── part_import.py                         # ハッシュキャッシュ付き URDF/USD 部品インポートと正規化層
│   ├── compose.py                             # 7 段階の冪等な USD 結合ファイル生成
│   ├── runtime.py                             # Kit フレーム更新・Timeline 制御・プローブ用ステージ管理
│   ├── official.py                            # 比較用の公式 RobotAssembler 再現ステージ構築
│   ├── probes.py                              # 静的部品検査・動的 PhysX / RobotPoser 測定
│   ├── kinematics_check.py                    # 解析的順運動学と PhysX 剛体姿勢の突き合わせ
│   ├── rules.py                               # 規則 R01〜R14 の事前判定と事後評価
│   ├── report.py                              # report.md / report.json 書き出しと前回結果比較
│   └── pipeline.py                            # build_assembly / run_pipeline オーケストレータ
├── run_pipeline.py                            # CLI エントリポイント (--gui / --check / --build-only / フル検証)
├── run_tests.ps1                              # 統合テストスイート実行ラッパー
└── tests_kit/
    ├── README.md                              # テスト方針・テスト一覧・追加手順
    ├── run_tests.py                           # 統合テストスイート (全 18 テスト)
    └── test_official_motion_match.py          # 公式アセット定量 PhysX 動作一致テスト (全 6 テスト)
```
