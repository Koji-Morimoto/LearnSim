# myext.robot.assembly_pipeline

Isaac Sim 6.2.0（URDF Importer 3.11.12 / Robot Assembler 3.3.9）向けの **URDF / USD ロボット結合パイプライン** です。

TOML 仕様ファイル 1 つから、URDF（`[[urdf_to_usda]]`）および既存 USD アセット（`[[usd_to_usda]]`）を **部品 usda** に変換し、それらをリンク単位で結合した **結合ファイル**（`[[join_file]]`）を冪等（何度実行しても同じ結果）に生成します。生成した結合ファイルは別の結合の親にも子にもでき、`VariantSet` によるエンドエフェクタやアームの切り替えに対応します。

- **本ドキュメント（README.md）**: 概要の把握、コマンド実行、TOML を手作業で書くためのガイド。
- **内部仕様書（[SPECIFICATION.md](./SPECIFICATION.md)）**: 全用語定義、処理フロー、manifest ファイルのハッシュ変更検知・キャッシュ、USD レイヤー構造・結合の 7 段階書き込み、規則 R01〜R14 の判定ロジック、動的プローブ・順運動学の計算式など、内部の仕組みを確認したいときに参照します。

---

## 目次

1. [基本用語](#基本用語)
2. [すぐ試す（4 つの実行モード）](#すぐ試す4-つの実行モード)
3. [ログの読み方：意図的に出るエラーログと本当の異常の見分け方](#ログの読み方意図的に出るエラーログと本当の異常の見分け方)
4. [終了コード一覧](#終了コード一覧)
5. [TOML 仕様ファイルの書き方](#toml-仕様ファイルの書き方)
6. [同梱サンプル一覧](#同梱サンプル一覧)
7. [事前チェックと修正規則（R01〜R14）の概要](#事前チェックと修正規則r01r14の概要)
8. [Python API](#python-api)
9. [テストスイート](#テストスイート)
10. [ファイル構成](#ファイル構成)

---

## 基本用語

本拡張では次の言葉を記載の意味に統一して使います（剛体・ワールド固定 joint・ゴーストリンク・manifest ファイルなどの物理・内部用語を含む完全版は [SPECIFICATION.md の用語集](./SPECIFICATION.md#用語集) を参照）。

| 言葉 | 意味 |
|---|---|
| usda | `defaultPrim` を持つ USD ファイル。部品 usda または結合ファイルを指す。TOML 内でもログでも常にファイル名（`arm.usda` のように拡張子 `.usda` まで）で指す |
| 部品 usda | `[[urdf_to_usda]]` または `[[usd_to_usda]]` から生成する単一部品の usda。生成物 USD や既存 USD を直接編集せず、Reference と `over` だけで構成する |
| 結合 | 子の usda を、親の usda のリンクに固定する 1 回の操作（USD への具体的な 7 段階の書き込み内容は [SPECIFICATION.md の結合の中身](./SPECIFICATION.md#結合の中身7-つの書き込み) を参照） |
| 結合ファイル | 親の usda に 1 回以上の結合を書き込んだ、新しい usda（`[[join_file]]` の出力）。次の結合の親にも子にもできる |
| 一番上の prim | usda の `defaultPrim`。名前は usda のファイル名の `.usda` より前（`robot.usda` なら `/robot`） |
| 公式のまま | URDF Importer の出力を Isaac Sim 公式の `RobotAssembler` でそのまま結合した場合。比較用にパイプライン内部で再現して実測する |
| 修正後 | 本パイプラインが出力した部品 usda・結合ファイル |
| 修正 | 既知の不具合を回避するために本パイプラインが適用する補正（規則ごとに `[checks.rules.<規則>]` の `fix = "skip"` で外せる） |
| 非破壊ビルド | `build_assembly` が行う `.usda` 生成処理。アクティブな USD Stage や Timeline を変更しないため `--build-only` や `--gui` から安全に使える |
| 構成要素 | 組み立て済みロボットを構成する部品単位（`AssembledRobotDescriptor.components` のキー）。ルート prim 自身および結合した子 prim に割り当てられる |

---

## すぐ試す（4 つの実行モード）

すべてのコマンドは、拡張ルートフォルダをカレントディレクトリにして `C:\isaacsim-6.2.0\tools\myext\run_sim.bat` 経由で実行します。`run_sim.bat` はコンソールを UTF-8 に設定したうえで Isaac Sim の Python 環境を起動します。

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline
```

### モード 1: `--gui`（Isaac Sim GUI を開いてロボットを確認・操作する）

組み立てたロボットを Isaac Sim の 3D ビューポートで実際に見て、Play・VariantSet 切り替え・関節操作を行いたい場合は、最初にこのコマンドを実行します。

- **画面の挙動**: **Isaac Sim GUI ウィンドウが開き、ロボットを配置した状態で開いたまま維持されます**（ウィンドウを閉じるか端末で `Ctrl+C` を押すまで終了しません）。
- **出力物と保存先**: TOML の `output_dir`（`examples\multi_arm.toml` では `examples\out\multi_arm\`）に各部品 usda と結合ファイル（`arm_unit.usda`, `robot.usda`）を保存します。`report.md` / `report.json` は出力しません。
- **成功の合図**: 端末に `[GUI] 組み立て済みロボットを Isaac Sim GUI に配置しました` とスポーン先 prim パス（`/World/robot`）・構成要素一覧・VariantSet 一覧が表示され、ビューポートにロボットが表示されます。公式比較を実行しないため、PhysX の `closed articulation` エラーログは出力されません。
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

- **画面の挙動**: **Isaac Sim を起動しません**。端末に仕様の読み取り結果（作る usda 一覧と結合一覧）を表示して数秒で自動終了します。
- **出力物と保存先**: ファイルは一切生成・変更しません。
- **成功の合図**: 最終行に `[CHECK] 仕様に誤りは無い` と表示され、終了コード `0` で終了します。
- **本当のエラー**: `[SPEC ERROR]` が出力され、終了コード `4` で終了します。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --check
```

### モード 3: `--build-only`（Headless で `.usda` の生成だけ行う）

動的検証や公式比較を省き、部品 usda と結合ファイルの生成だけを高速に行います。

- **画面の挙動**: **Headless（GUI なし）で起動し、`.usda` 生成が終わると自動終了します**。
- **出力物と保存先**: TOML の `output_dir` に各部品 usda と結合ファイルを保存します。`report.md` / `report.json` は出力しません。
- **成功の合図**: `[BUILD] .usda の非破壊生成が完了しました: <output_dir>` と生成ファイル一覧が表示され、終了コード `0` で終了します。公式比較を行わないため、意図的な PhysX エラーログは出力されません。
- **本当のエラー**: 終了コード `0` 以外、または `[SPEC ERROR]` / `[STOPPED]` / Traceback が出力された場合。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml --build-only
```

### モード 4: オプションなし（Headless フル検証 + レポート出力）

部品 usda と結合ファイルの生成に加えて、公式のまま（`RobotAssembler`）との比較（`compare_official = true` の場合）・PhysX 物理シミュレーション・`RobotPoser` 追従確認・純粋 USD 順運動学との突き合わせを自動実行し、`report.md` / `report.json` を書き出します。

- **画面の挙動**: **Headless（GUI なし）で起動し、全検証とレポート出力が終わると自動終了します**。ロボットを画面で操作したい場合はモード 1（`--gui`）を使用してください。
- **出力物と保存先**: TOML の `output_dir` にすべての usda と検証レポート `report.md` / `report.json` を保存します（2 回目以降の実行では前回結果を `report_previous.json` に退避します）。
- **成功の合図**: 最終行付近に `[RESULT] ok: <output_dir>\report.md` が表示され、終了コード `0` で終了します。
- **本当のエラー**: `[RESULT] problems: <output_dir>\report.md`（終了コード `1`）、`[STOPPED]`（終了コード `2`）、Traceback（終了コード `3`）、`[SPEC ERROR]`（終了コード `4`）。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py examples\multi_arm.toml
```

---

## ログの読み方：意図的に出るエラーログと本当の異常の見分け方

モード 4（オプションなしの Headless フル検証）では、TOML の既定設定 `compare_official = true` が有効なときに **公式のまま（`RobotAssembler`）で同じ結合を再現して既知不具合の発生を確認し、修正後出力で解消していることを証明する** 比較フェーズを実行します。

この比較フェーズの実行中には、公式 `RobotAssembler` が生成した不具合ありのステージを PhysX に読み込ませるため、端末ログに次のようなエラーメッセージが出力されます。

```text
[Error] [omni.physx.plugin] RigidBody (/World/arm_unit/Geometry/base_link/link1/link2) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints: ...
```

### 意図的な比較用ログと本当の異常を見分ける 3 つの基準

1. **コンソールの境界バナーを確認する**
   - 次の開始バナーと終了バナーのあいだに出る PhysX / `closed articulation` エラーは、公式 `RobotAssembler` の不具合再現による **意図的な比較用ログ** です。
     - 開始: `[INFO] === 公式 RobotAssembler 比較フェーズ開始（この区間の PhysX / closed articulation エラーは公式不具合の意図的な再現ログです） ===`
     - 終了: `[INFO] === 公式 RobotAssembler 比較フェーズ終了（ここ以降のエラーは意図しない異常です） ===`
   - 終了バナー以降（修正後 `.usda` の検証中）に出た PhysX エラーは、意図しない異常として確認が必要です。
2. **最終結果行と終了コードを確認する**
   - 端末の末尾に `[RESULT] ok: <output_dir>\report.md` が出力され、終了コードが `0` であれば、修正後出力の物理・`RobotPoser`・順運動学検証はすべて合格しています。
   - `[RESULT] problems: ...`（終了コード `1`）が出た場合は検証失敗です。
3. **`<output_dir>\report.md` の判定列を確認する**
   - 規則 R01〜R14 の最終判定が `ok` / `fixed` / `upstream_fixed` / `not_measured` のみで、順運動学検証にエラーが無ければ正常です。
   - `still_broken`・`unexpected`、または順運動学の誤差超過・DOF 不一致が記録されている場合は異常です。

> **補足**: `--gui` モードと `--build-only` モードでは公式比較フェーズを実行しないため、意図的な PhysX エラーログは出力されません。フル検証でも公式比較を省きたい場合は、TOML 先頭に `compare_official = false` を記述します。

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

## TOML 仕様ファイルの書き方

TOML 内のパス（`output_dir`, `urdf`, `usd`, `ros_package_paths` の `path`）はすべて **TOML ファイルのあるフォルダからの相対パス**（または絶対パス）で記述します（`usd` に `"://"` を含む URI はそのまま保持します）。

### 参照のしくみ（4 つのファイル名キー）

usda はどこでも **フォルダを含まないファイル名（拡張子 `.usda` まで）** で指します。ファイル名を書くキーは 4 つだけです。

| キー | セクション | 役割 | 書く値 |
|---|---|---|---|
| `usda` | `[[urdf_to_usda]]` / `[[usd_to_usda]]` | 作る | `output_dir` に出力する部品 usda のファイル名 |
| `usda` | `[[join_file]]` | 作る | `output_dir` に出力する結合ファイルのファイル名 |
| `parent_usda` | `[[join_file]]` | 使う | 作る側で宣言した usda のいずれか（親） |
| `child_usda` | `[[join_file.join]]` | 使う | 作る側で宣言した usda のいずれか（子） |

```text
[[urdf_to_usda]]  urdf = "urdf/arm.urdf"      usda = "arm.usda" ─────────────┐
[[urdf_to_usda]]  urdf = "urdf/gripper.urdf"  usda = "two_finger.usda" ──┐   │
                                                                         │   │
[[join_file]]  usda = "arm_unit.usda" ◀── 作る        parent_usda = "arm.usda" ◀┘
  [[join_file.join]]  name = "ee_two_finger"   child_usda = "two_finger.usda" ◀┘
        │
        └─ arm_unit.usda は次の [[join_file]] の parent_usda / child_usda に書ける
```

同じファイル名を 2 か所で作る（Windows の大文字小文字違いを含む）ことや、どこでも作っていない名前を `parent_usda` / `child_usda` に書くことはエラーになります。

### エディタでの補完・リアルタイム検証（JSON Schema + Even Better TOML）と生成スキル

本拡張には `spec.py` / `kinds.py` から生成された JSON Schema（`schema\assembly_pipeline.schema.json`）と `.taplo.toml`、さらに自然言語の説明から TOML 仕様書を自動生成・検証するエージェントスキル（`skills\assembly-pipeline-toml\SKILL.md`）を同梱しています。VS Code / Antigravity で **Even Better TOML**（`tamasfe.even-better-toml`）を入れておくと、キー補完・日本語ホバー説明・未知キーや型ミスの赤波線警告が編集時に自動で効きます。

新しく TOML ファイルを作るときは、1 行目に `schema/assembly_pipeline.schema.json` への相対パスを `#:schema` で記述してください（`examples\` 配下の全サンプルには設定済みです）。

```toml
#:schema ../schema/assembly_pipeline.schema.json
```

`spec.py` に新しいキーを追加したときは、次のコマンドでスキーマ JSON を再生成できます（Isaac Sim は起動しません）。

```powershell
C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py --export-schema
```

### トップレベル設定

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `output_dir` | 必須 | — | すべての usda・`_import\`・検証レポートの出力先フォルダ |
| `compare_official` | 任意 | `true` | フル検証時に公式のまま（`RobotAssembler`）との比較を実行するか |
| `check_kinematics` | 任意 | `true` | フル検証時に純粋 USD 順運動学と PhysX リンク姿勢の突き合わせを実行するか |

### `[[urdf_to_usda]]`（URDF から部品 usda を作る）

URDF 1 本につき 1 ブロック書きます（何個でも可）。

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `urdf` | 必須 | — | 入力 URDF ファイルのパス |
| `usda` | 必須 | — | 出力する部品 usda のファイル名（`.usda` より前は英数字と `_`、先頭は数字以外。例: `"arm.usda"` → 一番上の prim は `/arm`） |
| `fix_base` | 任意 | `None`（Importer 既定） | `false` で浮遊ベース（移動ロボット・ヒューマノイド等）、`true` でワールド固定 |
| `ros_package_paths` | 任意 | `[]` | URDF 内の `package://<パッケージ名>/...` の解決先（`[{ name = "...", path = "..." }]`） |
| `physics_variant` | 任意 | `"auto"` | 正規化層で選択する Physics variant（`"auto"` / `"physx"` / `"physics"` / `"mujoco"`。`"auto"` は `physx` 優先） |
| `collision_from_visuals` | 任意 | `None` | `true` で visual メッシュから collision を生成。省略時は URDF に `<collision>` が 1 つも無い場合のみ `true` |
| `collision_type` | 任意 | `"Convex Hull"` | `collision_from_visuals = true` 時の近似形状（例: `"Convex Hull"`, `"Convex Decomposition"`） |
| `joint_drive_stiffness` | 任意 | `None` | URDF Importer の `override_joint_stiffness`（全可動関節へ適用。mimic follower は R03 の修正で 0 に戻す） |
| `joint_drive_damping` | 任意 | `None` | URDF Importer の `override_joint_damping`（全可動関節へ適用。mimic follower は R03 の修正で 0 に戻す） |

記述例:

```toml
[[urdf_to_usda]]
urdf = "urdf/arm.urdf"
usda = "arm.usda"
fix_base = true
collision_from_visuals = false
ros_package_paths = [{ name = "my_robot_description", path = "meshes" }]
```

### `[[usd_to_usda]]`（既存 USD アセットから部品 usda を作る）

URDF Importer を介さず、既存の `.usd` / `.usda` アセットを部品 usda にラップします。

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `usd` | 必須 | — | 入力 USD のローカルパスまたは URI（`"://"` を含むパス） |
| `usda` | 必須 | — | 出力する部品 usda のファイル名（例: `"ur10e_base.usda"`） |
| `prim_path` | 任意 | `None`（`defaultPrim`） | 入力 USD 内で参照する prim パス（`"/"` で始める） |
| `variant_selections` | 任意 | `{}` | 入力 USD 側で事前選択する VariantSet の対応表（例: `{ Physics = "PhysX", Gripper = "None" }`） |
| `physics_variant` | 任意 | `"auto"` | 正規化層で補完選択する Physics variant 名（`variant_selections` に `Physics` がある場合は `variant_selections` が優先） |

記述例:

```toml
[[usd_to_usda]]
usd = "C:/isaacsim-6.2.0/assets/Isaac/Robots/UniversalRobots/ur10e/configuration/ur10e_physics.usd"
usda = "ur10e_base.usda"
variant_selections = { Physics = "PhysX" }
```

### `[[join_file]]` と `[[join_file.join]]`（結合ファイルを作る）

`[[join_file]]` で 1 つの結合ファイルを定義し、直後にそのファイルで行う結合を `[[join_file.join]]` で 1 つ以上列挙します（`[[join_file]]` の記述順は自由で、依存順に自動構築されます）。

#### `[[join_file]]` のキー

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `usda` | 必須 | — | 出力する結合ファイルのファイル名（例: `"robot.usda"` → 一番上の prim は `/robot`） |
| `parent_usda` | 必須 | — | 親となる usda のファイル名（部品 usda または先行の結合ファイル） |
| `parent_prim_name` | 任意 | `None` | 指定すると親 `parent_usda` を `/<usda stem>/<parent_prim_name>` の 1 段下に子 prim として配置し、一番親の部品名も Stage 上の prim 名・構成要素名として残す（例: `usda = "robot.usda"`, `parent_prim_name = "base"` → `/robot/base/.../child/.../grandchild`）。省略時は `/<usda stem>` へ直接 Reference 合成する |
| `variant_defaults` | 任意 | `{}` | この `[[join_file]]` 配下の `[[join_file.join]]` で定義した `VariantSet` ごとの既定選択肢を指定するテーブル。**キーに `variant_set` 名、値に `variant` 名または `"None"`（未装着）を書く**。省略した `VariantSet` は TOML 上で最初に定義された `[[join_file.join]]` の `variant` が既定になる |

#### `variant_defaults` と `parent_prim_name` の具体的な書き方

インラインテーブル `{ VariantSet名 = "選択肢名" }` 形式で `[[join_file]]` の直下に書きます（`[[join_file.join]]` より前に書いてください）。

```toml
[[join_file]]
usda = "robot.usda"
parent_usda = "base.usda"
parent_prim_name = "base"                                  # Stage 上は /robot/base/... になる
variant_defaults = { Gripper = "None", Sensor = "Lidar" }  # Gripper は既定で未装着("None")、Sensor は Lidar

[[join_file.join]]
name = "gripper_2f85"
child_usda = "robotiq_2f_85.usda"
parent_link = "flange"
child_link = "base_link"
child_prim_name = "gripper"
variant_set = "Gripper"
variant = "Robotiq_2F_85"

[[join_file.join]]
name = "sensor_lidar"
child_usda = "lidar.usda"
parent_link = "mount_link"
child_link = "lidar_base"
child_prim_name = "sensor"
variant_set = "Sensor"
variant = "Lidar"
```

> **注意**: `variant_defaults` を省略した場合、この `robot.usda` の例では `Gripper` の既定選択肢は最初に書いた `"Robotiq_2F_85"` になります。初期状態を未装着（`"None"`）にしたいときや、2 番目以降の選択肢を既定にしたいときに `variant_defaults` を使います。

#### `[[join_file.join]]` のキー

| キー | 必須 | 既定値 | 説明 |
|---|---|---|---|
| `name` | 必須 | — | この結合の名前（結合ファイル内で一意・英数字と `_`）。報告表示（`arm_unit.usda/ee_two_finger`）・`child_prim_name` の既定値・次段結合でのリンク前置き（`arm1/flange`）に使う |
| `child_usda` | 必須 | — | 子となる usda のファイル名 |
| `parent_link` | 必須 | — | 親の usda 側のリンク名（[リンクの書き方](#リンクの書き方) 参照） |
| `child_link` | 必須 | — | 子の usda 側のリンク名（通常は子のルートリンク） |
| `place_child_under` | 任意 | `parent_link` の prim | 子を Reference 配置する親側 prim。リンク名、`"."`（一番上の prim 直下）、または `"/"` 始まりの絶対 prim パス |
| `child_prim_name` | 任意 | `name` | 子を配置する prim 名（英数字と `_`）。同一 `variant_set` 内で揃えると選択肢を切り替えてもパスが一定になり、構成要素名も統一される |
| `offset_xyz` | 任意 | `[0.0, 0.0, 0.0]` | `parent_link` 座標系で見た `child_link` の位置オフセット `[x, y, z]`（m） |
| `offset_rpy` | 任意 | `[0.0, 0.0, 0.0]` | `parent_link` 座標系で見た `child_link` の姿勢オフセット `[roll, pitch, yaw]`（rad、URDF と同じ固定軸 X→Y→Z） |
| `variant_set` | 任意（`variant` と対） | `None` | この結合を選択肢にする VariantSet 名（結合ファイルの一番上の prim に付く） |
| `variant` | `variant_set` 指定時は必須 | `None` | 選択肢名（`"None"` は未装着選択肢として自動追加されるため予約済み） |
| `child_variant_selection` | 任意 | `{}` | 子の usda の一番上の prim が持つ VariantSet で選ぶ選択肢（例: `{ EndEffector = "Rod" }` または `{ EndEffector = "None" }`） |

### リンクの書き方

- **一番上の部品のリンク**: リンク名だけを書く（例: `"flange"`）。
- **結合ファイルの中で結合した子のリンク**: `結合の name/リンク名` のように `/` で前置きを付ける（例: `"arm1/flange"`、2 段入れ子なら `"arm1/ee_two_finger/finger_l"`）。前置きには `child_prim_name` ではなく常に `[[join_file.join]]` の `name` を使います。
- **基準となる usda**: `parent_link` とリンク表記の `place_child_under` は親 `parent_usda` の中での名前、`child_link` は子 `child_usda` の中での名前です（`place_child_under` の `"."` と `"/..."` は出力する結合ファイルの中の prim パスを指します）。
- **VariantSet とリンク前置き**: 前置きでたどる結合が VariantSet 内にある場合、その選択肢が選ばれているときだけ参照できます（`child_link` は `child_variant_selection` または子の既定選択、`parent_link` は親の既定選択）。選ばれていない選択肢の中のリンクを書くと終了コード `4` の仕様エラーになります。
- `--check` を実行すると、各リンクがどの usda のどのリンクに解決されたかが一覧表示されます。

### `[checks]` と `[checks.rules.<R01..R14>]`（事前チェック・修正の制御）

規則 R01〜R14 について「事前チェックで止めるか続行するか（`on_violation`）」と「パイプラインの修正を入れるか外すか（`fix`）」を制御します。全規則の既定値を `[checks]` に書き、個別の規則だけ `[checks.rules.<R01..R14>]` で上書きします。

| キー | 値 | 既定値 | 説明 |
|---|---|---|---|
| `on_violation` | `"continue"` / `"stop"` | `"continue"` | 事前チェックで判定が出たときの挙動。`"continue"` は続行して最後に実測確認する。`"stop"` は URDF 規則（R01・R04・R10）なら import の前、それ以外は結合ファイルを作る前に `report.md` / `report.json` を書いて停止する（終了コード `2`） |
| `fix` | `"apply"` / `"skip"` | `"apply"` | パイプラインの修正を適用するか。`"skip"` にすると修正を外した出力で検証でき、公式側の更新で修正が不要になったかを確かめられる |

記述例:

```toml
# 全体は続行・修正あり（既定値そのまま）にしつつ、
# R02 だけ検出時に停止、R07 だけ修正を skip して挙動を比較する例
[checks]
on_violation = "continue"
fix = "apply"

[checks.rules.R02]
on_violation = "stop"

[checks.rules.R07]
fix = "skip"
```

> **重要（2 つの注意点）**:
> 1. **`[checks]` 全体に `on_violation = "stop"` を書かない**: R11・R14 は結合が 1 回でもあれば必ず事前判定が出るため、全体設定に `"stop"` を書くと結合ファイルがある限り毎回止まります。止めたい規則だけ `[checks.rules.R02]` のように個別に指定してください。
> 2. **独自修正を持たない規則の `fix`**:
>    - `R01`・`R09`: `R02` の修正（子の外につながる joint を構造探索で無効化）によって同時に解消します。切り替えるには `R02` の `fix` を指定します。
>    - `R10`: 修正処理を持ちません（Asset Transformer rules 1.7.7 以降は公式のままでも起きないため測定のみ）。
>    - `R14`: 常に固定パス `<子>/JoinJoint` に 1 本だけ作る構造のため、`fix = "skip"` でも外せません。

### `[probe]`（フル検証時の物理・mimic・許容誤差条件）

モード 4（オプションなしのフル検証）で使う PhysX Play・mimic・`RobotPoser`・順運動学検証の測定パラメータです。省略可で、重いロボットや高剛性な関節を持つロボットで到達判定や誤差許容を調整したいときに書きます。

| キー | 既定値 | 単位 | 説明・計算式 |
|---|---|---|---|
| `steps` | `240` | step | Play 検証で可動関節を駆動しながら進める PhysX ステップ数 |
| `drive_stiffness` | `10000.0` | 回転: `N·m/deg` / 直動: `N/m` | Play 検証で可動関節に一時設定する drive 剛性（USD の `UsdPhysics.DriveAPI` と同一単位） |
| `drive_damping` | `1000.0` | 回転: `N·m·s/deg` / 直動: `N·s/m` | Play 検証で可動関節に一時設定する drive 減衰 |
| `revolute_target` | `0.5` | `rad` | 回転関節の目標角。可動域 `[lower, upper]` を持つ revolute 関節では両端 10% マージンを引いた `[lower + 0.1*(upper-lower), upper - 0.1*(upper-lower)]` へ自動クランプされる（`0` を含まない可動域でも可動域内に収まる）。`continuous` 関節はそのまま適用 |
| `prismatic_ratio` | `0.5` | 比率 (`0.0`〜`1.0`) | 直動関節の目標位置 `lower + prismatic_ratio * (upper - lower)`。可動域未定義時は `lower = 0.0 m`, `upper = 0.1 m` として計算 |
| `mimic_leader_stiffness_linear` | `20.0` | `N/m` | 直動 mimic 検証（R03）で leader 関節に与える柔らかい drive 剛性（follower の不要 drive による引きずりを顕在化させるため意図的に弱くする） |
| `mimic_leader_damping_linear` | `2.0` | `N·s/m` | 直動 mimic 検証（R03）で leader 関節に与える drive 減衰 |
| `mimic_leader_stiffness_angular` | `0.05` | `N·m/deg` | 回転 mimic 検証（R03）で leader 関節に与える柔らかい drive 剛性 |
| `mimic_leader_damping_angular` | `0.005` | `N·m·s/deg` | 回転 mimic 検証（R03）で leader 関節に与える drive 減衰 |
| `mimic_steps` | `300` | step | mimic 検証（R03）で進める PhysX ステップ数（10・30・60 ステップ目と最終ステップで評価） |
| `angle_tol_deg` | `1.0` | `deg` | 症状ありとする角度差の上限（回転関節の到達誤差・結合面の向きずれ・teleport 後の結合ずれがこれを超えると症状あり） |
| `length_tol_m` | `0.001` | `m` | 症状ありとする位置・長さ差の上限（直動関節の到達誤差・結合面の位置ずれ・teleport 後の位置ずれ・順運動学と PhysX のリンク位置差がこれを超えると症状あり） |

記述例（大型・重いアーム向けにステップ数と drive 剛性を上げる例）:

```toml
[probe]
steps = 360
drive_stiffness = 50000.0
drive_damping = 5000.0
revolute_target = 0.4
prismatic_ratio = 0.5
angle_tol_deg = 1.5
length_tol_m = 0.002
```

### よくある `[SPEC ERROR]` の例

`--check`（および実行時の事前検査）は、誤りの場所と候補一覧を明示して停止します。

```text
[SPEC ERROR] usda = 'arm.usda' を 2 か所で作ろうとしている: [[urdf_to_usda]] 1 番目 と [[urdf_to_usda]] 3 番目。出力のファイル名は 1 つずつ別にする
[SPEC ERROR] [[join_file]] usda = 'unit.usda' の parent_usda = 'arms.usda' は、この TOML のどこでも作っていない。作っている usda: ['arm.usda', 'two_finger.usda', 'unit.usda']
[SPEC ERROR] [[join_file]] usda = 'unit.usda' の [[join_file.join]] name = 'ee' の parent_link = 'flang': arm.usda(URDF arm.urdf)にリンク 'flang' が無い。ある名前: ['base_link', 'flange', 'link1', 'link2']
[SPEC ERROR] [[join_file]] usda = 'robot.usda' の variant_defaults: VariantSet 'Gripper' に選択肢 'Wrong' が無い。ある選択肢: ['Robotiq_2F_85', 'None']
[SPEC ERROR] [[join_file]] 1 番目(usda = 'unit.usda'): 知らないキー ['parent']。書けるキー: ['join', 'parent_prim_name', 'parent_usda', 'usda', 'variant_defaults']
```

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

## 事前チェックと修正規則（R01〜R14）の概要

`run_pipeline` は規則 `R01`〜`R14` について事前チェックによる予測と実測による答え合わせを行います。判定フェーズ・実測指標・最終判定（`ok` / `fixed` / `upstream_fixed` / `still_broken` / `unexpected` / `not_measured`）の詳細仕様は [SPECIFICATION.md の規則 R01〜R14 と最終判定の仕様](./SPECIFICATION.md#規則-r01r14-と最終判定の仕様) を参照してください。

| 規則 ID | 種別 | 検出内容・修正対象 |
|---|---|---|
| `R01` | 部品 | ルートリンクがゴーストリンクのためワールド固定 joint 名が `root_joint` にならない問題 |
| `R02` | 結合 | 子のワールド固定 joint が無効化されず親の関節が動かない問題 |
| `R03` | 部品 | mimic follower 関節に非ゼロの drive ゲインが入り leader が目標角に届かない問題 |
| `R04` | 部品 | 末端リンク prim の座標系がメッシュ原点へずれる問題 |
| `R05` | 結合 | `instanceable` な prim 配下への結合が失敗する問題 |
| `R06` | 部品 | `Physics` VariantSet が未選択のまま出力され剛体・関節が存在しない問題 |
| `R07` | 結合 | 子の `ArticulationRootAPI`（Newton API 含む）が残り二重 Articulation になる問題 |
| `R08` | 結合 | 子の基準リンクに剛体が無く固定関節が張れない問題 |
| `R09` | 結合 | 親剛体配下への子配置で閉ループ（closed articulation）になる問題 |
| `R10` | 部品 | 衝突メッシュが通常表示されて視覚メッシュと重なる問題 |
| `R11` | 結合 | 結合した親リンクと子リンク間に衝突除外（FilteredPairs）が無い問題 |
| `R12` | 結合 | `RobotPoser` の teleport で子ロボットが置いて行かれる問題 |
| `R13` | 部品 | 再インポート時に `_1` 等の別名フォルダが増殖する問題 |
| `R14` | 結合 | 再結合時に固定関節 prim が重複増加する問題 |

---

## Python API

Isaac Sim（`SimulationApp`）起動済みの Python スクリプトから直接呼び出せます。`SimulationApp` の起動には `C:\isaacsim-6.2.0\tools\myext\sim_app_launcher.py` の `create_simulation_app` を使用します。

```python
from myext.robot.assembly_pipeline import (
    AssembledRobotDescriptor,
    PipelineStopped,
    build_assembly,
    discover_components,
    inspect_assembled_robot,
    load_spec,
    run_pipeline,
    spawn_assembled_robot,
)

# 1. 非破壊ビルド（アクティブな USD Stage や Timeline をリセットせずに usda を生成）
spec = load_spec("examples/multi_arm.toml")
built = build_assembly(spec)
desc: AssembledRobotDescriptor = built["descriptors"]["robot.usda"]
print(desc.prim_path)                     # "/robot"
print(desc.components)                    # 構成要素名 -> prim パス
print(desc.to_articulation_source_dict()) # {"root_prim_path": ..., "components": ...}

# 2. 結合ファイルを任意ステージへ Reference 配置してディスクリプタを取得
spawn_desc = spawn_assembled_robot(stage, built["outputs"]["robot.usda"], "/World/robot")
all_comps = discover_components(stage, "/World/robot", include_inactive_variants=True)

# 3. 事前チェック・公式比較・動的プローブ・順運動学検証・レポート出力を含むフル実行
result = run_pipeline(spec)
print(result.report["status"], result.outputs["robot.usda"])
```

| API | 役割 |
|---|---|
| `load_spec(path)` | TOML 仕様を読み込み `PipelineSpec` を返す（Isaac Sim 不要） |
| `check_links(spec)` / `describe(spec)` | リンク・VariantSet の静的整合性検査と参照関係サマリ生成（Isaac Sim 不要） |
| `build_assembly(spec, check_idempotency=False)` | 非破壊ビルド（Step 1〜4）で usda 生成と `AssembledRobotDescriptor` 抽出を実行 |
| `run_pipeline(spec)` | フル検証（Step 1〜9）を実行し `report.md` / `report.json` を出力 |
| `spawn_assembled_robot(stage, usda_path, prim_path)` | 生成した結合ファイルをステージに Reference 配置し `AssembledRobotDescriptor` を返す |
| `inspect_assembled_robot(stage, prim_path)` | ステージ上のロボット prim から構成要素・Articulation Root・リンク・関節・VariantSet を抽出 |
| `discover_components(stage, robot_prim_path, include_inactive_variants=True)` | `IsaacRobotAPI` とメタデータから `{構成要素名: prim パス}` 辞書を返す |
| `AssembledRobotDescriptor` | 組み立て済みロボットの中立ディスクリプタ（`to_articulation_source_dict()` で外部 Articulation リーダーへ連携可能） |

---

## テストスイート

テスト方針・各テストの検証内容と合格基準・新しいテストの追加手順は [tests_kit/README.md](../tests_kit/README.md) にまとめています。

1. **パイプライン統合テスト（全 21 テスト・Headless 自動終了）**
   - 出力: `tests_kit\last_run.log`, `tests_kit\last_run.json`
   - 成功の合図: `[SUMMARY] passed 21 / failed 0` および `== exit=0`

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
myext.robot.assembly_pipeline/
├── .taplo.toml                                # Taplo (Even Better TOML) 用スキーマ関連付け設定
├── config/extension.toml                      # Kit 拡張マニフェスト
├── docs/
│   ├── README.md                              # 本ドキュメント（概要・実行手順・TOML 手作業ガイド）
│   └── SPECIFICATION.md                       # 内部仕様書（全用語集・ハッシュキャッシュ・結合 7 段階・R01〜R14 詳細）
├── schema/assembly_pipeline.schema.json       # TOML 補完・静的検証用 JSON Schema (Draft-07)
├── skills/assembly-pipeline-toml/             # 自然言語から TOML を設計・生成・検証するエージェントスキル
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
├── myext/robot/assembly_pipeline/
│   ├── __init__.py                            # 公開 API の再エクスポートと遅延読み込み
│   ├── extension.py                           # Kit 拡張エントリポイント (omni.ext.IExt)
│   ├── kinds.py                               # Literal 型定義と実行時バリデータ
│   ├── spec.py                                # TOML 仕様読み込みとデータクラス定義
│   ├── schema.py                              # JSON Schema 生成・同期検証・純 Python バリデータ
│   ├── spec_check.py                          # リンク・VariantSet の静的検査と仕様サマリ生成
│   ├── urdf_model.py                          # URDF パーサ・ゴーストリンク判定・解析的 FK
│   ├── usd_util.py                            # USD 行列変換・剛体/関節探索・API 判定
│   ├── sources.py                             # 多段結合の階層マップ (Owner / UsdaInfo) と複合 FK
│   ├── descriptor.py                          # AssembledRobotDescriptor / discover_components / spawn
│   ├── part_import.py                         # manifest ハッシュキャッシュ付き URDF/USD 部品インポートと正規化層
│   ├── compose.py                             # 7 段階の冪等な結合ファイル生成
│   ├── runtime.py                             # Kit フレーム更新・Timeline 制御・プローブ用ステージ管理
│   ├── official.py                            # 比較用の公式 RobotAssembler 再現ステージ構築
│   ├── probes.py                              # 静的部品検査・動的 PhysX / RobotPoser 測定
│   ├── kinematics_check.py                    # 純粋 USD 順運動学と PhysX 剛体姿勢の突き合わせ
│   ├── rules.py                               # 規則 R01〜R14 の事前判定と事後評価
│   ├── report.py                              # report.md / report.json 書き出しと前回結果比較
│   └── pipeline.py                            # build_assembly / run_pipeline オーケストレータ
├── run_pipeline.py                            # CLI エントリポイント (--gui / --check / --build-only / --export-schema / フル検証)
├── run_tests.ps1                              # 統合テストスイート実行ラッパー
└── tests_kit/
    ├── README.md                              # テスト方針・テスト一覧・追加手順
    ├── run_tests.py                           # 統合テストスイート (全 21 テスト)
    └── test_official_motion_match.py          # 公式アセット定量 PhysX 動作一致テスト (全 6 テスト)
```
