# assembly_pipeline TOML 設計パターン集

このリファレンスは [SKILL.md](../SKILL.md) の Step 3〜4 で、ユーザーの自然言語要件を正確な TOML 構造へ写像するための定型パターンと制約早見表です。

詳細仕様は拡張本体の [docs/README.md](file:///C:/isaacsim-6.2.0/extsUser/Extentions/myext.robot.assembly_pipeline/docs/README.md) と [docs/SPECIFICATION.md](file:///C:/isaacsim-6.2.0/extsUser/Extentions/myext.robot.assembly_pipeline/docs/SPECIFICATION.md) を参照してください。

---

## 1. 絶対制約（書くと即エラーになるルール）

| 項目 | 制約 |
|---|---|
| `#:schema` ヘッダ | 1 行目に `schema/assembly_pipeline.schema.json` への POSIX 相対パス（`/` 区切り）を記述する（UTF-8・BOM なし） |
| `usda` 名 | フォルダを含まず `.usda` で終わり、`.usda` より前が Python 識別子（`^[A-Za-z_][A-Za-z0-9_]*\.usda$`）。大文字小文字違いの重複も禁止（Windows 衝突防止） |
| `name` / `child_prim_name` / `parent_prim_name` | `^[A-Za-z_][A-Za-z0-9_]*$`。`name` は同一 `[[join_file]]` 内で一意 |
| 参照整合 | `parent_usda` と `child_usda` は同一 TOML 内の `[[urdf_to_usda]]` / `[[usd_to_usda]]` / `[[join_file]]` の `usda` と完全一致させる。循環禁止 |
| `variant_set` と `variant` | 必ず対で指定する。`variant = "None"` は予約済み（自動追加される）のため禁止 |
| `variant_defaults` | `[[join_file]]` 直下（`[[join_file.join]]` より前）にインラインテーブルで書く。値は定義済み `variant` または `"None"` |
| `offset_xyz` / `offset_rpy` | 必ず長さ 3 の数値配列（単位: `offset_xyz` は m、`offset_rpy` は rad 固定軸 X→Y→Z） |
| `[checks]` | 全体設定に `on_violation = "stop"` を書かない（結合があるだけで R11・R14 で必ず止まる）。止める規則だけ `[checks.rules.<Rxx>]` に書く |
| 相対パス基準 | `output_dir`・`urdf`・`usd`（URI 除く）・`ros_package_paths.path` はすべて **TOML ファイルのあるフォルダ基準** |

---

## 2. 要件別 TOML パターン

### パターン A: 最小構成（アーム + グリッパの 1 段結合 + オフセット回転）

```toml
#:schema ../schema/assembly_pipeline.schema.json
output_dir = "out/arm_gripper"

[[urdf_to_usda]]
urdf = "urdf/arm.urdf"
usda = "arm.usda"

[[urdf_to_usda]]
urdf = "urdf/gripper.urdf"
usda = "gripper.usda"

[[join_file]]
usda = "robot.usda"
parent_usda = "arm.usda"

[[join_file.join]]
name = "ee"
child_usda = "gripper.usda"
parent_link = "flange"
child_link = "gripper_base"
offset_xyz = [0.0, 0.0, 0.0]
offset_rpy = [0.0, 0.0, -0.7853981633974483] # z 軸まわりに -45 度
```

### パターン B: 一番親の部品名も Stage prim 名として残す（`parent_prim_name`）

「`robot` の下に `base`、`child`、`grandchild` が全部 prim 名として見えてほしい」場合に使います。

```toml
[[join_file]]
usda = "robot.usda"
parent_usda = "base.usda"
parent_prim_name = "base" # /robot/base/... に親を配置

[[join_file.join]]
name = "child"
child_usda = "child.usda"
parent_link = "mount_link"
child_link = "child_base"
```

### パターン C: VariantSet によるアタッチメント切替と初期未装着（`variant_defaults`）

- 同じ `variant_set` に属する結合は `child_prim_name` を同じ名前に揃えると、切替後もパスと構成要素名が一定になる。
- 初期状態を「何も付いていない状態」にしたい場合は `variant_defaults = { Gripper = "None" }` を明示する。

```toml
[[join_file]]
usda = "arm_unit.usda"
parent_usda = "arm.usda"
variant_defaults = { Gripper = "None" } # 既定は未装着（省略時は先頭結合 Robotiq_2F_85 が既定）

[[join_file.join]]
name = "gripper_85"
child_usda = "robotiq_2f_85.usda"
parent_link = "flange"
child_link = "base_link"
child_prim_name = "gripper"
variant_set = "Gripper"
variant = "Robotiq_2F_85"

[[join_file.join]]
name = "gripper_140"
child_usda = "robotiq_2f_140.usda"
parent_link = "flange"
child_link = "robotiq_base_link"
child_prim_name = "gripper"
variant_set = "Gripper"
variant = "Robotiq_2F_140"
```

### パターン D: 多段結合と同一部品の複数配置・子ごとの VariantSet 選択

- 同じ子部品（例: `arm_unit.usda` や車輪 `wheel.usda`）を複数箇所へ取り付ける場合、`[[urdf_to_usda]]` / `[[join_file]]` は 1 つだけ定義し、`[[join_file.join]]` を複数回書く。
- 子結合ファイルの VariantSet は `child_variant_selection` でインスタンスごとに変えられる。
- 2 段目以降で子の内部リンクを指定するときは `結合の name/リンク名`（`child_prim_name` ではなく `name`）を使う。

```toml
[[join_file]]
usda = "multi_robot.usda"
parent_usda = "base.usda"

[[join_file.join]]
name = "left_arm"
child_usda = "arm_unit.usda"
parent_link = "port_left"
child_link = "base_link"
child_variant_selection = { Gripper = "Robotiq_2F_85" }

[[join_file.join]]
name = "right_arm"
child_usda = "arm_unit.usda"
parent_link = "port_right"
child_link = "base_link"
offset_rpy = [0.0, 0.0, 3.141592653589793]
child_variant_selection = { Gripper = "None" }
```

### パターン E: 既存 USD アセット入力（`[[usd_to_usda]]`）と VariantSet 衝突回避

- Isaac Sim 公式ロボット USD（UR10e、Unitree G1 等）を使う場合、トップの `ur10e.usd` が既に `Gripper` VariantSet を持っていることがある。
- 同じ名前の VariantSet を `[[join_file]]` 側でも作りたい場合は、可能な限り内蔵 VariantSet の無い物理ベース層（例: `configuration/ur10e_physics.usd`）を参照するか、`parent_prim_name` で 1 段下げて衝突を避ける。
- 入力 USD に `defaultPrim` が無い場合のみ `prim_path = "/..."` を指定する。

```toml
[[usd_to_usda]]
usd = "C:/isaacsim-6.2.0/assets/Isaac/Robots/UniversalRobots/ur10e/configuration/ur10e_physics.usd"
usda = "ur10e_base.usda"
variant_selections = { Physics = "PhysX" }
```

### パターン F: 移動ロボット・ヒューマノイド（浮遊ベース）

ベースとなる `[[urdf_to_usda]]` に `fix_base = false` を明示する（子部品には原則不要）。

```toml
[[urdf_to_usda]]
urdf = "carter_base.urdf"
usda = "carter_base.usda"
fix_base = false
```

### パターン G: ROS `package://` メッシュ解決と衝突メッシュ制御

- URDF 内の `<mesh filename="package://franka_description/meshes/..."/>` に対して、`ros_package_paths` にパッケージ名とルートフォルダを対応付ける。
- 見た目のちらつき（visual と collision メッシュの重複）を URDF 側 `<collision>` 無視で解消したい場合だけ `collision_from_visuals = true` を指定する。通常は省略（URDF に `<collision>` があれば `false`、無ければ `true`）。

```toml
[[urdf_to_usda]]
urdf = "C:/isaacsim-6.2.0/assets/Isaac/Robots/FrankaRobotics/FrankaPanda/urdf/panda_arm.urdf"
usda = "panda_arm.usda"
ros_package_paths = [{ name = "franka_description", path = "C:/isaacsim-6.2.0/assets/Isaac/Robots/FrankaRobotics/FrankaPanda" }]
```

### パターン H: `[checks]` と `[probe]` の調整

- 特定規則だけ検出時に停止させたい場合: `[checks.rules.<Rxx>]` に `on_violation = "stop"` を書く。
- 大型・高慣性ロボットでフル検証の到達が間に合わない場合: `[probe]` の `steps`・`drive_stiffness`・`drive_damping`・許容誤差を増やす。
- フル検証で公式 `RobotAssembler` 比較（意図的な PhysX エラーログ）を省きたい場合: トップレベルに `compare_official = false` を書く。

```toml
compare_official = false

[checks.rules.R02]
on_violation = "stop"

[probe]
steps = 360
drive_stiffness = 50000.0
drive_damping = 5000.0
revolute_target = 0.4
angle_tol_deg = 1.5
length_tol_m = 0.002
```

---

## 3. 自然言語 → TOML マッピング早見表

| ユーザーの言葉 | 対応する TOML 要素 |
|---|---|
| 「URDF から作る」 | `[[urdf_to_usda]]` |
| 「既存の USD / Isaac Sim 公式アセットを使う」 | `[[usd_to_usda]]` |
| 「移動ロボット / ヒューマノイド / 地面に固定しない」 | ベースの `[[urdf_to_usda]]` に `fix_base = false` |
| 「A の X リンクに B の Y リンクをくっつける」 | `[[join_file]]` + `[[join_file.join]]` (`parent_link = "X"`, `child_link = "Y"`) |
| 「位置を少しずらす / Z 軸まわりに -45 度回す」 | `offset_xyz = [x, y, z]`（m）, `offset_rpy = [r, p, y]`（rad） |
| 「グリッパを切り替えられるようにしたい / 着脱したい」 | `variant_set` + `variant` + 共通 `child_prim_name` |
| 「最初はグリッパ無しにしたい」 | `[[join_file]]` に `variant_defaults = { Gripper = "None" }` |
| 「同じ車輪/アームを左右に 2 本（または 4 本）付ける」 | 部品定義は 1 つ、`[[join_file.join]]` を複数記述 |
| 「左腕は 2 本指、右腕は未装着にする」 | `child_variant_selection = { ... }` |
| 「親部品の名前も Stage の階層に残したい」 | `[[join_file]]` に `parent_prim_name = "..."` |
| 「一番上の prim 直下に子を置きたい」 | `[[join_file.join]]` に `place_child_under = "."` |
| 「公式 RobotAssembler 比較は要らない」 | トップレベルに `compare_official = false` |
| 「重いロボットなので強めに動かして検証したい」 | `[probe]` の `steps`, `drive_stiffness`, `drive_damping` |
