---
name: assembly-pipeline-toml
description: >-
  Use this skill whenever the user describes a robot assembly in natural language
  and asks to create, edit, or troubleshoot a TOML specification file for
  myext.robot.assembly_pipeline (combining URDFs and/or USD assets, VariantSets,
  multi-stage joins, offsets, floating-base robots, [checks], or [probe]).
  Inspects real URDF/USD assets to extract exact link/VariantSet names, authors
  the TOML with #:schema header, and mechanically validates it before delivery.
---

# assembly_pipeline TOML 作成スキル

ユーザーの自然言語によるロボット構成の説明から、`myext.robot.assembly_pipeline` 用の TOML 仕様ファイルを正確に設計・生成・検証するための手順書です。

定型パターンと制約早見表は [toml_patterns.md](./references/toml_patterns.md) を参照してください。

---

## 実行手順（必ずこの順で進める）

### Step 1: 要件の抽出と不足情報の確認

ユーザーの説明から以下を整理します。

1. **入力部品**: URDF（`[[urdf_to_usda]]`）か既存 USD（`[[usd_to_usda]]`）か、それぞれのファイルパス。
2. **結合構造**: どの親部品のどのリンクに、どの部品をいくつ結合するか（多段結合の中間結合ファイルが必要か）。
3. **配置・姿勢・階層**:
   - 位置・回転オフセット（`offset_xyz` [m], `offset_rpy` [rad]）
   - 浮遊ベースかワールド固定か（移動ロボット・ヒューマノイドなら `fix_base = false`）
   - 一番親の部品名も Stage 上の prim 名として残すか（`parent_prim_name`）
   - 子を親リンク配下以外に置く必要があるか（`place_child_under`）
4. **切替（VariantSet）**: 排他切替する部品、`variant_set` / `variant` 名、初期選択（`variant_defaults` で `"None"` 等にするか）、多段結合時の子側選択（`child_variant_selection`）。
5. **出力先**: TOML ファイルの保存先パスと、TOML 内の `output_dir`。

**確認の原則**:
- ファイルパスが未指定でもワークスペース内（`examples/` や `C:\isaacsim-6.2.0\assets\` 等）から一意に特定できる場合は先に探す。
- 必須の設計判断（複数候補があるパス、未指定の保存先、装着先の曖昧さ等）だけ簡潔に確認し、既定値で安全な項目（`compare_official`, `[probe]` 等）は不要に質問しない。

---

### Step 2: 入力アセットの実地検査（必須・リンク名推測禁止）

TOML を書く前に、必ず [inspect_assets.py](./scripts/inspect_assets.py) で入力 URDF / USD の実物を検査し、実在するリンク名・VariantSet 名・`package://` 参照・`<collision>` 有無を取得します。**リンク名や VariantSet 名を記憶や推測で書かないこと。**

```powershell
# URDF / USD / USDA の検査（SimulationApp 起動不要・数秒で完了）
$env:PYTHONPATH = ''
C:\isaacsim-6.2.0\kit\python\python.exe "<このスキルフォルダ>\scripts\inspect_assets.py" "<path1.urdf>" "<path2.usd>"
```

検査結果から以下を確認する:
- **URDF**: ルートリンク（`child_link` 候補）、末端リンク（`parent_link` 候補）、ゴーストリンク、`<collision>` 有無、`package://` パッケージ名（あれば `ros_package_paths` を設定）、`<mimic>` の有無。
- **USD**: `defaultPrim` の有無（無ければ `prim_path` を設定）、既存 `VariantSet` 名と選択肢（結合側と名前衝突するなら物理ベース層参照または `parent_prim_name` を検討）、剛体（リンク候補）名。

---

### Step 3: TOML の設計と書き出し

[toml_patterns.md](./references/toml_patterns.md) の制約とパターンに従い、TOML ファイルを **UTF-8（BOM なし）** で書き出します。

**必須ルール**:
1. **1 行目に `#:schema` を必ず書く**: 保存先 TOML から `C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline\schema\assembly_pipeline.schema.json` への POSIX 相対パス（`/` 区切り）を記載する。
2. **パスは TOML 基準の相対パス（または URI）** で書く。
3. **同一部品の再利用**: 同じ URDF / USD を複数回結合する場合、`[[urdf_to_usda]]` / `[[usd_to_usda]]` は 1 つだけ定義し、`[[join_file.join]]` を複数記述する。
4. **VariantSet 設計**: 同一 `variant_set` 内の結合は `child_prim_name` を揃える。初期未装着にしたい場合のみ `[[join_file]]` 直下に `variant_defaults = { <VariantSet> = "None" }` を書く。`variant = "None"` は予約済みのため書かない。
5. **`[checks]` の罠回避**: 全体に `on_violation = "stop"` を書かない（R11・R14 で必ず止まる）。止める規則だけ `[checks.rules.<Rxx>]` に書く。
6. **日本語コメントを添える**: 各部品の役割と、結合の意図（オフセット角度の度数換算など）を簡潔にコメントする。

---

### Step 4: 機械検証（必須・省略禁止）

TOML を生成・編集したら、ユーザーへ提示する前に**必ず次の 2 つの検証を両方実行**し、エラー 0 を確認します（いずれも Isaac Sim を起動せず数秒で完了）。

```powershell
$env:PYTHONPATH = ''
# 1. JSON Schema + #:schema 参照先チェック
C:\isaacsim-6.2.0\kit\python\python.exe "<このスキルフォルダ>\scripts\inspect_assets.py" --validate "<生成した.toml>"

# 2. パイプライン静的検査（リンク名・VariantSet・循環・Windows 名衝突）
C:\isaacsim-6.2.0\tools\myext\run_sim.bat "C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline\run_pipeline.py" "<生成した.toml>" --check
```

- どちらかが失敗した場合は、エラーメッセージの原因を修正して再検証し、両方が成功（`[SCHEMA OK]` かつ `[CHECK] 仕様に誤りは無い`）するまで繰り返す。
- 入力に `[[usd_to_usda]]` を含む場合、`--check` は USD ステージを開かないためリンク実在確認は行わない。Step 2 の USD 検査結果と照合し、必要ならユーザーと相談のうえ `--build-only` で実ビルド確認まで行う。

---

### Step 5: ユーザーへの報告

検証通過後、以下を簡潔に報告する:
1. 生成した TOML ファイルへのクリック可能なリンク
2. 採用した構成の要約（部品一覧、結合先リンク、オフセット、VariantSet、既定選択）
3. 機械検証の結果（`[SCHEMA OK]` と `--check` の結果）
4. 次に実行するコマンド（GUI 確認用 `--gui`、生成のみ `--build-only`、フル検証の 3 パターン）
