"""パイプライン仕様 TOML 用の JSON Schema (Draft-07) 生成と純 Python バリデーション。

責務:
  1. `spec.py` のデータクラス群と `kinds.py` の `Literal` 列挙型を唯一の正（Single Source of Truth）として、
     Even Better TOML / Taplo などのエディタ補完・静的検証で使える JSON Schema を生成する
     （`build_json_schema` / `write_json_schema`）。
  2. 外部パッケージ（`jsonschema` / `pydantic` 等）に一切依存せず、生成済みスキーマと `spec.py` の
     キー・列挙値が同期しているかを検査し（`verify_schema_sync`）、任意の TOML ファイルを
     スキーマに対して検証する（`validate_toml_file`）。

主な公開 API:
  - `DEFAULT_SCHEMA_PATH`: 既定の出力先（`<拡張ルート>/schema/assembly_pipeline.schema.json`）
  - `build_json_schema() -> dict[str, Any]`: JSON Schema (Draft-07) 辞書を生成する
  - `write_json_schema(path=None) -> Path`: スキーマ JSON をファイルへ書き出す
  - `verify_schema_sync(schema=None) -> list[str]`: `spec.py` / `kinds.py` とスキーマの差分を検査する
  - `validate_dict_against_schema(data, schema=None) -> list[str]`: 辞書データをスキーマ検証する
  - `validate_toml_file(path, schema=None) -> list[str]`: TOML ファイルを読み込んでスキーマ検証する

呼ばれ元:
  - `__init__.py`（公開 API として再エクスポート）
  - `run_pipeline.py`（`--export-schema` でスキーマ JSON を再生成）
  - `tests_kit/run_tests.py`（`spec.py` と `schema/assembly_pipeline.schema.json` の同期・全サンプル適合検証）

コマンドラインでの再生成:
  ```powershell
  python -m myext.robot.assembly_pipeline_v2.schema
  ```
"""

from __future__ import annotations

import json
import os
import re
import tomllib
from dataclasses import fields
from pathlib import Path
from typing import Any

from .kinds import FIX_MODES, ON_VIOLATION_VALUES, PHYSICS_VARIANTS, RULE_IDS
from .spec import (
    ALLOWED_TOP,
    JoinFileSpec,
    JoinSpec,
    ProbeSettings,
    RuleSetting,
    UrdfSource,
    UsdSource,
)

# 拡張ルート直下の schema/assembly_pipeline.schema.json を既定の出力先にする。
EXT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCHEMA_PATH = EXT_ROOT / "schema" / "assembly_pipeline.schema.json"

# .usda ファイル名と USD prim 名・結合名に許可する正規表現（spec.py の isidentifier() と対応）。
USDA_NAME_PATTERN = r"^[A-Za-z_][A-Za-z0-9_]*\.usda$"
IDENTIFIER_PATTERN = r"^[A-Za-z_][A-Za-z0-9_]*$"
PRIM_PATH_PATTERN = r"^/.+$"

# 規則ごとの短い日本語説明（[checks.rules.<R01..R14>] のホバー説明に使う）。
_RULE_DESCRIPTIONS: dict[str, str] = {
    "R01": "ルートリンクが空（ゴーストリンク）のためワールド固定 joint 名が root_joint にならない問題",
    "R02": "子のワールド固定 joint が無効化されず親の関節が動かない問題",
    "R03": "mimic follower 関節に非ゼロの drive ゲインが入り leader の動きを妨げる問題",
    "R04": "末端リンク prim の座標系がメッシュ原点へずれる問題",
    "R05": "instanceable な prim 配下への結合が失敗する問題",
    "R06": "Physics VariantSet が未選択のまま出力され剛体・関節が存在しない問題",
    "R07": "子の ArticulationRootAPI（Newton API 含む）が残り二重 Articulation になる問題",
    "R08": "子の基準リンクに剛体が無く固定関節が張れない問題",
    "R09": "親剛体配下への子配置で閉ループ（closed articulation）になる問題",
    "R10": "衝突メッシュが通常表示されて視覚メッシュと重なる問題",
    "R11": "結合した親リンクと子リンク間に衝突除外（FilteredPairs）が無い問題",
    "R12": "RobotPoser のテレポートで子ロボットが置いて行かれる問題",
    "R13": "再インポート時に _1 等の別名フォルダが増殖する問題",
    "R14": "再結合時に固定関節 prim が重複増加する問題",
}


def _vec3_schema(description: str, default: list[float]) -> dict[str, Any]:
    """長さ 3 の数値配列（xyz / rpy 等）を表す JSON Schema 断片を返す。"""
    return {
        "type": "array",
        "description": description,
        "items": {"type": "number"},
        "minItems": 3,
        "maxItems": 3,
        "default": default,
    }


def build_json_schema() -> dict[str, Any]:
    """`spec.py` と `kinds.py` の定義から JSON Schema (Draft-07) 辞書を構築して返す。

    Taplo（Even Better TOML）での自動補完・ホバー説明・静的エラー検出に最適化しており、
    すべてのテーブルに `additionalProperties: false` を設定してタイプミスを即時検知する。
    """
    probe_defaults = ProbeSettings()

    rule_setting_def: dict[str, Any] = {
        "type": "object",
        "description": "個別規則の上書き設定。省略した項目は [checks] の全体設定を継承する。",
        "additionalProperties": False,
        "properties": {
            "on_violation": {
                "type": "string",
                "enum": list(ON_VIOLATION_VALUES),
                "description": "事前チェックで違反を検知したときの挙動（'continue': 結合を継続、'stop': 結合前に停止）。",
            },
            "fix": {
                "type": "string",
                "enum": list(FIX_MODES),
                "description": "既知不具合への自動修正を適用するか（'apply': 修正適用、'skip': 比較用に修正をスキップ）。",
            },
        },
    }

    ros_package_def: dict[str, Any] = {
        "type": "object",
        "description": "URDF 内の package://<name>/... をローカルフォルダへ解決するための対応表エントリ。",
        "additionalProperties": False,
        "required": ["name", "path"],
        "properties": {
            "name": {
                "type": "string",
                "minLength": 1,
                "description": "package:// の直後に来る ROS パッケージ名（例: 'franka_description', 'meshes'）。",
            },
            "path": {
                "type": "string",
                "minLength": 1,
                "description": "パッケージに対応するローカルフォルダパス（TOML ファイルからの相対パス可）。",
            },
        },
    }

    urdf_source_def: dict[str, Any] = {
        "type": "object",
        "description": "URDF 1 本から正規化部品 .usda を 1 つ生成する設定（[[urdf_to_usda]]）。",
        "additionalProperties": False,
        "required": ["urdf", "usda"],
        "properties": {
            "urdf": {
                "type": "string",
                "minLength": 1,
                "description": "入力 URDF ファイルのパス（TOML ファイルからの相対パスまたは絶対パス）。",
            },
            "usda": {
                "type": "string",
                "pattern": USDA_NAME_PATTERN,
                "description": "output_dir に出力する部品 .usda のファイル名（フォルダなし、拡張子前が prim 名になる）。",
            },
            "collision_from_visuals": {
                "type": "boolean",
                "description": "true にすると visual メッシュから collision を生成し重複メッシュを作らない。省略時は URDF に <collision> が無い場合のみ true。",
            },
            "collision_type": {
                "type": "string",
                "minLength": 1,
                "default": "Convex Hull",
                "examples": ["Convex Hull", "Convex Decomposition", "Mesh Simplification", "Bounding Cube", "Bounding Sphere"],
                "description": "collision_from_visuals 有効時に使用する衝突近似形状。",
            },
            "physics_variant": {
                "type": "string",
                "enum": list(PHYSICS_VARIANTS),
                "default": "auto",
                "description": "正規化層で選択する Physics VariantSet の候補（'auto' は physx → physics の順に優先）。",
            },
            "ros_package_paths": {
                "type": "array",
                "items": {"$ref": "#/definitions/RosPackagePath"},
                "default": [],
                "description": "URDF 内の package:// メッシュ参照を解決するパッケージ名とフォルダの対応一覧。",
            },
            "joint_drive_stiffness": {
                "type": "number",
                "description": "URDF Importer の override_joint_stiffness（全可動関節へ適用。mimic follower は R03 で 0 に戻す）。",
            },
            "joint_drive_damping": {
                "type": "number",
                "description": "URDF Importer の override_joint_damping（全可動関節へ適用。mimic follower は R03 で 0 に戻す）。",
            },
            "fix_base": {
                "type": "boolean",
                "description": "URDF Importer の fix_base。false で浮遊ベース（移動ロボット・ヒューマノイド等）、true でワールド固定。省略時は Importer 既定。",
            },
        },
    }

    usd_source_def: dict[str, Any] = {
        "type": "object",
        "description": "既存の USD / USDA アセットから正規化部品 .usda を 1 つ生成する設定（[[usd_to_usda]]）。",
        "additionalProperties": False,
        "required": ["usd", "usda"],
        "properties": {
            "usd": {
                "type": "string",
                "minLength": 1,
                "description": "入力 USD ファイルのローカルパス（TOML 基準相対パス可）または URI（'://' を含むパス）。",
            },
            "usda": {
                "type": "string",
                "pattern": USDA_NAME_PATTERN,
                "description": "output_dir に出力する正規化 .usda のファイル名（フォルダなし、拡張子前が prim 名になる）。",
            },
            "prim_path": {
                "type": "string",
                "pattern": PRIM_PATH_PATTERN,
                "description": "入力 USD 内の参照対象 prim パス（'/' 始まり）。省略時は入力 USD の defaultPrim を参照する。",
            },
            "variant_selections": {
                "type": "object",
                "additionalProperties": {"type": "string"},
                "default": {},
                "description": "入力 USD の VariantSet に対して事前選択する { VariantSet名 = '選択肢' } マップ。",
            },
            "physics_variant": {
                "type": "string",
                "minLength": 1,
                "default": "auto",
                "examples": ["auto", "PhysX", "Physx", "Physx_Mimic", "SimplifiedPhysX", "Physics", "None"],
                "description": "正規化層で補完選択する Physics variant 名（variant_selections で Physics を明示した場合はそちらが優先）。",
            },
        },
    }

    join_spec_def: dict[str, Any] = {
        "type": "object",
        "description": "親リンクと子リンクの 1 回の結合定義（[[join_file.join]]）。",
        "additionalProperties": False,
        "required": ["name", "child_usda", "parent_link", "child_link"],
        "dependencies": {
            "variant_set": ["variant"],
            "variant": ["variant_set"],
        },
        "properties": {
            "name": {
                "type": "string",
                "pattern": IDENTIFIER_PATTERN,
                "description": "この結合ファイル内で一意な結合名（英数字と _ のみ）。次段結合でのリンク接頭辞や既定の child_prim_name に使われる。",
            },
            "child_usda": {
                "type": "string",
                "pattern": USDA_NAME_PATTERN,
                "description": "子として結合する .usda ファイル名（[[urdf_to_usda]] / [[usd_to_usda]] / 先行 [[join_file]] のいずれか）。",
            },
            "parent_link": {
                "type": "string",
                "minLength": 1,
                "description": "親側の結合先リンク名（多段結合では 'arm1/flange' のように結合名接頭辞を付けられる）。",
            },
            "child_link": {
                "type": "string",
                "minLength": 1,
                "description": "子側の基準リンク名（parent_link + offset に一致させるリンク）。",
            },
            "place_child_under": {
                "type": "string",
                "minLength": 1,
                "description": "子を Reference 配置する親側 prim（リンク名、'.' で結合ファイル最上位 prim 直下、または '/' 始まりの絶対パス）。省略時は parent_link の prim。",
            },
            "child_prim_name": {
                "type": "string",
                "pattern": IDENTIFIER_PATTERN,
                "description": "子を配置する prim 名（省略時は name）。同一 variant_set 内で揃えると選択肢を切り替えてもパスが一定になる。",
            },
            "offset_xyz": _vec3_schema("parent_link 座標系で見た child_link の位置オフセット [x, y, z] (m)。", [0.0, 0.0, 0.0]),
            "offset_rpy": _vec3_schema(
                "parent_link 座標系で見た child_link の姿勢オフセット [roll, pitch, yaw] (rad、URDF と同じ固定軸 X→Y→Z)。",
                [0.0, 0.0, 0.0],
            ),
            "variant_set": {
                "type": "string",
                "minLength": 1,
                "description": "この結合を選択肢として格納する VariantSet 名（variant と必ず対で指定する）。",
            },
            "variant": {
                "type": "string",
                "minLength": 1,
                "not": {"const": "None"},
                "description": "variant_set 内の選択肢名（'None' は未装着選択肢として自動追加されるため予約済み）。",
            },
            "child_variant_selection": {
                "type": "object",
                "additionalProperties": {"type": "string"},
                "default": {},
                "description": "子 .usda が持つ VariantSet の選択（例: { EndEffector = 'Rod' }）。",
            },
        },
    }

    join_file_def: dict[str, Any] = {
        "type": "object",
        "description": "親 .usda に 1 つ以上の子 .usda を結合した結合ファイルを生成する設定（[[join_file]]）。",
        "additionalProperties": False,
        "required": ["usda", "parent_usda"],
        "properties": {
            "usda": {
                "type": "string",
                "pattern": USDA_NAME_PATTERN,
                "description": "output_dir に出力する結合ファイルのファイル名（拡張子前が defaultPrim 名になる）。",
            },
            "parent_usda": {
                "type": "string",
                "pattern": USDA_NAME_PATTERN,
                "description": "親となる .usda のファイル名（[[urdf_to_usda]] / [[usd_to_usda]] / 先行 [[join_file]]）。",
            },
            "parent_prim_name": {
                "type": "string",
                "pattern": IDENTIFIER_PATTERN,
                "description": "指定すると親 parent_usda を /<usda stem>/<parent_prim_name> の 1 段下に子 prim として配置し、一番親のパーツ名も Stage 上に残す。",
            },
            "variant_defaults": {
                "type": "object",
                "additionalProperties": {"type": "string"},
                "default": {},
                "description": "この結合ファイルで作る VariantSet ごとの既定選択肢（例: { Gripper = 'Robotiq_2F_85' }。'None' 指定可）。",
            },
            "join": {
                "type": "array",
                "items": {"$ref": "#/definitions/JoinSpec"},
                "default": [],
                "description": "この結合ファイルで行う結合一覧（[[join_file.join]]）。",
            },
        },
    }

    checks_def: dict[str, Any] = {
        "type": "object",
        "description": "規則 R01〜R14 の事前チェック・修正適用の制御設定（[checks]）。",
        "additionalProperties": False,
        "properties": {
            "on_violation": {
                "type": "string",
                "enum": list(ON_VIOLATION_VALUES),
                "default": "continue",
                "description": "全規則の既定の違反時動作（'continue': 継続、'stop': 結合前に停止）。",
            },
            "fix": {
                "type": "string",
                "enum": list(FIX_MODES),
                "default": "apply",
                "description": "全規則の既定の修正適用モード（'apply': 修正適用、'skip': 修正なし）。",
            },
            "rules": {
                "type": "object",
                "description": "規則 ID（R01〜R14）ごとの個別上書き設定（[checks.rules.<R01..R14>]）。",
                "additionalProperties": False,
                "properties": {
                    rid: {
                        "allOf": [{"$ref": "#/definitions/RuleSetting"}],
                        "description": f"[{rid}] {_RULE_DESCRIPTIONS.get(rid, rid)}",
                    }
                    for rid in RULE_IDS
                },
            },
        },
    }

    probe_def: dict[str, Any] = {
        "type": "object",
        "description": "フル検証時の PhysX / mimic / RobotPoser 測定パラメータ（[probe]）。",
        "additionalProperties": False,
        "properties": {
            "steps": {
                "type": "integer",
                "minimum": 1,
                "default": probe_defaults.steps,
                "description": "結合ステージの Play 検証で進める PhysX ステップ数。",
            },
            "drive_stiffness": {
                "type": "number",
                "default": probe_defaults.drive_stiffness,
                "description": "Play 検証で関節を駆動する drive 剛性（回転は deg 基準、直動は m 基準）。",
            },
            "drive_damping": {
                "type": "number",
                "default": probe_defaults.drive_damping,
                "description": "Play 検証で関節を駆動する drive 減衰。",
            },
            "revolute_target": {
                "type": "number",
                "default": probe_defaults.revolute_target,
                "description": "回転関節の目標角 [rad]（リミットがある場合は自動的に可動域内側の安全範囲へクランプされる）。",
            },
            "prismatic_ratio": {
                "type": "number",
                "default": probe_defaults.prismatic_ratio,
                "description": "直動関節の目標位置を可動域の何割（0.0〜1.0）にするか。",
            },
            "mimic_leader_stiffness_linear": {
                "type": "number",
                "default": probe_defaults.mimic_leader_stiffness_linear,
                "description": "直動 mimic プローブ（R03）で leader 関節に与える柔らかい drive 剛性。",
            },
            "mimic_leader_damping_linear": {
                "type": "number",
                "default": probe_defaults.mimic_leader_damping_linear,
                "description": "直動 mimic プローブ（R03）で leader 関節に与える drive 減衰。",
            },
            "mimic_leader_stiffness_angular": {
                "type": "number",
                "default": probe_defaults.mimic_leader_stiffness_angular,
                "description": "回転 mimic プローブ（R03）で leader 関節に与える柔らかい drive 剛性。",
            },
            "mimic_leader_damping_angular": {
                "type": "number",
                "default": probe_defaults.mimic_leader_damping_angular,
                "description": "回転 mimic プローブ（R03）で leader 関節に与える drive 減衰。",
            },
            "mimic_steps": {
                "type": "integer",
                "minimum": 1,
                "default": probe_defaults.mimic_steps,
                "description": "mimic プローブ（R03）で進める PhysX ステップ数。",
            },
            "angle_tol_deg": {
                "type": "number",
                "minimum": 0.0,
                "default": probe_defaults.angle_tol_deg,
                "description": "角度誤差の許容上限 [deg]。",
            },
            "length_tol_m": {
                "type": "number",
                "minimum": 0.0,
                "default": probe_defaults.length_tol_m,
                "description": "位置・長さ誤差の許容上限 [m]。",
            },
        },
    }

    return {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "$id": "https://local.myext.robot.assembly_pipeline.v2/assembly_pipeline.schema.json",
        "title": "myext.robot.assembly_pipeline.v2 TOML Specification",
        "description": "Isaac Sim 6.2.0 向け URDF / USD ロボット結合パイプライン（myext.robot.assembly_pipeline.v2）の TOML 仕様スキーマ。",
        "type": "object",
        "additionalProperties": False,
        "required": ["output_dir"],
        "anyOf": [
            {"required": ["urdf_to_usda"]},
            {"required": ["usd_to_usda"]},
        ],
        "properties": {
            "output_dir": {
                "type": "string",
                "minLength": 1,
                "description": "生成した .usda・_import/・report.md / report.json の出力先フォルダ（TOML ファイルからの相対パス可）。",
            },
            "urdf_to_usda": {
                "type": "array",
                "items": {"$ref": "#/definitions/UrdfSource"},
                "description": "URDF ファイルから正規化部品 .usda を生成する定義一覧（[[urdf_to_usda]]）。",
            },
            "usd_to_usda": {
                "type": "array",
                "items": {"$ref": "#/definitions/UsdSource"},
                "description": "既存 USD アセットから正規化部品 .usda を生成する定義一覧（[[usd_to_usda]]）。",
            },
            "join_file": {
                "type": "array",
                "items": {"$ref": "#/definitions/JoinFileSpec"},
                "description": "親 .usda と子 .usda を結合した結合ファイルの定義一覧（[[join_file]]）。",
            },
            "checks": {
                "$ref": "#/definitions/ChecksConfig",
            },
            "probe": {
                "$ref": "#/definitions/ProbeSettings",
            },
            "compare_official": {
                "type": "boolean",
                "default": True,
                "description": "フル検証時に公式 RobotAssembler の再現ステージでも比較測定を行うか（既定 true）。",
            },
            "check_kinematics": {
                "type": "boolean",
                "default": True,
                "description": "フル検証時に純粋 USD 解析的 FK と PhysX リンク姿勢の突き合わせ検証を行うか（既定 true）。",
            },
            "check_geometric": {
                "type": "boolean",
                "deprecated": True,
                "description": "互換用エイリアス（check_kinematics を使用してください）。",
            },
        },
        "definitions": {
            "RosPackagePath": ros_package_def,
            "UrdfSource": urdf_source_def,
            "UsdSource": usd_source_def,
            "JoinSpec": join_spec_def,
            "JoinFileSpec": join_file_def,
            "RuleSetting": rule_setting_def,
            "ChecksConfig": checks_def,
            "ProbeSettings": probe_def,
        },
    }


def write_json_schema(path: str | Path | None = None) -> Path:
    """JSON Schema をファイルへ書き出して出力パスを返す（内容同一なら無変更でスキップ）。"""
    out_path = Path(path) if path is not None else DEFAULT_SCHEMA_PATH
    out_path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(build_json_schema(), ensure_ascii=False, indent=2) + "\n"
    if not out_path.exists() or out_path.read_text(encoding="utf-8") != text:
        out_path.write_text(text, encoding="utf-8")
    return out_path


def verify_schema_sync(schema: dict[str, Any] | None = None) -> list[str]:
    """`spec.py` / `kinds.py` の実定義と JSON Schema のキー・列挙値が完全一致しているか検査する。

    Returns:
        不一致メッセージのリスト（空なら完全同期）。
    """
    sc = schema if schema is not None else build_json_schema()
    defs = sc.get("definitions", {})
    errors: list[str] = []

    def _cmp(label: str, expected: set[str], actual: set[str]) -> None:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing:
            errors.append(f"{label}: スキーマに不足しているキー {missing}")
        if extra:
            errors.append(f"{label}: スキーマに余剰なキー {extra}")

    _cmp("top-level", set(ALLOWED_TOP), set(sc.get("properties", {})))
    _cmp("UrdfSource", {f.name for f in fields(UrdfSource)}, set(defs["UrdfSource"]["properties"]))
    _cmp("UsdSource", {f.name for f in fields(UsdSource)}, set(defs["UsdSource"]["properties"]))
    # JoinFileSpec の TOML キーは joins -> join に写像される
    jf_expected = {("join" if f.name == "joins" else f.name) for f in fields(JoinFileSpec)}
    _cmp("JoinFileSpec", jf_expected, set(defs["JoinFileSpec"]["properties"]))
    _cmp("JoinSpec", {f.name for f in fields(JoinSpec)}, set(defs["JoinSpec"]["properties"]))
    _cmp("RuleSetting", {f.name for f in fields(RuleSetting)}, set(defs["RuleSetting"]["properties"]))
    _cmp("ProbeSettings", {f.name for f in fields(ProbeSettings)}, set(defs["ProbeSettings"]["properties"]))
    _cmp("checks.rules", set(RULE_IDS), set(defs["ChecksConfig"]["properties"]["rules"]["properties"]))

    # enum 一致チェック
    if defs["UrdfSource"]["properties"]["physics_variant"]["enum"] != list(PHYSICS_VARIANTS):
        errors.append("UrdfSource.physics_variant の enum が PHYSICS_VARIANTS と不一致")
    if defs["RuleSetting"]["properties"]["on_violation"]["enum"] != list(ON_VIOLATION_VALUES):
        errors.append("RuleSetting.on_violation の enum が ON_VIOLATION_VALUES と不一致")
    if defs["RuleSetting"]["properties"]["fix"]["enum"] != list(FIX_MODES):
        errors.append("RuleSetting.fix の enum が FIX_MODES と不一致")

    return errors


# ==================================================================== 純 Python JSON Schema (Draft-07) 検証器
def _resolve_ref(root: dict[str, Any], ref: str) -> dict[str, Any]:
    """ローカル `$ref`（`#/definitions/...`）を解決する。"""
    if not ref.startswith("#/"):
        raise ValueError(f"未対応の $ref: {ref!r}")
    node: Any = root
    for part in ref[2:].split("/"):
        node = node[part]
    return node


def _is_type(value: Any, expected: str) -> bool:
    """JSON Schema のプリミティブ型判定（Python では bool が int のサブクラスである点に注意）。"""
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "null":
        return value is None
    return True


def _validate_node(root: dict[str, Any], node: dict[str, Any], value: Any, path: str, out: list[str]) -> None:
    """JSON Schema (Draft-07) の主要キーワードを再帰評価してエラーを `out` へ集める。"""
    if "$ref" in node:
        _validate_node(root, _resolve_ref(root, node["$ref"]), value, path, out)
    for sub in node.get("allOf", ()):
        _validate_node(root, sub, value, path, out)
    if "anyOf" in node:
        matched = False
        for sub in node["anyOf"]:
            probe_errs: list[str] = []
            _validate_node(root, sub, value, path, probe_errs)
            if not probe_errs:
                matched = True
                break
        if not matched:
            out.append(f"{path}: anyOf のいずれの条件も満たさない")
    if "not" in node:
        probe_errs = []
        _validate_node(root, node["not"], value, path, probe_errs)
        if not probe_errs:
            out.append(f"{path}: 禁止された値 {value!r} に一致した")
    if "const" in node and value != node["const"]:
        out.append(f"{path}: {node['const']!r} である必要がある（実際: {value!r}）")
    if "enum" in node and value not in node["enum"]:
        out.append(f"{path}: {node['enum']} のいずれかである必要がある（実際: {value!r}）")
    if "type" in node:
        expected = node["type"]
        ok = any(_is_type(value, t) for t in expected) if isinstance(expected, list) else _is_type(value, expected)
        if not ok:
            out.append(f"{path}: 型 {expected!r} が必要（実際: {type(value).__name__} = {value!r}）")
            return

    if isinstance(value, str):
        if "minLength" in node and len(value) < node["minLength"]:
            out.append(f"{path}: 長さは {node['minLength']} 以上が必要")
        if "pattern" in node and re.search(node["pattern"], value) is None:
            out.append(f"{path}: パターン {node['pattern']!r} に一致しない（実際: {value!r}）")

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in node and value < node["minimum"]:
            out.append(f"{path}: {node['minimum']} 以上が必要（実際: {value!r}）")

    if isinstance(value, list):
        if "minItems" in node and len(value) < node["minItems"]:
            out.append(f"{path}: 要素数は {node['minItems']} 以上が必要（実際: {len(value)}）")
        if "maxItems" in node and len(value) > node["maxItems"]:
            out.append(f"{path}: 要素数は {node['maxItems']} 以下が必要（実際: {len(value)}）")
        if "items" in node and isinstance(node["items"], dict):
            for idx, item in enumerate(value):
                _validate_node(root, node["items"], item, f"{path}[{idx}]", out)

    if isinstance(value, dict):
        for req in node.get("required", ()):
            if req not in value:
                out.append(f"{path}: 必須キー {req!r} が無い")
        for key, deps in node.get("dependencies", {}).items():
            if key in value and isinstance(deps, list):
                for dep in deps:
                    if dep not in value:
                        out.append(f"{path}: {key!r} を書くときは {dep!r} も必要")
        props = node.get("properties", {})
        addl = node.get("additionalProperties", True)
        for k, v in value.items():
            child_path = f"{path}.{k}" if path != "$" else k
            if k in props:
                _validate_node(root, props[k], v, child_path, out)
            elif addl is False:
                out.append(f"{path}: 未知のキー {k!r}")
            elif isinstance(addl, dict):
                _validate_node(root, addl, v, child_path, out)


def validate_dict_against_schema(data: dict[str, Any], schema: dict[str, Any] | None = None) -> list[str]:
    """TOML をパースした辞書 `data` を JSON Schema に対して検証し、エラー一覧を返す。"""
    sc = schema if schema is not None else build_json_schema()
    errors: list[str] = []
    _validate_node(sc, sc, data, "$", errors)
    return errors


def validate_toml_file(path: str | Path, schema: dict[str, Any] | None = None) -> list[str]:
    """TOML ファイルを読み込み、JSON Schema に対して検証したエラー一覧を返す。"""
    with open(os.fspath(path), "rb") as fh:
        data = tomllib.load(fh)
    return validate_dict_against_schema(data, schema)


__all__ = [
    "DEFAULT_SCHEMA_PATH",
    "USDA_NAME_PATTERN",
    "IDENTIFIER_PATTERN",
    "PRIM_PATH_PATTERN",
    "build_json_schema",
    "write_json_schema",
    "verify_schema_sync",
    "validate_dict_against_schema",
    "validate_toml_file",
]


if __name__ == "__main__":
    written = write_json_schema()
    sync_errs = verify_schema_sync()
    if sync_errs:
        for e in sync_errs:
            print(f"[SCHEMA SYNC ERROR] {e}")
        raise SystemExit(1)
    print(f"[SCHEMA] wrote {written}")
