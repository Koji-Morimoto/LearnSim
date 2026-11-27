"""URDF / USD → 正規化 usda → 結合ファイル → 順運動学検証 の冪等なパイプライン公開パッケージ。

責務:
  - パッケージ `myext.robot.assembly_pipeline` の公開 API を一箇所に集約する。
  - Isaac Sim / pxr を使わない軽量な部品（仕様読み込み・事前チェック・URDF モデル・JSON Schema）は即時 import する。
  - Kit / pxr に依存する部品（ビルド・結合・検証・記述子抽出）は `__getattr__` で初回参照時のみ遅延 import する。

主な入出力・公開 API:
  - 即時公開: `load_spec`, `PipelineSpec`, `check_links`, `describe`, `load_urdf`, `UrdfModel`, `kinds` の型群, `schema` 関連 API
  - 遅延公開(`_LAZY`): `run_pipeline`, `build_assembly`, `compose_file`, `build_part`, `build_usd_part`,
    `AssembledRobotDescriptor`, `discover_components`, `inspect_assembled_robot`, `spawn_assembled_robot`,
    `check_against_physx`, `RULES`, `evaluate_all`, `reload_layers_under`, `has_api`, `world_anchors` など

呼ばれ元:
  - `run_pipeline.py`（CLI 入口）
  - `tests_kit/run_tests.py`, `tests_kit/test_official_motion_match.py`
  - 外部スクリプトおよびユーザーコード（例: `from myext.robot.assembly_pipeline import load_spec, run_pipeline`）

拡張・変更ポイント:
  - Kit / pxr 依存の新しい公開シンボルを追加する場合は、`_LAZY` 辞書に登録するだけで
    `--check` などの純 Python 実行経路を重くせずに公開できる。
"""

from __future__ import annotations

from . import spec as _spec
from .kinds import FixMode, OnViolation, PhysicsVariant, RuleId, Verdict
from .spec import (
    JoinFileSpec,
    JoinSpec,
    PipelineSpec,
    ProbeSettings,
    RuleSetting,
    UrdfSource,
    UsdSource,
    join_id,
    load_spec,
)
from .schema import (
    DEFAULT_SCHEMA_PATH,
    build_json_schema,
    validate_dict_against_schema,
    validate_toml_file,
    verify_schema_sync,
    write_json_schema,
)
from .spec_check import check_links, describe
from .urdf_model import UrdfGeometry, UrdfJoint, UrdfLink, UrdfMimic, UrdfModel, load_urdf

DEFAULT_RULES = getattr(_spec, "DEFAULT_RULES", {})

# Kit / pxr 依存シンボルの遅延ロード表: {公開シンボル名: サブモジュール名}。
# パッケージ import 時点で pxr / omni が存在しない環境(例: `--check` や単体ツール)でも
# `load_spec` や `check_links` が安全に動作するように分離している。
_LAZY = {
    "run_pipeline": "pipeline",
    "build_assembly": "pipeline",
    "PipelineStopped": "pipeline",
    "PipelineResult": "pipeline",
    "compose_file": "compose",
    "FileComposition": "compose",
    "JoinResult": "compose",
    "build_part": "part_import",
    "build_usd_part": "part_import",
    "PartBuild": "part_import",
    "AssembledRobotDescriptor": "descriptor",
    "discover_components": "descriptor",
    "components_from_robot_api": "descriptor",
    "inspect_assembled_robot": "descriptor",
    "spawn_assembled_robot": "descriptor",
    "check_against_physx": "kinematics_check",
    "check_kinematics_against_physx": "kinematics_check",
    "RULES": "rules",
    "evaluate_all": "rules",
    "format_table": "rules",
    "reload_layers_under": "usd_util",
    "has_api": "usd_util",
    "world_anchors": "usd_util",
}


def __getattr__(name: str):
    """`_LAZY` に登録された公開シンボルを初回アクセス時に import し、`globals()` にキャッシュする。

    引数:
      name: 参照された属性名。

    戻り値:
      対象サブモジュールから解決されたオブジェクト。

    例外:
      AttributeError: `_LAZY` に存在しない属性名が参照された場合。
    """
    if name in _LAZY:
        import importlib

        mod = importlib.import_module(f".{_LAZY[name]}", __name__)
        val = getattr(mod, name)
        # 2 回目以降の属性参照で `__getattr__` を再実行しないようモジュール名前空間へキャッシュする。
        globals()[name] = val
        return val
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "FixMode",
    "OnViolation",
    "PhysicsVariant",
    "RuleId",
    "Verdict",
    "JoinFileSpec",
    "JoinSpec",
    "UrdfSource",
    "UsdSource",
    "PipelineSpec",
    "ProbeSettings",
    "RuleSetting",
    "DEFAULT_RULES",
    "join_id",
    "load_spec",
    "DEFAULT_SCHEMA_PATH",
    "build_json_schema",
    "write_json_schema",
    "verify_schema_sync",
    "validate_dict_against_schema",
    "validate_toml_file",
    "check_links",
    "describe",
    "UrdfGeometry",
    "UrdfMimic",
    "UrdfJoint",
    "UrdfLink",
    "UrdfModel",
    "load_urdf",
    *_LAZY,
]
