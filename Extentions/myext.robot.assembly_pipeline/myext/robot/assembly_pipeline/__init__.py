"""URDF → usda → 結合ファイル → 幾何モード の冪等なパイプラインと、事前チェック / 事後確認。

    from myext.robot.assembly_pipeline import load_spec, run_pipeline
    result = run_pipeline(load_spec("robot.toml"))

headless の Isaac Sim から呼ぶ(run_pipeline.py を参照)。Kit に触る部品は初めて名前を参照したときに読み込む。
"""

from .kinds import FixMode, OnViolation, PhysicsVariant, RuleId, Verdict
from .spec import JoinFileSpec, JoinSpec, PipelineSpec, ProbeSettings, RuleSetting, UrdfSource, join_id, load_spec
from .urdf_model import UrdfModel, load_urdf

_LAZY = {
    "run_pipeline": "pipeline", "PipelineStopped": "pipeline", "PipelineResult": "pipeline",
    "compose_file": "compose", "FileComposition": "compose", "JoinResult": "compose",
    "make_geometric_robot": "geometric_bridge", "components_from_robot_api": "geometric_bridge",
    "RULES": "rules", "world_anchors": "usd_util",
}


def __getattr__(name: str):
    if name in _LAZY:
        import importlib

        return getattr(importlib.import_module(f".{_LAZY[name]}", __name__), name)
    raise AttributeError(name)


__all__ = ["FixMode", "OnViolation", "PhysicsVariant", "RuleId", "Verdict", "JoinFileSpec", "JoinSpec", "UrdfSource",
           "PipelineSpec", "ProbeSettings", "RuleSetting", "join_id", "load_spec", "UrdfModel", "load_urdf", *_LAZY]
