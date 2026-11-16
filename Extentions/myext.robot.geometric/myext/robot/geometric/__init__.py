"""幾何モードのロボット API。

物理ソルバーも Drive も使わず、関節値からリンクの Transform を計算して書く。
Sublayer(GeometricLayer)を付けると幾何モード、外すと元の Articulation 構成に戻る。元の USD は書き換えない。

    from myext.robot.geometric import GeometricRobot, MDHParameters, MDHRow

numpy だけの部分(model 以下)は Isaac Sim の外でも import できる。USD / Kit に触るクラス
(GeometricRobot など)は、初めて名前を参照したときに読み込む。
"""

from .model import *  # noqa: F401,F403
from .model import __all__ as _model_all

_LAZY = {
    "UsdArticulationSource": "usd_reader",
    "UsdRobotInfo": "usd_reader",
    "read_articulation": "usd_reader",
    "GeometricLayer": "layer",
    "GeometricLayerOptions": "layer",
    "GeometricLayerReport": "layer",
    "CollisionMode": "layer",
    "LayerTarget": "layer",
    "GeometricTransformWriter": "writer",
    "GeometricUsdWriter": "writer",
    "GeometricFabricWriter": "writer",
    "WriteTarget": "writer",
    "GeometricRobot": "robot",
    "GeometricRobotComponent": "robot",
    "GeometricJoint": "robot",
    "GeometricLink": "robot",
    "LimitPolicy": "robot",
    "GeometricRobotView": "robot_view",
    "GeometricCollisionChecker": "collision",
    "GeometricCollisionPair": "collision",
    "GeometricLinkSet": "collision",
    "GeometricContactMonitor": "collision",
    "CollisionScope": "collision",
    "GeometricVisualStyle": "builder",
    "build_robot_prims": "builder",
}

__all__ = list(_model_all) + list(_LAZY)


def __getattr__(name: str):
    if name in _LAZY:
        import importlib

        module = importlib.import_module(f"{__name__}.{_LAZY[name]}")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
