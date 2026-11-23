"""NumPy のみで完結する純粋運動学・フィルタ・指令変換レイヤー (`myext.robot.geometric_newton.model`)。

設計上の意図:
    Isaac Sim / OpenUSD (`pxr`) / Warp / Newton に一切依存しないため、通常の Python + NumPy 環境だけで
    モデル構築 (`GeometricRobotModel`)・DH / 修正 DH / スクリュー軸 / URDF 変換・直列鎖 (`KinematicChain`)・
    IK (`JacobianIKSolver` 等)・干渉フィルタ (`GeometricCollisionFilter`)・指令変換 (`GeometricCommandConverter`)
    を単体テストおよびオフライン軌道計画に利用できる。
"""

from . import transforms
from .chain import KinematicChain, SerialJoint
from .collision_filter import GeometricCollisionFilter
from .command import AngleUnit, FunctionCommandConverter, GeometricCommandConverter, InterpCommandConverter, LinearCommandConverter
from .ik import (
    IKResult,
    IKSolver,
    IKSolverConfig,
    IKSolverName,
    JacobianIKSolver,
    OrientationMode,
    available_ik_solvers,
    create_ik_solver,
    register_ik_solver,
)
from .sources import DHParameters, DHRow, MDHParameters, MDHRow, MovableJointType, ScrewAxisRow, ScrewParameters, UrdfSource
from .spec import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, GeometricRobotModel, JointType
from .urdf import model_to_urdf, read_urdf, read_urdf_string

__all__ = [
    "transforms",
    "KinematicChain",
    "SerialJoint",
    "GeometricCollisionFilter",
    "AngleUnit",
    "GeometricCommandConverter",
    "LinearCommandConverter",
    "InterpCommandConverter",
    "FunctionCommandConverter",
    "IKResult",
    "IKSolver",
    "IKSolverConfig",
    "IKSolverName",
    "JacobianIKSolver",
    "OrientationMode",
    "available_ik_solvers",
    "create_ik_solver",
    "register_ik_solver",
    "DHParameters",
    "DHRow",
    "MDHParameters",
    "MDHRow",
    "MovableJointType",
    "ScrewAxisRow",
    "ScrewParameters",
    "UrdfSource",
    "GeometricJointSpec",
    "GeometricLinkSpec",
    "GeometricMimicSpec",
    "GeometricRobotModel",
    "JointType",
    "model_to_urdf",
    "read_urdf",
    "read_urdf_string",
]
