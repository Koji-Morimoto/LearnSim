"""ロボットの生成元になるパラメータ(すべてデータクラス)と、内部モデルへの変換。

    DHParameters      標準 DH。      リンク i の変換 = Rz(theta) Tz(d) Tx(a) Rx(alpha)。関節 i は z_{i-1} 軸まわり
    MDHParameters     修正 DH。      リンク i の変換 = Rx(alpha) Tx(a) Rz(theta) Tz(d)。関節 i は z_i 軸まわり
    ScrewParameters   スクリュー軸。 手先の姿勢 = exp(S_1 q_1) ... exp(S_n q_n) × tool_home(軸はすべて根の座標系で与える)
    UrdfSource        URDF ファイル

どれも to_model() で GeometricRobotModel になる。直列ロボットのリンク名は link0(根), link1, ..., tool。
関節名は行の name(省略時は joint1, joint2, ...)。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Sequence, get_args

import numpy as np

from . import transforms as tf
from .spec import GeometricJointSpec, GeometricLinkSpec, GeometricRobotModel

__all__ = [
    "MovableJointType",
    "DHRow",
    "DHParameters",
    "MDHRow",
    "MDHParameters",
    "ScrewAxisRow",
    "ScrewParameters",
    "UrdfSource",
]

MovableJointType = Literal["revolute", "prismatic"]


def _check_type(joint_type: str) -> None:
    if joint_type not in get_args(MovableJointType):
        raise ValueError(f"joint_type は {get_args(MovableJointType)} のどれか: {joint_type!r}")


def _matrix(value: np.ndarray | None) -> np.ndarray:
    out = np.eye(4) if value is None else np.array(value, dtype=float)
    if out.shape != (4, 4):
        raise ValueError("base / tool は 4x4 行列")
    return out


def _serial(name: str, joints: list[GeometricJointSpec], n: int, tool: np.ndarray, tool_parent_frame: np.ndarray) -> GeometricRobotModel:
    links = [GeometricLinkSpec(f"link{i}") for i in range(n + 1)] + [GeometricLinkSpec("tool")]
    joints = joints + [GeometricJointSpec("tool_fixed", "fixed", f"link{n}", "tool", parent_frame=tool_parent_frame @ tool)]
    return GeometricRobotModel(name=name, links=tuple(links), joints=tuple(joints), root_link="link0")


@dataclass(frozen=True)
class DHRow:
    """標準 DH の 1 行。revolute は theta に、prismatic は d に関節値が足される。角度は rad。"""

    theta: float = 0.0
    d: float = 0.0
    a: float = 0.0
    alpha: float = 0.0
    joint_type: MovableJointType = "revolute"
    lower_limit: float = -np.inf
    upper_limit: float = np.inf
    name: str | None = None

    def __post_init__(self) -> None:
        _check_type(self.joint_type)

    def matrix(self) -> np.ndarray:
        return tf.rot_z(self.theta) @ tf.translation(z=self.d) @ tf.translation(x=self.a) @ tf.rot_x(self.alpha)


@dataclass(frozen=True)
class DHParameters:
    """標準 DH。base は根リンク座標系で見た DH の 0 番座標系、tool は最後の DH 座標系で見た手先。"""

    rows: Sequence[DHRow]
    base: np.ndarray | None = None
    tool: np.ndarray | None = None
    name: str = "dh_robot"

    def to_model(self) -> GeometricRobotModel:
        joints = []
        for i, row in enumerate(self.rows, start=1):
            # Rz(theta + q) Tz(d) Tx(a) Rx(alpha) = 動き(q) × row.matrix()。prismatic の Tz(q) も Rz(theta) と交換できる
            joints.append(
                GeometricJointSpec(
                    name=row.name or f"joint{i}",
                    joint_type=row.joint_type,
                    parent_link=f"link{i - 1}",
                    child_link=f"link{i}",
                    parent_frame=_matrix(self.base) if i == 1 else np.eye(4),
                    child_frame=tf.invert(row.matrix()),
                    axis=(0.0, 0.0, 1.0),
                    lower_limit=row.lower_limit,
                    upper_limit=row.upper_limit,
                )
            )
        return _serial(self.name, joints, len(self.rows), _matrix(self.tool), np.eye(4))


@dataclass(frozen=True)
class MDHRow:
    """修正 DH の 1 行(alpha と a は 1 つ前の軸から測る)。revolute は theta に、prismatic は d に関節値が足される。"""

    alpha: float = 0.0
    a: float = 0.0
    theta: float = 0.0
    d: float = 0.0
    joint_type: MovableJointType = "revolute"
    lower_limit: float = -np.inf
    upper_limit: float = np.inf
    name: str | None = None

    def __post_init__(self) -> None:
        _check_type(self.joint_type)

    def matrix(self) -> np.ndarray:
        return tf.rot_x(self.alpha) @ tf.translation(x=self.a) @ tf.rot_z(self.theta) @ tf.translation(z=self.d)


@dataclass(frozen=True)
class MDHParameters:
    """修正 DH。base は根リンク座標系で見た 0 番座標系、tool は最後の座標系で見た手先。"""

    rows: Sequence[MDHRow]
    base: np.ndarray | None = None
    tool: np.ndarray | None = None
    name: str = "mdh_robot"

    def to_model(self) -> GeometricRobotModel:
        joints = []
        for i, row in enumerate(self.rows, start=1):
            # Rx(alpha) Tx(a) Rz(theta + q) Tz(d) = [Rx Tx] × 動き(q) × [Rz(theta) Tz(d)]
            before = tf.rot_x(row.alpha) @ tf.translation(x=row.a)
            after = tf.rot_z(row.theta) @ tf.translation(z=row.d)
            joints.append(
                GeometricJointSpec(
                    name=row.name or f"joint{i}",
                    joint_type=row.joint_type,
                    parent_link=f"link{i - 1}",
                    child_link=f"link{i}",
                    parent_frame=(_matrix(self.base) if i == 1 else np.eye(4)) @ before,
                    child_frame=tf.invert(after),
                    axis=(0.0, 0.0, 1.0),
                    lower_limit=row.lower_limit,
                    upper_limit=row.upper_limit,
                )
            )
        return _serial(self.name, joints, len(self.rows), _matrix(self.tool), np.eye(4))


@dataclass(frozen=True)
class ScrewAxisRow:
    """根の座標系で表した関節の軸(関節値がすべて 0 の姿勢で測る)。

    revolute : direction = 軸の向き(単位ベクトル)、moment = 軸上の点 × direction。through() なら点から作れる
    prismatic: direction = 進む向き、moment は使わない
    """

    direction: tuple[float, float, float]
    moment: tuple[float, float, float] = (0.0, 0.0, 0.0)
    joint_type: MovableJointType = "revolute"
    lower_limit: float = -np.inf
    upper_limit: float = np.inf
    name: str | None = None

    def __post_init__(self) -> None:
        _check_type(self.joint_type)
        d = np.asarray(self.direction, dtype=float)
        norm = np.linalg.norm(d)
        if norm < 1e-12:
            raise ValueError("direction の長さが 0")
        scale = 1.0 / norm
        object.__setattr__(self, "direction", tuple(float(v) for v in d * scale))
        object.__setattr__(self, "moment", tuple(float(v) for v in np.asarray(self.moment, dtype=float) * scale))

    @staticmethod
    def through(
        point: Sequence[float],
        direction: Sequence[float],
        joint_type: MovableJointType = "revolute",
        lower_limit: float = -np.inf,
        upper_limit: float = np.inf,
        name: str | None = None,
    ) -> "ScrewAxisRow":
        """軸が通る点と向きから作る。"""
        d = np.asarray(direction, dtype=float)
        d = d / np.linalg.norm(d)
        return ScrewAxisRow(tuple(d), tuple(np.cross(np.asarray(point, dtype=float), d)), joint_type, lower_limit, upper_limit, name)

    @property
    def point(self) -> np.ndarray:
        """軸上の点のうち、原点にいちばん近い点。"""
        return np.cross(np.asarray(self.direction), np.asarray(self.moment))


@dataclass(frozen=True)
class ScrewParameters:
    """スクリュー軸(指数積)。tool_home は関節値がすべて 0 のときの手先の姿勢(根の座標系)。"""

    rows: Sequence[ScrewAxisRow]
    tool_home: np.ndarray | None = None
    name: str = "screw_robot"

    def to_model(self) -> GeometricRobotModel:
        # 関節値が 0 のとき全リンクの座標系を根の座標系に重ねて置くと、
        # 子リンク = 親リンク × exp(S q) となり、exp(S q) = 平行移動(軸上の点) × 回転(q) × 平行移動(−軸上の点)
        joints = []
        for i, row in enumerate(self.rows, start=1):
            at_axis = tf.make_transform(position=row.point) if row.joint_type == "revolute" else np.eye(4)
            joints.append(
                GeometricJointSpec(
                    name=row.name or f"joint{i}",
                    joint_type=row.joint_type,
                    parent_link=f"link{i - 1}",
                    child_link=f"link{i}",
                    parent_frame=at_axis,
                    child_frame=at_axis,
                    axis=row.direction,
                    lower_limit=row.lower_limit,
                    upper_limit=row.upper_limit,
                )
            )
        return _serial(self.name, joints, len(self.rows), _matrix(self.tool_home), np.eye(4))


@dataclass(frozen=True)
class UrdfSource:
    """URDF ファイル。運動学(関節の位置・軸・種類・可動範囲・mimic)だけを読む。メッシュや質量は読まない。"""

    urdf_path: str
    name: str | None = None

    def to_model(self) -> GeometricRobotModel:
        from .urdf import read_urdf

        return read_urdf(self.urdf_path, name=self.name)
