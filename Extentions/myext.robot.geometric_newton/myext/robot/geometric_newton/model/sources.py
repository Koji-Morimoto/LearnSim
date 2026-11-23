"""ロボットの生成元パラメータ (標準 DH / 修正 DH / スクリュー軸 PoE / URDF) と `GeometricRobotModel` への統一変換。

すべての生成元が `to_model()` を実装し、共通の関節式:
    `X_wc = X_wp @ parent_frame @ motion(q) @ inv(child_frame)`
を持つ `GeometricRobotModel` へ正規化されるため、下流の順運動学・IK・`build_robot_prims`・
`GeometricScene` (Newton バックエンド) は生成元の違いを意識せず同一コードで扱える。

各規約の比較:
    - `DHParameters` (標準 DH / Denavit-Hartenberg):
        `T_i = Rz(theta_i) @ Tz(d_i) @ Tx(a_i) @ Rx(alpha_i)`。関節 `i` は `z_{i-1}` 軸まわり。
    - `MDHParameters` (修正 DH / Craig 規約):
        `T_i = Rx(alpha_{i-1}) @ Tx(a_{i-1}) @ Rz(theta_i) @ Tz(d_i)`。関節 `i` は `z_i` 軸まわり。
    - `ScrewParameters` (スクリュー軸 / 指数積 PoE: Product of Exponentials):
        `T_ee(q) = exp(S_1 q_1) @ ... @ exp(S_n q_n) @ tool_home`。全軸を `q = 0` 時の根リンク座標系で与える。
    - `UrdfSource`:
        URDF XML ファイルから運動学 (関節の origin / axis / type / limit / mimic) のみを読み取る。

直列生成元 (`DHParameters` / `MDHParameters` / `ScrewParameters`) のリンク名は `link0` (根), `link1`, ..., `link{n}`, `tool`、
関節名は各行の `name` (省略時は `joint1`, `joint2`, ...) と末尾の固定関節 `tool_fixed` になる。
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

# ==============================================================================
# 共通ヘルパー
# ==============================================================================

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
    """`n` 軸直列関節の末尾へ `tool` リンク (`tool_fixed`) を付けて `GeometricRobotModel` を組む。"""
    links = [GeometricLinkSpec(f"link{i}") for i in range(n + 1)] + [GeometricLinkSpec("tool")]
    joints = joints + [GeometricJointSpec("tool_fixed", "fixed", f"link{n}", "tool", parent_frame=tool_parent_frame @ tool)]
    return GeometricRobotModel(name=name, links=tuple(links), joints=tuple(joints), root_link="link0")


# ==============================================================================
# 標準 DH (Denavit-Hartenberg)
# ==============================================================================


@dataclass(frozen=True)
class DHRow:
    """標準 DH の 1 行。`revolute` は `theta` に、`prismatic` は `d` に関節値 `q` が加算される。角度は `rad`。"""

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
        """`q = 0` 時のリンク間変換 `Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)` を返す。"""
        return tf.rot_z(self.theta) @ tf.translation(z=self.d) @ tf.translation(x=self.a) @ tf.rot_x(self.alpha)


@dataclass(frozen=True)
class DHParameters:
    """標準 DH パラメータ。`base` は根リンク座標系で見た DH 第 0 座標系、`tool` は最終 DH 座標系で見た手先。"""

    rows: Sequence[DHRow]
    base: np.ndarray | None = None
    tool: np.ndarray | None = None
    name: str = "dh_robot"

    def to_model(self) -> GeometricRobotModel:
        joints = []
        for i, row in enumerate(self.rows, start=1):
            # 標準 DH の変換は `Rz(theta + q) @ Tz(d) @ Tx(a) @ Rx(alpha) = motion_z(q) @ row.matrix()` と分解できる
            # (直動 `Tz(q)` も Z 軸回転 `Rz(theta)` と可換なので同じ形になる)。
            # `X_wc = X_wp @ parent_frame @ motion(q) @ inv(child_frame)` と照合すると
            # `child_frame = inv(row.matrix())` と置けば `inv(child_frame) = row.matrix()` が厳密に得られる。
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


# ==============================================================================
# 修正 DH (Craig 規約)
# ==============================================================================


@dataclass(frozen=True)
class MDHRow:
    """修正 DH の 1 行 (`alpha`, `a` は 1 つ前の軸から測る)。`revolute` は `theta` に、`prismatic` は `d` に `q` を加算。"""

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
        """`q = 0` 時のリンク間変換 `Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)` を返す。"""
        return tf.rot_x(self.alpha) @ tf.translation(x=self.a) @ tf.rot_z(self.theta) @ tf.translation(z=self.d)


@dataclass(frozen=True)
class MDHParameters:
    """修正 DH パラメータ。`base` は根リンク座標系で見た第 0 座標系、`tool` は最終座標系で見た手先。"""

    rows: Sequence[MDHRow]
    base: np.ndarray | None = None
    tool: np.ndarray | None = None
    name: str = "mdh_robot"

    def to_model(self) -> GeometricRobotModel:
        joints = []
        for i, row in enumerate(self.rows, start=1):
            # 修正 DH の変換は `Rx(alpha) @ Tx(a) @ Rz(theta + q) @ Tz(d) = before @ motion_z(q) @ after` と分解できる。
            # よって `parent_frame = before`、`child_frame = inv(after)` と置くことで共通関節式と厳密に一致する。
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


# ==============================================================================
# スクリュー軸 (指数積 PoE: Product of Exponentials)
# ==============================================================================


@dataclass(frozen=True)
class ScrewAxisRow:
    """`q = 0` 時の根リンク座標系で表した関節のスクリュー軸。

    - `revolute` : `direction` = 単位方向ベクトル `omega`、`moment = point x direction` (`through()` で点と向きから生成可)。
    - `prismatic`: `direction` = 進行方向の単位ベクトル `v`、`moment` は不使用。
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
        # `direction` を単位ベクトル化するとき、`moment = p x d` も同じ倍率で割って軸上の点 `p` を保つ
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
        """軸上の 1 点 `point` と向き `direction` から `ScrewAxisRow` を生成する。"""
        d = np.asarray(direction, dtype=float)
        d = d / np.linalg.norm(d)
        return ScrewAxisRow(tuple(d), tuple(np.cross(np.asarray(point, dtype=float), d)), joint_type, lower_limit, upper_limit, name)

    @property
    def point(self) -> np.ndarray:
        """Plucker 座標 `(direction, moment)` から復元した、軸上で原点に最も近い点 (`direction x moment`)。"""
        return np.cross(np.asarray(self.direction), np.asarray(self.moment))


@dataclass(frozen=True)
class ScrewParameters:
    """スクリュー軸 (PoE) パラメータ。`tool_home` は全関節値 0 のときの手先姿勢 (根リンク座標系)。"""

    rows: Sequence[ScrewAxisRow]
    tool_home: np.ndarray | None = None
    name: str = "screw_robot"

    def to_model(self) -> GeometricRobotModel:
        # なぜ `parent_frame = child_frame = T(point)` と置くか:
        # `q = 0` のとき全中間リンク座標系を根リンク座標系に一致させておくと、回転関節の指数写像
        # `exp(S q) = T(point) @ Rot(direction, q) @ inv(T(point))` が共通関節式
        # `parent_frame @ motion(q) @ inv(child_frame)` そのものになり、直列積が
        # `exp(S_1 q_1) @ ... @ exp(S_n q_n) @ tool_home` と厳密に一致する。
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


# ==============================================================================
# URDF ソース
# ==============================================================================


@dataclass(frozen=True)
class UrdfSource:
    """URDF ファイル生成元。運動学 (関節の位置・軸・種類・可動範囲・mimic) のみ読み、メッシュや質量は読まない。"""

    urdf_path: str
    name: str | None = None

    def to_model(self) -> GeometricRobotModel:
        from .urdf import read_urdf

        return read_urdf(self.urdf_path, name=self.name)
