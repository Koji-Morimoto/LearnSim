"""テスト用の部品(numpy だけ)。"""

from __future__ import annotations

import os
import sys

import numpy as np

EXT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if EXT_ROOT not in sys.path:
    sys.path.insert(0, EXT_ROOT)

from myext.robot.geometric_newton.model import (  # noqa: E402
    GeometricJointSpec,
    GeometricLinkSpec,
    GeometricMimicSpec,
    GeometricRobotModel,
    MDHParameters,
    MDHRow,
)
from myext.robot.geometric_newton.model import transforms as tf  # noqa: E402

FRANKA_URDF = r"C:\isaacsim-6.2.0\exts\isaacsim.asset.importer.urdf\data\urdf\robots\franka_description\robots\panda_arm_hand.urdf"
UR10_URDF = r"C:\isaacsim-6.2.0\exts\isaacsim.asset.importer.urdf\data\urdf\robots\ur10\urdf\ur10.urdf"


def random_transform(rng: np.random.Generator, scale: float = 0.3) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    return tf.make_transform(tf.rotation_about_axis(axis, rng.uniform(-np.pi, np.pi)), rng.uniform(-scale, scale, size=3))


def random_tree(rng: np.random.Generator, num_links: int = 9, fixed_ratio: float = 0.2, with_mimic: bool = True) -> GeometricRobotModel:
    """枝分かれ・fixed・prismatic・mimic(連鎖を含む)を混ぜたランダムな木。"""
    links = [GeometricLinkSpec(f"L{i}") for i in range(num_links)]
    joints: list[GeometricJointSpec] = []
    for i in range(1, num_links):
        parent = int(rng.integers(0, i))
        kind = "fixed" if rng.random() < fixed_ratio else ("prismatic" if rng.random() < 0.3 else "revolute")
        axis = rng.normal(size=3)
        joints.append(
            GeometricJointSpec(
                name=f"J{i}",
                joint_type=kind,  # type: ignore[arg-type]
                parent_link=f"L{parent}",
                child_link=f"L{i}",
                parent_frame=random_transform(rng),
                child_frame=random_transform(rng),
                axis=tuple(axis),
                lower_limit=-2.5,
                upper_limit=2.5,
            )
        )
    if with_mimic:
        dofs = [k for k, j in enumerate(joints) if j.is_dof]
        if len(dofs) >= 4:
            # dofs[1] は dofs[0] に、dofs[3] は dofs[1] に従う(連鎖)
            joints[dofs[1]] = joints[dofs[1]].replaced(mimic=GeometricMimicSpec(joints[dofs[0]].name, -0.7, 0.2))
            joints[dofs[3]] = joints[dofs[3]].replaced(mimic=GeometricMimicSpec(joints[dofs[1]].name, 1.5, -0.1))
    return GeometricRobotModel(name="random", links=tuple(links), joints=tuple(joints), root_link="L0")


def six_axis_mdh() -> MDHParameters:
    """肩・肘・手首を持つ 6 軸(寸法はメートル)。"""
    pi = np.pi
    lim = dict(lower_limit=-pi, upper_limit=pi)
    return MDHParameters(
        rows=[
            MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.40, **lim),
            MDHRow(alpha=-pi / 2, a=0.05, theta=-pi / 2, d=0.0, **lim),
            MDHRow(alpha=0.0, a=0.45, theta=0.0, d=0.0, **lim),
            MDHRow(alpha=-pi / 2, a=0.04, theta=0.0, d=0.42, **lim),
            MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, **lim),
            MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.10, **lim),
        ],
        tool=tf.translation(z=0.08),
        name="six_axis",
    )


def numeric_jacobian(fk, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """fk(x) -> 4x4 の数値微分 (6, n)。行は [位置 3; 回転ベクトル 3]。"""
    base = fk(x)
    out = np.zeros((6, len(x)))
    for i in range(len(x)):
        dx = np.zeros(len(x))
        dx[i] = eps
        plus, minus = fk(x + dx), fk(x - dx)
        out[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * eps)
        out[3:, i] = tf.rotation_vector(plus[:3, :3] @ minus[:3, :3].T) / (2 * eps)
    del base
    return out
