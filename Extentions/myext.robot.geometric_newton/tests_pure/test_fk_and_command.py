"""まとめて計算する順運動学と、コマンド変換のテスト(numpy だけ)。"""

from __future__ import annotations

import unittest

import numpy as np
from helpers import random_tree, six_axis_mdh

from myext.robot.geometric_newton.model import (
    FunctionCommandConverter,
    InterpCommandConverter,
    LinearCommandConverter,
)
from myext.robot.geometric_newton.model.command import check_angle_unit


class BatchedForwardKinematicsTest(unittest.TestCase):
    def test_matches_reference_on_random_trees(self) -> None:
        """木の深さごとにまとめる実装が、リンクを 1 個ずつたどる実装と一致する(枝分かれ・fixed・直動・mimic を含む)。"""
        rng = np.random.default_rng(20)
        for trial in range(30):
            model = random_tree(rng, int(rng.integers(2, 25)), fixed_ratio=0.25, with_mimic=bool(trial % 2))
            q = rng.uniform(-2, 2, size=(3, 4, model.num_dofs))
            np.testing.assert_allclose(model.forward_kinematics(q), model.forward_kinematics_reference(q), atol=1e-12, err_msg=f"trial {trial}")
            np.testing.assert_allclose(model.forward_kinematics(q[0, 0]), model.forward_kinematics_reference(q[0, 0]), atol=1e-12)

    def test_levels_follow_tree_depth(self) -> None:
        model = six_axis_mdh().to_model()
        levels = model._c["fk_levels"]
        self.assertEqual(len(levels), model.num_links - 1)  # 直列なので深さ = リンク数 - 1
        branched = random_tree(np.random.default_rng(3), 40)
        self.assertLess(len(branched._c["fk_levels"]), branched.num_links - 1)  # 枝分かれがあれば、ループの回数はリンク数より少ない

    def test_no_dofs(self) -> None:
        from myext.robot.geometric_newton.model import GeometricJointSpec, GeometricLinkSpec, GeometricRobotModel
        from myext.robot.geometric_newton.model import transforms as tf

        model = GeometricRobotModel("m", (GeometricLinkSpec("a"), GeometricLinkSpec("b")), (GeometricJointSpec("f", "fixed", "a", "b", parent_frame=tf.translation(x=1.0)),))
        self.assertEqual(model.num_dofs, 0)
        np.testing.assert_allclose(model.forward_kinematics()[1][:3, 3], [1.0, 0.0, 0.0])


class CommandConverterTest(unittest.TestCase):
    def test_linear_forward_inverse_and_batch(self) -> None:
        conv = LinearCommandConverter(["grip"], ["left_jaw", "right_jaw"], [[0.5], [-0.5]], offset=[0.01, -0.01])
        np.testing.assert_allclose(conv.to_dof_positions([0.04]), [0.03, -0.03])
        np.testing.assert_allclose(conv.from_dof_positions([0.03, -0.03]), [0.04])
        batch = conv.to_dof_positions(np.array([[0.0], [0.02], [0.04]]))
        self.assertEqual(batch.shape, (3, 2))
        np.testing.assert_allclose(conv.from_dof_positions(batch)[:, 0], [0.0, 0.02, 0.04])

    def test_linear_without_unique_inverse(self) -> None:
        conv = LinearCommandConverter(["a", "b"], ["j"], [[1.0, 1.0]])  # 2 入力 → 1 出力: 入力は決まらない
        with self.assertRaises(NotImplementedError):
            conv.from_dof_positions([0.3])

    def test_interp(self) -> None:
        conv = InterpCommandConverter("opening", ["j1", "j2"], [0.0, 0.5, 1.0], [[0.0, 0.0], [0.2, -0.1], [0.8, -0.4]], output_for_inverse="j1")
        np.testing.assert_allclose(conv.to_dof_positions([0.25]), [0.1, -0.05])
        np.testing.assert_allclose(conv.to_dof_positions([0.75]), [0.5, -0.25])
        np.testing.assert_allclose(conv.to_dof_positions([2.0]), [0.8, -0.4])  # 表の外は端の値
        np.testing.assert_allclose(conv.to_dof_positions(np.array([[0.25], [0.75]])), [[0.1, -0.05], [0.5, -0.25]])
        np.testing.assert_allclose(conv.from_dof_positions([0.5, -0.25]), [0.75])
        with self.assertRaises(ValueError):
            InterpCommandConverter("x", ["j"], [0.0, 0.0], [[0.0], [1.0]])  # 単調増加でない
        with self.assertRaises(ValueError):
            InterpCommandConverter("x", ["j"], [0.0, 0.5, 1.0], [[0.0], [1.0], [0.5]], output_for_inverse="j")  # 単調でない出力

    def test_function(self) -> None:
        conv = FunctionCommandConverter(["w"], ["a", "b"], forward=lambda x: np.concatenate([x / 2, -x / 2], axis=-1), inverse=lambda y: y[..., :1] * 2)
        np.testing.assert_allclose(conv.to_dof_positions([0.4]), [0.2, -0.2])
        np.testing.assert_allclose(conv.from_dof_positions([0.2, -0.2]), [0.4])
        bad = FunctionCommandConverter(["w"], ["a", "b"], forward=lambda x: x)
        with self.assertRaises(ValueError):
            bad.to_dof_positions([0.4])
        with self.assertRaises(NotImplementedError):
            bad.from_dof_positions([0.1, 0.2])

    def test_input_checks(self) -> None:
        with self.assertRaises(ValueError):
            LinearCommandConverter(["a", "a"], ["j"], [[1.0, 1.0]])
        conv = LinearCommandConverter(["a"], ["j"], [[1.0]])
        with self.assertRaises(ValueError):
            conv.to_dof_positions([1.0, 2.0])
        with self.assertRaises(ValueError):
            LinearCommandConverter(["a"], ["j"], [[1.0]], output_angle_unit="degree")  # type: ignore[arg-type]
        self.assertEqual(check_angle_unit("deg"), "deg")
        with self.assertRaises(ValueError):
            check_angle_unit("radian")


if __name__ == "__main__":
    unittest.main()
