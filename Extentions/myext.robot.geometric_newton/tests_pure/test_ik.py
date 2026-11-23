"""IK ソルバーのテスト(numpy だけ)。ikpy が入っていれば IKPySolver も試す(無ければそのテストだけ飛ばす)。"""

from __future__ import annotations

import unittest

import numpy as np
from helpers import FRANKA_URDF, six_axis_mdh

from myext.robot.geometric_newton.model import (
    GeometricJointSpec,
    GeometricLinkSpec,
    GeometricMimicSpec,
    GeometricRobotModel,
    IKSolver,
    IKSolverConfig,
    JacobianIKSolver,
    UrdfSource,
    available_ik_solvers,
    create_ik_solver,
    register_ik_solver,
)
from myext.robot.geometric_newton.model import transforms as tf

try:
    import ikpy  # noqa: F401

    HAS_IKPY = True
except ImportError:
    HAS_IKPY = False


def mimic_arm() -> GeometricRobotModel:
    """5 関節のうち 3 番目が 2 番目に従う腕(鎖の途中に mimic がある)。"""
    links = tuple(GeometricLinkSpec(f"l{i}") for i in range(6))
    lim = dict(lower_limit=-2.0, upper_limit=2.0)
    joints = (
        GeometricJointSpec("j1", "revolute", "l0", "l1", parent_frame=tf.translation(z=0.2), axis=(0, 0, 1), **lim),
        GeometricJointSpec("j2", "revolute", "l1", "l2", parent_frame=tf.translation(z=0.1), axis=(0, 1, 0), **lim),
        GeometricJointSpec("j3", "revolute", "l2", "l3", parent_frame=tf.translation(x=0.3), axis=(0, 1, 0), mimic=GeometricMimicSpec("j2", -1.5, 0.2), **lim),
        GeometricJointSpec("j4", "revolute", "l3", "l4", parent_frame=tf.translation(x=0.3), axis=(1, 0, 0), **lim),
        GeometricJointSpec("j5", "prismatic", "l4", "l5", parent_frame=tf.translation(x=0.1), axis=(1, 0, 0), lower_limit=0.0, upper_limit=0.2),
    )
    return GeometricRobotModel("mimic_arm", links, joints)


class JacobianSolverTest(unittest.TestCase):
    def test_reaches_reachable_poses(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        solver = JacobianIKSolver(chain)
        rng = np.random.default_rng(0)
        for _ in range(25):
            truth = rng.uniform(-2.5, 2.5, size=6)
            target = chain.forward_kinematics(truth)
            result = solver.solve(target[:3, 3], tf.matrix_to_quat(target[:3, :3]), seed=np.zeros(6))
            self.assertTrue(result.success, f"pos {result.position_error} rot {result.orientation_error}")
            reached = chain.forward_kinematics(result.dof_positions)
            np.testing.assert_allclose(reached[:3, 3], target[:3, 3], atol=2e-5)
            self.assertTrue(np.all(result.dof_positions >= chain.lower_limits - 1e-12))
            self.assertTrue(np.all(result.dof_positions <= chain.upper_limits + 1e-12))

    def test_same_input_gives_same_answer(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        target = chain.forward_kinematics(np.array([0.4, -0.8, 1.1, 0.3, 0.9, -0.2]))
        a = JacobianIKSolver(chain).solve(target[:3, 3], target[:3, :3])
        b = JacobianIKSolver(chain).solve(target[:3, 3], target[:3, :3])
        np.testing.assert_array_equal(a.dof_positions, b.dof_positions)

    def test_unreachable_reports_failure(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        result = JacobianIKSolver(chain, IKSolverConfig(num_restarts=1)).solve(np.array([5.0, 0.0, 0.0]))
        self.assertFalse(result.success)
        self.assertGreater(result.position_error, 3.0)

    def test_position_only(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        target = chain.forward_kinematics(np.array([0.2, -0.5, 0.8, 0.1, 0.4, 0.0]))
        for solver in (
            JacobianIKSolver(chain, IKSolverConfig(orientation_mode="none")),
            JacobianIKSolver(chain),  # 向きを渡さなければ位置だけになる
        ):
            result = solver.solve(target[:3, 3], None if solver.config.orientation_mode == "full" else target[:3, :3])
            self.assertTrue(result.success)
            self.assertEqual(result.orientation_error, 0.0)

    def test_inactive_dofs_keep_seed(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        seed = np.array([0.1, -0.4, 0.6, 0.77, 0.3, -0.55])
        truth = seed + np.array([0.3, -0.2, 0.25, 0.0, 0.0, 0.0])
        target = chain.forward_kinematics(truth)
        solver = JacobianIKSolver(chain, IKSolverConfig(active_dof_mask=[True, True, True, False, False, False], orientation_mode="none"))
        self.assertEqual(solver.get_active_dofs(), [True, True, True, False, False, False])
        result = solver.solve(target[:3, 3], seed=seed)
        self.assertTrue(result.success)
        np.testing.assert_array_equal(result.dof_positions[3:], seed[3:])
        # 後から名前で変える
        solver.set_active_dofs(["joint1", "joint2", "joint3", "joint5"])
        self.assertEqual(solver.get_active_dofs(), [True, True, True, False, True, False])
        result = solver.solve(target[:3, 3], seed=seed)
        self.assertEqual(result.dof_positions[3], seed[3])
        self.assertEqual(result.dof_positions[5], seed[5])
        with self.assertRaises(KeyError):
            solver.set_active_dofs(["nope"])
        with self.assertRaises(ValueError):
            solver.set_active_dofs([True, False])

    def test_mimic_inside_chain_is_satisfied_exactly(self) -> None:
        model = mimic_arm()
        chain = model.get_kinematic_chain("l0", "l5")
        self.assertEqual(chain.variable_names, ("j1", "j2", "j4", "j5"))
        solver = JacobianIKSolver(chain, IKSolverConfig(orientation_mode="none"))
        rng = np.random.default_rng(1)
        for _ in range(15):
            truth = rng.uniform(chain.lower_limits, chain.upper_limits)
            target = chain.forward_kinematics(truth)
            result = solver.solve(target[:3, 3], seed=np.zeros(4))
            self.assertTrue(result.success, result.position_error)
            # 解をモデルの全 DOF に戻すと、従動軸は必ず mimic の式どおりで、手先も目標に来ている
            full = np.zeros(model.num_dofs)
            full[chain.dof_indices] = result.dof_positions
            full = model.resolve_mimic(full)
            self.assertAlmostEqual(full[2], -1.5 * full[1] + 0.2, places=12)
            np.testing.assert_allclose(model.forward_kinematics(full)[model.get_link_index("l5")][:3, 3], target[:3, 3], atol=2e-5)

    def test_registry(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        self.assertIsInstance(create_ik_solver("jacobian", chain), JacobianIKSolver)

        class Dummy(IKSolver):
            def solve(self, target_position, target_orientation=None, *, seed=None):
                return self._result(self._seed(seed), self._target_matrix(target_position, target_orientation), 0, False)

        register_ik_solver("dummy", Dummy)
        self.assertIn("dummy", available_ik_solvers())
        self.assertIsInstance(create_ik_solver("dummy", chain), Dummy)
        with self.assertRaises(KeyError):
            create_ik_solver("nope", chain)


@unittest.skipUnless(HAS_IKPY, "ikpy が入っていない")
class IKPySolverTest(unittest.TestCase):
    def test_reaches_reachable_poses(self) -> None:
        chain = UrdfSource(FRANKA_URDF).to_model().get_kinematic_chain("panda_link0", "panda_hand")
        solver = create_ik_solver("ikpy", chain, IKSolverConfig(position_tolerance=1e-3, orientation_tolerance=1e-2))
        rng = np.random.default_rng(0)
        ok = 0
        for _ in range(10):
            truth = rng.uniform(chain.lower_limits, chain.upper_limits)
            target = chain.forward_kinematics(truth)
            result = solver.solve(target[:3, 3], target[:3, :3], seed=truth + rng.normal(scale=0.2, size=7))
            ok += result.success
        self.assertGreaterEqual(ok, 8)

    def test_mask_keeps_seed(self) -> None:
        chain = six_axis_mdh().to_model().get_kinematic_chain("link0", "tool")
        seed = np.array([0.1, -0.4, 0.6, 0.77, 0.3, -0.55])
        target = chain.forward_kinematics(seed + np.array([0.3, -0.2, 0.25, 0.0, 0.0, 0.0]))
        solver = create_ik_solver("ikpy", chain, IKSolverConfig(active_dof_mask=[True, True, True, False, False, False], orientation_mode="none", position_tolerance=1e-3))
        result = solver.solve(target[:3, 3], seed=seed)
        self.assertTrue(result.success, result.position_error)
        np.testing.assert_array_equal(result.dof_positions[3:], seed[3:])

    def test_rejects_mimic_inside_chain(self) -> None:
        with self.assertRaises(ValueError):
            create_ik_solver("ikpy", mimic_arm().get_kinematic_chain("l0", "l5"))


if __name__ == "__main__":
    unittest.main()
