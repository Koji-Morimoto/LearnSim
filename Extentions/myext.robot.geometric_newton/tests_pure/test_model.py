"""内部モデル・生成元・鎖・URDF・干渉フィルタのテスト(numpy だけ。Isaac Sim は要らない)。

実行: <pixi の python> -m unittest discover -s tests_pure -v
"""

from __future__ import annotations

import os
import tempfile
import unittest

import numpy as np
from helpers import FRANKA_URDF, UR10_URDF, numeric_jacobian, random_transform, random_tree, six_axis_mdh

from myext.robot.geometric_newton.model import (
    DHParameters,
    DHRow,
    GeometricCollisionFilter,
    GeometricJointSpec,
    GeometricLinkSpec,
    GeometricMimicSpec,
    GeometricRobotModel,
    MDHParameters,
    MDHRow,
    ScrewAxisRow,
    ScrewParameters,
    UrdfSource,
    read_urdf_string,
)
from myext.robot.geometric_newton.model import transforms as tf

try:
    import pinocchio as pin
except ImportError:  # pinocchio が無い環境では、突き合わせのテストだけ飛ばす
    pin = None


class TransformsTest(unittest.TestCase):
    def test_quaternion_round_trip(self) -> None:
        rng = np.random.default_rng(0)
        q = rng.normal(size=(500, 4))
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        q = np.where(q[:, :1] < 0, -q, q)
        np.testing.assert_allclose(tf.matrix_to_quat(tf.quat_to_matrix(q)), q, atol=1e-12)

    def test_rpy_round_trip(self) -> None:
        rng = np.random.default_rng(1)
        for _ in range(200):
            rot = random_transform(rng)[:3, :3]
            np.testing.assert_allclose(tf.rpy_to_matrix(*tf.matrix_to_rpy(rot)), rot, atol=1e-12)
        for pitch in (np.pi / 2, -np.pi / 2):  # 特異な姿勢
            rot = tf.rpy_to_matrix(0.3, pitch, -0.8)
            np.testing.assert_allclose(tf.rpy_to_matrix(*tf.matrix_to_rpy(rot)), rot, atol=1e-9)

    def test_invert(self) -> None:
        rng = np.random.default_rng(2)
        t = np.array([random_transform(rng) for _ in range(20)])
        np.testing.assert_allclose(t @ tf.invert(t), np.broadcast_to(np.eye(4), t.shape), atol=1e-12)

    def test_rotation_vector(self) -> None:
        axis = np.array([1.0, 2.0, -0.5])
        axis /= np.linalg.norm(axis)
        for angle in (1e-9, 0.3, 2.0, 3.1):
            np.testing.assert_allclose(tf.rotation_vector(tf.rotation_about_axis(axis, angle)), axis * angle, atol=1e-9)


class SourcesTest(unittest.TestCase):
    def test_standard_dh_matches_formula(self) -> None:
        rng = np.random.default_rng(3)
        rows = [
            DHRow(theta=0.2, d=0.3, a=0.1, alpha=np.pi / 2),
            DHRow(theta=-0.4, d=0.0, a=0.5, alpha=0.0, joint_type="prismatic"),
            DHRow(theta=0.0, d=0.2, a=0.05, alpha=-np.pi / 3),
        ]
        base, tool = random_transform(rng), random_transform(rng)
        model = DHParameters(rows, base=base, tool=tool).to_model()
        for _ in range(20):
            q = rng.uniform(-1, 1, size=3)
            expect = base.copy()
            for row, qi in zip(rows, q):
                theta, d = (row.theta + qi, row.d) if row.joint_type == "revolute" else (row.theta, row.d + qi)
                expect = expect @ tf.rot_z(theta) @ tf.translation(z=d) @ tf.translation(x=row.a) @ tf.rot_x(row.alpha)
            got = model.forward_kinematics(q)[model.get_link_index("tool")]
            np.testing.assert_allclose(got, expect @ tool, atol=1e-12)

    def test_modified_dh_matches_formula(self) -> None:
        rng = np.random.default_rng(4)
        rows = [
            MDHRow(alpha=0.0, a=0.0, theta=0.1, d=0.3),
            MDHRow(alpha=-np.pi / 2, a=0.2, theta=0.0, d=0.1, joint_type="prismatic"),
            MDHRow(alpha=np.pi / 3, a=0.4, theta=-0.3, d=0.0),
        ]
        base, tool = random_transform(rng), random_transform(rng)
        model = MDHParameters(rows, base=base, tool=tool).to_model()
        for _ in range(20):
            q = rng.uniform(-1, 1, size=3)
            expect = base.copy()
            for row, qi in zip(rows, q):
                theta, d = (row.theta + qi, row.d) if row.joint_type == "revolute" else (row.theta, row.d + qi)
                expect = expect @ tf.rot_x(row.alpha) @ tf.translation(x=row.a) @ tf.rot_z(theta) @ tf.translation(z=d)
            got = model.forward_kinematics(q)[model.get_link_index("tool")]
            np.testing.assert_allclose(got, expect @ tool, atol=1e-12)

    def test_screw_matches_matrix_exponential_and_mdh(self) -> None:
        """修正 DH のロボットから読み取ったスクリュー軸で作ったロボットが、同じ手先の姿勢を出す。"""
        mdh = six_axis_mdh().to_model()
        home = mdh.forward_kinematics()
        rows = []
        for joint in mdh.dof_joints:
            frame = home[mdh.get_link_index(joint.parent_link)] @ joint.parent_frame
            rows.append(ScrewAxisRow.through(frame[:3, 3], frame[:3, :3] @ np.array(joint.axis)))
        screw = ScrewParameters(rows, tool_home=home[mdh.get_link_index("tool")]).to_model()
        rng = np.random.default_rng(5)
        q = rng.uniform(-np.pi, np.pi, size=(50, 6))
        np.testing.assert_allclose(
            screw.forward_kinematics(q)[:, screw.get_link_index("tool")], mdh.forward_kinematics(q)[:, mdh.get_link_index("tool")], atol=1e-10
        )

    def test_screw_prismatic(self) -> None:
        model = ScrewParameters([ScrewAxisRow((0, 0, 2.0), joint_type="prismatic"), ScrewAxisRow.through((1, 0, 0), (0, 0, 1))]).to_model()
        pose = model.forward_kinematics(np.array([0.5, np.pi / 2]))[model.get_link_index("tool")]
        # 2 軸目は (1, 0, *) を通る z 軸まわり。原点は (1, 0) を中心に 90° 回って (1, -1) へ。そこから z に 0.5
        np.testing.assert_allclose(pose[:3, 3], [1.0, -1.0, 0.5], atol=1e-12)

    def test_bad_joint_type_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            DHRow(joint_type="spherical")  # type: ignore[arg-type]
        with self.assertRaises(ValueError):
            GeometricJointSpec("j", "ball", "a", "b")  # type: ignore[arg-type]


class ModelTest(unittest.TestCase):
    def test_batch_equals_loop(self) -> None:
        rng = np.random.default_rng(6)
        model = random_tree(rng, 12)
        q = rng.uniform(-1, 1, size=(4, 5, model.num_dofs))
        batch = model.forward_kinematics(q)
        self.assertEqual(batch.shape, (4, 5, model.num_links, 4, 4))
        for i in range(4):
            for j in range(5):
                np.testing.assert_allclose(batch[i, j], model.forward_kinematics(q[i, j]), atol=1e-13)

    def test_mimic_resolution(self) -> None:
        links = tuple(GeometricLinkSpec(n) for n in "abcd")
        joints = (
            GeometricJointSpec("j1", "revolute", "a", "b"),
            GeometricJointSpec("j2", "revolute", "b", "c", mimic=GeometricMimicSpec("j1", 2.0, 0.5)),
            GeometricJointSpec("j3", "prismatic", "c", "d", mimic=GeometricMimicSpec("j2", -1.0, 0.1)),
        )
        model = GeometricRobotModel("m", links, joints)
        self.assertEqual(list(model.independent_dof_indices), [0])
        out = model.resolve_mimic(np.array([0.3, 99.0, 99.0]))
        np.testing.assert_allclose(out, [0.3, 2 * 0.3 + 0.5, -(2 * 0.3 + 0.5) + 0.1])

    def test_mimic_cycle_and_bad_reference(self) -> None:
        links = tuple(GeometricLinkSpec(n) for n in "abc")
        with self.assertRaises(ValueError):
            GeometricRobotModel(
                "m",
                links,
                (
                    GeometricJointSpec("j1", "revolute", "a", "b", mimic=GeometricMimicSpec("j2")),
                    GeometricJointSpec("j2", "revolute", "b", "c", mimic=GeometricMimicSpec("j1")),
                ),
            )
        with self.assertRaises(ValueError):
            GeometricRobotModel("m", links[:2], (GeometricJointSpec("j1", "revolute", "a", "b", mimic=GeometricMimicSpec("nope")),))

    def test_tree_validation(self) -> None:
        links = tuple(GeometricLinkSpec(n) for n in "abc")
        with self.assertRaises(ValueError):  # c に親が 2 つ
            GeometricRobotModel("m", links, (GeometricJointSpec("j1", "fixed", "a", "c"), GeometricJointSpec("j2", "fixed", "b", "c")))
        with self.assertRaises(ValueError):  # 根が 2 つ
            GeometricRobotModel("m", links, (GeometricJointSpec("j1", "fixed", "a", "b"),))

    def test_jacobian_matches_numeric(self) -> None:
        rng = np.random.default_rng(7)
        for trial in range(5):
            model = random_tree(rng, 10)
            q = model.resolve_mimic(rng.uniform(-1, 1, size=model.num_dofs))
            jac = model.jacobian(q)
            reduced = model.jacobian(q, reduced=True)
            indep = model.independent_dof_indices
            for li in range(model.num_links):
                # mimic を外した全 DOF での数値微分
                num = numeric_jacobian(lambda x, li=li: model.forward_kinematics(x, resolve_mimic=False)[li], q)
                np.testing.assert_allclose(jac[li], num, atol=1e-6, err_msg=f"trial {trial} link {li}")

                def fk_indep(x: np.ndarray, li: int = li) -> np.ndarray:
                    full = np.zeros(model.num_dofs)
                    full[indep] = x
                    return model.forward_kinematics(full)[li]

                np.testing.assert_allclose(reduced[li], numeric_jacobian(fk_indep, q[indep]), atol=1e-6)

    def test_attach_two_copies(self) -> None:
        rng = np.random.default_rng(8)
        base = random_tree(rng, 4, with_mimic=False)
        arm = six_axis_mdh().to_model()
        mount_l, mount_r = random_transform(rng), random_transform(rng)
        both = base.attach(arm, "L2", mount_l, name="left", base_name="cart").attach(arm, "L3", mount_r, name="right")
        self.assertEqual(set(both.components), {"cart", "left", "right"})
        self.assertEqual(both.num_dofs, base.num_dofs + 12)
        self.assertIn("left/joint3", both.dof_names)
        self.assertEqual(both.component_of_link("right/tool"), "right")
        q = rng.uniform(-1, 1, size=both.num_dofs)
        qb = q[: base.num_dofs]
        ql = q[[both.get_dof_index(f"left/joint{i}") for i in range(1, 7)]]
        qr = q[[both.get_dof_index(f"right/joint{i}") for i in range(1, 7)]]
        poses = both.forward_kinematics(q)
        base_poses = base.forward_kinematics(qb)
        expect_l = base_poses[base.get_link_index("L2")] @ mount_l @ arm.forward_kinematics(ql)[arm.get_link_index("tool")]
        expect_r = base_poses[base.get_link_index("L3")] @ mount_r @ arm.forward_kinematics(qr)[arm.get_link_index("tool")]
        np.testing.assert_allclose(poses[both.get_link_index("left/tool")], expect_l, atol=1e-12)
        np.testing.assert_allclose(poses[both.get_link_index("right/tool")], expect_r, atol=1e-12)
        with self.assertRaises(ValueError):
            both.attach(arm, "L1", name="left")
        # 元のモデルは変わらない
        self.assertEqual(base.components, {})
        self.assertEqual(arm.link_names[0], "link0")

    def test_kinematic_equality(self) -> None:
        a, b = six_axis_mdh().to_model(), six_axis_mdh().to_model()
        self.assertTrue(a.is_kinematically_equal(b))
        self.assertTrue(a.is_kinematically_equal(b.with_prefix("x/")))
        rows = list(six_axis_mdh().rows)
        rows[2] = MDHRow(alpha=0.0, a=0.46)
        self.assertFalse(a.is_kinematically_equal(MDHParameters(rows).to_model()))


class ChainTest(unittest.TestCase):
    def test_chain_matches_model_between_any_links(self) -> None:
        """根を通らない組み合わせ(逆向きにたどる関節を含む)でも、鎖の順運動学がモデルと一致する。"""
        rng = np.random.default_rng(9)
        for trial in range(6):
            model = random_tree(rng, 10)
            indep = model.independent_dof_indices
            for _ in range(8):
                a, b = rng.choice(model.num_links, size=2, replace=False)
                tip_offset = random_transform(rng)
                chain = model.get_kinematic_chain(model.link_names[a], model.link_names[b], tip_offset)
                full = np.zeros(model.num_dofs)
                full[indep] = rng.uniform(-1, 1, size=len(indep))
                full = model.resolve_mimic(full)
                poses = model.forward_kinematics(full)
                expect = tf.invert(poses[a]) @ poses[b] @ tip_offset
                x = full[chain.dof_indices]
                np.testing.assert_allclose(chain.forward_kinematics(x), expect, atol=1e-10, err_msg=f"trial {trial} {a}->{b}")
                if chain.num_variables:
                    np.testing.assert_allclose(chain.jacobian(x), numeric_jacobian(chain.forward_kinematics, x), atol=1e-6)

    def test_chain_variables_skip_followers(self) -> None:
        links = tuple(GeometricLinkSpec(n) for n in "abcd")
        joints = (
            GeometricJointSpec("j1", "revolute", "a", "b", parent_frame=tf.translation(z=0.1), lower_limit=-1.0, upper_limit=1.0),
            GeometricJointSpec("j2", "revolute", "b", "c", parent_frame=tf.translation(x=0.3), mimic=GeometricMimicSpec("j1", 2.0, 0.0), lower_limit=-1.0, upper_limit=1.0),
            GeometricJointSpec("j3", "revolute", "c", "d", parent_frame=tf.translation(x=0.3)),
        )
        chain = GeometricRobotModel("m", links, joints).get_kinematic_chain("a", "d")
        self.assertEqual(chain.variable_names, ("j1", "j3"))
        self.assertTrue(chain.has_mimic)
        self.assertEqual(chain.num_joints, 3)
        # j2 = 2 × j1 が ±1 に収まるよう、j1 の範囲は ±0.5 に狭まる
        np.testing.assert_allclose([chain.lower_limits[0], chain.upper_limits[0]], [-0.5, 0.5])
        np.testing.assert_allclose(chain.joint_values(np.array([0.2, 0.7])), [0.2, 0.4, 0.7])

    def test_chain_to_model_round_trip(self) -> None:
        rng = np.random.default_rng(10)
        model = random_tree(rng, 9, with_mimic=False)
        chain = model.get_kinematic_chain("L5", "L8", random_transform(rng))
        serial = chain.to_model()
        x = rng.uniform(-1, 1, size=chain.num_variables)
        np.testing.assert_allclose(serial.forward_kinematics(x)[serial.get_link_index("tip")], chain.forward_kinematics(x), atol=1e-12)


class UrdfTest(unittest.TestCase):
    def test_export_import_round_trip(self) -> None:
        """書き出して読み直すと、<名前>_frame リンクがモデル本来のリンク座標系に一致する。"""
        rng = np.random.default_rng(11)
        model = random_tree(rng, 10)
        back = read_urdf_string(model.to_urdf())
        self.assertEqual(back.dof_names, model.dof_names)
        q = rng.uniform(-1, 1, size=(20, model.num_dofs))
        a, b = model.forward_kinematics(q), back.forward_kinematics(q)
        for name in model.link_names:
            frame = name + "_frame" if name + "_frame" in back.link_names else name
            np.testing.assert_allclose(b[:, back.get_link_index(frame)], a[:, model.get_link_index(name)], atol=1e-9, err_msg=name)

    def test_dh_export_has_no_extra_frames_at_joint_origins(self) -> None:
        text = ScrewParameters([ScrewAxisRow((0, 0, 1.0), joint_type="prismatic")]).to_model().to_urdf()
        self.assertNotIn("_frame", text)

    def test_file_round_trip_and_source(self) -> None:
        model = six_axis_mdh().to_model()
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "six.urdf")
            model.to_urdf(path)
            back = UrdfSource(path).to_model()
        q = np.linspace(-1, 1, 6)
        # tool は固定関節の先にあり、リンク座標系が関節の位置にあるので tool_frame は出ない。link6 は出る
        self.assertNotIn("tool_frame", back.link_names)
        np.testing.assert_allclose(back.forward_kinematics(q)[back.get_link_index("tool")], model.forward_kinematics(q)[model.get_link_index("tool")], atol=1e-10)
        np.testing.assert_allclose(
            back.forward_kinematics(q)[back.get_link_index("link6_frame")], model.forward_kinematics(q)[model.get_link_index("link6")], atol=1e-10
        )

    def test_reads_sample_robot_with_mimic(self) -> None:
        model = UrdfSource(FRANKA_URDF).to_model()
        self.assertEqual(model.num_dofs, 9)
        finger2 = model.joints[model.get_joint_index("panda_finger_joint2")]
        self.assertEqual(finger2.mimic, GeometricMimicSpec("panda_finger_joint1", 1.0, 0.0))
        self.assertEqual(len(model.independent_dof_indices), 8)
        chain = model.get_kinematic_chain("panda_link0", "panda_hand")
        self.assertEqual(chain.num_variables, 7)
        self.assertFalse(chain.has_mimic)

    @unittest.skipIf(pin is None, "pinocchio が無い")
    def test_forward_kinematics_matches_pinocchio(self) -> None:
        """別実装(Pinocchio)と、全リンクの姿勢を突き合わせる。自前で書き出した URDF と、同梱のサンプル URDF の両方。"""
        rng = np.random.default_rng(12)
        cases: list[tuple[str, GeometricRobotModel, str]] = []
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(4):
                model = random_tree(rng, 9, with_mimic=False)
                path = os.path.join(tmp, f"random{i}.urdf")
                model.to_urdf(path)
                cases.append((f"random{i}", read_urdf_string(model.to_urdf()), path))
            cases.append(("ur10", UrdfSource(UR10_URDF).to_model(), UR10_URDF))
            for label, model, path in cases:
                pmodel = pin.buildModelFromUrdf(path)
                data = pmodel.createData()
                self.assertEqual(pmodel.nq, model.num_dofs, label)
                for _ in range(10):
                    q = rng.uniform(-1, 1, size=model.num_dofs)
                    qp = np.zeros(pmodel.nq)
                    for d, name in enumerate(model.dof_names):
                        qp[pmodel.joints[pmodel.getJointId(name)].idx_q] = q[d]
                    pin.forwardKinematics(pmodel, data, qp)
                    pin.updateFramePlacements(pmodel, data)
                    poses = model.forward_kinematics(q)
                    root = data.oMf[pmodel.getFrameId(model.root_link)].homogeneous
                    for name in model.link_names:
                        expect = np.linalg.inv(root) @ data.oMf[pmodel.getFrameId(name)].homogeneous
                        np.testing.assert_allclose(poses[model.get_link_index(name)], expect, atol=1e-9, err_msg=f"{label} {name}")


class CollisionFilterTest(unittest.TestCase):
    def test_rules(self) -> None:
        f = GeometricCollisionFilter()
        f.add_pair("a", "b", reason="adjacent")
        f.add_pattern("left/finger_*", "right/*", reason="grip")
        f.add_rule(lambda x, y: x == "base" and y.startswith("wheel"), reason="wheels")
        self.assertTrue(f.is_ignored("b", "a"))
        self.assertEqual(f.reason_for("right/link2", "left/finger_1"), "grip")
        self.assertTrue(f.is_ignored("wheel3", "base"))
        self.assertFalse(f.is_ignored("a", "c"))
        self.assertEqual(f.remove_by_reason("grip"), 1)
        self.assertFalse(f.is_ignored("right/link2", "left/finger_1"))
        self.assertTrue(f.remove_pair("a", "b"))
        self.assertFalse(f.is_ignored("a", "b"))

    def test_from_model_and_merge_and_json(self) -> None:
        arm = six_axis_mdh().to_model()
        base = GeometricRobotModel("cart", (GeometricLinkSpec("body"),), ())
        both = base.attach(arm, "body", name="left").attach(arm, "body", name="right")
        f = GeometricCollisionFilter.from_model(both)
        self.assertEqual(f.reason_for("body", "left/link0"), "mount")
        self.assertEqual(f.reason_for("right/link2", "right/link3"), "adjacent")
        self.assertFalse(f.is_ignored("left/link3", "right/link3"))
        merged = GeometricCollisionFilter.from_model(arm).merged_with(GeometricCollisionFilter.from_model(arm), prefix="left/", other_prefix="right/")
        self.assertTrue(merged.is_ignored("left/link1", "left/link2"))
        self.assertFalse(merged.is_ignored("left/link1", "right/link2"))
        back = GeometricCollisionFilter.from_json(f.to_json())
        self.assertEqual(back.pairs, f.pairs)


if __name__ == "__main__":
    unittest.main()
