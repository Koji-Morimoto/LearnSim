"""新 API のユニットテスト: IsaacRobotAPI 自動 Component・batch_update・名前/辞書 DOF・convex_meshes・DebugDraw。"""

from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_EXT_ROOT = os.path.abspath(os.path.join(_TESTS_DIR, ".."))
for _p in (_TESTS_DIR, _EXT_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# `test_usd_reader_and_collider_sublayer` の import 時に Isaac Sim 同梱の pxr / warp / newton パスが解決される
from test_usd_reader_and_collider_sublayer import _build_synthetic_multi_arm_stage

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from myext.robot.geometric_newton import GeometricRobot, GeometricScene, UsdArticulationSource


def _add_isaac_robot_api(prim: Usd.Prim, *, namespace: str = "", link_targets: list[str] | None = None) -> None:
    """純 OpenUSD 環境でも動く形で `IsaacRobotAPI` スキーマと関係属性を prim に付与する。"""
    prim.AddAppliedSchema("IsaacRobotAPI")
    if namespace:
        prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String).Set(namespace)
    if link_targets is not None:
        prim.CreateRelationship("isaac:physics:robotLinks").SetTargets([Sdf.Path(p) for p in link_targets])


def _make_u_shape_mesh(stage: Usd.Stage, path: str, translate: tuple[float, float, float]) -> UsdGeom.Mesh:
    """凹形状 (U 字型: 中央に幅 0.4 m の溝がある) の三角形メッシュを定義する。"""
    # 3 つの直方体 (底板 + 左壁 + 右壁) の三角形群を 1 つの Mesh にまとめる
    boxes = [
        ((-0.4, -0.1, 0.0), (0.4, 0.1, 0.1)),   # 底板
        ((-0.4, -0.1, 0.1), (-0.2, 0.1, 0.5)),  # 左壁
        ((0.2, -0.1, 0.1), (0.4, 0.1, 0.5)),    # 右壁
    ]
    pts: list[Gf.Vec3f] = []
    idx: list[int] = []
    counts: list[int] = []
    tri_template = [
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6),
        (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5),
    ]
    for (x0, y0, z0), (x1, y1, z1) in boxes:
        base = len(pts)
        pts.extend(
            [
                Gf.Vec3f(x0, y0, z0), Gf.Vec3f(x1, y0, z0), Gf.Vec3f(x1, y1, z0), Gf.Vec3f(x0, y1, z0),
                Gf.Vec3f(x0, y0, z1), Gf.Vec3f(x1, y0, z1), Gf.Vec3f(x1, y1, z1), Gf.Vec3f(x0, y1, z1),
            ]
        )
        for a, b, c in tri_template:
            idx.extend([base + a, base + b, base + c])
            counts.append(3)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.AddTranslateOp().Set(Gf.Vec3d(*translate))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


class TestIsaacRobotApiAndErgonomicApis(unittest.TestCase):
    def _build_stage_with_isaac_robot_api(self) -> Usd.Stage:
        stage = _build_synthetic_multi_arm_stage()
        _add_isaac_robot_api(
            stage.GetPrimAtPath("/robot"),
            link_targets=[
                "/robot/Geometry/base_body",
                "/robot/Geometry/turret",
                "/robot/Geometry/turret/port1/arm1",
                "/robot/Geometry/turret/port2/arm2",
            ],
        )
        _add_isaac_robot_api(stage.GetPrimAtPath("/robot/Geometry/turret/port1/arm1"))
        _add_isaac_robot_api(stage.GetPrimAtPath("/robot/Geometry/turret/port2/arm2"))
        return stage

    def test_auto_components_from_isaac_robot_api_eagerly_created(self) -> None:
        stage = self._build_stage_with_isaac_robot_api()
        robot = GeometricRobot.from_articulation(stage, "/robot", write_target="usd")
        try:
            # 生成直後から components が揃っていること
            self.assertEqual(set(robot.component_names), {"robot", "arm1", "arm2"})
            self.assertEqual(set(robot.components.keys()), {"robot", "arm1", "arm2"})
            arm1 = robot.get_component("arm1")
            arm2 = robot.get_component("arm2")
            self.assertEqual(arm1.num_dofs, 3)  # j1, finger_l_joint, finger_r_joint(mimic)
            self.assertEqual(arm2.num_dofs, 3)
            self.assertEqual(len(arm1.independent_dof_indices), 2)
            self.assertEqual(len(robot.independent_dof_indices), 5)  # turret + 2*(j1 + finger_l)
            self.assertEqual(len(robot.independent_dof_names), 5)

            # auto_components=False なら自動生成しない
            robot_off = GeometricRobot.from_articulation(stage, "/robot", auto_components=False, write_target="usd", apply_layer=False)
            self.assertEqual(robot_off.component_names, [])
            robot_off.close(remove_layer=False)
        finally:
            robot.close()

    def test_set_get_dof_positions_dict_and_names_and_mimic(self) -> None:
        stage = self._build_stage_with_isaac_robot_api()
        robot = GeometricRobot.from_articulation(stage, "/robot", write_target="usd", angle_unit="deg")
        try:
            # 辞書指定で部分更新 + mimic 追従確認
            robot.set_dof_positions({"turret_joint": 30.0, "j1": 45.0, "finger_l_joint": 0.02})
            got = robot.get_dof_positions_dict()
            self.assertAlmostEqual(got["turret_joint"], 30.0, places=6)
            self.assertAlmostEqual(got["j1"], 45.0, places=6)
            self.assertAlmostEqual(got["finger_l_joint"], 0.02, places=6)
            self.assertAlmostEqual(got["finger_r_joint"], -0.02, places=6)  # mimic = -1.0 * finger_l

            # independent_only=True では mimic 従動軸を含まない
            indep_dict = robot.get_dof_positions_dict(independent_only=True)
            self.assertNotIn("finger_r_joint", indep_dict)
            self.assertIn("finger_l_joint", indep_dict)

            # dof_names= での読み書き
            robot.set_dof_positions([15.0, -20.0], dof_names=["turret_joint", "j1"])
            vals = robot.get_dof_positions(dof_names=["turret_joint", "j1"])
            np.testing.assert_allclose(vals, [15.0, -20.0], atol=1e-6)

            # Component 側でも辞書・dof_names が使える
            arm2 = robot.get_component("arm2")
            j1_arm2 = [n for n in arm2.dof_names if n.startswith("j1")][0]
            arm2.set_dof_positions({j1_arm2: 25.0})
            self.assertAlmostEqual(arm2.get_dof_positions_dict()[j1_arm2], 25.0, places=6)

            # 排他チェック
            with self.assertRaises(ValueError):
                robot.set_dof_positions({"turret_joint": 0.0}, dof_indices=[0])
            with self.assertRaises(ValueError):
                robot.set_dof_positions([0.0], dof_indices=[0], dof_names=["turret_joint"])
            with self.assertRaises(KeyError):
                robot.set_dof_positions({"no_such_joint": 0.0})

            # describe() が主要項目を含むこと
            desc = robot.describe()
            self.assertIn("GeometricRobot", desc)
            self.assertIn("arm1", desc)
            self.assertIn("mimic <-", desc)
        finally:
            robot.close()

    def test_batch_update_defers_push_to_single_call(self) -> None:
        stage = self._build_stage_with_isaac_robot_api()
        robot = GeometricRobot.from_articulation(stage, "/robot", write_target="usd")
        try:
            push_count = 0
            orig_push = robot._push

            def counted_push() -> None:
                nonlocal push_count
                push_count += 1
                orig_push()

            robot._push = counted_push  # type: ignore[method-assign]
            arm1 = robot.get_component("arm1")
            arm2 = robot.get_component("arm2")
            with robot.batch_update():
                arm1.set_dof_positions([0.2, 0.01, 0.0])
                arm2.set_dof_positions([-0.3, 0.02, 0.0])
                robot.set_world_poses(positions=[0.5, 0.0, 0.1])
                self.assertEqual(push_count, 0)
            self.assertEqual(push_count, 1)
            # mimic も解決されていること
            self.assertAlmostEqual(robot.get_dof_positions_dict()["finger_r_joint"], -0.01, places=6)
        finally:
            robot.close()

    def test_convex_meshes_false_vs_true_on_concave_u_mesh(self) -> None:
        """U 字凹メッシュの溝の中に小球を置き、convex_meshes=True (凸包=過剰検知) と False (生メッシュ=非接触) の差を確認。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # ロボット: 1 リンクに半径 0.05 m の球コライダー (U 字の溝の中央 z=0.30 に配置)
        link_xf = UsdGeom.Xform.Define(stage, "/World/Probe/base")
        link_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))
        UsdPhysics.ArticulationRootAPI.Apply(link_xf.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(link_xf.GetPrim())
        sphere = UsdGeom.Sphere.Define(stage, "/World/Probe/base/col")
        sphere.CreateRadiusAttr(0.05)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/Probe/root_joint")
        fj.CreateBody1Rel().SetTargets(["/World/Probe/base"])

        # 環境障害物: 凹形状 U 字メッシュ (溝は x in [-0.2, +0.2], z in [0.1, 0.5])
        _make_u_shape_mesh(stage, "/World/UObstacle", (0.0, 0.0, 0.0))

        # 1) convex_meshes=True (既定): U 字の溝が凸包で塞がれるためヒットする
        robot_hull = GeometricRobot.from_articulation(
            stage, "/World/Probe", write_target="usd", convex_meshes=True, newton_device="cpu"
        )
        try:
            hits_hull = robot_hull.check_collisions("environment")
            self.assertTrue(len(hits_hull) >= 1, "convex_meshes=True では凸包が溝を埋めるため接触するはず")
            # 実行時に convex_meshes=False へ切り替えられること
            robot_hull.convex_meshes = False
            hits_mesh = robot_hull.check_collisions("environment")
            self.assertEqual(len(hits_mesh), 0, "convex_meshes=False では元の凹メッシュそのままなので溝の中は非接触")
        finally:
            robot_hull.close()

    def test_debug_draw_wireframe_and_color_spec(self) -> None:
        # 配色規約: 通常=シアン、干渉中=赤、接触点=オレンジ
        self.assertEqual(GeometricScene._COLOR_NORMAL, (0.0, 0.9, 1.0, 1.0))
        self.assertEqual(GeometricScene._COLOR_COLLIDING, (1.0, 0.15, 0.15, 1.0))
        self.assertEqual(GeometricScene._COLOR_CONTACT, (1.0, 0.55, 0.0, 1.0))

        for spec, min_edges in [
            (("box", (0.1, 0.2, 0.3)), 12),
            (("sphere", (0.2,)), 60),
            (("capsule", (0.1, 0.2)), 40),
            (("cylinder", (0.1, 0.2)), 36),
            (("cone", (0.1, 0.2)), 20),
            (("mesh", (np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32), np.array([0, 1, 2], dtype=np.int32), (1, 1, 1))), 3),
        ]:
            s, e = GeometricScene._build_local_wireframe(spec)
            self.assertEqual(s.shape, e.shape)
            self.assertGreaterEqual(len(s), min_edges)


if __name__ == "__main__":
    unittest.main()
