"""新 API のユニットテスト: IsaacRobotAPI 自動 Component・batch_update・名前/辞書 DOF・convex_meshes・DebugDraw。"""

from __future__ import annotations

import json
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

from myext.robot.geometric_newton import (
    GeometricLayerOptions,
    GeometricRobot,
    GeometricScene,
    UsdArticulationSource,
)


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

            # describe() が RFC 8259 準拠 JSON で主要項目を含むこと
            desc = robot.describe()
            parsed = json.loads(desc)
            self.assertEqual(parsed["summary"]["num_dofs"], robot.num_dofs)
            self.assertIn("arm1", parsed["components"])
            self.assertTrue(any(d["mimic"] is not None for d in parsed["dofs"]))
            self.assertEqual(parsed["config"]["collision_mode"], "none")
            self.assertEqual(parsed, robot.describe_dict())
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

    def test_broad_phase_filter_baking_and_dynamic_update(self) -> None:
        """既定の GeometricScene で collision_filter が Newton Broad Phase へ反映され、
        動的変更 (add_pair / remove_pair) と touching_pairs() 診断が両立することを検証する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # 3 リンク直列 (L0 - L1 - L2): 全部同じ原点に半径 0.2 の球を置いて全組が重なる状態を作る
        for name in ("L0", "L1", "L2"):
            lf = UsdGeom.Xform.Define(stage, f"/World/Bot/{name}")
            if name == "L0":
                UsdPhysics.ArticulationRootAPI.Apply(lf.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(lf.GetPrim())
            sp = UsdGeom.Sphere.Define(stage, f"/World/Bot/{name}/col")
            sp.CreateRadiusAttr(0.2)
            UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
        for jname, parent, child in (("j01", "L0", "L1"), ("j12", "L1", "L2")):
            j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Bot/{jname}")
            j.CreateAxisAttr("Z")
            j.CreateBody0Rel().SetTargets([f"/World/Bot/{parent}"])
            j.CreateBody1Rel().SetTargets([f"/World/Bot/{child}"])

        robot = GeometricRobot.from_articulation(stage, "/World/Bot", write_target="usd", newton_device="cpu")
        try:
            # 1) 初期状態: 隣接 (L0,L1) と (L1,L2) は Newton Broad Phase の除外リストへ登録済み
            #    非隣接 (L0,L2) だけが Broad Phase 候補 (shape_pairs_filtered) に残り、1 件ヒットする
            hits = robot.check_collisions("within")
            self.assertEqual([p.names for p in hits], [("L0", "L2")])
            sc = robot.scene
            self.assertEqual(sc.newton_model.shape_collision_filter_pairs_array().shape[0], 2)
            self.assertEqual(sc._pipeline.shape_pairs_filtered.shape[0], 1)

            # 2) 非隣接 (L0,L2) を add_pair で除外 -> Broad Phase 候補ペア自体が 0 件になる (Narrow Phase 0 回)
            robot.collision_filter.add_pair("L0", "L2", reason="always_touch")
            hits_after_add = robot.check_collisions("within")
            self.assertEqual(hits_after_add, [])
            self.assertEqual(sc.newton_model.shape_collision_filter_pairs_array().shape[0], 3)
            self.assertEqual(sc._pipeline.shape_pairs_filtered.shape[0], 0)

            # 3) それでも診断用 touching_pairs() は全 3 組の重なりを検出できる
            touching = robot.touching_pairs()
            self.assertEqual({p.names for p in touching}, {("L0", "L1"), ("L1", "L2"), ("L0", "L2")})

            # 4) 隣接ペア (L0,L1) の除外を remove_pair で外すと、Broad Phase に戻って検出される
            self.assertTrue(robot.collision_filter.remove_pair("L0", "L1"))
            hits_after_remove = robot.check_collisions("within")
            self.assertEqual([p.names for p in hits_after_remove], [("L0", "L1")])
            self.assertEqual(sc._pipeline.shape_pairs_filtered.shape[0], 1)
        finally:
            robot.close()

    def test_collision_mode_none_disables_physx_but_newton_still_detects(self) -> None:
        """既定 collision_mode='none' で USD 上は physics:collisionEnabled=False になるが、
        Newton は元コライダーを検出し、ユーザーが明示無効化したコライダーは無視すること。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        for name in ("L0", "L1"):
            lf = UsdGeom.Xform.Define(stage, f"/World/Bot/{name}")
            if name == "L0":
                UsdPhysics.ArticulationRootAPI.Apply(lf.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(lf.GetPrim())
            sp = UsdGeom.Sphere.Define(stage, f"/World/Bot/{name}/col")
            sp.CreateRadiusAttr(0.2)
            UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
        j = UsdPhysics.RevoluteJoint.Define(stage, "/World/Bot/j01")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets(["/World/Bot/L0"])
        j.CreateBody1Rel().SetTargets(["/World/Bot/L1"])
        # 障害物 (環境)
        obs = UsdGeom.Sphere.Define(stage, "/World/Obs")
        obs.CreateRadiusAttr(0.2)
        UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

        robot = GeometricRobot.from_articulation(stage, "/World/Bot", write_target="usd", newton_device="cpu")
        try:
            self.assertEqual(robot.layer.options.collision_mode, "none")
            # USD 上ではロボットのコライダーは physics:collisionEnabled=False (PhysX コライダー生成ゼロ)
            col0 = stage.GetPrimAtPath("/World/Bot/L0/col")
            self.assertIs(UsdPhysics.CollisionAPI(col0).GetCollisionEnabledAttr().Get(), False)
            # 環境コライダーは触っていない
            self.assertIsNot(UsdPhysics.CollisionAPI(obs.GetPrim()).GetCollisionEnabledAttr().Get(), False)
            # それでも Newton はロボットのコライダーを認識し、環境干渉を検出する
            self.assertEqual(robot.collider_paths["L0"], ["/World/Bot/L0/col"])
            self.assertTrue(robot.is_in_collision("environment"))
            # PhysX バックエンドは明示エラーになる
            with self.assertRaises(RuntimeError):
                robot.check_collisions("environment", backend="physx")
            # ユーザーが Root Layer 側で明示的に collisionEnabled=False にすれば Newton も無視する
            UsdPhysics.CollisionAPI(col0).CreateCollisionEnabledAttr().Set(False)
            self.assertEqual(robot.collider_paths["L0"], [])
        finally:
            robot.close()
        # layer.remove() 後は L1 の collisionEnabled は元の状態に戻る
        col1 = stage.GetPrimAtPath("/World/Bot/L1/col")
        self.assertIsNot(UsdPhysics.CollisionAPI(col1).GetCollisionEnabledAttr().Get(), False)

    def test_limit_policy_reject_and_violation_link_query(self) -> None:
        """limit_policy='clip' / 'reject' と get_limit_violations / get_limit_violation_links の動作検証。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        for name in ("base", "link1", "link2"):
            lf = UsdGeom.Xform.Define(stage, f"/World/Bot/{name}")
            if name == "base":
                UsdPhysics.ArticulationRootAPI.Apply(lf.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(lf.GetPrim())
        for jname, parent, child, lo, hi in (
            ("j1", "base", "link1", -45.0, 45.0),
            ("j2", "link1", "link2", -30.0, 60.0),
        ):
            j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Bot/{jname}")
            j.CreateAxisAttr("Z")
            j.CreateLowerLimitAttr(lo)
            j.CreateUpperLimitAttr(hi)
            j.CreateBody0Rel().SetTargets([f"/World/Bot/{parent}"])
            j.CreateBody1Rel().SetTargets([f"/World/Bot/{child}"])

        robot = GeometricRobot.from_articulation(stage, "/World/Bot", write_target="usd", angle_unit="deg", limit_policy="clip")
        try:
            # 安全範囲内 -> 違反なし
            robot.set_dof_positions([10.0, 20.0])
            self.assertFalse(robot.is_at_limit())
            self.assertEqual(robot.get_limit_violations(), [])
            self.assertEqual(robot.get_limit_violation_links(), {})

            # clip モード: 上限超過 -> 上限値に切り詰められ、超過情報 + child_link を取得できる
            robot.set_dof_positions([80.0, -10.0])
            np.testing.assert_allclose(robot.get_dof_positions(), [45.0, -10.0], atol=1e-6)
            self.assertTrue(robot.is_at_limit())
            viols = robot.get_limit_violations()
            self.assertEqual(len(viols), 1)
            self.assertEqual(viols[0].dof_name, "j1")
            self.assertEqual(viols[0].child_link, "link1")
            self.assertEqual(viols[0].child_link_path, "/World/Bot/link1")
            self.assertEqual(viols[0].side, "upper")
            self.assertTrue(viols[0].exceeded)
            self.assertAlmostEqual(viols[0].requested_value, 80.0, places=5)
            self.assertAlmostEqual(viols[0].actual_value, 45.0, places=5)
            self.assertEqual(robot.get_limit_violation_links(), {"link1": "upper"})

            # ちょうど上限にセット: include_at_limit=True で検出、False なら除外
            robot.set_dof_positions([45.0, 0.0])
            self.assertEqual(len(robot.get_limit_violations(include_at_limit=True)), 1)
            self.assertFalse(robot.get_limit_violations(include_at_limit=True)[0].exceeded)
            self.assertEqual(robot.get_limit_violations(include_at_limit=False), [])

            # reject モードへ切替: 範囲外指令を拒否し、直前姿勢 [10, 20] を維持する
            robot.set_dof_positions([10.0, 20.0])
            robot.limit_policy = "reject"
            robot.set_dof_positions([10.0, -90.0])
            np.testing.assert_allclose(robot.get_dof_positions(), [10.0, 20.0], atol=1e-6)
            viols_rej = robot.get_limit_violations(include_at_limit=False)
            self.assertEqual(len(viols_rej), 1)
            self.assertEqual(viols_rej[0].dof_name, "j2")
            self.assertEqual(viols_rej[0].child_link, "link2")
            self.assertEqual(viols_rej[0].side, "lower")
            self.assertTrue(viols_rej[0].exceeded)
            self.assertAlmostEqual(viols_rej[0].requested_value, -90.0, places=5)
            self.assertAlmostEqual(viols_rej[0].actual_value, 20.0, places=5)
            self.assertEqual(robot.get_limit_violation_links(include_at_limit=False), {"link2": "lower"})
        finally:
            robot.close()

    def test_merge_variant_components_and_dof_binder_across_variant_switch(self) -> None:
        """VariantSet にぶら下がる子 IsaacRobotAPI の親統合切替 + 異軸数 Variant 切替での
        ignore_unknown / bind_dofs() + is_render_ready を検証する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        root = UsdGeom.Xform.Define(stage, "/World/Arm").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)
        _add_isaac_robot_api(root)

        base = UsdGeom.Xform.Define(stage, "/World/Arm/base").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(base)
        link1 = UsdGeom.Xform.Define(stage, "/World/Arm/link1").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link1)
        j1 = UsdPhysics.RevoluteJoint.Define(stage, "/World/Arm/j1")
        j1.CreateAxisAttr("Z")
        j1.CreateBody0Rel().SetTargets(["/World/Arm/base"])
        j1.CreateBody1Rel().SetTargets(["/World/Arm/link1"])

        # VariantSet "Tool" を /World/Arm/link1/tool に定義:
        #   - "TwoFinger": 2 DOF (grip_l, grip_r) の子 IsaacRobotAPI コンポーネント
        #   - "Suction": 0 DOF (固定ノズルのみ) の子 IsaacRobotAPI コンポーネント
        tool_prim = UsdGeom.Xform.Define(stage, "/World/Arm/link1/tool").GetPrim()
        vset = tool_prim.GetVariantSets().AddVariantSet("Tool")
        vset.AddVariant("TwoFinger")
        vset.AddVariant("Suction")

        vset.SetVariantSelection("TwoFinger")
        with vset.GetVariantEditContext():
            groot = UsdGeom.Xform.Define(stage, "/World/Arm/link1/tool/gripper").GetPrim()
            _add_isaac_robot_api(groot)
            UsdPhysics.RigidBodyAPI.Apply(groot)
            mj = UsdPhysics.FixedJoint.Define(stage, "/World/Arm/link1/tool/mount")
            mj.CreateBody0Rel().SetTargets(["/World/Arm/link1"])
            mj.CreateBody1Rel().SetTargets(["/World/Arm/link1/tool/gripper"])
            for fname in ("fl", "fr"):
                fp = UsdGeom.Xform.Define(stage, f"/World/Arm/link1/tool/gripper/{fname}").GetPrim()
                UsdPhysics.RigidBodyAPI.Apply(fp)
                fj = UsdPhysics.PrismaticJoint.Define(stage, f"/World/Arm/link1/tool/gripper/grip_{fname}")
                fj.CreateAxisAttr("X")
                fj.CreateBody0Rel().SetTargets(["/World/Arm/link1/tool/gripper"])
                fj.CreateBody1Rel().SetTargets([f"/World/Arm/link1/tool/gripper/{fname}"])

        vset.SetVariantSelection("Suction")
        with vset.GetVariantEditContext():
            sroot = UsdGeom.Xform.Define(stage, "/World/Arm/link1/tool/suction").GetPrim()
            _add_isaac_robot_api(sroot)
            UsdPhysics.RigidBodyAPI.Apply(sroot)
            mj2 = UsdPhysics.FixedJoint.Define(stage, "/World/Arm/link1/tool/mount_s")
            mj2.CreateBody0Rel().SetTargets(["/World/Arm/link1"])
            mj2.CreateBody1Rel().SetTargets(["/World/Arm/link1/tool/suction"])

        vset.SetVariantSelection("TwoFinger")

        # 1) merge_variant_components=False (既定): Arm と gripper が別 Component
        r_sep = GeometricRobot.from_articulation(stage, "/World/Arm", write_target="usd", merge_variant_components=False)
        try:
            self.assertEqual(set(r_sep.component_names), {"Arm", "gripper"})
            self.assertEqual(r_sep.get_component("Arm").dof_names, ["j1"])
            self.assertEqual(set(r_sep.get_component("gripper").dof_names), {"grip_fl", "grip_fr"})
        finally:
            r_sep.close()

        # 2) merge_variant_components=True: VariantSet 配下の gripper が親 Arm に統合される
        r_merged = GeometricRobot.from_articulation(
            stage, "/World/Arm", write_target="usd", angle_unit="deg", merge_variant_components=True
        )
        try:
            self.assertEqual(r_merged.component_names, ["Arm"])
            self.assertEqual(set(r_merged.get_component("Arm").dof_names), {"j1", "grip_fl", "grip_fr"})

            # 3) Variant 間で軸数が変わるロボットに対し、binder と ignore_unknown=True で安全に書き込める
            binder = r_merged.bind_dofs(["j1", "grip_fl", "grip_fr"], ignore_unknown=True)
            self.assertEqual(binder.active_names, ("j1", "grip_fl", "grip_fr"))
            self.assertEqual(binder.missing_names, ())
            binder.set_positions([15.0, 0.02, -0.02])
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["j1"], 15.0, places=5)
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["grip_fl"], 0.02, places=5)

            # Suction (1 DOF: j1 のみ) へ切替
            r_merged.set_variant("Tool", "Suction")
            self.assertEqual(r_merged.dof_names, ["j1"])
            self.assertEqual(r_merged.component_names, ["Arm"])
            # 切替直後は _repush_frames > 0 のため is_render_ready は False、pending_render_sync_frames > 0
            self.assertGreater(r_merged.pending_render_sync_frames, 0)
            self.assertFalse(r_merged.is_render_ready)

            # 同じ 3 要素配列を binder へそのまま渡してもエラーにならず、存在する j1 だけ更新される
            binder.set_positions([30.0, 0.04, -0.04])
            self.assertEqual(binder.active_names, ("j1",))
            self.assertEqual(binder.missing_names, ("grip_fl", "grip_fr"))
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["j1"], 30.0, places=5)
            got_vec = binder.get_positions(fill_value=-999.0)
            self.assertAlmostEqual(got_vec[0], 30.0, places=5)
            self.assertEqual(got_vec[1], -999.0)
            self.assertEqual(got_vec[2], -999.0)

            # set_dof_positions の ignore_unknown=True (辞書 / dof_names / Component)
            r_merged.set_dof_positions({"j1": 40.0, "grip_fl": 0.05}, ignore_unknown=True)
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["j1"], 40.0, places=5)
            r_merged.set_dof_positions([45.0, 0.05], dof_names=["j1", "grip_fl"], ignore_unknown=True)
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["j1"], 45.0, places=5)
            r_merged.get_component("Arm").set_dof_positions({"j1": 50.0, "grip_fl": 0.05}, ignore_unknown=True)
            self.assertAlmostEqual(r_merged.get_dof_positions_dict()["j1"], 50.0, places=5)
        finally:
            r_merged.close()


if __name__ == "__main__":
    unittest.main()
