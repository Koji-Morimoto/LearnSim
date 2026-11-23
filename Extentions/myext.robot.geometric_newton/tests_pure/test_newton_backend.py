"""Newton バックエンド (GeometricScene / BatchedNewtonScene) の純粋 Python + OpenUSD + Newton テスト。

Kit / SimulationApp を起動せずに、順運動学・全 USD コライダー形状・自己干渉・環境干渉・
クリアランス(符号付き距離)・動的障害物同期・複数ロボット・非 USD ソース・バッチ GPU(CUDA Graph)
を検証する。newton / warp / pxr が無い環境では自動的にスキップされる。
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from helpers import FRANKA_URDF, random_transform, random_tree, six_axis_mdh

# Kit/Python 3.12 と競合する外部 OpenUSD が PYTHONPATH にある場合は除外し、
# Isaac Sim 同梱パッケージが未登録なら自動検出して読み込む
def _ensure_isaac_newton_imports() -> tuple[bool, str]:
    sys.path[:] = [p for p in sys.path if "BuiltOpenUSD" not in p]

    # すでに import できる環境(Kit / 外部で PYTHONPATH 設定済み)を優先
    def _try_import() -> bool:
        try:
            import newton  # noqa: F401
            import warp  # noqa: F401
            from pxr import Usd, UsdGeom, UsdPhysics  # noqa: F401
            return True
        except Exception:
            return False

    if not _try_import():
        # 拡張の親階層から Isaac Sim ルートを推定してバンドルを追加
        ext_root = Path(__file__).resolve().parents[1]
        isaac_candidates = [ext_root.parents[2], Path(r"C:\isaacsim-6.2.0")]
        for root in isaac_candidates:
            if not (root / "exts" / "isaacsim.pip.newton" / "pip_prebundle").exists():
                continue
            extra: list[Path] = [
                root / "exts" / "isaacsim.pip.newton" / "pip_prebundle",
                root / "exts" / "omni.pip.compute" / "pip_prebundle",
            ]
            for pattern in ("omni.kit.pip_archive-*", "omni.warp.core-*", "omni.usd.libs-*"):
                for match in sorted((root / "extscache").glob(pattern)):
                    if (match / "pip_prebundle").is_dir():
                        extra.append(match / "pip_prebundle")
                    extra.append(match)
                    bin_dir = match / "bin"
                    if pattern.startswith("omni.usd.libs") and bin_dir.is_dir():
                        try:
                            os.add_dll_directory(str(bin_dir))
                        except Exception:
                            pass
                        os.environ["PATH"] = str(bin_dir) + os.pathsep + os.environ.get("PATH", "")
            for p in reversed(extra):
                sp = str(p)
                if p.exists() and sp not in sys.path:
                    sys.path.insert(0, sp)
            break

    try:
        import warp as wp

        if os.environ.get("WARP_CACHE_PATH"):
            wp.config.kernel_cache_dir = os.environ["WARP_CACHE_PATH"]
        wp.init()
        import newton  # noqa: F401
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: F401

        return True, ""
    except Exception as exc:
        return False, str(exc)


HAS_NEWTON, _IMPORT_ERR = _ensure_isaac_newton_imports()

if HAS_NEWTON:
    import warp as wp
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from myext.robot.geometric_newton.builder import GeometricVisualStyle, build_robot_prims
    from myext.robot.geometric_newton.model import (
        DHParameters,
        DHRow,
        GeometricCollisionFilter,
        GeometricJointSpec,
        GeometricLinkSpec,
        GeometricMimicSpec,
        GeometricRobotModel,
        ScrewAxisRow,
        ScrewParameters,
        UrdfSource,
    )
    from myext.robot.geometric_newton.model import transforms as tf
    from myext.robot.geometric_newton.newton_backend import (
        BatchedNewtonScene,
        GeometricScene,
        resolve_newton_device,
    )
    from myext.robot.geometric_newton.usd_reader import UsdArticulationSource, read_articulation


def _set_xform(prim, pos=(0.0, 0.0, 0.0), rot_xyz_deg=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    if any(abs(v) > 0.0 for v in pos):
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    if any(abs(v) > 0.0 for v in rot_xyz_deg):
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rot_xyz_deg]))
    if any(abs(v - 1.0) > 0.0 for v in scale):
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))


def _make_sphere_collider(stage, path: str, radius: float, pos=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    s = UsdGeom.Sphere.Define(stage, path)
    s.GetRadiusAttr().Set(float(radius))
    _set_xform(s.GetPrim(), pos=pos, scale=scale)
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())
    return s.GetPrim()


def _make_box_collider(stage, path: str, size: float = 2.0, pos=(0.0, 0.0, 0.0), rot_xyz_deg=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(float(size))
    _set_xform(c.GetPrim(), pos=pos, rot_xyz_deg=rot_xyz_deg, scale=scale)
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c.GetPrim()


def _make_capsule_collider(stage, path: str, radius: float, height: float, axis: str = "Z", pos=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    cap = UsdGeom.Capsule.Define(stage, path)
    cap.GetRadiusAttr().Set(float(radius))
    cap.GetHeightAttr().Set(float(height))
    cap.GetAxisAttr().Set(axis)
    _set_xform(cap.GetPrim(), pos=pos, scale=scale)
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    return cap.GetPrim()


def _make_cylinder_collider(stage, path: str, radius: float, height: float, axis: str = "Z", pos=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(float(radius))
    cyl.GetHeightAttr().Set(float(height))
    cyl.GetAxisAttr().Set(axis)
    _set_xform(cyl.GetPrim(), pos=pos, scale=scale)
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()


def _make_cone_collider(stage, path: str, radius: float, height: float, axis: str = "Z", pos=(0.0, 0.0, 0.0)):
    cone = UsdGeom.Cone.Define(stage, path)
    cone.GetRadiusAttr().Set(float(radius))
    cone.GetHeightAttr().Set(float(height))
    cone.GetAxisAttr().Set(axis)
    _set_xform(cone.GetPrim(), pos=pos)
    UsdPhysics.CollisionAPI.Apply(cone.GetPrim())
    return cone.GetPrim()


def _make_cube_mesh_collider(stage, path: str, hx: float, hy: float, hz: float, pos=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [
        Gf.Vec3f(-hx, -hy, -hz),
        Gf.Vec3f(hx, -hy, -hz),
        Gf.Vec3f(hx, hy, -hz),
        Gf.Vec3f(-hx, hy, -hz),
        Gf.Vec3f(-hx, -hy, hz),
        Gf.Vec3f(hx, -hy, hz),
        Gf.Vec3f(hx, hy, hz),
        Gf.Vec3f(-hx, hy, hz),
    ]
    mesh.GetPointsAttr().Set(pts)
    mesh.GetFaceVertexCountsAttr().Set([4, 4, 4, 4, 4, 4])
    mesh.GetFaceVertexIndicesAttr().Set(
        [
            0, 3, 2, 1,
            4, 5, 6, 7,
            0, 1, 5, 4,
            2, 3, 7, 6,
            0, 4, 7, 3,
            1, 2, 6, 5,
        ]
    )
    _set_xform(mesh.GetPrim(), pos=pos, scale=scale)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh.GetPrim()


def _build_analytical_arm_on_stage(stage, root_path: str = "/World/Arm6", L: float = 0.30, R: float = 0.04):
    """先端に半径 R の球を持つ 6 軸アームを USD ステージと GeometricRobotModel の両方に構築する。"""
    axes = [(0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 0, 1)]
    links = [GeometricLinkSpec("base")] + [GeometricLinkSpec(f"link_{i+1}") for i in range(6)]
    joints = []
    for i in range(6):
        parent = "base" if i == 0 else f"link_{i}"
        pf = np.eye(4) if i == 0 else tf.translation(x=L)
        joints.append(
            GeometricJointSpec(
                name=f"j{i+1}",
                joint_type="revolute",
                parent_link=parent,
                child_link=f"link_{i+1}",
                parent_frame=pf,
                child_frame=np.eye(4),
                axis=axes[i],
                lower_limit=-math.pi,
                upper_limit=math.pi,
            )
        )
    model = GeometricRobotModel(name="arm6", links=tuple(links), joints=tuple(joints), root_link="base")

    UsdGeom.Xform.Define(stage, root_path)
    link_paths: dict[str, str] = {}
    base_path = f"{root_path}/base"
    UsdGeom.Xform.Define(stage, base_path)
    _make_sphere_collider(stage, f"{base_path}/col", radius=0.05)
    link_paths["base"] = base_path

    # 各リンク prim を初期姿勢(q=0)の位置に配置し、ローカル +X=L に球コライダーを付ける
    world_q0 = model.forward_kinematics(np.zeros(6))
    for i in range(6):
        lname = f"link_{i+1}"
        lpath = f"{root_path}/{lname}"
        lp = UsdGeom.Xform.Define(stage, lpath).GetPrim()
        p = world_q0[i + 1, :3, 3]
        _set_xform(lp, pos=tuple(p))
        _make_sphere_collider(stage, f"{lpath}/tip_col", radius=R, pos=(L, 0.0, 0.0))
        link_paths[lname] = lpath

    return model, link_paths


class _MockBatchView:
    """GeometricRobotView と同等のインターフェースを持つ BatchedNewtonScene テスト用軽量ラッパ。"""

    def __init__(self, proxies: list) -> None:
        self.robots = proxies
        for p in proxies:
            if not hasattr(p, "_prim_link_indices"):
                p._prim_link_indices = np.arange(p.model.num_links, dtype=int)


@unittest.skipUnless(HAS_NEWTON, f"newton/warp/pxr が利用不可: {_IMPORT_ERR}")
class GeometricSceneFkTest(unittest.TestCase):
    def setUp(self) -> None:
        self.device = resolve_newton_device("auto")

    def test_fk_matches_numpy_on_random_tree_with_mimic_prismatic_fixed(self) -> None:
        """枝分かれ・fixed・prismatic・連鎖 mimic・動的ベース姿勢で Newton FK が NumPy FK と一致する。"""
        rng = np.random.default_rng(7)
        model = random_tree(rng, num_links=10, fixed_ratio=0.2, with_mimic=True)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        link_paths: dict[str, str] = {}
        for ln in model.link_names:
            lp = f"/World/Tree/{ln}"
            UsdGeom.Xform.Define(stage, lp)
            _make_sphere_collider(stage, f"{lp}/col", radius=0.02)
            link_paths[ln] = lp

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            proxy = scene.add_model(model, link_paths)
            for _ in range(15):
                q_raw = rng.uniform(-1.5, 1.5, size=model.num_dofs)
                base = random_transform(rng, scale=1.0)
                expected = base @ model.forward_kinematics(q_raw, resolve_mimic=True)
                actual = scene.eval_fk(proxy, q_raw, base_matrix=base, resolve_mimic=True)
                np.testing.assert_allclose(actual[:, :3, 3], expected[:, :3, 3], atol=2e-5)
                np.testing.assert_allclose(actual[:, :3, :3], expected[:, :3, :3], atol=2e-5)
        finally:
            scene.close()

    def test_fk_reversed_usd_joint_and_custom_dof_order(self) -> None:
        """body0/body1 が逆転した USD 関節 + 任意の dof_order でも GeometricScene は正しく FK を評価する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        root = UsdGeom.Xform.Define(stage, "/World/RevBot").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)

        l0 = UsdGeom.Xform.Define(stage, "/World/RevBot/L0").GetPrim()
        l1 = UsdGeom.Xform.Define(stage, "/World/RevBot/L1").GetPrim()
        l2 = UsdGeom.Xform.Define(stage, "/World/RevBot/L2").GetPrim()
        for p in (l0, l1, l2):
            UsdPhysics.RigidBodyAPI.Apply(p)
        _set_xform(l1, pos=(0.3, 0.0, 0.0))
        _set_xform(l2, pos=(0.6, 0.0, 0.0))
        _make_sphere_collider(stage, "/World/RevBot/L0/col", radius=0.04)
        _make_sphere_collider(stage, "/World/RevBot/L1/col", radius=0.04)
        _make_sphere_collider(stage, "/World/RevBot/L2/col", radius=0.04)

        # J1: 通常の親子 (body0=L0, body1=L1)
        j1 = UsdPhysics.RevoluteJoint.Define(stage, "/World/RevBot/J1")
        j1.GetAxisAttr().Set("Z")
        j1.GetBody0Rel().SetTargets([l0.GetPath()])
        j1.GetBody1Rel().SetTargets([l1.GetPath()])
        j1.GetLocalPos0Attr().Set(Gf.Vec3f(0.3, 0.0, 0.0))
        j1.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        # J2: 逆転 (body0=L2, body1=L1) -> builder.add_usd は非対応だが GeometricRobotModel 経由なら動く
        j2 = UsdPhysics.RevoluteJoint.Define(stage, "/World/RevBot/J2")
        j2.GetAxisAttr().Set("Y")
        j2.GetBody0Rel().SetTargets([l2.GetPath()])
        j2.GetBody1Rel().SetTargets([l1.GetPath()])
        j2.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j2.GetLocalPos1Attr().Set(Gf.Vec3f(0.3, 0.0, 0.0))

        # カスタム dof_order=["J2", "J1"] で読み込む
        rev_model, rev_info = read_articulation(stage, UsdArticulationSource("/World/RevBot", dof_order=["J2", "J1"]))
        self.assertEqual(rev_model.dof_names, ["J2", "J1"])

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            proxy = scene.add_model(rev_model, rev_info.link_prim_paths)
            q = np.array([0.75, -0.5], dtype=float)
            expected = rev_model.forward_kinematics(q)
            actual = scene.eval_fk(proxy, q)
            np.testing.assert_allclose(actual[:, :3, 3], expected[:, :3, 3], atol=2e-5)
            np.testing.assert_allclose(actual[:, :3, :3], expected[:, :3, :3], atol=2e-5)
        finally:
            scene.close()

    def test_fk_zero_colliders_edge_case(self) -> None:
        """コライダーが 0 個のロボットでも GeometricScene.eval_fk と check_collisions が例外なく動く。"""
        model = six_axis_mdh().to_model()
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        link_paths = build_robot_prims(stage, model, "/World/NoCol", style=GeometricVisualStyle(collisions=False))

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            proxy = scene.add_model(model, link_paths)
            q = np.array([0.2, -0.4, 0.6, -0.3, 0.5, -0.1])
            expected = model.forward_kinematics(q)
            actual = scene.eval_fk(proxy, q)
            np.testing.assert_allclose(actual[:, :3, 3], expected[:, :3, 3], atol=2e-5)
            self.assertEqual(scene.check_collisions(proxy, scope="all"), [])
        finally:
            scene.close()

    def test_fk_non_usd_sources_dh_mdh_screw_urdf_and_attach(self) -> None:
        """DH / MDH / Screw / URDF / attach() で build_robot_prims() した全ソースで FK が一致する。"""
        dh = DHParameters(
            rows=[
                DHRow(theta=0.0, d=0.2, a=0.3, alpha=math.pi / 2),
                DHRow(theta=0.0, d=0.0, a=0.25, alpha=0.0),
            ],
            name="dh2",
        ).to_model()
        mdh = six_axis_mdh().to_model()
        screw = ScrewParameters(
            rows=[
                ScrewAxisRow.through(point=(0, 0, 0), direction=(0, 0, 1), joint_type="revolute", name="s1"),
                ScrewAxisRow(direction=(1, 0, 0), moment=(0, 0, 0), joint_type="prismatic", name="s2"),
            ],
            tool_home=tf.translation(x=0.4, z=0.2),
            name="screw2",
        ).to_model()
        attached = dh.attach(screw, parent_link=dh.link_names[-1], mount_transform=tf.translation(x=0.1), name="tool")

        models = [("dh", dh), ("mdh", mdh), ("screw", screw), ("attached", attached)]
        if os.path.exists(FRANKA_URDF):
            models.append(("urdf_franka", UrdfSource(FRANKA_URDF).to_model()))

        rng = np.random.default_rng(11)
        for tag, model in models:
            with self.subTest(source=tag):
                stage = Usd.Stage.CreateInMemory()
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
                UsdGeom.Xform.Define(stage, "/World")
                link_paths = build_robot_prims(stage, model, f"/World/{tag}", style=GeometricVisualStyle(collisions=True))

                scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
                try:
                    proxy = scene.add_model(model, link_paths)
                    q = rng.uniform(-0.8, 0.8, size=model.num_dofs)
                    expected = model.forward_kinematics(q, resolve_mimic=True)
                    actual = scene.eval_fk(proxy, q, resolve_mimic=True)
                    np.testing.assert_allclose(actual[:, :3, 3], expected[:, :3, 3], atol=2e-5)
                    np.testing.assert_allclose(actual[:, :3, :3], expected[:, :3, :3], atol=2e-5)
                finally:
                    scene.close()


@unittest.skipUnless(HAS_NEWTON, f"newton/warp/pxr が利用不可: {_IMPORT_ERR}")
class GeometricSceneCollisionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.device = resolve_newton_device("auto")

    def test_all_usd_collider_primitives_and_axes(self) -> None:
        """Sphere / Cube(非一様スケール) / Capsule(X,Y,Z) / Cylinder(X,Y,Z) / Cone / Mesh(convex hull) が正しく衝突判定される。"""
        # 1リンクの直動プローブ球(半径 0.05)を各形状へ当てて符号付き距離を検証
        probe_model = GeometricRobotModel(
            name="probe",
            links=(GeometricLinkSpec("base"), GeometricLinkSpec("tip")),
            joints=(
                GeometricJointSpec(
                    "slide_x", "prismatic", "base", "tip",
                    parent_frame=np.eye(4), child_frame=np.eye(4), axis=(1, 0, 0),
                    lower_limit=-2.0, upper_limit=2.0,
                ),
            ),
            root_link="base",
        )

        # (ケース名, 障害物生成関数(stage, path), 衝突時のプローブX, 期待する符号付き距離)
        # プローブ球は半径 0.05、中心 (x, 0, 0)
        cases = [
            ("sphere", lambda st, p: _make_sphere_collider(st, p, radius=0.10, pos=(0.50, 0.0, 0.0)), 0.38, -0.03),
            # Cube: size=2, scale=(0.05, 0.10, 0.15) -> hx=0.05 -> 左面 x=0.45; プローブ中心 0.42 -> 右端 0.47 -> sep=-0.02
            ("cube_scaled", lambda st, p: _make_box_collider(st, p, size=2.0, pos=(0.50, 0.0, 0.0), scale=(0.05, 0.10, 0.15)), 0.42, -0.02),
            # Capsule axis=X: r=0.05, h=0.20 -> X方向に half_height+r = 0.15 張り出す -> 左端 x=0.35; プローブ中心 0.32 -> 右端 0.37 -> sep=-0.02
            ("capsule_x", lambda st, p: _make_capsule_collider(st, p, radius=0.05, height=0.20, axis="X", pos=(0.50, 0.0, 0.0)), 0.32, -0.02),
            # Capsule axis=Y: X方向張り出しは r=0.05 -> 左端 x=0.45; プローブ中心 0.42 -> 右端 0.47 -> sep=-0.02
            ("capsule_y", lambda st, p: _make_capsule_collider(st, p, radius=0.05, height=0.20, axis="Y", pos=(0.50, 0.0, 0.0)), 0.42, -0.02),
            ("capsule_z", lambda st, p: _make_capsule_collider(st, p, radius=0.05, height=0.20, axis="Z", pos=(0.50, 0.0, 0.0)), 0.42, -0.02),
            # Cylinder axis=X: r=0.05, h=0.20 -> 左面 x=0.40; プローブ中心 0.37 -> 右端 0.42 -> sep=-0.02
            ("cylinder_x", lambda st, p: _make_cylinder_collider(st, p, radius=0.05, height=0.20, axis="X", pos=(0.50, 0.0, 0.0)), 0.37, -0.02),
            ("cylinder_y", lambda st, p: _make_cylinder_collider(st, p, radius=0.05, height=0.20, axis="Y", pos=(0.50, 0.0, 0.0)), 0.42, -0.02),
            ("cylinder_z", lambda st, p: _make_cylinder_collider(st, p, radius=0.05, height=0.20, axis="Z", pos=(0.50, 0.0, 0.0)), 0.42, -0.02),
            # Cone axis=Z / axis=X
            ("cone_z", lambda st, p: _make_cone_collider(st, p, radius=0.10, height=0.20, axis="Z", pos=(0.50, 0.0, 0.0)), 0.42, None),
            ("cone_x", lambda st, p: _make_cone_collider(st, p, radius=0.10, height=0.20, axis="X", pos=(0.50, 0.0, 0.0)), 0.37, None),
            # Mesh (convex hull): hx=0.10, scale=(0.5,1,1) -> 実効 hx=0.05 -> 左面 x=0.45; プローブ中心 0.42 -> 右端 0.47 -> sep=-0.02
            ("mesh_convex", lambda st, p: _make_cube_mesh_collider(st, p, 0.10, 0.10, 0.10, pos=(0.50, 0.0, 0.0), scale=(0.5, 1.0, 1.0)), 0.42, -0.02),
        ]

        for name, make_obs, q_hit, expected_sep in cases:
            with self.subTest(primitive=name):
                stage = Usd.Stage.CreateInMemory()
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
                UsdGeom.Xform.Define(stage, "/World")
                UsdGeom.Xform.Define(stage, "/World/Probe/base")
                UsdGeom.Xform.Define(stage, "/World/Probe/tip")
                _make_sphere_collider(stage, "/World/Probe/tip/col", radius=0.05)
                obs_path = "/World/Obs/geom"
                UsdGeom.Xform.Define(stage, "/World/Obs")
                make_obs(stage, obs_path)

                scene = GeometricScene(stage, device=self.device)
                try:
                    proxy = scene.add_model(probe_model, {"base": "/World/Probe/base", "tip": "/World/Probe/tip"})
                    # 離れた位置: 非衝突
                    proxy.set_pose(np.array([-0.5]))
                    self.assertEqual(scene.check_collisions(proxy, scope="environment"), [])

                    # 衝突位置
                    proxy.set_pose(np.array([q_hit]))
                    hits = scene.check_collisions(proxy, scope="environment")
                    self.assertEqual(len(hits), 1, f"{name}: 衝突を検出できなかった")
                    h = hits[0]
                    self.assertEqual(h.kind, "environment")
                    self.assertEqual(h.link_a, "tip")
                    self.assertIsNone(h.link_b)
                    self.assertEqual(h.path_b, obs_path)
                    self.assertIsNotNone(h.separation)
                    self.assertLess(h.separation, 0.0)
                    # 法線は A(左) -> B(右) なので +X 成分が正
                    self.assertIsNotNone(h.normal)
                    self.assertGreater(h.normal[0], 0.5)
                    if expected_sep is not None:
                        self.assertAlmostEqual(h.separation, expected_sep, delta=2e-4)
                finally:
                    scene.close()

    def test_self_collision_penetration_filtering_and_touching_pairs(self) -> None:
        """6 軸アームの自己干渉・符号付き距離・法線向き・フィルタ・touching_pairs を検証する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        model, link_paths = _build_analytical_arm_on_stage(stage)

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            proxy = scene.add_model(model, link_paths)
            # 直線姿勢: 衝突なし
            proxy.set_pose(np.zeros(6))
            self.assertEqual(scene.check_collisions(proxy, scope="within"), [])

            # 折りたたみ姿勢: link_1 先端と link_5 先端(半径 0.04 + 0.04 = 0.08)が距離 0.0599 m で干渉 -> sep = -0.020100 m
            q_self = np.array([0.0, 0.0, math.pi / 2, math.pi / 2, math.pi / 2 - 0.2, 0.0])
            proxy.set_pose(q_self)
            hits = scene.check_collisions(proxy, scope="within")
            self.assertEqual(len(hits), 1)
            h = hits[0]
            self.assertEqual(h.kind, "within_component")
            self.assertEqual(h.names, ("link_1", "link_5"))

            # 解析解と突き合わせ
            expected_sep = 2.0 * 0.30 * math.sin(0.1) - 0.08
            self.assertAlmostEqual(h.separation, expected_sep, delta=1e-4)
            self.assertIsNotNone(h.point_a)
            self.assertIsNotNone(h.point_b)
            self.assertIsNotNone(h.normal)
            # 法線は link_1 -> link_5 向き、かつ dot(point_b - point_a, normal) == separation
            diff = np.array(h.point_b) - np.array(h.point_a)
            nrm = np.array(h.normal)
            self.assertAlmostEqual(float(np.linalg.norm(nrm)), 1.0, delta=1e-4)
            self.assertAlmostEqual(float(np.dot(diff, nrm)), h.separation, delta=2e-4)

            # extra_filter で除外できる
            self.assertEqual(
                scene.check_collisions(proxy, scope="within", extra_filter=lambda a, b: {a, b} == {"link_1", "link_5"}),
                [],
            )

            # collision_filter に無視ペアを足せば消え、touching_pairs() はフィルタ無関係に検出する
            proxy.collision_filter.add_pair("link_1", "link_5")
            self.assertEqual(scene.check_collisions(proxy, scope="within"), [])
            touching = scene.touching_pairs(proxy.link_set)
            self.assertTrue(any(p.names == ("link_1", "link_5") for p in touching))
        finally:
            scene.close()

    def test_dynamic_obstacle_sync_environment_and_set_obstacle_pose_and_clearances(self) -> None:
        """モデル再構築なしで障害物移動(sync_environment / set_obstacle_pose)とクリアランス計測が 0 遅延で動く。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        model, link_paths = _build_analytical_arm_on_stage(stage)

        UsdGeom.Xform.Define(stage, "/World/Obstacles")
        obs_path = "/World/Obstacles/BoxA"
        obs_prim = _make_box_collider(stage, obs_path, size=0.20, pos=(5.0, 0.0, 0.0))  # half-extent = 0.10
        ignored_path = "/World/Obstacles/IgnoredBox"
        _make_box_collider(stage, ignored_path, size=0.20, pos=(1.80, 0.0, 0.0))  # 常に重なるが exclude する

        scene = GeometricScene(stage, device=self.device)
        try:
            proxy = scene.add_model(model, link_paths)
            scene.add_environment("/World/Obstacles", exclude_prim_paths=[ignored_path])
            proxy.set_pose(np.zeros(6))  # link_6 先端球は中心 x=1.80, 半径 0.04 -> 右端 x=1.84

            # finalize 時の newton_model インスタンスを覚えておき、再構築されないことを検証する
            model_before = scene.newton_model
            self.assertEqual(scene.environment_collider_paths, [obs_path])
            self.assertEqual(scene.check_collisions(proxy, scope="environment"), [])

            # 1) USD 上で障害物を x=1.97 へ移動 (左面 1.87 vs 球右端 1.84 -> クリアランス +0.03 m)
            _set_xform(obs_prim, pos=(1.97, 0.0, 0.0))
            scene.sync_environment(use_fabric=False)
            self.assertIs(scene.newton_model, model_before, "sync_environment はモデルを再構築してはいけない")

            # clearance=0.0 -> 非衝突
            self.assertEqual(scene.check_collisions(proxy, scope="environment", clearance=0.0), [])
            # compute_clearances(max_distance=0.05) -> +0.030 m を検出
            near = scene.compute_clearances(proxy, scope="environment", max_distance=0.05)
            self.assertEqual(len(near), 1)
            self.assertAlmostEqual(near[0].separation, 0.03, delta=1e-4)
            # 次の clearance=0.0 呼び出しで shape_gap が 0 に戻り非衝突に戻る
            self.assertEqual(scene.check_collisions(proxy, scope="environment", clearance=0.0), [])

            # 2) set_obstacle_pose で直接 x=1.915 へ移動 (左面 1.815 vs 球右端 1.84 -> sep = -0.025 m)
            scene.set_obstacle_pose(obs_path, position=(1.915, 0.0, 0.0))
            self.assertIs(scene.newton_model, model_before, "set_obstacle_pose はモデルを再構築してはいけない")
            # check_collisions(sync_env=True) でも manual_override は保持される
            hits = scene.check_collisions(proxy, scope="environment", sync_env=True)
            self.assertEqual(len(hits), 1)
            self.assertAlmostEqual(hits[0].separation, -0.025, delta=1e-4)

            # 負の clearance 閾値: -0.03 なら浅いめり込み(-0.025)は無視、-0.01 なら検出
            self.assertEqual(scene.check_collisions(proxy, scope="environment", clearance=-0.03), [])
            self.assertEqual(len(scene.check_collisions(proxy, scope="environment", clearance=-0.01)), 1)

            # 明示 sync_environment(reset_manual_overrides=True) で USD 位置 (x=1.97) へ戻る -> 非衝突
            scene.sync_environment(use_fabric=False, reset_manual_overrides=True)
            self.assertEqual(scene.check_collisions(proxy, scope="environment"), [])
        finally:
            scene.close()

    def test_set_obstacle_pose_on_parent_xform_preserves_child_local_offset(self) -> None:
        """親 Xform に対する set_obstacle_pose が子コライダーのローカルオフセットを回転込みで保持する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        probe_model = GeometricRobotModel(
            name="probe",
            links=(GeometricLinkSpec("base"), GeometricLinkSpec("tip")),
            joints=(GeometricJointSpec("slide", "prismatic", "base", "tip", parent_frame=np.eye(4), axis=(1, 0, 0)),),
            root_link="base",
        )
        UsdGeom.Xform.Define(stage, "/World/Probe/base")
        UsdGeom.Xform.Define(stage, "/World/Probe/tip")
        _make_sphere_collider(stage, "/World/Probe/tip/col", radius=0.05)

        # 親 Xform /World/Table の子 /World/Table/Top がローカル (0, 0, +0.5) にある
        table = UsdGeom.Xform.Define(stage, "/World/Table").GetPrim()
        _set_xform(table, pos=(5.0, 0.0, 0.0))
        _make_sphere_collider(stage, "/World/Table/Top", radius=0.05, pos=(0.0, 0.0, 0.5))

        scene = GeometricScene(stage, device=self.device)
        try:
            proxy = scene.add_model(probe_model, {"base": "/World/Probe/base", "tip": "/World/Probe/tip"})
            scene.add_environment("/World/Table")
            proxy.set_pose(np.array([0.50]))  # プローブ球中心 (0.50, 0, 0), 半径 0.05

            # 親 /World/Table を原点へ置き Y 軸 +90 度回転 -> ローカル (0,0,+0.5) がワールド (+0.5, 0, 0) へ来て完全に重なる!
            q_y90 = tf.matrix_to_quat(tf.rotation_about_axis((0.0, 1.0, 0.0), math.pi / 2.0))
            scene.set_obstacle_pose("/World/Table", position=(0.0, 0.0, 0.0), orientation=q_y90)
            hits = scene.check_collisions(proxy, scope="environment")
            self.assertEqual(len(hits), 1)
            self.assertAlmostEqual(hits[0].separation, -0.10, delta=2e-4)

            # 回転なし(単位四元数)で原点に置くと子は (0,0,+0.5) -> プローブ (0.5,0,0) とは距離 0.707m で非衝突
            scene.set_obstacle_pose("/World/Table", position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))
            self.assertEqual(scene.check_collisions(proxy, scope="environment"), [])
        finally:
            scene.close()

    def test_multi_robot_between_and_attached_components_and_rebuild(self) -> None:
        """共有 GeometricScene 上の複数ロボット間干渉・attach 済みコンポーネント区別・structure_version 自動再構築。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # 2 台の 1 リンクロボットを共有シーンへ登録
        m1, p1 = _build_analytical_arm_on_stage(stage, root_path="/World/ArmA")
        m2, p2 = _build_analytical_arm_on_stage(stage, root_path="/World/ArmB")

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            r1 = scene.add_model(m1, p1, name="ArmA", base_matrix=np.eye(4))
            # ArmB を x=3.65 に置き、Z 軸 180 度回して向かい合わせる
            # ArmA link_6 先端は (1.80,0,0), R=0.04; ArmB link_6 先端は (3.65-1.80,0,0)=(1.85,0,0), R=0.04 -> 距離 0.05 -> sep = -0.03
            base2 = tf.make_transform(tf.rot_z(math.pi)[:3, :3], np.array([3.65, 0.0, 0.0]))
            r2 = scene.add_model(m2, p2, name="ArmB", base_matrix=base2)
            r1.set_pose(np.zeros(6))
            r2.set_pose(np.zeros(6))

            between = scene.check_between(r1.link_set, r2.link_set)
            self.assertEqual(len(between), 1)
            b = between[0]
            self.assertEqual(b.kind, "between_robots")
            self.assertEqual(b.robot_a, "ArmA")
            self.assertEqual(b.robot_b, "ArmB")
            self.assertEqual((b.link_a, b.link_b), ("link_6", "link_6"))
            self.assertAlmostEqual(b.separation, -0.03, delta=1e-4)
            # 法線は ArmA(+1.80) -> ArmB(+1.85) なので +X
            self.assertGreater(b.normal[0], 0.99)

            # structure_version が増えると次回クエリで自動再構築される
            old_model = scene.newton_model
            r1.structure_version += 1
            _ = scene.check_between(r1.link_set, r2.link_set)
            self.assertIsNot(scene.newton_model, old_model)
        finally:
            scene.close()

        # close() 後のアクセスは RuntimeError
        with self.assertRaises(RuntimeError):
            _ = scene.newton_model

    def test_attached_robot_component_pair_kind(self) -> None:
        """attach() で結合したロボットの within_component / between_components の分類を検証する。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        arm = GeometricRobotModel(
            name="arm",
            links=(GeometricLinkSpec("a0"), GeometricLinkSpec("a1")),
            joints=(GeometricJointSpec("ja", "revolute", "a0", "a1", parent_frame=tf.translation(x=0.3), axis=(0, 1, 0)),),
            root_link="a0",
        )
        tool = GeometricRobotModel(
            name="tool",
            links=(GeometricLinkSpec("t0"), GeometricLinkSpec("t1")),
            joints=(GeometricJointSpec("jt", "revolute", "t0", "t1", parent_frame=np.eye(4), axis=(0, 1, 0)),),
            root_link="t0",
        )
        combined = arm.attach(tool, parent_link="a1", mount_transform=np.eye(4), name="tool")
        link_paths: dict[str, str] = {}
        world_q0 = combined.forward_kinematics(np.zeros(2))
        for i, ln in enumerate(combined.link_names):
            lp = f"/World/Comb/{ln.replace('/', '__')}"
            p = UsdGeom.Xform.Define(stage, lp).GetPrim()
            _set_xform(p, pos=tuple(world_q0[i, :3, 3]))
            # tool/t1 の先端(+0.3, 0, 0)に球を置くと jt=pi で a0 原点へ戻る
            col_pos = (0.3, 0.0, 0.0) if ln == "tool/t1" else (0.0, 0.0, 0.0)
            _make_sphere_collider(stage, f"{lp}/col", radius=0.06, pos=col_pos)
            link_paths[ln] = lp

        scene = GeometricScene(stage, device=self.device, auto_discover_environment=False)
        try:
            proxy = scene.add_model(combined, link_paths)
            # ja=0, jt=pi で tool/t1 先端球が (0.3 - 0.3, 0, 0) = (0, 0, 0) = a0 と完全に重なる (半径和 0.12 -> sep = -0.12)
            proxy.set_pose(np.array([0.0, math.pi]))
            hits = scene.check_collisions(proxy, scope="within")
            target = [h for h in hits if {h.link_a, h.link_b} == {"a0", "tool/t1"}]
            self.assertEqual(len(target), 1)
            self.assertEqual(target[0].kind, "between_components")
            self.assertEqual({target[0].component_a, target[0].component_b}, {"arm", "tool"})
            self.assertAlmostEqual(target[0].separation, -0.12, delta=1e-4)
        finally:
            scene.close()


@unittest.skipUnless(HAS_NEWTON, f"newton/warp/pxr が利用不可: {_IMPORT_ERR}")
class BatchedNewtonSceneTest(unittest.TestCase):
    def _build_batch_fixture(self, num_envs: int = 8):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # テンプレートは 6 軸アーム(j6 を j5 の mimic にした版も確認するため標準 6 軸を使用)
        model, link_paths0 = _build_analytical_arm_on_stage(stage, root_path="/World/Env0/Arm")
        proxies = []
        for i in range(num_envs):
            if i == 0:
                lp = link_paths0
            else:
                _, lp = _build_analytical_arm_on_stage(stage, root_path=f"/World/Env{i}/Arm")
            scene_tmp = GeometricScene(stage, device="cpu", auto_discover_environment=False)
            proxy = scene_tmp.add_model(model, lp, name=f"Arm_{i}")
            scene_tmp.close()
            proxies.append(proxy)

        # 共有環境障害物 (world=-1): 初期位置 x=1.915 (サイズ 0.20 -> 左面 1.815; 直線アーム先端右端 1.84 -> sep=-0.025)
        UsdGeom.Xform.Define(stage, "/World/SharedObs")
        obs_path = "/World/SharedObs/Box"
        obs_prim = _make_box_collider(stage, obs_path, size=0.20, pos=(1.915, 0.0, 0.0))

        view = _MockBatchView(proxies)
        return stage, model, view, obs_prim, obs_path

    def _run_batch_assertions(self, device: str, use_cuda_graph: bool) -> None:
        B = 8
        stage, model, view, obs_prim, obs_path = self._build_batch_fixture(num_envs=B)
        bscene = BatchedNewtonScene(
            view,
            device=device,
            environment_roots=["/World/SharedObs"],
            use_cuda_graph=use_cuda_graph,
        )
        try:
            rng = np.random.default_rng(99)
            q_batch = np.zeros((B, 6), dtype=float)
            base_batch = np.broadcast_to(np.eye(4), (B, 4, 4)).copy()

            # Env 0: 上を向いて障害物も自己干渉も回避 (safe)
            q_batch[0] = [0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0]
            # Env 1: 自己干渉ポーズ (link_1 vs link_5, sep = -0.0201 m)
            q_batch[1] = [0.0, 0.0, math.pi / 2, math.pi / 2, math.pi / 2 - 0.2, 0.0]
            # Env 2: 直線ポーズ -> 共有障害物 x=1.915 にめり込み (sep = -0.025 m)
            q_batch[2] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Env 3: ベースを -0.055 m 後ろへ -> 障害物左面 1.815 vs 先端右端 1.785 -> クリアランス +0.030 m
            base_batch[3, 0, 3] = -0.055
            # Env 4..7: Env 0 と同じ原点に重ねても別ワールド同士は一切干渉しないことを確認
            for w in range(4, B):
                q_batch[w] = [rng.uniform(-1.0, 1.0), -math.pi / 2, 0.0, 0.0, 0.0, 0.0]

            # 1) FK バッチ一致検証
            fk_actual = bscene.eval_fk_batch(q_batch, base_batch, resolve_mimic=True, return_matrices=True)
            fk_expected = base_batch[:, None] @ model.forward_kinematics(q_batch, resolve_mimic=True)
            np.testing.assert_allclose(fk_actual[:, :, :3, 3], fk_expected[:, :, :3, 3], atol=2e-5)
            np.testing.assert_allclose(fk_actual[:, :, :3, :3], fk_expected[:, :, :3, :3], atol=2e-5)

            # 2) バッチ干渉チェック (clearance=0.0)
            res = bscene.check_collisions_batch(q_batch, base_batch, scope="all", clearance=0.0, sync_env=False)
            self.assertEqual(len(res), B)
            self.assertEqual(res[0], [], "Env 0 は非衝突のはず")

            # Env 1: 自己干渉
            self.assertEqual(len(res[1]), 1)
            self.assertEqual(res[1][0].kind, "within_component")
            self.assertEqual(res[1][0].names, ("link_1", "link_5"))
            self.assertAlmostEqual(res[1][0].separation, 2.0 * 0.30 * math.sin(0.1) - 0.08, delta=1e-4)

            # Env 2: 共有障害物との衝突
            self.assertEqual(len(res[2]), 1)
            self.assertEqual(res[2][0].kind, "environment")
            self.assertEqual(res[2][0].link_a, "link_6")
            self.assertEqual(res[2][0].path_b, obs_path)
            self.assertAlmostEqual(res[2][0].separation, -0.025, delta=1e-4)

            # Env 3: クリアランス +0.030 m -> clearance=0.0 では非衝突
            self.assertEqual(res[3], [])
            # Env 4..7: 他ワールドと重なっていてもクロスワールド干渉ゼロ
            for w in range(4, B):
                self.assertEqual(res[w], [], f"Env {w} にクロスワールド誤検出")

            # 3) compute_clearances_batch(max_distance=0.05) -> Env 3 で +0.030 m を検出 (CUDA Graph 有効時も動作!)
            clr = bscene.compute_clearances_batch(q_batch, base_batch, scope="environment", max_distance=0.05, sync_env=False)
            self.assertEqual(len(clr[3]), 1)
            self.assertAlmostEqual(clr[3][0].separation, 0.030, delta=1e-4)

            # 4) 共有障害物を USD 上で移動 -> sync_environment -> CUDA Graph でも即座に反映 (0 フレーム遅延)
            _set_xform(obs_prim, pos=(5.0, 0.0, 0.0))
            bscene.sync_environment(use_fabric=False)
            res_after = bscene.check_collisions_batch(q_batch, base_batch, scope="environment", clearance=0.0, sync_env=False)
            self.assertEqual(res_after[2], [], "障害物退避後は Env 2 も非衝突になるはず")
        finally:
            bscene.close()

    def test_batched_cpu(self) -> None:
        self._run_batch_assertions(device="cpu", use_cuda_graph=False)

    def test_batched_cuda_with_and_without_graph(self) -> None:
        if not wp.is_cuda_available():
            self.skipTest("CUDA が利用不可")
        self._run_batch_assertions(device="cuda:0", use_cuda_graph=False)
        self._run_batch_assertions(device="cuda:0", use_cuda_graph=True)

    def test_batched_check_collisions_resolves_mimic_by_default(self) -> None:
        """BatchedNewtonScene.check_collisions_batch に未解決の mimic 関節値を直接渡しても従動軸が反映される。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # 2 本の直動指: finger2 は finger1 の mimic (-1.0) -> finger1=+0.06 で両方 x=0 に来て重なる
        gripper = GeometricRobotModel(
            name="gripper",
            links=(GeometricLinkSpec("palm"), GeometricLinkSpec("f1"), GeometricLinkSpec("f2")),
            joints=(
                GeometricJointSpec("j1", "prismatic", "palm", "f1", parent_frame=tf.translation(x=-0.06), axis=(1, 0, 0)),
                GeometricJointSpec(
                    "j2", "prismatic", "palm", "f2", parent_frame=tf.translation(x=0.06), axis=(1, 0, 0),
                    mimic=GeometricMimicSpec("j1", -1.0, 0.0),
                ),
            ),
            root_link="palm",
        )
        proxies = []
        for i in range(4):
            root = f"/World/G{i}"
            UsdGeom.Xform.Define(stage, root)
            lp = {}
            for ln, x0 in (("palm", 0.0), ("f1", -0.06), ("f2", 0.06)):
                p = f"{root}/{ln}"
                prim = UsdGeom.Xform.Define(stage, p).GetPrim()
                _set_xform(prim, pos=(x0, 0.0, 0.0))
                if ln != "palm":
                    _make_sphere_collider(stage, f"{p}/col", radius=0.03)
                lp[ln] = p
            stmp = GeometricScene(stage, device="cpu", auto_discover_environment=False)
            proxies.append(stmp.add_model(gripper, lp, name=f"G{i}"))
            stmp.close()

        bscene = BatchedNewtonScene(_MockBatchView(proxies), device=resolve_newton_device("auto"), environment_roots=[])
        try:
            # j2=0.0 のまま未解決の q_batch を渡す: Env 0 は開(0.0), Env 1 は閉(+0.05 -> 距離 0.02, 半径和 0.06 -> sep=-0.04)
            q_raw = np.array([[0.0, 0.0], [0.05, 0.0], [0.0, 0.0], [0.05, 0.0]], dtype=float)
            bases = np.broadcast_to(np.eye(4), (4, 4, 4)).copy()
            res = bscene.check_collisions_batch(q_raw, bases, scope="within", sync_env=False)
            self.assertEqual(res[0], [])
            self.assertEqual(len(res[1]), 1)
            self.assertEqual(res[1][0].names, ("f1", "f2"))
            self.assertAlmostEqual(res[1][0].separation, -0.04, delta=1e-4)
            self.assertEqual(res[2], [])
            self.assertEqual(len(res[3]), 1)
        finally:
            bscene.close()


if __name__ == "__main__":
    unittest.main()
