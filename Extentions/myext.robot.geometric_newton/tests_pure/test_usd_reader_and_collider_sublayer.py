"""`usd_reader` のマルチアーム・ルート解決と、カスタムコライダー USD Sublayer ON/OFF の純粋テスト。

Kit / SimulationApp を起動せずに、次の 2 系統を検証する:
1. URDF Converter v0.3.2 / `assembly_pipeline` (`multi_arm.toml`) 構造の USD 読取:
   - `ArticulationRootAPI` が `Geometry/base_body` にあり、関節が兄弟 `Physics/` に分かれている構成
   - `Geometry/` が `Physics/` より辞書順で先に走査される構成でのルート選定
   - 独立した複数アーム (非連結成分) が 1 ルート配下に同居する構成での仮想 `world` ルート統合
   - 兄弟ロボットを巻き込まないルート解決
   - 純 OpenUSD 環境での `NewtonMimicAPI` (`newton:mimicJoint`) 検出と `KinematicChain` FK 一致
2. カスタムコライダー用 `.usda` の USD Sublayer ON/OFF:
   - `GeometricRobot.attach_collider_layer` / `set_collider_layer_enabled` / `detach_collider_layer` / `reload_colliders`
   - 手動の `subLayerPaths` 挿抜・`MuteLayer` / `UnmuteLayer`・属性変更に対する自動検知 (0 フレーム遅延)
   - 通常姿勢更新 (`write_target="usd"`) での誤キャッシュ破棄が無いこと
   - `GeometricScene.invalidate()` と `GeometricRobotView.reload_colliders()` / 自動同期
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


def _ensure_isaac_newton_imports() -> tuple[bool, str]:
    sys.path[:] = [p for p in sys.path if "BuiltOpenUSD" not in p]

    def _try_import() -> bool:
        try:
            import newton  # noqa: F401
            import warp  # noqa: F401
            from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: F401

            return True
        except Exception:
            return False

    if not _try_import():
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
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    from myext.robot.geometric_newton.newton_backend import GeometricScene
    from myext.robot.geometric_newton.robot import GeometricRobot
    from myext.robot.geometric_newton.robot_view import GeometricRobotView
    from myext.robot.geometric_newton.usd_reader import read_articulation


# ==============================================================================
# USD ステージ構築ヘルパー
# ==============================================================================


def _def_body(stage: "Usd.Stage", path: str, pos: tuple[float, float, float] = (0.0, 0.0, 0.0), *, art_root: bool = False):
    xf = UsdGeom.Xform.Define(stage, path)
    if pos != (0.0, 0.0, 0.0):
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    if art_root:
        UsdPhysics.ArticulationRootAPI.Apply(xf.GetPrim())
    return xf.GetPrim()


def _def_joint(
    stage: "Usd.Stage",
    path: str,
    kind: str,
    body0: str | None,
    body1: str,
    *,
    pos0: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pos1: tuple[float, float, float] = (0.0, 0.0, 0.0),
    axis: str = "Z",
    limits: tuple[float, float] | None = None,
):
    if kind == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    elif kind == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateAxisAttr(axis)
        if limits is not None:
            joint.CreateLowerLimitAttr(float(limits[0]))
            joint.CreateUpperLimitAttr(float(limits[1]))
    elif kind == "prismatic":
        joint = UsdPhysics.PrismaticJoint.Define(stage, path)
        joint.CreateAxisAttr(axis)
        if limits is not None:
            joint.CreateLowerLimitAttr(float(limits[0]))
            joint.CreateUpperLimitAttr(float(limits[1]))
    else:
        raise ValueError(kind)
    if body0 is not None:
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return joint.GetPrim()


def _build_synthetic_multi_arm_stage() -> "Usd.Stage":
    """`multi_arm.toml` 相当の URDF Converter v0.3.2 構造ステージを生成する。

    - `/robot/Geometry/base_body` に `ArticulationRootAPI`
    - `/robot/Physics/root_joint`, `/robot/Physics/turret_joint` が兄弟 `Physics/` 配下に配置
    - `/robot/Geometry/turret/port1/arm1`, `/robot/Geometry/turret/port2/arm2` にそれぞれ
      2-DOF アーム + `NewtonMimicAPI` 付き 2 指グリッパ (`finger_r_joint -> finger_l_joint`) を接続
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/robot")
    UsdGeom.Xform.Define(stage, "/robot/Geometry")
    UsdGeom.Xform.Define(stage, "/robot/Physics")

    _def_body(stage, "/robot/Geometry/base_body", (0.0, 0.0, 0.1), art_root=True)
    _def_body(stage, "/robot/Geometry/turret", (0.0, 0.0, 0.3))
    _def_joint(stage, "/robot/Physics/root_joint", "fixed", None, "/robot/Geometry/base_body", pos0=(0.0, 0.0, 0.1))
    _def_joint(
        stage,
        "/robot/Physics/turret_joint",
        "revolute",
        "/robot/Geometry/base_body",
        "/robot/Geometry/turret",
        pos0=(0.0, 0.0, 0.2),
        axis="Z",
        limits=(-180.0, 180.0),
    )

    for idx, x_off in ((1, 0.4), (2, -0.4)):
        port = f"/robot/Geometry/turret/port{idx}"
        arm = f"{port}/arm{idx}"
        UsdGeom.Xform.Define(stage, port)
        UsdGeom.Xform.Define(stage, arm)
        UsdGeom.Xform.Define(stage, f"{arm}/Geometry")
        UsdGeom.Xform.Define(stage, f"{arm}/Physics")
        _def_body(stage, f"{arm}/Geometry/arm_base", (x_off, 0.0, 0.3))
        _def_body(stage, f"{arm}/Geometry/arm_link1", (x_off, 0.0, 0.6))
        _def_body(stage, f"{arm}/Geometry/gripper_palm", (x_off, 0.0, 0.8))
        _def_body(stage, f"{arm}/Geometry/finger_l", (x_off + 0.02, 0.0, 0.85))
        _def_body(stage, f"{arm}/Geometry/finger_r", (x_off - 0.02, 0.0, 0.85))

        _def_joint(
            stage,
            f"{arm}/Physics/mount_joint",
            "fixed",
            "/robot/Geometry/turret",
            f"{arm}/Geometry/arm_base",
            pos0=(x_off, 0.0, 0.0),
        )
        _def_joint(
            stage,
            f"{arm}/Physics/j1",
            "revolute",
            f"{arm}/Geometry/arm_base",
            f"{arm}/Geometry/arm_link1",
            pos0=(0.0, 0.0, 0.3),
            axis="Y",
            limits=(-90.0, 90.0),
        )
        _def_joint(
            stage,
            f"{arm}/Physics/palm_mount",
            "fixed",
            f"{arm}/Geometry/arm_link1",
            f"{arm}/Geometry/gripper_palm",
            pos0=(0.0, 0.0, 0.2),
        )
        fl = _def_joint(
            stage,
            f"{arm}/Physics/finger_l_joint",
            "prismatic",
            f"{arm}/Geometry/gripper_palm",
            f"{arm}/Geometry/finger_l",
            pos0=(0.02, 0.0, 0.05),
            axis="X",
            limits=(0.0, 0.04),
        )
        fr = _def_joint(
            stage,
            f"{arm}/Physics/finger_r_joint",
            "prismatic",
            f"{arm}/Geometry/gripper_palm",
            f"{arm}/Geometry/finger_r",
            pos0=(-0.02, 0.0, 0.05),
            axis="X",
            limits=(-0.04, 0.0),
        )
        # 純 OpenUSD 環境で `NewtonMimicAPI` スキーマ未登録でも関係属性だけで mimic を記述
        fr.CreateRelationship("newton:mimicJoint").SetTargets([fl.GetPath()])
        fr.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(0.0)
        fr.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(-1.0)
    return stage


def _build_collider_toggle_stage(robot_root: str = "/World/Robot", obstacle_x: float = 1.35) -> "Usd.Stage":
    """2 リンクロボット (`base_link`, `tip_link`) と静的障害物 1 個のステージを作る。

    `tip_link` は原点から `x = 1.0` にあり、元コライダー `orig_col` は半径 `0.35 m` の球。
    障害物 `/World/Obstacle/box` は中心 `x = 1.35`・一辺 `0.20 m` (手前面 `x = 1.25`)。
    - 元コライダー (`r = 0.35`): 球面 `x = 1.35` vs 障害物面 `x = 1.25` -> めり込み `0.10 m` (`separation ≈ -0.10`)
    - カスタムコライダー (`r = 0.10`): 球面 `x = 1.10` vs 障害物面 `x = 1.25` -> 隙間 `0.15 m` (`separation ≈ +0.15`)
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    _add_two_link_robot(stage, robot_root, y_offset=0.0)

    UsdGeom.Xform.Define(stage, "/World/Obstacle")
    box = UsdGeom.Cube.Define(stage, "/World/Obstacle/box")
    box.CreateSizeAttr(0.20)
    box.AddTranslateOp().Set(Gf.Vec3d(obstacle_x, 0.0, 0.0))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    return stage


def _add_two_link_robot(stage: "Usd.Stage", robot_root: str, *, y_offset: float = 0.0) -> None:
    UsdGeom.Xform.Define(stage, robot_root)
    base = _def_body(stage, f"{robot_root}/base_link", (0.0, y_offset, 0.0), art_root=True)
    base_col = UsdGeom.Sphere.Define(stage, f"{robot_root}/base_link/base_col")
    base_col.CreateRadiusAttr(0.05)
    UsdPhysics.CollisionAPI.Apply(base_col.GetPrim())

    tip = _def_body(stage, f"{robot_root}/tip_link", (1.0, y_offset, 0.0))
    tip_col = UsdGeom.Sphere.Define(stage, f"{robot_root}/tip_link/orig_col")
    tip_col.CreateRadiusAttr(0.35)
    UsdPhysics.CollisionAPI.Apply(tip_col.GetPrim())

    _def_joint(
        stage,
        f"{robot_root}/j1",
        "revolute",
        str(base.GetPath()),
        str(tip.GetPath()),
        pos0=(1.0, 0.0, 0.0),
        axis="Z",
        limits=(-180.0, 180.0),
    )


def _custom_collider_usda_text(robot_roots: list[str], custom_radius: float = 0.10) -> str:
    """元 `orig_col` を `physics:collisionEnabled = false` にし、`custom_col` を定義する Sublayer テキスト。"""
    blocks = []
    for root in robot_roots:
        name = root.strip("/").split("/")[-1]
        blocks.append(
            f'''        over "{name}"
        {{
            over "tip_link"
            {{
                over "orig_col"
                {{
                    bool physics:collisionEnabled = false
                }}
                def Sphere "custom_col" (
                    prepend apiSchemas = ["PhysicsCollisionAPI"]
                )
                {{
                    double radius = {custom_radius}
                }}
            }}
        }}'''
        )
    inner = "\n".join(blocks)
    return f"""#usda 1.0
(
    upAxis = "Z"
)

over "World"
{{
{inner}
}}
"""


# ==============================================================================
# Task A: `usd_reader` マルチアーム・ルート解決・純 OpenUSD mimic テスト
# ==============================================================================


@unittest.skipUnless(HAS_NEWTON, f"newton / warp / pxr が使えない: {_IMPORT_ERR}")
class TestUsdReaderMultiArmAndRootResolution(unittest.TestCase):
    def test_articulation_root_under_geometry_with_sibling_physics(self) -> None:
        """`ArticulationRootAPI` prim (`/robot/Geometry/base_body`) 渡しでも `/robot` 渡しと完全一致すること。"""
        stage = _build_synthetic_multi_arm_stage()
        model_root, info_root = read_articulation(stage, "/robot")
        model_art, info_art = read_articulation(stage, "/robot/Geometry/base_body")

        self.assertEqual(model_art.root_link, model_root.root_link)
        self.assertEqual(model_art.link_names, model_root.link_names)
        self.assertEqual(model_art.dof_names, model_root.dof_names)
        self.assertEqual(model_art.joint_names, model_root.joint_names)
        self.assertEqual(len(model_root.links), 12)
        self.assertEqual(len(model_root.dof_names), 7)  # turret + 2 * (j1 + finger_l + finger_r)
        self.assertFalse(any("たどれない" in w for w in info_root.warnings), info_root.warnings)
        self.assertFalse(any("たどれない" in w for w in info_art.warnings), info_art.warnings)

        # 純 OpenUSD でも両アームの `NewtonMimicAPI` が読まれていること
        mimics = [j for j in model_root.joints if j.mimic is not None]
        self.assertEqual(len(mimics), 2)
        for j in mimics:
            self.assertEqual(j.mimic.multiplier, -1.0)
            self.assertEqual(j.mimic.offset, 0.0)

        # 全 (base, tip) ペアで `KinematicChain.forward_kinematics` が `model.forward_kinematics` と一致すること
        rng = np.random.default_rng(42)
        q_raw = rng.uniform(-0.2, 0.2, size=model_root.num_dofs)
        q = model_root.resolve_mimic(q_raw)
        world = model_root.forward_kinematics(q, resolve_mimic=False)
        for base_name in model_root.link_names:
            for tip_name in model_root.link_names:
                if base_name == tip_name:
                    continue
                chain = model_root.get_kinematic_chain(base_name, tip_name)
                rel_chain = chain.forward_kinematics(q[chain.dof_indices])
                rel_ref = np.linalg.inv(world[model_root.get_link_index(base_name)]) @ world[model_root.get_link_index(tip_name)]
                np.testing.assert_allclose(rel_chain, rel_ref, atol=1e-12)

    def test_traversal_order_geometry_world_fixed_before_physics(self) -> None:
        """`Geometry/` 配下の補助 FixedJoint が `Physics/root_joint` より先に走査されても `ArticulationRootAPI` を根に選ぶこと。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/robot")
        UsdGeom.Xform.Define(stage, "/robot/Geometry")
        UsdGeom.Xform.Define(stage, "/robot/Physics")
        _def_body(stage, "/robot/Geometry/base_body", (0.0, 0.0, 0.0), art_root=True)
        _def_body(stage, "/robot/Geometry/turret", (0.0, 0.0, 0.5))
        UsdGeom.Xform.Define(stage, "/robot/Geometry/turret/non_body_anchor")
        # 辞書順で `Physics/root_joint` より先に現れる非剛体への FixedJoint
        _def_joint(
            stage,
            "/robot/Geometry/turret/sub_fixed_joint",
            "fixed",
            "/robot/Geometry/turret/non_body_anchor",
            "/robot/Geometry/turret",
        )
        _def_joint(stage, "/robot/Physics/root_joint", "fixed", None, "/robot/Geometry/base_body")
        _def_joint(
            stage,
            "/robot/Physics/turret_joint",
            "revolute",
            "/robot/Geometry/base_body",
            "/robot/Geometry/turret",
            pos0=(0.0, 0.0, 0.5),
        )

        model, info = read_articulation(stage, "/robot")
        self.assertEqual(model.root_link, "base_body")
        self.assertEqual(model.dof_names, ["turret_joint"])
        self.assertFalse(any("たどれない" in w for w in info.warnings), info.warnings)
        tj = next(j for j in model.joints if j.name == "turret_joint")
        self.assertEqual(tj.parent_link, "base_body")
        self.assertEqual(tj.child_link, "turret")

    def test_multi_component_disconnected_arms_under_one_root(self) -> None:
        """1 ルート配下に非連結の複数アームが世界固定されている場合、仮想 `world` ルートで無警告統合されること。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/Cell")
        # 非剛体の静的テーブル (複数アームがここへ固定されている)
        UsdGeom.Xform.Define(stage, "/World/Cell/StaticTable")

        offsets = {"ArmA": -1.0, "ArmB": 0.0, "ArmC": 1.0}
        for name, x_off in offsets.items():
            arm = f"/World/Cell/{name}"
            UsdGeom.Xform.Define(stage, arm)
            _def_body(stage, f"{arm}/base", (x_off, 0.0, 0.2), art_root=True)
            _def_body(stage, f"{arm}/link1", (x_off, 0.0, 0.7))
            _def_body(stage, f"{arm}/tip", (x_off + 0.5, 0.0, 0.7))
            _def_joint(stage, f"{arm}/mount", "fixed", "/World/Cell/StaticTable", f"{arm}/base", pos0=(x_off, 0.0, 0.2))
            _def_joint(stage, f"{arm}/j1", "revolute", f"{arm}/base", f"{arm}/link1", pos0=(0.0, 0.0, 0.5), axis="Z")
            _def_joint(stage, f"{arm}/j2", "revolute", f"{arm}/link1", f"{arm}/tip", pos0=(0.5, 0.0, 0.0), axis="Y")

        model, info = read_articulation(stage, "/World/Cell")
        self.assertFalse(any("たどれない" in w for w in info.warnings), info.warnings)
        self.assertEqual(model.root_link, "world")
        self.assertEqual(model.num_dofs, 6)
        self.assertEqual(model.num_links, 1 + 3 * 3)

        robot = GeometricRobot.from_articulation(stage, "/World/Cell", write_target="usd", newton_device="cpu")
        try:
            q = np.array([0.3, -0.2, -0.4, 0.25, 0.5, -0.15])
            robot.set_dof_positions(q)
            fk_np = robot.get_link_world_matrices()
            fk_newton = robot.scene.eval_fk(robot, q)
            np.testing.assert_allclose(fk_newton[:, :3, 3], fk_np[:, :3, 3], atol=1e-5)

            # アーム内およびアーム横断の `KinematicChain` が両方機能すること
            path_to_name = {v: k for k, v in info.link_prim_paths.items()}
            base_a = path_to_name["/World/Cell/ArmA/base"]
            tip_a = path_to_name["/World/Cell/ArmA/tip"]
            tip_c = path_to_name["/World/Cell/ArmC/tip"]
            for b, t in ((base_a, tip_a), (tip_a, tip_c)):
                chain = model.get_kinematic_chain(b, t)
                rel = chain.forward_kinematics(q[chain.dof_indices])
                ref = np.linalg.inv(fk_np[model.get_link_index(b)]) @ fk_np[model.get_link_index(t)]
                np.testing.assert_allclose(rel, ref, atol=1e-12)
        finally:
            robot.close()

    def test_articulation_root_widening_does_not_drag_in_sibling_robot(self) -> None:
        """`/World/RobotA/Geometry/base_body` を渡したとき、兄弟の `/World/RobotB` を巻き込まないこと。"""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        for rname, x_off in (("RobotA", 0.0), ("RobotB", 2.0)):
            r = f"/World/{rname}"
            UsdGeom.Xform.Define(stage, r)
            UsdGeom.Xform.Define(stage, f"{r}/Geometry")
            UsdGeom.Xform.Define(stage, f"{r}/Physics")
            _def_body(stage, f"{r}/Geometry/base_body", (x_off, 0.0, 0.0), art_root=True)
            _def_body(stage, f"{r}/Geometry/link1", (x_off, 0.0, 0.5))
            _def_joint(stage, f"{r}/Physics/root_joint", "fixed", None, f"{r}/Geometry/base_body", pos0=(x_off, 0.0, 0.0))
            _def_joint(
                stage,
                f"{r}/Physics/j1",
                "revolute",
                f"{r}/Geometry/base_body",
                f"{r}/Geometry/link1",
                pos0=(0.0, 0.0, 0.5),
            )

        model, info = read_articulation(stage, "/World/RobotA/Geometry/base_body")
        self.assertFalse(any("たどれない" in w for w in info.warnings), info.warnings)
        self.assertEqual(model.num_links, 2)
        self.assertEqual(model.num_dofs, 1)
        self.assertTrue(all(p.startswith("/World/RobotA/") for p in info.link_prim_paths.values()))

    def test_real_multi_arm_usda_if_present(self) -> None:
        """ワークスペースに `assembly_pipeline.v2` の `multi_arm/robot.usda` があれば実ファイルで回帰検証する。"""
        ext_root = Path(__file__).resolve().parents[1]
        usda = ext_root.parent / "myext.robot.assembly_pipeline.v2" / "examples" / "out" / "multi_arm" / "robot.usda"
        if not usda.exists():
            self.skipTest(f"実ファイルが存在しない: {usda}")
        stage = Usd.Stage.Open(str(usda))
        m_root, i_root = read_articulation(stage, "/robot")
        m_art, i_art = read_articulation(stage, "/robot/Geometry/base_body")
        self.assertEqual(m_art.link_names, m_root.link_names)
        self.assertEqual(m_art.dof_names, m_root.dof_names)
        self.assertEqual(m_art.root_link, m_root.root_link)
        self.assertFalse(any("たどれない" in w for w in i_root.warnings), i_root.warnings)
        self.assertFalse(any("たどれない" in w for w in i_art.warnings), i_art.warnings)
        self.assertEqual(sum(1 for j in m_root.joints if j.mimic is not None), 2)


# ==============================================================================
# Task B: カスタムコライダー USD Sublayer ON/OFF テスト
# ==============================================================================


@unittest.skipUnless(HAS_NEWTON, f"newton / warp / pxr が使えない: {_IMPORT_ERR}")
class TestCustomColliderSublayerToggle(unittest.TestCase):
    def test_attach_mute_unmute_detach_file_sublayer_zero_frame_lag(self) -> None:
        """カスタムコライダー `.usda` の attach / Mute / Unmute / detach が 0 フレーム遅延で Newton に反映されること。"""
        stage = _build_collider_toggle_stage()
        orig_root_usda = stage.GetRootLayer().ExportToString()
        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="usd", newton_device="cpu")
        try:
            # 初期状態: 元コライダー `orig_col` (r=0.35) が障害物へ 0.10 m めり込んでいる
            self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/orig_col"])
            pairs_orig = robot.check_collisions()
            self.assertEqual(len(pairs_orig), 1)
            self.assertAlmostEqual(pairs_orig[0].separation, -0.10, delta=1e-2)

            with tempfile.TemporaryDirectory() as tmpdir:
                custom_path = os.path.join(tmpdir, "custom_colliders.usda")
                with open(custom_path, "w", encoding="utf-8") as fh:
                    fh.write(_custom_collider_usda_text(["/World/Robot"], custom_radius=0.10))

                # 1. Sublayer 取り付け (ON) -> `GeometricLayer` が最強位置 (index 0)、カスタム Sublayer は index 1
                ident = robot.attach_collider_layer(custom_path)
                self.assertEqual(robot.collider_layers, [ident])
                session_subs = list(stage.GetSessionLayer().subLayerPaths)
                self.assertEqual(session_subs[0], robot.layer.sdf_layer.identifier)
                self.assertEqual(session_subs[1], ident)
                self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/custom_col"])

                # 元 Root Layer は一切汚染されていないこと
                self.assertEqual(stage.GetRootLayer().ExportToString(), orig_root_usda)

                # 0 フレーム遅延で Newton 衝突判定が非干渉 (クリアランス +0.15 m) へ切り替わること
                self.assertEqual(robot.check_collisions(), [])
                near = robot.compute_clearances(max_distance=0.30)
                self.assertEqual(len(near), 1)
                self.assertAlmostEqual(near[0].separation, 0.15, delta=1e-2)

                # 2. Mute (OFF) -> 即座に元の `orig_col` へ戻り干渉を検出すること
                robot.set_collider_layer_enabled(custom_path, False)
                self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/orig_col"])
                pairs_muted = robot.check_collisions()
                self.assertEqual(len(pairs_muted), 1)
                self.assertAlmostEqual(pairs_muted[0].separation, -0.10, delta=1e-2)

                # 3. Unmute (ON) -> 再び非干渉へ切り替わること
                robot.set_collider_layer_enabled(ident, True)
                self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/custom_col"])
                self.assertEqual(robot.check_collisions(), [])

                # 4. Detach -> Sublayer が外れ、元の `orig_col` に戻ること
                self.assertTrue(robot.detach_collider_layer(ident))
                self.assertEqual(robot.collider_layers, [])
                self.assertNotIn(ident, list(stage.GetSessionLayer().subLayerPaths))
                self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/orig_col"])
                self.assertTrue(robot.is_in_collision())
        finally:
            robot.close()

    def test_manual_usd_sublayer_toggle_auto_invalidates_without_manual_reload(self) -> None:
        """`reload_colliders()` を呼ばずに USD API で直接 Sublayer 挿抜・Mute/Unmute しても自動追従すること。"""
        stage = _build_collider_toggle_stage()
        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="usd", newton_device="cpu")
        try:
            self.assertTrue(robot.is_in_collision())

            custom_layer = Sdf.Layer.CreateAnonymous("manual_custom_colliders.usda")
            custom_layer.ImportFromString(_custom_collider_usda_text(["/World/Robot"], custom_radius=0.10))
            ident = custom_layer.identifier

            # 手動 insert (reload_colliders 未呼び出し)
            stage.GetSessionLayer().subLayerPaths.insert(1, ident)
            self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/custom_col"])
            self.assertFalse(robot.is_in_collision())

            # 手動 Mute
            stage.MuteLayer(ident)
            self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/orig_col"])
            self.assertTrue(robot.is_in_collision())

            # 手動 Unmute
            stage.UnmuteLayer(ident)
            self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/custom_col"])
            self.assertFalse(robot.is_in_collision())

            # 手動 remove
            stage.GetSessionLayer().subLayerPaths.remove(ident)
            self.assertEqual(robot.collider_paths["tip_link"], ["/World/Robot/tip_link/orig_col"])
            self.assertTrue(robot.is_in_collision())
        finally:
            robot.close()

    def test_direct_collider_attr_edit_auto_invalidates_and_pose_write_does_not(self) -> None:
        """コライダー属性の直接編集は自動再構築を発火させ、通常の姿勢書き込み (`write_target="usd"`) は発火させないこと。"""
        stage = _build_collider_toggle_stage()
        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="usd", newton_device="cpu")
        try:
            self.assertTrue(robot.is_in_collision())
            cver_before = robot.collider_version

            # 通常姿勢更新で `collider_version` や `scene._dirty` が増えないこと (性能回帰ガード)
            robot.set_dof_positions([0.05])
            self.assertEqual(robot.collider_version, cver_before)
            self.assertFalse(robot.scene._dirty)

            # `radius` を 0.35 -> 0.10 へ直接編集 -> 自動検知して非干渉になること
            col_prim = stage.GetPrimAtPath("/World/Robot/tip_link/orig_col")
            UsdGeom.Sphere(col_prim).GetRadiusAttr().Set(0.10)
            self.assertGreater(robot.collider_version, cver_before)
            self.assertFalse(robot.is_in_collision())

            # `radius` を 0.35 へ戻し、`physics:collisionEnabled = false` にすると干渉が消えること
            UsdGeom.Sphere(col_prim).GetRadiusAttr().Set(0.35)
            self.assertTrue(robot.is_in_collision())
            UsdPhysics.CollisionAPI(col_prim).CreateCollisionEnabledAttr().Set(False)
            self.assertEqual(robot.collider_paths["tip_link"], [])
            self.assertFalse(robot.is_in_collision())
        finally:
            robot.close()

    def test_scene_invalidate_and_robot_view_collider_sublayer_toggle(self) -> None:
        """`GeometricScene.invalidate()` (`_ModelProxy`) と `GeometricRobotView` の両方で Sublayer 切替が効くこと。"""
        # 1. `GeometricScene` + `_ModelProxy`
        stage1 = _build_collider_toggle_stage()
        model, info = read_articulation(stage1, "/World/Robot")
        scene = GeometricScene(stage1, device="cpu")
        try:
            proxy = scene.add_model(model, info.link_prim_paths)
            self.assertEqual(len(scene.check_collisions(proxy)), 1)

            custom1 = Sdf.Layer.CreateAnonymous("proxy_custom.usda")
            custom1.ImportFromString(_custom_collider_usda_text(["/World/Robot"], custom_radius=0.10))
            stage1.GetSessionLayer().subLayerPaths.insert(0, custom1.identifier)
            self.assertEqual(scene.check_collisions(proxy), [])

            stage1.MuteLayer(custom1.identifier)
            scene.invalidate()
            self.assertEqual(len(scene.check_collisions(proxy)), 1)
        finally:
            scene.close()

        # 2. `GeometricRobotView` (2 台バッチ)
        stage2 = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage2, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage2, "/World")
        _add_two_link_robot(stage2, "/World/Robot0", y_offset=0.0)
        _add_two_link_robot(stage2, "/World/Robot1", y_offset=2.0)
        UsdGeom.Xform.Define(stage2, "/World/Obstacles")
        for idx, y_off in ((0, 0.0), (1, 2.0)):
            box = UsdGeom.Cube.Define(stage2, f"/World/Obstacles/box{idx}")
            box.CreateSizeAttr(0.20)
            box.AddTranslateOp().Set(Gf.Vec3d(1.35, y_off, 0.0))
            UsdPhysics.CollisionAPI.Apply(box.GetPrim())

        view = GeometricRobotView.from_articulations(
            stage2,
            ["/World/Robot0", "/World/Robot1"],
            write_target="usd",
            newton_device="cpu",
            use_cuda_graph=False,
        )
        try:
            batch_orig = view.check_collisions_batch()
            self.assertEqual([len(p) for p in batch_orig], [1, 1])

            custom2 = Sdf.Layer.CreateAnonymous("view_custom.usda")
            custom2.ImportFromString(_custom_collider_usda_text(["/World/Robot0", "/World/Robot1"], custom_radius=0.10))
            # Sublayer 取り付け -> 自動検知でバッチ判定も非干渉になること
            stage2.GetSessionLayer().subLayerPaths.insert(1, custom2.identifier)
            batch_custom = view.check_collisions_batch()
            self.assertEqual([len(p) for p in batch_custom], [0, 0])

            # Mute + 明示 `view.reload_colliders()` -> 再び 2 台とも干渉検出すること
            stage2.MuteLayer(custom2.identifier)
            view.reload_colliders()
            batch_muted = view.check_collisions_batch()
            self.assertEqual([len(p) for p in batch_muted], [1, 1])
        finally:
            view.close()

    def test_close_detaches_collider_layers(self) -> None:
        """`robot.close(remove_layer=True)` が取り付け済みカスタムコライダー Sublayer も外すこと。"""
        stage = _build_collider_toggle_stage()
        robot = GeometricRobot.from_articulation(stage, "/World/Robot", write_target="usd", newton_device="cpu")
        custom_layer = Sdf.Layer.CreateAnonymous("close_cleanup.usda")
        custom_layer.ImportFromString(_custom_collider_usda_text(["/World/Robot"], custom_radius=0.10))
        ident = robot.attach_collider_layer(custom_layer)
        self.assertIn(ident, list(stage.GetSessionLayer().subLayerPaths))

        robot.close(remove_layer=True)
        self.assertNotIn(ident, list(stage.GetSessionLayer().subLayerPaths))
        self.assertEqual(robot.collider_layers, [])


# ==============================================================================
# Item #5: `ignore_touching_pairs(samples=...)` 多姿勢サンプリング検証
# ==============================================================================


@unittest.skipUnless(HAS_NEWTON, f"newton / warp / pxr が使えない: {_IMPORT_ERR}")
class TestIgnoreTouchingPairsSampling(unittest.TestCase):
    @staticmethod
    def _build_always_vs_pose_dependent_stage() -> "Usd.Stage":
        """常時接触ペア `(link0, link2)` と姿勢依存ペア `(link0, link3)` が混在する 4 リンクロボットを作る。

        - `link0` (原点, r=0.25) と `link2` (短いスペーサ `link1` 越しに最大距離 0.10 m, r=0.25): 全姿勢で常に接触。
        - `link3` (`j3` から +0.40 m 先に球 r=0.15): `q3 = pi` では折り返して `link0` と接触するが、
          `q3 = 0` では `x ≈ 0.90 m` に離れて非接触。
        """
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/Arm")

        l0 = _def_body(stage, "/World/Arm/link0", (0.0, 0.0, 0.0), art_root=True)
        s0 = UsdGeom.Sphere.Define(stage, "/World/Arm/link0/col")
        s0.CreateRadiusAttr(0.25)
        UsdPhysics.CollisionAPI.Apply(s0.GetPrim())

        l1 = _def_body(stage, "/World/Arm/link1", (0.05, 0.0, 0.0))

        l2 = _def_body(stage, "/World/Arm/link2", (0.10, 0.0, 0.0))
        s2 = UsdGeom.Sphere.Define(stage, "/World/Arm/link2/col")
        s2.CreateRadiusAttr(0.25)
        UsdPhysics.CollisionAPI.Apply(s2.GetPrim())

        l3 = _def_body(stage, "/World/Arm/link3", (0.50, 0.0, 0.0))
        s3 = UsdGeom.Sphere.Define(stage, "/World/Arm/link3/col")
        s3.CreateRadiusAttr(0.15)
        s3.AddTranslateOp().Set(Gf.Vec3d(0.40, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(s3.GetPrim())

        _def_joint(stage, "/World/Arm/j1", "revolute", str(l0.GetPath()), str(l1.GetPath()), pos0=(0.05, 0.0, 0.0), axis="Z", limits=(-180.0, 180.0))
        _def_joint(stage, "/World/Arm/j2", "revolute", str(l1.GetPath()), str(l2.GetPath()), pos0=(0.05, 0.0, 0.0), axis="Z", limits=(-180.0, 180.0))
        _def_joint(stage, "/World/Arm/j3", "revolute", str(l2.GetPath()), str(l3.GetPath()), pos0=(0.40, 0.0, 0.0), axis="Z", limits=(-180.0, 180.0))
        return stage

    def test_samples_excludes_only_always_touching_and_restores_pose(self) -> None:
        """`samples=16` は常時接触ペアのみ除外して姿勢依存ペアを残し、元の姿勢を復元すること。"""
        stage = self._build_always_vs_pose_dependent_stage()
        robot = GeometricRobot.from_articulation(stage, "/World/Arm", write_target="usd", newton_device="cpu")
        try:
            # 意図的に姿勢依存の自己干渉 (`link0` vs `link3`) が起きている折り返し姿勢へ移動
            folded_q = np.array([0.0, 0.0, np.pi])
            robot.set_dof_positions(folded_q)

            # 1. 多姿勢サンプリング (`samples=16`): 常時接触の `(link0, link2)` だけを除外
            added_multi = robot.ignore_touching_pairs(samples=16, seed=42)
            self.assertEqual(added_multi, [("link0", "link2")])
            self.assertEqual(robot.collision_filter.reason_for("link0", "link2"), "always_touching")
            self.assertIsNone(robot.collision_filter.reason_for("link0", "link3"))
            # 元の姿勢が復元されていること
            np.testing.assert_allclose(robot.get_dof_positions(), folded_q, atol=1e-12)
            # 折り返し姿勢では姿勢依存の `(link0, link3)` が引き続き検出されること
            within_folded = [(p.link_a, p.link_b) for p in robot.check_collisions("within")]
            self.assertIn(("link0", "link3"), within_folded)
            self.assertNotIn(("link0", "link2"), within_folded)

            # 伸ばした姿勢では干渉 0 になること
            robot.set_dof_positions([0.0, 0.0, 0.0])
            self.assertEqual(robot.check_collisions("within"), [])

            # 2. 既定 `samples=0` の後方互換: 折り返し姿勢の 1 姿勢だけを見て両方除外 (`reason="touching_at_setup"`)
            robot.collision_filter.remove_by_reason("always_touching")
            robot.set_dof_positions(folded_q)
            added_single = robot.ignore_touching_pairs()
            self.assertIn(("link0", "link2"), added_single)
            self.assertIn(("link0", "link3"), added_single)
            self.assertEqual(robot.collision_filter.reason_for("link0", "link2"), "touching_at_setup")
            self.assertEqual(robot.collision_filter.reason_for("link0", "link3"), "touching_at_setup")
            self.assertEqual(robot.check_collisions("within"), [])
        finally:
            robot.close()


if __name__ == "__main__":
    unittest.main()
