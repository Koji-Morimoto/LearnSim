"""USD / PhysX に触る部分のテスト。GUI とは別プロセスの headless Isaac Sim で走る。

実行:  C:\\isaacsim-6.2.0\\python.bat tests_kit\\run_kit_tests.py --no-ros-env          (run_tests.ps1 から呼ばれる)
       --only=<名前の一部> で、そのテストだけ走らせる
結果:  tests_kit/last_run.json、終了コード(失敗があれば 1)

答え合わせの相手:
    - Play した Articulation(PhysX)のリンク名・DOF 名・リンクの姿勢・mimic で決まった関節値
    - 配置から計算できる干渉の有無
"""

from __future__ import annotations

import os
import tempfile

import harness

app = harness.start()

import numpy as np
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from myext.robot.geometric_newton import (
    GeometricCollisionChecker,
    GeometricContactMonitor,
    GeometricLayer,
    GeometricLayerOptions,
    GeometricRobot,
    GeometricRobotView,
    GeometricScene,
    GeometricVisualStyle,
    IKSolverConfig,
    MDHParameters,
    MDHRow,
    ScrewAxisRow,
    ScrewParameters,
    UrdfSource,
    UsdArticulationSource,
    read_articulation,
)
from myext.robot.geometric_newton import transforms as tf
from myext.robot.geometric_newton.usd_reader import _np, split_scale
from myext.robot.geometric_newton.writer import GeometricFabricWriter

ROBOT = "/World/Robot"
rng = np.random.default_rng(0)


# ====================================================================== 部品


def add_ur10e(stage, path=ROBOT, gripper=None, position=(0.0, 0.0, 0.0)):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(harness.UR10E_USD)
    if gripper:
        prim.GetVariantSet("Gripper").SetVariantSelection(gripper)
    UsdGeom.Xformable(prim).ClearXformOpOrder()
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
    harness.update(3)
    return prim


def add_box(stage, path, position, size=0.2, kind="static", scale=None):
    """箱のコライダー。kind: static(剛体なし) / dynamic / kinematic。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    if scale is not None:
        cube.AddScaleOp().Set(Gf.Vec3d(*scale))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kind != "static":
        body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        if kind == "kinematic":
            body.CreateKinematicEnabledAttr(True)
    return cube.GetPrim()


def usd_world(stage, path) -> np.ndarray:
    """USD から読んだワールド姿勢(スケール抜き、列ベクトル形式の 4x4)。"""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    return split_scale(_np(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))))[0]


def assert_poses(got: np.ndarray, expect: np.ndarray, atol=2e-4, label=""):
    np.testing.assert_allclose(got[..., :3, 3], expect[..., :3, 3], atol=atol, err_msg=f"位置 {label}")
    np.testing.assert_allclose(got[..., :3, :3], expect[..., :3, :3], atol=atol * 5, err_msg=f"回転 {label}")


def physx_truth(root_path: str, configs: list[np.ndarray] | None, settle: int = 3, targets: dict[str, float] | None = None) -> dict:
    """Play して、Articulation(PhysX)から名前と、各関節値でのリンクのワールド姿勢を読む。読んだら Stop する。

    configs の各関節値を直接書き込み、同じ値を位置ドライブの目標にも書いて settle 回進めてから、
    「実際の関節値」と「そのときのリンクの姿勢」を一緒に読む(ドライブや mimic で動いた分も含めて対応が取れる)。
    targets を渡すと、関節値は書かず、その関節の目標値だけ書いて settle 回進める(mimic の従動を見る用)。
    """
    from isaacsim.core.experimental.prims import Articulation, RigidPrim

    harness.play(3)
    art = Articulation(root_path)
    link_paths = art.link_paths[0]
    links = RigidPrim(link_paths)
    out = {"dof_names": list(art.dof_names), "link_names": list(art.link_names), "joint_names": list(art.joint_names), "link_paths": list(link_paths), "samples": []}
    n = art.num_dofs

    def read() -> dict:
        q = art.get_dof_positions().numpy()[0].astype(float)
        pos, quat = links.get_world_poses()
        mats = tf.make_transform(tf.quat_to_matrix(quat.numpy().astype(float)), pos.numpy().astype(float))
        return {"q": q, "poses": mats}

    if targets is not None:
        tgt = np.zeros((1, n), dtype=np.float32)
        for name, value in targets.items():
            tgt[0, out["dof_names"].index(name)] = value
        art.set_dof_position_targets(tgt)
        harness.update(settle)
        out["samples"].append(read())
    for q in configs or []:
        arr = np.asarray(q, dtype=np.float32).reshape(1, n)
        art.set_dof_positions(arr)
        art.set_dof_velocities(np.zeros((1, n), dtype=np.float32))
        art.set_dof_position_targets(arr)
        harness.update(settle)
        out["samples"].append(read())
    harness.stop(2)
    return out


def check_against_physx(stage, source, configs, label, settle=3) -> tuple:
    """モデルの名前・並びと順運動学を PhysX と突き合わせる。(model, info, truth) を返す。"""
    model, info = read_articulation(stage, source)
    truth = physx_truth(info.root_prim_paths[0], configs(model) if callable(configs) else configs, settle=settle)
    assert model.link_names == truth["link_names"], f"{label}: リンク名 {model.link_names} != PhysX {truth['link_names']}"
    assert model.dof_names == truth["dof_names"], f"{label}: DOF 名 {model.dof_names} != PhysX {truth['dof_names']}"
    assert [info.link_prim_paths[n] for n in model.link_names] == truth["link_paths"], f"{label}: リンクの prim パスの並び"
    worst = 0.0
    for sample in truth["samples"]:
        base = sample["poses"][0]
        expect = sample["poses"]
        got = base @ model.forward_kinematics(sample["q"], resolve_mimic=False)
        worst = max(worst, float(np.abs(got[:, :3, 3] - expect[:, :3, 3]).max()))
        assert_poses(got, expect, label=f"{label} q={np.round(sample['q'], 3).tolist()}")
    harness.note(f"fk_vs_physx_max_position_error[{label}]", worst)
    return model, info, truth


def random_configs(model, n=6, scale=1.0) -> list[np.ndarray]:
    lo = np.maximum(model.dof_lower_limits, -scale)
    hi = np.minimum(model.dof_upper_limits, scale)
    return [model.resolve_mimic(rng.uniform(lo, hi)) for _ in range(n)]


def build_articulation(stage, links: list[dict], joints: list[dict], root=ROBOT):
    """手組みのアーティキュレーション。links: name, pos, [scale], [parent]。joints: name, type, body0, body1, [axis, p0, r0, p1, r1, lower, upper]。

    Joint の prim は joints の並び順に <root>/joints/<name> に作る。body0 / body1 が None ならワールド。
    """
    UsdGeom.Xform.Define(stage, root)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root))
    paths = {}
    for link in links:
        parent = paths[link["parent"]] if link.get("parent") else root
        path = f"{parent}/{link['name']}"
        paths[link["name"]] = path
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(*link.get("pos", (0, 0, 0))))
        q = link.get("quat", (1, 0, 0, 0))
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*q))
        xform.AddScaleOp().Set(Gf.Vec3d(*link.get("scale", (1, 1, 1))))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(1.0)
        cube = UsdGeom.Cube.Define(stage, path + "/collider")
        cube.CreateSizeAttr(link.get("size", 0.1))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Scope.Define(stage, root + "/joints")
    for joint in joints:
        path = f"{root}/joints/{joint['name']}"
        cls = {"revolute": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint, "fixed": UsdPhysics.FixedJoint}[joint["type"]]
        j = cls.Define(stage, path)
        if joint.get("body0"):
            j.CreateBody0Rel().SetTargets([paths[joint["body0"]]])
        if joint.get("body1"):
            j.CreateBody1Rel().SetTargets([paths[joint["body1"]]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*joint.get("p0", (0, 0, 0))))
        j.CreateLocalRot0Attr(Gf.Quatf(*joint.get("r0", (1, 0, 0, 0))))
        j.CreateLocalPos1Attr(Gf.Vec3f(*joint.get("p1", (0, 0, 0))))
        j.CreateLocalRot1Attr(Gf.Quatf(*joint.get("r1", (1, 0, 0, 0))))
        if joint["type"] != "fixed":
            j.CreateAxisAttr(joint.get("axis", "Z"))
            j.CreateLowerLimitAttr(joint.get("lower", -90.0 if joint["type"] == "revolute" else -0.3))
            j.CreateUpperLimitAttr(joint.get("upper", 90.0 if joint["type"] == "revolute" else 0.3))
    harness.update(3)
    return paths


def snapshot(stage) -> dict:
    """合成後のステージの見え方(prim ごとの型・API・属性値・関係)。Sublayer を外した後の比較用。"""
    out = {}
    for prim in stage.TraverseAll():
        path = str(prim.GetPath())
        if path.startswith(("/OmniverseKit", "/Render")):
            continue
        d = {"@type": prim.GetTypeName(), "@api": list(prim.GetAppliedSchemas()), "@active": prim.IsActive()}
        for a in prim.GetAuthoredAttributes():
            d[a.GetName()] = repr(a.Get())
        for r in prim.GetAuthoredRelationships():
            d["rel:" + r.GetName()] = [str(t) for t in r.GetTargets()]
        out[path] = d
    return out


def six_axis() -> MDHParameters:
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


# ====================================================================== 既存ロボットの読み取り(PhysX と突き合わせ)


@harness.test
def test_ur10e_matches_physx():
    stage = harness.new_stage()
    add_ur10e(stage)
    model, info, _ = check_against_physx(stage, ROBOT, lambda m: random_configs(m, 8, scale=2.0), "ur10e")
    assert model.num_dofs == 6 and model.root_link == "base_link"
    assert info.articulation_root_paths == [f"{ROBOT}/root_joint"], info.articulation_root_paths
    assert info.self_collision_enabled is False
    np.testing.assert_allclose(model.dof_lower_limits[2], -np.pi)  # elbow は ±180°


@harness.test
def test_ur10e_with_gripper_matches_physx_including_mimic():
    stage = harness.new_stage()
    add_ur10e(stage, gripper="Robotiq_2F_85")
    source = UsdArticulationSource(ROBOT, components={"arm": ROBOT, "gripper": f"{ROBOT}/Robotiq_2F_85"})
    model, info, _ = check_against_physx(stage, source, lambda m: random_configs(m, 5, scale=0.6), "ur10e+gripper", settle=4)
    assert "base_link_0" in model.link_names  # 同名リンクの 2 個目
    assert set(model.components) == {"arm", "gripper"}
    assert "wrist_3_link" in model.components["arm"] and "left_inner_finger" in model.components["gripper"]
    followers = {j.name: j.mimic for j in model.dof_joints if j.mimic is not None}
    assert len(followers) == 5, followers
    assert all(m.reference_joint == "finger_joint" for m in followers.values())
    # PhysX に指の目標値だけを与えて、mimic で決まる従動軸の値を読む。読み取った倍率・offset と合うか
    truth = physx_truth(ROBOT, None, settle=240, targets={"finger_joint": 0.5})
    q = truth["samples"][0]["q"]
    names = truth["dof_names"]
    leader = q[names.index("finger_joint")]
    assert abs(leader - 0.5) < 0.05, f"指が目標に届いていない: {leader}"
    measured = {}
    for name, mimic in followers.items():
        value = q[names.index(name)]
        measured[name] = {"physx": round(float(value), 4), "model": round(mimic.multiplier * leader + mimic.offset, 4)}
        assert abs(value - (mimic.multiplier * leader + mimic.offset)) < 0.03, f"{name}: {measured[name]}"
    harness.note("mimic_physx_vs_model(finger_joint=0.5 目標)", measured)


@harness.test
def test_handmade_reversed_joints_scale_and_sibling_order():
    """逆向きの Joint(木の親が body1)、スケール付きリンク、枝分かれ(Joint の並びと剛体の並びが逆)を PhysX と突き合わせる。"""
    stage = harness.new_stage()
    links = [
        dict(name="base", pos=(0, 0, 0.5)),
        dict(name="a", pos=(0.3, 0, 0.5)),
        dict(name="b", pos=(0.6, 0.1, 0.5), scale=(2, 2, 2)),
        dict(name="c", pos=(0, 0.4, 0.5)),
        dict(name="d", pos=(0, 0.7, 0.6)),
    ]
    q45 = (0.9238795, 0, 0.3826834, 0)
    joints = [
        dict(name="world", type="fixed", body0=None, body1="base", p0=(0, 0, 0.5)),
        # 並び: c への関節を a への関節より先に書く(剛体は a が先)
        dict(name="j_c", type="prismatic", body0="base", body1="c", axis="X", p0=(0, 0.4, 0), r0=q45),
        dict(name="j_a", type="revolute", body0="base", body1="a", axis="Y", p0=(0.15, 0, 0), p1=(-0.15, 0, 0)),
        # 逆向き: 親(a)が body1、子(b)が body0。b はスケール 2 なので localPos0 は b の座標で半分の値
        dict(name="j_b", type="revolute", body0="b", body1="a", axis="Z", p0=(-0.075, -0.05, 0), p1=(0.15, 0, 0), r1=q45),
        dict(name="j_d", type="prismatic", body0="d", body1="c", axis="Z", p0=(0, -0.15, -0.05), p1=(0, 0.15, 0.05)),
    ]
    build_articulation(stage, links, joints)
    model, info, truth = check_against_physx(stage, ROBOT, lambda m: random_configs(m, 8, scale=0.25), "handmade")
    assert set(info.reversed_joints) == {"j_b", "j_d"}, info.reversed_joints
    harness.note("handmade_dof_order(PhysX)", truth["dof_names"])


@harness.test
def test_nested_prims_and_world_joint():
    """リンクの prim が入れ子(子リンクが親リンクの prim の下)で、根がワールドに対して動くロボット。"""
    stage = harness.new_stage()
    links = [
        dict(name="cart", pos=(0, 0, 0.3)),
        dict(name="arm", pos=(0, 0, 0.3), parent="cart"),  # cart の子 prim。ローカル (0,0,0.3) = ワールド z 0.6
        dict(name="hand", pos=(0.2, 0, 0), parent="arm"),
    ]
    joints = [
        dict(name="rail", type="prismatic", body0=None, body1="cart", axis="X", p0=(0, 0, 0.3)),
        dict(name="shoulder", type="revolute", body0="cart", body1="arm", axis="Y", p0=(0, 0, 0.3)),
        dict(name="wrist", type="revolute", body0="arm", body1="hand", axis="X", p0=(0.2, 0, 0)),
    ]
    paths = build_articulation(stage, links, joints)
    # 既定(world_joints="ignore"): PhysX の Articulation と同じく、ワールドに対して動く rail は DOF に入らない
    model, info, truth = check_against_physx(stage, ROBOT, lambda m: random_configs(m, 6, scale=0.25), "nested")
    assert model.root_link == "cart" and model.dof_names == ["shoulder", "wrist"], (model.root_link, model.dof_names)
    assert any("rail" in w for w in info.warnings), info.warnings
    harness.note("world_prismatic_joint_dof_names(PhysX)", truth["dof_names"])
    # world_joints="dof": rail も DOF になり、prim を持たないリンク "world" が根になる
    source = UsdArticulationSource(ROBOT, world_joints="dof")
    model, info = read_articulation(stage, source)
    assert model.root_link == "world" and model.dof_names == ["rail", "shoulder", "wrist"], (model.root_link, model.dof_names)
    # 幾何モードで書いた結果も、入れ子の prim で正しいワールド姿勢になる
    robot = GeometricRobot.from_articulation(stage, source, write_target="usd")
    q = np.array([0.2, 0.5, -0.4])
    robot.set_dof_positions(q)
    expect = model.forward_kinematics(q)
    for name in ("cart", "arm", "hand"):
        assert_poses(usd_world(stage, paths[name]), expect[model.get_link_index(name)], atol=1e-9, label=name)
    robot.close()


@harness.test
def test_mimic_offset_sign_and_unit_match_physx():
    """手組みの mimic(gearing と offset の両方が 0 でない)で、読み取った倍率・offset が PhysX の動きと合うか。"""
    stage = harness.new_stage()
    names = ["a", "b", "c", "d", "e", "f"]
    links = [dict(name="base", pos=(0, 0, 0.5))] + [dict(name=n, pos=(0.3 * np.cos(k), 0.3 * np.sin(k), 0.5)) for k, n in enumerate(names)]
    def hinge(name, body, kind="revolute"):
        k = names.index(body)
        p = (0.15 * np.cos(k), 0.15 * np.sin(k), 0.0)
        return dict(name=name, type=kind, body0="base", body1=body, axis="Z", p0=p, p1=tuple(-v for v in p), lower=-170.0 if kind == "revolute" else -0.4, upper=170.0 if kind == "revolute" else 0.4)

    joints = [
        dict(name="world", type="fixed", body0=None, body1="base", p0=(0, 0, 0.5)),
        hinge("lead", "a"),
        hinge("physx_rot", "b"),  # PhysxMimicJointAPI、回転 → 回転
        hinge("newton_rot", "c"),  # NewtonMimicAPI、回転 → 回転
        hinge("lead_lin", "d", "prismatic"),
        hinge("newton_lin", "e", "prismatic"),  # NewtonMimicAPI、直動 → 直動
        hinge("cross", "f", "prismatic"),  # 回転の lead に従う直動(種類が違う)
    ]
    build_articulation(stage, links, joints)
    for name, kind in (("lead", "angular"), ("lead_lin", "linear")):
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(f"{ROBOT}/joints/{name}"), kind)
        drive.CreateStiffnessAttr(1e5)
        drive.CreateDampingAttr(1e3)
    api = PhysxSchema.PhysxMimicJointAPI.Apply(stage.GetPrimAtPath(f"{ROBOT}/joints/physx_rot"), "rotZ")
    api.CreateGearingAttr(-2.0)
    api.CreateOffsetAttr(12.0)
    api.CreateReferenceJointRel().SetTargets([f"{ROBOT}/joints/lead"])
    api.CreateReferenceJointAxisAttr("rotZ")
    for name, leader, coef0, coef1 in (("newton_rot", "lead", 8.0, -1.5), ("newton_lin", "lead_lin", 0.05, 0.5), ("cross", "lead", 0.0, 0.2)):
        prim = stage.GetPrimAtPath(f"{ROBOT}/joints/{name}")
        prim.AddAppliedSchema("NewtonMimicAPI")
        prim.CreateRelationship("newton:mimicJoint").SetTargets([f"{ROBOT}/joints/{leader}"])
        prim.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(coef0)
        prim.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(coef1)
    harness.update(3)
    model, info = read_articulation(stage, ROBOT)
    mimics = {j.name: j.mimic for j in model.dof_joints}
    # 読み取り: 式どおりに倍率と offset(回転は度 → ラジアン)になっている
    assert mimics["physx_rot"].multiplier == 2.0 and abs(mimics["physx_rot"].offset + np.deg2rad(12.0)) < 1e-9
    assert mimics["newton_rot"].multiplier == -1.5 and abs(mimics["newton_rot"].offset - np.deg2rad(8.0)) < 1e-9
    assert mimics["newton_lin"].multiplier == 0.5 and abs(mimics["newton_lin"].offset - 0.05) < 1e-7
    assert mimics["cross"] is None and any("cross" in w and "種類の違う" in w for w in info.warnings), info.warnings
    # PhysX: 参照軸の目標値だけを与えて、従動軸がどこへ行くかを読む
    truth = physx_truth(ROBOT, None, settle=300, targets={"lead": 0.3, "lead_lin": 0.1})
    q = dict(zip(truth["dof_names"], truth["samples"][0]["q"].tolist()))
    assert abs(q["lead"] - 0.3) < 0.02 and abs(q["lead_lin"] - 0.1) < 0.01, q
    out = {"physx": {k: round(v, 4) for k, v in q.items()}}
    for name in ("physx_rot", "newton_rot", "newton_lin"):
        m = mimics[name]
        out[name] = {"model": round(m.multiplier * q[m.reference_joint] + m.offset, 4), "physx": round(q[name], 4)}
    harness.note("mimic_offset_units", out)
    assert abs(out["physx_rot"]["physx"] - out["physx_rot"]["model"]) < 0.01, out
    # NewtonMimicAPI を PhysX が実行時に使うかどうかは構成による。使われた場合だけ値を突き合わせる
    for name in ("newton_rot", "newton_lin"):
        followed = abs(out[name]["physx"]) > 1e-3
        out[name]["physx_followed"] = followed
        if followed:
            assert abs(out[name]["physx"] - out[name]["model"]) < 0.01, out
    harness.note("mimic_newton_schema_followed_by_physx", {n: out[n]["physx_followed"] for n in ("newton_rot", "newton_lin")})


# ====================================================================== Sublayer


@harness.test
def test_layer_apply_remove_restores_everything():
    stage = harness.new_stage(gravity=9.81)
    add_ur10e(stage, gripper="Robotiq_2F_85")
    add_box(stage, "/World/Box", (2, 0, 0.1), kind="dynamic")
    harness.update(2)
    before = snapshot(stage)
    root_text = stage.GetRootLayer().ExportToString()
    session_before = list(stage.GetSessionLayer().subLayerPaths)

    robot = GeometricRobot.from_articulation(stage, ROBOT)
    assert robot.layer.is_applied()
    assert len(stage.GetSessionLayer().subLayerPaths) == len(session_before) + 1
    report = robot.layer.report
    names = {name for _, name, _, _ in report.changes}
    assert {"physxArticulation:articulationEnabled", "physics:jointEnabled", "physics:rigidBodyEnabled"} <= names, names
    link = stage.GetPrimAtPath(f"{ROBOT}/forearm_link")
    assert UsdPhysics.RigidBodyAPI(link).GetRigidBodyEnabledAttr().Get() is False
    assert stage.GetPrimAtPath(f"{ROBOT}/root_joint").GetAttribute("physxArticulation:articulationEnabled").Get() is False
    assert UsdPhysics.Joint(stage.GetPrimAtPath(f"{ROBOT}/joints/elbow_joint")).GetJointEnabledAttr().Get() is False
    # コリジョンは残っている(CollisionAPI にも形状にも触っていない)
    assert not any("collision" in name.lower() for name in names), names
    robot.set_dof_positions(np.full(robot.num_dofs, 0.3))
    robot.flush_to_usd()
    # 幾何モード中に Play しても、ロボットは落ちない・動かない(箱は落ちる)
    z0 = usd_world(stage, f"{ROBOT}/wrist_3_link")[:3, 3].copy()
    harness.play(40)
    np.testing.assert_allclose(usd_world(stage, f"{ROBOT}/wrist_3_link")[:3, 3], z0, atol=1e-9)
    harness.stop(2)
    assert stage.GetRootLayer().ExportToString() == root_text, "幾何モード中に Root Layer が変わった"

    assert robot.layer.remove()
    harness.update(2)
    after = snapshot(stage)
    diff = [p for p in set(before) | set(after) if before.get(p) != after.get(p)]
    assert not diff, f"外した後に差分がある: {diff[:5]}"
    assert stage.GetRootLayer().ExportToString() == root_text
    assert list(stage.GetSessionLayer().subLayerPaths) == session_before
    # 元の Articulation として動く
    truth = physx_truth(ROBOT, [np.zeros(robot.num_dofs)])
    assert truth["dof_names"] == robot.dof_names
    # もう一度付けられる
    robot.layer.apply()
    robot.set_dof_positions(np.full(robot.num_dofs, 0.1))
    robot.close()
    assert list(stage.GetSessionLayer().subLayerPaths) == session_before


@harness.test
def test_layer_target_root_uses_a_file():
    stage = harness.new_stage()
    add_ur10e(stage)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "geometric.usda").replace("\\", "/")
        try:
            GeometricLayerOptions(layer_target="root")
            raise AssertionError("root_layer_file なしで作れてしまった")
        except ValueError:
            pass
        robot = GeometricRobot.from_articulation(stage, ROBOT, layer_options=GeometricLayerOptions(layer_target="root", root_layer_file=path))
        assert path in list(stage.GetRootLayer().subLayerPaths)
        assert len(stage.GetSessionLayer().subLayerPaths) == 0
        robot.layer.save()
        assert os.path.exists(path)
        robot.close()
        assert path not in list(stage.GetRootLayer().subLayerPaths)


# ====================================================================== 書き込み


@harness.test
def test_set_dof_positions_writes_link_transforms_usd_and_fabric():
    stage = harness.new_stage()
    add_ur10e(stage, gripper="Robotiq_2F_85", position=(0.3, -0.2, 0.1))
    for target in ("usd", "both", "fabric"):
        robot = GeometricRobot.from_articulation(stage, ROBOT, write_target=target)
        reader = GeometricFabricWriter(stage, [p for p in robot.link_paths if p])
        for _ in range(3):
            q = rng.uniform(-1, 1, size=robot.num_dofs)
            robot.set_dof_positions(q)
            got_q = robot.get_dof_positions()
            np.testing.assert_allclose(got_q, robot.model.resolve_mimic(np.clip(q, *robot.get_dof_limits())), atol=1e-12)
            expect = robot.get_link_world_matrices()
            assert abs(expect[0, 0, 3] - 0.3) < 1e-9, "根リンクは参照先の位置にある"
            if target in ("usd", "both"):
                got = np.array([usd_world(stage, p) for p in robot.link_paths])
                assert_poses(got, expect, atol=1e-9, label=f"USD {target}")
            if target in ("fabric", "both"):
                harness.update(2)
                fab = np.array([split_scale(m)[0] for m in reader.read_world()])
                assert_poses(fab, expect, atol=1e-5, label=f"Fabric {target}")
        if target == "fabric":
            # Fabric だけに書いた直後、USD は前の姿勢のまま。flush_to_usd() で追いつく
            usd_before = np.array([usd_world(stage, p) for p in robot.link_paths])
            assert np.abs(usd_before - expect).max() > 1e-3
            assert robot.flush_to_usd() is True and robot.flush_to_usd() is False
            assert_poses(np.array([usd_world(stage, p) for p in robot.link_paths]), expect, atol=1e-9, label="flush")
            # USD に書いた直後に Fabric へ新しい姿勢を書いても、update の同期で古い姿勢に戻されない
            q2 = rng.uniform(-1, 1, size=robot.num_dofs)
            robot.set_dof_positions(q2)
            harness.update(4)
            fab = np.array([split_scale(m)[0] for m in reader.read_world()])
            assert_poses(fab, robot.get_link_world_matrices(), atol=1e-5, label="Fabric after flush")
        robot.close()
        harness.update(2)
    # 外した後は Fabric も元の姿勢(関節値 0)に戻る
    model, info = read_articulation(stage, ROBOT)
    reader = GeometricFabricWriter(stage, [info.link_prim_paths[n] for n in model.link_names])
    harness.update(2)
    home = usd_world(stage, f"{ROBOT}/base_link") @ model.forward_kinematics()
    assert_poses(np.array([split_scale(m)[0] for m in reader.read_world()]), home, atol=1e-5, label="Fabric after remove")


@harness.test
def test_fabric_batch_and_loop_write_the_same():
    stage = harness.new_stage()
    add_ur10e(stage, gripper="Robotiq_2F_85")
    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="usd")
    paths = [p for p in robot.link_paths if p]
    reader = GeometricFabricWriter(stage, paths)
    for batch in (False, True):
        writer = GeometricFabricWriter(stage, paths, batch=batch)
        robot.set_dof_positions(rng.uniform(-1, 1, size=robot.num_dofs))
        harness.update(3)
        robot._q = robot.model.resolve_mimic(rng.uniform(-1, 1, size=robot.num_dofs))
        world = robot.get_link_world_matrices() * 0 + (robot._base @ robot.model.forward_kinematics(robot._q))
        writer.write(robot._solver, world)
        fab = np.array([split_scale(m)[0] for m in reader.read_world()])
        assert_poses(fab, world, atol=1e-5, label=f"batch={batch}")
    robot.close()


@harness.test
def test_base_pose_limits_default_state_and_names():
    stage = harness.new_stage()
    add_ur10e(stage)
    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="usd", limit_policy="clip")
    assert robot.dof_paths[0] == f"{ROBOT}/joints/shoulder_pan_joint"
    assert robot.get_dof_indices(["elbow_joint", "wrist_3_joint"]).tolist() == [2, 5]
    assert robot.get_link_indices("forearm_link").tolist() == [3]
    assert robot.dof_types == ["revolute"] * 6
    # 範囲外は切り詰める(elbow は ±π)
    robot.set_dof_positions([9.0], dof_indices=[2])
    assert abs(robot.get_dof_positions()[2] - np.pi) < 1e-9
    robot.joints["elbow_joint"].set_position(0.4)
    assert abs(robot.joints["elbow_joint"].get_position() - 0.4) < 1e-12
    strict = GeometricRobot.from_articulation(stage, ROBOT, layer=robot.layer, limit_policy="error")
    try:
        strict.set_dof_positions([9.0], dof_indices=[2])
        raise AssertionError("範囲外で例外が出なかった")
    except ValueError:
        pass
    strict.close(remove_layer=False)
    # 根の姿勢を変えると全リンクが動く
    quat = tf.matrix_to_quat(tf.rotation_about_axis(np.array([0, 0, 1.0]), 0.7))
    robot.set_world_poses([1.0, 2.0, 0.5], quat)
    pos, ori = robot.get_world_poses()
    np.testing.assert_allclose(pos, [1.0, 2.0, 0.5])
    np.testing.assert_allclose(ori, quat, atol=1e-12)
    assert_poses(np.array([usd_world(stage, p) for p in robot.link_paths]), robot.get_link_world_matrices(), atol=1e-9, label="base moved")
    tool, _ = robot.links["wrist_3_link"].get_world_pose()
    np.testing.assert_allclose(tool, robot.get_link_world_poses(link_indices=[6])[0][0])
    # 既定の状態
    robot.set_default_state(positions=[0, 0, 0], orientations=[1, 0, 0, 0], dof_positions=np.full(6, 0.2))
    robot.reset_to_default_state()
    np.testing.assert_allclose(robot.get_dof_positions(), 0.2)
    np.testing.assert_allclose(robot.get_world_poses()[0], 0.0)
    # ヤコビアン(ワールド座標)を数値微分と比べる
    q = robot.get_dof_positions()
    jac = robot.get_jacobian_matrices()[6]
    eps = 1e-6
    for d in range(6):
        dq = np.zeros(6)
        dq[d] = eps
        robot.set_dof_positions(q + dq)
        plus = robot.get_link_world_matrices()[6]
        robot.set_dof_positions(q - dq)
        minus = robot.get_link_world_matrices()[6]
        np.testing.assert_allclose(jac[:3, d], (plus[:3, 3] - minus[:3, 3]) / (2 * eps), atol=1e-6)
        np.testing.assert_allclose(jac[3:, d], tf.rotation_vector(plus[:3, :3] @ minus[:3, :3].T) / (2 * eps), atol=1e-6)
    robot.close()


# ====================================================================== 生成元(DH・スクリュー・URDF)と結合


@harness.test
def test_built_robots_from_parameters():
    stage = harness.new_stage()
    mdh = GeometricRobot.from_mdh(stage, six_axis(), "/World/Mdh", write_target="usd")
    screw = GeometricRobot.from_screw(
        stage,
        ScrewParameters([ScrewAxisRow.through((0, 0, 0), (0, 0, 1)), ScrewAxisRow((1, 0, 0), joint_type="prismatic", lower_limit=0, upper_limit=0.5)], tool_home=tf.translation(x=0.3)),
        "/World/Screw",
        base_transform=tf.translation(y=1.0),
        write_target="usd",
    )
    urdf = GeometricRobot.from_urdf(stage, harness.FRANKA_URDF, "/World/Franka", base_transform=tf.translation(y=-1.0), write_target="usd")
    harness.update(2)
    assert mdh.num_dofs == 6 and screw.num_dofs == 2 and urdf.num_dofs == 9
    assert stage.GetPrimAtPath("/World/Mdh/link3/visuals").IsValid() and stage.GetPrimAtPath("/World/Mdh/link3/collisions").IsValid()
    # Articulation も剛体も Joint も作らない
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Mdh")):
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.HasAPI(UsdPhysics.ArticulationRootAPI) and not prim.IsA(UsdPhysics.Joint)
    for robot in (mdh, screw, urdf):
        q = rng.uniform(-1, 1, size=robot.num_dofs)
        robot.set_dof_positions(q)
        got = np.array([usd_world(stage, p) for p in robot.link_paths])
        assert_poses(got, robot.get_link_world_matrices(), atol=1e-9, label=robot.name)
    np.testing.assert_allclose(screw.get_world_poses()[0], [0, 1, 0])
    # Franka: 指 2 は指 1 に従う
    urdf.joints["panda_finger_joint1"].set_position(0.03)
    assert abs(urdf.joints["panda_finger_joint2"].get_position() - 0.03) < 1e-12
    urdf.joints["panda_finger_joint2"].set_position(0.0)  # 従動軸に書いても参照軸の値が勝つ
    assert abs(urdf.joints["panda_finger_joint2"].get_position() - 0.03) < 1e-12
    for robot in (mdh, screw, urdf):
        robot.close()


@harness.test
def test_ik_on_robot_world_targets():
    stage = harness.new_stage()
    robot = GeometricRobot.from_mdh(stage, six_axis(), "/World/Arm", base_transform=tf.make_transform(tf.rotation_about_axis(np.array([0, 0, 1.0]), 0.5), np.array([0.4, -0.3, 0.2])), write_target="usd")
    solver = robot.create_ik_solver("jacobian", "link0", "tool")
    truth = np.array([0.5, -0.6, 0.9, 0.3, 0.7, -0.4])
    robot.set_dof_positions(truth)
    target = robot.links["tool"].get_world_matrix()
    robot.set_dof_positions(np.zeros(6))
    result = robot.solve_ik(solver, target[:3, 3], tf.matrix_to_quat(target[:3, :3]))
    assert result.success, (result.position_error, result.orientation_error)
    reached = robot.links["tool"].get_world_matrix()
    assert_poses(reached, target, atol=5e-5, label="ik")
    assert_poses(usd_world(stage, robot.link_path_map["tool"]), reached, atol=1e-9, label="ik written")
    # 使う軸を後から変える: 手首 3 軸を固定して、位置だけ合わせる
    solver.set_active_dofs(["joint1", "joint2", "joint3"])
    solver.set_config(orientation_mode="none")
    before = robot.get_dof_positions()
    result = robot.solve_ik(solver, target[:3, 3] + np.array([0.03, 0.02, -0.02]))
    assert result.success, result.position_error
    np.testing.assert_array_equal(robot.get_dof_positions()[3:], before[3:])
    # 届かない目標では関節値を変えない
    before = robot.get_dof_positions()
    result = robot.solve_ik(solver, [5.0, 5.0, 5.0])
    assert not result.success
    np.testing.assert_array_equal(robot.get_dof_positions(), before)
    robot.close()


@harness.test
def test_ikpy_solver():
    from myext.robot.geometric_newton.model.ik_ikpy import import_ikpy

    try:
        ikpy = import_ikpy()
    except ImportError as exc:
        harness.note("ikpy", f"入れられなかったので IKPySolver のテストは実行していない: {exc}")
        raise
    harness.note("ikpy_version", getattr(ikpy, "__version__", "?"))
    stage = harness.new_stage()
    robot = GeometricRobot.from_urdf(stage, harness.FRANKA_URDF, "/World/Franka", write_target="usd")
    solver = robot.create_ik_solver("ikpy", "panda_link0", "panda_hand", IKSolverConfig(position_tolerance=1e-3, orientation_tolerance=1e-2))
    chain = solver.chain
    assert list(chain.variable_names) == [f"panda_joint{i}" for i in range(1, 8)]
    ok = 0
    for _ in range(8):
        truth = rng.uniform(chain.lower_limits, chain.upper_limits)
        robot.set_dof_positions(truth, dof_indices=chain.dof_indices)
        target = robot.links["panda_hand"].get_world_matrix()
        robot.set_dof_positions(truth + rng.normal(scale=0.15, size=7), dof_indices=chain.dof_indices)
        result = robot.solve_ik(solver, target[:3, 3], tf.matrix_to_quat(target[:3, :3]))
        if result.success:
            ok += 1
            assert np.linalg.norm(robot.links["panda_hand"].get_world_matrix()[:3, 3] - target[:3, 3]) < 2e-3
    harness.note("ikpy_success", f"{ok}/8")
    assert ok >= 6, ok
    # マスク: 動かさない軸は seed のまま
    solver.set_active_dofs([True, True, True, True, False, False, False])
    before = robot.get_dof_positions(dof_indices=chain.dof_indices)
    result = solver.solve(chain.forward_kinematics(before)[:3, 3] + np.array([0.02, 0.0, 0.01]), seed=before)
    np.testing.assert_array_equal(result.dof_positions[4:], before[4:])
    robot.close()


# ====================================================================== 干渉チェック


def dual_arm(stage, **kwargs) -> GeometricRobot:
    """台(1 リンク)に同じ 6 軸アームを左右 2 本付けた結合ロボット。"""
    from myext.robot.geometric_newton import GeometricJointSpec, GeometricLinkSpec, GeometricRobotModel

    base = GeometricRobotModel("cart", (GeometricLinkSpec("body"), GeometricLinkSpec("top")), (GeometricJointSpec("lift", "prismatic", "body", "top", axis=(0, 0, 1), parent_frame=tf.translation(z=0.3), lower_limit=0, upper_limit=0.3),))
    arm = six_axis().to_model()
    model = base.attach(arm, "top", tf.translation(y=0.25), name="left", base_name="cart").attach(arm, "top", tf.translation(y=-0.25), name="right")
    return GeometricRobot.from_model(stage, model, "/World/Dual", **kwargs)


@harness.test
def test_collision_scopes_on_combined_robot():
    stage = harness.new_stage()
    robot = dual_arm(stage)
    harness.update(2)
    assert robot.component_names == ["cart", "left", "right"]
    left, right = robot.get_component("left"), robot.get_component("right")
    assert left.num_dofs == 6 and left.dof_names[0] == "left/joint1"
    # 組んだ直後の姿勢で常に重なっている組(短いリンクをはさんだ組)を無視する組に足す
    added = robot.ignore_touching_pairs()
    harness.note("built_robot_touching_pairs_at_home", added)
    assert robot.check_collisions("within") == []
    comps = [{robot.model.component_of_link(a), robot.model.component_of_link(b)} for a, b in added]
    assert {"left", "right"} not in comps, f"関節値 0 では左右のアームは離れているはず: {added}"

    # 左右のアームの手先を、IK で同じ点(2 本の真ん中の前方)へ持っていってぶつける
    meet = robot.links["top"].get_world_matrix()[:3, 3] + np.array([0.45, 0.0, 0.45])
    for side in ("left", "right"):
        solver = robot.create_ik_solver("jacobian", f"{side}/link0", f"{side}/tool", IKSolverConfig(orientation_mode="none"))
        assert list(solver.chain.dof_indices) == list(robot.get_component(side).dof_indices)
        result = robot.solve_ik(solver, meet)
        assert result.success, (side, result.position_error)
    np.testing.assert_allclose(robot.links["left/tool"].get_world_matrix()[:3, 3], robot.links["right/tool"].get_world_matrix()[:3, 3], atol=1e-4)
    pairs = robot.check_collisions("within")
    cross = [p for p in pairs if {p.component_a, p.component_b} == {"left", "right"}]
    assert cross, f"左右のアームの手先が同じ点にあるのに検出されない: {[(p.names, p.kind) for p in pairs]}"
    assert all(p.kind == "between_components" for p in cross)
    assert pairs == cross, f"左右の間以外の組が出ている: {[(p.names, p.kind) for p in pairs]}"
    harness.note("dual_arm_cross_pairs", [p.names for p in pairs])
    # 構成要素の「内側」は、結合していても自分のリンク同士だけを見る
    assert left.check_collisions("within") == [] and right.check_collisions("within") == []
    # 同じ結果を、2 つの集合の間として問い合わせても得られる
    between = robot.collision_checker.check_between(left.link_set, right.link_set)
    assert {frozenset(p.names) for p in between} == {frozenset(p.names) for p in pairs}
    # フィルタの拡張: パターンで左右の組を全部無視 → 検出されない。外すと戻る
    robot.collision_filter.add_pattern("left/*", "right/*", reason="test")
    assert robot.check_collisions("within") == []
    assert robot.collision_filter.remove_by_reason("test") == 1
    assert robot.check_collisions("within")
    # 1 回だけの追加フィルタ
    assert robot.check_collisions("within", extra_filter=lambda a, b: a.startswith("left/") and b.startswith("right/")) == []

    # 片腕の中の干渉: 左腕の手先を、IK で自分の根元の柱(link1)の途中へ持っていく
    robot.reset_to_default_state()
    solver = robot.create_ik_solver("jacobian", "left/link0", "left/tool", IKSolverConfig(orientation_mode="none"))
    column = robot.links["left/link0"].get_world_matrix()[:3, 3] + np.array([0.0, 0.0, 0.2])
    assert robot.solve_ik(solver, column).success
    own = left.check_collisions("within")
    harness.note("dual_arm_left_self_pairs", [p.names for p in own])
    assert own and all(p.kind == "within_component" and p.component_a == "left" for p in own), [(p.names, p.kind) for p in own]
    assert right.check_collisions("within") == []
    assert {frozenset(p.names) for p in own} <= {frozenset(p.names) for p in robot.check_collisions("within")}
    robot.close()


@harness.test
def test_environment_collision_follows_joint_values_while_stopped():
    stage = harness.new_stage()
    robot = GeometricRobot.from_mdh(stage, six_axis(), "/World/Arm")  # 既定の書き込み先(fabric)
    robot.ignore_touching_pairs()
    # 手先の通り道に箱を置く: joint1 = 0.8 のときの手先の位置
    robot.set_dof_positions([0.8, 0.3, 0.2, 0, 0, 0])
    tip = robot.links["tool"].get_world_matrix()[:3, 3]
    robot.reset_to_default_state()
    add_box(stage, "/World/Obstacle", tip, size=0.12)
    harness.update(2)
    assert harness.timeline().is_stopped()
    assert robot.check_collisions("environment") == []
    hits_per_q = {}
    for q1 in (0.0, 0.4, 0.8, 1.2):
        robot.set_dof_positions([q1, 0.3, 0.2, 0, 0, 0])
        pairs = robot.check_collisions("environment")  # Stop のまま。チェックの前に姿勢が USD に書かれる
        hits_per_q[q1] = sorted({p.link_a for p in pairs})
        assert all(p.kind == "environment" and p.link_b is None and p.path_b.startswith("/World/Obstacle") for p in pairs)
    harness.note("environment_hits_per_joint1", hits_per_q)
    assert hits_per_q[0.0] == [] and hits_per_q[0.4] == [] and hits_per_q[1.2] == [], hits_per_q
    assert hits_per_q[0.8], hits_per_q
    # 既定の scope: 簡易形状のロボットは Articulation 由来ではないので "all"
    assert robot.is_in_collision() is False
    robot.set_dof_positions([0.8, 0.3, 0.2, 0, 0, 0])
    assert robot.is_in_collision() is True
    assert robot.check_collisions("environment", ignore_prim_paths=["/World/Obstacle"]) == []
    # Play 中も同じ結果
    harness.play(3)
    assert robot.is_in_collision("environment") is True
    robot.set_dof_positions([0.0, 0.3, 0.2, 0, 0, 0])
    assert robot.is_in_collision("environment") is False
    harness.stop(2)
    robot.close()


@harness.test
def test_existing_articulation_collisions_with_mesh_colliders():
    stage = harness.new_stage()
    add_ur10e(stage, gripper="Robotiq_2F_85")
    robot = GeometricRobot.from_articulation(stage, UsdArticulationSource(ROBOT, components={"arm": ROBOT, "gripper": f"{ROBOT}/Robotiq_2F_85"}))
    colliders = robot.collider_paths
    harness.note("ur10e_collider_counts", {k: len(v) for k, v in colliders.items()})
    assert all(len(colliders[n]) >= 1 for n in ("upper_arm_link", "forearm_link", "wrist_3_link")), colliders
    # 元の Articulation は自己干渉を無効にしているので、既定では環境との干渉だけを見る
    assert robot.info.self_collision_enabled is False
    robot.set_dof_positions(np.zeros(robot.num_dofs))
    assert robot.check_collisions() == []
    tip = robot.links["wrist_3_link"].get_world_matrix()[:3, 3]
    add_box(stage, "/World/Obstacle", tip, size=0.1)
    harness.update(2)
    pairs = robot.check_collisions()
    assert pairs and all(p.kind == "environment" for p in pairs), pairs
    harness.note("ur10e_environment_hits", sorted({p.link_a for p in pairs}))
    # 関節を動かして離す
    robot.set_dof_positions([1.2], dof_indices=[0])
    assert robot.check_collisions() == []
    # 明示すれば内側も調べられる。肘を折りたたむと前腕側と上腕側が当たる
    robot.set_dof_positions([0, -1.2, 2.9, 0, 0, 0], dof_indices=list(range(6)))
    within = robot.check_collisions("within")
    harness.note("ur10e_folded_within_pairs", [p.names for p in within])
    assert within, "肘を折りたたんでも内側の干渉が検出されない"
    robot.close()


@harness.test
def test_kinematic_mode_contact_monitor_in_play():
    stage = harness.new_stage()
    options = GeometricLayerOptions(collision_mode="kinematic")
    robot = GeometricRobot.from_mdh(stage, six_axis(), "/World/Arm", layer_options=options)
    robot.set_dof_positions([0.8, 0.3, 0.2, 0, 0, 0])
    tip = robot.links["tool"].get_world_matrix()[:3, 3]
    robot.reset_to_default_state()
    add_box(stage, "/World/Obstacle", tip, size=0.12)
    harness.update(2)
    link = stage.GetPrimAtPath(robot.link_path_map["link3"])
    assert UsdPhysics.RigidBodyAPI(link).GetKinematicEnabledAttr().Get() is True
    assert link.HasAPI(PhysxSchema.PhysxContactReportAPI)
    scene = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/physicsScene"))
    assert scene.GetReportKinematicKinematicPairsAttr().Get() is True and scene.GetReportKinematicStaticPairsAttr().Get() is True
    try:  # PhysX バックエンドの Stop 中 overlap は、書いた姿勢が PhysX に入らないので断る
        robot.check_collisions("environment", backend="physx")
        raise AssertionError("kinematic + Stop (physx) で例外が出なかった")
    except RuntimeError:
        pass
    # Newton バックエンドなら kinematic + Stop でも例外にならず動作する
    assert robot.check_collisions("environment", backend="newton") == []
    monitor = GeometricContactMonitor(robot)
    harness.play(3)
    seen = {}
    for q1 in (0.0, 0.3, 0.6, 0.8, 0.8, 0.8, 1.2, 1.2, 1.2):
        robot.set_dof_positions([q1, 0.3, 0.2, 0, 0, 0])
        harness.update(1)
        seen.setdefault(q1, []).append(sorted({p.link_a for p in monitor.penetrating() if p.kind == "environment"}))
    harness.note("kinematic_contact_monitor_penetrating", seen)
    assert seen[0.0] == [[]] and seen[0.3] == [[]], seen
    assert seen[0.8][-1], f"箱の位置に手先を置いて 3 update 待っても報告が無い: {seen}"
    assert seen[1.2][-1] == [], f"離れて 3 update 待っても報告が残っている: {seen}"
    # ロボットの中の隣同士は報告されない(FilteredPairs とフィルタ)
    assert not any(p.kind != "environment" and robot.collision_filter.is_ignored(p.link_a, p.link_b) for p in monitor.touching())
    # Play 中は overlap も使える(反映は次の update)
    robot.set_dof_positions([0.8, 0.3, 0.2, 0, 0, 0])
    harness.update(2)
    assert robot.is_in_collision("environment") is True
    monitor.close()
    harness.stop(2)
    robot.close()
    harness.update(2)
    assert not stage.GetPrimAtPath("/World/Arm/link3").HasAPI(UsdPhysics.RigidBodyAPI), "外した後に剛体の設定が残っている"


# ====================================================================== 複数台


@harness.test
def test_view_matches_articulation_shapes_and_single_robots():
    import warp as wp

    stage = harness.new_stage()
    paths = [f"/World/Robot{i}" for i in range(4)]
    for i, path in enumerate(paths):
        add_ur10e(stage, path, position=(0.0, 1.5 * i, 0.0))
    for target in ("fabric", "usd"):
        view = GeometricRobotView.from_articulations(stage, paths, write_target=target)
        assert len(view) == 4 and view.num_dofs == 6 and len(stage.GetSessionLayer().subLayerPaths) == 1
        q = rng.uniform(-1, 1, size=(4, 6)).astype(np.float32)
        view.set_dof_positions(q)
        got = view.get_dof_positions()
        assert isinstance(got, wp.array) and got.shape == (4, 6)
        np.testing.assert_allclose(got.numpy(), q, atol=1e-6)
        lower, upper = view.get_dof_limits()
        assert lower.shape == (4, 6) and upper.shape == (4, 6)
        pos, quat = view.get_world_poses()
        assert pos.shape == (4, 3) and quat.shape == (4, 4)
        np.testing.assert_allclose(pos.numpy()[:, 1], [0.0, 1.5, 3.0, 4.5], atol=1e-6)
        # 一部の台・一部の軸だけ書く(Articulation と同じ引数)
        view.set_dof_positions(np.array([[0.5, -0.5]]), indices=[1, 3], dof_indices=view.get_dof_indices(["elbow_joint", "wrist_1_joint"]))
        q[[1, 3], 2], q[[1, 3], 3] = 0.5, -0.5
        np.testing.assert_allclose(view.get_dof_positions().numpy(), q, atol=1e-6)
        np.testing.assert_allclose(view.get_dof_positions(indices=[3], dof_indices=[2, 3]).numpy(), [[0.5, -0.5]], atol=1e-6)
        # まとめて計算した姿勢が、1 台ずつ計算した姿勢と一致し、ステージにも書かれている
        world = view.get_link_world_matrices()
        model = view.robots[0].model
        for i, robot in enumerate(view.robots):
            expect = usd_world(stage, f"{paths[i]}/base_link") @ model.forward_kinematics(q[i].astype(float))
            assert_poses(world[i], expect, atol=1e-6, label=f"view robot {i}")
            np.testing.assert_allclose(robot.get_dof_positions(), view.get_dof_positions().numpy()[i], atol=1e-6)
        link_pos, link_quat = view.get_link_world_poses()
        assert link_pos.shape == (4, 7, 3) and link_quat.shape == (4, 7, 4)
        if target == "usd":
            got_usd = np.array([[usd_world(stage, p) for p in r.link_paths] for r in view.robots])
            assert_poses(got_usd, world, atol=1e-9, label="view usd")
        else:
            harness.update(2)
            reader = GeometricFabricWriter(stage, [p for r in view.robots for p in r.link_paths])
            fab = np.array([split_scale(m)[0] for m in reader.read_world()]).reshape(4, 7, 4, 4)
            assert_poses(fab, world, atol=1e-5, label="view fabric")
            # Newton バックエンド(既定)では USD へ flush せずに干渉チェックできる
            assert view.robots[2].check_collisions("within") is not None
            assert view._usd_dirty is True, "Newton バックエンドは不要な flush_to_usd() を呼ばないはず"
            # PhysX バックエンドでは View 経由で USD に flush してから干渉チェックが行われる
            assert view.robots[2].check_collisions("within", backend="physx") is not None
            assert view._usd_dirty is False
            got_usd = np.array([[usd_world(stage, p) for p in r.link_paths] for r in view.robots])
            assert_poses(got_usd, world, atol=1e-9, label="view usd after check")
        view.close()
        harness.update(2)
        assert len(stage.GetSessionLayer().subLayerPaths) == 0


# ====================================================================== VariantSet と作り直し(2026-11-18)

import tempfile as _tempfile

from multi_arm_robot import ARMS, add_multi_arm_robot, write_multi_arm_asset

from myext.robot.geometric_newton import (
    GeometricStructureChangedError,
    LinearCommandConverter,
)

_MULTI_ARM_DIR = _tempfile.mkdtemp()
MULTI_ARM_USD = write_multi_arm_asset(os.path.join(_MULTI_ARM_DIR, "multi_arm.usda"))
ARM_COMPONENTS = {**{f"arm{i}": f"{ROBOT}/arm{i}_*" for i in range(ARMS)}, "cart": [f"{ROBOT}/base", f"{ROBOT}/column", f"{ROBOT}/boom"]}


def multi_arm(stage, path=ROBOT, position=(0.0, 0.0, 0.0)):
    add_multi_arm_robot(stage, path, MULTI_ARM_USD, position)
    harness.update(3)


def orphan_overs(robot) -> list[str]:
    """幾何モードの Sublayer にある、いまは存在しない prim への上書き。"""
    layer = robot.layer.sdf_layer
    out = []
    if layer is None:
        return out
    stage = robot.stage

    def visit(path):
        if path.IsPrimPath() and not stage.GetPrimAtPath(path).IsDefined():
            out.append(str(path))

    layer.Traverse("/", visit)
    return out


def tool_body_paths(stage, arm: int) -> list[str]:
    root = stage.GetPrimAtPath(f"{ROBOT}/arm{arm}_tool")
    return [str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


@harness.test
def test_multi_arm_matches_physx():
    """4 本アーム(54 軸・59 リンク)の名前・並び・順運動学が、Play した Articulation と一致する。"""
    stage = harness.new_stage()
    multi_arm(stage)
    model, _info, _truth = check_against_physx(stage, ROBOT, lambda m: random_configs(m, 4, scale=0.3), "multi_arm")
    assert model.num_dofs == 54 and model.num_links == 59, (model.num_dofs, model.num_links)


def _variant_switch_case(collision_mode: str):
    stage = harness.new_stage(gravity=9.81)
    multi_arm(stage)
    root_text = stage.GetRootLayer().ExportToString()
    before = snapshot(stage)
    robot = GeometricRobot.from_articulation(stage, UsdArticulationSource(ROBOT, components=ARM_COMPONENTS), layer_options=GeometricLayerOptions(collision_mode=collision_mode))
    events = []
    robot.add_structure_listener(lambda old, new: events.append((len(old), len(new))))
    q = rng.uniform(-0.3, 0.3, size=robot.num_dofs)
    robot.set_dof_positions(q)
    kept = {n: v for n, v in zip(robot.dof_names, robot.get_dof_positions()) if not n.startswith("t_") and not n.startswith("jaw")}
    version = robot.structure_version

    # wrist(5 軸) → camera(0 軸)
    robot.set_variant("Instrument", "camera", prim="arm0_tool")
    assert robot.num_dofs == 49, robot.num_dofs
    assert robot.structure_version == version + 1 and events[-1] == (54, 49), (robot.structure_version, events)
    after = dict(zip(robot.dof_names, robot.get_dof_positions()))
    for name, value in kept.items():  # 残った軸は名前で値を引き継ぐ
        assert abs(after[name] - value) < 1e-12, name
    assert orphan_overs(robot) == [], orphan_overs(robot)[:3]
    # 選択は Session Layer に書かれ、Root Layer は変わらない
    assert stage.GetRootLayer().ExportToString() == root_text
    session_spec = stage.GetSessionLayer().GetPrimAtPath(f"{ROBOT}/arm0_tool")
    assert session_spec is not None and session_spec.variantSelections.get("Instrument") == "camera"
    # 書き込まれたリンクの姿勢が、新しいモデルの計算と一致する
    robot.flush_to_usd()
    assert_poses(np.array([usd_world(stage, p) for p in robot.link_paths]), robot.get_link_world_matrices(), atol=1e-9, label="after switch")

    # camera → stapler(3 軸): 新しい器具の剛体にも幾何モードの上書きが入る
    robot.set_variant("Instrument", "stapler", prim=f"{ROBOT}/arm0_tool")
    assert robot.num_dofs == 52
    for path in tool_body_paths(stage, 0):
        body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
        if collision_mode == "static":
            assert body.GetRigidBodyEnabledAttr().Get() is False, path
        else:
            assert body.GetKinematicEnabledAttr().Get() is True and stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxContactReportAPI), path
    joints = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROBOT}/arm0_tool")) if p.IsA(UsdPhysics.Joint)]
    assert joints and all(UsdPhysics.Joint(p).GetJointEnabledAttr().Get() is False for p in joints)
    assert orphan_overs(robot) == []
    # Play しても、新しい器具は動かない(物理で落ちない)
    tip = tool_body_paths(stage, 0)[-1]
    z0 = usd_world(stage, tip)[2, 3]
    harness.play(30)
    z1 = usd_world(stage, tip)[2, 3]
    harness.stop(2)
    harness.note(f"variant_switch_play_tip_z[{collision_mode}]", [round(z0, 5), round(z1, 5)])
    assert abs(z1 - z0) < 1e-6, (z0, z1)
    if collision_mode == "static":
        assert isinstance(robot.check_collisions("within"), list)  # 作り直した後の干渉チェック(PhysX の読み直しを含む)
    # 元に戻して Sublayer を外すと、最初と差分 0
    robot.set_variant("Instrument", "wrist", prim="arm0_tool")
    assert robot.num_dofs == 54
    robot.close()
    harness.update(2)
    after_all = snapshot(stage)
    diff = [p for p in set(before) | set(after_all) if before.get(p) != after_all.get(p)]
    assert not diff, diff[:5]
    harness.note(f"variant_rebuild_timings[{collision_mode}]", robot.last_rebuild_timings)


@harness.test
def test_variant_switch_rebuilds_static():
    _variant_switch_case("static")


@harness.test
def test_variant_switch_rebuilds_kinematic():
    _variant_switch_case("kinematic")


@harness.test
def test_set_variant_validates_and_batches():
    stage = harness.new_stage()
    multi_arm(stage)
    robot = GeometricRobot.from_articulation(stage, UsdArticulationSource(ROBOT, components=ARM_COMPONENTS))
    choices = robot.get_variant_choices()
    assert sorted(choices[f"{ROBOT}/arm2_tool"]["Instrument"]) == ["camera", "stapler", "wrist"], choices
    assert robot.get_variants()[f"{ROBOT}/arm2_tool"] == {"Instrument": "wrist"}
    version, dofs = robot.structure_version, robot.num_dofs
    bad = [
        dict(variant_set="Instrument", variant="camera", prim=None),  # 4 本のアームにあるので決まらない
        dict(variant_set="Instrument", variant="camera", prim="arm9_tool"),  # prim が無い
        dict(variant_set="Gripper", variant="camera", prim="arm0_tool"),  # VariantSet が無い
        dict(variant_set="Instrument", variant="laser", prim="arm0_tool"),  # Variant が無い
        dict(variant_set="Instrument", variant="camera", prim="/World/physicsScene"),  # ロボットの範囲の外
    ]
    messages = []
    for kwargs in bad:
        try:
            robot.set_variant(kwargs["variant_set"], kwargs["variant"], prim=kwargs["prim"])
            raise AssertionError(f"例外が出なかった: {kwargs}")
        except ValueError as exc:
            messages.append(str(exc)[:120])
    harness.note("set_variant_error_messages", messages)
    # まとめて切り替え: 1 つでも不正なら何も変えない
    try:
        robot.set_variants({("arm0_tool", "Instrument"): "camera", ("arm1_tool", "Instrument"): "laser"})
        raise AssertionError("例外が出なかった")
    except ValueError:
        pass
    assert robot.get_variants()[f"{ROBOT}/arm0_tool"] == {"Instrument": "wrist"} and robot.structure_version == version and robot.num_dofs == dofs
    # 正しいまとめて切り替え: 作り直しは 1 回
    robot.set_variants({("arm0_tool", "Instrument"): "camera", ("arm1_tool", "Instrument"): "stapler", ("arm3_tool", "Instrument"): "camera"})
    assert robot.structure_version == version + 1 and robot.num_dofs == 54 - 5 - 2 - 5, (robot.structure_version, robot.num_dofs)
    # 構成要素から: 範囲の中に Instrument が 1 つだけなので prim を省略できる
    arm2 = robot.get_component("arm2")
    assert arm2.num_dofs == 13, arm2.num_dofs
    arm2.set_variant("Instrument", "stapler")
    assert arm2.num_dofs == 11 and robot.get_variants()[f"{ROBOT}/arm2_tool"] == {"Instrument": "stapler"}
    assert arm2.get_variants() == {f"{ROBOT}/arm2_tool": {"Instrument": "stapler"}}
    assert robot.get_component("arm0").num_dofs == 8 and robot.get_component("cart").num_dofs == 2
    # パラメータから作ったロボットは VariantSet を持たない
    built = GeometricRobot.from_mdh(stage, six_axis(), "/World/Mdh", layer=robot.layer)
    try:
        built.set_variant("Instrument", "camera")
        raise AssertionError("例外が出なかった")
    except RuntimeError:
        pass
    built.close(remove_layer=False)
    robot.close()


@harness.test
def test_external_variant_change_rebuilds_on_next_update():
    """Stage タブと同じく、編集先(Root Layer)に Variant の選択を書くと、次の update で作り直される。"""
    stage = harness.new_stage()
    multi_arm(stage)
    robot = GeometricRobot.from_articulation(stage, ROBOT)
    harness.update(2)
    version = robot.structure_version
    # 自分の書き込み(関節値・Sublayer への色)では作り直さない
    robot.set_dof_positions(np.full(robot.num_dofs, 0.1))
    robot.flush_to_usd()
    with robot.layer.edit_context():
        UsdGeom.Gprim(stage.GetPrimAtPath(f"{ROBOT}/arm0_s1/collision")).CreateDisplayColorAttr([Gf.Vec3f(1, 0, 0)])
    harness.update(2)
    assert robot.__dict__["_structure_version"] == version, "自分の書き込みで作り直された"
    # 外から(Root Layer に)書く
    stage.GetPrimAtPath(f"{ROBOT}/arm1_tool").GetVariantSet("Instrument").SetVariantSelection("camera")
    assert robot.__dict__["_structure_version"] == version  # 通知の中では作り直さない
    harness.update(1)
    assert robot.__dict__["_structure_version"] == version + 1 and robot.__dict__["_model"].num_dofs == 49, "次の update で作り直されていない"
    # API 呼び出しでも追いつく(update を待たない)
    stage.GetPrimAtPath(f"{ROBOT}/arm1_tool").GetVariantSet("Instrument").SetVariantSelection("stapler")
    assert robot.num_dofs == 52
    assert orphan_overs(robot) == []
    # Sublayer を外した状態(元の Articulation で使うとき)でも、名前と並びは追従する
    robot.layer.remove()
    stage.GetPrimAtPath(f"{ROBOT}/arm1_tool").GetVariantSet("Instrument").SetVariantSelection("wrist")
    harness.update(1)
    assert robot.num_dofs == 54
    truth = physx_truth(ROBOT, None)
    assert truth["dof_names"] == robot.dof_names
    robot.layer.apply()
    assert orphan_overs(robot) == []
    robot.close()


@harness.test
def test_stale_chain_and_view_raise_after_rebuild():
    stage = harness.new_stage()
    multi_arm(stage, f"{ROBOT}")
    multi_arm(stage, "/World/Robot2", position=(3.0, 0.0, 0.0))
    view = GeometricRobotView.from_articulations(stage, [ROBOT, "/World/Robot2"])
    view.set_dof_positions(np.zeros((2, view.num_dofs)))
    robot = view.robots[0]
    solver = robot.create_ik_solver("jacobian", "boom", "arm1_m4")
    robot.set_variant("Instrument", "camera", prim="arm0_tool")  # arm1 の鎖の軸の番号がずれる
    try:
        robot.solve_ik(solver, [0.5, 0.0, 1.5])
        raise AssertionError("古い鎖で例外が出なかった")
    except GeometricStructureChangedError:
        pass
    fresh = robot.create_ik_solver("jacobian", "boom", "arm1_m4", IKSolverConfig(orientation_mode="none"))
    target = robot.links["arm1_m4"].get_world_matrix()[:3, 3] + np.array([0.02, 0.0, 0.0])
    assert robot.solve_ik(fresh, target).success
    try:
        view.set_dof_positions(np.zeros((2, 54)))
        raise AssertionError("View で例外が出なかった")
    except GeometricStructureChangedError:
        pass
    view.close()


@harness.test
def test_ur10e_gripper_variant_switch_in_geometric_mode():
    stage = harness.new_stage(gravity=9.81)
    add_ur10e(stage, gripper="Robotiq_2F_85")
    robot = GeometricRobot.from_articulation(stage, ROBOT)
    assert robot.num_dofs == 12
    robot.set_variant("Gripper", "None")  # VariantSet はロボットの prim 自体に付いている。1 つしか無いので prim を省略できる
    assert robot.num_dofs == 6 and orphan_overs(robot) == []
    robot.set_variant("Gripper", "Robotiq_2F_140")
    assert robot.num_dofs == 14 and orphan_overs(robot) == []
    gripper = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if "2F_140" in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    z0 = usd_world(stage, gripper[0])[2, 3]
    harness.play(60)
    z1 = usd_world(stage, gripper[0])[2, 3]
    harness.stop(2)
    harness.note("ur10e_2F_140_play_z(作り直し後)", [round(z0, 5), round(z1, 5)])
    assert abs(z1 - z0) < 1e-6, "切り替えた後のグリッパが物理で動いた"
    robot.close()


@harness.test
def test_angle_units():
    stage = harness.new_stage()
    multi_arm(stage)
    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="usd")
    names = robot.dof_names
    lift, yaw = names.index("lift"), names.index("boom_yaw")
    robot.set_dof_positions([0.1, 30.0], dof_indices=[lift, yaw], unit="deg")  # 直動はそのまま、回転だけ度
    np.testing.assert_allclose(robot.get_dof_positions(dof_indices=[lift, yaw]), [0.1, np.deg2rad(30.0)])
    np.testing.assert_allclose(robot.get_dof_positions(dof_indices=[lift, yaw], unit="deg"), [0.1, 30.0])
    lower, upper = robot.get_dof_limits(dof_indices=[lift, yaw], unit="deg")
    np.testing.assert_allclose([lower[0], upper[0], lower[1], upper[1]], [-0.1, 0.3, -170.0, 170.0], atol=1e-9)
    robot.angle_unit = "deg"
    robot.joints["boom_yaw"].set_position(45.0)
    assert abs(robot.joints["boom_yaw"].get_position() - 45.0) < 1e-12
    assert abs(robot.joints["boom_yaw"].get_position(unit="rad") - np.pi / 4) < 1e-12
    assert robot.joints["boom_yaw"].get_limits() == (-170.0, 170.0)
    for bad in ("degree", "RAD"):
        try:
            robot.set_dof_positions([0.0], dof_indices=[yaw], unit=bad)  # type: ignore[arg-type]
            raise AssertionError("不正な単位で例外が出なかった")
        except ValueError:
            pass
    deg_robot = GeometricRobot.from_articulation(stage, ROBOT, layer=robot.layer, angle_unit="deg")  # 既定が度のロボット(関節値は 0 から始まる)
    deg_robot.set_dof_positions([10.0], dof_indices=[yaw])
    np.testing.assert_allclose(deg_robot.get_dof_positions(dof_indices=[yaw]), [10.0])
    np.testing.assert_allclose(deg_robot.get_dof_positions(dof_indices=[yaw], unit="rad"), [np.deg2rad(10.0)])
    deg_robot.close(remove_layer=False)
    robot.close()


@harness.test
def test_command_converter_follows_variant():
    stage = harness.new_stage()
    multi_arm(stage)
    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="usd")
    # wrist のときだけ: ログの「grip」(度)1 つ → 2 本の顎に半分ずつ、逆向き
    wrist = LinearCommandConverter(["arm0_grip"], ["jaw_a", "jaw_b"], [[0.5], [-0.5]], output_angle_unit="deg")
    stapler = LinearCommandConverter(["arm0_grip"], ["jaw"], [[1.0]], output_angle_unit="deg")
    robot.set_command_converter("arm0_wrist", wrist, when={("arm0_tool", "Instrument"): "wrist"})
    robot.set_command_converter("arm0_stapler", stapler, when={("arm0_tool", "Instrument"): "stapler"})
    assert robot.active_command_converters() == ["arm0_wrist"]
    robot.apply_command({"arm0_grip": 40.0, "boom_yaw": 0.2})  # 変換の入力と、DOF 名そのものを混ぜられる
    np.testing.assert_allclose(robot.get_dof_positions(dof_indices=robot.get_dof_indices(["jaw_a", "jaw_b", "boom_yaw"])), [np.deg2rad(20), np.deg2rad(-20), 0.2])
    assert abs(robot.read_command()["arm0_grip"] - 40.0) < 1e-9
    try:
        robot.apply_command({"arm0_grip": 10.0, "jaw_a": 0.0})  # 同じ DOF への二重の書き込み
        raise AssertionError("例外が出なかった")
    except ValueError:
        pass
    try:
        robot.apply_command({"nothing": 1.0})
        raise AssertionError("例外が出なかった")
    except KeyError:
        pass
    # Variant を切り替えると、使われる変換も切り替わる
    robot.set_variant("Instrument", "stapler", prim="arm0_tool")
    assert robot.active_command_converters() == ["arm0_stapler"]
    robot.apply_command({"arm0_grip": 30.0})
    assert abs(robot.joints["jaw"].get_position() - np.deg2rad(30.0)) < 1e-12
    robot.set_variant("Instrument", "camera", prim="arm0_tool")
    assert robot.active_command_converters() == []
    try:
        robot.apply_command({"arm0_grip": 30.0})  # camera には顎が無いので、どの変換も使われない
        raise AssertionError("例外が出なかった")
    except KeyError:
        pass
    robot.close()


@harness.test
def test_rebuild_and_per_frame_timing_on_multi_arm():
    """作り直しと毎フレームの処理の時間を測って残す(合否は緩い上限だけ)。"""
    import statistics
    import time

    stage = harness.new_stage()
    multi_arm(stage)
    robot = GeometricRobot.from_articulation(stage, ROBOT)
    qs = [robot.model.resolve_mimic(rng.uniform(-0.3, 0.3, size=robot.num_dofs)) for _ in range(32)]

    def median_ms(fn, n=100):
        samples = []
        for k in range(n):
            t0 = time.perf_counter()
            fn(k)
            samples.append(time.perf_counter() - t0)
        return round(statistics.median(samples) * 1e3, 3)

    fk = median_ms(lambda k: robot.model.forward_kinematics(qs[k % 32]))
    fk_ref = median_ms(lambda k: robot.model.forward_kinematics_reference(qs[k % 32]))
    set_q = median_ms(lambda k: robot.set_dof_positions(qs[k % 32]))
    rebuilds = []
    for variant in ("camera", "stapler", "wrist", "camera", "wrist"):
        t0 = time.perf_counter()
        robot.set_variant("Instrument", variant, prim="arm2_tool")
        rebuilds.append(round((time.perf_counter() - t0) * 1e3, 2))
    harness.note("multi_arm_timing_ms", {"forward_kinematics": fk, "forward_kinematics_reference(1 リンクずつ)": fk_ref, "set_dof_positions_fabric": set_q, "set_variant_rebuild": rebuilds, "last_breakdown": robot.last_rebuild_timings})
    assert fk < fk_ref, (fk, fk_ref)
    assert max(rebuilds) < 200.0, rebuilds
    robot.close()


# ====================================================================== デモ


@harness.test
def test_script_editor_demo_runs():
    """demo/demo_script_editor.py を「貼って実行」と同じ形(exec + 非同期タスク)で最後まで走らせる。"""
    path = os.path.join(os.path.dirname(harness.HERE), "demo", "demo_script_editor.py")
    namespace = {"__name__": "__main__"}
    exec(compile(open(path, encoding="utf-8").read(), path, "exec"), namespace)
    for _ in range(3000):
        if namespace["task"].done():
            break
        harness.update(1)
    result = namespace["task"].result()
    hits = result["hit_frames"]
    harness.note("script_editor_demo", {"hit_frames": len(hits), "first": hits[:1], "tool": np.round(result["tool_position"], 3).tolist()})
    assert 0 < len(hits) < namespace["FRAMES"], "箱に当たるフレームと当たらないフレームの両方があるはず"
    stage = omni.usd.get_context().get_stage()
    assert len(stage.GetSessionLayer().subLayerPaths) == 0, "close() の後に Sublayer が残っている"


# ====================================================================== Newton バックエンド & Play=ON (0 フレーム遅延)


@harness.test
def test_newton_playon_zero_frame_lag_fabric_only():
    """Play=ON + Fabric のみ書き込みで、USD フラッシュなし・0 フレーム遅延の自己/環境干渉判定と PhysX 剛体共存を確かめる。"""
    stage = harness.new_stage(gravity=9.81)
    add_ur10e(stage)
    # 別の動的 PhysX 剛体(重力で落下する)を離れた位置に置き、PhysX との共存を確認
    add_box(stage, "/World/DynamicCube", position=(-2.0, -2.0, 2.0), size=0.2, kind="dynamic")
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath("/World/DynamicCube")).CreateMassAttr(1.0)
    # 環境障害物(q_hit で wrist_1_link に重なる位置へ置く)
    add_box(stage, "/World/Obstacle", position=(0.0, 0.0, 5.0), size=0.30, kind="static")

    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="fabric", collision_backend="newton")
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])
    q_hit = np.array([0.35, -0.35, 0.65, -0.3, 0.5, 0.0])

    # q_hit の wrist_1_link 位置へ障害物を配置してから、q_safe に戻して USD に反映
    robot.set_dof_positions(q_hit)
    w1_pos = robot.links["wrist_1_link"].get_world_pose()[0]
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/Obstacle")).GetOrderedXformOps()[0].Set(Gf.Vec3d(*w1_pos.tolist()))
    robot.set_dof_positions(q_safe)
    robot.flush_to_usd()
    robot.scene.sync_environment(use_fabric=False)
    harness.update(2)

    fab_reader = GeometricFabricWriter(stage, list(robot._prim_paths), batch=False)
    dyn_reader = GeometricFabricWriter(stage, ["/World/DynamicCube"], batch=False)

    harness.play(3)
    z_dyn_0 = float(dyn_reader.read_world()[0, 2, 3])
    safe_world_0 = robot.get_link_world_matrices()[robot._prim_link_indices]
    harness.update(5)
    z_dyn_1 = float(dyn_reader.read_world()[0, 2, 3])
    assert z_dyn_0 - z_dyn_1 > 0.01, f"共存する PhysX 動的剛体が重力で落ちていない: {z_dyn_0} -> {z_dyn_1}"
    persist_err = float(np.max(np.linalg.norm(fab_reader.read_world()[:, :3, 3] - safe_world_0[:, :3, 3], axis=1)))
    assert persist_err < 1e-5, f"Play=ON 中に GeometricRobot の姿勢がずれた: {persist_err}"

    # 1. q_safe -> q_self へジャンプ(Fabric のみ、update 前の Frame 0)
    usd_w1_before = usd_world(stage, robot.link_path_map["wrist_1_link"])[:3, 3]
    robot.set_dof_positions(q_self)
    assert robot._usd_dirty, "write_target='fabric' なのに USD が dirty でない"
    pairs_self_f0 = robot.check_collisions("within")
    assert robot._usd_dirty, "Newton バックエンドが不要な flush_to_usd() を呼んだ"
    usd_w1_after = usd_world(stage, robot.link_path_map["wrist_1_link"])[:3, 3]
    assert np.linalg.norm(usd_w1_after - usd_w1_before) < 1e-6, "Newton 判定後に USD が書き換わっている"

    got_self = {frozenset((p.link_a, p.link_b)) for p in pairs_self_f0}
    expect_self = {
        frozenset(("base_link", "forearm_link")),
        frozenset(("forearm_link", "shoulder_link")),
        frozenset(("upper_arm_link", "wrist_1_link")),
        frozenset(("upper_arm_link", "wrist_2_link")),
        frozenset(("upper_arm_link", "wrist_3_link")),
    }
    assert got_self == expect_self, f"Frame 0 自己干渉不一致: got={got_self}, expect={expect_self}"
    for p in pairs_self_f0:
        assert p.separation is not None and p.separation <= 0.0, f"separation が非負: {p}"
        assert p.point_a is not None and p.point_b is not None and p.normal is not None

    # 2. q_safe に戻した直後(Frame 0)に 0 件へ即クリア
    robot.set_dof_positions(q_safe)
    assert robot.check_collisions("within") == []

    # 3. q_hit へジャンプした直後(Frame 0)に環境障害物を即検出
    robot.set_dof_positions(q_hit)
    pairs_env_f0 = robot.check_collisions("environment")
    assert any(p.path_b == "/World/Obstacle" and p.separation is not None and p.separation <= 0.0 for p in pairs_env_f0), pairs_env_f0

    harness.note(
        "newton_playon_zero_frame_lag",
        {
            "dyn_cube_fell_m": round(z_dyn_0 - z_dyn_1, 4),
            "persist_err_m": persist_err,
            "self_f0_count": len(pairs_self_f0),
            "env_f0_count": len(pairs_env_f0),
            "min_env_sep_m": round(min(p.separation for p in pairs_env_f0 if p.separation is not None), 5),
        },
    )
    harness.stop(2)
    robot.close()


@harness.test
def test_newton_playon_no_physics_scene_needed():
    """ステージに UsdPhysics.Scene が一切無くても、Newton の GeometricScene が Stop / Play=ON で動作することを確かめる。"""
    if not harness.timeline().is_stopped():
        harness.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    harness.update(2)
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    assert not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()), "PhysicsScene が無い前提"

    add_ur10e(stage)
    add_box(stage, "/World/Obstacle", position=(0.0, 0.0, 5.0), size=0.25, kind="static")
    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="fabric", collision_backend="newton")
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])

    # Stop 状態
    robot.set_dof_positions(q_self)
    assert len(robot.check_collisions("within")) == 5

    # Play=ON 状態(PhysicsScene なし)
    harness.play(3)
    robot.set_dof_positions(q_safe)
    assert robot.check_collisions("within") == []
    robot.set_dof_positions(q_self)
    pairs = robot.check_collisions("within")
    assert len(pairs) == 5
    clearances = robot.compute_clearances("within", max_distance=0.10)
    assert len(clearances) >= 5
    harness.stop(2)
    robot.close()


@harness.test
def test_newton_playon_dynamic_obstacle_sync_and_clearance():
    """Play=ON 中に障害物を動かしたとき、sync_environment() でモデル再構築なし・0 フレーム遅延で干渉とクリアランスが追従する。"""
    stage = harness.new_stage(gravity=0.0)
    add_ur10e(stage)
    add_box(stage, "/World/MovingObstacle", position=(3.0, 3.0, 1.0), size=0.20, kind="static")
    obs_op = UsdGeom.Xformable(stage.GetPrimAtPath("/World/MovingObstacle")).GetOrderedXformOps()[0]

    robot = GeometricRobot.from_articulation(stage, ROBOT, write_target="fabric", collision_backend="newton")
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    robot.set_dof_positions(q_safe)
    robot.flush_to_usd()
    harness.update(2)

    harness.play(3)
    assert robot.check_collisions("environment") == []
    w3_pos = robot.links["wrist_3_link"].get_world_pose()[0]

    # 1. wrist_3_link にめり込む位置へ障害物を直接移動 -> sync_environment(use_fabric=False) で即検出
    obs_op.Set(Gf.Vec3d(*w3_pos.tolist()))
    robot.scene.sync_environment(use_fabric=False)
    hit_pairs = robot.check_collisions("environment")
    assert any(p.path_b == "/World/MovingObstacle" and p.separation is not None and p.separation < 0.0 for p in hit_pairs), hit_pairs

    # 2. 少し離した位置(クリアランス帯内)へ set_obstacle_pose -> 干渉なし、compute_clearances で正の距離を返す
    far_pos = w3_pos + np.array([0.19, 0.0, 0.0])
    robot.scene.set_obstacle_pose("/World/MovingObstacle", far_pos.tolist())
    assert robot.check_collisions("environment") == []
    cl_pairs = robot.compute_clearances("environment", max_distance=0.20)
    obs_cl = [p for p in cl_pairs if p.path_b == "/World/MovingObstacle"]
    assert obs_cl and obs_cl[0].separation is not None and 0.0 < obs_cl[0].separation < 0.20, obs_cl

    harness.note(
        "newton_dynamic_obstacle_clearance",
        {"penetration_m": round(min(p.separation for p in hit_pairs if p.separation is not None), 5), "clearance_m": round(obs_cl[0].separation, 5)},
    )
    harness.stop(2)
    robot.close()


@harness.test
def test_newton_playon_variant_rebuild_and_shared_scene():
    """共有 GeometricScene での 2 台間干渉チェックと、VariantSet 切り替え後の自動再構築(Play=ON)を確かめる。"""
    stage = harness.new_stage(gravity=9.81)
    add_ur10e(stage, path="/World/R0", gripper="Robotiq_2F_85", position=(0.0, 0.0, 0.0))
    add_ur10e(stage, path="/World/R1", gripper="None", position=(1.2, 0.0, 0.0))

    scene = GeometricScene(stage)
    layer = GeometricLayer(stage)
    r0 = GeometricRobot.from_articulation(stage, "/World/R0", layer=layer, scene=scene, collision_backend="newton", apply_layer=False)
    r1 = GeometricRobot.from_articulation(stage, "/World/R1", layer=layer, scene=scene, collision_backend="newton", apply_layer=False)
    layer.apply()

    q_safe = np.zeros(r0.num_dofs)
    q_safe[1] = -np.pi / 2
    r0.set_dof_positions(q_safe)
    r1.set_dof_positions([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    harness.update(2)

    harness.play(3)
    assert scene.check_between(r0.link_set, r1.link_set) == []

    # R1 のベースを R0 と同じ場所へ動かすと between_robots 干渉が出る
    r1.set_world_poses(positions=[0.05, 0.0, 0.0])
    between = scene.check_between(r0.link_set, r1.link_set)
    assert between and all(p.kind == "between_robots" and p.separation is not None and p.separation <= 0.0 for p in between), between

    # Stop して Variant 切り替え -> GeometricScene が自動再構築され Play=ON でも正常動作
    harness.stop(2)
    r0.set_variant("Gripper", "None")
    assert r0.num_dofs == 6
    r0.set_variant("Gripper", "Robotiq_2F_140")
    assert r0.num_dofs == 14
    harness.play(3)
    between_after = scene.check_between(r0.link_set, r1.link_set)
    assert len(between_after) > 0
    r1.set_world_poses(positions=[2.0, 0.0, 0.0])
    assert scene.check_between(r0.link_set, r1.link_set) == []

    harness.stop(2)
    r0.close(remove_layer=False)
    r1.close(remove_layer=False)
    layer.remove()
    scene.close()


@harness.test
def test_newton_playon_view_batch_and_gpu_fabric():
    """GeometricRobotView の Newton GPU バッチ FK・0 フレーム遅延バッチ干渉判定・GeometricGpuFabricWriter を Play=ON で確かめる。"""
    stage = harness.new_stage(gravity=9.81)
    paths = [f"/World/UR_{i}" for i in range(4)]
    for i, p in enumerate(paths):
        add_ur10e(stage, path=p, position=(0.0, i * 2.0, 0.0))

    view = GeometricRobotView.from_articulations(
        stage,
        paths,
        write_target="fabric",
        collision_backend="newton",
        fk_backend="newton",
        use_gpu_fabric_writer=True,
    )
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])

    view.set_dof_positions(np.tile(q_safe, (4, 1)))
    view.flush_to_usd()
    harness.update(2)

    harness.play(3)
    # 0, 2 番は安全、1, 3 番は自己干渉姿勢へ一括ジャンプ(Fabric のみ、update 前の Frame 0)
    q_batch = np.stack([q_safe, q_self, q_safe, q_self])
    view.set_dof_positions(q_batch)

    batch_hits = view.check_collisions_batch("within")
    assert len(batch_hits) == 4
    assert len(batch_hits[0]) == 0 and len(batch_hits[2]) == 0, [len(h) for h in batch_hits]
    assert len(batch_hits[1]) == 5 and len(batch_hits[3]) == 5, [len(h) for h in batch_hits]

    batch_cl = view.compute_clearances_batch("within", max_distance=0.10)
    assert len(batch_cl) == 4 and len(batch_cl[1]) >= 5 and len(batch_cl[3]) >= 5

    # GPU Fabric 書き込み(omni:fabric:worldMatrix)が Play=ON の app.update() を越えて維持されることを確認
    import usdrt
    from pxr import UsdUtils

    expected = view.get_link_world_matrices()[:, view._prim_link_indices].reshape(-1, 4, 4)
    harness.update(3)
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    rt_stage = usdrt.Usd.Stage.Attach(stage_id)
    got_pos = []
    for p in view._prim_paths:
        rt_prim = rt_stage.GetPrimAtPath(usdrt.Sdf.Path(p))
        if rt_prim.HasAttribute("omni:fabric:worldMatrix"):
            mat = np.array(rt_prim.GetAttribute("omni:fabric:worldMatrix").Get(), dtype=float).reshape(4, 4).T
        else:
            mat = GeometricFabricWriter(stage, [p], batch=False).read_world()[0]
        got_pos.append(mat[:3, 3])
    got_pos_arr = np.asarray(got_pos)
    max_err = float(np.max(np.linalg.norm(got_pos_arr - expected[:, :3, 3], axis=1)))
    assert max_err < 1e-4, f"Play=ON での GPU Fabric 書き込み誤差が大きい: {max_err}"

    harness.note(
        "newton_playon_view_batch",
        {"hit_counts": [len(h) for h in batch_hits], "gpu_fabric_persist_err_m": round(max_err, 7)},
    )
    harness.stop(2)
    view.close()


harness.finish()
