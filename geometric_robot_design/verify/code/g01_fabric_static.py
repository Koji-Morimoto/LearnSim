"""G01: 幾何モードの設計判断に必要な実測(headless、GUI とは別プロセス)。

F 系  リンクの姿勢を Fabric だけに書いたとき、PhysX のシーンクエリ(overlap)に届くか
D 系  physics:rigidBodyEnabled=false を Sublayer に書くと、剛体が静的コライダーとして Stop 中に追従するか
T 系  リンク数 N ごとの書き込み時間(USD ループ / USD ChangeBlock / XformPrim usd / XformPrim fabric / Fabric 階層ループ)

実行: run.ps1 g01_fabric_static.py            (既定構成)
      run.ps1 g01_fabric_static.py --physx-fabric
"""

from __future__ import annotations

import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code")
import bplib

bplib.RESULT_DIR = os.path.join(os.path.dirname(HERE), "results")
app = bplib.start(render=False)

from typing import Any

import carb
import numpy as np
import omni.usd
import usdrt
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

rec = bplib.Recorder("g01_fabric_static")
ROBOT = "/World/Robot"
OBST = "/World/Obstacle"


def link_path(i: int) -> str:
    return f"{ROBOT}/L{i}"


def make_link(stage: Usd.Stage, i: int, kind: str, pos: tuple[float, float, float]) -> None:
    """リンク = Xform(translate/orient/scale) + 子の Cube コライダー(一辺 0.2)。kind: static / rb / kinematic"""
    xf = UsdGeom.Xform.Define(stage, link_path(i))
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xf.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    cube = UsdGeom.Cube.Define(stage, link_path(i) + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kind in ("rb", "kinematic"):
        rb = UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        if kind == "kinematic":
            rb.CreateKinematicEnabledAttr(True)


def make_obstacle(stage: Usd.Stage, pos: tuple[float, float, float]) -> None:
    cube = UsdGeom.Cube.Define(stage, OBST)
    cube.CreateSizeAttr(0.2)
    cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def build(kind: str, n: int = 1, gravity: float = 0.0) -> Usd.Stage:
    stage = bplib.new_stage(gravity=gravity)
    UsdGeom.Xform.Define(stage, ROBOT)
    for i in range(n):
        make_link(stage, i, kind, (0.0, float(i), 1.0))
    bplib.update(2)
    return stage


def load_physics() -> None:
    get_physx_interface().force_load_physics_from_usd()
    bplib.update(1)


def probe(pos: tuple[float, float, float], radius: float = 0.05) -> list[str]:
    """pos に置いた小球と重なるコライダーのパス(PhysX 側の位置を見る)。"""
    hits: list[str] = []

    def report(hit: Any) -> bool:
        hits.append(str(hit.collision))
        return True

    get_physx_scene_query_interface().overlap_sphere(radius, carb.Float3(*pos), report, False)
    return sorted(hits)


def shape_hits(path: str) -> list[str]:
    """path のコライダーを問い合わせ形状にした overlap_shape の相手(自分自身は除く)。"""
    hits: list[str] = []

    def report(hit: Any) -> bool:
        hits.append(str(hit.collision))
        return True

    p0, p1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))
    get_physx_scene_query_interface().overlap_shape(p0, p1, report, False)
    return sorted(h for h in hits if h != path)


def where(positions: dict[str, tuple[float, float, float]]) -> dict[str, bool]:
    """各候補位置に L0 のコライダーがいるか。"""
    col = link_path(0) + "/collider"
    return {k: col in probe(p) for k, p in positions.items()}


P_OLD = (0.0, 0.0, 1.0)
P_USD = (2.0, 0.0, 1.0)
P_FAB = (4.0, 0.0, 1.0)
POS = {"old_x0": P_OLD, "usd_x2": P_USD, "fabric_x4": P_FAB}


def fabric_write(pos: tuple[float, float, float]) -> None:
    bplib.fabric_set_translate(link_path(0), pos)
    bplib._fabric_hierarchy().update_world_xforms()


def xformprim_fabric_write(xp: Any, pos: tuple[float, float, float]) -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils

    with backend_utils.use_backend("fabric"):
        xp.set_world_poses(positions=np.array([pos], dtype=np.float32), orientations=np.array([[1, 0, 0, 0]], dtype=np.float32))


def after_steps() -> dict[str, Any]:
    out: dict[str, Any] = {"0_immediately": where(POS)}
    bplib.update(1)
    out["1_after_1_update"] = where(POS)
    bplib.update(3)
    out["2_after_4_updates"] = where(POS)
    return out


# ---------------------------------------------------------------- F 系


@bplib.guarded(rec, "F-00")
def f00() -> None:
    rec.add("F-00", "環境", **bplib.environment())


@bplib.guarded(rec, "F-01")
def f01() -> None:
    stage = build("static")
    make_obstacle(stage, (2.15, 0.0, 1.0))
    load_physics()
    out: dict[str, Any] = {"before": where(POS), "shape_before": shape_hits(link_path(0) + "/collider")}
    bplib.usd_set_translate(stage, link_path(0), P_USD)
    out["after_usd_write_immediately"] = where(POS)
    out["shape_after"] = shape_hits(link_path(0) + "/collider")
    rec.add("F-01", "対照: 静的コライダー、Stop、USD に書く", **out)


@bplib.guarded(rec, "F-02")
def f02() -> None:
    stage = build("static")
    make_obstacle(stage, (4.15, 0.0, 1.0))
    load_physics()
    out: dict[str, Any] = {"before": where(POS)}
    fabric_write(P_FAB)
    out.update(after_steps())
    out["shape_hits_L0"] = shape_hits(link_path(0) + "/collider")
    out["fabric_state"] = bplib.fabric_state(link_path(0))
    out["usd_x"] = stage.GetPrimAtPath(link_path(0)).GetAttribute("xformOp:translate").Get()[0]
    get_physx_interface().update_simulation(1.0 / 60.0, 0.0)
    out["3_after_update_simulation"] = where(POS)
    get_physx_interface().force_load_physics_from_usd()
    out["4_after_force_load"] = where(POS)
    rec.add("F-02", "静的コライダー、Stop、Fabric(IFabricHierarchy)だけに書く", **out)


@bplib.guarded(rec, "F-03")
def f03() -> None:
    from isaacsim.core.experimental.prims import XformPrim

    stage = build("static")
    xp = XformPrim([link_path(0)])
    bplib.update(3)
    load_physics()
    out: dict[str, Any] = {"before": where(POS)}
    xformprim_fabric_write(xp, P_FAB)
    out.update(after_steps())
    out["fabric_state"] = bplib.fabric_state(link_path(0))
    rec.add("F-03", "静的コライダー、Stop、XformPrim の fabric バックエンドで書く", **out)


def play_case(kind: str, writer: str) -> dict[str, Any]:
    stage = build(kind)
    bplib.play(3)
    out: dict[str, Any] = {"before": where(POS)}
    if writer == "fabric":
        fabric_write(P_FAB)
    else:
        bplib.usd_set_translate(stage, link_path(0), P_USD)
    out.update(after_steps())
    return out


@bplib.guarded(rec, "F-04")
def f04() -> None:
    rec.add("F-04", "静的コライダー、Play 中、Fabric だけに書く", **play_case("static", "fabric"))


@bplib.guarded(rec, "F-05")
def f05() -> None:
    rec.add("F-05", "kinematic 剛体、Play 中、Fabric だけに書く", **play_case("kinematic", "fabric"))


@bplib.guarded(rec, "F-06")
def f06() -> None:
    rec.add("F-06", "対照: kinematic 剛体、Play 中、USD に書く", **play_case("kinematic", "usd"))


@bplib.guarded(rec, "F-07")
def f07() -> None:
    rec.add("F-07", "対照: 静的コライダー、Play 中、USD に書く", **play_case("static", "usd"))


# ---------------------------------------------------------------- D 系


def session_sublayer(stage: Usd.Stage) -> Sdf.Layer:
    layer = Sdf.Layer.CreateAnonymous("geometric")
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    return layer


@bplib.guarded(rec, "D-01")
def d01() -> None:
    out: dict[str, Any] = {}
    for kind, disable in (("rb", True), ("kinematic", False), ("rb", False)):
        stage = build(kind)
        layer = session_sublayer(stage)
        if disable:
            with Usd.EditContext(stage, layer):
                UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(link_path(0))).CreateRigidBodyEnabledAttr(False)
        load_physics()
        r: dict[str, Any] = {"before": where(POS)}
        with Usd.EditContext(stage, layer):
            bplib.usd_set_translate(stage, link_path(0), P_USD)
        r["after_usd_write_immediately"] = where(POS)
        bplib.update(1)
        r["after_1_update"] = where(POS)
        out[f"{kind}{'+rigidBodyEnabled=false' if disable else ''}"] = r
    rec.add("D-01", "Stop 中に Sublayer へ姿勢を書いた直後、PhysX の位置が追従するか", **out)


def build_articulation(gravity: float = 9.81) -> Usd.Stage:
    """Base(世界に固定)と Link(Y 軸まわりのヒンジ、水平に伸びる)のアーティキュレーション。"""
    stage = bplib.new_stage(gravity=gravity)
    robot = UsdGeom.Xform.Define(stage, ROBOT)
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    make_link(stage, 0, "rb", (0.0, 0.0, 1.0))
    make_link(stage, 1, "rb", (0.5, 0.0, 1.0))
    fixed = UsdPhysics.FixedJoint.Define(stage, ROBOT + "/world_joint")
    fixed.CreateBody1Rel().SetTargets([link_path(0)])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 1))
    fixed.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    rev = UsdPhysics.RevoluteJoint.Define(stage, ROBOT + "/joint1")
    rev.CreateBody0Rel().SetTargets([link_path(0)])
    rev.CreateBody1Rel().SetTargets([link_path(1)])
    rev.CreateAxisAttr("Y")
    rev.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    rev.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    rev.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0, 0))
    rev.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    bplib.update(2)
    return stage


def link1_z(stage: Usd.Stage) -> float:
    m = UsdGeom.Xformable(stage.GetPrimAtPath(link_path(1))).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return round(m.ExtractTranslation()[2], 4)


@bplib.guarded(rec, "D-02")
def d02() -> None:
    stage = build_articulation()
    out: dict[str, Any] = {"z0": link1_z(stage)}
    bplib.play(40)
    out["1_original_play_40_updates_z"] = link1_z(stage)
    bplib.stop(2)
    out["1_after_stop_z"] = link1_z(stage)

    layer = session_sublayer(stage)
    with Usd.EditContext(stage, layer):
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROBOT)).CreateArticulationEnabledAttr(False)
        for j in ("world_joint", "joint1"):
            UsdPhysics.Joint(stage.GetPrimAtPath(f"{ROBOT}/{j}")).CreateJointEnabledAttr(False)
        for i in (0, 1):
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(link_path(i))).CreateRigidBodyEnabledAttr(False)
    bplib.update(2)
    load_physics()
    col1 = link_path(1) + "/collider"
    spots = {"home": (0.5, 0.0, 1.0), "moved": (0.0, 0.0, 1.5)}
    out["2_geo_stop_before"] = {k: col1 in probe(p) for k, p in spots.items()}
    with Usd.EditContext(stage, layer):
        bplib.usd_set_translate(stage, link_path(1), spots["moved"])
    out["2_geo_stop_after_write"] = {k: col1 in probe(p) for k, p in spots.items()}
    bplib.play(40)
    out["3_geo_play_40_updates_z"] = link1_z(stage)
    out["3_geo_play_probe"] = {k: col1 in probe(p) for k, p in spots.items()}
    bplib.stop(2)

    stage.GetSessionLayer().subLayerPaths.remove(layer.identifier)
    bplib.update(2)
    out["4_removed_z"] = link1_z(stage)
    out["4_removed_rigidBodyEnabled"] = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(link_path(1))).GetRigidBodyEnabledAttr().Get()
    bplib.play(40)
    out["4_removed_play_40_updates_z"] = link1_z(stage)
    bplib.stop(2)
    rec.add("D-02", "アーティキュレーションに Sublayer(articulation 無効 + joint 無効 + rigidBodyEnabled=false)を付け外し", **out)


# ---------------------------------------------------------------- T 系


def median_us(fn: Any, repeat: int = 15) -> float:
    samples = []
    for k in range(repeat):
        t0 = time.perf_counter()
        fn(k)
        samples.append(time.perf_counter() - t0)
    return round(statistics.median(samples) * 1e6, 1)


def timing(n: int) -> dict[str, Any]:
    import isaacsim.core.experimental.utils.backend as backend_utils
    from isaacsim.core.experimental.prims import XformPrim

    stage = build("static", n)
    paths = [link_path(i) for i in range(n)]
    cols = [p + "/collider" for p in paths]
    layer = session_sublayer(stage)
    prims = [stage.GetPrimAtPath(p) for p in paths]
    t_attrs = [p.GetAttribute("xformOp:translate") for p in prims]
    o_attrs = [p.GetAttribute("xformOp:orient") for p in prims]
    with Usd.EditContext(stage, layer):
        for i in range(n):
            t_attrs[i].Set(Gf.Vec3d(0, i, 1))
            o_attrs[i].Set(Gf.Quatd(1, 0, 0, 0))
    t_specs = [layer.GetAttributeAtPath(Sdf.Path(p).AppendProperty("xformOp:translate")) for p in paths]
    o_specs = [layer.GetAttributeAtPath(Sdf.Path(p).AppendProperty("xformOp:orient")) for p in paths]
    xp = XformPrim(paths)
    bplib.update(3)
    load_physics()
    h = bplib._fabric_hierarchy()
    rt_paths = [usdrt.Sdf.Path(p) for p in paths]
    quat = Gf.Quatd(1, 0, 0, 0)
    ori = np.tile(np.array([[1, 0, 0, 0]], dtype=np.float32), (n, 1))

    def pos_np(k: int) -> np.ndarray:
        a = np.zeros((n, 3), dtype=np.float32)
        a[:, 0] = 0.01 * (k + 1)
        a[:, 1] = np.arange(n)
        a[:, 2] = 1.0
        return a

    def usd_loop(k: int) -> None:
        with Usd.EditContext(stage, layer):
            for i in range(n):
                t_attrs[i].Set(Gf.Vec3d(0.01 * (k + 1), i, 1))
                o_attrs[i].Set(quat)

    def usd_block(k: int) -> None:
        with Sdf.ChangeBlock():
            for i in range(n):
                t_specs[i].default = Gf.Vec3d(0.012 * (k + 1), i, 1)
                o_specs[i].default = quat

    def usd_block_and_query(k: int) -> None:
        usd_block(k + 20)
        for c in cols:
            shape_hits(c)

    def query_only(k: int) -> None:
        for c in cols:
            shape_hits(c)

    def xp_usd(k: int) -> None:
        with Usd.EditContext(stage, layer), backend_utils.use_backend("usd"):
            xp.set_world_poses(positions=pos_np(k), orientations=ori)

    def xp_fabric(k: int) -> None:
        with backend_utils.use_backend("fabric"):
            xp.set_world_poses(positions=pos_np(k), orientations=ori)

    def fabric_loop(k: int) -> None:
        for i in range(n):
            m = usdrt.Gf.Matrix4d(1.0)
            m.SetTranslateOnly(usdrt.Gf.Vec3d(0.013 * (k + 1), i, 1))
            h.set_local_xform(rt_paths[i], m)

    def one_update(k: int) -> None:
        bplib.update(1)

    out: dict[str, Any] = {}
    out["usd_loop_us"] = median_us(usd_loop)
    out["update_after_usd_us"] = median_us(lambda k: (usd_block(k + 40), bplib.update(1))[1], 8)
    out["usd_changeblock_us"] = median_us(usd_block)
    out["query_only_us"] = median_us(query_only)
    out["usd_changeblock_plus_query_us"] = median_us(usd_block_and_query)
    out["xformprim_usd_us"] = median_us(xp_usd)
    out["xformprim_fabric_us"] = median_us(xp_fabric)
    import warp as wp

    wp_pos = [wp.array(pos_np(k), dtype=wp.float32, device="cpu") for k in range(15)]
    wp_ori = wp.array(ori, dtype=wp.float32, device="cpu")

    def xp_fabric_wp(k: int) -> None:
        with backend_utils.use_backend("fabric"):
            xp.set_world_poses(positions=wp_pos[k], orientations=wp_ori)

    out["xformprim_fabric_wp_input_us"] = median_us(xp_fabric_wp)

    def xp_fabric_local(k: int) -> None:
        with backend_utils.use_backend("fabric"):
            xp.set_local_poses(translations=wp_pos[k], orientations=wp_ori)

    def xp_fabric_local_np(k: int) -> None:
        with backend_utils.use_backend("fabric"):
            xp.set_local_poses(translations=pos_np(k), orientations=ori)

    out["xformprim_fabric_LOCAL_wp_input_us"] = median_us(xp_fabric_local)
    out["xformprim_fabric_LOCAL_np_input_us"] = median_us(xp_fabric_local_np)
    h.update_world_xforms()
    out["check_fabric_local_written_x"] = round(h.get_world_xform(rt_paths[n - 1]).ExtractTranslation()[0], 4)
    out["check_fabric_local_expected_x"] = round(0.01 * 15, 4)
    out["fabric_hierarchy_loop_us"] = median_us(fabric_loop)
    out["update_after_fabric_us"] = median_us(lambda k: (fabric_loop(k + 40), bplib.update(1))[1], 8)
    out["idle_update_us"] = median_us(one_update, 8)
    # 書いた値が PhysX に届いているかの確認(最後の usd 書き込み位置に L0 がいるか)
    usd_block(99)
    out["check_usd_follow"] = cols[0] in probe((0.012 * 100, 0.0, 1.0), 0.02)
    return out


@bplib.guarded(rec, "T-01")
def t01() -> None:
    for n in (10, 30, 100, 300, 1000):
        rec.add(f"T-01 N={n}", "N 個のリンク(静的コライダー、Stop、PhysX 読み込み済み)へ姿勢を書く時間の中央値 [µs]", **timing(n))


for fn in (f00, f01, f02, f03, f04, f05, f06, f07, d01, d02, t01):
    fn()
bplib.finish(rec)
