"""G02: Stop 中の update_simulation() は何をするか。kinematic 剛体だけのシーンで干渉を取れるか。

U-01  kinematic、Stop、USD に書く → update_simulation → PhysX の位置
U-02  kinematic、Stop、Fabric だけに書く → update_simulation → PhysX の位置
U-03  動的剛体(重力あり)、Stop、update_simulation を 30 回 → PhysX の位置 / USD の値 / update_transformations 後の USD の値
U-04  kinematic 2 個(兄弟並び、報告フラグあり)、Stop:
        最初から重なっている組 → overlap と Contact Report
        離れた位置から USD で重ねる → update_simulation → overlap と Contact Report
実行: run.ps1 g02_update_simulation.py   /   -ScriptArgs '--physx-fabric'
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code")
import bplib

bplib.RESULT_DIR = os.path.join(os.path.dirname(HERE), "results")
app = bplib.start(render=False)

from typing import Any

import carb
from omni.physx import get_physx_interface, get_physx_scene_query_interface, get_physx_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

rec = bplib.Recorder("g02_update_simulation")
ROBOT = "/World/Robot"
DT = 1.0 / 60.0


def link_path(i: int) -> str:
    return f"{ROBOT}/L{i}"


def make_link(stage: Usd.Stage, i: int, kind: str, pos: tuple[float, float, float], report: bool = False) -> None:
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
    if report:
        PhysxSchema.PhysxContactReportAPI.Apply(xf.GetPrim()).CreateThresholdAttr(0.0)


def new_stage(gravity: float = 0.0, flags: bool = False) -> Usd.Stage:
    stage = bplib.new_stage(gravity=gravity)
    UsdGeom.Xform.Define(stage, ROBOT)
    if flags:
        api = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/physicsScene"))
        api.CreateReportKinematicKinematicPairsAttr(True)
        api.CreateReportKinematicStaticPairsAttr(True)
    return stage


def load_physics() -> None:
    bplib.update(2)
    get_physx_interface().force_load_physics_from_usd()
    bplib.update(1)


def probe(pos: tuple[float, float, float], radius: float = 0.05) -> list[str]:
    hits: list[str] = []
    get_physx_scene_query_interface().overlap_sphere(radius, carb.Float3(*pos), lambda h: hits.append(str(h.collision)) or True, False)
    return sorted(hits)


def shape_hits(path: str) -> list[str]:
    hits: list[str] = []
    p0, p1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))
    get_physx_scene_query_interface().overlap_shape(p0, p1, lambda h: hits.append(str(h.collision)) or True, False)
    return sorted(h for h in hits if h != path)


POS = {"old_x0": (0.0, 0.0, 1.0), "usd_x2": (2.0, 0.0, 1.0), "fabric_x4": (4.0, 0.0, 1.0)}


def where() -> dict[str, bool]:
    col = link_path(0) + "/collider"
    return {k: col in probe(p) for k, p in POS.items()}


def step(n: int = 1) -> None:
    for i in range(n):
        get_physx_interface().update_simulation(DT, DT * i)


def physx_z(path: str) -> Any:
    t = get_physx_interface().get_rigidbody_transformation(path)
    return round(t["position"][2], 4) if t.get("ret_val") else None


def usd_z(stage: Usd.Stage, path: str) -> float:
    return round(stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Get()[2], 4)


@bplib.guarded(rec, "U-01")
def u01() -> None:
    stage = new_stage()
    make_link(stage, 0, "kinematic", POS["old_x0"])
    load_physics()
    out: dict[str, Any] = {"timeline": bplib.timeline_state(), "before": where()}
    bplib.usd_set_translate(stage, link_path(0), POS["usd_x2"])
    out["after_usd_write"] = where()
    bplib.update(2)
    out["after_2_app_updates"] = where()
    step(1)
    out["after_update_simulation_x1"] = where()
    out["timeline_after"] = bplib.timeline_state()
    rec.add("U-01", "kinematic、Stop、USD に書いてから update_simulation", **out)


@bplib.guarded(rec, "U-02")
def u02() -> None:
    stage = new_stage()
    make_link(stage, 0, "kinematic", POS["old_x0"])
    load_physics()
    out: dict[str, Any] = {"before": where()}
    bplib.fabric_set_translate(link_path(0), POS["fabric_x4"])
    bplib._fabric_hierarchy().update_world_xforms()
    step(1)
    out["after_fabric_write_and_update_simulation_x1"] = where()
    step(3)
    out["after_update_simulation_x4"] = where()
    bplib.update(2)
    step(1)
    out["after_2_app_updates_and_1_more_step"] = where()
    out["fabric_state"] = bplib.fabric_state(link_path(0))
    rec.add("U-02", "kinematic、Stop、Fabric だけに書いてから update_simulation", **out)


@bplib.guarded(rec, "U-03")
def u03() -> None:
    stage = new_stage(gravity=9.81)
    make_link(stage, 0, "rb", (0.0, 0.0, 5.0))
    load_physics()
    p = link_path(0)
    out: dict[str, Any] = {"timeline": bplib.timeline_state(), "physx_z_before": physx_z(p), "usd_z_before": usd_z(stage, p)}
    step(30)
    out["physx_z_after_30_steps"] = physx_z(p)
    out["usd_z_after_30_steps"] = usd_z(stage, p)
    out["expected_free_fall_z"] = round(5.0 - 0.5 * 9.81 * 0.5**2, 4)
    bplib.update(2)
    out["usd_z_after_2_app_updates"] = usd_z(stage, p)
    get_physx_interface().update_transformations(False, True, False)
    out["usd_z_after_update_transformations"] = usd_z(stage, p)
    out["root_layer_has_moved_value"] = stage.GetRootLayer().GetAttributeAtPath(p + ".xformOp:translate").default[2]
    out["timeline_after"] = bplib.timeline_state()
    rec.add("U-03", "動的剛体(重力 9.81)、Stop、update_simulation を 30 回(各 1/60 s)", **out)


class Contacts:
    def __init__(self) -> None:
        self.events: list[tuple[str, str, str]] = []
        self._sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on)

    def _on(self, headers: Any, data: Any) -> None:
        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            self.events.append((str(h.type).split(".")[-1], a, b))

    def take(self) -> list[str]:
        ev, self.events = self.events, []
        return [f"{t} {a.split('/')[-1]}-{b.split('/')[-1]}" for t, a, b in ev]


@bplib.guarded(rec, "U-04")
def u04() -> None:
    out: dict[str, Any] = {}
    # (a) 最初から重なっている kinematic 2 個
    stage = new_stage(flags=True)
    make_link(stage, 0, "kinematic", (0.0, 0.0, 1.0), report=True)
    make_link(stage, 1, "kinematic", (0.15, 0.0, 1.0), report=True)
    load_physics()
    c = Contacts()
    a: dict[str, Any] = {"overlap_L0_before_any_step": shape_hits(link_path(0) + "/collider")}
    step(3)
    bplib.update(1)
    a["contacts_after_3_steps_no_motion"] = c.take()
    a["overlap_L0_after_steps"] = shape_hits(link_path(0) + "/collider")
    bplib.usd_set_translate(stage, link_path(0), (0.01, 0.0, 1.0))
    step(1)
    bplib.update(1)
    a["contacts_after_moving_1cm_and_1_step"] = c.take()
    out["a_initially_overlapping"] = a
    c._sub = None

    # (b) 離れた位置から重ねる
    stage = new_stage(flags=True)
    make_link(stage, 0, "kinematic", (0.0, 0.0, 1.0), report=True)
    make_link(stage, 1, "kinematic", (1.0, 0.0, 1.0), report=True)
    load_physics()
    c = Contacts()
    b: dict[str, Any] = {"overlap_L0_apart": shape_hits(link_path(0) + "/collider")}
    bplib.usd_set_translate(stage, link_path(0), (0.9, 0.0, 1.0))
    b["overlap_L0_after_usd_write_no_step"] = shape_hits(link_path(0) + "/collider")
    b["probe_L1_side_sees_L0_no_step"] = (link_path(0) + "/collider") in probe((0.82, 0.0, 1.0), 0.01)
    step(1)
    b["contacts_right_after_1_step"] = c.take()
    b["overlap_L0_after_1_step"] = shape_hits(link_path(0) + "/collider")
    b["probe_L1_side_sees_L0_after_1_step"] = (link_path(0) + "/collider") in probe((0.82, 0.0, 1.0), 0.01)
    bplib.update(1)
    b["contacts_after_app_update"] = c.take()
    step(2)
    bplib.update(1)
    b["contacts_after_2_more_steps_no_motion"] = c.take()
    bplib.usd_set_translate(stage, link_path(0), (0.0, 0.0, 1.0))
    step(1)
    bplib.update(1)
    b["contacts_after_moving_away_1_step"] = c.take()
    out["b_moved_into_overlap"] = b
    c._sub = None

    # (c) 報告フラグなしの対照
    stage = new_stage(flags=False)
    make_link(stage, 0, "kinematic", (0.0, 0.0, 1.0), report=True)
    make_link(stage, 1, "kinematic", (1.0, 0.0, 1.0), report=True)
    load_physics()
    c = Contacts()
    bplib.usd_set_translate(stage, link_path(0), (0.9, 0.0, 1.0))
    step(1)
    bplib.update(1)
    out["c_no_flags"] = {"contacts": c.take(), "overlap_L0": shape_hits(link_path(0) + "/collider")}
    c._sub = None
    rec.add("U-04", "kinematic 剛体 2 個だけのシーン、Stop のまま update_simulation で干渉を取る", **out)


for fn in (u01, u02, u03, u04):
    fn()
bplib.finish(rec)
