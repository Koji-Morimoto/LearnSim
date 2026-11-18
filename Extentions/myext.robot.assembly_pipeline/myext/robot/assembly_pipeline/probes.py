"""症状を測る(事後確認)。同じ関数を、公式のままのステージ(Robot Assembler)と修正後のステージの両方に使う。"""

from __future__ import annotations

import math

import numpy as np
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from . import runtime
from .compose import JoinResult
from .sources import Owner
from .spec import ProbeSettings
from .urdf_model import UrdfModel, pose_matrix
from .usd_util import (
    ALL_PRIMS,
    articulation_roots,
    joint_prims,
    joint_targets,
    joints_under,
    link_prims,
    physics_variant_choice,
    pose_error,
    resolve_body,
    rigid,
    rigid_bodies,
    rigid_body_at_or_above,
    world,
    world_anchors,
)

LOG_KEYS = ("closed articulation", "nested articulation", "was skipped, since body1", "mimic", "articulation")


# ================================================================ 部品単体(静的)
def static_part(generated_usd: str, fixed_usd: str, model: UrdfModel, variant_hint: str) -> dict:
    """部品の USD を開いて調べる。公式のまま = 生成物そのまま、修正後 = 正規化層。"""
    out: dict = {}
    raw = Usd.Stage.Open(generated_usd)
    dp = raw.GetDefaultPrim()
    root = str(dp.GetPath())
    vs = dp.GetVariantSet("Physics") if dp.GetVariantSets().HasVariantSet("Physics") else None
    out["physics_variant_selection"] = vs.GetVariantSelection() if vs else None
    out["physics_variants"] = vs.GetVariantNames() if vs else []
    out["raw_rigid_bodies_as_is"] = len(rigid_bodies(raw, root))
    v = physics_variant_choice(dp, variant_hint)
    if vs and v:
        with Usd.EditContext(raw, raw.GetSessionLayer()):
            vs.SetVariantSelection(v)
    out["raw_variant_used"] = v
    # 剛体・関節・アンカー・Articulation Root
    out["anchors"] = world_anchors(raw, root)
    out["articulation_roots"] = articulation_roots(raw, root)
    out["newton_articulation_roots"] = [str(p.GetPath()) for p in Usd.PrimRange(dp, ALL_PRIMS) if "NewtonArticulationRootAPI" in p.GetAppliedSchemas()]
    links = link_prims(raw, root, list(model.links))
    out["link_prims"] = links
    out["rigid_link_names"] = [n for n, p in links.items() if raw.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI)]
    out["instanceable_links"] = [n for n, p in links.items() if raw.GetPrimAtPath(p).IsInstanceable()]
    out["links_without_body_above"] = [n for n, p in links.items() if rigid_body_at_or_above(raw, p) is None]
    # 末端リンクの座標ずれ:prim のワールド位置と URDF の順運動学
    cache = UsdGeom.XformCache()
    top = rigid(world(raw, root, cache))
    shifts = {}
    for name, path in links.items():
        if name not in model.links:
            continue
        err_m, err_deg = pose_error(top @ model.fk_zero(name), world(raw, path, cache))
        shifts[name] = (round(err_m, 6), round(err_deg, 4))
    out["link_frame_shift"] = shifts
    # mimic follower の drive
    jp = joint_prims(raw, root)
    gains = {}
    for follower, _leader in model.mimic_pairs():
        p = raw.GetPrimAtPath(jp.get(follower.name, ""))
        if not p:
            continue
        for kind in ("angular", "linear"):
            if p.HasAPI(UsdPhysics.DriveAPI, kind):
                d = UsdPhysics.DriveAPI(p, kind)
                gains[follower.name] = {"stiffness": float(d.GetStiffnessAttr().Get() or 0.0), "damping": float(d.GetDampingAttr().Get() or 0.0)}
    out["raw_follower_drive"] = gains
    # 衝突メッシュが見えるか:collider で purpose が guide でなく、同じリンクに同じ形の visual がある
    out["visible_duplicate_colliders"] = _visible_duplicates(raw, root)
    # 修正後(正規化層)
    fx = Usd.Stage.Open(fixed_usd)
    fdp = fx.GetDefaultPrim()
    out["fixed_rigid_bodies"] = len(rigid_bodies(fx, str(fdp.GetPath())))
    fjp = joint_prims(fx, str(fdp.GetPath()))
    fgains = {}
    for follower, _leader in model.mimic_pairs():
        p = fx.GetPrimAtPath(fjp.get(follower.name, ""))
        if not p:
            continue
        for kind in ("angular", "linear"):
            if p.HasAPI(UsdPhysics.DriveAPI, kind):
                d = UsdPhysics.DriveAPI(p, kind)
                fgains[follower.name] = {"stiffness": float(d.GetStiffnessAttr().Get() or 0.0), "damping": float(d.GetDampingAttr().Get() or 0.0)}
    out["fixed_follower_drive"] = fgains
    return out


def _visible_duplicates(stage: Usd.Stage, root: str) -> list[str]:
    gprims = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root), ALL_PRIMS) if p.IsA(UsdGeom.Gprim)]
    cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide])

    def key(p: Usd.Prim) -> tuple:
        r = bbox.ComputeWorldBound(p).ComputeAlignedRange()
        return tuple(round(v, 5) for v in list(r.GetMin()) + list(r.GetMax()))

    out = []
    visuals = {}
    for p in gprims:
        purpose = UsdGeom.Imageable(p).ComputePurpose()
        if not p.HasAPI(UsdPhysics.CollisionAPI) and purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
            visuals.setdefault(key(p), []).append(str(p.GetPath()))
    for p in gprims:
        purpose = UsdGeom.Imageable(p).ComputePurpose()
        if p.HasAPI(UsdPhysics.CollisionAPI) and purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
            if key(p) in visuals and str(p.GetPath()) not in visuals[key(p)]:
                out.append(str(p.GetPath()))
    return out


# ================================================================ mimic(部品単体を Play)
def probe_mimic(usd_path: str, model: UrdfModel, settings: ProbeSettings, variant_hint: str) -> dict:
    """leader を柔らかい drive で動かし、follower の drive の影響で leader が目標に届かないかを見る。"""
    pairs = model.mimic_pairs()
    if not pairs:
        return {}
    stage = runtime.new_probe_stage(gravity=0.0)
    path = "/Part"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    v = physics_variant_choice(prim, variant_hint)
    if v and not prim.GetVariantSet("Physics").GetVariantSelection():
        prim.GetVariantSet("Physics").SetVariantSelection(v)
    runtime.update(2)
    roots = articulation_roots(stage, path)  # スリープは既定のまま(follower の damping で遅くなると、眠って途中で止まる。これも症状)
    jp = joint_prims(stage, path)
    targets = {}
    for _follower, leader in pairs:
        p = stage.GetPrimAtPath(jp.get(leader.name, ""))
        if not p or leader.name in targets:
            continue
        lo, hi = (leader.lower, leader.upper) if leader.has_limit and leader.upper > leader.lower else (0.0, 0.5)
        t = lo + 0.75 * (hi - lo)
        targets[leader.name] = t
        if leader.type == "prismatic":
            d = UsdPhysics.DriveAPI.Apply(p, "linear")
            d.CreateTargetPositionAttr(t)
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_linear)
            d.CreateDampingAttr(settings.mimic_leader_damping_linear)
        else:
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
            d.CreateTargetPositionAttr(math.degrees(t))
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_angular)
            d.CreateDampingAttr(settings.mimic_leader_damping_angular)
    out: dict = {"targets": targets}
    with runtime.capture_warnings() as logs:
        runtime.timeline().play()
        runtime.update(2)
        q = _run_and_read(stage, roots[0] if roots else None, settings.mimic_steps, samples=(10, 30, 60))
        runtime.stop()
    out["logs"] = [m for m in logs if any(k in m.lower() for k in LOG_KEYS)]
    if q is None:
        out["error"] = "Articulation を読めなかった"
        return out
    names, values, series = q
    pos = dict(zip(names, values))
    for follower, leader in pairs:
        if follower.name not in pos or leader.name not in pos:
            continue
        expected = follower.mimic.multiplier * pos[leader.name] + follower.mimic.offset  # type: ignore[union-attr]
        out[follower.name] = {
            "leader": leader.name, "leader_target": targets.get(leader.name), "leader_final": pos[leader.name],
            "follower_final": pos[follower.name], "relation_error": abs(pos[follower.name] - expected),
            "leader_error": abs(pos[leader.name] - targets.get(leader.name, pos[leader.name])), "type": leader.type,
            "leader_series": {str(k): dict(zip(names, v))[leader.name] for k, v in series.items()}}
    return out


def _sim_view(stage: Usd.Stage):
    import omni.physics.tensors as tensors
    import omni.usd

    return tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())


def _run_and_read(stage: Usd.Stage, art_root: str | None, steps: int, samples: tuple[int, ...] = ()):
    if not art_root:
        runtime.update(steps)
        return None
    sv = _sim_view(stage)
    av = sv.create_articulation_view(art_root)
    if av is None:
        runtime.update(steps)
        return None
    names = list(av.shared_metatype.dof_names)  # Stop すると view が無効になるので先に読む
    series = {}
    for i in range(steps):
        runtime.update(1)
        if i + 1 in samples:
            series[i + 1] = [float(v) for v in av.get_dof_positions()[0]]
    return names, [float(v) for v in av.get_dof_positions()[0]], series


# ================================================================ 結合(Play と Stop 中の teleport)
def _root_art_root(stage: Usd.Stage, top: str, child_paths: list[str]) -> str | None:
    """結合した子の中にない Articulation Root(親の Articulation)。"""
    for r in articulation_roots(stage, top):
        if not any(r == cp or r.startswith(cp + "/") for cp in child_paths):
            return r
    return None


def _drive_targets(stage: Usd.Stage, top: str, owners: list[Owner], prefixes: set[str] | None,
                   settings: ProbeSettings) -> dict[str, tuple[str, float, str, str]]:
    """動かす関節:"前置き:関節名" -> (prim パス, 目標(rad か m), 種類, 前置き)。mimic の follower は除く。"""
    out = {}
    for o in owners:
        if (prefixes is not None and o.prefix not in prefixes) or not o.present(stage, top):
            continue
        followers = {f.name for f, _ in o.model.mimic_pairs()}
        for j in o.model.joints.values():
            if not j.movable or j.name in followers or j.name not in o.joint_rel:
                continue
            path = o.joint_path(top, j.name)
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsActive():
                continue
            if j.type == "prismatic":
                lo, hi = (j.lower, j.upper) if j.has_limit else (0.0, 0.1)
                t = lo + settings.prismatic_ratio * (hi - lo)
            else:
                t = settings.revolute_target
                if j.type == "revolute" and j.has_limit:
                    t = float(np.clip(t, 0.8 * j.lower, 0.8 * j.upper))
            out[f"{o.prefix}:{j.name}" if o.prefix else j.name] = (path, t, j.type, o.prefix)
    return out


def join_present(stage: Usd.Stage, top: str, res: JoinResult) -> bool:
    if not res.child_path or not stage.GetPrimAtPath(res.child_path):
        return False
    for rel, vs_name, choice in res.conditions:
        prim = stage.GetPrimAtPath(top if not rel else f"{top}/{rel}")
        vs = prim.GetVariantSets() if prim else None
        if not vs or not vs.HasVariantSet(vs_name) or vs.GetVariantSet(vs_name).GetVariantSelection() != choice:
            return False
    return True


def probe_stage(stage: Usd.Stage, top: str, owners: list[Owner], joins: list[JoinResult], settings: ProbeSettings,
                drive_prefixes: set[str] | None = None, include_offset: bool = True, with_poser: bool = True) -> dict:
    """ステージにある全部の結合を測る。drive_prefixes = 動かす関節の持ち主(None = 全部)。

    include_offset = False は公式のまま(Robot Assembler)のステージ用(Assembler には offset を指定する機能が無い)。
    """
    out: dict = {"joins": {}}
    live = [r for r in joins if join_present(stage, top, r)]
    child_paths = [r.child_path for r in live]  # type: ignore[misc]
    join_joints = tuple(r.joint_path for r in live if r.joint_path)
    out["articulation_roots"] = articulation_roots(stage, top)
    out["robot_anchors"] = world_anchors(stage, top)
    out["rigid_bodies"] = len(rigid_bodies(stage, top))
    out["present_joins"] = [r.jid for r in live]
    for r in live:
        info: dict = {}
        if r.parent_owner and r.child_owner and r.parent_link in r.parent_owner.model.links and r.child_link in r.child_owner.model.links:
            offset = pose_matrix(r.spec.offset_xyz, r.spec.offset_rpy) if include_offset else np.eye(4)
            want = rigid(world(stage, r.parent_owner.top(top))) @ r.parent_owner.model.fk_zero(r.parent_link) @ offset
            got = rigid(world(stage, r.child_owner.top(top))) @ r.child_owner.model.fk_zero(r.child_link)
            info["urdf_mount_alignment"] = pose_error(want, got)
        info["external_anchors"] = world_anchors(stage, r.child_path, scope=r.child_path, skip=join_joints)  # type: ignore[arg-type]
        info["articulation_roots_in_child"] = articulation_roots(stage, r.child_path)  # type: ignore[arg-type]
        if r.parent_body and r.child_body:
            pair = {r.parent_body, r.child_body}
            n = 0
            for p in joints_under(stage, top):
                if not p.IsA(UsdPhysics.FixedJoint) or not p.IsActive():
                    continue
                t0, t1 = joint_targets(p)
                b0, b1 = resolve_body(stage, t0), resolve_body(stage, t1)
                if b0 is not None and b1 is not None and {str(b0.GetPath()), str(b1.GetPath())} == pair:
                    n += 1
            info["fixed_joints_on_mount_pair"] = n
        if r.joint_path and stage.GetPrimAtPath(r.joint_path):
            _t0, t1 = joint_targets(stage.GetPrimAtPath(r.joint_path))
            b1 = resolve_body(stage, t1)
            info["joint_body1_resolved"] = str(b1.GetPath()) if b1 is not None else None
        out["joins"][r.jid] = info

    # ---- Play:関節を drive で動かし、関節の到達、結合のずれ、接触、ログを見る
    targets = _drive_targets(stage, top, owners, drive_prefixes, settings)
    for _key, (path, t, kind, _prefix) in targets.items():
        d = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(path), "linear" if kind == "prismatic" else "angular")
        d.CreateTargetPositionAttr(t if kind == "prismatic" else math.degrees(t))
        d.CreateStiffnessAttr(settings.drive_stiffness)
        d.CreateDampingAttr(settings.drive_damping)
    watch = [r for r in live if r.parent_body and r.child_body]
    child_bodies = {r.jid: set(rigid_bodies(stage, r.child_path)) for r in live}  # type: ignore[arg-type]
    parent_bodies = {}
    for r in live:
        o = r.parent_owner
        parent_bodies[r.jid] = {str(rb.GetPath()) for ln in (o.link_rel if o else {})
                                for rb in [rigid_body_at_or_above(stage, o.link_path(top, ln))] if rb is not None} if o else set()
    for bodies in child_bodies.values():
        for b in bodies:
            prim = stage.GetPrimAtPath(b)
            if not prim.IsInstanceProxy():
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    contacts: list[tuple[str, str]] = []

    def on_contact(headers, data):  # noqa: ANN001
        from pxr import PhysicsSchemaTools

        for h in headers:
            contacts.append((str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))))

    import omni.physx

    sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    art_root = _root_art_root(stage, top, child_paths)
    out["root_articulation"] = art_root
    with runtime.capture_warnings() as logs:
        runtime.timeline().play()
        runtime.update(2)
        sv = _sim_view(stage)
        av = sv.create_articulation_view(art_root) if art_root else None
        body_paths = list(dict.fromkeys(p for r in watch for p in (r.parent_body, r.child_body)))  # 同じ剛体が何度も出る(4 本のアームの親など)
        body_index = {p: i for i, p in enumerate(body_paths)}
        rb = sv.create_rigid_body_view(body_paths) if body_paths else None
        dof_paths = list(av.dof_paths[0]) if av is not None and hasattr(av, "dof_paths") else []
        names = list(av.shared_metatype.dof_names) if av is not None else []
        t0 = rb.get_transforms().copy() if rb is not None else None
        runtime.update(settings.steps)
        q = [float(v) for v in av.get_dof_positions()[0]] if av is not None else []
        t1 = rb.get_transforms().copy() if rb is not None else None
        runtime.stop()
    sub = None  # noqa: F841
    out["logs"] = [m for m in logs if any(k in m.lower() for k in LOG_KEYS)]
    out["dof_names"] = names
    by_path = dict(zip(dof_paths, q)) if dof_paths else {}
    by_name = dict(zip(names, q))
    reach = {}
    for key, (path, t, kind, prefix) in targets.items():
        v = by_path.get(path)
        if v is None and not dof_paths:
            v = by_name.get(key.split(":")[-1])
        if v is None:
            reach[key] = {"target": t, "final": None, "error": None, "kind": kind, "prefix": prefix, "note": "親の Articulation の DOF に無い"}
        else:
            reach[key] = {"target": t, "final": v, "error": abs(v - t), "kind": kind, "prefix": prefix}
    out["reach"] = reach
    for r in watch:
        if t0 is None or t1 is None or len(t0) != len(body_index):
            out["rigid_body_view_error"] = "剛体の view の数が合わない"
            break
        a, b = body_index[r.parent_body], body_index[r.child_body]
        m0 = [_pose_from_row(t0[a]), _pose_from_row(t0[b])]
        m1 = [_pose_from_row(t1[a]), _pose_from_row(t1[b])]
        out["joins"][r.jid]["mount_drift_during_play"] = pose_error(np.linalg.inv(m0[0]) @ m0[1], np.linalg.inv(m1[0]) @ m1[1])
    for r in live:
        joined = {(r.parent_body, r.child_body), (r.child_body, r.parent_body)} if r.parent_body and r.child_body else set()
        cb, pb = child_bodies[r.jid], parent_bodies[r.jid] - child_bodies[r.jid]
        hits = sorted({(x, y) for x, y in contacts if ((x in cb and y in pb) or (y in cb and x in pb)) and (x, y) not in joined})
        out["joins"][r.jid]["contacts_with_parent"] = hits
        out["joins"][r.jid]["closed_loop_logs"] = [m for m in out["logs"] if "closed articulation" in m.lower() and r.child_path in m]

    # ---- Stop 中の teleport(RobotPoser)
    if with_poser and watch and targets:
        try:
            from isaacsim.robot.poser.robot_poser import apply_joint_state

            from .usd_util import display_name

            before = {r.jid: (world(stage, r.parent_body), world(stage, r.child_body)) for r in watch}
            tops_before = {o.prefix: rigid(world(stage, o.top(top))) for o in owners if o.present(stage, top)}
            apply_joint_state(stage, stage.GetPrimAtPath(top), {p: t for p, t, _k, _x in targets.values()})
            runtime.update(1)
            for r in watch:
                b0, b1 = before[r.jid]
                n0, n1 = world(stage, r.parent_body), world(stage, r.child_body)
                info = out["joins"][r.jid]
                info["poser_drift"] = pose_error(np.linalg.inv(rigid(b0)) @ rigid(b1), np.linalg.inv(rigid(n0)) @ rigid(n1))
                info["poser_parent_moved_deg"] = pose_error(b0, n0)[1]
                # 親の剛体が URDF の順運動学どおりの姿勢に来たか(親のリンクの持ち主が一番上の URDF のときだけ。木の根を取り違えると土台ごと動く)
                o = r.parent_owner
                link = display_name(stage.GetPrimAtPath(r.parent_body))
                if o is not None and o.prefix == "" and link in o.model.links:
                    q_o = {key.split(":")[-1]: t for key, (_p, t, _k, prefix) in targets.items() if prefix == ""}
                    top_w = tops_before[""]
                    offset = np.linalg.inv(top_w @ o.model.fk_zero(link)) @ rigid(b0)  # prim と URDF のリンク座標のずれは打ち消す
                    info["poser_fk_error"] = pose_error(top_w @ o.model.fk(link, q_o) @ offset, rigid(n0))
        except Exception as exc:  # noqa: BLE001
            out["poser_error"] = f"{type(exc).__name__}: {exc}"
    return out


def _pose_from_row(row) -> np.ndarray:
    from pxr import Gf

    x, y, z, w = (float(v) for v in row[3:7])
    m = np.eye(4)
    m[:3, :3] = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(w, x, y, z)))).T
    m[:3, 3] = [float(v) for v in row[:3]]
    return m
