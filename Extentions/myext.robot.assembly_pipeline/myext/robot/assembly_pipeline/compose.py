"""結合ファイルを作る。毎回まるごと作り直し、中身が同じならファイルは書き換えない。

結合 1 回 = 次の 7 つの書き込み(Robot Assembler がしていることを、修正を入れて書いたもの。修正は規則ごとに skip できる):
  1. 親の usda の指定した prim の子として、子の usda を Reference で置く
     (省略時は親のリンクの prim の子 = stage の親子が実際のリンクの親子に揃う。その prim が instanceable なら外す = R05)
  2. 子の prim の Transform を、親のリンクと子のリンクの URDF の座標系(+ offset)が一致する値にする(R04。skip すると prim の位置で合わせる)
  3. 親と子の剛体をつなぐ固定関節を、決まったパス <子>/JoinJoint に 1 本作る(R14)。
     子のリンクに剛体が無ければ、fixed joint でつながる子孫の剛体に固定する(R08)
  4. 子のワールド固定 joint(子の外につながる joint)を名前でなく構造で探して無効にする(R02)
  5. 子の Articulation Root を Physics・Newton・Physx の全部外す(R07)
  6. 親の剛体と子の剛体の組の衝突を除外する(R11)
  7. 固定関節に IsaacJointAPI を付け、結合ファイルのロボットスキーマの関係の末尾に子を足し、
     無効にしたワールド固定 joint を子の robotJoints から消す(R12)

VariantSet: variant_set を持つ結合は、結合ファイルの一番上の prim の VariantSet の選択肢の中に 1〜7 を書く。
"""

from __future__ import annotations

import contextlib
import os
from dataclasses import dataclass, field

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .part_import import _write_if_changed, open_for_rewrite
from .sources import Owner, UsdaInfo, resolve
from .spec import JoinFileSpec, JoinSpec, PipelineSpec, join_id
from .urdf_model import pose_matrix
from .usd_util import (
    ARTICULATION_ROOT_APIS,
    ROBOT_JOINTS,
    ROBOT_LINKS,
    articulation_roots,
    joint_local,
    rigid,
    rigid_bodies,
    rigid_body_at_or_above,
    set_local_pose,
    world,
    world_anchors,
)

JOINT_NAME = "JoinJoint"


class CompositionError(RuntimeError):
    pass


@dataclass
class JoinResult:
    """結合 1 回の結果。公式のまま(Robot Assembler)と修正後で同じ形にして、測定と評価に使う。"""

    jid: str
    spec: JoinSpec
    child_path: str | None = None
    joint_path: str | None = None
    parent_body: str | None = None
    child_body: str | None = None
    parent_link_prim: str | None = None
    child_link_prim: str | None = None
    parent_owner: Owner | None = None  # 結合ファイルの中での親のリンクの持ち主(top からの相対)
    child_owner: Owner | None = None  # 同じく子のリンクの持ち主
    parent_link: str | None = None
    child_link: str | None = None
    conditions: tuple[tuple[str, str, str], ...] = ()  # この結合が現れる VariantSet の選択
    deactivated_anchors: list[str] = field(default_factory=list)
    remaining_anchors: list[str] = field(default_factory=list)
    removed_articulation_roots: list[str] = field(default_factory=list)
    uninstanced: list[str] = field(default_factory=list)
    body_from_fixed_descendant: str | None = None
    error: str | None = None


@dataclass
class FileComposition:
    name: str
    path: str
    top: str
    joins: list[JoinResult]
    info: UsdaInfo
    changed: bool


def _rel(path: str, start: str) -> str:
    r = os.path.relpath(path, start).replace("\\", "/")
    return r if r.startswith(".") else "./" + r


def _relpath(top: str, path: str) -> str:
    return path[len(top) + 1:] if path.startswith(top + "/") else ("" if path == top else path)


def compose_file(spec: PipelineSpec, f: JoinFileSpec, infos: dict[str, UsdaInfo]) -> FileComposition:
    os.makedirs(spec.output_dir, exist_ok=True)
    path = os.path.join(spec.output_dir, f.usda)
    layer = open_for_rewrite(path)
    layer.customLayerData = {"myext.robot.assembly_pipeline": {"join_file": f.usda}}
    stage = Usd.Stage.Open(layer)
    parent = infos[f.parent]
    src = Usd.Stage.Open(parent.path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src))
    top = f"/{f.stem}"
    top_prim = stage.DefinePrim(top, "Xform")
    top_prim.GetReferences().AddReference(_rel(parent.path, spec.output_dir))
    stage.SetDefaultPrim(top_prim)
    owners: list[Owner] = list(parent.owners)

    defaults: dict[str, str] = {}
    for j in f.joins:
        if j.variant_set:
            defaults.setdefault(j.variant_set, j.variant)  # type: ignore[arg-type]

    results: list[JoinResult] = []
    for j in f.joins:
        res = JoinResult(jid=join_id(f, j), spec=j)
        results.append(res)
        edit = contextlib.nullcontext()
        if j.variant_set:
            vset = top_prim.GetVariantSets().AddVariantSet(j.variant_set)
            if "None" not in vset.GetVariantNames():
                vset.AddVariant("None")
            vset.AddVariant(j.variant)
            for other_set, v in defaults.items():  # 書いている間だけ、ほかの VariantSet は既定の選択にする
                top_prim.GetVariantSet(other_set).SetVariantSelection(v)
            vset.SetVariantSelection(j.variant)
            edit = vset.GetVariantEditContext()
            res.conditions = (("", j.variant_set, j.variant),)  # type: ignore[assignment]
        try:
            with edit:
                new_owners = _join(stage, spec, top, owners, infos[j.child], j, res)
            owners.extend(new_owners)
        except CompositionError as exc:
            res.error = str(exc)
        except Exception as exc:  # noqa: BLE001
            res.error = f"{type(exc).__name__}: {exc}"

    for vs_name, v in defaults.items():
        top_prim.GetVariantSet(vs_name).SetVariantSelection(v)
    changed = _write_if_changed(layer, path)
    return FileComposition(name=f.name, path=path, top=top, joins=results,
                           info=UsdaInfo(name=f.name, path=path, official_path=None, owners=owners), changed=changed)


def resolve_prim(stage: Usd.Stage, top: str, owners: list[Owner], where: str) -> str:
    """parent_prim の書き方(リンクの書き方、"." = 一番上の prim、"/" で始まる結合ファイルの中のパス)を、ステージ上のパスにする。"""
    if where == ".":
        return top
    if where.startswith("/"):
        if not stage.GetPrimAtPath(where):
            raise CompositionError(f"prim {where} が無い")
        return where
    o, link = resolve(owners, where)
    return o.link_path(top, link)


def _join(stage: Usd.Stage, spec: PipelineSpec, top: str, owners: list[Owner], child: UsdaInfo, j: JoinSpec,
          res: JoinResult) -> list[Owner]:
    # ---- 親のリンク
    try:
        p_owner, p_link = resolve(owners, j.parent_link)
    except KeyError as exc:
        raise CompositionError(f"親のリンク: {exc}") from None
    if p_link not in p_owner.link_rel:
        raise CompositionError(f"親のリンク {j.parent_link!r} の prim が見つからない")
    res.parent_owner, res.parent_link = p_owner, p_link
    res.parent_link_prim = p_owner.link_path(top, p_link)

    # ---- 1. 子を Reference で置く
    place = resolve_prim(stage, top, owners, j.parent_prim) if j.parent_prim else res.parent_link_prim
    place_prim = stage.GetPrimAtPath(place)
    if place_prim.IsInstanceable():
        if not spec.fix_applied("R05"):
            raise CompositionError(f"R05: 子を置く prim {place} が instanceable なので子を作れない(修正を skip)")
        place_prim.SetInstanceable(False)
        res.uninstanced.append(place)
    child_path = f"{place}/{j.prim}"
    res.child_path = child_path
    cp = stage.DefinePrim(child_path, "Xform")
    cp.GetReferences().AddReference(_rel(child.path, spec.output_dir))
    for vs_name, choice in j.select:
        vs = cp.GetVariantSets()
        if not vs.HasVariantSet(vs_name):
            raise CompositionError(f"子 {j.child} に VariantSet {vs_name!r} が無い")
        vs.GetVariantSet(vs_name).SetVariantSelection(choice)
    rel = _relpath(top, child_path)
    new_owners = [o.under(j.name, rel, res.conditions) for o in child.owners]
    try:
        # 子のリンクは子の usda の中での書き方(前置きなし = 子の一番上の URDF)で解決し、結合ファイルの中の持ち主に置き換える
        c_src, c_link = resolve(child.owners, j.child_link)
    except KeyError as exc:
        raise CompositionError(f"子のリンク: {exc}") from None
    c_owner = new_owners[child.owners.index(c_src)]
    if c_link not in c_owner.link_rel:
        raise CompositionError(f"子のリンク {j.child_link!r} の prim が見つからない")
    res.child_owner, res.child_link = c_owner, c_link
    res.child_link_prim = c_owner.link_path(top, c_link)

    # ---- 2. 位置合わせ
    set_local_pose(cp, np.eye(4))
    offset = pose_matrix(j.offset_xyz, j.offset_rpy)
    if spec.fix_applied("R04"):
        target = rigid(world(stage, p_owner.top(top))) @ p_owner.model.fk_zero(p_link) @ offset
        mount_in_child = (np.linalg.inv(rigid(world(stage, child_path))) @ rigid(world(stage, c_owner.top(top)))
                          @ c_owner.model.fk_zero(c_link))
    else:
        # 修正なし:Robot Assembler と同じく prim の位置どうしを合わせる
        target = rigid(world(stage, res.parent_link_prim)) @ offset
        mount_in_child = np.linalg.inv(rigid(world(stage, child_path))) @ rigid(world(stage, res.child_link_prim))
    set_local_pose(cp, np.linalg.inv(rigid(world(stage, place))) @ target @ np.linalg.inv(mount_in_child))

    # ---- 3 の準備:固定する剛体
    b0 = rigid_body_at_or_above(stage, res.parent_link_prim)
    if b0 is None:
        raise CompositionError(f"親のリンク {res.parent_link_prim} の位置にも上にも剛体が無い")
    b1 = rigid_body_at_or_above(stage, res.child_link_prim)
    if b1 is not None and not str(b1.GetPath()).startswith(child_path + "/"):
        b1 = None
    if b1 is None and spec.fix_applied("R08"):
        for d in c_owner.model.fixed_descendants(c_link):
            if d not in c_owner.link_rel:
                continue
            rb = rigid_body_at_or_above(stage, c_owner.link_path(top, d))
            if rb is not None and str(rb.GetPath()).startswith(child_path + "/"):
                b1 = rb
                res.body_from_fixed_descendant = d
                break
    body1_target = str(b1.GetPath()) if b1 is not None else res.child_link_prim  # 修正なし:剛体の無い prim をそのまま使う
    res.parent_body, res.child_body = str(b0.GetPath()), (str(b1.GetPath()) if b1 is not None else None)

    # ---- 4. 子のワールド固定 joint
    for anc in world_anchors(stage, child_path, scope=child_path):
        if spec.fix_applied("R02") or not anc["both_targets_authored"]:
            # 修正なしでも、片側が空の joint は Robot Assembler と同じく無効にする
            stage.GetPrimAtPath(anc["path"]).SetActive(False)
            res.deactivated_anchors.append(anc["path"])
            if spec.fix_applied("R12"):
                # RobotPoser は active = false の joint もロボットスキーマの robotJoints から木に入れるので、関係からも消す
                for p in Usd.PrimRange(cp, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
                    r = p.GetRelationship(ROBOT_JOINTS) if p.HasAPI("IsaacRobotAPI") else None
                    if r and Sdf.Path(anc["path"]) in r.GetTargets():
                        r.RemoveTarget(Sdf.Path(anc["path"]))
    res.remaining_anchors = [x["path"] for x in world_anchors(stage, child_path, scope=child_path)]

    # ---- 5. 子の Articulation Root
    for p in articulation_roots(stage, child_path):
        prim = stage.GetPrimAtPath(p)
        if prim.IsInstanceProxy():
            continue
        for t in ARTICULATION_ROOT_APIS if spec.fix_applied("R07") else ("PhysicsArticulationRootAPI",):
            prim.RemoveAPI(t)
        res.removed_articulation_roots.append(p)

    # ---- 3. 固定関節
    jpath = f"{child_path}/{JOINT_NAME}"
    joint = UsdPhysics.FixedJoint.Define(stage, jpath)
    joint.CreateBody0Rel().SetTargets([b0.GetPath()])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_target)])
    cache = UsdGeom.XformCache()
    w0 = rigid(world(stage, b0.GetPath(), cache))
    w1 = rigid(world(stage, body1_target, cache))
    mount_world = target if spec.fix_applied("R04") else rigid(world(stage, res.child_link_prim, cache))
    p0, q0 = joint_local(np.linalg.inv(w0) @ target)
    p1, q1 = joint_local(np.linalg.inv(w1) @ mount_world)
    joint.CreateLocalPos0Attr(p0)
    joint.CreateLocalRot0Attr(q0)
    joint.CreateLocalPos1Attr(p1)
    joint.CreateLocalRot1Attr(q1)
    res.joint_path = jpath

    # ---- 7. ロボットスキーマ
    if spec.fix_applied("R12"):
        joint.GetPrim().ApplyAPI("IsaacJointAPI")
    top_prim = stage.GetPrimAtPath(top)
    for name in (ROBOT_LINKS, ROBOT_JOINTS):
        r = top_prim.GetRelationship(name) or top_prim.CreateRelationship(name, custom=False)
        # 末尾に足す(先頭だと RobotPoser の木の根が子のリンクになり、teleport で親が動いてしまう)
        position = Usd.ListPositionBackOfAppendList if spec.fix_applied("R12") else Usd.ListPositionBackOfPrependList
        r.AddTarget(Sdf.Path(child_path), position)

    # ---- 6. 衝突除外
    if spec.fix_applied("R11"):
        r = UsdPhysics.FilteredPairsAPI.Apply(b0).CreateFilteredPairsRel()
        for rb in rigid_bodies(stage, child_path):
            r.AddTarget(Sdf.Path(rb))
    return new_owners
