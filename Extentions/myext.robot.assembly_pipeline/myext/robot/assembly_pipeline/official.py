"""公式のまま:結合 1 回ごとに、親と子の URDF Importer の生成物を Robot Assembler で実際に結合したステージを作る。

事前チェックで「○○が起きる」と判定したことが、公式の部品をそのまま使ったときに本当に起きるかを、ここで測る。
公式の修正が入れば、ここでは症状が出なくなる。

測れるのは、親のリンクが親の usda の一番上の URDF のリンクで、子のリンクも子の usda の一番上の URDF のリンクの結合
(リンクの書き方に "/" が無いもの)。そのとき、親・子それぞれの一番上の URDF の生成物どうしを結合する
(子が結合ファイルでも、その中の結合はここでは再現しない)。
"""

from __future__ import annotations

from dataclasses import dataclass

from pxr import Usd, UsdGeom, UsdPhysics

from . import runtime
from .compose import JoinResult
from .part_import import PartBuild
from .sources import Owner
from .spec import JoinFileSpec, JoinSpec, PipelineSpec, join_id
from .usd_util import joint_prims, joints_under, link_prims, physics_variant_choice, rigid_body_at_or_above

ROOT = "/World"


@dataclass
class OfficialStage:
    stage: Usd.Stage
    top: str  # 親の prim(Robot Assembler に渡す base)
    owners: list[Owner]  # 親("")と子(結合の名前)
    join: JoinResult


def measurable(j: JoinSpec) -> str | None:
    """公式のままを測れないなら、その理由。"""
    if "/" in j.parent_link or "/" in j.child_link:
        return "親か子のリンクが、結合した子の中のリンク(\"/\" を含む書き方)なので、公式のままでは再現しない"
    if j.parent_prim and j.parent_prim != "." and (j.parent_prim.startswith("/") or "/" in j.parent_prim):
        return "parent_prim がパスか、結合した子の中のリンクなので、公式のままでは再現しない"
    return None


def _add(stage: Usd.Stage, path: str, build: PartBuild) -> Usd.Prim:
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(build.generated_usd)
    v = physics_variant_choice(prim, build.spec.physics_variant)
    if v:
        prim.GetVariantSet("Physics").SetVariantSelection(v)
    return prim


def _owner(stage: Usd.Stage, top: str, build: PartBuild, prefix: str) -> Owner:
    links = link_prims(stage, top, list(build.model.links))
    joints = joint_prims(stage, top)
    rel = lambda p: p[len(top) + 1:] if p.startswith(top + "/") else p  # noqa: E731
    return Owner(prefix=prefix, rel_top="", model=build.model, link_rel={k: rel(v) for k, v in links.items()},
                 joint_rel={k: rel(v) for k, v in joints.items() if v.startswith(top + "/")}, urdf_name=build.spec.name)


def build_official(spec: PipelineSpec, f: JoinFileSpec, j: JoinSpec, builds: dict[str, PartBuild]) -> OfficialStage:
    from isaacsim.robot_setup.assembler import RobotAssembler

    parent_b = builds[spec.root_urdf(f.parent).name]
    child_b = builds[spec.root_urdf(j.child).name]
    stage = runtime.new_probe_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, ROOT).GetPrim())  # base を default prim にしない = 直接編集ではない結合
    top = f"{ROOT}/{f.name}"
    _add(stage, top, parent_b)
    runtime.update(2)
    p_owner = _owner(stage, top, parent_b, "")
    res = JoinResult(jid=join_id(f, j), spec=j, parent_owner=p_owner, parent_link=j.parent_link)
    out = OfficialStage(stage=stage, top=top, owners=[p_owner], join=res)
    if j.parent_link not in p_owner.link_rel:
        res.error = f"親のリンク {j.parent_link!r} が無い"
        return out
    res.parent_link_prim = p_owner.link_path(top, j.parent_link)
    if j.parent_prim == ".":
        place = top
    elif j.parent_prim and j.parent_prim in p_owner.link_rel:
        place = p_owner.link_path(top, j.parent_prim)
    else:
        place = res.parent_link_prim
    child_path = f"{place}/{j.prim}"
    try:
        _add(stage, child_path, child_b)
    except Exception as exc:  # noqa: BLE001
        res.error = f"子を置けない: {type(exc).__name__}: {exc}"
        return out
    runtime.update(2)
    res.child_path = child_path
    rel = child_path[len(top) + 1:]
    c_owner = _owner(stage, child_path, child_b, "").under(j.name, rel)
    out.owners.append(c_owner)
    res.child_owner, res.child_link = c_owner, j.child_link
    if j.child_link in c_owner.link_rel:
        res.child_link_prim = c_owner.link_path(top, j.child_link)
    b0 = rigid_body_at_or_above(stage, res.parent_link_prim)
    b1 = rigid_body_at_or_above(stage, res.child_link_prim) if res.child_link_prim else None
    res.parent_body = str(b0.GetPath()) if b0 else None
    res.child_body = str(b1.GetPath()) if b1 else None
    asm = RobotAssembler()
    try:
        asm.begin_assembly(stage, top, res.parent_link_prim, child_path, res.child_link_prim or "", j.variant_set or "Join", j.name)
        asm.assemble()
        runtime.run_async(asm.finish_assemble())
        runtime.update(3)
    except Exception as exc:  # noqa: BLE001
        res.error = f"{type(exc).__name__}: {str(exc).strip()[:500]}"
        try:
            asm.cancel_assembly()
            runtime.update(3)
        except Exception:  # noqa: BLE001
            pass
    res.joint_path = next((str(p.GetPath()) for p in joints_under(stage, child_path) if p.GetName().startswith("AssemblerFixedJoint")), None)
    return out


def reassemble_count(o: OfficialStage) -> int | None:
    """R14:同じ結合をもう一度したとき、Robot Assembler の固定関節が何本になるか。"""
    from isaacsim.robot_setup.assembler import RobotAssembler

    res = o.join
    if res.error or not res.child_path or not res.child_link_prim:
        return None
    try:
        asm = RobotAssembler()
        asm.begin_assembly(o.stage, o.top, res.parent_link_prim, res.child_path, res.child_link_prim, res.spec.variant_set or "Join", res.spec.name)
        asm.assemble()
        runtime.run_async(asm.finish_assemble())
        runtime.update(3)
    except Exception:  # noqa: BLE001
        pass
    return len([p for p in joints_under(o.stage, res.child_path) if p.IsA(UsdPhysics.FixedJoint) and p.GetName().startswith("AssemblerFixedJoint")])
