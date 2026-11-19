"""パイプライン全体:URDF の判定 → import → usda の判定 → 結合ファイルを順に作る → 症状の確認 → 幾何モード → 報告。

    from myext.robot.assembly_pipeline import load_spec, run_pipeline
    result = run_pipeline(load_spec("robot.toml"))
    print(result.report["status"])

headless の Isaac Sim(SimulationApp)から呼ぶ。判定で止めると PipelineStopped(報告は書いてある)。
"""

from __future__ import annotations

import os
from dataclasses import dataclass

from pxr import Usd

from . import probes, runtime
from .compose import FileComposition, compose_file
from .official import build_official, measurable, reassemble_count
from .part_import import PartBuild, build_part
from .report import write_report
from .rules import Context, Finding, detect, evaluate
from .sources import Owner, UsdaInfo
from .spec import PipelineSpec, join_id
from .urdf_model import load_urdf
from .usd_util import joint_prims, link_prims, reload_layers_under


class PipelineStopped(RuntimeError):
    def __init__(self, message: str, report: dict):
        super().__init__(message)
        self.report = report


@dataclass
class PipelineResult:
    spec: PipelineSpec
    outputs: dict[str, str]  # usda の名前 -> 修正後の usda のパス
    report: dict


def _stoppers(spec: PipelineSpec, findings: list[Finding]) -> list[str]:
    return [f"[{f.rule}] {f.target}: {f.reason}" for f in findings if spec.on_violation_of(f.rule) == "stop"]


def urdf_info(b: PartBuild) -> UsdaInfo:
    """URDF から作った usda の地図(一番上の URDF の部分だけ)。"""
    st = Usd.Stage.Open(b.fixed_usd)
    top = str(st.GetDefaultPrim().GetPath())
    rel = lambda p: p[len(top) + 1:] if p.startswith(top + "/") else p  # noqa: E731
    links = {k: rel(v) for k, v in link_prims(st, top, list(b.model.links)).items()}
    joints = {k: rel(v) for k, v in joint_prims(st, top).items()}
    return UsdaInfo(name=b.spec.name, path=b.fixed_usd, official_path=b.generated_usd,
                    owners=[Owner(prefix="", rel_top="", model=b.model, link_rel=links, joint_rel=joints, urdf_name=b.spec.name)])


def run_pipeline(spec: PipelineSpec, log=print) -> PipelineResult:
    from .spec_check import check_links, describe

    models = {u.name: load_urdf(u.path) for u in spec.urdfs}
    problems = check_links(spec, models)
    if problems:
        raise ValueError("仕様の誤り:\n" + "\n".join(f"- {p}" for p in problems))
    description = describe(spec)
    _enable_extensions()
    ctx = Context(spec=spec, models=models)

    # ---- 1. URDF だけで判定できるもの(import の前)
    findings = detect(ctx, "urdf")
    stop = _stoppers(spec, findings)
    if stop:
        rep = write_report(spec.output_dir, status="stopped_before_import", findings=findings, outcomes=[], extra={"stopped_by": stop, "spec_description": description})
        raise PipelineStopped("URDF の判定で止めた:\n" + "\n".join(stop), rep)

    # ---- 2. import と正規化層
    infos: dict[str, UsdaInfo] = {}
    for u in spec.urdfs:
        b = build_part(spec, u, ctx.models[u.name])
        ctx.builds[u.name] = b
        log(f"[import] {u.name}: {'import した' if b.imported else '入力が同じなので飛ばした'} -> {b.generated_usd}")
    reload_layers_under(spec.output_dir)
    for u in spec.urdfs:
        b = ctx.builds[u.name]
        ctx.statics[u.name] = probes.static_part(b.generated_usd, b.fixed_usd, b.model, u.physics_variant)
        infos[u.name] = urdf_info(b)

    # ---- 3. usda で判定(結合ファイルを作る前)。URDF の段階の判定も残す(同じ規則・対象は後のものを使う)
    later = detect(ctx, "usd")
    seen = {(f.rule, f.target) for f in later}
    findings = [f for f in findings if (f.rule, f.target) not in seen] + later
    stop = _stoppers(spec, findings)
    if stop:
        rep = write_report(spec.output_dir, status="stopped_before_join", findings=findings, outcomes=[], extra={"stopped_by": stop, "spec_description": description})
        raise PipelineStopped("usda の判定で止めた:\n" + "\n".join(stop), rep)

    # ---- 4. 結合ファイルを依存の順に作る(2 回作って、2 回目に中身が変わらないことも確かめる)
    comps: dict[str, FileComposition] = {}
    for f in spec.file_order():
        c1 = compose_file(spec, f, infos)
        reload_layers_under(spec.output_dir)
        c2 = compose_file(spec, f, infos)
        reload_layers_under(spec.output_dir)
        comps[f.name] = c2
        infos[f.name] = c2.info
        ctx.recompose_changed[f.name] = c2.changed
        for r in c2.joins:
            ctx.fixed_join[r.jid] = r
            if r.error:
                log(f"[join] {r.jid}: {r.error}")
        log(f"[join_file] {f.name}: {c2.path}(1 回目 {'書き換えた' if c1.changed else '変化なし'}、2 回目 {'書き換えた' if c2.changed else '変化なし'})")

    # ---- 5. 症状の確認:mimic(URDF ごと。公式のまま / 修正後)
    for u in spec.urdfs:
        b = ctx.builds[u.name]
        if b.model.mimic_pairs():
            ctx.mimic_official[u.name] = probes.probe_mimic(b.generated_usd, b.model, spec.probe, u.physics_variant)
            ctx.mimic_fixed[u.name] = probes.probe_mimic(b.fixed_usd, b.model, spec.probe, u.physics_variant)

    # ---- 6. 症状の確認:公式のまま(結合ごとに Robot Assembler で結合)
    if spec.compare_official:
        for f, j in spec.all_joins():
            jid = join_id(f, j)
            why = measurable(j)
            if why:
                ctx.official_skip[jid] = why
                continue
            o = build_official(spec, f, j, ctx.builds)
            ctx.official_join[jid] = o.join
            ctx.official[jid] = probes.probe_stage(o.stage, o.top, o.owners, [o.join], spec.probe, drive_prefixes={""},
                                                   include_offset=False)
            ctx.official_reassemble[jid] = reassemble_count(o)
        log("[official] Robot Assembler での結合と測定を終えた")

    # ---- 7. 症状の確認:修正後(結合ファイルごと。VariantSet の選択肢を 1 つずつ)
    for name, c in comps.items():
        for selection in _selections(spec, name):
            stage = runtime.new_probe_stage()
            prim = stage.DefinePrim(c.top, "Xform")
            prim.GetReferences().AddReference(c.path)
            for vs_name, choice in selection.items():
                prim.GetVariantSet(vs_name).SetVariantSelection(choice)
            runtime.update(2)
            result = probes.probe_stage(stage, c.top, c.info.owners, c.joins, spec.probe)
            for jid in result["present_joins"]:
                if jid not in ctx.fixed:
                    ctx.fixed[jid] = result
            log(f"[fixed] {name} {selection or ''}: 測った結合 {result['present_joins']}")

    # ---- 8. 幾何モード(結合ファイルごと、既定の選択)
    geometric = {}
    if spec.check_geometric:
        from .geometric_bridge import check_against_physx

        for name, c in comps.items():
            geometric[name] = check_against_physx(c.path, c.top.lstrip("/"))
            g = geometric[name]
            log(f"[geometric] {name}: {g.get('error') or 'リンク位置の最大差 %s m' % g.get('max_link_position_error_m')}")
    runtime.new_probe_stage()  # 測定用ステージを閉じる

    # ---- 9. 評価と報告
    outcomes = evaluate(ctx, findings)
    bad = [o for o in outcomes if o.verdict in ("still_broken", "unexpected")]
    geo_bad = {n: g for n, g in geometric.items() if g.get("error") or not g.get("dof_names_match")
               or (g.get("max_link_position_error_m") or 0.0) > spec.probe.length_tol_m}
    for n, g in geo_bad.items():
        log(f"[geometric] {n}: 幾何モードと PhysX が食い違う: {g}")
    status = "ok" if not bad and not geo_bad else "problems"
    outputs = {**{n: b.fixed_usd for n, b in ctx.builds.items()}, **{n: c.path for n, c in comps.items()}}
    rep = write_report(spec.output_dir, status=status, findings=findings, outcomes=outcomes, extra={
        "spec_description": description, "geometric": geometric, "outputs": outputs,
        "join_errors": {r.jid: r.error for c in comps.values() for r in c.joins if r.error},
        "measurements": {"statics": ctx.statics, "mimic_official": ctx.mimic_official, "mimic_fixed": ctx.mimic_fixed,
                         "official": ctx.official, "fixed": ctx.fixed, "official_reassemble": ctx.official_reassemble,
                         "official_skip": ctx.official_skip,
                         "official_join_errors": {k: v.error for k, v in ctx.official_join.items()}},
    })
    for o in outcomes:
        log(o.message)
    return PipelineResult(spec=spec, outputs=outputs, report=rep)


def _selections(spec: PipelineSpec, name: str) -> list[dict[str, str]]:
    """結合ファイルの VariantSet の選び方:既定の選択 1 つと、既定でない選択肢を 1 つずつ(ほかは既定)。"""
    f = spec.join_file(name)
    sets: dict[str, list[str]] = {}
    for j in f.joins:  # type: ignore[union-attr]
        if j.variant_set:
            sets.setdefault(j.variant_set, []).append(j.variant)  # type: ignore[arg-type]
    out: list[dict[str, str]] = [{}]
    for vs_name, choices in sets.items():
        for c in choices[1:]:
            out.append({vs_name: c})
    return out


def _enable_extensions() -> None:
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    for e in ("isaacsim.asset.importer.urdf", "isaacsim.robot_setup.assembler", "isaacsim.robot.poser", "omni.physx.tensors"):
        if not mgr.is_extension_enabled(e):
            mgr.set_extension_enabled_immediate(e, True)
    here = os.path.dirname(os.path.abspath(__file__))
    ext_root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))  # .../Extentions
    if os.path.isdir(os.path.join(ext_root, "myext.robot.geometric")):
        mgr.add_path(ext_root)
    runtime.update(2)
