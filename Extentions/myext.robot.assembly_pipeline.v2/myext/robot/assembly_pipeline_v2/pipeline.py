"""ロボット結合パイプライン全体のオーケストレーションを担うエントリポイントモジュール。

責務と 2 つの公開エントリの役割分担:
  - ``build_assembly(spec, *, check_idempotency=False, log=print)``（非破壊ビルド）:
    Step 1〜4 を実行する。URDF ロード・静的仕様検査・事前チェック（URDF / USD）・部品 import と正規化・
    結合ファイルの依存順生成・構成要素ディスクリプタ抽出のみを行い、グローバルステージや Timeline を
    一切触らない。そのため GUI モード（``--gui``）や ``--build-only``、外部スクリプトから安全に呼べる。
  - ``run_pipeline(spec, log=print)``（フル検証実行）:
    ``build_assembly(check_idempotency=True)`` を実行したうえで、Step 5〜9 の動的検証
    （mimic プローブ → 公式 ``RobotAssembler`` 再現比較 → 修正後ステージの VariantSet 別プローブ →
    純粋 USD 順運動学 vs PhysX 検証 → R01〜R14 最終評価と ``report.json`` / ``report.md`` 出力）まで一括実行する。

全体フロー（9 ステップ）:
  1. URDF 事前チェック（``rules.detect(ctx, "urdf")``）
  2. 部品の import・正規化（``part_import.build_part`` / ``build_usd_part``）と静的検査・USD リンク検証
  3. USD 事前チェック（``rules.detect(ctx, "usd")``）
  4. 結合ファイルを依存順に構築（``compose.compose_file``）し、構成要素ディスクリプタを抽出
  5. mimic 動的プローブ（``probes.probe_mimic`` / ``probe_mimic_usd``）
  6. 公式 ``RobotAssembler`` 再現比較（``official.probe_official``、``spec.compare_official`` が真のとき）
  7. 修正後結合ファイルの VariantSet 切替プローブ（``probes.probe_stage``）
  8. 純粋 USD 順運動学検証（``kinematics_check.check_against_physx``）
  9. R01〜R14 評価（``rules.evaluate_all``）とレポート書き出し（``report.write_report``）

使用例:
    from myext.robot.assembly_pipeline_v2 import load_spec, run_pipeline, build_assembly
    build_info = build_assembly(load_spec("robot.toml"))
    result = run_pipeline(load_spec("robot.toml"))
    print(result.report["status"])

前提:
  Isaac Sim（``SimulationApp``）起動後のプロセスから呼ぶ。``on_violation = "stop"`` の規則が
  事前チェックで検出された場合は、その時点のレポートを書き出してから ``PipelineStopped`` を送出する。

主な公開 API:
  - ``PipelineStopped``: 事前チェックによる意図的な停止例外（``report`` 属性付き）
  - ``PipelineResult``: 辞書兼属性アクセス可能な ``run_pipeline`` の戻り値
  - ``urdf_info(b)`` / ``usd_info(b)``: 正規化済み部品 ``PartBuild`` から ``UsdaInfo`` 地図を作成
  - ``build_assembly(...)``: 非破壊な USD アセンブリ構築（Step 1〜4）
  - ``run_pipeline(...)``: 構築から動的検証・レポート出力までのフル実行（Step 1〜9）

手を入れやすい箇所:
  - 新しい事後検証フェーズを追加したい場合は ``run_pipeline`` の Step 8〜9 周辺へ組み込み、
    ``write_report`` の ``extra`` に結果を追加する。
"""

from __future__ import annotations

import os
import sys
from typing import Any

from pxr import Usd

from . import probes, runtime
from .compose import FileComposition, compose_file
from .descriptor import AssembledRobotDescriptor, inspect_assembled_robot
from .official import probe_official
from .part_import import PartBuild, build_part, build_usd_part
from .report import write_report
from .rules import Context, Finding, detect, evaluate_all
from .sources import Owner, UsdaInfo
from .spec import JoinFileSpec, PipelineSpec
from .urdf_model import load_urdf
from .usd_util import joint_prims, link_prims, reload_layers_under, usd_link_prims


class PipelineStopped(RuntimeError):
    """事前チェックの規則に ``on_violation = "stop"`` が設定され、パイプラインを途中停止した際の例外。

    ``self.report`` に停止時点のレポート辞書（``report.json`` と同一内容）を保持する。
    """

    def __init__(self, message: str, report: dict):
        super().__init__(message)
        self.report = report


class PipelineResult(dict):
    """``run_pipeline`` の実行結果。辞書キーとしても属性（``.spec`` / ``.outputs`` / ``.report`` / ``.descriptors``）としても参照できる。"""

    def __init__(
        self,
        spec: PipelineSpec,
        outputs: dict[str, str],
        report: dict[str, Any],
        descriptors: dict[str, AssembledRobotDescriptor] | None = None,
        **extra: Any,
    ) -> None:
        descriptors = descriptors or {}
        super().__init__(
            spec=spec,
            outputs=outputs,
            report=report,
            descriptors=descriptors,
            **extra,
        )
        self.spec = spec
        self.outputs = outputs
        self.report = report
        self.descriptors = descriptors


def _stoppers(spec: PipelineSpec, findings: list[Finding]) -> list[str]:
    """事前チェックの ``findings`` のうち ``on_violation == "stop"`` に該当する停止理由メッセージ一覧を返す。"""
    return [f"[{f.rule}] {f.target}: {f.reason}" for f in findings if spec.on_violation_of(f.rule) == "stop"]


def urdf_info(b: PartBuild) -> UsdaInfo:
    """URDF 部品の正規化層 ``b.fixed_usd`` を開き、リンク・関節の相対パス地図（``UsdaInfo``）を構築する。副作用なし。"""
    st = Usd.Stage.Open(str(b.fixed_usd))
    top = str(st.GetDefaultPrim().GetPath())
    rel = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else p  # noqa: E731
    links = {k: rel(v) for k, v in link_prims(st, top, list(b.model.links)).items()}
    joints = {k: rel(v) for k, v in joint_prims(st, top).items()}
    return UsdaInfo(
        name=b.spec.name,
        path=str(b.fixed_usd),
        official_path=str(b.generated_usd),
        owners=[Owner(prefix="", rel_top="", model=b.model, link_rel=links, joint_rel=joints, urdf_name=b.spec.name)],
    )


def usd_info(b: PartBuild) -> UsdaInfo:
    """USD 部品の正規化層 ``b.fixed_usd`` を開き、``model=None`` の ``UsdaInfo`` 地図を構築する。副作用なし。"""
    st = Usd.Stage.Open(str(b.fixed_usd))
    top = str(st.GetDefaultPrim().GetPath())
    rel = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else ("" if p == top else p)  # noqa: E731
    links = {k: rel(v) for k, v in usd_link_prims(st, top).items()}
    joints = {k: rel(v) for k, v in joint_prims(st, top).items()}
    return UsdaInfo(
        name=b.spec.name,
        path=str(b.fixed_usd),
        official_path=str(b.generated_usd),
        owners=[Owner(prefix="", rel_top="", model=None, link_rel=links, joint_rel=joints, urdf_name=b.spec.name)],
    )


def _owner_has_link(owner: Owner, link: str) -> bool:
    fn = getattr(owner, "has_link", None)
    if fn is not None:
        return bool(fn(link))
    rfn = getattr(owner, "resolve_link_rel", None)
    if rfn is not None:
        return rfn(link) is not None
    return link in owner.link_rel or link in owner.link_rel.values()


def _validate_usd_links(spec: PipelineSpec, infos: dict[str, UsdaInfo]) -> None:
    """USD 部品を参照する結合のリンク名・配置先・子 VariantSet 選択を結合開始前に検証する。

    URDF 部品は ``spec_check.check_links`` で XML ベースに事前検証できるが、USD 部品は実際に
    USD ステージを開かないとリンク名や VariantSet 候補が分からないため、Step 2 の正規化直後にここで検証する。
    """
    if not getattr(spec, "usds", ()):
        return
    problems: list[str] = []
    for f, j in spec.all_joins():
        where = f"[[join_file]] usda = {f.usda!r} の [[join_file.join]] name = {j.name!r}"

        # 1. child_variant_selection (子が USD 部品のとき)
        child_stage: Usd.Stage | None = None
        child_dp: Usd.Prim | None = None
        if j.select and spec.usd(j.child) is not None and j.child in infos:
            child_stage = Usd.Stage.Open(infos[j.child].path)
            child_dp = child_stage.GetDefaultPrim() if child_stage else None
            vs = child_dp.GetVariantSets() if child_dp and child_dp.IsValid() else None
            for vs_name, choice in j.select:
                if vs is None or not vs.HasVariantSet(vs_name):
                    avail = vs.GetNames() if vs is not None else []
                    problems.append(f"{where}: child_usda = {j.child!r} に VariantSet {vs_name!r} が無い (候補: {avail})")
                else:
                    choices = vs.GetVariantSet(vs_name).GetVariantNames()
                    if choice not in choices:
                        problems.append(
                            f"{where}: child_usda = {j.child!r} の VariantSet {vs_name!r} に選択肢 {choice!r} が無い (候補: {choices})"
                        )

        # 2. parent_link
        p_usda, p_link = spec.resolve_link(f.parent, j.parent_link)
        if spec.usd(p_usda) is not None and p_usda in infos:
            owner = infos[p_usda].owners[0]
            if not _owner_has_link(owner, p_link):
                problems.append(
                    f"{where}: parent_link = {j.parent_link!r} が {p_usda!r} に見つからない (候補: {sorted(owner.link_rel)})"
                )

        # 3. child_link
        c_usda, c_link = spec.resolve_link(j.child, j.child_link)
        if spec.usd(c_usda) is not None and c_usda in infos:
            owner = infos[c_usda].owners[0]
            found = _owner_has_link(owner, c_link)
            if not found and j.select and c_usda == j.child and child_stage is not None and child_dp and child_dp.IsValid():
                # child_variant_selection 適用後に現れるリンクを確認する
                vs = child_dp.GetVariantSets()
                with Usd.EditContext(child_stage, child_stage.GetSessionLayer()):
                    for vs_name, choice in j.select:
                        if vs.HasVariantSet(vs_name) and choice in vs.GetVariantSet(vs_name).GetVariantNames():
                            vs.GetVariantSet(vs_name).SetVariantSelection(choice)
                    top = str(child_dp.GetPath())
                    rel = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else ("" if p == top else p)  # noqa: E731
                    sel_links = {k: rel(v) for k, v in usd_link_prims(child_stage, top).items()}
                    found = c_link in sel_links or c_link in sel_links.values()
            if not found:
                problems.append(
                    f"{where}: child_link = {j.child_link!r} が {c_usda!r} に見つからない (候補: {sorted(owner.link_rel)})"
                )

        # 4. place_child_under
        if j.parent_prim and j.parent_prim != "." and not j.parent_prim.startswith("/"):
            u_usda, u_link = spec.resolve_link(f.parent, j.parent_prim)
            if spec.usd(u_usda) is not None and u_usda in infos:
                owner = infos[u_usda].owners[0]
                if not _owner_has_link(owner, u_link):
                    problems.append(
                        f"{where}: place_child_under = {j.parent_prim!r} が {u_usda!r} に見つからない (候補: {sorted(owner.link_rel)})"
                    )

    if problems:
        raise ValueError("USD 仕様の誤り:\n" + "\n".join(f"- {p}" for p in problems))


def _normalize_composition(f: JoinFileSpec, spec: PipelineSpec, raw: Any) -> FileComposition:
    if isinstance(raw, FileComposition):
        return raw
    if isinstance(raw, dict):
        return FileComposition(
            name=raw.get("name", f.name),
            path=str(raw.get("path", os.path.join(str(spec.output_dir), f.usda))),
            top=raw.get("top", f"/{f.stem}"),
            joins=list(raw.get("joins", [])),
            info=raw["info"],
            changed=bool(raw.get("changed", False)),
        )
    return raw


def _call_compose_file(spec: PipelineSpec, f: JoinFileSpec, infos: dict[str, UsdaInfo]) -> FileComposition:
    raw = compose_file(spec, f, infos)
    return _normalize_composition(f, spec, raw)


def _call_static_part(b: PartBuild, u: Any) -> dict[str, Any]:
    try:
        return probes.static_part(str(b.generated_usd), str(b.fixed_usd), b.model, u.physics_variant)
    except TypeError:
        return probes.static_part(b.fixed_usd, b.model, True)


def build_assembly(
    spec: PipelineSpec,
    *,
    check_idempotency: bool = False,
    log=print,
) -> dict[str, Any]:
    """非破壊な USD アセンブリ構築（パイプライン Step 1〜4）。

    静的リンク検査（``spec_check.check_links``）・URDF 読み込み・事前チェック（URDF / USD）・
    部品の import と正規化（``build_part`` / ``build_usd_part``）・静的部品検査（``probes.static_part``）・
    結合ファイルの依存順生成（``compose_file``）・構成要素ディスクリプタ抽出（``inspect_assembled_robot``）
    を実行する。

    ``run_pipeline`` との役割分担:
      この関数はグローバルステージ（``runtime.new_probe_stage``）や Timeline を一切変更しない。
      そのため GUI ビューアに結合済みロボットを読み込む前段ビルド（``--build-only`` / ``--gui``）にもそのまま使える。

    Args:
        spec: 読み込み済みの ``PipelineSpec``。
        check_idempotency: ``True`` のとき各結合ファイルを 2 回連続で生成し、2 回目で差分が出ないか
            （R14 冪等性）を ``ctx.recompose_changed`` に記録する。
        log: 進捗ログ出力用コールバック（既定は ``print``）。

    Returns:
        ``spec``・``description``・``models``・``builds``・``statics``・``infos``・``compositions``・
        ``outputs``・``descriptors``・``findings``・``context`` を格納した辞書。

    Raises:
        ValueError: 仕様上のリンク名・VariantSet 名等に不整合がある場合。
        PipelineStopped: 事前チェックで ``on_violation = "stop"`` の規則に抵触した場合。

    副作用:
        ``spec.output_dir`` 配下に正規化 ``.usda``・結合 ``.usda``・``_import/`` 生成物を書き出し、
        必要な Kit 拡張を有効化し、該当ディレクトリ配下の SdfLayer キャッシュをリロードする。
    """
    from .spec_check import check_links, describe

    models = {u.name: load_urdf(u.path) for u in spec.urdfs}
    try:
        problems = check_links(spec, models)
    except TypeError:
        problems = check_links(spec)
    if problems:
        raise ValueError("仕様の誤り:\n" + "\n".join(f"- {p}" for p in problems))
    raw_desc = describe(spec)
    description = raw_desc.splitlines() if isinstance(raw_desc, str) else list(raw_desc)
    _enable_extensions()
    ctx = Context(spec=spec, models=models)

    # ---- 1. URDF だけで判定できるもの(import の前)
    findings = detect(ctx, "urdf")
    stop = _stoppers(spec, findings)
    if stop:
        rep = write_report(
            str(spec.output_dir),
            status="stopped_before_import",
            findings=findings,
            outcomes=[],
            extra={"stopped_by": stop, "spec_description": description},
        )
        raise PipelineStopped("URDF の判定で止めた:\n" + "\n".join(stop), rep)

    # ---- 2. import と正規化層
    infos: dict[str, UsdaInfo] = {}
    for u in spec.urdfs:
        b = build_part(spec, u, ctx.models[u.name])
        ctx.builds[u.name] = b
        log(f"[import] {u.name}: {'import した' if b.imported else '入力が同じなので飛ばした'} -> {b.generated_usd}")
    for s in getattr(spec, "usds", ()):
        b = build_usd_part(spec, s)
        ctx.builds[s.name] = b
        log(f"[usd] {s.name}: 正規化 -> {b.fixed_usd}")
    reload_layers_under(str(spec.output_dir))
    for u in spec.urdfs:
        b = ctx.builds[u.name]
        ctx.statics[u.name] = _call_static_part(b, u)
        infos[u.name] = urdf_info(b)
    for s in getattr(spec, "usds", ()):
        b = ctx.builds[s.name]
        ctx.statics[s.name] = _call_static_part(b, s)
        infos[s.name] = usd_info(b)
    _validate_usd_links(spec, infos)

    # ---- 3. usda で判定(結合ファイルを作る前)。URDF の段階の判定も残す(同じ規則・対象は後のものを使う)
    later = detect(ctx, "usd")
    seen = {(f.rule, f.target) for f in later}
    findings = [f for f in findings if (f.rule, f.target) not in seen] + later
    stop = _stoppers(spec, findings)
    if stop:
        rep = write_report(
            str(spec.output_dir),
            status="stopped_before_join",
            findings=findings,
            outcomes=[],
            extra={"stopped_by": stop, "spec_description": description},
        )
        raise PipelineStopped("usda の判定で止めた:\n" + "\n".join(stop), rep)

    # ---- 4. 結合ファイルを依存の順に作る
    comps: dict[str, FileComposition] = {}
    descriptors: dict[str, AssembledRobotDescriptor] = {}
    for f in spec.file_order():
        c1 = _call_compose_file(spec, f, infos)
        reload_layers_under(str(spec.output_dir))
        if check_idempotency:
            c2 = _call_compose_file(spec, f, infos)
            reload_layers_under(str(spec.output_dir))
            c_final = c2
            ctx.recompose_changed[f.name] = c2.changed
            log(
                f"[join_file] {f.name}: {c2.path}"
                f"(1 回目 {'書き換えた' if c1.changed else '変化なし'}、2 回目 {'書き換えた' if c2.changed else '変化なし'})"
            )
        else:
            c_final = c1
            ctx.recompose_changed[f.name] = False
            log(f"[join_file] {f.name}: {c1.path}({'書き換えた' if c1.changed else '変化なし'})")
        comps[f.name] = c_final
        infos[f.name] = c_final.info
        for r in c_final.joins:
            ctx.fixed_join[r.jid] = r
            if r.error:
                log(f"[join] {r.jid}: {r.error}")
        st = Usd.Stage.Open(str(c_final.path))
        descriptors[f.name] = inspect_assembled_robot(st, c_final.top, usda_path=str(c_final.path))

    outputs = {
        **{n: str(b.fixed_usd) for n, b in ctx.builds.items()},
        **{n: str(c.path) for n, c in comps.items()},
    }
    return {
        "spec": spec,
        "description": description,
        "models": models,
        "builds": ctx.builds,
        "statics": ctx.statics,
        "infos": infos,
        "compositions": comps,
        "outputs": outputs,
        "descriptors": descriptors,
        "findings": findings,
        "context": ctx,
    }


def run_pipeline(spec: PipelineSpec, log=print) -> PipelineResult:
    """構築から動的プローブ・公式比較・順運動学検証・レポート書き出しまでを一括実行する（Step 1〜9）。

    内部で ``build_assembly(spec, check_idempotency=True, log=log)`` を呼んだ後、
    使い捨てのプローブ用ステージ上で以下を実行する:
      - Step 5: URDF / USD 部品の mimic 動的プローブ（R03）
      - Step 6: 公式 ``RobotAssembler`` 再現比較（``spec.compare_official`` 有効時）
      - Step 7: 修正後結合ファイルの VariantSet 切替プローブ
      - Step 8: 純粋 USD 順運動学 vs PhysX 突き合わせ検証（``spec.check_kinematics`` 有効時）
      - Step 9: 全規則 R01〜R14 の評価と ``report.json`` / ``report.md`` の書き出し

    Args:
        spec: 読み込み済みの ``PipelineSpec``。
        log: 進捗ログ出力用コールバック（既定は ``print``）。

    Returns:
        ``spec``・``outputs``・``report``・``descriptors`` を持つ ``PipelineResult``。

    副作用:
        ``build_assembly`` のファイル書き出しに加え、グローバル USD コンテキストのステージを
        プローブごとに作り直して Timeline を Play / Stop し、最後に ``report.json`` / ``report.md`` を出力する。
    """
    built = build_assembly(spec, check_idempotency=True, log=log)
    ctx: Context = built["context"]
    comps: dict[str, FileComposition] = built["compositions"]
    findings: list[Finding] = built["findings"]
    description: list[str] = built["description"]
    outputs: dict[str, str] = built["outputs"]
    descriptors: dict[str, AssembledRobotDescriptor] = built["descriptors"]

    # ---- 5. 症状の確認: mimic(URDF / USD ごと。公式のまま / 修正後)
    for u in spec.urdfs:
        b = ctx.builds[u.name]
        if b.model.mimic_pairs():
            try:
                ctx.mimic_official[u.name] = probes.probe_mimic(str(b.generated_usd), b.model, spec.probe, u.physics_variant)
                ctx.mimic_fixed[u.name] = probes.probe_mimic(str(b.fixed_usd), b.model, spec.probe, u.physics_variant)
            except TypeError:
                m_res = probes.probe_mimic(spec, u, b.model)
                if isinstance(m_res, dict) and "official" in m_res and "fixed" in m_res:
                    ctx.mimic_official[u.name] = m_res["official"]
                    ctx.mimic_fixed[u.name] = m_res["fixed"]
                else:
                    ctx.mimic_official[u.name] = m_res
                    ctx.mimic_fixed[u.name] = m_res
    for s in getattr(spec, "usds", ()):
        st = ctx.statics.get(s.name, {})
        raw_drives = st.get("raw_follower_drive", {})
        if st.get("mimic_followers") and any(g.get("stiffness") or g.get("damping") for g in raw_drives.values()):
            b = ctx.builds[s.name]
            ctx.mimic_official[s.name] = probes.probe_mimic_usd(
                str(b.generated_usd),
                spec.probe,
                s.physics_variant,
                prim_path=s.prim_path,
                variant_selections=s.variant_selections,
            )
            ctx.mimic_fixed[s.name] = probes.probe_mimic_usd(str(b.fixed_usd), spec.probe, s.physics_variant)

    # ---- 6. 症状の確認: 公式のまま(結合ごとに Robot Assembler で結合)
    if spec.compare_official:
        # 公式 RobotAssembler の既知不具合を意図的に再現する区間では、PhysX が
        # [Error] [omni.physx.plugin] ... closed articulation 等のエラーを端末へ大量出力する。
        # ユーザーが「意図的な比較ログ」と「本当の異常」を端末上で即座に見分けられるよう境界バナーを出し、
        # C++ 側出力と順序が入れ替わらないよう stdout をフラッシュする。
        log(
            "[INFO] === 公式 RobotAssembler 比較フェーズ開始"
            "（この区間の PhysX / closed articulation エラーは公式不具合の意図的な再現ログです） ==="
        )
        sys.stdout.flush()
        try:
            for f in spec.file_order():
                c = comps[f.name]
                off_res = probe_official(spec, f, c.info, ctx.builds)
                ctx.official.update(off_res.get("official", {}))
                ctx.official_join.update(off_res.get("official_join", {}))
                ctx.official_skip.update(off_res.get("official_skip", {}))
                ctx.official_reassemble.update(off_res.get("official_reassemble", {}))
            log("[official] Robot Assembler での結合と測定を終えた")
        finally:
            log("[INFO] === 公式 RobotAssembler 比較フェーズ終了（ここ以降のエラーは意図しない異常です） ===")
            sys.stdout.flush()

    # ---- 7. 症状の確認: 修正後(結合ファイルごと。VariantSet の選択肢を 1 つずつ)
    for name, c in comps.items():
        for selection in _selections(spec, name):
            stage = runtime.new_probe_stage()
            prim = stage.DefinePrim(c.top, "Xform")
            prim.GetReferences().AddReference(str(c.path))
            for vs_name, choice in selection.items():
                prim.GetVariantSet(vs_name).SetVariantSelection(choice)
            runtime.step(2)
            try:
                result = probes.probe_stage(stage, c.top, c.info.owners, c.joins, spec.probe)
            except TypeError:
                result = probes.probe_stage(spec, spec.join_file(name), c.info)
            for jid in result.get("present_joins", []):
                if jid not in ctx.fixed:
                    ctx.fixed[jid] = result
            log(f"[fixed] {name} {selection or ''}: 測った結合 {result.get('present_joins', [])}")

    # ---- 8. 順運動学検証(結合ファイルごと、既定の選択)
    kin_results: dict[str, dict[str, Any]] = {}
    check_kin = getattr(spec, "check_kinematics", getattr(spec, "check_geometric", True))
    if check_kin:
        from .kinematics_check import check_against_physx

        for name, c in comps.items():
            f_spec = spec.join_file(name)
            try:
                kin_results[name] = check_against_physx(spec, f_spec, c.info)
            except TypeError:
                kin_results[name] = check_against_physx(str(c.path), c.top.lstrip("/"))
            g = kin_results[name]
            log(f"[kinematics] {name}: {g.get('error') or 'リンク位置の最大差 %s m' % g.get('max_link_position_error_m')}")
    runtime.new_probe_stage()  # 測定用ステージを閉じる

    # ---- 9. 評価と報告
    outcomes = evaluate_all(ctx, findings)
    bad = [o for o in outcomes if o.verdict in ("still_broken", "unexpected")]
    kin_bad = {
        n: g
        for n, g in kin_results.items()
        if g.get("error")
        or not g.get("dof_names_match")
        or (g.get("max_link_position_error_m") or 0.0) > spec.probe.length_tol_m
    }
    for n, g in kin_bad.items():
        log(f"[kinematics] {n}: 順運動学と PhysX が食い違う: {g}")
    status = "ok" if not bad and not kin_bad else "problems"
    rep = write_report(
        str(spec.output_dir),
        status=status,
        findings=findings,
        outcomes=outcomes,
        extra={
            "spec_description": description,
            "kinematics": kin_results,
            "geometric": kin_results,
            "outputs": outputs,
            "descriptors": {k: v.to_dict() for k, v in descriptors.items()},
            "join_errors": {r.jid: r.error for c in comps.values() for r in c.joins if r.error},
            "measurements": {
                "statics": ctx.statics,
                "mimic_official": ctx.mimic_official,
                "mimic_fixed": ctx.mimic_fixed,
                "official": ctx.official,
                "fixed": ctx.fixed,
                "official_reassemble": ctx.official_reassemble,
                "official_skip": ctx.official_skip,
                "official_join_errors": {k: v.error for k, v in ctx.official_join.items()},
            },
        },
    )
    for o in outcomes:
        log(o.message)
    return PipelineResult(spec=spec, outputs=outputs, report=rep, descriptors=descriptors)


def _selections(spec: PipelineSpec, name: str) -> list[dict[str, str]]:
    """結合ファイル ``name`` の全結合を最低 1 回は出現させるための VariantSet 選択パターン一覧を返す。

    全組み合わせの直積を回すと VariantSet 数に対して指数的に増えるため、
    「全 VariantSet が既定値の 1 回（``{}``）」＋「各 VariantSet の 2 番目以降の選択肢を 1 つずつ切り替える」
    最小カバー集合で全結合を網羅する。
    """
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
    """パイプラインが使用する Isaac Sim 拡張（URDF Importer / RobotAssembler / RobotPoser / PhysX Tensors）を有効化する。

    headless 起動時の設定によってはこれらが未ロードの場合があるため、構築開始前にオンデマンドで有効化し
    2 フレーム進めて登録を反映させる。
    """
    try:
        import omni.kit.app

        app = omni.kit.app.get_app()
        if app is None:
            return
        mgr = app.get_extension_manager()
        for e in (
            "isaacsim.asset.importer.urdf",
            "isaacsim.robot_setup.assembler",
            "isaacsim.robot.poser",
            "omni.physx.tensors",
        ):
            if not mgr.is_extension_enabled(e):
                mgr.set_extension_enabled_immediate(e, True)
        runtime.step(2)
    except Exception:  # noqa: BLE001
        pass


__all__ = [
    "PipelineStopped",
    "PipelineResult",
    "urdf_info",
    "usd_info",
    "build_assembly",
    "run_pipeline",
]
