"""規則 R01〜R14 の事前チェック（``detect``）と事後確認（``symptom`` / ``evaluate``）を定義するモジュール。

責務:
  Isaac Sim の URDF Importer / Robot Assembler / RobotPoser で発生し得る既知の不具合 R01〜R14 を
  統一的な規則テーブル ``RULES`` として管理し、以下 2 段階の評価を行う:
    1. 事前チェック（``detect(ctx, phase)``）:
       URDF パース直後（``phase="urdf"``）または import・正規化直後（``phase="usd"``）に
       静的な構造から「このまま公式手順で進めると何が起きるか」を ``Finding`` として予測する。
    2. 事後確認（``evaluate`` / ``evaluate_all``）:
       ``probes.py``・``official.py``・``compose.py`` の実測値から、公式のまま（``official``）と
       修正後（``fixed``）のそれぞれで症状が発生したかを判定し、6 種類の ``Verdict`` へ分類する。
       事前チェックで予測が出なかった対象に対しても毎回 ``symptom`` を評価することで、
       既知の条件に当てはまらない新たな不具合（``unexpected``）も見逃さない設計になっている。

規則の分類（全 14 規則）:
  - URDF / 部品規則（対象 = 部品名、R03 のみ ``部品名:follower名``）:
    R01（空ルートリンク）、R03（mimic follower の drive）、R04（末端リンク座標ずれ）、
    R06（Physics variant 未選択）、R10（衝突メッシュ可視重複）、R13（再 import 連番フォルダ）
  - 結合規則（対象 = ``結合ファイル名/結合名``）:
    R02（子のワールド固定 joint 残留）、R05（instanceable 配置先）、R07（Articulation Root 残留）、
    R08（子リンク剛体なし）、R09（閉ループ）、R11（親との衝突除外なし）、
    R12（RobotPoser teleport 置き去り）、R14（再結合時の固定関節増殖）

主な公開 API:
  - ``Finding`` / ``Side`` / ``Outcome`` / ``Context`` / ``Rule`` / ``JoinFacts``
  - ``RULES``: R01〜R14 の規則定義辞書（``kinds.RuleId`` との完全一致を import 時に assert）
  - ``join_facts(ctx)``: 全結合の親・子・配置先・剛体配下フラグを事前チェック用にまとめる
  - ``detect(ctx, phase)`` / ``evaluate(ctx, findings)`` / ``evaluate_all(ctx, findings=None)``
  - ``format_table(outcomes)``: ``Outcome`` 一覧を Markdown 表文字列へ整形する

パイプライン全体での位置づけ:
  ``pipeline.build_assembly`` の Step 1（``detect("urdf")``）・Step 3（``detect("usd")``）および
  ``pipeline.run_pipeline`` の Step 9（``evaluate_all``）から呼ばれ、結果は ``report.py`` へ渡される。

手を入れやすい箇所:
  - 新しい規則 R15 を追加する場合は、``kinds.RuleId`` に ID を追加し、``_r15_detect`` と
    ``_r15_symptom`` を実装して ``RULES`` に 1 エントリ足すだけでパイプラインとレポートへ自動統合される。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Callable, get_args

from .kinds import RuleId, Verdict
from .sources import static_resolve
from .spec import PipelineSpec, join_id


@dataclass
class Finding:
    """事前チェック（``detect``）が出した 1 件ぶんの予測結果。"""

    rule: RuleId
    target: str
    reason: str
    data: dict = field(default_factory=dict)


@dataclass
class Side:
    """公式のまま / 修正後 のどちらか一方での症状判定結果。``occurred=None`` は未測定・測定不能を表す。"""

    occurred: bool | None
    desc: str


@dataclass
class Outcome:
    """1 規則 × 1 対象に対する最終評価結果（事前予測・公式実測・修正後実測・最終 ``verdict``）。"""

    rule: RuleId
    title: str
    target: str
    predicted: bool
    reason: str
    official: Side
    fixed: Side
    fix_mode: str
    verdict: Verdict
    message: str


@dataclass
class Context:
    """事前チェックと事後評価に必要な全材料を保持するコンテナ。``pipeline.py`` が段階的に埋める。"""

    spec: PipelineSpec
    models: dict[str, Any] = field(default_factory=dict)  # URDF 名 -> UrdfModel
    builds: dict[str, Any] = field(default_factory=dict)  # 部品名 -> PartBuild
    statics: dict[str, dict] = field(default_factory=dict)  # 部品名 -> probes.static_part の結果
    mimic_official: dict[str, dict] = field(default_factory=dict)  # 部品名 -> 公式 mimic プローブ結果
    mimic_fixed: dict[str, dict] = field(default_factory=dict)  # 部品名 -> 修正後 mimic プローブ結果
    official: dict[str, dict] = field(default_factory=dict)  # 結合 ID -> probe_stage 結果（公式のまま）
    official_join: dict[str, Any] = field(default_factory=dict)  # 結合 ID -> JoinResult（公式のまま）
    official_skip: dict[str, str] = field(default_factory=dict)  # 結合 ID -> 公式のままを測らなかった理由
    official_reassemble: dict[str, int | None] = field(default_factory=dict)  # 結合 ID -> R14 2回結合後の関節数
    fixed: dict[str, dict] = field(default_factory=dict)  # 結合 ID -> probe_stage 結果（修正後）
    fixed_join: dict[str, Any] = field(default_factory=dict)  # 結合 ID -> JoinResult（修正後）
    recompose_changed: dict[str, bool] = field(default_factory=dict)  # 結合ファイル名 -> 2回目生成で変化したか


@dataclass
class Rule:
    """規則 1 件のメタデータと ``detect`` / ``symptom`` 関数ペア。"""

    rid: RuleId
    title: str
    kind: str  # "urdf" | "join"
    phase: str  # "urdf" = import の前に判定可能 / "usd" = import 後の静的検査結果を使って判定
    symptom_word: str  # レポート文面で「」内に差し込む症状名
    not_occurred_hint: str  # 事前予測あり・公式症状なしのときに添える補足説明（公式修正の可能性等）
    detect: Callable[[Context], list[Finding]]
    symptom: Callable[[Context, str], tuple[Side, Side]]


RuleSpec = Rule


# ================================================================ 共通
def _na(desc: str) -> Side:
    return Side(None, desc)


def _mm(v: float) -> str:
    return f"{v * 1000:.2f} mm"


def _resolve_link(spec: PipelineSpec, usda_name: str, scoped: str) -> tuple[str, str]:
    try:
        res = static_resolve(spec, usda_name, scoped)  # type: ignore[call-arg]
        if isinstance(res, tuple) and len(res) == 2:
            return res
    except TypeError:
        pass
    return spec.resolve_link(usda_name, scoped)


@dataclass
class JoinFacts:
    """結合 1 回について、仕様と URDF の調べた結果から分かること(事前チェック用)。"""

    jid: str
    spec: Any
    parent_urdf: str
    parent_link: str
    child_urdf: str  # 子の usda の一番上の URDF
    child_link_urdf: str  # 子のリンクの持ち主の URDF
    child_link: str
    place_urdf: str | None  # 子を置く prim の持ち主の URDF(パスで書かれていれば None)
    place_link: str | None
    under_rigid_body: bool | None  # 子を置く prim の位置か上に剛体があるか(分からなければ None)


def _resolve_static_link_key(st: dict | None, link: str) -> str | None:
    if not st:
        return None
    lps = st.get("link_prims", {})
    if link in lps:
        return link
    if "/" in link:
        leaf = link.rsplit("/", 1)[-1]
        if leaf in lps and str(lps[leaf]).endswith("/" + link):
            return leaf
        for k, p in lps.items():
            if str(p).endswith("/" + link):
                return k
    return None


def join_facts(ctx: Context) -> list[JoinFacts]:
    """仕様内の全結合について、事前チェックに必要な親・子・配置先・剛体配下フラグを整理して返す。副作用なし。"""
    out = []
    for f, j in ctx.spec.all_joins():
        pu, pl = _resolve_link(ctx.spec, f.parent, j.parent_link)
        cu, cl = _resolve_link(ctx.spec, j.child, j.child_link)
        if j.parent_prim == ".":
            place_u, place_l, under = None, None, False  # 結合ファイル最上位 prim（剛体ではない）
        elif j.parent_prim and j.parent_prim.startswith("/"):
            place_u, place_l, under = None, None, None
        else:
            place_u, place_l = _resolve_link(ctx.spec, f.parent, j.parent_prim) if j.parent_prim else (pu, pl)
            st = ctx.statics.get(place_u)
            lk = _resolve_static_link_key(st, place_l)
            under = None if st is None or lk is None else lk not in st["links_without_body_above"]
        out.append(JoinFacts(join_id(f, j), j, pu, pl, ctx.spec.root_source(j.child).name, cu, cl, place_u, place_l, under))
    return out


def _facts(ctx: Context, jid: str) -> JoinFacts:
    return next(x for x in join_facts(ctx) if x.jid == jid)


def _reach_side(probe: dict | None, prefixes: set[str] | None, tol_deg: float, tol_m: float) -> Side:
    """Play プローブの ``reach`` 結果から、対象関節が目標値に許容誤差内で到達したか判定する。"""
    if not probe:
        return _na("測っていない")
    reach = {k: r for k, r in probe.get("reach", {}).items() if prefixes is None or r.get("prefix") in prefixes}
    if not reach:
        return _na("動かす関節が無い")
    bad, parts = False, []
    for name, r in reach.items():
        if r.get("error") is None:
            bad = True
            parts.append(f"{name}: 親の Articulation の DOF に無い")
            continue
        if r["kind"] == "prismatic":
            ok = r["error"] <= tol_m
            parts.append(f"{name} {r['final']:.4f}/{r['target']:.4f} m")
        else:
            ok = math.degrees(r["error"]) <= tol_deg
            parts.append(f"{name} {math.degrees(r['final']):.2f}°/目標 {math.degrees(r['target']):.2f}°")
        bad |= not ok
    return Side(bad, "、".join(parts))


def _sides(ctx: Context, jid: str) -> list[tuple[dict | None, Any, str | None]]:
    """(測定, JoinResult, 測らなかった理由) を 公式のまま・修正後 の順に。"""
    return [(ctx.official.get(jid), ctx.official_join.get(jid), ctx.official_skip.get(jid)),
            (ctx.fixed.get(jid), ctx.fixed_join.get(jid), None)]


def _join_info(probe: dict | None, jid: str) -> dict | None:
    return (probe or {}).get("joins", {}).get(jid)


# ================================================================ R01 ルートリンクが空
# 目的: 空（ghost）ルートリンクによりワールド固定 joint 名が root_joint でなくなるケースを検出する
def _r01_detect(ctx: Context) -> list[Finding]:
    out = []
    for name, m in ctx.models.items():
        if m.links[m.root_link].ghost:
            first = next((j.name for j in m.child_joints(m.root_link)), "?")
            out.append(Finding("R01", name, f"URDF のルートリンク {m.root_link!r} が空のリンク(visual・collision なし、質量なし)なので、"
                                           f"ワールド固定 joint は root_joint ではなく URDF の joint 名({first})になる"))
    return out


def _r01_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    st = ctx.statics.get(target)
    if not st:
        return _na("調べていない"), _na("調べていない")
    names = [a["name"] for a in st["anchors"]]
    official = Side(bool(names) and any(n != "root_joint" for n in names), f"ワールド固定 joint の名前 {names or 'なし'}")
    if not ctx.spec.fix_applied("R02"):
        return official, _na("R02 の修正を skip したので、ワールド固定 joint は残してある(修正後は測らない)")
    left, used = [], False
    for x in join_facts(ctx):
        if x.child_urdf != target:
            continue
        used = True
        info = _join_info(ctx.fixed.get(x.jid), x.jid)
        if info is not None:
            left += [a["path"] for a in info.get("external_anchors", [])]
    if not used:
        return official, Side(False, "結合の子に使っていない")
    return official, Side(bool(left), f"子として結合した後に残ったワールド固定 joint {left or 'なし'}")


# ================================================================ R02 子のワールド固定 joint を Robot Assembler が無効にしない
# 目的: body0・body1 とも埋まったワールド固定 joint が無効化されず親関節がロックする不具合を検出する
def _r02_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        if x.under_rigid_body is not False:
            continue  # 剛体の子に置くと、その joint は親の剛体につながる（ワールドではない）。このケースは R09（閉ループ）で扱う
        hard = [a for a in ctx.statics.get(x.child_urdf, {}).get("anchors", []) if a["both_targets_authored"]]
        if hard:
            out.append(Finding("R02", x.jid, f"子 {x.spec.child} のワールド固定 joint {[a['name'] for a in hard]} は body0 も body1 も空でないので、"
                                             "Robot Assembler が無効にせず、子がワールドに固定されて親の関節が動かなくなる"))
    return out


def _r02_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    p = ctx.spec.probe
    out = []
    for probe, res, skip in _sides(ctx, target):
        if skip:
            out.append(_na(skip))
            continue
        if res is not None and res.error:
            out.append(_na(f"結合できなかった({res.error[:120]})"))
            continue
        prefix = res.parent_owner.prefix if res is not None and res.parent_owner is not None else ""
        s = _reach_side(probe, {prefix}, p.angle_tol_deg, p.length_tol_m)
        drift = (_join_info(probe, target) or {}).get("mount_drift_during_play")
        if drift is not None and s.occurred is not None:
            s = Side(s.occurred or drift[0] > p.length_tol_m or drift[1] > p.angle_tol_deg,
                     s.desc + f"、結合のずれ {_mm(drift[0])} / {drift[1]:.2f}°")
        out.append(s)
    return out[0], out[1]


# ================================================================ R03 mimic follower の drive
# 目的: follower 関節に残った不要 drive が leader の目標到達を妨害・遅延させる不具合を検出する
def _usd_mimic_entries(ctx: Context) -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []
    for name, st in ctx.statics.items():
        if name in ctx.models:
            continue
        for item in st.get("mimic_followers", []) or []:
            if isinstance(item, dict):
                fname = str(item.get("follower") or item.get("name") or "")
                lname = str(item.get("leader") or item.get("reference") or item.get("reference_joint") or "?")
            else:
                fname = str(item)
                lname = "?"
            if fname:
                out.append((name, fname, lname))
    return out


def _r03_detect(ctx: Context) -> list[Finding]:
    # URDF の <dynamics damping> は newton:damping に入るだけで、PhysX の drive の damping にはならない
    # (urdf_usd_converter/_impl/link.py:489。実測でも drive の damping は 0)ので判定に使わない
    out = []
    for name, m in ctx.models.items():
        for follower, leader in m.mimic_pairs():
            g = ctx.statics.get(name, {}).get("raw_follower_drive", {}).get(follower.name)
            if g and (g["stiffness"] or g["damping"]):
                out.append(Finding("R03", f"{name}:{follower.name}", f"{name} の mimic follower {follower.name}(leader {leader.name})の drive が "
                                                                     f"stiffness={g['stiffness']}, damping={g['damping']} なので、follower が力を出して leader が目標に届かない・遅れる"))
    for name, fname, lname in _usd_mimic_entries(ctx):
        g = ctx.statics.get(name, {}).get("raw_follower_drive", {}).get(fname)
        if g and (g["stiffness"] or g["damping"]):
            out.append(Finding("R03", f"{name}:{fname}", f"{name} の mimic follower {fname}(leader {lname})の drive が "
                                                         f"stiffness={g['stiffness']}, damping={g['damping']} なので、follower が力を出して leader が目標に届かない・遅れる"))
    return out


def _r03_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    name, follower = target.split(":", 1)
    off = ctx.mimic_official.get(name, {}).get(follower)
    fx = ctx.mimic_fixed.get(name, {}).get(follower)
    p = ctx.spec.probe

    def tol(r: dict) -> float:
        return p.length_tol_m if r["type"] == "prismatic" else math.radians(p.angle_tol_deg)

    def desc(r: dict) -> str:
        u = "m" if r["type"] == "prismatic" else "rad"
        ser = "、".join(f"{k} ステップ目 {v:.4f}" for k, v in r.get("leader_series", {}).items())
        return f"leader {r['leader_final']:.4f}/目標 {r['leader_target']:.4f} {u}({ser})、mimic の関係の誤差 {r['relation_error']:.5f} {u}"

    if off is None or fx is None:
        return _na("測っていない"), _na("測っていない")
    lag = max([abs(off["leader_series"][k] - fx["leader_series"][k]) for k in off.get("leader_series", {})
               if k in fx.get("leader_series", {})] or [0.0])
    off_occ = (off["leader_error"] - fx["leader_error"]) > tol(off) or off["relation_error"] > tol(off) or lag > tol(off)
    fx_occ = fx["leader_error"] > tol(fx) or fx["relation_error"] > tol(fx)
    return Side(off_occ, desc(off)), Side(fx_occ, desc(fx))


def _r03_targets(ctx: Context) -> list[str]:
    targets = [f"{n}:{f.name}" for n, m in ctx.models.items() for f, _ in m.mimic_pairs()]
    for name, fname, _lname in _usd_mimic_entries(ctx):
        t = f"{name}:{fname}"
        if t not in targets:
            targets.append(t)
    return targets


# ================================================================ R04 末端リンクの座標ずれ
# 目的: 形状が 1 つだけの末端リンクで prim 原点が形状原点へ移され結合面がずれる不具合を検出する
def _r04_detect(ctx: Context) -> list[Finding]:
    out = []
    for name, m in ctx.models.items():
        cfv = ctx.builds[name].collision_from_visuals if name in ctx.builds else (not m.has_collisions())
        hits = [ln for ln, lnk in m.links.items() if m.is_leaf(ln) and len(lnk.visuals) + (0 if cfv else len(lnk.collisions)) == 1]
        if hits:
            out.append(Finding("R04", name, f"{name} の末端リンク {hits} は子の形状が 1 つだけなので、リンクの prim が形状の原点へ移され、"
                                            "prim の位置が URDF のリンク座標からずれる(prim の位置で結合するとずれる)"))
    return out


def _r04_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    st = ctx.statics.get(target)
    if not st:
        return _na("調べていない"), _na("調べていない")
    tol_m, tol_deg = ctx.spec.probe.length_tol_m, ctx.spec.probe.angle_tol_deg
    shifts = {k: v for k, v in st["link_frame_shift"].items() if v[0] > 1e-5 or v[1] > 1e-3}
    off_desc = "prim と URDF のリンク座標のずれ " + (", ".join(f"{k} {_mm(v[0])}" for k, v in shifts.items()) or "なし")
    off_occ = bool(shifts)
    fixed_parts, fixed_occ, used = [], False, False
    for x in join_facts(ctx):
        if target not in (x.parent_urdf, x.child_link_urdf):
            continue
        used = True
        res = ctx.official_join.get(x.jid)
        al = (_join_info(ctx.official.get(x.jid), x.jid) or {}).get("urdf_mount_alignment")
        if al is not None and not (res is not None and res.error):
            off_desc += f"。{x.jid} を Robot Assembler で結合した結合面のずれ {_mm(al[0])} / {al[1]:.2f}°"
            off_occ = off_occ or al[0] > tol_m
        fal = (_join_info(ctx.fixed.get(x.jid), x.jid) or {}).get("urdf_mount_alignment")
        if fal is not None:
            fixed_parts.append(f"{x.jid} {_mm(fal[0])} / {fal[1]:.3f}°")
            fixed_occ = fixed_occ or fal[0] > tol_m or fal[1] > tol_deg
    if not used:
        return Side(off_occ, off_desc), Side(False, "結合に使っていない(prim の位置は使わない)")
    return Side(off_occ, off_desc), Side(fixed_occ, "結合面(URDF のリンク座標)のずれ " + ("、".join(fixed_parts) or "測れなかった"))


# ================================================================ R05 instanceable な prim に子を作れない
# 目的: 配置先または子リンクが instanceable のため instance proxy 書き込み失敗で結合が落ちる不具合を検出する
def _r05_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        why = []
        if x.place_urdf and x.place_link in ctx.statics.get(x.place_urdf, {}).get("instanceable_links", []):
            why.append(f"子を置く prim({x.place_urdf}:{x.place_link})が instanceable(子の prim を作れない)")
        if x.child_link in ctx.statics.get(x.child_link_urdf, {}).get("instanceable_links", []):
            why.append(f"子のリンク {x.child_link_urdf}:{x.child_link} が instanceable(Robot Assembler はこの下に固定関節を作る)")
        if why:
            out.append(Finding("R05", x.jid, "、".join(why) + "ので、instance proxy への書き込みで結合が失敗する"))
    return out


def _r05_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    out = []
    for _probe, res, skip in _sides(ctx, target):
        if skip:
            out.append(_na(skip))
        elif res is None:
            out.append(_na("測っていない"))
        else:
            e = res.error or ""
            out.append(Side("instance proxy" in e or "instanceable" in e or e.startswith("R05"), f"結合の結果: {e[:160] if e else '成功'}"))
    return out[0], out[1]


# ================================================================ R06 Physics variant が未選択
# 目的: URDF Importer 出力の Physics VariantSet が未選択で剛体・関節が空になる不具合を検出する
def _r06_detect(ctx: Context) -> list[Finding]:
    out = []
    for name in ctx.models:
        st = ctx.statics.get(name, {})
        if st.get("physics_variants") and not st.get("physics_variant_selection"):
            out.append(Finding("R06", name, f"{name} の出力 USD の Physics variant が未選択({st['physics_variants']})なので、"
                                            "そのまま参照すると剛体も関節も無い"))
    return out


def _r06_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    st = ctx.statics.get(target)
    if not st:
        return _na("調べていない"), _na("調べていない")
    return (Side(st["raw_rigid_bodies_as_is"] == 0, f"生成物をそのまま開いたときの剛体 {st['raw_rigid_bodies_as_is']} 個"),
            Side(st["fixed_rigid_bodies"] == 0, f"正規化層の剛体 {st['fixed_rigid_bodies']} 個"))


# ================================================================ R07 Articulation Root が消えない
# 目的: NewtonArticulationRootAPI が残り、子ツリーにネストした Articulation Root が残留する不具合を検出する
def _r07_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        n = ctx.statics.get(x.child_urdf, {}).get("newton_articulation_roots", [])
        if n:
            out.append(Finding("R07", x.jid, f"子 {x.spec.child} の {n} に NewtonArticulationRootAPI がある(PhysicsArticulationRootAPI を内蔵)ので、"
                                             "Robot Assembler が PhysicsArticulationRootAPI だけ外しても Articulation Root が残る"))
    return out


def _r07_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    out = []
    for probe, res, skip in _sides(ctx, target):
        info = _join_info(probe, target)
        if skip:
            out.append(_na(skip))
        elif info is None or (res is not None and res.error):
            out.append(_na("結合できなかったか、測っていない"))
        else:
            roots = info.get("articulation_roots_in_child", [])
            out.append(Side(bool(roots), f"結合後に子に残った Articulation Root {roots or 'なし'}"))
    return out[0], out[1]


# ================================================================ R08 子のリンクに剛体が無い
# 目的: 子の接続リンク自体に剛体が無く、固定関節の body1 が剛体へ解決されない不具合を検出する
def _r08_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        child_st = ctx.statics.get(x.child_urdf, {})
        if child_st and child_st.get("fixed_rigid_bodies") == 0:
            continue  # 剛体を 1 つも持たない compound 部品(例: Thor バックパック)は固定関節を作らない
        link_st = ctx.statics.get(x.child_link_urdf, {})
        lk = _resolve_static_link_key(link_st, x.child_link) or x.child_link
        if lk in link_st.get("links_without_body_above", []):
            out.append(
                Finding(
                    "R08",
                    x.jid,
                    f"子のリンク {x.child_link_urdf}:{x.child_link} の位置にも上にも剛体が無いので、"
                    "Robot Assembler が結合できない(3.3.8 より前は固定されずに分離した)",
                )
            )
    return out


def _r08_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    out = []
    for probe, res, skip in _sides(ctx, target):
        if skip:
            out.append(_na(skip))
        elif res is None:
            out.append(_na("測っていない"))
        elif res.error:
            out.append(Side(True, f"結合が失敗した: {res.error[:160]}"))
        else:
            b1 = (_join_info(probe, target) or {}).get("joint_body1_resolved", "?")
            out.append(Side(b1 is None, f"固定関節の子の側の剛体 {b1}"))
    return out[0], out[1]


# ================================================================ R09 閉ループ
# 目的: 剛体配下に置いた子のワールド固定 joint が親剛体へ接続され意図しない閉ループになる不具合を検出する
def _r09_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        anchors = ctx.statics.get(x.child_urdf, {}).get("anchors", [])
        if x.under_rigid_body and any(a["both_targets_authored"] for a in anchors):
            out.append(Finding("R09", x.jid, f"子 {x.spec.child} を剛体の子に置き、body0 が空でないワールド固定 joint "
                                             f"{[a['name'] for a in anchors]} が残ると、その body0 が親の剛体に解決されて閉ループになる"))
    return out


def _r09_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    out = []
    for probe, _res, skip in _sides(ctx, target):
        info = _join_info(probe, target)
        if skip:
            out.append(_na(skip))
        elif info is None:
            out.append(_na("測っていない"))
        else:
            hits = info.get("closed_loop_logs", [])
            out.append(Side(bool(hits), f"Play 中の閉ループのログ {len(hits)} 件" + (f"(例: {hits[0][:120]})" if hits else "")))
    return out[0], out[1]


# ================================================================ R10 衝突メッシュが見える
# 目的: collision_from_visuals=False かつ同一メッシュ時に衝突側が guide でなく可視重複する不具合を検出する
def _r10_detect(ctx: Context) -> list[Finding]:
    out = []
    for name, m in ctx.models.items():
        cfv = ctx.builds[name].collision_from_visuals if name in ctx.builds else (not m.has_collisions())
        if cfv:
            continue
        hits = [ln for ln, lnk in m.links.items() if {g.mesh for g in lnk.visuals if g.mesh} & {g.mesh for g in lnk.collisions if g.mesh}]
        if hits:
            out.append(Finding("R10", name, f"{name} のリンク {hits} は visual と collision が同じメッシュで collision_from_visuals = False なので、"
                                            "衝突メッシュが見えて重なる恐れがある(旧版の Asset Transformer の不具合)"))
    return out


def _r10_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    st = ctx.statics.get(target)
    if not st:
        return _na("調べていない"), _na("調べていない")
    v = st["visible_duplicate_colliders"]
    s = Side(bool(v), f"見えている衝突メッシュ(purpose が guide でなく、visual と同じ位置・大きさ) {v or 'なし'}")
    return s, s


# ================================================================ R11 衝突除外が無い
# 目的: 親剛体と子剛体の間に FilteredPairsAPI が設定されず Play 中に不要な接触が起きる不具合を検出する
def _r11_detect(ctx: Context) -> list[Finding]:
    return [Finding("R11", x.jid, "Robot Assembler は親と子の間に衝突除外(FilteredPairs)を作らないので、子が親に当たる恐れがある")
            for x in join_facts(ctx)]


def _r11_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    out = []
    for probe, res, skip in _sides(ctx, target):
        info = _join_info(probe, target)
        if skip:
            out.append(_na(skip))
        elif info is None or (res is not None and res.error) or "contacts_with_parent" not in info:
            out.append(_na("結合できなかったか、測っていない"))
        else:
            c = info["contacts_with_parent"]
            out.append(Side(bool(c), f"Play 中の親との接触 {len(c)} 組" + (f"(例: {c[0]})" if c else "")))
    return out[0], out[1]


# ================================================================ R12 RobotPoser の teleport で置いて行かれる
# 目的: Stop 中の RobotPoser.apply_joint_state で子が追従しない・親が逆向きに動く不具合を検出する
def _r12_detect(ctx: Context) -> list[Finding]:
    out = []
    for x in join_facts(ctx):
        why = []
        if x.under_rigid_body is False:
            why.append("子を剛体の子に置かない(stage の親子が実際のリンクの親子と違う)うえ、Robot Assembler の固定関節には IsaacJointAPI が無い")
        m = ctx.models.get(x.child_link_urdf)
        if m is not None and (x.child_link_urdf != x.child_urdf or x.child_link != m.root_link):
            why.append(f"子のリンク {x.child_link_urdf}:{x.child_link} が子のルートリンクでない(RobotPoser の木が子の中の関節を逆向きにたどる)")
        if why:
            out.append(Finding("R12", x.jid, "、".join(why) + "ので、RobotPoser の apply_joint_state(Stop 中)で子が置いて行かれる"))
    return out


def _r12_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    p = ctx.spec.probe
    out = []
    for probe, res, skip in _sides(ctx, target):
        info = _join_info(probe, target)
        if skip:
            out.append(_na(skip))
        elif info is None or (res is not None and res.error) or "poser_drift" not in info:
            out.append(_na((probe or {}).get("poser_error", "結合できなかったか、測っていない")))
        else:
            d = info["poser_drift"]
            fk = info.get("poser_fk_error", (0.0, 0.0))
            bad = d[0] > p.length_tol_m or d[1] > p.angle_tol_deg or fk[0] > p.length_tol_m or fk[1] > p.angle_tol_deg
            out.append(Side(bad, f"teleport 後の結合のずれ {_mm(d[0])} / {d[1]:.2f}°、親の剛体の順運動学との差 {_mm(fk[0])} / {fk[1]:.2f}°"
                                 f"(親は {info.get('poser_parent_moved_deg', 0):.1f}° 動いた)"))
    return out[0], out[1]


# ================================================================ R13 再 import で別名のフォルダ
# 目的: 既存出力先へ再 import した際に URDF Importer が _1 等の連番フォルダを作る不具合を検出する
def _r13_detect(ctx: Context) -> list[Finding]:
    return [Finding("R13", name, f"出力先 {b.expected_dir} が空でないので、importer にそのまま出させると別名(_1 など)のフォルダになる")
            for name, b in ctx.builds.items() if name in ctx.models and b.target_was_nonempty]


def _r13_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    b = ctx.builds.get(target)
    if b is None:
        return _na("import していない"), _na("import していない")
    final_dir = os.path.normcase(os.path.abspath(os.path.dirname(b.generated_usd)))
    ok = final_dir == os.path.normcase(os.path.abspath(b.expected_dir))
    return (Side(b.importer_would_suffix, f"importer の出力先の決め方では {'別名になる' if b.importer_would_suffix else '同じ名前'}"),
            Side(not ok, f"実際の出力先 {os.path.dirname(b.generated_usd)}(今回の import: {'した' if b.imported else '入力が同じなので飛ばした'})"))


# ================================================================ R14 やり直すと固定関節が増える
# 目的: 同一結合をやり直した際に固定関節が複数本増殖する（冪等性が壊れる）不具合を検出する
def _r14_detect(ctx: Context) -> list[Finding]:
    return [Finding("R14", x.jid, "Robot Assembler は固定関節に毎回空いている名前を付けるので、同じ結合をやり直すと固定関節が増える")
            for x in join_facts(ctx)]


def _r14_symptom(ctx: Context, target: str) -> tuple[Side, Side]:
    skip = ctx.official_skip.get(target)
    n = ctx.official_reassemble.get(target)
    off = _na(skip) if skip else (_na("測っていない") if n is None else Side(n > 1, f"Robot Assembler で 2 回結合した後の固定関節 {n} 本"))
    info = _join_info(ctx.fixed.get(target), target)
    if info is None or "fixed_joints_on_mount_pair" not in info:
        return off, _na("結合できなかったか、測っていない")
    k = info["fixed_joints_on_mount_pair"]
    changed = ctx.recompose_changed.get(target.split("/")[0])
    return off, Side(k != 1 or bool(changed), f"親と子の剛体をつなぐ固定関節 {k} 本、作り直したときのファイルの変化 {'あり' if changed else 'なし'}")


# ================================================================ 一覧
RULES: dict[str, Rule] = {r.rid: r for r in [
    Rule("R01", "ルートリンクが空", "urdf", "urdf", "ワールド固定 joint の名前が root_joint にならない", "空のルートリンクでも root_joint という名前になっている",
         _r01_detect, _r01_symptom),
    Rule("R02", "子のワールド固定 joint が無効にならない", "join", "usd", "親の関節が動かない",
         "公式側の修正(Robot Assembler が body0 の埋まった root_joint も無効にする)の可能性", _r02_detect, _r02_symptom),
    Rule("R03", "mimic follower の drive", "urdf", "usd", "leader が目標に届かない", "follower の drive の力が小さく、この条件では影響が出なかった可能性",
         _r03_detect, _r03_symptom),
    Rule("R04", "末端リンクの座標ずれ", "urdf", "urdf", "リンク座標のずれ", "公式側の修正(リンク prim を形状の原点へ移さない)の可能性", _r04_detect, _r04_symptom),
    Rule("R05", "instanceable な prim", "join", "usd", "結合の失敗", "公式側の修正の可能性", _r05_detect, _r05_symptom),
    Rule("R06", "Physics variant が未選択", "urdf", "usd", "剛体も関節も無い", "公式側の修正(既定の variant を選ぶ)の可能性", _r06_detect, _r06_symptom),
    Rule("R07", "Articulation Root が消えない", "join", "usd", "Articulation Root が残る", "公式側の修正(Newton の API も外す)の可能性",
         _r07_detect, _r07_symptom),
    Rule("R08", "子のリンクに剛体が無い", "join", "usd", "結合の失敗", "公式側の修正の可能性", _r08_detect, _r08_symptom),
    Rule("R09", "閉ループ", "join", "usd", "閉ループのエラー", "公式側の修正の可能性", _r09_detect, _r09_symptom),
    Rule("R10", "衝突メッシュが見える", "urdf", "urdf", "衝突メッシュが見える", "Asset Transformer rules 1.7.7 以降の修正で、衝突側が guide のまま",
         _r10_detect, _r10_symptom),
    Rule("R11", "衝突除外が無い", "join", "usd", "子と親の接触", "この配置では形が当たらなかった(公式側の修正とは限らない)", _r11_detect, _r11_symptom),
    Rule("R12", "RobotPoser で置いて行かれる", "join", "usd", "teleport で子が置いて行かれる", "公式側の修正(結合の固定関節を木に入れる)の可能性",
         _r12_detect, _r12_symptom),
    Rule("R13", "再 import で別名のフォルダ", "urdf", "usd", "別名のフォルダへの出力", "公式側の修正(上書きする)の可能性", _r13_detect, _r13_symptom),
    Rule("R14", "やり直すと固定関節が増える", "join", "usd", "固定関節の増加", "公式側の修正(既存の固定関節を使い回す)の可能性", _r14_detect, _r14_symptom),
]}
assert set(RULES) == set(get_args(RuleId)), "RULES と RuleId の一覧が食い違っている"


def targets_of(ctx: Context, r: Rule) -> list[str]:
    """規則 ``r`` の事後評価対象 ID 一覧を返す（R03 は ``部品:follower``、URDF 規則は部品名、結合規則は結合 ID）。"""
    if r.rid == "R03":
        return _r03_targets(ctx)
    if r.kind == "urdf":
        return list(ctx.models)
    return [join_id(f, j) for f, j in ctx.spec.all_joins()]


def detect(ctx: Context, phase: str) -> list[Finding]:
    """指定フェーズ（``"urdf"`` または ``"usd"``）に該当する全規則の事前チェックを実行する。

    R04 と R10 は URDF 段階でも仮判定するが、実際の ``collision_from_visuals`` 値は
    ``build_part`` 実行後に確定するため、``phase="usd"`` で最新条件のもと再判定して上書きする。

    Args:
        ctx: 判定材料を格納した ``Context``。
        phase: ``"urdf"``（import 前）または ``"usd"``（import・静的検査後）。

    Returns:
        ``(rule, target)`` ごとに重複排除された ``Finding`` の一覧。副作用なし。
    """
    out: list[Finding] = []
    for r in RULES.values():
        if r.phase == phase or (phase == "usd" and r.rid in ("R04", "R10")):
            # R04 / R10 は URDF でも判定し、import 後に確定した collision_from_visuals の結果で判定し直す
            out.extend(r.detect(ctx))
    seen: dict[tuple[str, str], Finding] = {}
    for f in out:
        seen[(f.rule, f.target)] = f
    return list(seen.values())


def evaluate(ctx: Context, findings: list[Finding]) -> list[Outcome]:
    """事前チェックの ``findings`` と ``ctx`` の実測値を突き合わせ、全規則×全対象の ``Outcome`` を返す。

    Args:
        ctx: 静的検査・動的プローブ結果まで埋まった ``Context``。
        findings: 事前チェック（``detect``）で得た ``Finding`` 一覧。

    Returns:
        全規則・全対象に対する ``Outcome`` リスト。副作用なし。
    """
    by = {(f.rule, f.target): f for f in findings}
    outcomes: list[Outcome] = []
    for r in RULES.values():
        for target in targets_of(ctx, r):
            f = by.get((r.rid, target))
            try:
                off, fixed = r.symptom(ctx, target)
            except Exception as exc:  # noqa: BLE001
                off, fixed = _na(f"測定に失敗: {type(exc).__name__}: {exc}"), _na("測定に失敗")
            mode = ctx.spec.fix_of(r.rid)
            verdict, message = _judge(r, target, f, off, fixed, mode)
            outcomes.append(Outcome(r.rid, r.title, target, f is not None, f.reason if f else "", off, fixed, mode, verdict, message))
    return outcomes


def evaluate_all(ctx: Context, findings: list[Finding] | None = None) -> list[Outcome]:
    """事前チェック（``findings`` 省略時は ``urdf`` + ``usd`` の両フェーズを実行）と事後測定から全規則の ``Outcome`` を返す。"""
    if findings is None:
        early = detect(ctx, "urdf")
        later = detect(ctx, "usd")
        seen = {(f.rule, f.target) for f in later}
        findings = [f for f in early if (f.rule, f.target) not in seen] + later
    return evaluate(ctx, findings)


def format_table(outcomes: list[Outcome]) -> str:
    """``Outcome`` の一覧を読みやすい Markdown 表文字列に整形する。副作用なし。"""
    lines = ["| 規則 | 対象 | 判定 | 内容 |", "|---|---|---|---|"]
    for o in outcomes:
        lines.append(f"| {o.rule} ({o.title}) | {o.target} | {o.verdict} | {o.message} |")
    return "\n".join(lines)


def _judge(r: Rule, target: str, f: Finding | None, off: Side, fixed: Side, mode: str) -> tuple[Verdict, str]:
    """事前予測・公式症状・修正後症状の 3 つから最終 ``Verdict`` と人間向け説明文を決定する。

    判定の優先順位:
      1. ``fixed.occurred`` が True → ``still_broken``（修正後でも問題が残っている最優先異常）
      2. 予測あり かつ 公式で症状あり → ``fixed``（期待どおり修正で解消）
      3. 予測あり かつ 公式で症状なし → ``upstream_fixed``（Isaac Sim 側の公式修正等で再現しなくなった）
      4. 予測なし かつ 公式で症状あり → ``unexpected``（事前チェックの漏れ・未知の原因）
      5. 公式または両方が未測定 → ``not_measured``
      6. それ以外 → ``ok``
    """
    head = f"[{r.rid}] {target}: "
    reason = f"{f.reason} と判定した" if f else "判定は出なかった"
    fix_word = "修正後" if mode == "apply" else "修正を skip した出力"
    w = f"「{r.symptom_word}」"
    if fixed.occurred:
        return "still_broken", f"{head}{reason} → {fix_word}でも{w}が起きた({fixed.desc})"
    if f and off.occurred:
        return "fixed", f"{head}{reason} → 公式のままでは、やはり{w}が起きた({off.desc})。{fix_word}は起きなかった({fixed.desc})"
    if f and off.occurred is False:
        return "upstream_fixed", f"{head}{reason} → 公式のままでも、最終的には{w}は起きなかった({off.desc})。{r.not_occurred_hint}"
    if not f and off.occurred:
        return "unexpected", f"{head}判定は出なかったが、公式のままで{w}が起きた({off.desc})。新しい原因の可能性"
    if off.occurred is None and fixed.occurred is None:
        return "not_measured", f"{head}{reason}。症状は測れなかった({off.desc} / {fixed.desc})"
    if f and off.occurred is None:
        return "not_measured", f"{head}{reason}。公式のままの症状は測れなかった({off.desc})。{fix_word}は起きなかった({fixed.desc})"
    return "ok", f"{head}判定なし・問題なし({off.desc} / {fixed.desc})"


__all__ = [
    "Finding",
    "Side",
    "Outcome",
    "Context",
    "Rule",
    "RuleSpec",
    "JoinFacts",
    "RULES",
    "join_facts",
    "targets_of",
    "detect",
    "evaluate",
    "evaluate_all",
    "format_table",
]
