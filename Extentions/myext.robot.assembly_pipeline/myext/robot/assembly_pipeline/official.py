"""公式 ``RobotAssembler`` の挙動を再現し、事前チェックで予測した不具合が実際に起きるか実測するモジュール。

責務:
  1. 親・子それぞれの未修正生成物（URDF Importer の生出力または元 USD アセット）を
     使い捨てステージへ読み込み、Isaac Sim 公式の ``RobotAssembler`` で結合する（``build_official``）。
  2. 同じ結合を 2 回実行して固定関節 ``AssemblerFixedJoint*`` が増殖するか（R14）を測る（``reassemble_count``）。
  3. 構築した公式ステージに ``probes.probe_stage`` を適用し、``compose.py`` による修正後ステージと
     同じ指標で比較できる測定結果を返す（``probe_official``）。

なぜ公式 ``RobotAssembler`` の不具合をわざわざ再現するのか:
  - 事前チェック（``rules.detect``）の仮説が「現行の Isaac Sim で本当に症状として出るか」を裏付けるため。
  - Isaac Sim 側の将来のアップデートで公式不具合が修正されたとき、公式再現で症状が出なくなり
    ``rules`` の判定が自動的に ``upstream_fixed`` に切り替わるようにするため。

再現対象の制約:
  親リンク・子リンクがいずれも最上位部品の直接リンク（表記に ``"/"`` を含まない単段結合）のみ対象。
  多段結合された子の内部リンクは ``measurable`` で理由を返してスキップする。

主な公開 API:
  - ``ROOT``: 公式再現ステージのルート prim パス（``"/World"``）
  - ``OfficialStage``: 公式再現ステージと親・子 ``Owner``・``JoinResult`` を束ねるデータクラス
  - ``measurable(j)``: 指定の結合を公式 ``RobotAssembler`` で再現できるか判定する
  - ``build_official(spec, f, j, builds=None)``: 単一結合の公式再現ステージを構築する
  - ``reassemble_count(o)``: 同一結合を 2 回実行した後の ``AssemblerFixedJoint*`` 本数を返す
  - ``probe_official(spec, f, info=None, builds=None)``: 結合ファイル全体の公式再現とプローブを実行する

パイプライン全体での位置づけ:
  ``pipeline.run_pipeline`` の Step 6（``spec.compare_official`` が有効な場合のみ）で呼ばれ、
  得られた測定値は ``rules.Context.official*`` 経由で ``rules.evaluate_all`` に渡される。

手を入れやすい箇所:
  - 公式 ``RobotAssembler`` の呼び出し手順や再現除外条件を調整したい場合は
    ``measurable``・``build_official``・``reassemble_count`` を編集する。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from . import runtime
from . import usd_util as _u
from .compose import JoinResult
from .sources import Owner, UsdaInfo
from .spec import JoinFileSpec, JoinSpec, PipelineSpec, UrdfSource, UsdSource, join_id
from .urdf_model import UrdfModel, load_urdf

ROOT = "/World"


# ---- usd_util ヘルパーの薄いラッパー群（関数名やシグネチャ差を吸収してフォールバックする） ----
def _joint_prims(stage: Usd.Stage, robot_root: str) -> dict[str, str]:
    fn = getattr(_u, "joint_prims", None)
    return fn(stage, robot_root)


def _joints_under(stage: Usd.Stage, root: str) -> list[Usd.Prim]:
    fn = getattr(_u, "joints_under", None)
    if fn is not None:
        return fn(stage, root)
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    all_prims = getattr(_u, "ALL_PRIMS", Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
    return [p for p in Usd.PrimRange(prim, all_prims) if p.IsA(UsdPhysics.Joint)]


def _link_prims(stage: Usd.Stage, robot_root: str, names: list[str] | None = None) -> dict[str, str]:
    fn = getattr(_u, "link_prims")
    return fn(stage, robot_root, names)


def _usd_link_prims(stage: Usd.Stage, robot_root: str) -> dict[str, str]:
    fn = getattr(_u, "usd_link_prims", None)
    if fn is not None:
        return fn(stage, robot_root)
    return _link_prims(stage, robot_root, None)


def _physics_variant_choice(prim: Usd.Prim, wanted: str, *, keep_existing: bool = False) -> str | None:
    fn = getattr(_u, "physics_variant_choice", getattr(_u, "select_physics_variant", None))
    if fn is not None:
        try:
            return fn(prim, wanted, keep_existing=keep_existing)
        except TypeError:
            return fn(prim, wanted)
    vs = prim.GetVariantSets()
    if not vs.HasVariantSet("Physics"):
        return None
    vset = vs.GetVariantSet("Physics")
    names = vset.GetVariantNames()
    if keep_existing and wanted == "auto":
        cur = vset.GetVariantSelection()
        if cur and cur != "None" and cur in names:
            return cur
    if wanted == "auto":
        for n in ("physx", "physics"):
            if n in names:
                return n
        lower_map = {n.lower(): n for n in names}
        for key in ("physx", "physics", "physx_mimic", "simplifiedphysx"):
            if key in lower_map:
                return lower_map[key]
        return None
    return wanted if wanted in names else None


def _has_owner_link(owner: Owner, link: str) -> bool:
    fn = getattr(owner, "resolve_link_rel", None)
    if fn is not None:
        return fn(link) is not None
    return link in owner.link_rel


def _rigid_body_at_or_above(stage: Usd.Stage, path: str) -> Usd.Prim | None:
    fn = getattr(_u, "rigid_body_at_or_above")
    return fn(stage, path)


@dataclass
class _FallbackPartBuild:
    """``builds`` 辞書が渡されなかった場合にマニフェスト等から復元する最小の ``PartBuild`` 互換構造。"""

    spec: UrdfSource | UsdSource
    model: UrdfModel | None
    generated_usd: str
    fixed_usd: str


@dataclass
class OfficialStage:
    """公式 ``RobotAssembler`` による単一結合の再現結果を保持するデータクラス。"""

    stage: Usd.Stage
    top: str  # 親の prim パス（RobotAssembler に base として渡す）
    owners: list[Owner]  # 親 ("") と子（結合の名前）の 2 要素
    join: JoinResult  # probes.probe_stage にそのまま渡せる形式の結合結果


def measurable(j: JoinSpec) -> str | None:
    """結合 ``j`` を公式 ``RobotAssembler`` で再現・測定できるか判定する。

    Args:
        j: 対象の ``JoinSpec``。

    Returns:
        再現可能なら ``None``。再現できない場合はレポートに載せる理由文字列。副作用なし。
    """
    if "/" in j.parent_link or "/" in j.child_link:
        return '親か子のリンクが、結合した子の中のリンク("/" を含む書き方)なので、公式のままでは再現しない'
    if j.parent_prim and j.parent_prim != "." and (j.parent_prim.startswith("/") or "/" in j.parent_prim):
        return "parent_prim がパスか、結合した子の中のリンクなので、公式のままでは再現しない"
    return None


def _resolve_part_build(spec: PipelineSpec, src: UrdfSource | UsdSource, builds: dict[str, Any] | None) -> Any:
    """``builds`` キャッシュ、または ``_import/<stem>.manifest.json`` から部品の生成物パスを解決する。"""
    if builds and src.name in builds:
        return builds[src.name]
    fixed_usd = os.path.join(spec.output_dir, src.usda)
    if isinstance(src, UsdSource):
        return _FallbackPartBuild(
            spec=src,
            model=None,
            generated_usd=src.usd,
            fixed_usd=fixed_usd,
        )
    parts_dir = os.path.join(spec.output_dir, "_import")
    manifest_path = os.path.join(parts_dir, f"{src.stem}.manifest.json")
    generated_usd = os.path.join(parts_dir, src.stem, f"{os.path.splitext(os.path.basename(src.path))[0]}.usd")
    if os.path.exists(manifest_path):
        try:
            with open(manifest_path, encoding="utf-8") as fh:
                data = json.load(fh)
            if data.get("generated_usd"):
                generated_usd = data["generated_usd"]
        except OSError:
            pass
    return _FallbackPartBuild(
        spec=src,
        model=load_urdf(src.path),
        generated_usd=generated_usd,
        fixed_usd=fixed_usd,
    )


def _add(stage: Usd.Stage, path: str, build: Any) -> Usd.Prim:
    """未修正の部品 USD（``build.generated_usd``）をステージへ Reference 配置し、Physics variant を揃える。

    R06（Physics variant 未選択）自体は ``static_part`` 側で測るため、公式結合テストでは
    剛体が見える状態にしてから ``RobotAssembler`` の結合挙動（R02 / R04 / R05 / R07 / R08 / R09 / R11 / R12 / R14）を測る。
    """
    prim = stage.DefinePrim(path, "Xform")
    prim_path = getattr(build.spec, "prim_path", None)
    if prim_path:
        prim.GetReferences().AddReference(build.generated_usd, Sdf.Path(prim_path))
    else:
        prim.GetReferences().AddReference(build.generated_usd)
    var_sels = dict(getattr(build.spec, "variant_selections", ()) or ())
    for vs_name, choice in var_sels.items():
        vs = prim.GetVariantSets()
        if vs.HasVariantSet(vs_name):
            vs.GetVariantSet(vs_name).SetVariantSelection(choice)
    if "Physics" not in var_sels:
        is_usd = isinstance(build.spec, UsdSource) or build.model is None
        v = _physics_variant_choice(prim, build.spec.physics_variant, keep_existing=is_usd)
        if v and prim.GetVariantSets().HasVariantSet("Physics"):
            prim.GetVariantSet("Physics").SetVariantSelection(v)
    return prim


def _owner(stage: Usd.Stage, top: str, build: Any, prefix: str) -> Owner:
    """ステージ上に配置した部品 prim から ``Owner``（リンク・関節の相対パス地図）を構築する。"""
    if build.model is not None:
        links = _link_prims(stage, top, list(build.model.links))
    else:
        links = _usd_link_prims(stage, top)
    joints = _joint_prims(stage, top)
    rel = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else ("" if p == top else p)  # noqa: E731
    return Owner(
        prefix=prefix,
        rel_top="",
        model=build.model,
        link_rel={k: rel(v) for k, v in links.items()},
        joint_rel={k: rel(v) for k, v in joints.items() if v.startswith(top + "/")},
        urdf_name=build.spec.name,
    )


def build_official(
    spec: PipelineSpec,
    f: JoinFileSpec,
    j: JoinSpec,
    builds: dict[str, Any] | None = None,
) -> OfficialStage:
    """単一の結合 ``j`` を公式 ``RobotAssembler`` で実行したプローブ用ステージを作る。

    Args:
        spec: パイプライン全体仕様。
        f: 結合が属する ``JoinFileSpec``。
        j: 再現対象の ``JoinSpec``。
        builds: 構築済み ``PartBuild`` の辞書（省略時はマニフェストから復元）。

    Returns:
        構築された ``OfficialStage``。結合が例外で失敗した場合も例外は握りつぶして
        ``out.join.error`` に記録する（R05 / R08 の症状証拠として使うため）。

    副作用:
        ``runtime.new_probe_stage()`` でグローバルステージを作り直し、Kit フレームを進める。
    """
    from isaacsim.robot_setup.assembler import RobotAssembler

    parent_b = _resolve_part_build(spec, spec.root_source(f.parent), builds)
    child_b = _resolve_part_build(spec, spec.root_source(j.child), builds)
    stage = runtime.new_probe_stage()
    # /World を defaultPrim にして base をその子に置くことで、RobotAssembler を
    # 「元ファイルを直接編集するモード」ではなく「参照ベースで結合する通常モード」で動作させる
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, ROOT).GetPrim())
    top = f"{ROOT}/{f.stem}"
    _add(stage, top, parent_b)
    runtime.step(2)
    p_owner = _owner(stage, top, parent_b, "")
    res = JoinResult(jid=join_id(f, j), spec=j, parent_owner=p_owner, parent_link=j.parent_link)
    out = OfficialStage(stage=stage, top=top, owners=[p_owner], join=res)
    if not _has_owner_link(p_owner, j.parent_link):
        res.error = f"親のリンク {j.parent_link!r} が無い"
        return out
    res.parent_link_prim = p_owner.link_path(top, j.parent_link)
    if j.parent_prim == ".":
        place = top
    elif j.parent_prim and _has_owner_link(p_owner, j.parent_prim):
        place = p_owner.link_path(top, j.parent_prim)
    else:
        place = res.parent_link_prim
    child_path = f"{place}/{j.prim}"
    try:
        _add(stage, child_path, child_b)
    except Exception as exc:  # noqa: BLE001
        res.error = f"子を置けない: {type(exc).__name__}: {exc}"
        return out
    runtime.step(2)
    res.child_path = child_path
    rel = child_path[len(top) + 1 :]
    c_owner = _owner(stage, child_path, child_b, "").under(j.name, rel)
    out.owners.append(c_owner)
    res.child_owner, res.child_link = c_owner, j.child_link
    if _has_owner_link(c_owner, j.child_link):
        res.child_link_prim = c_owner.link_path(top, j.child_link)
    b0 = _rigid_body_at_or_above(stage, res.parent_link_prim)
    b1 = _rigid_body_at_or_above(stage, res.child_link_prim) if res.child_link_prim else None
    res.parent_body = str(b0.GetPath()) if b0 else None
    res.child_body = str(b1.GetPath()) if b1 else None
    asm = RobotAssembler()
    try:
        asm.begin_assembly(
            stage,
            top,
            res.parent_link_prim,
            child_path,
            res.child_link_prim or "",
            j.variant_set or "Join",
            j.name,
        )
        asm.assemble()
        runtime.run_async(asm.finish_assemble())
        runtime.step(3)
    except Exception as exc:  # noqa: BLE001
        res.error = f"{type(exc).__name__}: {str(exc).strip()[:500]}"
        try:
            asm.cancel_assembly()
            runtime.step(3)
        except Exception:  # noqa: BLE001
            pass
    # 公式 RobotAssembler が生成した固定関節パスを記録（probes.probe_stage の R08 / R14 検証用）
    res.joint_path = next(
        (str(p.GetPath()) for p in _joints_under(stage, child_path) if p.GetName().startswith("AssemblerFixedJoint")),
        None,
    )
    return out


def reassemble_count(o: OfficialStage) -> int | None:
    """R14 測定用: 同じ結合を公式 ``RobotAssembler`` で 2 回目実行した後の固定関節本数を返す。

    公式 ``RobotAssembler`` は固定関節名に毎回連番を振って新規作成するため、再結合すると
    ``AssemblerFixedJoint`` と ``AssemblerFixedJoint_01`` のように 2 本以上に増える。

    Args:
        o: 1 回目の結合が完了している ``OfficialStage``。

    Returns:
        子ツリー内の ``AssemblerFixedJoint*`` 固定関節本数。1 回目が失敗していた場合は ``None``。

    副作用:
        ``o.stage`` 上で ``RobotAssembler`` を追加実行し、Kit フレームを進める。
    """
    from isaacsim.robot_setup.assembler import RobotAssembler

    res = o.join
    if res.error or not res.child_path or not res.child_link_prim:
        return None
    try:
        asm = RobotAssembler()
        asm.begin_assembly(
            o.stage,
            o.top,
            res.parent_link_prim,
            res.child_path,
            res.child_link_prim,
            res.spec.variant_set or "Join",
            res.spec.name,
        )
        asm.assemble()
        runtime.run_async(asm.finish_assemble())
        runtime.step(3)
    except Exception:  # noqa: BLE001
        pass
    return len(
        [
            p
            for p in _joints_under(o.stage, res.child_path)
            if p.IsA(UsdPhysics.FixedJoint) and p.GetName().startswith("AssemblerFixedJoint")
        ]
    )


def probe_official(
    spec: PipelineSpec,
    f: JoinFileSpec,
    info: UsdaInfo | JoinSpec | dict[str, Any] | None = None,
    builds: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """結合ファイル ``f``（または単一 ``JoinSpec``）を公式 ``RobotAssembler`` で結合し、症状を測る。

    各結合について ``measurable`` を確認し、再現可能なものだけ ``build_official`` →
    ``probes.probe_stage``（親側関節のみ駆動・offset なし）→ ``reassemble_count`` を順に実行する。

    Args:
        spec: パイプライン全体仕様。
        f: 対象の ``JoinFileSpec``。
        info: ``UsdaInfo``・単一 ``JoinSpec``・または ``builds`` 辞書（互換性用）。
        builds: 構築済み ``PartBuild`` の辞書。

    Returns:
        ``official``・``official_join``・``official_skip``・``official_reassemble``・``joins`` を
        格納した辞書。

    副作用:
        結合ごとにグローバルプローブ用ステージを作り直し、Timeline を Play / Stop する。
    """
    from . import probes

    if isinstance(info, dict) and builds is None:
        builds = info
    joins_to_run = (info,) if isinstance(info, JoinSpec) else f.joins

    official: dict[str, dict[str, Any]] = {}
    official_join: dict[str, JoinResult] = {}
    official_skip: dict[str, str] = {}
    official_reassemble: dict[str, int | None] = {}

    for j in joins_to_run:
        jid = join_id(f, j)
        why = measurable(j)
        if why:
            official_skip[jid] = why
            continue
        o = build_official(spec, f, j, builds)
        official_join[jid] = o.join
        # 公式 RobotAssembler は TOML の offset_xyz / offset_rpy を受け取らないため include_offset=False で測る
        official[jid] = probes.probe_stage(
            o.stage,
            o.top,
            o.owners,
            [o.join],
            spec.probe,
            drive_prefixes={""},
            include_offset=False,
        )
        official_reassemble[jid] = reassemble_count(o)

    return {
        "official": official,
        "official_join": official_join,
        "official_skip": official_skip,
        "official_reassemble": official_reassemble,
        "joins": official,
    }


__all__ = [
    "ROOT",
    "OfficialStage",
    "measurable",
    "build_official",
    "reassemble_count",
    "probe_official",
]
