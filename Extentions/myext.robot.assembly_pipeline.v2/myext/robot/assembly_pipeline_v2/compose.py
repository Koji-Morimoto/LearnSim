"""結合ファイル（合成 ``.usda``）を生成するモジュール。

責務:
  親 ``.usda`` に 1 つ以上の子 ``.usda`` を結合し、Isaac Sim の ``RobotAssembler`` が行う処理に
  規則 R02 / R04 / R05 / R07 / R08 / R11 / R12 / R14 の修正を組み込んだ結合ファイルを毎回一から構築する。
  書き出しは差分がある場合のみ行い（``_write_if_changed``）、再実行時の冪等性を保つ。

結合 1 回で行う 7 段階の書き込み（各修正は仕様の ``[fixes]`` で規則単位に skip 可能）:
  1. Reference 配置: 親の指定 prim 配下に子 ``.usda`` を Reference として配置する
     （省略時は親リンク prim 配下。配置先が instanceable なら解除 = R05）。
  2. Transform 位置合わせ: 親リンクと子リンクの URDF 座標系（+ offset）が一致するローカル Transform を設定する
     （R04。skip 時は USD prim の位置で合わせる）。
  3. 固定関節生成: 親と子の剛体をつなぐ ``UsdPhysics.FixedJoint`` を固定パス ``<子>/JoinJoint`` に 1 本だけ作る（R14）。
     子のリンクに剛体が無ければ、固定関節でつながる子孫の剛体へ接続する（R08）。
     なお剛体を 1 つも持たない複合パーツ（例: バックパック等）は固定関節を作らず親剛体配下の複合要素として扱う。
  4. ワールド固定 joint 無効化: 子の外につながるワールド固定 joint を構造から探して無効化する（R02）。
  5. Articulation Root 削除: 子ツリー内の Articulation Root を Physics / Newton / Physx すべて外す（R07）。
  6. 衝突除外: 親の剛体と子の全剛体の組を ``UsdPhysics.FilteredPairsAPI`` に登録する（R11）。
  7. ロボットスキーマ更新: ``JoinJoint`` に ``IsaacJointAPI`` を付与し、結合ファイル最上位の
     ``isaac:physics:robotLinks`` / ``isaac:physics:robotJoints`` の末尾へ子を追加し、
     無効化したワールド固定 joint を子の ``robotJoints`` から削除する（R12）。

VariantSet 対応:
  ``variant_set`` を持つ結合は、結合ファイル最上位 prim の VariantSet 内に 1〜7 の書き込みを閉じ込める。
  複数 VariantSet 間で編集コンテキストが干渉しないよう、ループ前に全 VariantSet と ``"None"`` 選択肢を
  登録して既定値に揃え、各 ``VariantEditContext`` 終了直後に必ず既定選択へ戻す。

主な公開 API:
  - ``JOINT_NAME``: 結合用固定関節の固定名（``"JoinJoint"``）
  - ``CompositionError``: 結合処理中の構造・設定エラー
  - ``JoinResult``: 結合 1 回ぶんの解決結果・診断情報
  - ``FileComposition``: 結合ファイル 1 本ぶんの生成結果
  - ``compose_file(spec, f, infos=None)``: 結合ファイルを生成して保存する
  - ``resolve_prim(stage, top, owners, where)``: ``parent_prim`` 指定をステージ上の絶対パスへ解決する

パイプライン全体での位置づけ:
  ``pipeline.build_assembly`` の Step 4 で ``spec.file_order()`` の依存順に呼ばれる。
  出力された結合ファイルは ``descriptor.inspect_assembled_robot``・``probes.probe_stage``・
  ``kinematics_check.check_against_physx`` の入力になる。

手を入れやすい箇所:
  - 結合時の USD 書き込み内容（固定関節の属性、衝突除外の対象範囲、カスタムメタデータ等）を変えたい場合は
    ``_join`` の該当ステップ番号（1〜7）を編集する。
"""

from __future__ import annotations

import contextlib
import os
from dataclasses import dataclass, field
from typing import Any, Iterator

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .descriptor import CUSTOM_LAYER_KEY, _extract_authored_components
from .part_import import _write_if_changed, open_for_rewrite
from .sources import Owner, UsdaInfo, _register_component_rel, resolve
from .spec import JoinFileSpec, JoinSpec, PipelineSpec, join_id
from .urdf_model import pose_matrix
from .usd_util import (
    ALL_PRIMS,
    ARTICULATION_ROOT_APIS,
    ROBOT_JOINTS,
    ROBOT_LINKS,
    display_name,
    has_api,
    joint_local,
    rigid,
    rigid_bodies,
    rigid_body_at_or_above,
    rigid_descendants_via_fixed,
    set_local_pose,
    world,
    world_anchors,
)

JOINT_NAME = "JoinJoint"


class CompositionError(RuntimeError):
    """結合ファイル構築中に解決不能な prim・剛体・VariantSet 等を検出した際の例外。"""

    pass


@dataclass
class JoinResult:
    """結合 1 回の解決結果と診断情報。

    公式のまま（``official.py``）と修正後（``compose.py``）で同一の型を使い、
    ``probes.probe_stage`` と ``rules.evaluate`` が両者を同じロジックで比較できるようにする。
    """

    jid: str
    spec: JoinSpec
    child_path: str | None = None
    joint_path: str | None = None
    parent_body: str | None = None
    child_body: str | None = None
    parent_link_prim: str | None = None
    child_link_prim: str | None = None
    parent_owner: Owner | None = None  # 結合ファイルの中での親リンクの持ち主（top からの相対）
    child_owner: Owner | None = None  # 同じく子リンクの持ち主
    parent_link: str | None = None
    child_link: str | None = None
    conditions: tuple[tuple[str, str, str], ...] = ()  # この結合が現れる VariantSet の選択条件
    deactivated_anchors: list[str] = field(default_factory=list)
    remaining_anchors: list[str] = field(default_factory=list)
    removed_articulation_roots: list[str] = field(default_factory=list)
    uninstanced: list[str] = field(default_factory=list)
    body_from_fixed_descendant: str | None = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """全フィールドを辞書として返す。"""
        return {
            "jid": self.jid,
            "spec": self.spec,
            "child_path": self.child_path,
            "joint_path": self.joint_path,
            "parent_body": self.parent_body,
            "child_body": self.child_body,
            "parent_link_prim": self.parent_link_prim,
            "child_link_prim": self.child_link_prim,
            "parent_owner": self.parent_owner,
            "child_owner": self.child_owner,
            "parent_link": self.parent_link,
            "child_link": self.child_link,
            "conditions": self.conditions,
            "deactivated_anchors": list(self.deactivated_anchors),
            "remaining_anchors": list(self.remaining_anchors),
            "removed_articulation_roots": list(self.removed_articulation_roots),
            "uninstanced": list(self.uninstanced),
            "body_from_fixed_descendant": self.body_from_fixed_descendant,
            "error": self.error,
        }

    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)


@dataclass
class FileComposition:
    """結合ファイル 1 本ぶんの生成結果。

    辞書インターフェース（``__getitem__`` / ``keys`` / ``items`` 等）も備え、
    辞書として扱う呼び出し元との互換性を保つ。
    """

    name: str
    path: str
    top: str
    joins: list[JoinResult]
    info: UsdaInfo
    changed: bool

    def to_dict(self) -> dict[str, Any]:
        """全フィールドを辞書として返す。"""
        return {
            "name": self.name,
            "path": self.path,
            "top": self.top,
            "joins": self.joins,
            "info": self.info,
            "changed": self.changed,
        }

    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(("name", "path", "top", "joins", "info", "changed"))

    def __len__(self) -> int:
        return 6

    def __contains__(self, key: object) -> bool:
        return isinstance(key, str) and hasattr(self, key)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)

    def keys(self):
        return self.to_dict().keys()

    def values(self):
        return self.to_dict().values()

    def items(self):
        return self.to_dict().items()


def _rel(path: str, start: str) -> str:
    """USD の Reference 用に POSIX 形式の相対パス（``./`` 始まりまたは ``../`` 始まり）を作る。"""
    r = os.path.relpath(path, start).replace("\\", "/")
    return r if r.startswith(".") else "./" + r


def _relpath(top: str, path: str) -> str:
    """ステージ上の絶対パス ``path`` から ``top`` 基準の相対パスを切り出す。"""
    return path[len(top) + 1 :] if path.startswith(top + "/") else ("" if path == top else path)


def compose_file(
    spec: PipelineSpec,
    f: JoinFileSpec,
    infos: dict[str, UsdaInfo] | UsdaInfo | None = None,
) -> FileComposition:
    """1 つの ``JoinFileSpec`` から結合 ``.usda`` ファイルを一から構築して書き出す。

    結合ファイル内の各 ``JoinSpec`` を順に ``_join`` へ渡し、VariantSet 指定がある場合は
    事前にすべての VariantSet と ``"None"`` 選択肢を登録したうえで ``VariantEditContext`` 内に書き込む。
    構成要素マップ（``components_rel``）は ``customLayerData`` と ``top_prim`` の ``customData`` の両方に保存する。

    Args:
        spec: パイプライン全体仕様。
        f: 生成対象の結合ファイル仕様（``[[join_file]]``）。
        infos: 構築済み ``UsdaInfo`` の辞書（親・子 ``.usda`` の解決用）。省略時は ``sources.resolve`` で補完する。

    Returns:
        生成した結合ファイルの情報と各結合の ``JoinResult`` を格納した ``FileComposition``。

    副作用:
        既存ファイルと内容に差分がある場合のみ ``<output_dir>/<f.usda>`` をディスクへ書き出す。
        グローバルな ``omni.usd`` ステージや Timeline は変更しない。
    """
    if infos is None:
        infos_map: dict[str, UsdaInfo] = resolve(spec)
    elif isinstance(infos, UsdaInfo):
        infos_map = resolve(spec)
        infos_map[infos.name] = infos
    else:
        infos_map = infos

    os.makedirs(spec.output_dir, exist_ok=True)
    path = os.path.join(spec.output_dir, f.usda)
    layer = open_for_rewrite(path)
    stage = Usd.Stage.Open(layer)
    parent = infos_map[f.parent]
    src = Usd.Stage.Open(parent.path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src))
    top = f"/{f.stem}"
    top_prim = stage.DefinePrim(top, "Xform")
    stage.SetDefaultPrim(top_prim)

    join_vsets = {j.variant_set for j in f.joins if j.variant_set}
    src_dp = src.GetDefaultPrim()
    parent_vsets = set(src_dp.GetVariantSets().GetNames()) if (src_dp and src_dp.IsValid()) else set()
    colliding_vsets = join_vsets & parent_vsets

    if colliding_vsets:
        # 親の defaultPrim が結合側と同名の VariantSet を持つ場合、top_prim へ直接 Reference すると
        # top_prim の VariantSet 選択が親の内部 VariantSet にも波及して二重ロード・衝突が起きる。
        # そのため親を 1 段下の子 prim (<top>/<parent_stem>) に隔離して Reference する。
        parent_stem = f.parent[: -len(".usda")]
        parent_holder = stage.DefinePrim(f"{top}/{parent_stem}", "Xform")
        parent_holder.GetReferences().AddReference(_rel(parent.path, spec.output_dir))
        owners: list[Owner] = [o.under("", parent_stem) for o in parent.owners]
        components_rel: dict[str, str] = {k: f"{parent_stem}/{v}".strip("/") for k, v in parent.components_rel.items()}
        for k, v in _extract_authored_components(parent_holder).items():
            if k and v:
                components_rel.setdefault(k, f"{parent_stem}/{v}".strip("/"))
        default_variants: dict[tuple[str, str], str] = {
            (f"{parent_stem}/{r}".strip("/"), vs): val for (r, vs), val in parent.default_variants.items()
        }
        if has_api(parent_holder, "IsaacRobotAPI"):
            top_prim.ApplyAPI("IsaacRobotAPI")
            for rname in (ROBOT_LINKS, ROBOT_JOINTS):
                r = top_prim.GetRelationship(rname) or top_prim.CreateRelationship(rname, custom=False)
                r.AddTarget(Sdf.Path(f"{top}/{parent_stem}"), Usd.ListPositionBackOfAppendList)
    else:
        top_prim.GetReferences().AddReference(_rel(parent.path, spec.output_dir))
        owners = list(parent.owners)
        components_rel = dict(parent.components_rel)
        for k, v in _extract_authored_components(top_prim).items():
            if k and v:
                components_rel.setdefault(k, v)
        default_variants = dict(parent.default_variants)

    # 事前準備: ループの前にすべての VariantSet と選択肢（"None" + 宣言された variant）を登録し既定選択へ揃える。
    # 理由: 複数の VariantSet を持つ結合（例: left_hand / right_hand / Thor）では、後続の結合が
    # 親のステージ構造や Transform を読む際に、他 VariantSet の状態が既定値で安定している必要があるため。
    defaults: dict[str, str] = {}
    for j in f.joins:
        if j.variant_set and j.variant:
            v_def = f.default_variant(j.variant_set) if hasattr(f, "default_variant") else j.variant
            defaults.setdefault(j.variant_set, v_def or j.variant)
            default_variants.setdefault(("", j.variant_set), v_def or j.variant)
            vset = top_prim.GetVariantSets().AddVariantSet(j.variant_set)
            if "None" not in vset.GetVariantNames():
                vset.AddVariant("None")
            if j.variant not in vset.GetVariantNames():
                vset.AddVariant(j.variant)
    for vset_name, v_default in defaults.items():
        top_prim.GetVariantSet(vset_name).SetVariantSelection(v_default)

    # 本処理: 各結合を順に書き込む。VariantEditContext を抜けても USD の選択状態は自動で戻らないため、
    # finally ブロックで毎回すべての VariantSet を既定選択へ復元する。
    results: list[JoinResult] = []
    for j in f.joins:
        res = JoinResult(jid=join_id(f, j), spec=j)
        results.append(res)
        edit: Any = contextlib.nullcontext()
        if j.variant_set and j.variant:
            for other_set, v_default in defaults.items():
                top_prim.GetVariantSet(other_set).SetVariantSelection(v_default)
            vset = top_prim.GetVariantSet(j.variant_set)
            vset.SetVariantSelection(j.variant)
            edit = vset.GetVariantEditContext()
            res.conditions = (("", j.variant_set, j.variant),)
        try:
            try:
                with edit:
                    new_owners = _join(
                        stage,
                        spec,
                        top,
                        owners,
                        infos_map[j.child],
                        j,
                        res,
                        components_rel,
                        default_variants,
                    )
                owners.extend(new_owners)
            finally:
                if j.variant_set:
                    for vset_name, v_default in defaults.items():
                        top_prim.GetVariantSet(vset_name).SetVariantSelection(v_default)
        except CompositionError as exc:
            res.error = str(exc)
        except Exception as exc:  # noqa: BLE001
            res.error = f"{type(exc).__name__}: {exc}"

    for vs_name, v_default in defaults.items():
        top_prim.GetVariantSet(vs_name).SetVariantSelection(v_default)

    # Reference 経由で読まれると customLayerData は合成先から見えないことがあるため、
    # layer.customLayerData と top_prim.customData の両方に構成要素メタデータを書き込む。
    meta_payload = {"join_file": f.usda, "components": dict(components_rel)}
    layer.customLayerData = {CUSTOM_LAYER_KEY: meta_payload}
    top_prim.SetCustomDataByKey(CUSTOM_LAYER_KEY, meta_payload)

    changed = _write_if_changed(layer, path)
    return FileComposition(
        name=f.name,
        path=path,
        top=top,
        joins=results,
        info=UsdaInfo(
            name=f.name,
            path=path,
            official_path=None,
            owners=owners,
            components_rel=components_rel,
            default_variants=default_variants,
        ),
        changed=changed,
    )


def resolve_prim(stage: Usd.Stage, top: str, owners: list[Owner], where: str) -> str:
    """``parent_prim`` の指定をステージ上の絶対パスへ解決する。

    Args:
        stage: 構築中の USD ステージ。
        top: 結合ファイル最上位 prim のパス（例: ``"/robot"``）。
        owners: 現時点でステージ上に存在する部品 ``Owner`` 一覧。
        where: ``"."``（最上位 prim）、``"/"`` 始まりのステージ絶対パス、またはリンク表記。

    Returns:
        解決されたステージ上の prim パス。

    Raises:
        CompositionError: ``"/"`` 始まりのパスがステージ上に存在しない場合。
    """
    if where == ".":
        return top
    if where.startswith("/"):
        if not stage.GetPrimAtPath(where):
            raise CompositionError(f"prim {where} が無い")
        return where
    o, link = resolve(owners, where)
    return o.link_path(top, link)


def _link_world_pose(
    stage: Usd.Stage,
    top: str,
    owner: Owner,
    link: str,
    link_prim: str,
    *,
    use_r04: bool,
) -> np.ndarray:
    """リンクのワールド姿勢（4x4 同次変換行列）を返す。

    URDF 由来かつ R04 有効時は URDF 順運動学（``fk_zero``）から正確なリンク座標を求め、
    USD 部品（``owner.model is None``）または R04 skip 時は USD prim のワールド Transform を使う。
    """
    if use_r04 and owner.model is not None:
        return rigid(world(stage, owner.top(top))) @ owner.model.fk_zero(link)
    return rigid(world(stage, link_prim))


def _join(
    stage: Usd.Stage,
    spec: PipelineSpec,
    top: str,
    owners: list[Owner],
    child: UsdaInfo,
    j: JoinSpec,
    res: JoinResult,
    components_rel: dict[str, str],
    default_variants: dict[tuple[str, str], str],
) -> list[Owner]:
    """結合 1 回ぶんの 7 段階の書き込みを ``stage`` 上で実行し、新しく追加された ``Owner`` 群を返す。

    書き込み順序の意図:
      Step 1（Reference 配置）と Step 2（Transform 位置合わせ）を先に完了させることで、
      後続の Step 3（固定関節の localPos/localRot 算出）が最終ワールド姿勢を直接参照できる。
      また Step 4（子のワールド固定 joint 無効化）と Step 5（Articulation Root 削除）を
      固定関節生成より前に済ませ、剛体を持たない複合パーツはここで早期 return して固定関節を作らない。
    """
    # ---- 親のリンクを解決
    try:
        p_owner, p_link = resolve(owners, j.parent_link)
    except KeyError as exc:
        raise CompositionError(f"親のリンク: {exc}") from None
    if p_owner.resolve_link_rel(p_link) is None:
        raise CompositionError(f"親のリンク {j.parent_link!r} の prim が見つからない")
    res.parent_owner, res.parent_link = p_owner, p_link
    res.parent_link_prim = p_owner.link_path(top, p_link)

    # ---- Step 1. 子を Reference で配置（R05: 配置先が instanceable なら解除）
    place = resolve_prim(stage, top, owners, j.parent_prim) if j.parent_prim else res.parent_link_prim
    place_prim = stage.GetPrimAtPath(place)
    if place_prim.IsInstanceable():
        if not spec.fix_applied("R05"):
            raise CompositionError(f"R05: 子を置く prim {place} が instanceable なので子を作れない(修正を skip)")
        # USD の instanceable prim の配下には直接子 prim を定義できないため解除する
        place_prim.SetInstanceable(False)
        res.uninstanced.append(place)
    child_path = f"{place}/{j.prim}"
    res.child_path = child_path
    cp = stage.DefinePrim(child_path, "Xform")
    if not cp.IsActive():
        cp.SetActive(True)
    cp.GetReferences().AddReference(_rel(child.path, spec.output_dir))
    for vs_name, choice in j.select:
        vs = cp.GetVariantSets()
        if not vs.HasVariantSet(vs_name):
            raise CompositionError(f"子 {j.child} に VariantSet {vs_name!r} が無い")
        vset = vs.GetVariantSet(vs_name)
        vnames = list(vset.GetVariantNames())
        if choice not in vnames:
            raise CompositionError(
                f"子 {j.child} の VariantSet {vs_name!r} に選択肢 {choice!r} が無い。ある選択肢: {vnames}"
            )
        vset.SetVariantSelection(choice)
    rel = _relpath(top, child_path)

    # 子の構成要素テーブル（メモリ上の child.components_rel と USD レイヤーの customLayerData / customData を統合）
    child_subcomponents = dict(child.components_rel)
    for k, v in _extract_authored_components(cp).items():
        if k and v:
            child_subcomponents.setdefault(k, v)
    _register_component_rel(components_rel, rel, j.prim, child_subcomponents)

    for (sub_rel, vs_name), v_def in child.default_variants.items():
        nested_rel = f"{rel}/{sub_rel}".rstrip("/") if sub_rel else rel
        default_variants.setdefault((nested_rel, vs_name), v_def)
    for vs_name, choice in j.select:
        default_variants[(rel, vs_name)] = choice

    try:
        # 子のリンクは子の usda 基準（接頭辞なし = 子の最上位部品）で解決し、結合ファイル内の Owner へ写像する
        c_src, c_link = resolve(child.owners, j.child_link)
    except KeyError as exc:
        raise CompositionError(f"子のリンク: {exc}") from None

    offset = pose_matrix(j.offset_xyz, j.offset_rpy)
    new_owners = [
        o.under(
            j.name,
            rel,
            res.conditions,
            parent_prefix=p_owner.prefix if o is c_src else None,
            parent_link=p_link if o is c_src else None,
            child_link=c_link if o is c_src else None,
            join_offset=offset if o is c_src else None,
        )
        for o in child.owners
    ]
    c_owner = new_owners[child.owners.index(c_src)]
    if c_owner.resolve_link_rel(c_link) is None:
        raise CompositionError(f"子のリンク {j.child_link!r} の prim が見つからない")
    res.child_owner, res.child_link = c_owner, c_link
    res.child_link_prim = c_owner.link_path(top, c_link)

    # ---- Step 2. 位置合わせ（R04: URDF のリンク座標系基準で親リンク + offset と子リンクを一致させる）
    # 一旦単位行列に戻してから「子ルートから見た子マウントリンクの相対姿勢 mount_in_child」を計測し、
    # 親配置先ローカルの最終姿勢 inv(W_place) @ target @ inv(mount_in_child) を設定する。
    set_local_pose(cp, np.eye(4))
    use_r04 = spec.fix_applied("R04")
    target = _link_world_pose(stage, top, p_owner, p_link, res.parent_link_prim, use_r04=use_r04) @ offset
    mount_in_child = (
        np.linalg.inv(rigid(world(stage, child_path)))
        @ _link_world_pose(stage, top, c_owner, c_link, res.child_link_prim, use_r04=use_r04)
    )
    set_local_pose(cp, np.linalg.inv(rigid(world(stage, place))) @ target @ np.linalg.inv(mount_in_child))

    # ---- Step 3 の準備: 固定関節でつなぐ親剛体 b0 と子剛体 b1 を探索
    b0 = rigid_body_at_or_above(stage, res.parent_link_prim)
    if b0 is None:
        raise CompositionError(f"親のリンク {res.parent_link_prim} の位置にも上にも剛体が無い")
    child_rbs = rigid_bodies(stage, child_path)
    has_child_rb = bool(child_rbs)
    b1: Usd.Prim | None = None
    if has_child_rb:
        b1 = rigid_body_at_or_above(stage, res.child_link_prim)
        if b1 is not None:
            # 親側へ遡って拾った剛体は子剛体として無効にする（子ツリー内の剛体のみ許可）
            b1p = str(b1.GetPath())
            if b1p != child_path and not b1p.startswith(child_path + "/"):
                b1 = None
        if b1 is None and spec.fix_applied("R08"):
            # R08: 子の接続リンク自体が仮想リンク等で剛体を持たない場合、
            # fixed joint で剛結合されている子孫リンクの剛体を代表ボディとして採用する。
            if c_owner.model is not None:
                for d in c_owner.model.fixed_descendants(c_link):
                    if d not in c_owner.link_rel:
                        continue
                    rb = rigid_body_at_or_above(stage, c_owner.link_path(top, d))
                    if rb is not None:
                        rbp = str(rb.GetPath())
                        if rbp == child_path or rbp.startswith(child_path + "/"):
                            b1 = rb
                            res.body_from_fixed_descendant = d
                            break
            else:
                for rb in rigid_descendants_via_fixed(stage, child_path, res.child_link_prim):
                    b1 = rb
                    res.body_from_fixed_descendant = display_name(rb)
                    break
    res.parent_body, res.child_body = str(b0.GetPath()), (str(b1.GetPath()) if b1 is not None else None)

    # ---- Step 4. 子のワールド固定 joint を構造探索で無効化（R02 / R12）
    for anc in world_anchors(stage, child_path, scope=child_path):
        if spec.fix_applied("R02") or not anc["both_targets_authored"]:
            # R02 skip 時でも片側が空の joint は公式 RobotAssembler と同様に無効化する
            stage.GetPrimAtPath(anc["path"]).SetActive(False)
            res.deactivated_anchors.append(anc["path"])
            if spec.fix_applied("R12"):
                # RobotPoser は active=False の joint も robotJoints 関係から運動学木に含めるため、
                # 関係ターゲット自体からも明示的に取り除く。
                for p in Usd.PrimRange(cp, ALL_PRIMS):
                    r = p.GetRelationship(ROBOT_JOINTS) if has_api(p, "IsaacRobotAPI") else None
                    if r and Sdf.Path(anc["path"]) in r.GetTargets():
                        r.RemoveTarget(Sdf.Path(anc["path"]))
    res.remaining_anchors = [x["path"] for x in world_anchors(stage, child_path, scope=child_path)]

    # ---- Step 5. 子の Articulation Root を除去（R07: Physics / Newton / Physx のすべてを外す）
    # NewtonArticulationRootAPI は PhysicsArticulationRootAPI を内包するため、R07 では全種を外さないと
    # ネストした Articulation Root が残ってしまう。
    apis_to_remove = ARTICULATION_ROOT_APIS if spec.fix_applied("R07") else ("PhysicsArticulationRootAPI",)
    seen_art: set[str] = set()
    for p in Usd.PrimRange(cp, ALL_PRIMS):
        if p.IsInstanceProxy():
            continue
        if not any(has_api(p, t) for t in apis_to_remove):
            continue
        for t in apis_to_remove:
            p.RemoveAPI(t)
        pp = str(p.GetPath())
        if pp not in seen_art:
            seen_art.add(pp)
            res.removed_articulation_roots.append(pp)

    # 剛体を持たない子（例: G1 Thor バックパックのように PhysicsMassAPI + collider のみで
    # 親剛体に複合結合するパーツ）は、PhysX が親剛体の複合形状・質量として自動的に統合する。
    # そのため JoinJoint・ロボットスキーマ・FilteredPairs は作らず、配置と Transform のみで完了とする。
    if not has_child_rb:
        return new_owners

    # ---- Step 3. 固定関節 <子>/JoinJoint を生成（R14: 固定パスで 1 本に保ち増殖を防ぐ）
    body1_target = str(b1.GetPath()) if b1 is not None else res.child_link_prim  # R08 skip 時: 剛体の無い prim をそのまま使う
    jpath = f"{child_path}/{JOINT_NAME}"
    joint = UsdPhysics.FixedJoint.Define(stage, jpath)
    joint.CreateBody0Rel().SetTargets([b0.GetPath()])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_target)])
    cache = UsdGeom.XformCache()
    w0 = rigid(world(stage, b0.GetPath(), cache))
    w1 = rigid(world(stage, body1_target, cache))
    mount_world = target if (use_r04 or c_owner.model is None) else rigid(world(stage, res.child_link_prim, cache))
    p0, q0 = joint_local(np.linalg.inv(w0) @ target)
    p1, q1 = joint_local(np.linalg.inv(w1) @ mount_world)
    joint.CreateLocalPos0Attr(p0)
    joint.CreateLocalRot0Attr(q0)
    joint.CreateLocalPos1Attr(p1)
    joint.CreateLocalRot1Attr(q1)
    res.joint_path = jpath

    # ---- Step 7. ロボットスキーマ更新（R12: IsaacJointAPI 付与 + robotLinks / robotJoints 末尾追加）
    if spec.fix_applied("R12"):
        joint.GetPrim().ApplyAPI("IsaacJointAPI")
    top_prim = stage.GetPrimAtPath(top)
    for name in (ROBOT_LINKS, ROBOT_JOINTS):
        r = top_prim.GetRelationship(name) or top_prim.CreateRelationship(name, custom=False)
        # 末尾に足す理由: 先頭に追加すると RobotPoser が子のリンクを運動学木の根とみなしてしまい、
        # Stop 中の teleport 時に子ではなく親ロボット側が逆方向に動いてしまうため。
        position = Usd.ListPositionBackOfAppendList if spec.fix_applied("R12") else Usd.ListPositionBackOfPrependList
        r.AddTarget(Sdf.Path(child_path), position)

    # ---- Step 6. 親剛体と子の全剛体の衝突を除外（R11: 接続部での不要な干渉・跳ね返りを防ぐ）
    if spec.fix_applied("R11"):
        r = UsdPhysics.FilteredPairsAPI.Apply(b0).CreateFilteredPairsRel()
        for rb in child_rbs:
            r.AddTarget(Sdf.Path(rb))
    return new_owners


__all__ = [
    "JOINT_NAME",
    "CompositionError",
    "JoinResult",
    "FileComposition",
    "compose_file",
    "resolve_prim",
]
