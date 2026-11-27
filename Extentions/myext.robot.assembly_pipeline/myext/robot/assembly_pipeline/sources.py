"""usda 内部の構造マップ(`UsdaInfo` / `Owner`)の構築、リンク解決、および複合 URDF 順運動学(FK)。

責務:
  - 任意の usda(単体パーツまたは多段の結合ファイル)が「どの URDF / USD 部品(`Owner`)を、
    どの相対 prim パス・どの VariantSet 条件で内包しているか」を表す構造マップ `UsdaInfo` を管理する。
  - スコープ付きリンク名(例: `"flange"`, `"arm1/gripper/finger_l"`)から所有者 `Owner` とリンク名を解決する。
  - ステージを開かずに仕様だけで静的解決する経路(`static_resolve`)と、生成済みステージから
    実 prim パスを反映する経路(`resolve`)の両方を提供する。
  - URDF 由来の結合木全体に対して、VariantSet 選択を考慮した順運動学(`compute_composite_fk`)を計算する。

主な入出力・公開 API:
  - データ構造: `Owner`, `UsdaInfo`
  - 解決 API: `resolve(...)`, `static_resolve(...)`
  - 順運動学: `compute_composite_fk(info, q_by_scoped_joint=None, active_variants=None)`
  - 内部共有補助: `_register_component_rel`（`compose.py` が構成要素メタデータ登録に利用）

呼ばれ元:
  - `compose.py`, `official.py`, `probes.py`, `kinematics_check.py`, `rules.py`, `pipeline.py`

概念整理(`Owner` のフィールド):
  - `prefix`     : リンク/関節名のスコープ前置き(ルート部品は `""`、子結合は `"arm1"`、入れ子は `"arm1/gripper"`)。
  - `rel_top`    : その usda の defaultPrim から見た、この部品ルート prim の相対パス。
  - `model`      : URDF 由来なら `UrdfModel`、既存 USD 由来なら `None`。
  - `link_rel`   : `{リンク名: 部品ルートからの相対 prim パス}`。
  - `joint_rel`  : `{関節名: 部品ルートからの相対 prim パス}`。
  - `conditions` : この部品がステージ上に現れるための VariantSet 条件
                   `((VariantSet を持つ prim の相対パス, VariantSet 名, 選択肢), ...)`。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from .urdf_model import UrdfModel, load_urdf, pose_matrix

if TYPE_CHECKING:
    from pxr import Usd


@dataclass(frozen=True)
class Owner:
    """usda 内に含まれる 1 つのリーフ部品(URDF または USD)の配置・リンク対応・結合親情報を表す。"""

    prefix: str
    rel_top: str
    model: UrdfModel | None = None
    link_rel: dict[str, str] = field(default_factory=dict, compare=False, hash=False)
    joint_rel: dict[str, str] = field(default_factory=dict, compare=False, hash=False)
    urdf_name: str = ""
    conditions: tuple[tuple[str, str, str], ...] = ()
    parent_prefix: str | None = field(default=None, compare=False, hash=False)
    parent_link: str | None = field(default=None, compare=False, hash=False)
    child_link: str | None = field(default=None, compare=False, hash=False)
    join_offset: np.ndarray | None = field(default=None, repr=False, compare=False, hash=False)

    def under(
        self,
        prefix: str,
        rel: str,
        conditions: tuple[tuple[str, str, str], ...] = (),
        *,
        parent_prefix: str | None = None,
        parent_link: str | None = None,
        child_link: str | None = None,
        join_offset: np.ndarray | None = None,
    ) -> "Owner":
        """この `Owner` を別の結合ファイルへ子として埋め込んだときの新しい `Owner` を返す。

        なぜ `self.prefix == ""` で分岐するか:
          - `self.prefix == ""` は子 usda 自身のルート部品なので、今回の結合で新しく親側へ接続される
            (`parent_prefix`, `parent_link`, `child_link`, `join_offset` を今回の結合情報で埋める)。
          - `self.prefix != ""` は子 usda の内部に既にネストしていた孫部品なので、
            子内部での結合関係(`parent_link`, `child_link`, `join_offset`)はそのまま保ち、
            親のスコープ前置き(`parent_prefix`)にだけ外側の `prefix` を足す。
        """
        moved = tuple((_join(rel, c[0]), c[1], c[2]) for c in self.conditions)
        if self.prefix == "":
            new_parent_prefix = parent_prefix if parent_prefix is not None else self.parent_prefix
            new_parent_link = parent_link if parent_link is not None else self.parent_link
            new_child_link = child_link if child_link is not None else self.child_link
            new_join_offset = join_offset if join_offset is not None else self.join_offset
        else:
            new_parent_prefix = _join_prefix(prefix, self.parent_prefix or "")
            new_parent_link = self.parent_link
            new_child_link = self.child_link
            new_join_offset = self.join_offset
        return Owner(
            prefix=_join_prefix(prefix, self.prefix),
            rel_top=_join(rel, self.rel_top),
            model=self.model,
            link_rel=dict(self.link_rel),
            joint_rel=dict(self.joint_rel),
            urdf_name=self.urdf_name,
            conditions=conditions + moved,
            parent_prefix=new_parent_prefix,
            parent_link=new_parent_link,
            child_link=new_child_link,
            join_offset=new_join_offset,
        )

    # ---- 絶対パス変換(`top` = そのステージ上での usda の defaultPrim パス)
    def top(self, top: str) -> str:
        """ステージ上でのこの部品ルート prim の絶対パスを返す。"""
        return _join(top, self.rel_top)

    def resolve_link_rel(self, link: str) -> str | None:
        """リンク名 `link` を部品ルートからの相対 prim パスへ解決する(無ければ `None`)。

        USD 由来部品(`model is None`)では、表示名だけでなく `"sub/frame"` のような
        相対パス直接指定も許すため `link_rel.values()` との一致もフォールバック確認する。
        """
        if link in self.link_rel:
            return self.link_rel[link]
        if self.model is None:
            norm = link.strip("/")
            for rel_p in self.link_rel.values():
                if rel_p == norm:
                    return rel_p
        return None

    def has_link(self, link: str) -> bool:
        """この部品がリンク `link` を所有していれば `True`。"""
        if self.model is not None:
            return link in self.model.links
        return self.resolve_link_rel(link) is not None

    def link_path(self, top: str, link: str) -> str:
        """ステージ上での `link` prim の絶対パスを返す(見つからなければ `KeyError`)。"""
        rel = self.resolve_link_rel(link)
        if rel is None:
            raise KeyError(link)
        return _join(self.top(top), rel)

    def joint_path(self, top: str, joint: str) -> str:
        """ステージ上での `joint` prim の絶対パスを返す。"""
        return _join(self.top(top), self.joint_rel[joint])

    def scoped_link(self, link: str) -> str:
        """`prefix` を前に付けたスコープ付きリンク名(例: `"arm1/flange"`)を返す。"""
        return f"{self.prefix}/{link}" if self.prefix else link

    def scoped_joint(self, joint: str, sep: str = "/") -> str:
        """`prefix` を前に付けたスコープ付き関節名を返す(`sep` で区切り文字を指定可能)。"""
        return f"{self.prefix}{sep}{joint}" if self.prefix else joint

    def present(self, stage: Usd.Stage, top: str) -> bool:
        """`stage` の現在の VariantSet 選択状態で、この `Owner` が実際に展開されていれば `True`。"""
        for rel, vs_name, choice in self.conditions:
            prim = stage.GetPrimAtPath(_join(top, rel))
            if not prim:
                return False
            vs = prim.GetVariantSets()
            if not vs.HasVariantSet(vs_name) or vs.GetVariantSet(vs_name).GetVariantSelection() != choice:
                return False
        return bool(stage.GetPrimAtPath(self.top(top)))


@dataclass
class UsdaInfo:
    """1 つの usda ファイル(リーフ部品または結合ファイル)の解決済み構造情報。

    属性:
      name            : usda ファイル名(例: `"arm_unit.usda"`)。
      path            : 修正後 usda の絶対パス。
      official_path   : 比較対象となる公式のままの USD パス(結合ファイルの場合は `None`)。
      owners          : 内包する全 `Owner` のリスト(必ず親→子の結合順)。
      components_rel  : `{構成要素名: defaultPrim からの相対 prim パス}`。
      default_variants: `{(prim 相対パス, VariantSet 名): 既定選択肢}`。
    """

    name: str
    path: str
    official_path: str | None
    owners: list[Owner]
    components_rel: dict[str, str] = field(default_factory=dict)
    default_variants: dict[tuple[str, str], str] = field(default_factory=dict)

    def resolve(self, scoped: str) -> tuple[Owner, str]:
        """スコープ付きリンク名 `scoped` を `(所有 Owner, 部品内リンク名)` へ解決する。"""
        return _resolve_in_owners(self.owners, scoped)

    def compute_composite_fk(
        self,
        q_by_scoped_joint: dict[str, float] | None = None,
        active_variants: dict[str, str] | None = None,
    ) -> dict[str, np.ndarray]:
        """この `UsdaInfo` に対して `compute_composite_fk` を呼び出すショートカット。"""
        return compute_composite_fk(self, q_by_scoped_joint=q_by_scoped_joint, active_variants=active_variants)


def _resolve_in_owners(owners: list[Owner], scoped: str) -> tuple[Owner, str]:
    """スコープ付きリンク名(`"flange"`, `"arm1/gripper/finger_l"` 等)を `(Owner, リンク名)` へ解決する。

    なぜ最長プレフィックス一致を取るか:
      入れ子の結合(例: `""`, `"arm1"`, `"arm1/gripper"`)や、USD 部品内のスラッシュ付き相対リンク名が混在しても、
      最も具体的に一致する `Owner` を優先して曖昧さをなくすため。
    """
    best: tuple[Owner, str] | None = None
    for o in owners:
        if o.prefix == "":
            rest = scoped
        elif scoped.startswith(o.prefix + "/"):
            rest = scoped[len(o.prefix) + 1 :]
        else:
            continue
        if o.has_link(rest) and (best is None or len(o.prefix) > len(best[0].prefix)):
            best = (o, rest)
    if best is None:
        known = sorted({(o.prefix + "/" if o.prefix else "") + "<リンク名>" for o in owners})
        raise KeyError(f"リンク {scoped!r} が見つからない。書き方: {known}")
    return best


def resolve(first: Any, second: Any = None) -> Any:
    """用途に応じて 2 通りの呼び出しに対応する多態ヘルパー。

    呼び出し形式:
      1. `resolve(owners_or_info, scoped: str) -> tuple[Owner, str]`
         `UsdaInfo` または `list[Owner]` の中からスコープ付きリンク名を解決する。
      2. `resolve(spec: PipelineSpec, stages: dict[str, Usd.Stage] | None = None) -> dict[str, UsdaInfo]`
         全 usda の `UsdaInfo` 辞書を構築する(必要に応じ生成済みステージを開いて実 prim パスを取得する)。
    """
    if isinstance(second, str):
        if isinstance(first, UsdaInfo):
            return first.resolve(second)
        return _resolve_in_owners(list(first), second)
    return _resolve_spec_infos(first, stages=second)


def static_resolve(spec: Any, usda_name: str | None = None, scoped: str | None = None) -> Any:
    """ステージを開かず、仕様と URDF だけで解決する 2 通り対応ヘルパー。

    呼び出し形式:
      1. `static_resolve(spec, usda_name: str, scoped: str) -> tuple[str, str]`
         リンクの書き方を `(所有リーフ usda 名, リンク名)` へ静的に解決する。
      2. `static_resolve(spec) -> dict[str, UsdaInfo]`
         ステージを一切開かずに全 usda の推定 `UsdaInfo` 辞書を構築する(`rules.py` の事前チェック用)。
    """
    if usda_name is not None and scoped is not None:
        return spec.resolve_link(usda_name, scoped)
    return _resolve_spec_infos(spec, stages=None, allow_open_stage=False)


def _resolve_spec_infos(
    spec: Any,
    stages: dict[str, Usd.Stage] | None = None,
    *,
    allow_open_stage: bool = True,
) -> dict[str, UsdaInfo]:
    """`PipelineSpec` の全リーフ部品と結合ファイルを依存順にたどり `{usda名: UsdaInfo}` を構築する。

    処理の流れ:
      1. `spec.urdfs`: URDF を読み込み、実ステージが利用可能なら `link_prims`/`joint_prims` で実パスを、
         オフライン(`allow_open_stage=False`)なら URDF Importer の標準パス規約(`Geometry/<link>`, `Joints/<joint>`)を詰める。
      2. `spec.usds`: 実ステージから `usd_link_prims`/`joint_prims` でリンク・関節パスを収集する。
      3. `spec.file_order()`: 親 `UsdaInfo` を土台に各結合の子 `Owner`・`components_rel`・`default_variants` を合成する。
    """
    infos: dict[str, UsdaInfo] = {}
    for u in spec.urdfs:
        model = load_urdf(u.path)
        fixed_usd = os.path.join(spec.output_dir, u.usda)
        gen_usd = os.path.join(spec.output_dir, "_import", u.stem, f"{u.stem}.usd")
        st: Usd.Stage | None = None
        if stages and u.name in stages:
            st = stages[u.name]
        elif allow_open_stage and os.path.exists(fixed_usd):
            try:
                from pxr import Usd

                st = Usd.Stage.Open(fixed_usd)
            except Exception:  # noqa: BLE001
                st = None
        if st is not None and st.GetDefaultPrim():
            from .usd_util import joint_prims, link_prims

            top = str(st.GetDefaultPrim().GetPath())
            rel_fn = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else ("" if p == top else p)  # noqa: E731
            links = {k: rel_fn(v) for k, v in link_prims(st, top, list(model.links)).items()}
            joints = {k: rel_fn(v) for k, v in joint_prims(st, top).items()}
        else:
            links = {k: f"Geometry/{k}" if k != model.root_link else "" for k in model.links}
            joints = {k: f"Joints/{k}" for k in model.joints}
        infos[u.name] = UsdaInfo(
            name=u.name,
            path=fixed_usd,
            official_path=gen_usd if os.path.exists(gen_usd) else None,
            owners=[
                Owner(
                    prefix="",
                    rel_top="",
                    model=model,
                    link_rel=links,
                    joint_rel=joints,
                    urdf_name=u.name,
                )
            ],
        )

    for u in getattr(spec, "usds", ()):
        fixed_usd = os.path.join(spec.output_dir, u.usda)
        st = None
        if stages and u.name in stages:
            st = stages[u.name]
        elif allow_open_stage:
            candidate = fixed_usd if os.path.exists(fixed_usd) else (u.usd if "://" not in u.usd and os.path.exists(u.usd) else None)
            if candidate:
                try:
                    from pxr import Usd

                    st = Usd.Stage.Open(candidate)
                except Exception:  # noqa: BLE001
                    st = None
        if st is not None and st.GetDefaultPrim():
            from .usd_util import joint_prims, usd_link_prims

            top = str(st.GetDefaultPrim().GetPath())
            rel_fn = lambda p: p[len(top) + 1 :] if p.startswith(top + "/") else ("" if p == top else p)  # noqa: E731
            links = {k: rel_fn(v) for k, v in usd_link_prims(st, top).items()}
            joints = {k: rel_fn(v) for k, v in joint_prims(st, top).items()}
        else:
            links = {}
            joints = {}
        infos[u.name] = UsdaInfo(
            name=u.name,
            path=fixed_usd,
            official_path=u.usd,
            owners=[
                Owner(
                    prefix="",
                    rel_top="",
                    model=None,
                    link_rel=links,
                    joint_rel=joints,
                    urdf_name=u.name,
                )
            ],
        )

    for f in spec.file_order():
        parent = infos[f.parent]
        owners: list[Owner] = list(parent.owners)
        comp_rel: dict[str, str] = dict(parent.components_rel)
        def_vars: dict[tuple[str, str], str] = dict(parent.default_variants)
        # 親パーツを 1 段下の子 prim (<holder_name>) に配置する条件:
        #   1. TOML の [[join_file]] で parent_prim_name が明示指定されている場合
        #   2. 親 usda の defaultPrim が結合側と同名の VariantSet を持っていて衝突回避が必要な場合
        # `compose.py` と同一のオフセットを適用し、Owner.rel_top と実際の結合 USD パスを一致させる。
        join_vsets = {j.variant_set for j in f.joins if j.variant_set}
        holder_name: str | None = f.parent_prim_name
        if holder_name is None and join_vsets and allow_open_stage and os.path.exists(parent.path):
            try:
                from pxr import Usd

                pst = Usd.Stage.Open(parent.path)
                pdp = pst.GetDefaultPrim() if pst else None
                parent_vsets = set(pdp.GetVariantSets().GetNames()) if (pdp and pdp.IsValid()) else set()
            except Exception:  # noqa: BLE001
                parent_vsets = set()
            if join_vsets & parent_vsets:
                holder_name = f.parent[: -len(".usda")]
        if holder_name is not None:
            owners = [o.under("", holder_name) for o in owners]
            parent_sub = dict(comp_rel)
            if f.parent_prim_name is not None:
                comp_rel = {}
                _register_component_rel(comp_rel, holder_name, holder_name, parent_sub)
            else:
                comp_rel = {k: _join(holder_name, v) for k, v in parent_sub.items()}
            def_vars = {(_join(holder_name, r), vs): val for (r, vs), val in def_vars.items()}
        for j in f.joins:
            if j.variant_set and j.variant:
                v_def = f.default_variant(j.variant_set) if hasattr(f, "default_variant") else j.variant
                def_vars.setdefault(("", j.variant_set), v_def or j.variant)
        for j in f.joins:
            try:
                p_owner, p_link = _resolve_in_owners(owners, j.parent_link)
            except KeyError:
                # オフライン静的解決(`allow_open_stage=False`)では USD 部品のリンク一覧が空のため仮解決を許す。
                if not allow_open_stage and owners and owners[0].model is None:
                    p_owner, p_link = owners[0], j.parent_link
                else:
                    raise
            parent_link_rel = _join(p_owner.rel_top, p_owner.resolve_link_rel(p_link) or "")
            if j.parent_prim == ".":
                place_rel = ""
            elif j.parent_prim and j.parent_prim.startswith("/"):
                place_rel = j.parent_prim.lstrip("/")
                top_prefix = f"{f.stem}/"
                if place_rel.startswith(top_prefix):
                    place_rel = place_rel[len(top_prefix) :]
            elif j.parent_prim:
                try:
                    po, pl = _resolve_in_owners(owners, j.parent_prim)
                except KeyError:
                    if not allow_open_stage and owners and owners[0].model is None:
                        po, pl = owners[0], j.parent_prim
                    else:
                        raise
                place_rel = _join(po.rel_top, po.resolve_link_rel(pl) or "")
            else:
                place_rel = parent_link_rel
            child_rel = _join(place_rel, j.prim)
            cond: tuple[tuple[str, str, str], ...] = (("", j.variant_set, j.variant),) if (j.variant_set and j.variant) else ()
            child_info = infos[j.child]
            try:
                c_src, c_link = _resolve_in_owners(child_info.owners, j.child_link)
            except KeyError:
                if not allow_open_stage and child_info.owners and child_info.owners[0].model is None:
                    c_src, c_link = child_info.owners[0], j.child_link
                else:
                    raise
            offset = pose_matrix(j.offset_xyz, j.offset_rpy)
            for (sub_rel, vs_name), v_def in child_info.default_variants.items():
                def_vars.setdefault((_join(child_rel, sub_rel), vs_name), v_def)
            for vs_name, choice in j.select:
                def_vars[(child_rel, vs_name)] = choice
            new_owners = [
                o.under(
                    j.name,
                    child_rel,
                    cond,
                    parent_prefix=p_owner.prefix if o is c_src else None,
                    parent_link=p_link if o is c_src else None,
                    child_link=c_link if o is c_src else None,
                    join_offset=offset if o is c_src else None,
                )
                for o in child_info.owners
            ]
            owners.extend(new_owners)
            _register_component_rel(comp_rel, child_rel, j.prim, child_info.components_rel)
        infos[f.name] = UsdaInfo(
            name=f.name,
            path=os.path.join(spec.output_dir, f.usda),
            official_path=None,
            owners=owners,
            components_rel=comp_rel,
            default_variants=def_vars,
        )
    return infos


def _register_component_rel(
    comp_rel: dict[str, str],
    child_rel: str,
    child_prim_name: str,
    child_subcomponents: dict[str, str],
) -> str:
    """結合した子の prim とその配下の構成要素を `{構成要素名: top からの相対パス}` に登録する。

    既存の構成要素パスから最長一致する祖先を親構成要素として見つけ、`"親/子"` 形式の階層名を作る。
    別パスで同名衝突が起きた場合は末尾に `"_"` を補って一意性を保つ(`descriptor.discover_components` と同一規則)。
    """
    parent_comp = ""
    best_len = -1
    for cname, crel in comp_rel.items():
        if crel and child_rel.startswith(crel + "/") and len(crel) > best_len:
            parent_comp = cname
            best_len = len(crel)
    comp_name = f"{parent_comp}/{child_prim_name}" if parent_comp else child_prim_name
    while comp_name in comp_rel and comp_rel[comp_name] != child_rel:
        comp_name += "_"
    comp_rel[comp_name] = child_rel
    for sub_name, sub_rel in child_subcomponents.items():
        if not sub_name or not sub_rel:
            continue
        nested_name = f"{comp_name}/{sub_name}"
        nested_rel = _join(child_rel, sub_rel)
        while nested_name in comp_rel and comp_rel[nested_name] != nested_rel:
            nested_name += "_"
        comp_rel[nested_name] = nested_rel
    return comp_name


def compute_composite_fk(
    info: UsdaInfo,
    q_by_scoped_joint: dict[str, float] | None = None,
    active_variants: dict[str, str] | None = None,
) -> dict[str, np.ndarray]:
    """結合木全体のアクティブな `Owner` について、URDF 順運動学で全リンクの 4x4 ワールド変換行列を計算する。

    引数:
      info: 対象結合ファイル(または単体 usda)の `UsdaInfo`。
      q_by_scoped_joint: 関節値辞書。子部品の関節キーは `"arm1/j1"` と `"arm1:j1"` の両形式を受け付ける。
      active_variants: VariantSet 選択の上書き辞書。
                       `{"<rel_path>:<vs_name>": choice}` で特定 prim を指定するか、
                       `{"<vs_name>": choice}` で該当する全 VariantSet を上書きできる。

    戻り値:
      `{スコープ付きリンク名: 4x4 列ベクトル同次変換行列}` 辞書(ルート部品のルートリンク座標系基準)。

    例外:
      NotImplementedError: アクティブな `Owner` に既存 USD 由来(`model is None`)が含まれる場合。
                           USD 由来パーツを含む検証は USD ツリーベースの `kinematics_check` を使う。
    """
    q_map = dict(q_by_scoped_joint or {})
    # 1. 実効 VariantSet 選択を「仕様既定 → Owner 出現条件の既定 → 引数 active_variants」の順で確定する。
    effective_variants: dict[tuple[str, str], str] = dict(info.default_variants)
    for o in info.owners:
        for rel_path, vs_name, choice in o.conditions:
            effective_variants.setdefault((rel_path, vs_name), choice)
    if active_variants:
        for k, v in active_variants.items():
            if ":" in k:
                rel_p, vs_n = k.split(":", 1)
                effective_variants[(rel_p, vs_n)] = v
            else:
                for (rel_p, vs_n) in list(effective_variants):
                    if vs_n == k or rel_p == k or rel_p.endswith("/" + k):
                        effective_variants[(rel_p, vs_n)] = v
                effective_variants[("", k)] = v

    def is_owner_active(o: Owner) -> bool:
        for rel_path, vs_name, choice in o.conditions:
            selected = effective_variants.get((rel_path, vs_name))
            if selected is not None and selected != choice:
                return False
        return True

    # 2. `info.owners` は親→子の順に並んでいるため、各 Owner のルート姿勢を親リンクから順次計算できる。
    world_links: dict[str, np.ndarray] = {}
    for o in info.owners:
        if not is_owner_active(o):
            continue
        if o.model is None:
            raise NotImplementedError(
                f"compute_composite_fk は URDF 由来の Owner のみ対応している"
                f"(USD 部品 {o.urdf_name or o.prefix!r} には kinematics_check を使用してください)"
            )
        q_local: dict[str, float] = {}
        for jname in o.model.joints:
            if o.prefix:
                k_slash = f"{o.prefix}/{jname}"
                k_colon = f"{o.prefix}:{jname}"
                if k_slash in q_map:
                    q_local[jname] = float(q_map[k_slash])
                elif k_colon in q_map:
                    q_local[jname] = float(q_map[k_colon])
            else:
                if jname in q_map:
                    q_local[jname] = float(q_map[jname])

        if o.prefix == "":
            t_owner_root = np.eye(4)
        else:
            p_prefix = o.parent_prefix or ""
            p_link = o.parent_link
            p_scoped = f"{p_prefix}/{p_link}" if (p_prefix and p_link) else (p_link or "")
            if p_scoped not in world_links:
                # 親側が非アクティブな Variant だった場合など、親リンクが存在しない子はスキップする。
                continue
            t_parent_link = world_links[p_scoped]
            offset = o.join_offset if o.join_offset is not None else np.eye(4)
            c_link = o.child_link or o.model.root_link
            # `child_link` が子のルートリンクでない場合でも、`T_world(child_link) == T_world(parent_link) @ offset` を
            # 満たすように、子のルート姿勢を `inv(T_child_root_to_child_mount)` で逆算する。
            t_child_mount = o.model.fk(c_link, q_local)
            t_owner_root = t_parent_link @ offset @ np.linalg.inv(t_child_mount)

        for ln in o.model.links:
            scoped_ln = f"{o.prefix}/{ln}" if o.prefix else ln
            world_links[scoped_ln] = t_owner_root @ o.model.fk(ln, q_local)
    return world_links


def _join(a: str, b: str) -> str:
    """スラッシュ重複を防ぎながら 2 つの相対/絶対 USD パス断片を連結する。"""
    if not b:
        return a
    if not a:
        return b
    return a.rstrip("/") + "/" + b.lstrip("/")


def _join_prefix(a: str, b: str) -> str:
    """空文字を無視して 2 つのスコープ前置きを `"/"` で連結する。"""
    return a if not b else (b if not a else f"{a}/{b}")


__all__ = [
    "Owner",
    "UsdaInfo",
    "resolve",
    "static_resolve",
    "compute_composite_fk",
]
