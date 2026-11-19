"""仕様の検査と、参照関係の一覧。Isaac Sim なしで動く(URDF を読むだけ)。

    python run_pipeline.py robot.toml --check     # Isaac Sim を起動せずに、検査して一覧を表示する
"""

from __future__ import annotations

import os

from .spec import PipelineSpec
from .urdf_model import UrdfModel, load_urdf


def _join_where(fu: str, name: str) -> str:
    return f"[[join_file]] usda = {fu!r} の [[join_file.join]] name = {name!r}"


def check_links(spec: PipelineSpec, models: dict[str, UrdfModel] | None = None) -> list[str]:
    """リンク名・子を置く prim・子の VariantSet の選択が実在するか。問題の一覧を返す(空なら問題なし)。"""
    models = models or {u.usda: load_urdf(u.urdf) for u in spec.urdfs}
    problems: list[str] = []

    def link_problem(usda: str, scoped: str, key: str, where: str) -> None:
        owner, link = spec.resolve_link(usda, scoped)
        m = models[owner]
        if link not in m.links:
            u = spec.urdf(owner)
            hint = ""
            if "/" in scoped:
                hint = (f"。\"{scoped.split('/')[0]}/\" は {usda} の中の結合の名前として見つからなかった"
                        f"(前置きに書けるのは、その usda を作った [[join_file]] の [[join_file.join]] の name)")
            problems.append(f"{where} の {key} = {scoped!r}: {owner}(URDF {os.path.basename(u.urdf)})にリンク {link!r} が無い。"
                            f"ある名前: {sorted(m.links)}{hint}")

    def variant_problem(usda: str, scoped: str, selection: dict[str, str], key: str, where: str) -> None:
        """前置きでたどった結合が、選ばれていない VariantSet の選択肢の中にあれば問題にする。"""
        f = spec.join_file(usda)
        if f is None:
            return
        head, _, rest = scoped.partition("/")
        j = next((j for j in f.joins if j.name == head), None) if rest else None
        if j is None:
            variant_problem(f.parent_usda, scoped, {}, key, where)
            return
        if j.variant_set:
            default = next(x.variant for x in f.joins if x.variant_set == j.variant_set)
            chosen = selection.get(j.variant_set, default)
            if chosen != j.variant:
                problems.append(f"{where} の {key} = {scoped!r}: 結合 {head!r} は {usda} の VariantSet {j.variant_set!r} の選択肢 "
                                f"{j.variant!r} の中にあるが、選ばれているのは {chosen!r}。選択肢の中のリンクは、その選択肢が選ばれているときだけ書ける")
                return
        variant_problem(j.child_usda, rest, dict(j.child_variant_selection), key, where)

    for f, j in spec.all_joins():
        where = _join_where(f.usda, j.name)
        link_problem(f.parent_usda, j.parent_link, "parent_link", where)
        link_problem(j.child_usda, j.child_link, "child_link", where)
        variant_problem(f.parent_usda, j.parent_link, {}, "parent_link", where)
        variant_problem(j.child_usda, j.child_link, dict(j.child_variant_selection), "child_link", where)
        if j.place_child_under and j.place_child_under != "." and not j.place_child_under.startswith("/"):
            link_problem(f.parent_usda, j.place_child_under, "place_child_under", where)
            variant_problem(f.parent_usda, j.place_child_under, {}, "place_child_under", where)
        child_file = spec.join_file(j.child_usda)
        for vs_name, choice in j.child_variant_selection:
            if child_file is None:
                problems.append(f"{where} の child_variant_selection の {vs_name!r}: 子 {j.child_usda} は URDF から作った usda なので、"
                                "選べる VariantSet が無い(VariantSet は [[join_file.join]] の variant_set で作る)")
                continue
            sets: dict[str, list[str]] = {}
            for cj in child_file.joins:
                if cj.variant_set:
                    sets.setdefault(cj.variant_set, []).append(cj.variant)  # type: ignore[arg-type]
            if vs_name not in sets:
                problems.append(f"{where} の child_variant_selection: 子 {j.child_usda} に VariantSet {vs_name!r} が無い。"
                                f"ある VariantSet: {sorted(sets) or 'なし'}({j.child_usda} を作った [[join_file.join]] の variant_set)")
            elif choice not in sets[vs_name] + ["None"]:
                problems.append(f"{where} の child_variant_selection: 子 {j.child_usda} の VariantSet {vs_name!r} に選択肢 {choice!r} が無い。"
                                f"ある選択肢: {sets[vs_name] + ['None']}")
    return problems


def describe(spec: PipelineSpec) -> list[str]:
    """参照関係の一覧(Markdown の行)。"""
    used: dict[str, list[str]] = {}
    for f in spec.join_files:
        used.setdefault(f.parent_usda, []).append(f"{f.usda} の parent_usda")
        for j in f.joins:
            used.setdefault(j.child_usda, []).append(f"{f.usda} の child_usda(結合 {j.name})")
    lines = ["## 仕様の読み取り結果", "", f"出力先フォルダ(output_dir): `{spec.output_dir}`", "",
             "### 作る usda", "", "| usda(出力のファイル名) | 作り方 | 入力 | この usda を使う場所 |", "|---|---|---|---|"]
    for u in spec.urdfs:
        lines.append(f"| {u.usda} | [[urdf_to_usda]] | URDF `{os.path.basename(u.urdf)}` | {'、'.join(used.get(u.usda, [])) or '使われない'} |")
    for f in spec.file_order():
        kids = "、".join(dict.fromkeys(j.child_usda for j in f.joins)) or "なし"
        lines.append(f"| {f.usda} | [[join_file]] | 親 {f.parent_usda}、子 {kids} | {'、'.join(used.get(f.usda, [])) or 'どこにも使われない(最後の出力)'} |")
    if spec.join_files:
        lines += ["", "### 結合", "",
                  "| 結合(結合ファイル/name) | 親の usda のリンク | 子の usda | 子の usda のリンク | 子を置く prim | 子の prim の名前 | VariantSet の選択肢 | 子の VariantSet の選択 |",
                  "|---|---|---|---|---|---|---|---|"]
        for f, j in spec.all_joins():
            p_owner, p_link = spec.resolve_link(f.parent_usda, j.parent_link)
            c_owner, c_link = spec.resolve_link(j.child_usda, j.child_link)
            place = {None: f"{j.parent_link} の prim(既定)", ".": f"{f.usda} の一番上の prim"}.get(j.place_child_under, j.place_child_under)
            variant = f"{j.variant_set} = {j.variant}" if j.variant_set else "-"
            sel = "、".join(f"{a} = {b}" for a, b in j.child_variant_selection) or "-"
            lines.append(f"| {f.usda}/{j.name} | {j.parent_link}({p_owner} の {p_link}) | {j.child_usda} | {j.child_link}({c_owner} の {c_link}) "
                         f"| {place} | {j.prim} | {variant} | {sel} |")
    return lines
