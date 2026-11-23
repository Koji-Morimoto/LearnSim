"""パイプライン仕様のオフライン検査と、usda 参照関係の一覧整形。

責務:
  - Isaac Sim / pxr を起動せず、`PipelineSpec` と URDF XML だけを使って仕様ミスを早期検出する。
  - 具体的には、リンク名のスペルミス、未選択の VariantSet 配下リンク参照、`child_variant_selection` の
    存在しない VariantSet / 選択肢指定をすべて洗い出してエラー文字列一覧で返す。
  - 仕様が生成する usda と結合の一覧を Markdown 表に整形し、`--check` やレポート冒頭で可視化する。

主な入出力・公開 API:
  - `check_links(spec, models=None) -> list[str]`: 仕様整合性の問題一覧を返す（空リストなら問題なし）。
  - `describe(spec) -> list[str]`: 作る usda と結合一覧の Markdown 行リストを返す。

呼ばれ元:
  - `__init__.py`（公開 API として再エクスポート）
  - `pipeline.py`（パイプライン実行開始前のゲート検査およびレポート用サマリ生成）
  - `run_pipeline.py`（`--check` オプション時の高速オフライン検査）

注意:
  - `[[usd_to_usda]]` 由来パーツのリンク名・VariantSet は USD ステージを開かないと列挙できないため、
    オフライン段階の `check_links` では意図的にスキップし、後段の USD 解決・結合時に検証する。
"""

from __future__ import annotations

import os

from .spec import PipelineSpec
from .urdf_model import UrdfModel, load_urdf


def _join_where(fu: str, name: str) -> str:
    """エラーメッセージで結合の定義位置を示す共通ラベルを生成する。"""
    return f"[[join_file]] usda = {fu!r} の [[join_file.join]] name = {name!r}"


def check_links(spec: PipelineSpec, models: dict[str, UrdfModel] | None = None) -> list[str]:
    """結合仕様のリンク名・配置先 prim・子の VariantSet 選択が実在するかを静的に検査する。

    引数:
      spec: 検査対象の `PipelineSpec`。
      models: 読み込み済みの `{usda_name: UrdfModel}` 辞書。省略時は `spec.urdfs` から都度読み込む。

    戻り値:
      検出された問題点の説明文リスト。空リストであれば仕様に問題なし。
    """
    models = models if models is not None else {u.usda: load_urdf(u.urdf) for u in spec.urdfs}
    problems: list[str] = []

    def link_problem(usda: str, scoped: str, key: str, where: str) -> None:
        """スコープ付きリンク名 `scoped` が所有先 URDF に実在するかを検証する。"""
        owner, link = spec.resolve_link(usda, scoped)
        # USD 入力パーツはステージを開かないとリンク集合が確定しないため、オフライン検査はスキップする。
        if spec.usd(owner) is not None:
            return
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
        """結合名プレフィックスでたどる途中経路に、現在選ばれていない VariantSet 選択肢が含まれないか検証する。

        なぜ必要か:
          例えば親 usda の VariantSet `tool` の既定選択が `gripper_a` なのに、
          未選択の `gripper_b/tip` を `parent_link` に指定すると結合時に実体が存在せず失敗するため、
          TOML 検証段階で明確なエラーとして止める。
        """
        f = spec.join_file(usda)
        if f is None:
            return
        head, _, rest = scoped.partition("/")
        j = next((j for j in f.joins if j.name == head), None) if rest else None
        if j is None:
            # 先頭セグメントが結合名でなければ、親側 usda のリンクとしてさらに上位へたどる。
            variant_problem(f.parent_usda, scoped, {}, key, where)
            return
        if j.variant_set:
            default = f.default_variant(j.variant_set)
            chosen = selection.get(j.variant_set, default)
            if chosen != j.variant:
                problems.append(f"{where} の {key} = {scoped!r}: 結合 {head!r} は {usda} の VariantSet {j.variant_set!r} の選択肢 "
                                f"{j.variant!r} の中にあるが、選ばれているのは {chosen!r}。選択肢の中のリンクは、その選択肢が選ばれているときだけ書ける")
                return
        # 子 usda の中でさらにネストした結合をたどる際は、この結合が子に適用する選択を引き継ぐ。
        variant_problem(j.child_usda, rest, dict(j.child_variant_selection), key, where)

    for f, j in spec.all_joins():
        where = _join_where(f.usda, j.name)
        link_problem(f.parent_usda, j.parent_link, "parent_link", where)
        link_problem(j.child_usda, j.child_link, "child_link", where)
        variant_problem(f.parent_usda, j.parent_link, {}, "parent_link", where)
        variant_problem(j.child_usda, j.child_link, dict(j.child_variant_selection), "child_link", where)
        # `place_child_under` は "."(トップ prim) や "/..."(明示 prim パス) 以外のときリンク名として扱う。
        if j.place_child_under and j.place_child_under != "." and not j.place_child_under.startswith("/"):
            link_problem(f.parent_usda, j.place_child_under, "place_child_under", where)
            variant_problem(f.parent_usda, j.place_child_under, {}, "place_child_under", where)
        child_file = spec.join_file(j.child_usda)
        for vs_name, choice in j.child_variant_selection:
            if child_file is None:
                # 既存 USD パーツ自身が持つ VariantSet(例: Gripper / Physics)の選択はオフラインでは検証不能なため通す。
                if spec.usd(j.child_usda) is not None:
                    continue
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
    """`PipelineSpec` が作る usda と結合の参照関係を Markdown テーブルの行リストに整形する。

    引数:
      spec: 可視化対象の `PipelineSpec`。

    戻り値:
      改行を含まない Markdown テキスト行のリスト。`"\n".join(describe(spec))` でそのまま出力できる。
    """
    # 各 usda が他の [[join_file]] から親・子としてどこで使われているかを逆引き集計する。
    used: dict[str, list[str]] = {}
    for f in spec.join_files:
        used.setdefault(f.parent_usda, []).append(f"{f.usda} の parent_usda")
        for j in f.joins:
            used.setdefault(j.child_usda, []).append(f"{f.usda} の child_usda(結合 {j.name})")
    lines = ["## 仕様の読み取り結果", "", f"出力先フォルダ(output_dir): `{spec.output_dir}`", "",
             "### 作る usda", "", "| usda(出力のファイル名) | 作り方 | 入力 | この usda を使う場所 |", "|---|---|---|---|"]
    for u in spec.urdfs:
        lines.append(f"| {u.usda} | [[urdf_to_usda]] | URDF `{os.path.basename(u.urdf)}` | {'、'.join(used.get(u.usda, [])) or '使われない'} |")
    for u in spec.usds:
        label = u.usd if "://" in u.usd else os.path.basename(u.usd)
        lines.append(f"| {u.usda} | [[usd_to_usda]] | USD `{label}` | {'、'.join(used.get(u.usda, [])) or '使われない'} |")
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


__all__ = ["check_links", "describe"]
