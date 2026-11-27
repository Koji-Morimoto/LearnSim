"""パイプライン実行レポート（``report.json`` / ``report.md``）の書き出しと前回結果との差分比較を担うモジュール。

責務:
  1. ``Finding``・``Outcome``・順運動学検証結果・各種測定値を JSON シリアライズ可能な形へ変換し、
     ``report.json`` と人間が読みやすい ``report.md`` を出力する（``write_report``）。
  2. 前回実行の ``report.json`` が存在する場合は ``report_previous.json`` へ退避したうえで差分を比較し、
     Isaac Sim のバージョン更新による公式不具合の解消や新規退行を ``changes_since_previous`` として強調する
     （``compare_with_previous``）。

主な公開 API:
  - ``VERDICT_JA``: 英語 ``Verdict`` コードから日本語表示ラベルへの対応表
  - ``compare_with_previous(outcomes, previous)``: 前回 ``report.json`` との症状・判定の差分リスト生成
  - ``write_report(output_dir, *, status, findings, outcomes, extra)`` / ``write_reports``: JSON/MD レポート出力
  - ``to_markdown(data)`` / ``render_markdown``: レポート辞書から Markdown 文字列を生成

パイプライン全体での位置づけ:
  ``pipeline.build_assembly`` の事前停止時および ``pipeline.run_pipeline`` の Step 9（最終出力）から呼ばれる。

手を入れやすい箇所:
  - ``report.md`` に独自の検証セクションを追加したい場合は ``to_markdown`` を編集する。
"""

from __future__ import annotations

import dataclasses
import json
import os
import time
from typing import Any

from .rules import Finding, Outcome

VERDICT_JA = {
    "ok": "問題なし", "fixed": "修正で解消", "upstream_fixed": "公式のままでも起きず(公式側の修正の可能性)",
    "still_broken": "修正後も起きた", "unexpected": "判定外の症状", "not_measured": "測れず",
}


def _plain(o: Any) -> Any:
    """dataclass や NumPy 配列を含むオブジェクトを ``json.dump`` 可能な組み込み型へ再帰変換する。"""
    if dataclasses.is_dataclass(o):
        return {k: _plain(v) for k, v in dataclasses.asdict(o).items()}
    if isinstance(o, dict):
        return {str(k): _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if hasattr(o, "tolist"):
        return o.tolist()
    return o


def compare_with_previous(outcomes: list[Outcome], previous: dict | None) -> list[str]:
    """前回の ``report.json`` と今回の ``outcomes`` を比較し、公式症状や判定が変わった項目を列挙する。

    Args:
        outcomes: 今回の ``rules.evaluate_all`` 結果。
        previous: 前回 ``report.json`` を読み込んだ辞書（初回実行時は ``None``）。

    Returns:
        差分メッセージ文字列のリスト（変化が無ければ空リスト）。副作用なし。
    """
    if not previous:
        return []
    prev = {(o["rule"], o["target"]): o for o in previous.get("outcomes", [])}
    out = []
    for o in outcomes:
        p = prev.get((o.rule, o.target))
        if not p:
            continue
        before, after = p.get("official", {}).get("occurred"), o.official.occurred
        if before is True and after is False:
            out.append(f"[{o.rule}] {o.target}: 前回は公式のままで症状あり → 今回は無し(公式側の修正が入った可能性)")
        elif before is False and after is True:
            out.append(f"[{o.rule}] {o.target}: 前回は公式のままで症状なし → 今回は有り")
        if p["verdict"] != o.verdict:
            out.append(f"[{o.rule}] {o.target}: 判定 {VERDICT_JA.get(p['verdict'], p['verdict'])} → {VERDICT_JA.get(o.verdict, o.verdict)}")
    return out


def write_report(output_dir: str, *, status: str, findings: list[Finding], outcomes: list[Outcome], extra: dict) -> dict:
    """``report.json`` と ``report.md`` を ``output_dir`` へ書き出し、生成したレポート辞書を返す。

    既存の ``report.json`` がある場合は先に ``report_previous.json`` として退避し、
    ``compare_with_previous`` で前回からの変化を記録する。

    Args:
        output_dir: 出力先ディレクトリパス。
        status: パイプライン最終状態（``"ok"`` / ``"problems"`` / ``"stopped_before_import"`` / ``"stopped_before_join"``）。
        findings: 事前チェックの ``Finding`` 一覧。
        outcomes: 事後評価の ``Outcome`` 一覧。
        extra: ``kinematics``・``outputs``・``descriptors``・``measurements`` 等の追加辞書。

    Returns:
        ``report.json`` に書き出した内容と同じ辞書。

    副作用:
        ``<output_dir>/report.json``・``report.md``（および既存時 ``report_previous.json``）を書き出す。
    """
    os.makedirs(output_dir, exist_ok=True)
    jpath = os.path.join(output_dir, "report.json")
    previous = None
    if os.path.exists(jpath):
        with open(jpath, encoding="utf-8") as f:
            try:
                previous = json.load(f)
            except json.JSONDecodeError:
                previous = None
    changes = compare_with_previous(outcomes, previous)
    data = {"status": status, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "findings": _plain(findings),
            "outcomes": _plain(outcomes), "changes_since_previous": changes, **_plain(extra)}
    if previous is not None:
        with open(os.path.join(output_dir, "report_previous.json"), "w", encoding="utf-8") as f:
            json.dump(previous, f, ensure_ascii=False, indent=1)
    with open(jpath, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    with open(os.path.join(output_dir, "report.md"), "w", encoding="utf-8") as f:
        f.write(to_markdown(data))
    return data


write_reports = write_report


def to_markdown(data: dict) -> str:
    """レポート辞書 ``data`` を人間が一覧しやすい Markdown レポート文字列へ整形する。

    規則ごとの結果セクションでは、要対応の深刻度が高い ``still_broken`` / ``unexpected`` を先頭に並べ、
    問題なしの ``ok`` を末尾に配置して確認負荷を下げる。副作用なし。
    """
    lines = ["# 結合パイプラインの報告", "", f"{data['time']} / 状態: **{data['status']}**", ""]
    desc = data.get("spec_description")
    if desc:
        desc_lines = desc.splitlines() if isinstance(desc, str) else list(desc)
        lines += desc_lines + [""]
    if data.get("stopped_by"):
        lines += ["## 止めた理由", ""] + [f"- {s}" for s in data["stopped_by"]] + [""]
    if data.get("changes_since_previous"):
        lines += ["## 前回からの変化", ""] + [f"- {s}" for s in data["changes_since_previous"]] + [""]
    if data.get("outcomes"):
        counts: dict[str, int] = {}
        for o in data["outcomes"]:
            counts[o["verdict"]] = counts.get(o["verdict"], 0) + 1
        lines += ["## まとめ", "", "| 判定 | 件数 |", "|---|---|"] + [f"| {VERDICT_JA.get(k, k)} | {v} |" for k, v in counts.items()] + [""]
        order = ["still_broken", "unexpected", "upstream_fixed", "fixed", "not_measured", "ok"]
        lines += ["## 規則ごとの結果", ""]
        for v in order:
            for o in data["outcomes"]:
                if o["verdict"] == v:
                    lines.append(f"- **{VERDICT_JA.get(v, v)}** {o['message']}")
        lines.append("")
    elif data.get("findings"):
        lines += ["## 判定", ""] + [f"- [{f['rule']}] {f['target']}: {f['reason']}" for f in data["findings"]] + [""]
    kin = data.get("kinematics") or data.get("geometric")
    if kin:
        lines += ["## 順運動学の検証(結合ファイルごと)", ""]
        for name, g in kin.items():
            if g.get("error"):
                lines.append(f"- {name}: 検証できなかった: {g['error']}")
            else:
                lines.append(f"- {name}: 構成要素 {g.get('components_in_model')}、DOF の名前が PhysX と一致 {g.get('dof_names_match')}、"
                             f"同じ関節値でのリンク位置の最大差 {g.get('max_link_position_error_m')} m({g.get('links_compared')} リンク)")
        lines.append("")
    if data.get("outputs"):
        lines += ["## 出力", ""] + [f"- {k}: `{v}`" for k, v in data["outputs"].items()] + [""]
    return "\n".join(lines)


render_markdown = to_markdown

__all__ = [
    "VERDICT_JA",
    "compare_with_previous",
    "write_report",
    "write_reports",
    "to_markdown",
    "render_markdown",
]
