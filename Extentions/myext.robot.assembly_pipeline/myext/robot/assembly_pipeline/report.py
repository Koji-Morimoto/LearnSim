"""報告の書き出し(report.json / report.md)と、前回の報告との比較。"""

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
    """前回の報告と比べて、公式のままの症状が変わった(公式側の修正が入った / 新しく壊れた)ものを挙げる。"""
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
        f.write(render_markdown(data))
    return data


def render_markdown(data: dict) -> str:
    lines = [f"# 結合パイプラインの報告", "", f"{data['time']} / 状態: **{data['status']}**", ""]
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
    if data.get("geometric"):
        lines += ["## 幾何モード(GeometricRobot、結合ファイルごと)", ""]
        for name, g in data["geometric"].items():
            if g.get("error"):
                lines.append(f"- {name}: 作れなかった: {g['error']}")
            else:
                lines.append(f"- {name}: 構成要素 {g.get('components_in_model')}、DOF の名前が PhysX と一致 {g.get('dof_names_match')}、"
                             f"同じ関節値でのリンク位置の最大差 {g.get('max_link_position_error_m')} m({g.get('links_compared')} リンク)")
        lines.append("")
    if data.get("outputs"):
        lines += ["## 出力", ""] + [f"- {k}: `{v}`" for k, v in data["outputs"].items()] + [""]
    return "\n".join(lines)
