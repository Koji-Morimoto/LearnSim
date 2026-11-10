"""S1〜S3 のスニペットを、GUI とは別プロセスの headless Isaac Sim で「貼って実行」と同じ形(exec + 非同期タスク)で走らせ、
結果を期待値と比べる。結果は ../results/verify_snippets.json。

期待値(立方体は一辺 0.2 m。中心間距離が 0.2 未満なら重なる):
    Target(x=0.5) と重なる x: 0.35 0.45 0.55 0.65     Wall(x=1.0) と重なる x: 0.85 0.95 1.05
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code")
import bplib

bplib.RESULT_DIR = os.path.join(os.path.dirname(HERE), "results")
app = bplib.start(render=False)

from typing import Any

rec = bplib.Recorder("verify_snippets")
T = "/World/Target/collider"
W = "/World/Wall/collider"


def expected_hits(x: float) -> list[str]:
    out = []
    if abs(x - 0.5) < 0.2:
        out.append(T)
    if abs(x - 1.0) < 0.2:
        out.append(W)
    return sorted(out)


def run_snippet(name: str, replace: tuple[str, str] | None = None) -> Any:
    src = open(os.path.join(HERE, name), encoding="utf-8").read()
    if replace:
        assert replace[0] in src
        src = src.replace(*replace)
    ns: dict[str, Any] = {"__name__": "__main__"}
    exec(compile(src, name, "exec"), ns)
    for _ in range(2000):
        if ns["task"].done():
            break
        bplib.update(1)
    return ns["task"].result()


def s1_rows(res: Any) -> list[dict[str, Any]]:
    return [
        {"x": x, "hits": hits, "seen_from_target": seen, "ok": hits == expected_hits(x) and seen == (T in expected_hits(x))}
        for x, hits, seen in res
    ]


@bplib.guarded(rec, "S1-usd")
def s1_usd() -> None:
    rows = s1_rows(run_snippet("s1_overlap_play.py"))
    rec.add("S1-usd", "Play 中、USD に書く、overlap_shape", all_ok=all(r["ok"] for r in rows), rows=rows)


@bplib.guarded(rec, "S1-fabric")
def s1_fabric() -> None:
    rows = s1_rows(run_snippet("s1_overlap_play.py", ("USE_FABRIC = False", "USE_FABRIC = True")))
    rec.add("S1-fabric", "Play 中、Fabric だけに書く、overlap_shape", all_ok=all(r["ok"] for r in rows), rows=rows)


@bplib.guarded(rec, "S2-no-filter")
def s2_no_filter() -> None:
    res = run_snippet("s2_contact_report_play.py", ("filtered.CreateFilteredPairsRel().AddTarget(NEIGHBOR)", "pass"))
    names = sorted({e[1] for _, ev, _ in res for e in ev})
    rec.add("S2-no-filter", "対照: FilteredPairs を書かない場合", pairs_seen=names, neighbor_reported=any("Neighbor" in n for n in names))


@bplib.guarded(rec, "S2-no-flags")
def s2_no_flags() -> None:
    src_a = "physx_scene.CreateReportKinematicKinematicPairsAttr(True)"
    res = run_snippet("s2_contact_report_play.py", (src_a, "pass"))
    names = sorted({e[1] for _, ev, _ in res for e in ev})
    rec.add("S2-no-flags", "対照: reportKinematicKinematicPairs を書かない場合", pairs_seen=names)


@bplib.guarded(rec, "S2-threshold-default")
def s2_threshold_default() -> None:
    res = run_snippet("s2_contact_report_play.py", ("report.CreateThresholdAttr(0.0)", "pass"))
    names = sorted({e[1] for _, ev, _ in res for e in ev})
    rec.add("S2-threshold-default", "対照: threshold を書かない(既定値)場合", pairs_seen=names)


@bplib.guarded(rec, "S2")
def s2() -> None:
    res = run_snippet("s2_contact_report_play.py")
    rows = [{"x": x, "events": ev, "penetrating": pen} for x, ev, pen in res]
    names = {e[1] for r in rows for e in r["events"]}
    rec.add("S2", "Play 中、Contact Report", pairs_seen=sorted(names), neighbor_reported=any("Neighbor" in n for n in names), rows=rows)


@bplib.guarded(rec, "S3")
def s3() -> None:
    res = run_snippet("s3_overlap_stop.py")
    rows = [{"x": x, "hits": hits, "seen_from_target": seen, "ok": hits == expected_hits(x) and seen == (T in expected_hits(x))} for x, hits, seen in res]
    rec.add("S3", "Stop 中、USD に書いて update_simulation、overlap_shape", all_ok=all(r["ok"] for r in rows), timeline=bplib.timeline_state(), rows=rows)


for fn in (s1_usd, s1_fabric, s2, s2_no_filter, s2_no_flags, s2_threshold_default, s3):
    fn()
bplib.finish(rec)
