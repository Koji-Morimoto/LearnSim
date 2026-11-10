"""B03: ContactReport のバッファは、誰も読まなくても毎ステップ作られているか。

内部のメモリ確保は Python から直接見えないので、2 つの間接的な証拠で判断する。
    (1) 時間: ContactReportAPI の有無だけを変えて、simulate + fetch_results の時間を比べる。
        「読まない」条件でも API ありの方が遅く、その差が pull・購読の有無で変わらなければ、
        バッファ作成は fetch_results の中で毎ステップ無条件に行われている。
    (2) 中身: 長く回したあとに 1 回だけ pull したときのヘッダ数が、接触ペア数(1 ステップ分)と等しいか。

確かめること:
    C-01  N 個の立方体が床に接触し続けるシーンで、次の 4 条件の 1 ステップ時間を比べる
            no_api        : ContactReportAPI なし(報告対象ゼロ)
            api_no_read   : API あり、pull も購読もしない
            api_pull      : API あり、毎ステップ pull する(pull の時間は別に測る)
            api_subscribe : API あり、push 購読する(コールバックは何もしない)
          さらに pull 1 回のコスト(中身のコピー)が N に比例するかを見る。
          立方体はスリープさせない(sleepThreshold=0)。スリープすると接触自体が計算されなくなるため。
    C-02  スリープを許した(既定の)静止剛体では、何ステップ目から報告が止まるか
"""

from __future__ import annotations

import crblib

app = crblib.start()

import statistics
import time
from typing import Any, Literal

from pxr import PhysicsSchemaTools

rec = crblib.Recorder("b03_per_step_cost")

Condition = Literal["no_api", "api_no_read", "api_pull", "api_subscribe"]
CONDITIONS: list[Condition] = ["no_api", "api_no_read", "api_pull", "api_subscribe"]
SIZES: list[int] = [10, 30, 50]  # 一辺の個数。立方体は SIZE^2 個 = 100, 900, 2500
STEPS: int = 200  # 計測するステップ数
WARMUP: int = 10  # 計測前に捨てるステップ数(初回の FOUND やキャッシュ確保を除く)
REPEATS: int = 2  # 条件の並びを何周するか(時間ドリフトの影響を均すため交互に回す)
SPACING: float = 1.5  # 立方体の間隔 [m]。互いには触れない


def build_grid(side: int, report: bool, can_sleep: bool = False) -> tuple[Any, int]:
    """床の上に side x side 個の立方体を GAP だけ浮かせて並べる。重力 0 なので誰も動かない。

    Args:
        side: 一辺の個数。
        report: 立方体に ContactReportAPI を付けるか。
        can_sleep: False(既定)ならスリープさせず、毎ステップ全ペアの接触を発生させる。
    """
    stage, sid = crblib.new_stage(gravity=0.0)
    extent = side * SPACING + 2.0
    crblib.make_box(stage, "/World/Ground", (0.0, 0.0, -0.5), size=(extent, extent, 1.0), kind="static")
    origin = -(side - 1) * SPACING / 2.0
    for i in range(side):
        for j in range(side):
            pos = (origin + i * SPACING, origin + j * SPACING, 0.5 + crblib.GAP)
            crblib.make_box(stage, f"/World/B_{i}_{j}", pos, report=report, can_sleep=can_sleep)
    return stage, sid


def measure(side: int, cond: Condition) -> dict[str, Any]:
    """1 条件を計測して、ステップ時間・pull 時間・最後の pull のヘッダ数を返す。"""
    _, sid = build_grid(side, report=(cond != "no_api"))
    # 購読条件では何もしないコールバックを張る(Python 側の変換コストを含めないため)
    sub: Any = crblib.phys_sim().subscribe_contact_report_events(lambda h, d: None) if cond == "api_subscribe" else None
    st = crblib.Stepper(sid)
    for _ in range(WARMUP):
        st.step()
    step_ms: list[float] = []
    pull_ms: list[float] = []
    for _ in range(STEPS):
        t0 = time.perf_counter()
        st.step()
        step_ms.append((time.perf_counter() - t0) * 1e3)
        if cond == "api_pull":
            t1 = time.perf_counter()
            crblib.phys_sim().get_contact_report()
            pull_ms.append((time.perf_counter() - t1) * 1e3)
    # 最後に 1 回だけ pull: 何ステップ分のヘッダが入っているか
    t2 = time.perf_counter()
    last = crblib.phys_sim().get_contact_report()
    final_pull_ms = (time.perf_counter() - t2) * 1e3
    n_headers, n_points = len(last[0]), len(last[1])
    sub = None  # 購読解除
    st.close()
    crblib.cleanup()
    return {
        "step_ms_median": round(statistics.median(step_ms), 4),
        "step_ms_mean": round(statistics.fmean(step_ms), 4),
        "pull_ms_median": round(statistics.median(pull_ms), 4) if pull_ms else None,
        "final_pull_ms": round(final_pull_ms, 4),
        "final_pull_headers": n_headers,
        "final_pull_contact_points": n_points,
    }


@crblib.guarded(rec, "C-01")
def cost() -> None:
    out: dict[str, Any] = {}
    for side in SIZES:
        runs: dict[str, list[dict[str, Any]]] = {c: [] for c in CONDITIONS}
        for _ in range(REPEATS):
            for cond in CONDITIONS:
                runs[cond].append(measure(side, cond))
        n = side * side
        summary: dict[str, Any] = {}
        for cond, rs in runs.items():
            summary[cond] = {
                "step_ms_median_per_repeat": [r["step_ms_median"] for r in rs],
                "pull_ms_median_per_repeat": [r["pull_ms_median"] for r in rs],
                "final_pull_ms_per_repeat": [r["final_pull_ms"] for r in rs],
                "final_pull_headers": rs[-1]["final_pull_headers"],
                "final_pull_contact_points": rs[-1]["final_pull_contact_points"],
            }
        base = statistics.fmean(summary["no_api"]["step_ms_median_per_repeat"])
        for cond in CONDITIONS:
            summary[cond]["step_ms_over_no_api"] = round(statistics.fmean(summary[cond]["step_ms_median_per_repeat"]) - base, 4)
        out[f"N={n}"] = {"contact_pairs": n, "steps_run_before_final_pull": WARMUP + STEPS, **summary}
    rec.add("C-01", "ContactReportAPI の有無・読み方ごとの 1 ステップ時間と pull のコスト", **out)


@crblib.guarded(rec, "C-02")
def sleeping_pairs() -> None:
    # スリープを許した(既定の)剛体で、毎ステップ pull したときのヘッダ数の推移を見る。
    # 剛体がスリープするとそのペアの接触は計算されず、PERSIST も来なくなるはず。
    _, sid = build_grid(3, report=True, can_sleep=True)
    st = crblib.Stepper(sid)
    counts: list[int] = []
    types_seen: dict[int, list[str]] = {}
    for s in range(120):
        st.step()
        snap = crblib.pull()
        counts.append(len(snap.headers))
        if s == 0 or (s > 0 and counts[s] != counts[s - 1]):
            types_seen[s] = sorted(set(snap.types()))
    sleeping = crblib.phys_sim().is_sleeping(sid, int(PhysicsSchemaTools.sdfPathToInt("/World/B_0_0")))
    st.close()
    first_zero = next((i for i, c in enumerate(counts) if c == 0), None)
    rec.add(
        "C-02",
        "スリープを許した静止剛体(9 個)で、ステップごとのヘッダ数",
        headers_per_step_compressed=crblib.compress(counts),
        first_step_with_zero_headers=first_zero,
        event_types_when_count_changed=types_seen,
        B_0_0_is_sleeping_at_end=sleeping,
    )


cost()
sleeping_pairs()
crblib.finish(rec)
