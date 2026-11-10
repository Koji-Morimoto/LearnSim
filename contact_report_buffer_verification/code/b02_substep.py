"""B02: 1 回の simulate / update_simulation が複数のサブステップに分かれたとき、バッファに何が残るか。

「1 ステップ分だけ保持」の「1 ステップ」が、PhysX の 1 サブステップなのか、
simulate(elapsed) 1 回の呼び出しなのかを切り分ける。

確かめること:
    S-01  timeStepsPerSecond=60 で simulate(k/60) を 1 回呼ぶと、サブステップは何回走るか
          (物理ステップイベントの回数と、自由落下の落下量の 2 通りで数える)
    S-02  接触し続けている状態で simulate(k/60) を 1 回呼ぶと、PERSIST ヘッダはサブステップ数だけ溜まるか
    S-03  1 回の呼び出しで複数サブステップ走るとき、各サブステップのイベント(FOUND / LOST)が全部残るか
          (高速で壁の横をかすめる立方体。1/60 刻みで呼んだ列を正解として、2 サブステップずつの列と比べる)
    S-04  同じ条件で push 購読(subscribe_contact_report_events)を併用すると、push は全サブステップ分届くか
"""

from __future__ import annotations

import crblib

app = crblib.start()

from typing import Any

import carb.settings
from omni.physx.bindings._physx import SETTING_MIN_FRAME_RATE

rec = crblib.Recorder("b02_substep")

GRAVITY: float = 9.81  # S-01 の自由落下用 [m/s^2]
SPEED: float = 30.0  # S-03 の立方体の速さ [m/s]。1/60 s で 0.5 m 進む
KS: list[int] = [1, 2, 3, 4, 8]  # 1 回の呼び出しに渡す elapsed の倍数(elapsed = k * DT)


def rigid_z(path: str) -> float:
    """PhysX 内部の剛体位置(USD ではなく PhysX 側)の z を返す。"""
    t = crblib.physx().get_rigidbody_transformation(path)
    return float(t["position"][2])


def drop_in_dt_units(drop: float) -> float:
    """静止から自由落下した距離を g * DT^2 で割った値。

    PhysX は半陰的オイラー法なので、静止から落下させたとき
        DT のサブステップを n 回 → 落下量 = g * DT^2 * n(n+1)/2  → 1, 3, 6, 10, ...
        k*DT の 1 ステップを 1 回 → 落下量 = g * (k*DT)^2          → 1, 4, 9, 16, ...
    になる。この値を見れば「分割されたか、大きな 1 ステップだったか」が分かる。
    """
    return drop / (GRAVITY * crblib.DT * crblib.DT)


@crblib.guarded(rec, "S-00")
def env() -> None:
    s = carb.settings.get_settings()
    rec.add("S-00", "サブステップ数に関係する設定", minFrameRate=s.get(SETTING_MIN_FRAME_RATE), **crblib.versions())


@crblib.guarded(rec, "S-01")
def count_substeps() -> None:
    out: dict[str, Any] = {}
    for mode in ["simfetch", "update"]:
        for k in KS:
            stage, sid = crblib.new_stage(gravity=GRAVITY, steps_per_second=60)
            crblib.make_box(stage, "/World/Faller", (0.0, 0.0, 100.0), kind="dynamic", report=False)
            ctr = crblib.StepCounter()
            st = crblib.Stepper(sid, mode)  # type: ignore[arg-type]
            z0 = rigid_z("/World/Faller")
            st.step(k * crblib.DT)
            drop = z0 - rigid_z("/World/Faller")
            out[f"{mode}/k={k}"] = {
                "step_events": ctr.steps,
                "step_event_dts": ctr.dts,
                "drop_over_g_dt2": round(drop_in_dt_units(drop), 3),
                "if_split_into_k_substeps": k * (k + 1) / 2,
                "if_single_step_of_k_dt": k * k,
            }
            ctr.close()
            st.close()
            crblib.cleanup()
    rec.add("S-01", "simulate(k/60) 1 回で走るサブステップ数", **out)


@crblib.guarded(rec, "S-02")
def persist_accumulation() -> None:
    out: dict[str, Any] = {}
    for mode in ["simfetch", "update"]:
        for k in KS:
            stage, sid = crblib.make_toggle_scene()
            ctr = crblib.StepCounter()
            st = crblib.Stepper(sid, mode)  # type: ignore[arg-type]
            st.step()  # 1 回目は FOUND。ここから先を観測する
            ctr.steps = 0
            st.step(k * crblib.DT)
            snap = crblib.pull()
            out[f"{mode}/k={k}"] = {"substeps": ctr.steps, "pulled": snap.summary(), "contact_points": len(snap.data)}
            ctr.close()
            st.close()
            crblib.cleanup()
    rec.add("S-02", "接触継続中に k サブステップを 1 回で進めたときのヘッダ数", **out)


def build_graze_scene(start_x: float) -> tuple[Any, int]:
    """高速で薄い壁の横をかすめる立方体のシーン。重力 0。

    立方体(一辺 1 m)は y=0 を +x 方向へ SPEED で進む。壁(x 方向 5 cm、y 方向 1 m)は y = 1 + GAP にある。
    x 方向に重なっている間だけ、y 方向の隙間 GAP で接触が報告される(押し合いは起きないので速度は変わらない)。

    Args:
        start_x: 立方体の初期 x [m]。0.5 m ずらすと接触の始まるサブステップが 1 つずれる。
    """
    stage, sid = crblib.new_stage(gravity=0.0, steps_per_second=60)
    crblib.make_box(stage, "/World/Wall", (0.0, 1.0 + crblib.GAP, 0.0), size=(0.05, 1.0, 1.0), kind="static")
    crblib.make_box(stage, "/World/Box", (start_x, 0.0, 0.0), kind="dynamic", report=True, velocity=(SPEED, 0.0, 0.0))
    return stage, sid


def run_graze(mode: crblib.StepperMode, k: int, start_x: float, total_substeps: int = 16) -> dict[str, Any]:
    """かすめシーンを、合計 total_substeps サブステップ分(呼び出し 1 回 = elapsed k*DT)進める。

    Returns:
        イベントがあった呼び出しの一覧(各呼び出しがカバーしたサブステップ番号つき)と最終位置。
    """
    _, sid = build_graze_scene(start_x)
    ctr = crblib.StepCounter()
    st = crblib.Stepper(sid, mode)
    calls: list[dict[str, Any]] = []
    c = 0
    while ctr.steps < total_substeps and c < 4 * total_substeps:
        before = ctr.steps
        st.step(k * crblib.DT)
        calls.append({"call": c, "substeps": list(range(before, ctr.steps)), "pulled": crblib.pull().summary()})
        c += 1
    result = {
        "calls_with_events": [x for x in calls if x["pulled"]],
        "final_x": round(float(crblib.physx().get_rigidbody_transformation("/World/Box")["position"][0]), 4),
        "total_substeps": ctr.steps,
    }
    ctr.close()
    st.close()
    crblib.cleanup()
    return result


@crblib.guarded(rec, "S-03")
def found_and_lost_inside_one_call() -> None:
    # k=1 はサブステップ 1 つ = 呼び出し 1 回なので、これがサブステップ単位の正解の列になる。
    # update/k=2 は 1 回の呼び出しで 2 サブステップ走る(S-01)。開始位置を 0.5 m ずらした 2 通りで、
    # 「FOUND と PERSIST」「PERSIST と LOST」が同じ呼び出しに入る組み合わせを作る。
    out: dict[str, Any] = {}
    for start_x in [-3.0, -2.5]:
        out[f"start_x={start_x}/simfetch/k=1"] = run_graze("simfetch", 1, start_x)
        out[f"start_x={start_x}/update/k=1"] = run_graze("update", 1, start_x)
        out[f"start_x={start_x}/update/k=2"] = run_graze("update", 2, start_x)
    rec.add(
        "S-03",
        "1 回の呼び出しで複数サブステップ走るとき、各サブステップのイベントが全部残るか",
        speed=SPEED,
        **out,
    )


@crblib.guarded(rec, "S-04")
def push_vs_pull_with_substeps() -> None:
    # S-03 の update/k=2 と同じ条件で、push 購読(subscribe_contact_report_events)も同時に張る。
    # push がサブステップごとに届くなら、pull で消えたイベントも push 側では受け取れている。
    out: dict[str, Any] = {}
    for start_x in [-3.0, -2.5]:
        _, sid = build_graze_scene(start_x)
        sub = crblib.ContactSubscriber()
        ctr = crblib.StepCounter()
        st = crblib.Stepper(sid, "update")
        calls: list[dict[str, Any]] = []
        while ctr.steps < 16:
            before_steps, before_cb = ctr.steps, len(sub.calls)
            st.step(2 * crblib.DT)
            calls.append(
                {
                    "substeps": list(range(before_steps, ctr.steps)),
                    "push": [c["summary"] for c in sub.calls[before_cb:]],
                    "pull": crblib.pull().summary(),
                }
            )
        out[f"start_x={start_x}/update/k=2"] = [c for c in calls if c["push"] or c["pull"]]
        sub.close()
        ctr.close()
        st.close()
        crblib.cleanup()
    rec.add("S-04", "2 サブステップ/呼び出しのとき、push 購読と pull が受け取るものの違い", **out)


env()
count_substeps()
persist_accumulation()
found_and_lost_inside_one_call()
push_vs_pull_with_substeps()
crblib.finish(rec)
