"""B01: get_contact_report()(pull 型)が「何を」「いつまで」返すかを測る。

確かめること:
    P-01  ステップ前(attach 直後)に pull すると何が返るか
    P-02  同じステップのあいだに何度 pull しても同じ内容か(読むと消える=消費型ではないか)
    P-03  返り値は PhysX 内部バッファへの参照(ビュー)か、呼び出し時点のコピーか
    P-04  毎ステップ pull したときのイベント列(基準となる正解)
    P-05  pull をサボったステップのイベントは後から取れるか(何ステップ分キャッシュされるか)
    P-06  バッファが差し替わるのは simulate 時点か fetch_results 時点か
    P-07  detach / 再 attach でバッファはどうなるか
    P-08  push 型(subscribe_contact_report_events)を併用したとき、pull と同じ内容か・消費し合わないか
    P-09  update_simulation 経路(Play 中に近い経路)でも同じ挙動か

シーンは crblib.make_toggle_scene(床+1 mm 浮かせた動的立方体、重力 0)。
SCHEDULE のステップ番号の「直前」に立方体をテレポートして、FOUND / PERSIST / LOST を仕込む。
"""

from __future__ import annotations

import crblib

app = crblib.start()

from typing import Any

import carb.settings
from omni.physx.bindings._physx import SETTING_UPDATE_TO_USD

rec = crblib.Recorder("b01_pull_semantics")

# ステップ番号 → そのステップの直前に置く位置。0 番は初期位置(床の上)から始まる。
#   step0: FOUND(初回接触) / step1: PERSIST / step2: 直前に離す → LOST / step3: 何もない
#   step4: 直前に戻す → FOUND / step5, step6: PERSIST
SCHEDULE: dict[int, crblib.Vec3] = {2: crblib.away(), 4: crblib.on_ground()}
N_STEPS: int = 7


def run_schedule(stage: Any, stepper: crblib.Stepper, pull_after: set[int] | None) -> dict[int, list[str]]:
    """SCHEDULE どおりに N_STEPS ステップ進め、pull_after に含まれるステップの直後だけ pull する。

    Args:
        stage: make_toggle_scene のステージ。
        stepper: ステップ実行器。
        pull_after: pull するステップ番号の集合。None なら毎ステップ pull する。

    Returns:
        ステップ番号 → そのステップ直後に pull した要約。
    """
    out: dict[int, list[str]] = {}
    for s in range(N_STEPS):
        if s in SCHEDULE:
            crblib.set_pos(stage, "/World/Box", SCHEDULE[s])
        stepper.step()
        if pull_after is None or s in pull_after:
            out[s] = crblib.pull().summary()
    return out


@crblib.guarded(rec, "P-00")
def env() -> None:
    s = carb.settings.get_settings()
    rec.add(
        "P-00",
        "バージョンと関係する設定",
        **crblib.versions(),
        updateToUsd=s.get_as_bool(SETTING_UPDATE_TO_USD),
        fabricEnabled=s.get_as_bool("/physics/fabricEnabled"),
        get_contact_report_doc=crblib.phys_sim().get_contact_report.__doc__,
    )


@crblib.guarded(rec, "P-01")
def before_first_step() -> None:
    _, sid = crblib.make_toggle_scene()
    before_attach = crblib.pull()
    st = crblib.Stepper(sid)
    after_attach = crblib.pull()
    st.step()
    after_step = crblib.pull()
    st.close()
    rec.add(
        "P-01",
        "attach 前 / attach 直後(ステップ前) / 1 ステップ後の pull",
        before_attach=before_attach.summary(),
        after_attach_no_step=after_attach.summary(),
        after_first_step=after_step.summary(),
        tuple_len=after_step.tuple_len,
    )


@crblib.guarded(rec, "P-02")
def idempotent() -> None:
    _, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid)
    st.step()
    raws = [crblib.phys_sim().get_contact_report() for _ in range(5)]
    snaps = [crblib.to_snapshot(r) for r in raws]
    st.close()
    rec.add(
        "P-02",
        "同じステップ内で 5 回 pull",
        summaries=[s.summary() for s in snaps],
        all_equal_including_contact_points=all(s.as_dict() == snaps[0].as_dict() for s in snaps),
        # 毎回別の Python オブジェクトが作られるか(同じなら内部バッファを共有している可能性)
        distinct_tuple_objects=len({id(r) for r in raws}) == len(raws),
        distinct_header_vector_objects=len({id(r[0]) for r in raws}) == len(raws),
        first_contact_points=[asdict_short(d) for d in snaps[0].data],
    )


def asdict_short(d: crblib.DataRec) -> dict[str, Any]:
    """接触点 1 つを記録用の短い辞書にする。"""
    return {"pos": d.position, "normal": d.normal, "sep": d.separation}


@crblib.guarded(rec, "P-03")
def copy_or_view() -> None:
    stage, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid)
    st.step()
    st.step()  # step1 = PERSIST の状態にする

    # (a) 返り値を書き換えても次の pull に影響しないか
    r1 = crblib.phys_sim().get_contact_report()
    len_before = (len(r1[0]), len(r1[1]))
    r1[0].clear()
    r1[1].clear()
    r2 = crblib.phys_sim().get_contact_report()
    len_after_mutation = (len(r2[0]), len(r2[1]))

    # (b) 古い返り値を握ったままステップを進めると、古い返り値の中身は変わるか
    kept = crblib.phys_sim().get_contact_report()
    kept_before = crblib.to_snapshot(kept)
    crblib.set_pos(stage, "/World/Box", crblib.away())
    st.step()  # LOST のステップ
    kept_after = crblib.to_snapshot(kept)
    fresh = crblib.pull()
    st.close()
    rec.add(
        "P-03",
        "返り値はコピーか内部バッファへのビューか",
        len_before_mutation=len_before,
        len_next_pull_after_clearing_returned_vectors=len_after_mutation,
        kept_before_step=kept_before.summary(),
        kept_after_next_step=kept_after.summary(),
        kept_object_unchanged=kept_before.as_dict() == kept_after.as_dict(),
        fresh_pull_after_next_step=fresh.summary(),
        vector_types=[type(kept[0]).__name__, type(kept[1]).__name__],
    )


@crblib.guarded(rec, "P-04")
def every_step() -> None:
    stage, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid)
    out = run_schedule(stage, st, None)
    st.close()
    rec.add("P-04", "毎ステップ pull(基準)", schedule={k: list(v) for k, v in SCHEDULE.items()}, per_step=out)


@crblib.guarded(rec, "P-05")
def skipped_pulls() -> None:
    # 1 つの実行で 1 回だけ pull する。どのステップで pull しても、そのステップの分だけが返るなら
    # 「バッファは直近 1 ステップ分だけ。pull しなかったステップの分は次のステップで上書きされる」。
    out: dict[str, Any] = {}
    for k in [1, 2, 3, 6]:
        stage, sid = crblib.make_toggle_scene()
        st = crblib.Stepper(sid)
        out[f"pull_only_after_step{k}"] = run_schedule(stage, st, {k})[k]
        st.close()
        crblib.cleanup()
    rec.add(
        "P-05",
        "途中のステップで pull しないと、そのステップのイベントは後から取れるか",
        note="step2 は LOST、step3 は空、step6 は PERSIST が P-04 の基準。step3 で pull すると step2 の LOST は見えるか",
        **out,
    )


@crblib.guarded(rec, "P-06")
def swap_timing() -> None:
    stage, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid)
    st.step()
    st.step()  # step1 = PERSIST
    after_step1 = crblib.pull().summary()
    crblib.set_pos(stage, "/World/Box", crblib.away())
    after_usd_teleport = crblib.pull().summary()  # USD を書いただけ(まだ simulate していない)
    st.simulate()
    between_simulate_and_fetch = crblib.pull().summary()
    st.fetch()
    after_fetch = crblib.pull().summary()
    st.close()
    rec.add(
        "P-06",
        "バッファが差し替わるタイミング",
        after_step1=after_step1,
        after_usd_teleport_before_simulate=after_usd_teleport,
        between_simulate_and_fetch_results=between_simulate_and_fetch,
        after_fetch_results=after_fetch,
    )


@crblib.guarded(rec, "P-07")
def detach_reattach() -> None:
    _, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid)
    st.step()
    st.step()
    before = crblib.pull().summary()
    crblib.phys_sim().detach_stage()
    after_detach = crblib.pull().summary()
    crblib.phys_sim().attach_stage(sid)
    after_reattach = crblib.pull().summary()
    crblib.phys_sim().simulate(crblib.DT, 0.0)
    crblib.phys_sim().fetch_results()
    after_reattach_step = crblib.pull().summary()
    crblib.phys_sim().detach_stage()
    rec.add(
        "P-07",
        "detach / 再 attach とバッファ",
        before_detach=before,
        after_detach=after_detach,
        after_reattach_no_step=after_reattach,
        after_reattach_one_step=after_reattach_step,
    )


@crblib.guarded(rec, "P-08")
def with_subscription() -> None:
    stage, sid = crblib.make_toggle_scene()
    sub = crblib.ContactSubscriber()
    st = crblib.Stepper(sid)
    per_step: dict[int, Any] = {}
    for s in range(N_STEPS):
        if s in SCHEDULE:
            crblib.set_pos(stage, "/World/Box", SCHEDULE[s])
        n_calls_before = len(sub.calls)
        st.simulate()
        sub.in_fetch_flag = True
        st.fetch()
        sub.in_fetch_flag = False
        cb_this_step = sub.calls[n_calls_before:]
        p1 = crblib.pull()
        p2 = crblib.pull()
        per_step[s] = {
            "callbacks": len(cb_this_step),
            "callback_inside_fetch_results": [c["inside_fetch_results"] for c in cb_this_step],
            "callback_summary": [c["summary"] for c in cb_this_step],
            "pull_summary": p1.summary(),
            "pull_equals_callback": bool(cb_this_step) and cb_this_step[-1]["snapshot"].as_dict() == p1.as_dict(),
            "second_pull_equal": p1.as_dict() == p2.as_dict(),
        }
    # app.update() でコールバックが追加で来るか(遅延配信の有無)
    n = len(sub.calls)
    app.update()
    late = len(sub.calls) - n
    st.close()
    sub.close()
    rec.add("P-08", "push 購読と pull の併用", per_step=per_step, callbacks_during_app_update_after_last_step=late)


@crblib.guarded(rec, "P-09")
def update_simulation_path() -> None:
    stage, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid, mode="update")
    every = run_schedule(stage, st, None)
    st.close()
    crblib.cleanup()
    stage, sid = crblib.make_toggle_scene()
    st = crblib.Stepper(sid, mode="update")
    only3 = run_schedule(stage, st, {3})
    st.close()
    rec.add("P-09", "update_simulation 経路", every_step=every, pull_only_after_step3=only3)


env()
before_first_step()
idempotent()
copy_or_view()
every_step()
skipped_pulls()
swap_timing()
detach_reattach()
with_subscription()
update_simulation_path()
crblib.finish(rec)
