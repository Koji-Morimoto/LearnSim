"""B04: 検出器の本体 get_initial_collider_pairs() を、PhysX の状態を変えて呼ぶ。

get_initial_collider_pairs() の要点(omni/physx/scripts/physicsUtils.py):
    - 全剛体に ContactReportAPI(threshold 0)を一時サブレイヤーで付ける
    - IPhysxSimulation にステージが未アタッチなら attach_stage する(アタッチ済みなら何もしない)
    - simulate(1/60) → fetch_results → get_contact_report() を 1 回だけ
    - ヘッダのうち type == CONTACT_FOUND だけを集める(PERSIST は捨てる)
    - 自分でアタッチした場合だけ detach_stage する

B01 / B02 の結果から、バッファには直近 1 ステップ分しかなく、すでに接触済みのペアは PERSIST になる。
したがって「ステージがすでにアタッチ済みで、接触がすでに報告済み」だと FOUND が出ず、取りこぼすはず。

確かめること:
    D-01  未アタッチ(通常の Stop 状態)で呼ぶ → ペアが取れるか。呼んだあとの後始末
    D-02  IPhysxSimulation にアタッチ済み・数ステップ済み、ContactReportAPI なしで呼ぶ
    D-03  IPhysxSimulation にアタッチ済み・数ステップ済み、ContactReportAPI がもともと付いている状態で呼ぶ
    D-04  タイムライン Play → Pause の状態で呼ぶ(API なし / あり)
    D-05  タイムライン Play 中に呼ぶ(API なし / あり)
    D-06  D-02 の原因切り分け: アタッチ済みで接触中のペアに、あとから API を付けたら報告されるか
各ケースで、関数が見たはずのバッファ内容(関数が戻った直後の pull。B01 P-07 より detach でも消えない)も記録する。
"""

from __future__ import annotations

import crblib

app = crblib.start()

from typing import Any

import omni.timeline
from omni.physx.scripts.physicsUtils import get_initial_collider_pairs
from pxr import PhysxSchema, Usd

rec = crblib.Recorder("b04_detector_path")

ROBOT: str = "/World/Robot"
MOVER: str = f"{ROBOT}/Mover"  # 1 m/s で上昇し続ける剛体。PhysX の時間が進んだかを見る


def build_robot(pre_api: bool) -> tuple[Usd.Stage, int]:
    """ロボットの代わりの 3 リンク。A と B は隙間 GAP で接触、C は離れている。重力 0。

    Args:
        pre_api: True なら A, B にあらかじめ ContactReportAPI を付けておく。
    """
    stage, sid = crblib.new_stage(gravity=0.0)
    crblib.make_box(stage, f"{ROBOT}/A", (0.0, 0.0, 0.0), report=pre_api)
    crblib.make_box(stage, f"{ROBOT}/B", (1.0 + crblib.GAP, 0.0, 0.0), report=pre_api)
    crblib.make_box(stage, f"{ROBOT}/C", (5.0, 0.0, 0.0), report=pre_api)
    crblib.make_box(stage, MOVER, (0.0, 10.0, 0.0), report=False, velocity=(0.0, 0.0, 1.0))
    return stage, sid


def physx_z(path: str) -> float | None:
    """PhysX 内部の剛体の z。PhysX にオブジェクトがなければ None。"""
    t = crblib.physx().get_rigidbody_transformation(path)
    return round(float(t["position"][2]), 5) if t.get("ret_val") else None


def usd_z(stage: Usd.Stage, path: str) -> float:
    """USD 上の剛体の z。"""
    from pxr import UsdGeom

    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return round(float(m.ExtractTranslation()[2]), 5)


def call_detector(stage: Usd.Stage, sid: int) -> dict[str, Any]:
    """get_initial_collider_pairs を呼び、結果と前後の状態を返す。"""
    before = {
        "attached_stage_matches": crblib.phys_sim().get_attached_stage() == sid,
        "mover_physx_z": physx_z(MOVER),
        "mover_usd_z": usd_z(stage, MOVER),
    }
    pairs = sorted("-".join(crblib.short(p) for p in pair) for pair in get_initial_collider_pairs(stage))
    seen = crblib.pull().summary()  # 関数が見たバッファ(関数が戻ったあとも残っている)
    after = {
        "attached_stage_matches": crblib.phys_sim().get_attached_stage() == sid,
        "mover_physx_z": physx_z(MOVER),
        "mover_usd_z": usd_z(stage, MOVER),
        "A_has_contact_report_api": stage.GetPrimAtPath(f"{ROBOT}/A").HasAPI(PhysxSchema.PhysxContactReportAPI),
        "session_sublayers": len(stage.GetSessionLayer().subLayerPaths),
    }
    return {"pairs_returned": pairs, "buffer_seen_by_function": seen, "before": before, "after": after}


@crblib.guarded(rec, "D-01")
def not_attached() -> None:
    stage, sid = build_robot(pre_api=False)
    rec.add("D-01", "未アタッチ(Stop 状態)で呼ぶ", **call_detector(stage, sid))


def attached_case(test_id: str, pre_api: bool) -> None:
    """IPhysxSimulation にアタッチして 3 ステップ進めてから呼ぶ。"""
    stage, sid = build_robot(pre_api)
    st = crblib.Stepper(sid)
    for _ in range(3):
        st.step()
    last_before_call = crblib.pull().summary()
    res = call_detector(stage, sid)
    st.close()
    rec.add(
        test_id,
        f"IPhysxSimulation にアタッチ済み・3 ステップ済み、ContactReportAPI {'あり' if pre_api else 'なし'}",
        buffer_before_call=last_before_call,
        **res,
    )


@crblib.guarded(rec, "D-02")
def attached_no_api() -> None:
    attached_case("D-02", pre_api=False)


@crblib.guarded(rec, "D-03")
def attached_pre_api() -> None:
    attached_case("D-03", pre_api=True)


def timeline_case(pre_api: bool, pause: bool) -> dict[str, Any]:
    """タイムラインを数フレーム Play してから(必要なら Pause して)呼ぶ。呼んだあとも数フレーム回す。"""
    stage, sid = build_robot(pre_api)
    tl = omni.timeline.get_timeline_interface()
    ctr = crblib.StepCounter()
    tl.play()
    for _ in range(10):
        app.update()
    if pause:
        tl.pause()
        app.update()
    steps_before = ctr.steps
    last_before_call = crblib.pull().summary()
    res = call_detector(stage, sid)
    steps_by_call = ctr.steps - steps_before
    # 呼んだあとも Play を続けて、シミュレーションが壊れていないか(Mover が上昇し続けるか)を見る
    if pause:
        tl.play()
    z0 = physx_z(MOVER)
    for _ in range(10):
        app.update()
    z1 = physx_z(MOVER)
    ctr.close()
    return {
        "physics_steps_before_call": steps_before,
        "physics_step_events_during_call": steps_by_call,
        "buffer_before_call": last_before_call,
        **res,
        "mover_physx_z_10_frames_after_resume": [z0, z1],
    }


@crblib.guarded(rec, "D-04")
def paused() -> None:
    rec.add("D-04", "タイムライン Play → Pause の状態で呼ぶ", no_api=timeline_case(False, True))
    crblib.cleanup()
    rec.add("D-04b", "タイムライン Play → Pause の状態で呼ぶ(API あり)", pre_api=timeline_case(True, True))


@crblib.guarded(rec, "D-05")
def playing() -> None:
    rec.add("D-05", "タイムライン Play 中に呼ぶ", no_api=timeline_case(False, False))
    crblib.cleanup()
    rec.add("D-05b", "タイムライン Play 中に呼ぶ(API あり)", pre_api=timeline_case(True, False))


@crblib.guarded(rec, "D-06")
def api_added_while_attached() -> None:
    # D-02 で何も報告されなかった理由の切り分け。アタッチ済みで接触中のペアに、あとから USD で
    # ContactReportAPI を付けると、(a) 次のステップから報告されるか (b) 一度離して付け直すまで報告されないか。
    stage, sid = build_robot(pre_api=False)
    st = crblib.Stepper(sid)
    for _ in range(3):
        st.step()
    for p in ["A", "B", "C"]:
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{ROBOT}/{p}")).CreateThresholdAttr(0.0)
    per_step: dict[str, list[str]] = {}
    for s in range(4):
        st.step()
        per_step[f"after_api_step{s}"] = crblib.pull().summary()
    crblib.set_pos(stage, f"{ROBOT}/B", (5.0, 5.0, 0.0))  # 離す
    st.step()
    per_step["after_separate"] = crblib.pull().summary()
    crblib.set_pos(stage, f"{ROBOT}/B", (1.0 + crblib.GAP, 0.0, 0.0))  # 戻す
    st.step()
    per_step["after_rejoin"] = crblib.pull().summary()
    st.close()
    rec.add("D-06", "アタッチ済み・接触中のペアに、あとから ContactReportAPI を付けた場合", **per_step)


not_attached()
attached_no_api()
attached_pre_api()
paused()
playing()
api_added_while_attached()
crblib.finish(rec)
