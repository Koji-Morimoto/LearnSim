"""B05: 静止したシーンで ContactReport が途中から来なくなる現象を、push と pull で並べて測る。

push 購読(subscribe_contact_report_events のコールバック)で「PERSIST が何回か来たあと来なくなる」
という現象がある。B03 C-02 では pull でも 24 ステップ目から 0 件になった。
ここでは同じ実行の中で push と pull を同時に記録し、剛体のスリープ状態と突き合わせる。

確かめること:
    Z-01  重力 0 / 重力あり × スリープ許可 / 禁止 の 4 通りで、push・pull・スリープ状態をステップごとに記録する
          → push と pull は同じステップで止まるか。止まるのはスリープした瞬間か
    Z-02  スリープ後に wake_up すると報告は戻るか。戻るとき FOUND か PERSIST か
    Z-03  起きている剛体を put_to_sleep で強制的に寝かせると、その場で報告が止まるか(因果の確認)
    Z-04  タイムライン Play(GUI で on_report を使うときの経路)でも同じか
"""

from __future__ import annotations

import crblib

app = crblib.start()

from typing import Any, Callable

import omni.timeline
from pxr import PhysicsSchemaTools

rec = crblib.Recorder("b05_sleep")

BOX: str = "/World/Box"
STEPS: int = 80  # 観測するステップ数。既定のスリープ設定なら数十ステップで寝る


def build(gravity: float, can_sleep: bool) -> tuple[Any, int]:
    """静的な床の上に動的立方体 1 個を GAP だけ浮かせて置く(重力ありなら 1 mm 落ちて床に乗る)。"""
    stage, sid = crblib.new_stage(gravity=gravity)
    crblib.make_box(stage, "/World/Ground", (0.0, 0.0, -0.5), size=(10.0, 10.0, 1.0), kind="static")
    crblib.make_box(stage, BOX, crblib.on_ground(), kind="dynamic", report=True, can_sleep=can_sleep)
    return stage, sid


def box_id() -> int:
    """BOX の prim パスを PhysX API 用の整数に符号化する。"""
    return int(PhysicsSchemaTools.sdfPathToInt(BOX))


def label(types: list[str]) -> str:
    """1 ステップ分のイベント種別を 1 語にする。何もなければ '-'。"""
    return "+".join(types) if types else "-"


class Observer:
    """push 購読と pull を同時に行い、ステップごとに push・pull・スリープ状態を記録する。"""

    def __init__(self, stage_id: int) -> None:
        self.stage_id: int = stage_id
        self.sub: crblib.ContactSubscriber = crblib.ContactSubscriber()
        self.push: list[str] = []
        self.pull: list[str] = []
        self.sleeping: list[bool] = []
        self._n_calls: int = 0

    def record(self) -> None:
        """直前のステップ(または app.update 1 回)の分を記録する。"""
        calls = self.sub.calls[self._n_calls :]
        self._n_calls = len(self.sub.calls)
        self.push.append(label([t for c in calls for t in c["snapshot"].types()]))
        self.pull.append(label(crblib.pull().types()))
        self.sleeping.append(bool(crblib.phys_sim().is_sleeping(self.stage_id, box_id())))

    def result(self) -> dict[str, Any]:
        first_sleep = next((i for i, s in enumerate(self.sleeping) if s), None)
        last_push = max((i for i, p in enumerate(self.push) if p != "-"), default=None)
        last_pull = max((i for i, p in enumerate(self.pull) if p != "-"), default=None)
        return {
            "push": crblib.compress(self.push),
            "pull": crblib.compress(self.pull),
            "sleeping": crblib.compress(self.sleeping),
            "first_step_sleeping": first_sleep,
            "last_step_with_push": last_push,
            "last_step_with_pull": last_pull,
            "push_equals_pull_every_step": self.push == self.pull,
        }

    def close(self) -> None:
        self.sub.close()


def run_steps(gravity: float, can_sleep: bool, actions: dict[int, Callable[[int], None]] | None = None) -> dict[str, Any]:
    """simulate + fetch_results で STEPS ステップ進め、Observer の結果を返す。

    Args:
        gravity: 重力加速度の大きさ [m/s^2]。
        can_sleep: 立方体のスリープを許すか。
        actions: ステップ番号 → そのステップの直前に呼ぶ関数(引数は stage_id)。
    """
    _, sid = build(gravity, can_sleep)
    obs = Observer(sid)
    st = crblib.Stepper(sid)
    for s in range(STEPS):
        if actions and s in actions:
            actions[s](sid)
        st.step()
        obs.record()
    res = obs.result()
    obs.close()
    st.close()
    crblib.cleanup()
    return res


@crblib.guarded(rec, "Z-01")
def push_vs_pull_until_sleep() -> None:
    out: dict[str, Any] = {}
    for gravity in [0.0, 9.81]:
        for can_sleep in [True, False]:
            out[f"gravity={gravity}/can_sleep={can_sleep}"] = run_steps(gravity, can_sleep)
    rec.add("Z-01", "静止シーンで push と pull が止まるステップとスリープの関係", **out)


@crblib.guarded(rec, "Z-02")
def wake_after_sleep() -> None:
    wake_at = 50  # Z-01 で寝ているはずのステップ
    res = run_steps(9.81, True, {wake_at: lambda sid: crblib.phys_sim().wake_up(sid, box_id())})
    rec.add("Z-02", f"重力あり・スリープ許可。step{wake_at} の直前に wake_up", wake_at=wake_at, **res)


@crblib.guarded(rec, "Z-03")
def forced_sleep() -> None:
    sleep_at = 5  # まだ起きているステップ
    res = run_steps(0.0, True, {sleep_at: lambda sid: crblib.phys_sim().put_to_sleep(sid, box_id())})
    rec.add("Z-03", f"重力 0・スリープ許可。step{sleep_at} の直前に put_to_sleep", sleep_at=sleep_at, **res)


@crblib.guarded(rec, "Z-04")
def timeline_play() -> None:
    _, sid = build(9.81, True)
    ctr = crblib.StepCounter()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    app.update()  # Play 開始フレーム(ここで PhysX オブジェクトが作られる)
    obs = Observer(sid)
    steps_per_frame: list[int] = []
    for _ in range(STEPS):
        before = ctr.steps
        app.update()
        steps_per_frame.append(ctr.steps - before)
        obs.record()
    res = obs.result()
    obs.close()
    ctr.close()
    rec.add(
        "Z-04",
        "タイムライン Play 中。app.update 1 回ごとに push と pull を記録(1 フレーム = 1 行)",
        physics_steps_per_frame=crblib.compress(steps_per_frame),
        **res,
    )


push_vs_pull_until_sleep()
wake_after_sleep()
forced_sleep()
timeline_play()
crblib.finish(rec)
