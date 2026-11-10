"""P01: USD / Fabric / PhysX / tensor の間で、値がどの向きに・いつ流れるか(パイプライン)を調べる。

ラッパー(RigidPrim など)は使わず、各層の下位 API で直接書いて、全層を観測する。
--physx-fabric / --gpu 付きで実行すると、その構成(omni.physx.fabric 有効 / GPU 物理)で同じ測定をする。

確かめること:
    P-01  Stop 中に USD を書くと Fabric に届くか。届くのは書いた瞬間か、次の update か
    P-02  Stop 中に Fabric を書くと USD に届くか。その後 USD を書くと Fabric の値はどうなるか
    P-03  Play 開始時、PhysX は USD と Fabric のどちらから剛体の初期位置を読むか(両者を食い違わせて確かめる)
    P-04  Play 中、PhysX の計算結果は毎ステップどの層に書き出されるか(重力で落下させて追う)
    P-05  Stop したとき、各層の値はどうなるか(resetOnStop = true)
    P-06  Play 中に USD / Fabric / tensor をそれぞれ直接書くと、PhysX に届くか。その後どうなるか
    P-07  P-06 で Play 中の Fabric 書き込みが PhysX に届いた経路の切り分け(物理なし Marker と比べる)
    P-08  Stop 中の Fabric 書き込みが、同じ prim の未処理の USD 変更で上書きされるか(M01 の結果の切り分け)
    P-09  Play 中の Fabric 書き込みを PhysX が拾う経路は IPhysxSimulation の Fabric change tracking か
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any

from pxr import Usd

rec = bplib.Recorder("p01_pipeline")


def scene(gravity: float = 0.0) -> Usd.Stage:
    """剛体(/World/Body, z=1)と物理なし Xform(/World/Marker, 原点)だけのステージ。"""
    stage = bplib.new_stage(gravity=gravity)
    bplib.make_body(stage)
    bplib.make_marker(stage)
    bplib.update()
    return stage


@bplib.guarded(rec, "P-00")
def env() -> None:
    rec.add("P-00", "バージョンと設定", **bplib.environment())


@bplib.guarded(rec, "P-01")
def usd_to_fabric() -> None:
    stage = scene()
    out: dict[str, Any] = {}
    for path in [bplib.MARKER, bplib.BODY]:
        bplib.usd_set_translate(stage, path, (2.0, 0.0, 0.0))
        out[path] = {
            "fabric_immediately": bplib.fabric_state(path),
            "usd_immediately": bplib.usd_state(stage, path),
        }
        bplib.update()
        out[path]["fabric_after_update"] = bplib.fabric_state(path)
    rec.add("P-01", "Stop 中に pxr で xformOp:translate = (2,0,0) を書いたあとの Fabric", **out)


@bplib.guarded(rec, "P-02")
def fabric_to_usd() -> None:
    stage = scene()
    path = bplib.MARKER
    bplib.fabric_set_translate(path, (0.0, 7.0, 0.0))
    out: dict[str, Any] = {
        "1_fabric_write/fabric": bplib.fabric_state(path),
        "1_fabric_write/usd": bplib.usd_state(stage, path),
    }
    bplib.update(3)
    out["2_after_3_updates/fabric"] = bplib.fabric_state(path)
    out["2_after_3_updates/usd"] = bplib.usd_state(stage, path)
    # 変換と無関係な USD 属性を書く → Fabric の変換は USD から取り直されるか
    stage.GetPrimAtPath(path).CreateAttribute("test:dummy", _float_type()).Set(1.0)
    bplib.update()
    out["3_after_unrelated_usd_write/fabric"] = bplib.fabric_state(path)
    # 同じ xformOp:translate を USD で書く → Fabric の値は上書きされるか
    bplib.usd_set_translate(stage, path, (0.0, 0.0, 5.0))
    bplib.update()
    out["4_after_usd_translate_write/fabric"] = bplib.fabric_state(path)
    out["4_after_usd_translate_write/usd"] = bplib.usd_state(stage, path)
    rec.add("P-02", "Stop 中に Fabric の local 行列を (0,7,0) にしたあと", **out)


def _float_type() -> Any:
    from pxr import Sdf

    return Sdf.ValueTypeNames.Float


@bplib.guarded(rec, "P-03")
def play_reads_from() -> None:
    stage = scene()
    # USD は z=1 のまま、Fabric だけ x=3 にずらしてから Play する
    bplib.fabric_set_translate(bplib.BODY, (3.0, 0.0, 1.0))
    bplib.update()
    before = bplib.snapshot(stage)
    bplib.play(1)
    after_play = bplib.snapshot(stage)
    bplib.stop(1)
    after_stop = bplib.snapshot(stage)
    rec.add(
        "P-03",
        "USD=(0,0,1), Fabric=(3,0,1) に食い違わせて Play → Stop",
        before_play=before,
        after_play_1_update=after_play,
        after_stop=after_stop,
    )


@bplib.guarded(rec, "P-04")
def step_output() -> None:
    stage = scene(gravity=9.81)
    steps: dict[str, Any] = {"0_before_play": bplib.snapshot(stage)}
    bplib.play(1)
    steps["1_after_play_update1"] = bplib.snapshot(stage)
    for i in range(2, 4):
        bplib.update()
        steps[f"{i}_after_update{i}"] = bplib.snapshot(stage)
    rec.add("P-04", "重力 9.81 で落下させ、毎 update の各層の z を追う", **steps)
    # P-05: そのまま Stop
    out: dict[str, Any] = {}
    bplib.timeline().stop()
    out["stop_called_no_update"] = bplib.snapshot(stage)
    bplib.update()
    out["after_stop_update1"] = bplib.snapshot(stage)
    bplib.update()
    out["after_stop_update2"] = bplib.snapshot(stage)
    rec.add("P-05", "P-04 の続きで Stop(resetOnStop = true)", **out)


@bplib.guarded(rec, "P-06")
def direct_writes_while_playing() -> None:
    writers: dict[str, Any] = {
        "usd(pxr)": lambda stage: bplib.usd_set_translate(stage, bplib.BODY, (0.0, 0.0, 2.0)),
        "fabric(IFabricHierarchy)": lambda stage: bplib.fabric_set_translate(bplib.BODY, (0.0, 0.0, 3.0)),
        "tensor(RigidBodyView.set_transforms)": lambda stage: bplib.tensor_set_position(bplib.BODY, (0.0, 0.0, 4.0)),
    }
    out: dict[str, Any] = {}
    for label, write in writers.items():
        stage = scene()
        bplib.play(3)
        res: dict[str, Any] = {"before": bplib.snapshot(stage)}
        write(stage)
        res["immediately"] = bplib.snapshot(stage)
        bplib.update()
        res["after_update1"] = bplib.snapshot(stage)
        bplib.update(5)
        res["after_update6"] = bplib.snapshot(stage)
        bplib.stop(1)
        res["after_stop"] = bplib.snapshot(stage)
        out[label] = res
    rec.add("P-06", "Play 中(重力 0、静止)に各層を直接書く → update → Stop", **out)


@bplib.guarded(rec, "P-07")
def fabric_write_while_playing_marker() -> None:
    """P-06 で Play 中の Fabric 書き込みが PhysX に届いた。その経路を切り分ける。

    物理を持たない Marker でも Play 中の Fabric 書き込みが USD に戻るなら「Fabric → USD の汎用の書き戻し」、
    戻らないなら「PhysX(または omni.physx.fabric)が剛体の Fabric 変換を読んでいる」と判断できる。
    """
    stage = scene()
    bplib.play(3)
    bplib.fabric_set_translate(bplib.MARKER, (0.0, 6.0, 0.0))
    bplib.fabric_set_translate(bplib.BODY, (0.0, 6.0, 1.0))
    out: dict[str, Any] = {
        "immediately/marker_usd": bplib.usd_state(stage, bplib.MARKER),
        "immediately/marker_fabric": bplib.fabric_state(bplib.MARKER),
        "immediately/body": bplib.snapshot(stage),
    }
    bplib.update()
    out["after_update1/marker_usd"] = bplib.usd_state(stage, bplib.MARKER)
    out["after_update1/marker_fabric"] = bplib.fabric_state(bplib.MARKER)
    out["after_update1/body"] = bplib.snapshot(stage)
    bplib.update(5)
    out["after_update6/marker_usd"] = bplib.usd_state(stage, bplib.MARKER)
    out["after_update6/marker_fabric"] = bplib.fabric_state(bplib.MARKER)
    bplib.stop(1)
    out["after_stop/marker_usd"] = bplib.usd_state(stage, bplib.MARKER)
    out["after_stop/marker_fabric"] = bplib.fabric_state(bplib.MARKER)
    rec.add("P-07", "Play 中に Marker(物理なし)と Body の Fabric を y=6 に書く", **out)


@bplib.guarded(rec, "P-08")
def fabric_write_overwritten_by_pending_usd_change() -> None:
    """M01 で、Stop 中に usdrt / fabric バックエンドで書いた位置が次の update で USD の値に戻った。
    P-03 では戻らなかった。違いは RigidPrim を作ったこと(コンストラクタが PhysxRigidBodyAPI を USD に適用する)。
    「Fabric を書いた時点で、その prim に未処理の USD 変更があると、次の update で Fabric が USD から作り直される」
    という仮説を、USD 変更の種類とタイミングを変えて確かめる。
    """
    from isaacsim.core.experimental.prims import RigidPrim
    from pxr import PhysxSchema, Sdf

    def body_fabric_after(prepare: Any, flush_before_write: bool) -> dict[str, Any]:
        stage = scene()
        keep = prepare(stage)  # RigidPrim を返すケースは、GC で消えないよう参照を保持する
        if flush_before_write:
            bplib.update()
        bplib.fabric_set_translate(bplib.BODY, (0.0, 8.0, 1.0))
        res = {"immediately": bplib.fabric_state(bplib.BODY)["local"]}
        bplib.update()
        res["after_update1"] = bplib.fabric_state(bplib.BODY)["local"]
        bplib.update(2)
        res["after_update3"] = bplib.fabric_state(bplib.BODY)["local"]
        res["usd"] = bplib.usd_state(stage, bplib.BODY)["translate"]
        del keep
        return res

    cases: dict[str, tuple[Any, bool]] = {
        "a_何もしない": (lambda st: None, False),
        "b_APIスキーマ適用(PhysxRigidBodyAPI)→即Fabric書き込み": (lambda st: PhysxSchema.PhysxRigidBodyAPI.Apply(st.GetPrimAtPath(bplib.BODY)), False),
        "c_APIスキーマ適用→update→Fabric書き込み": (lambda st: PhysxSchema.PhysxRigidBodyAPI.Apply(st.GetPrimAtPath(bplib.BODY)), True),
        "d_無関係な属性を作成→即Fabric書き込み": (lambda st: st.GetPrimAtPath(bplib.BODY).CreateAttribute("test:dummy", Sdf.ValueTypeNames.Float).Set(1.0), False),
        "e_RigidPrim作成→即Fabric書き込み": (lambda st: RigidPrim(bplib.BODY), False),
        "f_RigidPrim作成→update→Fabric書き込み": (lambda st: RigidPrim(bplib.BODY), True),
    }
    out = {label: body_fabric_after(prep, flush) for label, (prep, flush) in cases.items()}
    rec.add("P-08", "Stop 中、Body の Fabric を y=8 に書く前に USD 変更を仕込む", **out)


@bplib.guarded(rec, "P-09")
def fabric_change_tracking() -> None:
    """P-07 の続き。Play 中の Fabric 書き込みを PhysX が拾う経路が、IPhysxSimulation の
    「physics listener の Fabric change tracking」(_physx.pyi の pause_change_tracking の説明文)かどうかを、
    それを一時停止して確かめる。
    """
    out: dict[str, Any] = {}
    for paused in [False, True]:
        stage = scene()
        bplib.play(3)
        sim = bplib.phys_sim()
        sim.pause_change_tracking(paused)
        res: dict[str, Any] = {"is_change_tracking_paused": sim.is_change_tracking_paused()}
        bplib.fabric_set_translate(bplib.BODY, (0.0, 9.0, 1.0))
        bplib.update()
        res["after_update1"] = bplib.snapshot(stage)
        sim.pause_change_tracking(False)  # 再開すると、止めていた間の変更が後から届くか
        bplib.update()
        res["after_resume_update1"] = bplib.snapshot(stage)
        out["paused" if paused else "not_paused"] = res
        bplib.stop(1)
    rec.add("P-09", "Play 中に pause_change_tracking を切り替えて Body の Fabric を y=9 に書く", **out)


env()
usd_to_fabric()
fabric_to_usd()
play_reads_from()
step_output()
direct_writes_while_playing()
fabric_write_while_playing_marker()
fabric_write_overwritten_by_pending_usd_change()
fabric_change_tracking()
bplib.finish(rec)
