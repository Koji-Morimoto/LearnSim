"""ContactReport の pull 型 API(get_contact_report)のバッファ挙動を調べる共通ライブラリ。

使い方:
    import crblib
    app = crblib.start()      # pxr / omni.* の import より先に必ず呼ぶ
    ...
    crblib.finish(rec)

調べる対象:
    Robot Self-Collision Detector(isaacsim.robot_setup.collision_detector)は
    omni.physx.scripts.physicsUtils.get_initial_collider_pairs() を呼び、その中で
        get_physx_simulation_interface().attach_stage(stage_id)
        get_physx_simulation_interface().simulate(1/60, 0)
        get_physx_simulation_interface().fetch_results()
        get_physx_simulation_interface().get_contact_report()   ← pull
    の順に実行している。本ライブラリはこの手順を部品に分けて、1 手ずつ観測できるようにする。

基本シーン(make_toggle_scene):
    静的な床(上面 z=0)の上に、1 mm 浮かせた動的立方体を置く。重力 0 なので立方体は動かない。
    PhysxContactReportAPI の threshold=0 なので、隙間 1 mm でも接触として毎ステップ報告される。
    立方体を USD 上で z=+10 m へテレポートすると接触が切れ(LOST)、戻すと再び接触する(FOUND)。
    これで「どのステップで何が起きたか」を正確に仕込める。
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import traceback
from dataclasses import asdict, dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Literal

if TYPE_CHECKING:
    # 型ヒント専用。実行時の pxr import は SimulationApp 起動後でないとできない。
    from pxr import Usd

_app: Any = None  # isaacsim.SimulationApp。start() で設定する

HERE: str = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR: str = os.path.join(os.path.dirname(HERE), "results")

DT: float = 1.0 / 60.0  # 1 物理ステップの時間 [s]。検出器と同じ値
GAP: float = 0.001  # 床と立方体の隙間 [m]。threshold=0 なら接触として報告される
AWAY_Z: float = 10.0  # テレポート先の高さ [m]。床から十分離れていて接触しない

BodyKind = Literal["dynamic", "static"]
StepperMode = Literal["simfetch", "update"]
Vec3 = tuple[float, float, float]


# ------------------------------------------------------------------ 起動・記録


def start(timeout_s: float = 900.0) -> Any:
    """headless の SimulationApp を起動して返す。

    ハングしても残らないよう、timeout_s 経過で自プロセスを終了するウォッチドッグを付ける。
    """
    global _app

    def _die() -> None:
        print(f"[WATCHDOG] {timeout_s}s 経過のため終了", flush=True)
        os._exit(3)

    t = threading.Timer(timeout_s, _die)
    t.daemon = True
    t.start()

    from isaacsim import SimulationApp

    _app = SimulationApp({"headless": True}, experience=os.path.join(HERE, "crb_verify.kit"))
    return _app


def app() -> Any:
    """起動済みの SimulationApp を返す。"""
    return _app


class Recorder:
    """測定結果を results/<group>.json に逐次保存する。"""

    def __init__(self, group: str) -> None:
        self.group: str = group
        self.items: list[dict[str, Any]] = []
        os.makedirs(RESULT_DIR, exist_ok=True)
        self.path: str = os.path.join(RESULT_DIR, f"{group}.json")

    def add(self, test_id: str, description: str, **data: Any) -> None:
        item = {"id": test_id, "description": description, **data}
        self.items.append(item)
        print(f"[RESULT] {test_id}: " + json.dumps(data, ensure_ascii=False, default=str)[:2000], flush=True)
        self.save()

    def save(self) -> None:
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"group": self.group, "items": self.items}, f, ensure_ascii=False, indent=1, default=str)


def guarded(rec: Recorder, test_id: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
    """テスト関数の例外を記録して握りつぶすデコレータ。1 件の失敗で残りを止めない。

    終了時は必ず cleanup() を呼び、次のテストへ PhysX の状態(アタッチ・Play)を持ち越さない。
    """

    def deco(fn: Callable[[], None]) -> Callable[[], None]:
        def wrapper() -> None:
            print(f"[START] {test_id}", flush=True)
            t0 = time.perf_counter()
            try:
                fn()
            except Exception:
                rec.add(test_id + "!EXC", "例外", exception=traceback.format_exc())
            finally:
                cleanup()
                print(f"[END] {test_id} {time.perf_counter() - t0:.1f}s", flush=True)

        return wrapper

    return deco


def finish(rec: Recorder) -> None:
    rec.save()
    print("[DONE]", rec.group, len(rec.items), flush=True)
    sys.stdout.flush()
    _app.close()


def versions() -> dict[str, str]:
    """記録用に、関係する拡張のバージョンと Isaac Sim の VERSION を返す。"""
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    out: dict[str, str] = {n: str(mgr.get_enabled_extension_id(n)) for n in ["omni.physx", "omni.physics.physx", "omni.physics.stageupdate"]}
    with open(r"C:\isaacsim-6.2.0\VERSION", encoding="utf-8") as f:
        out["isaac_sim_VERSION"] = f.read().strip()
    return out


# ------------------------------------------------------------------ PhysX インターフェース


def phys_sim() -> Any:
    """IPhysxSimulation(attach_stage / simulate / fetch_results / get_contact_report を持つ)。"""
    from omni.physx import get_physx_simulation_interface

    return get_physx_simulation_interface()


def physx() -> Any:
    """IPhysx(update_simulation / force_load_physics_from_usd / 物理ステップイベントを持つ)。"""
    from omni.physx import get_physx_interface

    return get_physx_interface()


def cleanup() -> None:
    """タイムラインを止め、IPhysxSimulation のアタッチを外し、PhysX オブジェクトを解放する。"""
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()
        _app.update()
    if phys_sim().get_attached_stage() != 0:
        phys_sim().detach_stage()
    physx().release_physics_objects()
    _app.update()


# ------------------------------------------------------------------ ステージ構築


def new_stage(gravity: float = 0.0, steps_per_second: int | None = None) -> tuple[Usd.Stage, int]:
    """Z-up、1 unit = 1 m の空ステージと PhysicsScene(CPU, MBP)を作り、(stage, stage_id) を返す。

    Args:
        gravity: 重力加速度の大きさ [m/s^2]。0 なら物体は衝突以外で動かない。
        steps_per_second: PhysxSceneAPI の timeStepsPerSecond。None なら書かない(既定 60)。
    """
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdUtils

    ctx = omni.usd.get_context()
    ctx.new_stage()
    _app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr("MBP")
    if steps_per_second is not None:
        api.CreateTimeStepsPerSecondAttr(steps_per_second)
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    return stage, stage_id


def make_box(
    stage: Usd.Stage,
    path: str,
    pos: Vec3,
    size: Vec3 = (1.0, 1.0, 1.0),
    kind: BodyKind = "dynamic",
    report: bool = True,
    velocity: Vec3 | None = None,
    can_sleep: bool = True,
) -> Usd.Prim:
    """直方体コライダーを作る。

    Args:
        stage: 対象ステージ。
        path: prim パス。
        pos: 中心位置 [m]。
        size: 各軸の辺の長さ [m]。一辺 1 の Cube を scale で伸ばす。
        kind: "dynamic" なら RigidBodyAPI を付ける。"static" なら静的コライダー。
        report: True なら PhysxContactReportAPI(threshold=0 = 全接触を報告)を付ける。
            静的コライダーには付けても効かないので、報告は動的側で受ける。
        velocity: 動的剛体の初速 [m/s]。
        can_sleep: False なら sleepThreshold=0 にしてスリープさせない。
            静止した剛体は数十ステップでスリープし、スリープ中のペアは報告されなくなる(B03 C-02)。
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
    cube.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if kind == "dynamic":
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if velocity is not None:
            rb.CreateVelocityAttr(Gf.Vec3f(*velocity))
        if report:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        if not can_sleep:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr(0.0)
    return prim


def set_pos(stage: Usd.Stage, path: str, pos: Vec3) -> None:
    """USD の translate を書き換えて剛体をテレポートする(omni.physx の USD 変更監視が PhysX に反映する)。"""
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    raise RuntimeError(f"translate op がない: {path}")


def make_toggle_scene() -> tuple[Usd.Stage, int]:
    """基本シーン: 静的な床 /World/Ground と、その上に GAP だけ浮かせた動的立方体 /World/Box。"""
    stage, stage_id = new_stage(gravity=0.0)
    make_box(stage, "/World/Ground", (0.0, 0.0, -0.5), size=(10.0, 10.0, 1.0), kind="static")
    make_box(stage, "/World/Box", on_ground(), kind="dynamic", report=True)
    return stage, stage_id


def on_ground() -> Vec3:
    """make_toggle_scene の立方体を床に接触させる位置。"""
    return (0.0, 0.0, 0.5 + GAP)


def away() -> Vec3:
    """make_toggle_scene の立方体を床から離す位置。"""
    return (0.0, 0.0, AWAY_Z)


# ------------------------------------------------------------------ ステップ実行


class Stepper:
    """物理ステップを進める 2 通りの方法を同じ口で扱う。

    mode="simfetch": 検出器(get_initial_collider_pairs)と同じ手順。
        attach_stage → simulate → fetch_results。
    mode="update": Play 中に Kit が毎フレーム使う経路に近い手順。
        force_load_physics_from_usd → update_simulation(内部で simulate + fetch_results)。
    """

    def __init__(self, stage_id: int, mode: StepperMode = "simfetch") -> None:
        self.stage_id: int = stage_id
        self.mode: StepperMode = mode
        self.t: float = 0.0  # 累積シミュレーション時刻 [s]
        if mode == "simfetch":
            phys_sim().attach_stage(stage_id)
        else:
            physx().force_load_physics_from_usd()

    def simulate(self, dt: float = DT) -> None:
        """simulate だけを発行する(fetch_results は呼ばない)。simfetch 専用。"""
        assert self.mode == "simfetch"
        phys_sim().simulate(dt, self.t)
        self.t += dt

    def fetch(self) -> None:
        """fetch_results を呼んで結果を確定させる(ブロッキング)。simfetch 専用。"""
        assert self.mode == "simfetch"
        phys_sim().fetch_results()

    def step(self, dt: float = DT) -> None:
        """1 回分の simulate + fetch_results(update は update_simulation 1 回)。"""
        if self.mode == "simfetch":
            self.simulate(dt)
            self.fetch()
        else:
            physx().update_simulation(dt, self.t)
            self.t += dt

    def close(self) -> None:
        if self.mode == "simfetch":
            phys_sim().detach_stage()
        else:
            physx().release_physics_objects()


class StepCounter:
    """IPhysx の物理ステップイベント(サブステップごとに 1 回届く)を数える。"""

    def __init__(self) -> None:
        self.steps: int = 0
        self.dts: list[float] = []
        self.sub: Any = physx().subscribe_physics_step_events(self._cb)

    def _cb(self, dt: float) -> None:
        self.steps += 1
        self.dts.append(round(float(dt), 6))

    def close(self) -> None:
        self.sub = None


# ------------------------------------------------------------------ ContactReport のスナップショット


@dataclass
class HeaderRec:
    """ContactEventHeader 1 件を Python の値に写したもの。"""

    type: str  # "CONTACT_FOUND" / "CONTACT_PERSIST" / "CONTACT_LOST"
    collider0: str
    collider1: str
    contact_data_offset: int
    num_contact_data: int
    stage_id: int


@dataclass
class DataRec:
    """ContactData 1 件(接触点 1 つ)を Python の値に写したもの。"""

    position: Vec3
    normal: Vec3
    impulse: Vec3
    separation: float


@dataclass
class ReportSnapshot:
    """get_contact_report() の戻り値 1 回分を、PhysX 側の寿命から切り離した値として保持する。"""

    tuple_len: int
    headers: list[HeaderRec] = field(default_factory=list)
    data: list[DataRec] = field(default_factory=list)

    def summary(self) -> list[str]:
        """'FOUND Box-Ground n=4' 形式の 1 行要約のリスト。"""
        return [f"{h.type.replace('CONTACT_', '')} {short(h.collider0)}-{short(h.collider1)} n={h.num_contact_data}" for h in self.headers]

    def types(self) -> list[str]:
        return [h.type.replace("CONTACT_", "") for h in self.headers]

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def compress(xs: list[Any]) -> list[str]:
    """[9, 9, 9, 0, 0] → ['9 x3 (step0-2)', '0 x2 (step3-4)'] のように連長圧縮する(ログを短くするため)。"""
    out: list[str] = []
    i = 0
    while i < len(xs):
        j = i
        while j + 1 < len(xs) and xs[j + 1] == xs[i]:
            j += 1
        out.append(f"{xs[i]} x{j - i + 1} (step{i}-{j})")
        i = j + 1
    return out


def short(path: str) -> str:
    """prim パスの末尾要素だけを返す(ログを短くするため)。"""
    return path.rsplit("/", 1)[-1]


def _f3(v: Any) -> Vec3:
    """carb.Float3 を (x, y, z) に変換する。小数第 7 位で丸めて比較しやすくする。"""
    return (round(float(v.x), 7), round(float(v.y), 7), round(float(v.z), 7))


def to_snapshot(report: tuple[Any, ...]) -> ReportSnapshot:
    """get_contact_report() の戻り値(ヘッダ配列, データ配列[, ...])を ReportSnapshot に変換する。"""
    from pxr import PhysicsSchemaTools

    headers_vec, data_vec = report[0], report[1]
    snap = ReportSnapshot(tuple_len=len(report))
    for h in headers_vec:
        snap.headers.append(
            HeaderRec(
                type=str(h.type.name),
                collider0=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                collider1=str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
                contact_data_offset=int(h.contact_data_offset),
                num_contact_data=int(h.num_contact_data),
                stage_id=int(h.stage_id),
            )
        )
    for d in data_vec:
        snap.data.append(DataRec(position=_f3(d.position), normal=_f3(d.normal), impulse=_f3(d.impulse), separation=round(float(d.separation), 7)))
    return snap


def pull() -> ReportSnapshot:
    """検出器と同じ get_contact_report() を 1 回呼び、スナップショットにして返す。"""
    return to_snapshot(phys_sim().get_contact_report())


class ContactSubscriber:
    """push 型(subscribe_contact_report_events)で届いた内容を、届いた時点の状況と一緒に覚える。

    in_fetch_flag は呼び出し側が fetch_results の前後で立て下げするフラグ。
    コールバックが fetch_results の中(同期)で呼ばれるのか、後(app.update 等)で呼ばれるのかを判別する。
    """

    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []
        self.in_fetch_flag: bool = False
        self.sub: Any = phys_sim().subscribe_contact_report_events(self._cb)

    def _cb(self, headers: Any, data: Any) -> None:
        snap = to_snapshot((headers, data))
        self.calls.append({"inside_fetch_results": self.in_fetch_flag, "summary": snap.summary(), "snapshot": snap})

    def close(self) -> None:
        self.sub = None
