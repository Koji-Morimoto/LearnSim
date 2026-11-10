"""幾何モード検証用のシーンと、幾何モード中に物体を動かすドライバ。

シーン(Z-up、1 unit = 1 m)。どれも「普通の物理シーン」として作る(重力 9.81、質量・摩擦・反発・ドライブあり):
    Lane1 (y=0)    : Mover(青)が X 方向に進み、Target(緑・剛体)と Wall(灰・静的コライダー)を通り抜ける
    Lane2 (y=2.5)  : OverlapA / OverlapB(黄)が最初から 5 cm めり込んでいる
    Lane3 (y=-2.5) : Floating(橙の球)が高さ 2 m に浮いている
    Arm   (y=-5)   : 2 リンクのアーティキュレーション。Base は世界に固定、Link は水平に置いた腕(ドライブなし)

物理のまま Play すると: Mover/Target/Floating は落ち、Overlap は押し離され、Arm の Link は垂れ下がる。
幾何モードで Play すると: 何も落ちず、Mover だけがドライバに書かれた位置へ動き、
    接触レポートの種類で色が変わる(FOUND = 赤、PERSIST = 紫、LOST = 青)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import omni.timeline
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

import geometric_mode as gm

BOX: float = 0.5  # Lane1/2 の立方体の一辺 [m]
LANE_Z: float = 0.6  # 立方体の中心の高さ [m]。床(上面 z=0)から 0.35 m 浮かせる
MOVER_START: float = -2.5  # Mover の X 開始位置 [m]
MOVER_SPEED: float = 1.0  # Mover の速さ [m/s]
MOVER_RANGE: float = 5.5  # 往復の片道距離 [m](-2.5 → 3.0)
# 接触イベントの種類ごとの表示色。FOUND / LOST は 1 回しか来ないので、一定時間(タイムライン時刻)色を残す
STATE_COLORS: dict[str, Gf.Vec3f] = {
    "found": Gf.Vec3f(0.95, 0.05, 0.05),  # 赤: 接触が始まった
    "persist": Gf.Vec3f(0.45, 0.0, 0.75),  # 紫: 接触が続いている(橙は描画で黄色に見え、kinematic の黄色い線と紛れる)
    "lost": Gf.Vec3f(0.0, 0.15, 1.0),  # 青: 接触が終わった
}
STATE_PRIORITY: tuple[str, ...] = ("found", "persist", "lost")  # 1 つのコライダーが複数の組に入るときの優先順
FOUND_HOLD: float = 0.3  # FOUND の色を残す時間 [s]
LOST_HOLD: float = 0.6  # LOST の色を残す時間 [s]

PATHS: dict[str, str] = {
    "Mover": "/World/Lane1/Mover",
    "Target": "/World/Lane1/Target",
    "Wall": "/World/Lane1/Wall",
    "OverlapA": "/World/Lane2/OverlapA",
    "OverlapB": "/World/Lane2/OverlapB",
    "Floating": "/World/Lane3/Floating",
    "Base": "/World/Arm/Base",
    "Link": "/World/Arm/Link",
    "LinkGeom": "/World/Arm/Link/Geom",
    "Ground": "/World/Ground/Geom",
}


# ------------------------------------------------------------------ シーン構築


def _cube(stage: Usd.Stage, path: str, pos: tuple[float, float, float], size: float, color: tuple[float, float, float]) -> Usd.Prim:
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(size)
    c.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return c.GetPrim()


def _dynamic(prim: Usd.Prim, mass: float, material: str) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material.Get(prim.GetStage(), material), UsdShade.Tokens.weakerThanDescendants, "physics"
    )


def build_original_scene(stage: Usd.Stage) -> None:
    """幾何モードを掛ける前の「普通の物理シーン」を作る。"""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(60 * 3600)  # 長時間 Play してもタイムラインが巻き戻らないように
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr(800)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(2500)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(40, 0, 30))

    # 物理マテリアル(幾何モードでは効かなくなる値の例)
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterial")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.8)
    pm.CreateDynamicFrictionAttr(0.6)
    pm.CreateRestitutionAttr(0.6)

    # 床: 位置は親 Xform、スケールは子に付ける(スケール付き静的コライダーの定石)
    g = UsdGeom.Xform.Define(stage, "/World/Ground")
    g.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    gc = UsdGeom.Cube.Define(stage, PATHS["Ground"])
    gc.CreateSizeAttr(1.0)
    gc.AddScaleOp().Set(Gf.Vec3f(12, 14, 0.1))
    gc.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.27)])
    UsdPhysics.CollisionAPI.Apply(gc.GetPrim())

    for lane in ("Lane1", "Lane2", "Lane3"):
        UsdGeom.Xform.Define(stage, f"/World/{lane}")

    _dynamic(_cube(stage, PATHS["Mover"], (MOVER_START, 0, LANE_Z), BOX, (0.15, 0.35, 0.9)), 2.0, "/World/PhysicsMaterial")
    _dynamic(_cube(stage, PATHS["Target"], (0, 0, LANE_Z), BOX, (0.2, 0.75, 0.25)), 1.0, "/World/PhysicsMaterial")
    wall = UsdGeom.Xform.Define(stage, PATHS["Wall"])
    wall.AddTranslateOp().Set(Gf.Vec3d(2.0, 0, LANE_Z))
    wg = UsdGeom.Cube.Define(stage, PATHS["Wall"] + "/Geom")
    wg.CreateSizeAttr(1.0)
    wg.AddScaleOp().Set(Gf.Vec3f(0.1, 1.2, 1.2))
    wg.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.6)])
    UsdPhysics.CollisionAPI.Apply(wg.GetPrim())  # RigidBodyAPI なし = 静的コライダー

    half = BOX / 2 - 0.025  # 5 cm めり込ませる
    _dynamic(_cube(stage, PATHS["OverlapA"], (-half, 2.5, LANE_Z), BOX, (0.95, 0.8, 0.1)), 1.0, "/World/PhysicsMaterial")
    _dynamic(_cube(stage, PATHS["OverlapB"], (half, 2.5, LANE_Z), BOX, (0.95, 0.8, 0.1)), 1.0, "/World/PhysicsMaterial")

    sp = UsdGeom.Sphere.Define(stage, PATHS["Floating"])
    sp.CreateRadiusAttr(0.25)
    sp.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, -2.5, 2.0))
    sp.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.5, 0.1)])
    _dynamic(sp.GetPrim(), 0.5, "/World/PhysicsMaterial")
    sp.GetPrim().CreateAttribute("physics:velocity", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(0.5, 0, 0))

    _build_arm(stage)


def _build_arm(stage: Usd.Stage) -> None:
    """世界に固定した Base と、Y 軸ヒンジで繋いだ水平の Link からなるアーティキュレーション。"""
    arm = UsdGeom.Xform.Define(stage, "/World/Arm")
    UsdPhysics.ArticulationRootAPI.Apply(arm.GetPrim())

    base = _cube(stage, PATHS["Base"], (0, -5, 1.5), 0.3, (0.5, 0.3, 0.7))
    UsdPhysics.CollisionAPI.Apply(base)
    UsdPhysics.RigidBodyAPI.Apply(base)
    UsdPhysics.MassAPI.Apply(base).CreateMassAttr(1.0)

    link = UsdGeom.Xform.Define(stage, PATHS["Link"])
    link.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.65, -5, 1.5))
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(0.5)
    lg = UsdGeom.Cube.Define(stage, PATHS["LinkGeom"])
    lg.CreateSizeAttr(1.0)
    lg.AddScaleOp().Set(Gf.Vec3f(1.0, 0.1, 0.1))
    lg.CreateDisplayColorAttr([Gf.Vec3f(0.75, 0.5, 0.95)])
    UsdPhysics.CollisionAPI.Apply(lg.GetPrim())

    fix = UsdPhysics.FixedJoint.Define(stage, "/World/Arm/FixToWorld")
    fix.CreateBody1Rel().SetTargets([PATHS["Base"]])
    fix.CreateLocalPos0Attr(Gf.Vec3f(0, -5, 1.5))

    hinge = UsdPhysics.RevoluteJoint.Define(stage, "/World/Arm/Hinge")
    hinge.CreateBody0Rel().SetTargets([PATHS["Base"]])
    hinge.CreateBody1Rel().SetTargets([PATHS["Link"]])
    hinge.CreateAxisAttr(UsdPhysics.Tokens.y)
    hinge.CreateLocalPos0Attr(Gf.Vec3f(0.15, 0, 0))
    hinge.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0, 0))
    drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")  # 剛性 0 = 支えない。減衰だけ
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.5)


# ------------------------------------------------------------------ 計測・駆動


def world_pos(stage: Usd.Stage, path: str) -> Gf.Vec3d:
    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m.ExtractTranslation()


def mover_x(t: float) -> float:
    """時刻 t [s] の Mover の X。MOVER_START から MOVER_RANGE を往復する。"""
    period = 2 * MOVER_RANGE / MOVER_SPEED
    u = (t % period) * MOVER_SPEED
    return MOVER_START + (u if u <= MOVER_RANGE else 2 * MOVER_RANGE - u)


@dataclass
class PairStat:
    """1 組の接触の記録。separation は PhysX が返す距離(負 = めり込み、正 = contactOffset 内の手前)。"""

    first_time: float
    last_time: float
    events: int = 0
    min_separation: float = math.inf
    max_separation: float = -math.inf
    first_mover_x: float | None = None
    counts: dict[str, int] = field(default_factory=lambda: {"found": 0, "persist": 0, "lost": 0})
    log: list[tuple[float, str]] = field(default_factory=list)  # FOUND / LOST の (時刻, 種類)。最大 20 件


@dataclass
class Driver:
    """幾何モード中だけ Mover を mover_x(t) の位置へ動かし、接触中のコライダーをイベントの種類で色分けする。

    FOUND = 赤(FOUND_HOLD 秒)、PERSIST = 紫、LOST = 青(LOST_HOLD 秒)。
    書き込みは全部幾何レイヤーに入れるので、disable() すれば Mover の位置も色も元に戻る。
    物理モード(幾何モード無効)のときは何もしない。
    """

    stage: Usd.Stage
    pairs: dict[tuple[str, str], PairStat] = field(default_factory=dict)
    active: set[tuple[str, str]] = field(default_factory=set)
    _found_at: dict[tuple[str, str], float] = field(default_factory=dict)
    _lost_at: dict[tuple[str, str], float] = field(default_factory=dict)
    _painted: dict[str, str] = field(default_factory=dict)  # コライダー → 今塗っている状態
    _subs: list[Any] = field(default_factory=list)
    _last_t: float = 0.0

    def start(self) -> "Driver":
        from omni.physx import get_physx_simulation_interface

        self._subs.append(get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact))
        self._subs.append(_subscribe_update(self._on_update))
        return self

    def stop(self) -> None:
        self._subs.clear()

    def reset_log(self) -> None:
        self.pairs.clear()
        self.active.clear()
        self._found_at.clear()
        self._lost_at.clear()

    def _on_contact(self, headers: Any, data: Any) -> None:
        from omni.physx.bindings._physx import ContactEventType

        kinds = {ContactEventType.CONTACT_FOUND: "found", ContactEventType.CONTACT_PERSIST: "persist", ContactEventType.CONTACT_LOST: "lost"}
        t = self._last_t
        mx = mover_x(t) if gm.is_enabled(self.stage) else None
        for h in headers:
            key: tuple[str, str] = tuple(sorted((str(PhysicsSchemaTools.intToSdfPath(h.collider0)), str(PhysicsSchemaTools.intToSdfPath(h.collider1)))))  # type: ignore[assignment]
            st = self.pairs.get(key)
            if st is None:
                st = self.pairs[key] = PairStat(t, t, first_mover_x=mx)
            st.last_time = t
            st.events += 1
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                s = float(data[i].separation)
                st.min_separation = min(st.min_separation, s)
                st.max_separation = max(st.max_separation, s)
            kind = kinds[h.type]
            st.counts[kind] += 1
            if kind != "persist" and len(st.log) < 20:
                st.log.append((t, kind))
            if kind == "found":
                self._found_at[key] = t
                self.active.add(key)
            elif kind == "persist":
                self.active.add(key)
            else:
                self._lost_at[key] = t
                self.active.discard(key)

    def _on_update(self, *_: Any) -> None:
        tl = omni.timeline.get_timeline_interface()
        if not gm.is_enabled(self.stage):
            return
        if tl.is_stopped():
            self.active.clear()
            self._found_at.clear()
            self._lost_at.clear()
        self._last_t = tl.get_current_time()
        with gm.edit_context(self.stage):
            if tl.is_playing():
                op = UsdGeom.Xformable(self.stage.GetPrimAtPath(PATHS["Mover"])).GetOrderedXformOps()[0]
                op.Set(Gf.Vec3d(mover_x(self._last_t), 0, LANE_Z))
            self._paint(self._last_t)

    def collider_states(self, t: float) -> dict[str, str]:
        """時刻 t に各コライダーをどの状態の色で塗るか。床は除く。"""
        pair_state: dict[tuple[str, str], str] = {}
        for k in self.active:
            pair_state[k] = "found" if t - self._found_at.get(k, -math.inf) < FOUND_HOLD else "persist"
        for k, tl_ in self._lost_at.items():
            if k not in self.active and 0.0 <= t - tl_ < LOST_HOLD:
                pair_state[k] = "lost"
        out: dict[str, str] = {}
        for k, s in pair_state.items():
            for p in k:
                if p == PATHS["Ground"]:
                    continue
                if p not in out or STATE_PRIORITY.index(s) < STATE_PRIORITY.index(out[p]):
                    out[p] = s
        return out

    def _paint(self, t: float) -> None:
        """状態が変わったコライダーだけ塗り直す。状態が無くなったら幾何レイヤーの色の上書きを消して元の色に戻す。"""
        want = self.collider_states(t)
        for p, s in want.items():
            if self._painted.get(p) != s:
                UsdGeom.Gprim(self.stage.GetPrimAtPath(p)).GetDisplayColorAttr().Set(Vt.Vec3fArray([STATE_COLORS[s]]))
        for p in set(self._painted) - set(want):
            UsdGeom.Gprim(self.stage.GetPrimAtPath(p)).GetDisplayColorAttr().Clear()  # 編集先(幾何レイヤー)の値だけ消える
        self._painted = want


def _subscribe_update(fn: Any) -> Any:
    """毎フレームの更新イベントを購読する(Kit 110 の eventdispatcher、無ければ旧 API)。"""
    import omni.kit.app

    try:
        import carb.eventdispatcher

        return carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE, on_event=fn, observer_name="geometric_mode.Driver"
        )
    except (ImportError, AttributeError):
        return omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(fn, name="geometric_mode.Driver")
