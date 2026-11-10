"""アーティキュレーションを無効化しないと幾何モードで動かせないのかを調べる。GUI の中で実行する。

Arm(Base は世界に固定、Link は Y 軸ヒンジ)の Link が通る位置に、kinematic の障害物 Obstacle を幾何レイヤーに置く。
Link を θ = 0° → 90° → 0° に振って、障害物との干渉が取れるか・Link が指令どおりの角度になるかを見る。

    A  アーティキュレーション有効 + リンクを kinematic   : 関節角(set_dof_positions)で振る
    B  アーティキュレーション有効 + リンクは dynamic(重力 0): 関節角を毎フレーム直接書く(テレポート)
    B2 アーティキュレーション有効 + リンクは dynamic(重力 0): 硬い位置ドライブの目標角で振る(普通の物理の動かし方)
    C  アーティキュレーション無効 + リンクを kinematic(現行の幾何モード): 順運動学で Link の xform を直接書く
結果: ../results/v04_articulation.json
"""

import importlib
import json
import math
import os
import sys

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
ROOT_DIR = os.path.dirname(CODE_DIR)
DEMO_PATH = os.path.join(ROOT_DIR, "demo", "geometric_mode_demo.usda")
RESULT_PATH = os.path.join(ROOT_DIR, "results", "v04_articulation.json")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)
import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import geo_demo  # noqa: E402

importlib.reload(geo_demo)
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
ctx = omni.usd.get_context()
R: dict = {}

PIVOT = Gf.Vec3d(0.15, -5.0, 1.5)  # ヒンジの位置(Base の +X 面)
OBST = "/World/Obstacle"
OBST_ANGLE = 45.0  # 障害物を置く角度 [deg]
FRAMES = 180  # 0→90→0 を何フレームで振るか

try:
    _geo_driver.stop()  # type: ignore[name-defined]  # noqa: F821
except NameError:
    pass


async def frames(n: int) -> None:
    for _ in range(n):
        await app.next_update_async()


async def stop() -> None:
    tl.stop()
    await frames(3)


def save() -> None:
    with open(RESULT_PATH, "w", encoding="utf-8") as f:
        json.dump(R, f, ensure_ascii=False, indent=1, default=str)


def cmd_deg(i: int) -> float:
    """i フレーム目の指令角 [deg]。0→90→0 の三角波。"""
    u = i / FRAMES * 2
    return 90.0 * (u if u <= 1 else 2 - u)


def link_angle(st: Usd.Stage) -> float:
    """Link の実際の角度 [deg](ワールドでの Link の +X 方向から求める。下向きが正)。"""
    m = UsdGeom.Xformable(st.GetPrimAtPath(geo_demo.PATHS["Link"])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    d = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.degrees(math.atan2(-d[2], d[0]))


class Watch:
    """接触レポートと carb のエラー/警告ログを集める。"""

    def __init__(self) -> None:
        self.pairs: dict[str, int] = {}
        self.logs: list[str] = []
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(self._cb)
        self._logging = carb.logging.acquire_logging()
        self._logger = self._logging.add_logger(self._log)

    def _cb(self, headers, data) -> None:
        for h in headers:
            k = " - ".join(sorted(str(PhysicsSchemaTools.intToSdfPath(x)).split("/")[-1] for x in (h.collider0, h.collider1)))
            self.pairs[k] = self.pairs.get(k, 0) + 1

    def _log(self, source, level, filename, line, msg) -> None:
        if level >= carb.logging.LEVEL_WARN and len(self.logs) < 30 and ("hysx" in source or "hysics" in source or "hysX" in msg):
            self.logs.append(f"[{source}] {msg}"[:300])

    def close(self) -> None:
        self.sub = None
        self._logging.remove_logger(self._logger)


async def open_fresh() -> Usd.Stage:
    await ctx.new_stage_async()
    await frames(2)
    cached = Sdf.Layer.Find(DEMO_PATH)
    if cached is not None:
        cached.Reload(force=True)
    await ctx.open_stage_async(DEMO_PATH)
    await frames(5)
    return ctx.get_stage()


def add_obstacle(st: Usd.Stage) -> None:
    """Link の通り道(θ = OBST_ANGLE、ヒンジから 0.6 m)に kinematic の箱を置く。幾何レイヤーに書く。"""
    a = math.radians(OBST_ANGLE)
    c = UsdGeom.Cube.Define(st, OBST)
    c.CreateSizeAttr(0.2)
    c.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(PIVOT + Gf.Vec3d(0.6 * math.cos(a), 0, -0.6 * math.sin(a)))
    c.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.8, 0.8)])
    p = c.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    UsdPhysics.RigidBodyAPI.Apply(p).CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)


def set_link_pose(st: Usd.Stage, deg: float) -> None:
    """順運動学: 角度 deg のときの Link の位置と向きを xform に直接書く(ケース C 用)。"""
    a = math.radians(deg)
    prim = st.GetPrimAtPath(geo_demo.PATHS["Link"])
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    orient = ops.get("xformOp:orient") or xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    ops["xformOp:translate"].Set(PIVOT + Gf.Vec3d(0.5 * math.cos(a), 0, -0.5 * math.sin(a)))
    orient.Set(Gf.Quatd(math.cos(a / 2), 0, math.sin(a / 2), 0))  # Y 軸まわりに a 回転


async def run_case(name: str) -> dict:
    await stop()
    st = await open_fresh()
    if name == "C":
        gm.enable(st)  # 現行: アーティキュレーション・ジョイントを無効化
    else:
        gm.enable(st, gm.Options(disable_articulations=False, disable_joints=False))
    with gm.edit_context(st):
        add_obstacle(st)
        if name in ("B", "B2"):
            for k in ("Base", "Link"):
                UsdPhysics.RigidBodyAPI(st.GetPrimAtPath(geo_demo.PATHS[k])).GetKinematicEnabledAttr().Set(False)
        if name == "B2":
            drive = UsdPhysics.DriveAPI(st.GetPrimAtPath("/World/Arm/Hinge"), "angular")
            drive.GetStiffnessAttr().Set(1.0e4)
            drive.GetDampingAttr().Set(1.0e2)
    obst0 = geo_demo.world_pos(st, OBST)
    w = Watch()
    out: dict = {"errors": None}
    tl.play()
    await frames(3)
    art = None
    if name != "C":
        try:
            from isaacsim.core.experimental.prims import Articulation

            art = Articulation("/World/Arm")
            out["dof_names"] = list(art.dof_names)
        except Exception as e:  # noqa: BLE001
            out["errors"] = f"Articulation: {type(e).__name__}: {e}"[:400]
    track: list[tuple[float, float]] = []
    for i in range(FRAMES + 1):
        deg = cmd_deg(i)
        try:
            if name == "C":
                with gm.edit_context(st):
                    set_link_pose(st, deg)
            elif name == "B2":
                UsdPhysics.DriveAPI(st.GetPrimAtPath("/World/Arm/Hinge"), "angular").GetTargetPositionAttr().Set(deg)
            elif art is not None:
                art.set_dof_positions(np.array([[math.radians(deg)]], dtype=np.float32))
                art.set_dof_velocities(np.zeros((1, 1), dtype=np.float32))
        except Exception as e:  # noqa: BLE001
            out["errors"] = f"frame {i}: {type(e).__name__}: {e}"[:400]
            break
        await frames(1)
        track.append((deg, link_angle(st)))
    err = [abs(a - c) for c, a in track]
    out.update(
        {
            "max_abs_angle_error_deg": round(max(err), 2) if err else None,
            "actual_angle_at_cmd_90": round(track[FRAMES // 2][1], 2) if len(track) > FRAMES // 2 else None,
            "actual_angle_samples": [(round(c, 1), round(a, 1)) for c, a in track[:: FRAMES // 12]],
            "obstacle_moved_m": round((geo_demo.world_pos(st, OBST) - obst0).GetLength(), 5),
            "contacts": dict(w.pairs),
            "logs": w.logs,
        }
    )
    await stop()
    w.close()
    gm.disable(st)
    return out


for case in ("A", "B", "B2", "C"):
    R[case] = await run_case(case)
    save()

# 後片付け: デモを開き直して幾何モード + ドライバに戻す
st = await open_fresh()
gm.enable(st)
_geo_driver = geo_demo.Driver(st).start()
print(json.dumps(R, ensure_ascii=False, indent=1))
