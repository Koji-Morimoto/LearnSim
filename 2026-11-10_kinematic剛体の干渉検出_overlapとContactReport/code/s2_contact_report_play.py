"""S2: kinematic 剛体を Play 中に動かし、Contact Report で接触の始まり・継続・終わりを受け取る。

Script Editor にそのまま貼って実行できる。開いているステージは破棄して新しいステージを作る。
シーン: Mover(kinematic、動かす) / Target(kinematic、動かさない) / Wall(静的コライダー) /
        Neighbor(kinematic、Mover と最初から重なっているが FilteredPairs で除外する)。一辺 0.2 m の立方体。
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import ContactEventType
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics

MOVER = "/World/Mover"
TARGET = "/World/Target"
WALL = "/World/Wall"
NEIGHBOR = "/World/Neighbor"


def make_box(stage, path, x, kinematic, y=0.0):
    """一辺 0.2 m の箱。親 Xform が剛体、子の Cube がコライダー。kinematic=False なら静的コライダー。"""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.5))
    cube = UsdGeom.Cube.Define(stage, path + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        body = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        body.CreateKinematicEnabledAttr(True)  # ← 設定 1: kinematic 剛体にする
        report = PhysxSchema.PhysxContactReportAPI.Apply(xform.GetPrim())  # ← 設定 2: 剛体に ContactReportAPI を付ける
        report.CreateThresholdAttr(0.0)  # ← 設定 3: 力のしきい値を 0 にする(既定は 1。kinematic 同士は力が出ないので 0 が必要)


def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)  # CPU 物理
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateReportKinematicKinematicPairsAttr(True)  # ← 設定 4: kinematic 同士の接触を報告させる
    physx_scene.CreateReportKinematicStaticPairsAttr(True)  # ← 設定 5: kinematic と静的コライダーの接触を報告させる
    make_box(stage, MOVER, 0.05, kinematic=True)
    make_box(stage, TARGET, 0.5, kinematic=True)
    make_box(stage, WALL, 1.0, kinematic=False)
    make_box(stage, NEIGHBOR, 0.05, kinematic=True, y=0.15)  # Mover と 5 cm 重なった位置
    # ← 設定 6(必要な組だけ): 報告させたくない組は FilteredPairsAPI で除外する(関節でつながった隣のリンクなど)
    filtered = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(MOVER))
    filtered.CreateFilteredPairsRel().AddTarget(NEIGHBOR)


class ContactListener:
    """Contact Report を受け取り、いま接触している組と、その最小の separation を持つ。"""

    def __init__(self):
        self.touching = {}  # (剛体パス, 剛体パス) -> 最小の separation [m]。負ならめり込んでいる
        self.events = []  # この update で届いたイベント
        self._subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact)

    def _on_contact(self, headers, contact_data):
        for header in headers:
            body0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            body1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            pair = tuple(sorted((body0, body1)))
            points = [contact_data[header.contact_data_offset + i] for i in range(header.num_contact_data)]
            separation = min((p.separation for p in points), default=None)
            if header.type == ContactEventType.CONTACT_FOUND:
                self.touching[pair] = separation
                self.events.append(("FOUND", pair, separation))
            elif header.type == ContactEventType.CONTACT_PERSIST:
                self.touching[pair] = separation
                self.events.append(("PERSIST", pair, separation))
            elif header.type == ContactEventType.CONTACT_LOST:
                self.touching.pop(pair, None)
                self.events.append(("LOST", pair, None))

    def take_events(self):
        events, self.events = self.events, []
        return events

    def penetrating(self):
        """separation が 0 以下の組だけ(Contact Report は触れる少し手前から届くので、干渉はこれで判定する)。"""
        return sorted(pair for pair, sep in self.touching.items() if sep is not None and sep <= 0.0)

    def close(self):
        self._subscription = None


def short(pair):
    """表示用。静的コライダーは剛体が無いので、コライダーの prim パスがそのまま相手の名前になる。"""
    return " - ".join(p.replace("/World/", "") for p in pair)


async def main():
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    await context.new_stage_async()
    stage = context.get_stage()
    build_scene(stage)
    await app.next_update_async()

    listener = ContactListener()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(3):
        await app.next_update_async()
    listener.take_events()

    results = []
    xs = [round(0.05 + 0.1 * k, 2) for k in range(11)] + [1.05, 1.05]  # 最後の 2 回は動かさない
    for x in xs:
        stage.GetPrimAtPath(MOVER).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, 0.5))
        await app.next_update_async()
        events = [(kind, short(pair), None if sep is None else round(sep, 4)) for kind, pair, sep in listener.take_events()]
        penetrating = [short(p) for p in listener.penetrating()]
        results.append((x, events, penetrating))
        print(f"x={x:.2f}  イベント: {events}  めり込んでいる組: {penetrating}")

    timeline.stop()
    await app.next_update_async()
    listener.close()
    print("[DONE] S2")
    return results


task = asyncio.ensure_future(main())
