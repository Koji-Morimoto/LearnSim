# kinematic 剛体の干渉検出：overlap と Contact Report の使い方

作成日：2026-11-10
対象：Isaac Sim 6.2.0（`6.2.0-rc.14`）、omni.physx 110.4.4、CPU 物理
確認方法：この文書のコード 3 本を、GUI とは別プロセスの headless Isaac Sim（描画なし）で「貼って実行」と同じ形で走らせ、結果を期待値と比べた（[code/verify_snippets.py](code/verify_snippets.py)、結果は [results/verify_snippets.json](results/verify_snippets.json)）。**GUI の Script Editor に貼っての実行はしていない。**

## 1. どちらを使うか

kinematic 剛体 = `RigidBodyAPI` があり `physics:kinematicEnabled = true` の剛体。力では動かず、書いた位置にだけ動く。

| | overlap（シーンクエリ） | Contact Report |
|---|---|---|
| 分かること | いま重なっている相手のコライダー | 接触の始まり・継続・終わり、めり込み量（`separation`）、接触点 |
| 呼び方 | 調べたいコライダーごとに自分で問い合わせる | 登録したコールバックに、物理ステップごとにまとめて届く |
| 結果が出るタイミング | 書いて update 1 回後（Play 中） | **書いて update 2 回後**（1 回ぶん遅れる。§4） |
| 止まったまま重なっている組 | 拾える | **拾えない**（最初から重なっていて一度も動いていない組。§6） |
| 必要な設定 | kinematic にするだけ | kinematic + ContactReportAPI + シーンの報告フラグ 2 つ |
| 時間（箱 1000 個、うち 500 個が毎フレーム動く） | 全コライダーの問い合わせに 29 ms | 物理ステップ 1 回 0.95 ms に含まれる |
| Play しないで使う | 使える（§5） | 使える（`update_simulation` を回す。書いてから 2 ステップ目以降に届く。この文書ではコードを載せていない） |

## 2. 設定の一覧

| # | 設定 | 書く場所 | overlap | Contact Report | 無いとどうなるか（実測） |
|---|---|---|---|---|---|
| 1 | `physics:kinematicEnabled = true` | 剛体の prim（`RigidBodyAPI`） | 必要 | 必要 | 動的剛体になり、重力で落ちる |
| 2 | `PhysxContactReportAPI` を付ける | 剛体の prim | 不要 | 必要 | その剛体の接触が報告されない |
| 3 | `physxContactReport:threshold = 0` | 剛体の prim | 不要 | 推奨 | このコードの条件（めり込んだ組）では既定値のままでも報告された。11 月 1 日の実測では、既定値だと「触れる手前（すき間 +5 mm）」の接触が報告されなかった |
| 4 | `physxScene:reportKinematicKinematicPairs = true` | PhysicsScene の prim（`PhysxSceneAPI`） | 不要 | 必要 | kinematic 同士の接触が 0 件（kinematic と静的の組だけ届いた） |
| 5 | `physxScene:reportKinematicStaticPairs = true` | PhysicsScene の prim | 不要 | 必要 | kinematic と静的コライダーの接触が届かない見込み（11 月 2 日の実測は「フラグ 2 つとも無いと 0 件」。このフラグだけを外した場合は測っていない） |
| 6 | `FilteredPairsAPI` で相手を指定 | 組の片方の剛体 prim | 効かない前提で自分で除外する | 必要な組だけ | 除外しないと、その組も報告される（対照で確認） |
| 7 | CPU 物理（`enableGPUDynamics = false`） | PhysicsScene の prim | 推奨 | 推奨 | GPU 構成（`suppressReadback = true`）では、Play 中に USD / Fabric に書いた位置が PhysX に届かない（11 月 3 日の実測） |

設定は Play を始める前に書く。

ロボット（既存の Articulation）を kinematic にする場合は、上に加えて次が要る（11 月 2 日の実測、[検証結果_幾何モード.md](../geometric_mode_verification/検証結果_幾何モード.md)）。

| 設定 | 理由 |
|---|---|
| `physxArticulation:articulationEnabled = false` | リンクが kinematic だと、PhysX がジョイントを作れずアーティキュレーション自体ができない |
| 各 Joint の `physics:jointEnabled = false` | 同上 |
| Joint の両端のリンクに `FilteredPairsAPI` | Joint を無効にすると「つながった 2 リンクは衝突しない」除外も消え、隣のリンクが常に接触として報告される |
| リンクの prim を入れ子にしない（兄弟並び） | 入れ子の kinematic 剛体は、自分の xformOp を変えても接触ペアが作られない（11 月 5 日の実測） |

## 3. overlap を使う（Play 中）

Mover（kinematic）を x = 0.05 から 1.05 まで 0.1 ずつ動かす。Target（kinematic、x = 0.5）と Wall（静的、x = 1.0）は動かさない。箱は一辺 0.2 m なので、中心の距離が 0.2 m 未満なら重なる。

```python
"""S1: kinematic 剛体を Play 中に動かし、overlap_shape で「いま重なっている相手」を調べる。

Script Editor にそのまま貼って実行できる。開いているステージは破棄して新しいステージを作る。
シーン: Mover(kinematic、動かす) / Target(kinematic、動かさない) / Wall(静的コライダー)。一辺 0.2 m の立方体。
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
import usdrt
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

USE_FABRIC = False  # True: Mover の位置を Fabric だけに書く / False: USD に書く

MOVER = "/World/Mover"
TARGET = "/World/Target"
WALL = "/World/Wall"


def make_box(stage, path, x, kinematic):
    """一辺 0.2 m の箱。親 Xform が剛体、子の Cube がコライダー。kinematic=False なら静的コライダー。"""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.5))
    cube = UsdGeom.Cube.Define(stage, path + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        body = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        body.CreateKinematicEnabledAttr(True)  # ← kinematic 剛体にする設定


def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")  # 重力は既定(9.81)のまま。kinematic は落ちない
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)  # CPU 物理。GPU 構成では Play 中の USD / Fabric 書き込みが届かないことがある
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    # overlap だけなら reportKinematic*Pairs と ContactReportAPI は不要
    make_box(stage, MOVER, 0.05, kinematic=True)
    make_box(stage, TARGET, 0.5, kinematic=True)
    make_box(stage, WALL, 1.0, kinematic=False)


def move_usd(stage, path, x):
    stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, 0.5))


def move_fabric(path, x):
    rt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(rt_stage.GetFabricId(), rt_stage.GetStageIdAsStageId())
    matrix = hierarchy.get_local_xform(usdrt.Sdf.Path(path))
    matrix.SetTranslateOnly(usdrt.Gf.Vec3d(x, 0.0, 0.5))
    hierarchy.set_local_xform(usdrt.Sdf.Path(path), matrix)


def overlapping(collider_path):
    """collider_path の形状と重なっているコライダーのパス一覧(自分自身は除く)。"""
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True  # True を返すと次のヒットも受け取る

    id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(collider_path))
    get_physx_scene_query_interface().overlap_shape(id0, id1, report, False)
    return sorted(h for h in hits if h != collider_path)


async def main():
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    await context.new_stage_async()
    stage = context.get_stage()
    build_scene(stage)
    await app.next_update_async()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(3):  # 最初の update で PhysX にシーンが作られる
        await app.next_update_async()

    results = []
    for k in range(11):
        x = round(0.05 + 0.1 * k, 2)
        if USE_FABRIC:
            move_fabric(MOVER, x)
        else:
            move_usd(stage, MOVER, x)
        await app.next_update_async()  # ← Play 中は、書いてから update 1 回後に PhysX 上の位置が変わる
        hits = overlapping(MOVER + "/collider")
        # Target を問い合わせ側にして、Mover が PhysX 上でも動いていることを確かめる
        seen_from_target = MOVER + "/collider" in overlapping(TARGET + "/collider")
        results.append((x, hits, seen_from_target))
        print(f"x={x:.2f}  重なっている相手: {hits}  Target 側から見て Mover と重なる: {seen_from_target}")

    timeline.stop()
    await app.next_update_async()
    print("[DONE] S1")
    return results


task = asyncio.ensure_future(main())
```

出力（`USE_FABRIC = False` と `True` で同じ。期待値と全行一致）：

```text
x=0.05  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.15  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.25  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.35  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.45  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.55  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.65  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.75  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.85  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
x=0.95  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
x=1.05  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
[DONE] S1
```

- 位置を書いた後、`await app.next_update_async()` を 1 回はさんでから問い合わせる。Play 中は、書いた位置が PhysX に入るのは次の update。
- 「Target 側から見て Mover と重なる」は、Mover が PhysX の中でも動いていることの確認。`overlap_shape` は問い合わせた側の形状だけは書いた直後の位置で判定するので、片側からだけ調べると、PhysX に届いていなくても重なって見える。
- `USE_FABRIC = True` にすると Mover の位置を Fabric だけに書く。Play 中はこれでも PhysX に届く。Stop 中は届かない（§5）。
- overlap は CollisionGroup の設定に影響されない（11 月 2 日の実測）。調べたくない組は、返ってきた結果から自分で除く。

## 4. Contact Report を使う（Play 中）

§3 と同じ動きに加えて、Neighbor（kinematic、Mover と最初から 5 cm 重なっている）を置き、`FilteredPairsAPI` で Mover との組を除外する。最後の 2 回は Mover を動かさない。

```python
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
```

出力：

```text
x=0.05  イベント: []  めり込んでいる組: []
x=0.15  イベント: []  めり込んでいる組: []
x=0.25  イベント: []  めり込んでいる組: []
x=0.35  イベント: []  めり込んでいる組: []
x=0.45  イベント: [('FOUND', 'Mover - Target', -0.05)]  めり込んでいる組: ['Mover - Target']
x=0.55  イベント: [('PERSIST', 'Mover - Target', -0.15)]  めり込んでいる組: ['Mover - Target']
x=0.65  イベント: [('PERSIST', 'Mover - Target', -0.15)]  めり込んでいる組: ['Mover - Target']
x=0.75  イベント: [('PERSIST', 'Mover - Target', -0.05)]  めり込んでいる組: ['Mover - Target']
x=0.85  イベント: [('LOST', 'Mover - Target', None)]  めり込んでいる組: []
x=0.95  イベント: [('FOUND', 'Mover - Wall/collider', -0.05)]  めり込んでいる組: ['Mover - Wall/collider']
x=1.05  イベント: [('PERSIST', 'Mover - Wall/collider', -0.15)]  めり込んでいる組: ['Mover - Wall/collider']
x=1.05  イベント: [('PERSIST', 'Mover - Wall/collider', -0.15)]  めり込んでいる組: ['Mover - Wall/collider']
x=1.05  イベント: [('PERSIST', 'Mover - Wall/collider', -0.15)]  めり込んでいる組: ['Mover - Wall/collider']
[DONE] S2
```

読み方：

- **報告は update 1 回ぶん遅れる。** Mover が Target に重なるのは x = 0.35 からだが、FOUND が届くのは次の x = 0.45 のとき。届いた `separation`（−0.05）は、1 回前の位置 x = 0.35 のめり込み量（0.2 − 0.15 = 0.05 m）に一致する。LOST も同じく 1 回遅れる（離れたのは x = 0.75、届くのは x = 0.85）。
- `separation` は負ならめり込んでいる量 [m]。Contact Report は触れる少し手前から届くので（11 月 2 日の実測で約 1.7 cm 手前）、干渉かどうかは `separation <= 0` で判定する。
- 相手が静的コライダーのときは剛体が無いので、相手の名前はコライダーの prim パス（`Wall/collider`）になる。
- 一度始まった接触は、動かさなくても PERSIST が毎 update 届く（最後の 2 行）。
- Neighbor との組は 1 回も届かない。`FilteredPairsAPI` の行を外した対照では `Mover - Neighbor` の FOUND が届いた。
- コールバックを登録した戻り値（`self._subscription`）は変数に持ち続ける。手放すと登録が解除される。

## 5. overlap を Play しないで使う（Stop 中）

```python
"""S3: Play しないまま(Stop 中)、kinematic 剛体を動かして overlap_shape で調べる。

Script Editor にそのまま貼って実行できる。開いているステージは破棄して新しいステージを作る。
注意: update_simulation() はシーン全体の物理を 1 ステップ進める。動的剛体があると PhysX 上で動くので、
      動的剛体が無いシーンで使う。
"""

import asyncio

import omni.kit.app
import omni.usd
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

MOVER = "/World/Mover"
TARGET = "/World/Target"
WALL = "/World/Wall"


def make_box(stage, path, x, kinematic):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.5))
    cube = UsdGeom.Cube.Define(stage, path + "/collider")
    cube.CreateSizeAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim()).CreateKinematicEnabledAttr(True)


def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    make_box(stage, MOVER, 0.05, kinematic=True)
    make_box(stage, TARGET, 0.5, kinematic=True)
    make_box(stage, WALL, 1.0, kinematic=False)


def overlapping(collider_path):
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True

    id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(collider_path))
    get_physx_scene_query_interface().overlap_shape(id0, id1, report, False)
    return sorted(h for h in hits if h != collider_path)


async def main():
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    await context.new_stage_async()
    stage = context.get_stage()
    build_scene(stage)
    await app.next_update_async()

    physx = get_physx_interface()
    physx.force_load_physics_from_usd()  # ← Stop 中は PhysX にシーンが無いので、USD から読み込ませる

    results = []
    for k in range(11):
        x = round(0.05 + 0.1 * k, 2)
        stage.GetPrimAtPath(MOVER).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, 0.5))  # USD に書く(Fabric だけでは届かない)
        physx.update_simulation(1.0 / 60.0, 0.0)  # ← 1 ステップ進めると、kinematic の位置が PhysX に入る
        hits = overlapping(MOVER + "/collider")
        # Target を問い合わせ側にして、Mover が PhysX 上で動いていることも確かめる
        seen_from_target = MOVER + "/collider" in overlapping(TARGET + "/collider")
        results.append((x, hits, seen_from_target))
        print(f"x={x:.2f}  重なっている相手: {hits}  Target 側から見て Mover と重なる: {seen_from_target}")

    print("[DONE] S3")
    return results


task = asyncio.ensure_future(main())
```

出力（期待値と全行一致。タイムラインは Stop のまま）：

```text
x=0.05  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.15  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.25  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.35  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.45  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.55  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.65  重なっている相手: ['/World/Target/collider']  Target 側から見て Mover と重なる: True
x=0.75  重なっている相手: []  Target 側から見て Mover と重なる: False
x=0.85  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
x=0.95  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
x=1.05  重なっている相手: ['/World/Wall/collider']  Target 側から見て Mover と重なる: False
[DONE] S3
```

- Stop 中は PhysX にシーンが無いので、最初に `force_load_physics_from_usd()` を呼ぶ。
- 位置は USD に書く。Stop 中に Fabric だけへ書いた位置は、`update_simulation()` を呼んでも PhysX に届かない。
- `update_simulation()` を 1 回呼ぶと、kinematic 剛体の位置が USD から PhysX に入る。呼ばないと PhysX 上では元の位置のまま。
- `update_simulation()` はシーン全体の物理を 1 ステップ進める。動的剛体があると PhysX の中で動く（重力 9.81 の箱が 30 回で 5.0 → 3.73 m）。動的剛体が無いシーンで使う。
- `update_simulation()` は結果を USD に書かない。`update_transformations(False, True)` を呼ぶと Root Layer に書かれるので、元のファイルを変えたくないなら呼ばない。

## 6. 速度（Play 中）

測り方：箱 N 個を 0.25 m 間隔で並べ、偶数番の箱（N/2 個）を 1 フレームおきに 6 cm 動かして隣に 1 cm めり込ませ、次のフレームで戻す。headless、描画なし、CPU 物理。値は 60 回の中央値。コードは [code/p01_play_speed.py](code/p01_play_speed.py)、結果は [results/p01_play_speed.json](results/p01_play_speed.json)。

| N | 物理ステップ 1 回：動かさない | 物理ステップ 1 回：動かす、Contact Report あり | 物理ステップ 1 回：動かす、Contact Report なし | N/2 個を USD に書く | N/2 個を Fabric に一括で書く | 全 N 個に `overlap_shape` |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 0.18 ms | 0.27 ms | 0.22 ms | 0.28 ms | 0.34 ms | 0.29 ms |
| 30 | 0.19 ms | 0.27 ms | 0.23 ms | 0.36 ms | 0.33 ms | 0.83 ms |
| 100 | 0.19 ms | 0.33 ms | 0.24 ms | 0.72 ms | 0.33 ms | 3.03 ms |
| 300 | 0.20 ms | 0.52 ms | 0.31 ms | 1.81 ms | 0.34 ms | 8.35 ms |
| 1000 | 0.21 ms | 0.95 ms | 0.55 ms | 5.67 ms | 0.35 ms | 29.2 ms |

- **kinematic 剛体だけのシーンの物理ステップは、1000 個でも 1 ms 未満だった。** Contact Report ありで、毎 update 約 490 件の報告が届く条件での値。
- 物理ステップは「ステップ直前」と「ステップ直後」のコールバックの間の時間。その前に行われる USD / Fabric の変更の取り込みは含まない。
- update 全体の時間は、N = 1000 でもどの条件でも 16.70 ms のままだった。Play 中の update は 1/60 秒に揃えて待つので（`/app/player/useFixedTimeStepping = true`）、取り込みを含めた全部の処理が 1/60 秒に収まっていることだけが分かる。取り込みだけの時間は測れていない。
- 全部の組を調べるなら、`overlap_shape` を全コライダーに呼ぶより Contact Report のほうが軽い（N = 1000 で 29 ms 対 物理ステップの増加分 0.74 ms）。ただしコールバックの中で Python が接触点を 1 個ずつ読む時間は、この表に入っていない（測定では件数を数えただけ）。
- 書き込みは、動かす個数が約 15 個（N = 30）を超えると Fabric の一括書き込みのほうが速い。

## 7. 注意点のまとめ

| 注意点 | 内容 | 根拠 |
|---|---|---|
| Contact Report は 1 update 遅れる | 書いた位置での接触は、次の次の update で届く | §4 |
| 一度も動いていない組は Contact Report に出ない | 最初から重なっていて、どちらも動かしていない組は 0 件。1 cm 動かすと FOUND が届く。overlap なら拾える | 11 月 10 日の実測（G02） |
| 触れる前から報告される | `separation <= 0` で判定する | 11 月 2 日の実測 |
| 入れ子の kinematic 剛体 | 接触ペアが作られない。兄弟並びにする | 11 月 5 日の実測 |
| Stop 中の Fabric 書き込み | PhysX に届かない。Stop 中は USD に書く | §5 |
| GPU 物理 | Play 中の USD / Fabric 書き込みが届かない構成がある。CPU 物理で使う | 11 月 3 日の実測 |
| 動的剛体があるシーン | Play すると普通に物理で動く。kinematic とぶつかれば押される | — |

## 8. ファイル

| ファイル | 内容 |
|---|---|
| [code/s1_overlap_play.py](code/s1_overlap_play.py) | §3 のコード |
| [code/s2_contact_report_play.py](code/s2_contact_report_play.py) | §4 のコード |
| [code/s3_overlap_stop.py](code/s3_overlap_stop.py) | §5 のコード |
| [code/verify_snippets.py](code/verify_snippets.py) | 上の 3 本を headless で走らせて期待値と比べる。設定を 1 つずつ外した対照（FilteredPairs なし、kinematic 同士の報告フラグなし、threshold 既定値）も走らせる |
| [code/p01_play_speed.py](code/p01_play_speed.py) | §6 の測定 |
| [code/p00_pacing_probe.py](code/p00_pacing_probe.py) | Play 中の update が 16.7 ms に揃う原因の設定を探した下調べ |
| [code/run.ps1](code/run.ps1) | 実行用。`powershell -File run.ps1 verify_snippets.py` |
| [results/](results/) | 結果（`*.json`）と実行ログ（`*.log`） |

検証スクリプトは `C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code\bplib.py` と `bp_verify.kit` を使う。関連する実測：[設計案_GeometricRobot.md](../geometric_robot_design/設計案_GeometricRobot.md) の §9.1・§9.2。
