# ハウツー / レシピ集（`myext.robot.geometric_newton`）

この文書は、起動中の Isaac Sim 6.2.0 の Script Editor にそのまま貼り付けて動く実用コード集（**How-to**）です。各レシピは先頭の `ISAAC_ROOT`（既定 `r"C:\isaacsim-6.2.0"`）から同梱アセットを自動参照し、ステージ作成・照明・ロボット配置までを自己完結で行います（実行すると開いているステージは新規ステージへ置き換わります）。

- コマンドライン単独起動にも対応した完全版デモは `demo/demo_01_*.py`〜`demo/demo_06_*.py`（一覧は [README.md](README.md)）にあります。
- 設計の背景や内部構造は [explanation_architecture.md](explanation_architecture.md) を参照してください。
- 全クラス・引数・既定値の仕様は [reference_api.md](reference_api.md) を参照してください。

---

## レシピ 1：既存 Articulation を `Play=ON` で 0 フレーム遅延・Fabric 駆動する

`Play=ON` のまま周囲の PhysX 動的剛体（`/World/DynamicBox`）を重力で落下させつつ、UR10e は `GeometricLayer` で物理を無効化して Fabric 駆動し、同一フレーム（Frame 0）で自己干渉 5 組を検出します。

```python
import asyncio
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim()).CreateEnableGPUDynamicsAttr(False)

    # 共存する PhysX 動的剛体
    dyn = UsdGeom.Cube.Define(stage, "/World/DynamicBox")
    dyn.CreateSizeAttr(0.18)
    dyn.AddTranslateOp().Set(Gf.Vec3d(-0.9, -0.5, 1.4))
    UsdPhysics.RigidBodyAPI.Apply(dyn.GetPrim())
    UsdPhysics.CollisionAPI.Apply(dyn.GetPrim())
    UsdPhysics.MassAPI.Apply(dyn.GetPrim()).CreateMassAttr(1.0)

    stage.DefinePrim("/World/UR10e", "Xform").GetReferences().AddReference(UR10E_USD.as_posix())
    await app.next_update_async()

    # 既定値: collision_backend="newton", write_target="fabric", apply_layer=True
    robot = GeometricRobot.from_articulation(stage, "/World/UR10e")
    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])
    robot.set_dof_positions(q_safe)
    robot.flush_to_usd()

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await app.next_update_async()

    # Fabric のみに書き込み（USD フラッシュなし）-> Frame 0 で 5 組すべて検出
    robot.set_dof_positions(q_self)
    pairs = robot.check_collisions("within")
    print(f"Frame 0 検出={len(pairs)}組:", [(p.names, round(p.separation * 1e3, 1)) for p in pairs])

    tl.stop()
    await app.next_update_async()
    robot.close()


asyncio.ensure_future(main())
```

---

## レシピ 2：自前コリジョン空間 `GeometricScene` でロボット＋環境物（静的・動的）の干渉とクリアランスを測る

`UsdPhysics.Scene` を作成せず、`GeometricScene` 単体で移動障害物の同期（`sync_environment()` / `set_obstacle_pose()`）とクリアランス計測（`compute_clearances()`）を行います。

```python
import asyncio
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot, GeometricScene


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")  # UsdPhysics.Scene は不要

    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    box = UsdGeom.Cube.Define(stage, "/World/Obstacles/Box0")
    box.CreateSizeAttr(0.22)
    box_op = box.AddTranslateOp()
    box_op.Set(Gf.Vec3d(2.0, 0.0, 0.5))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    stage.DefinePrim("/World/UR10e", "Xform").GetReferences().AddReference(UR10E_USD.as_posix())
    await app.next_update_async()

    scene = GeometricScene(stage)
    robot = GeometricRobot.from_articulation(stage, "/World/UR10e", scene=scene)
    scene.add_environment("/World/Obstacles", exclude_robots=True)
    robot.set_dof_positions(np.array([0.0, -0.55, 0.90, -0.35, 1.57, 0.0]))
    tip = robot.links["wrist_3_link"].get_world_pose()[0]

    # 1. USD 上で障害物を動かし sync_environment() でモデル再構築なし同期
    box_op.Set(Gf.Vec3d(*tip.tolist()))
    scene.sync_environment()
    print("干渉:", [(p.names, round(p.separation * 1e3, 1)) for p in robot.check_collisions("environment")])

    # 2. USD を触らず set_obstacle_pose() で少し離し、クリアランス計測
    scene.set_obstacle_pose("/World/Obstacles/Box0", position=(tip + np.array([0.0, 0.20, 0.0])).tolist())
    near = robot.compute_clearances("environment", max_distance=0.15)
    print("クリアランス:", [(p.names, round(p.separation * 1e3, 1)) for p in near])

    # 3. 安全マージン込みの干渉判定（5 cm 以内を干渉扱い）
    print("マージン5cm:", [p.names for p in robot.check_collisions("environment", clearance=0.05)])

    scene.sync_environment(reset_manual_overrides=True)
    robot.close()
    scene.close()


asyncio.ensure_future(main())
```

---

## レシピ 3：複数ロボットの共有シーンとロボット間干渉（`check_between`）

2 台の UR10e を 1 つの `GeometricLayer` と 1 つの `GeometricScene` に登録し、ベース移動とロボット間干渉（`kind == "between_robots"`）を調べます。

```python
import asyncio
from pathlib import Path
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricLayer, GeometricRobot, GeometricScene


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    for path, x in (("/World/Robot0", 0.0), ("/World/Robot1", 1.5)):
        p = stage.DefinePrim(path, "Xform")
        p.GetReferences().AddReference(UR10E_USD.as_posix())
        UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.0))
    await app.next_update_async()

    layer = GeometricLayer(stage)
    scene = GeometricScene(stage)
    r0 = GeometricRobot.from_articulation(stage, "/World/Robot0", layer=layer, scene=scene, apply_layer=False)
    r1 = GeometricRobot.from_articulation(stage, "/World/Robot1", layer=layer, scene=scene, apply_layer=False)
    layer.apply()

    print("初期距離(1.5m)の干渉:", scene.check_between(r0.link_set, r1.link_set))

    # ベース姿勢を動的に接近させると root FREE joint 経由で即座に反映される
    r1.set_world_poses(positions=[0.10, 0.0, 0.0])
    hits = scene.check_between(r0.link_set, r1.link_set)
    print("接近時(0.1m)のロボット間干渉:", [(p.names, p.kind, round(p.separation * 1e3, 1)) for p in hits])

    r0.close(remove_layer=False)
    r1.close(remove_layer=False)
    layer.remove()
    scene.close()


asyncio.ensure_future(main())
```

---

## レシピ 4：`GeometricRobotView` で多数台・多数姿勢を GPU バッチ処理する

8 台の UR10e を `GeometricRobotView` でまとめ、`BatchedNewtonScene`（CUDA Graph + GPU 直接 Fabric 書込）で一括 FK・一括干渉・一括クリアランスを実行します。

```python
import asyncio
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobotView


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    paths = []
    for i in range(8):
        p = f"/World/UR_{i}"
        prim = stage.DefinePrim(p, "Xform")
        prim.GetReferences().AddReference(UR10E_USD.as_posix())
        r, c = divmod(i, 4)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(c * 1.6, r * 1.8, 0.0))
        paths.append(p)
    await app.next_update_async()

    view = GeometricRobotView.from_articulations(
        stage,
        paths,
        write_target="fabric",
        collision_backend="newton",
        fk_backend="newton",
        use_cuda_graph=True,
        use_gpu_fabric_writer=True,
    )

    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0], dtype=np.float32)
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0], dtype=np.float32)
    q_batch = np.stack([q_self if i % 2 else q_safe for i in range(len(view))])
    view.set_dof_positions(q_batch)

    hits_per_robot = view.check_collisions_batch("within")
    cl_per_robot = view.compute_clearances_batch("within", max_distance=0.08)
    print("各ロボットの干渉数:", [len(h) for h in hits_per_robot])
    print("各ロボットの近接数:", [len(c) for c in cl_per_robot])

    view.close()


asyncio.ensure_future(main())
```

---

## レシピ 5：パラメータ（DH / 修正 DH / スクリュー軸 / URDF）からのロボット構築と `attach()` 結合

外部 USD 不要で DH・MDH・スクリュー軸・同梱 Franka URDF からモデルを生成し、台車＋双腕へ結合します。

```python
import asyncio
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.usd
from pxr import UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
FRANKA_URDF = ISAAC_ROOT / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import (
    DHParameters,
    DHRow,
    GeometricLinkSpec,
    GeometricRobot,
    GeometricRobotModel,
    GeometricVisualStyle,
    MDHParameters,
    MDHRow,
    ScrewAxisRow,
    ScrewParameters,
    UrdfSource,
)
from myext.robot.geometric_newton import transforms as tf


async def main():
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    pi = np.pi
    lim = dict(lower_limit=-pi, upper_limit=pi)
    mdh_arm = MDHParameters(
        rows=[
            MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.40, **lim),
            MDHRow(alpha=-pi / 2, a=0.05, theta=-pi / 2, d=0.0, **lim),
            MDHRow(alpha=0.0, a=0.45, theta=0.0, d=0.0, **lim),
            MDHRow(alpha=-pi / 2, a=0.04, theta=0.0, d=0.42, **lim),
            MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, **lim),
            MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.10, **lim),
        ],
        tool=tf.translation(z=0.08),
    ).to_model()

    # 台車 + 左右 2 本のアームを attach() で結合
    cart = GeometricRobotModel("cart", (GeometricLinkSpec("body"),), ())
    dual = cart.attach(mdh_arm, "body", tf.translation(y=0.25), name="left").attach(mdh_arm, "body", tf.translation(y=-0.25), name="right")
    robot = GeometricRobot.from_model(stage, dual, "/World/DualArm", style=GeometricVisualStyle(collisions=True))
    # 既定 samples=0 は現在の 1 姿勢のみ検査（複数姿勢で常に重なる組だけ除外する場合は samples=16 等を指定）
    robot.ignore_touching_pairs()

    # 標準 DH / スクリュー軸 / 同梱 URDF からの生成
    dh_r = GeometricRobot.from_dh(stage, DHParameters(rows=[DHRow(theta=0.0, d=0.3, a=0.25, alpha=0.0)]), "/World/DH", base_transform=tf.translation(x=-1.0))
    sc_r = GeometricRobot.from_screw(stage, ScrewParameters(rows=[ScrewAxisRow.through((0, 0, 0.3), (0, 1, 0))]), "/World/Screw", base_transform=tf.translation(x=-1.0, y=0.6))
    ur_r = GeometricRobot.from_urdf(stage, UrdfSource(str(FRANKA_URDF)), "/World/Panda", base_transform=tf.translation(x=1.0))

    left = robot.get_component("left")
    left.set_dof_positions([0.3, 0.2, 0.4, 0.0, 0.2, 0.0])
    print("DualArm 左アーム内干渉:", left.check_collisions("within"))
    print("Panda DOFs:", ur_r.dof_names)

    for r in (robot, dh_r, sc_r, ur_r):
        r.close()


asyncio.ensure_future(main())
```

---

## レシピ 6：IK（`jacobian` / `ikpy` / 自作ソルバー登録）とアクティブ軸マスク

UR10e に対して 3 軸マスク付き位置 IK、6 軸位置＋姿勢 IK、自作 IK ソルバー登録を実行します。

```python
import asyncio
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.usd
from pxr import UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot, IKResult, IKSolver, IKSolverConfig, register_ik_solver


class ZeroIKSolver(IKSolver):
    def solve(self, target_position, target_orientation=None, *, seed=None) -> IKResult:
        return IKResult(dof_positions=np.zeros(self.chain.num_variables), success=True, position_error=0.0, orientation_error=0.0)


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.DefinePrim("/World/UR10e", "Xform").GetReferences().AddReference(UR10E_USD.as_posix())
    await app.next_update_async()

    robot = GeometricRobot.from_articulation(stage, "/World/UR10e")
    solver = robot.create_ik_solver("jacobian", "base_link", "wrist_3_link", IKSolverConfig(orientation_mode="none", num_restarts=4))

    # 1. 全軸で位置追従
    res = robot.solve_ik(solver, target_position=[0.55, 0.20, 0.45], apply=True)
    print("位置IK:", res.success, f"err={res.position_error:.2e} m")

    # 2. アクティブ軸マスク（最初の 3 軸のみ動かす）
    solver.set_active_dofs(["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"])
    res_3 = robot.solve_ik(solver, target_position=[0.50, 0.25, 0.50])
    print("3軸マスクIK:", res_3.success, f"err={res_3.position_error:.2e} m")

    # 3. 6-DOF 位置＋姿勢 IK（(w, x, y, z) 四元数）
    solver.set_active_dofs([True] * solver.chain.num_variables)
    solver.set_config(orientation_mode="full", orientation_tolerance=1e-3)
    res_6 = robot.solve_ik(solver, target_position=[0.50, 0.25, 0.50], target_orientation=[1.0, 0.0, 0.0, 0.0])
    print("6軸IK:", res_6.success, f"pos={res_6.position_error:.2e}, ori={res_6.orientation_error:.2e}")

    # 4. 自作ソルバー登録
    register_ik_solver("my_zero", ZeroIKSolver)
    custom = robot.create_ik_solver("my_zero", "base_link", "wrist_3_link")
    print("自作ソルバー:", robot.solve_ik(custom, [0.0, 0.0, 0.5]).success)

    robot.close()


asyncio.ensure_future(main())
```

---

## レシピ 7：VariantSet 切り替え・角度単位（度）・`CommandConverter` による外部指令変換

UR10e の `Gripper` VariantSet を実行中に切り替え、`angle_unit="deg"` と `when=` 条件付き `LinearCommandConverter` が連動する流れを確認します。

```python
import asyncio
from pathlib import Path
import omni.kit.app
import omni.usd
from pxr import UsdGeom

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot, LinearCommandConverter


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    prim = stage.DefinePrim("/World/UR10e", "Xform")
    prim.GetReferences().AddReference(UR10E_USD.as_posix())
    prim.GetVariantSet("Gripper").SetVariantSelection("Robotiq_2F_85")
    await app.next_update_async()

    robot = GeometricRobot.from_articulation(stage, "/World/UR10e", angle_unit="deg")
    robot.add_structure_listener(lambda old, new: print(f"構造更新: {len(old)} -> {len(new)} DOFs"))

    # 度単位と呼び出しごとの rad 指定の混用
    robot.set_dof_positions([45.0, -75.0], dof_indices=[0, 1])
    robot.set_dof_positions([0.5], dof_indices=[2], unit="rad")

    # Variant 条件付きコマンド変換を登録
    conv_140 = LinearCommandConverter(["grip_ratio"], ["finger_joint"], [[35.0]], output_angle_unit="deg")
    robot.set_command_converter("grip_140", conv_140, when={("", "Gripper"): "Robotiq_2F_140"})

    # Variant 切り替え -> GeometricLayer と GeometricScene が自動再構築
    robot.set_variant("Gripper", "Robotiq_2F_140")
    robot.apply_command({"grip_ratio": 0.6, "shoulder_pan_joint": 30.0})
    print("変換後 DOF(deg):", robot.joints["finger_joint"].get_position(), "逆変換:", robot.read_command())

    robot.close()


asyncio.ensure_future(main())
```

---

## レシピ 8：カスタムコライダー用 `.usda` Sublayer の実行時 ON/OFF 切替（非破壊・0 フレーム遅延）

元のロボット USD ファイルを変更せず、カスタムコライダー `.usda` を `GeometricLayer` 直下の Session Sublayer として差し込み、`set_collider_layer_enabled()` で実行時に Mute / Unmute を切り替えます。完全版アニメーションデモは `demo/demo_03_collider_sublayer_toggle.py` にあります。

```python
import asyncio
import shutil
import tempfile
from pathlib import Path
import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.add_path((ISAAC_ROOT / "extsUser/Extentions").as_posix())
mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)
from myext.robot.geometric_newton import GeometricRobot


async def main():
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    probe = UsdGeom.Cube.Define(stage, "/World/SafetyProbe")
    probe.CreateSizeAttr(0.16)
    probe_op = probe.AddTranslateOp()
    UsdPhysics.CollisionAPI.Apply(probe.GetPrim())

    stage.DefinePrim("/World/UR10e", "Xform").GetReferences().AddReference(UR10E_USD.as_posix())
    await app.next_update_async()

    robot = GeometricRobot.from_articulation(stage, "/World/UR10e")
    robot.set_dof_positions(np.array([0.0, -1.15, 1.35, -1.57, -1.57, 0.0]))
    wrist_pos, _ = robot.links["wrist_1_link"].get_world_pose()
    probe_op.Set(Gf.Vec3d(*(wrist_pos + np.array([0.0, 0.24, 0.0])).tolist()))
    robot.scene.sync_environment()

    # 1. 元コライダーを無効化し、wrist_1_link に半径 0.22 m の安全球を追加する .usda を一時生成
    tmp_dir = Path(tempfile.mkdtemp(prefix="geom_newton_recipe08_"))
    usda_path = tmp_dir / "custom_safety_colliders.usda"
    sub = Usd.Stage.CreateNew(str(usda_path))
    for cpaths in robot.collider_paths.values():
        for cp in cpaths:
            for anc in Sdf.Path(cp).GetPrefixes()[:-1]:
                if anc != Sdf.Path.absoluteRootPath:
                    sub.OverridePrim(anc)
            UsdPhysics.CollisionAPI.Apply(sub.OverridePrim(cp)).CreateCollisionEnabledAttr(False)
    lpath = robot.link_path_map["wrist_1_link"]
    for anc in Sdf.Path(lpath).GetPrefixes():
        if anc != Sdf.Path.absoluteRootPath:
            sub.OverridePrim(anc)
    bubble = UsdGeom.Sphere.Define(sub, f"{lpath}/CustomSafetyBubble")
    bubble.CreateRadiusAttr(0.22)
    UsdPhysics.CollisionAPI.Apply(bubble.GetPrim()).CreateCollisionEnabledAttr(True)
    sub.GetRootLayer().Save()

    print("元コライダー干渉:", len(robot.check_collisions("environment")))

    # 2. Session Layer（GeometricLayer 直下）へ差し込み -> 0 フレーム遅延で干渉検出
    layer_id = robot.attach_collider_layer(usda_path, target="session")
    print("カスタムON 干渉:", len(robot.check_collisions("environment")), "ver=", robot.collider_version)

    # 3. Mute で元コライダーへ復帰 -> Unmute で再びカスタム有効化
    robot.set_collider_layer_enabled(layer_id, False)
    print("カスタムOFF(Mute) 干渉:", len(robot.check_collisions("environment")), "ver=", robot.collider_version)
    robot.set_collider_layer_enabled(layer_id, True)
    print("カスタム再ON(Unmute) 干渉:", len(robot.check_collisions("environment")), "ver=", robot.collider_version)

    # 4. Sublayer を外して元構成へ完全復帰
    robot.detach_collider_layer(layer_id)
    robot.close()
    shutil.rmtree(tmp_dir, ignore_errors=True)


asyncio.ensure_future(main())
```

---

## レシピ 9：既存 `myext.robot.geometric` からの移行手順と PhysX バックエンドへの切替

既存の `myext.robot.geometric` 向けコードを `myext.robot.geometric_newton` へ移す手順です。

### 移行チェックリスト

1. **import パスと拡張機能 ID の置換**：
   - 拡張機能有効化：`"myext.robot.geometric"` → `"myext.robot.geometric_newton"`
   - Python import：`from myext.robot.geometric import ...` → `from myext.robot.geometric_newton import ...`
   - Kit 起動引数設定プレフィックス：`--/exts/myext.robot.geometric/...` → `--/exts/myext.robot.geometric_newton/...`
2. **既定の干渉判定バックエンドの違いを確認**：
   - `myext.robot.geometric_newton` は `collision_backend="newton"` が既定です。`check_collisions()` は `flush_to_usd()` を呼ばないため、USD 側に最新姿勢を書きたい処理がある場合は明示的に `robot.flush_to_usd()` を呼びます。
   - `GeometricCollisionPair` の既存フィールド（`link_a`, `link_b`, `path_a`, `path_b`, `kind`, `component_a`, `component_b`, `robot_a`, `robot_b`, `.names`）はすべて互換のまま、`separation`, `point_a`, `point_b`, `normal` が追加されています。

### PhysX バックエンド（従来挙動）へ切り替える場合

```python
# 方法 A: コンストラクタでロボット単位に指定
robot = GeometricRobot.from_articulation(
    stage,
    "/World/UR10e",
    collision_backend="physx",
    fk_backend="numpy",
)

# 方法 B: 呼び出し単位で一時的に PhysX を使う
physx_pairs = robot.check_collisions("all", backend="physx")

# 方法 C: 実行時にプロパティで切り替え
robot.collision_backend = "physx"

# 方法 D: 起動引数で全ロボットの既定値を変更
# --/exts/myext.robot.geometric_newton/collision_backend=physx
# --/exts/myext.robot.geometric_newton/fk_backend=numpy
```
