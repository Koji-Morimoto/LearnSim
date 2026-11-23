# ハウツー / レシピ集（`myext.robot.geometric_newton`）

この文書は、特定の実装タスクを最短で解決するための実用コード集（**How-to**）です。

- 設計の背景や内部構造は [explanation_architecture.md](explanation_architecture.md) を参照してください。
- 全クラス・引数・既定値の仕様は [reference_api.md](reference_api.md) を参照してください。

---

## レシピ 1：既存 Articulation を `Play=ON` で 0 フレーム遅延・Fabric 駆動する

タイムラインが `Play=ON` のまま、周囲の PhysX 動的剛体を物理シミュレーションさせつつ、ロボットだけは `GeometricLayer` で物理を無効化して Fabric 駆動し、同一フレーム（Frame 0）で干渉判定します。

```python
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from myext.robot.geometric_newton import GeometricRobot

stage = omni.usd.get_context().get_stage()
app = omni.kit.app.get_app()

# 既定値で collision_backend="newton", write_target="fabric", apply_layer=True
robot = GeometricRobot.from_articulation(stage, "/World/UR10e")

q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0])

# Play 開始前に初期姿勢を USD へ反映しておく
robot.set_dof_positions(q_safe)
robot.flush_to_usd()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()

# Fabric のみに書き込み（USD 書き出しなし）
robot.set_dof_positions(q_self)

# app.update() を待たず Frame 0 で全組を検出（USD は書き換わらない）
pairs = robot.check_collisions("within")
for p in pairs:
    print(p.names, f"separation={p.separation:.4f} m", "normal=", p.normal)

# 元の Articulation 構成へ戻す（Root Layer は無変更）
timeline.stop()
robot.close()
```

---

## レシピ 2：自前コリジョン空間 `GeometricScene` でロボット＋環境物（静的・動的）の干渉とクリアランスを測る

`UsdPhysics.Scene` の有無にかかわらず、静的・動的な環境コライダーとの干渉判定、めり込み量、近接クリアランスを 0 フレーム遅延で取得します。

```python
import numpy as np
from pxr import Gf, UsdGeom
from myext.robot.geometric_newton import GeometricRobot, GeometricScene

# 1. 明示的に GeometricScene を作り、対象環境ルートを登録する
scene = GeometricScene(stage)
robot = GeometricRobot.from_articulation(stage, "/World/UR10e", scene=scene)
scene.add_environment("/World/Obstacles", exclude_robots=True)

# 2. USD 上で静的障害物を動かした直後にモデル再構築なしで同期（use_fabric="auto" が既定）
obs_prim = stage.GetPrimAtPath("/World/Obstacles/Box0")
UsdGeom.Xformable(obs_prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(0.60, 0.20, 0.40))
scene.sync_environment()

hits = robot.check_collisions("environment")
if hits:
    print("干渉あり:", [(p.names, round(p.separation * 1e3, 2)) for p in hits])

# 3. USD / Fabric を触らず、Newton 側の障害物姿勢だけを高速に差し替えてクリアランス計測
scene.set_obstacle_pose("/World/Obstacles/Box0", position=[0.78, 0.20, 0.40], orientation=(1.0, 0.0, 0.0, 0.0))
near = robot.compute_clearances("environment", max_distance=0.15)
for p in near:
    print(f"{p.names}: distance={p.separation * 1e3:.1f} mm, point_a={p.point_a}, point_b={p.point_b}")

# 4. clearance 引数で「安全マージン込みの干渉判定」（3 cm 未満を干渉扱い）
margin_hits = robot.check_collisions("all", clearance=0.03)

# 5. set_obstacle_pose の手動上書きを解除し、USD / Fabric の姿勢へ戻す
scene.sync_environment(reset_manual_overrides=True)

robot.close()
scene.close()
```

---

## レシピ 3：複数ロボットの共有シーンとロボット間干渉（`check_between`）

複数台のロボットを 1 つの `GeometricLayer` と 1 つの `GeometricScene` に登録し、ロボット間干渉（`kind == "between_robots"`）や構成要素間干渉（`kind == "between_components"`）を調べます。

```python
from myext.robot.geometric_newton import GeometricLayer, GeometricRobot, GeometricScene

layer = GeometricLayer(stage)
scene = GeometricScene(stage)

# apply_layer=False で登録をまとめ、最後に 1 回だけ layer.apply() する
r0 = GeometricRobot.from_articulation(stage, "/World/Robot0", layer=layer, scene=scene, apply_layer=False)
r1 = GeometricRobot.from_articulation(stage, "/World/Robot1", layer=layer, scene=scene, apply_layer=False)
layer.apply()

# ベース姿勢を動的に移動しても root FREE joint 経由で即反映される
r1.set_world_poses(positions=[0.6, 0.0, 0.0])

# ロボット間の干渉とクリアランス
between_hits = scene.check_between(r0.link_set, r1.link_set)
between_margin = scene.check_between(r0.link_set, r1.link_set, clearance=0.05)

# 片方のロボットの特定リンク集合だけを対象にする場合
arm0_set = r0.get_component("arm").link_set if "arm" in r0.component_names else r0.link_set
partial_hits = scene.check_between(arm0_set, r1.link_set)

# 共有リソースの後片付け
r0.close(remove_layer=False)
r1.close(remove_layer=False)
layer.remove()
scene.close()
```

---

## レシピ 4：`GeometricRobotView` で多数台・多数姿勢を GPU バッチ処理する

同じ構造のロボット複数台を `GeometricRobotView` にまとめ、`BatchedNewtonScene`（multi-world + CUDA Graph）で順運動学と干渉・クリアランスを一括評価し、`GeometricGpuFabricWriter` で GPU から直接 Fabric へ書き込みます。

```python
import numpy as np
from myext.robot.geometric_newton import GeometricRobotView

paths = [f"/World/UR_{i}" for i in range(32)]
view = GeometricRobotView.from_articulations(
    stage,
    paths,
    write_target="fabric",
    collision_backend="newton",
    fk_backend="auto",                    # N >= 16 で自動的に Newton GPU FK を選択（強制は "newton"）
    environment_roots=["/World/Obstacles"],
    use_cuda_graph=True,                  # CUDA 環境で eval_fk + collide をグラフキャプチャ
    use_gpu_fabric_writer=True,           # wp.fabricarray("omni:fabric:worldMatrix") へ直接書き込み
)

# 全 32 台分の関節値 (32, num_dofs) を一括反映
q_batch = np.zeros((len(paths), view.num_dofs), dtype=np.float32)
q_batch[1] = [1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0]
view.set_dof_positions(q_batch)

# 一部ロボット・一部軸だけを更新する（Articulation と同じ indices / dof_indices 引数）
view.set_dof_positions([[0.5]], indices=[0], dof_indices=[2])

# 共有環境障害物（world=-1）を同期し、全 32 台の干渉とクリアランスを 1 回で判定
view.sync_environment()
hits_per_robot = view.check_collisions_batch("all")
clearances_per_robot = view.compute_clearances_batch("all", max_distance=0.05)

for i, hits in enumerate(hits_per_robot):
    if hits:
        print(f"robot[{i}]: {len(hits)} pairs, min_sep={min(p.separation for p in hits):.4f} m")

view.close()
```

---

## レシピ 5：パラメータ（DH / 修正 DH / スクリュー軸 / URDF）からのロボット構築と `attach()` 結合

既存の USD Articulation がないロボットを数式パラメータや URDF から構築し、`attach()` で台車＋双腕などの複合ロボットへ結合します。

```python
import numpy as np
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

# 1. 修正 DH パラメータから 6 軸モデルを作成
pi = np.pi
mdh = MDHParameters(
    rows=[
        MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.40, lower_limit=-pi, upper_limit=pi),
        MDHRow(alpha=-pi / 2, a=0.05, theta=-pi / 2, d=0.0, lower_limit=-pi, upper_limit=pi),
        MDHRow(alpha=0.0, a=0.45, theta=0.0, d=0.0, lower_limit=-pi, upper_limit=pi),
        MDHRow(alpha=-pi / 2, a=0.04, theta=0.0, d=0.42, lower_limit=-pi, upper_limit=pi),
        MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, lower_limit=-pi, upper_limit=pi),
        MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.10, lower_limit=-pi, upper_limit=pi),
    ],
    tool=tf.translation(z=0.08),
    name="arm6",
)
arm_model = mdh.to_model()

# 2. その他の生成元（標準 DH / スクリュー軸 / URDF）
dh_model = DHParameters(rows=[DHRow(theta=0.0, d=0.3, a=0.25, alpha=0.0)]).to_model()
screw_model = ScrewParameters(
    rows=[ScrewAxisRow.through(point=(0.0, 0.0, 0.3), direction=(0.0, 1.0, 0.0))],
    tool_home=tf.translation(x=0.3, z=0.3),
).to_model()
urdf_model = UrdfSource(r"C:\path\to\robot.urdf").to_model()

# 3. 台車に左右 2 本のアームを結合し、簡易形状 prim ごとステージへ生成
cart = GeometricRobotModel("cart", (GeometricLinkSpec("body"),), ())
dual_model = (
    cart.attach(arm_model, "body", tf.translation(y=0.25), name="left")
        .attach(arm_model, "body", tf.translation(y=-0.25), name="right")
)

style = GeometricVisualStyle(link_radius=0.03, collisions=True)
dual_robot = GeometricRobot.from_model(stage, dual_model, "/World/DualArm", style=style)

# 簡易形状で初期姿勢から常に重なる非隣接組をフィルタへ登録
dual_robot.ignore_touching_pairs()

# 構成要素ごとの操作と干渉チェック
left = dual_robot.get_component("left")
right = dual_robot.get_component("right")
left.set_dof_positions([0.3, 0.2, 0.4, 0.0, 0.2, 0.0])
print("左アーム内:", left.check_collisions("within"))
print("左右間:", dual_robot.scene.check_between(left.link_set, right.link_set))
```

---

## レシピ 6：IK（`jacobian` / `ikpy` / 自作ソルバー登録）とアクティブ軸マスク

`GeometricRobot.create_ik_solver()` で手先目標を解き、使う軸のマスクや姿勢制約を動的に切り替えます。

```python
import numpy as np
from myext.robot.geometric_newton import (
    IKResult,
    IKSolver,
    IKSolverConfig,
    register_ik_solver,
)

# 1. 位置のみ追従（orientation_mode="none"）で JacobianIKSolver を作成
cfg = IKSolverConfig(orientation_mode="none", num_restarts=4, damping=0.01)
solver = robot.create_ik_solver("jacobian", base_link="base_link", tip_link="wrist_3_link", config=cfg)

# ワールド座標で目標を渡し、成功時にそのまま関節値へ反映
res = robot.solve_ik(solver, target_position=[0.55, 0.20, 0.45], apply=True)
print(res.success, f"pos_err={res.position_error:.2e} m", f"iters={res.iterations}")

# 2. 特定の軸だけを動かす（名前リストまたは bool マスク）
solver.set_active_dofs(["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"])
res_3dof = robot.solve_ik(solver, target_position=[0.50, 0.25, 0.50])

# 3. 位置 + 姿勢（(w, x, y, z) 四元数）の 6-DOF IK へ切り替え
solver.set_active_dofs([True] * solver.chain.num_variables)
solver.set_config(orientation_mode="full", orientation_tolerance=1e-3)
res_6dof = robot.solve_ik(solver, target_position=[0.50, 0.25, 0.50], target_orientation=[1.0, 0.0, 0.0, 0.0])

# 4. 自作 IK ソルバーを登録して名前で呼び出す
class ZeroIKSolver(IKSolver):
    def solve(self, target_position, target_orientation=None, *, seed=None) -> IKResult:
        q = np.zeros(self.chain.num_variables, dtype=float)
        return IKResult(dof_positions=q, success=True, position_error=0.0, orientation_error=0.0)

register_ik_solver("my_zero", ZeroIKSolver)
custom = robot.create_ik_solver("my_zero", "base_link", "wrist_3_link")
```

---

## レシピ 7：VariantSet 切り替え・角度単位（度）・`CommandConverter` による外部指令変換

グリッパ等の `VariantSet` を切り替えつつ、度単位での指令とログ形式のコマンド変換を適用します。`VariantSet` 切り替え時は `GeometricScene` も自動的に再構築されます。

```python
from myext.robot.geometric_newton import (
    GeometricRobot,
    LinearCommandConverter,
    UsdArticulationSource,
)

source = UsdArticulationSource(
    "/World/UR10e",
    components={"arm": "/World/UR10e", "gripper": "/World/UR10e/Robotiq_2F_85"},
)
robot = GeometricRobot.from_articulation(stage, source, angle_unit="deg")

# 1. 既定単位（deg）と呼び出し単位（rad）の混用（直動軸は常にステージの長さ単位）
robot.set_dof_positions([90.0, -60.0], dof_indices=[0, 1])
robot.set_dof_positions([0.5], dof_indices=[2], unit="rad")
print("deg:", robot.get_dof_positions(unit="deg"))

# 2. VariantSet の確認と切り替え（選択は Session Layer に書かれる）
print("選択肢:", robot.get_variant_choices())
robot.set_variant("Gripper", "Robotiq_2F_140")   # DOF 数・GeometricLayer・GeometricScene が自動更新

# 構造変更通知を受け取って IK ソルバーを作り直す
def on_rebuilt(old_dofs: list[str], new_dofs: list[str]) -> None:
    print(f"構造更新: {len(old_dofs)} -> {len(new_dofs)} DOFs")

robot.add_structure_listener(on_rebuilt)

# 3. 外部コマンド名 -> 複数 DOF の変換を Variant 条件付きで登録
grip_conv = LinearCommandConverter(
    input_names=["grip_width"],
    output_dofs=["finger_joint"],
    matrix=[[12.0]],
    offset=[0.0],
    output_angle_unit="deg",
)
robot.set_command_converter("gripper_140", grip_conv, when={("", "Gripper"): "Robotiq_2F_140"})
robot.apply_command({"grip_width": 0.03, "shoulder_pan_joint": 45.0})
print("逆変換:", robot.read_command())
```

---

## レシピ 8：既存 `myext.robot.geometric` からの移行手順と PhysX バックエンドへの切替

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
