"""幾何モードのロボット API (Newton 統合版) のデモ(これだけで Isaac Sim を起動して動く)。

実行:
    C:\\isaacsim-6.2.0\\python.bat demo\\demo_geometric_robot.py --no-ros-env
    (または try_geometric_robot_newton.bat をダブルクリック)

画面に出るもの:
    左  : 修正 DH パラメータから組んだ 6 軸アーム。黄色の球を IK で追いかける。
          通り道の箱に当たっている間、当たったリンクが赤くなる(Newton GeometricScene による
          USD フラッシュなし・0 フレーム遅延の干渉チェック＋めり込み量表示)
    右  : 既存のロボット(Articulation を持つ USD)を幾何モードにしたもの。関節値を直接書いて動かす。
          グリッパの指は 1 軸だけを書き、残りは mimic で従う
    約 10 秒ごとに、右のロボットの Sublayer を外して Play する(元の Articulation に戻り、物理で動く) →
    Stop して Sublayer を付け直す(また幾何モードで動く)、を繰り返す。

オプション:
    --headless            ウィンドウを出さない
    --frames N            N フレームで終了する(省略時はウィンドウを閉じるまで)
    --screenshot PATH     終了の直前にビューポートを PNG に保存する
    --no-existing-robot   右の既存ロボットを出さない(同梱のテスト用アセットが無い環境用)
"""

from __future__ import annotations

import argparse
import os
import time

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--frames", type=int, default=0)
parser.add_argument("--screenshot", default="")
parser.add_argument("--screenshot-frames", default="", help="この番号のフレームでも保存する(カンマ区切り。確認用)")
parser.add_argument("--no-existing-robot", action="store_true")
args, _ = parser.parse_known_args()

EXT_FOLDER = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # この拡張機能が入っているフォルダ

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "extra_args": ["--ext-folder", EXT_FOLDER]})

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("myext.robot.geometric_newton", True)

from myext.robot.geometric_newton import GeometricRobot, IKSolverConfig, MDHParameters, MDHRow, UsdArticulationSource
from myext.robot.geometric_newton import transforms as tf

ISAAC_DIR = os.environ.get("ISAAC_PATH") or os.path.dirname(os.path.dirname(EXT_FOLDER))  # python.bat が ISAAC_PATH を設定する
EXISTING_ROBOT_USD = os.path.join(ISAAC_DIR, "exts", "isaacsim.asset.transformer.rules", "data", "tests", "ur10e", "ur10e.usd")
RED = Gf.Vec3f(0.95, 0.15, 0.10)
GRAY = Gf.Vec3f(0.75, 0.78, 0.82)


def log(message: str) -> None:
    print(f"[demo] {message}", flush=True)


# ---------------------------------------------------------------------- シーン


def build_scene():
    """空のステージに、物理シーン・照明・カメラ・箱(障害物)・目標の球を置く。"""
    context = omni.usd.get_context()
    context.new_stage()
    app.update()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 20.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.7, 0.7, -0.01))
    floor.AddScaleOp().Set(Gf.Vec3d(5.0, 5.0, 0.02))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.34, 0.38)])

    box = UsdGeom.Cube.Define(stage, "/World/Obstacle")  # 静的コライダー(剛体は付けない)
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.33, 0.62))
    box.AddScaleOp().Set(Gf.Vec3d(0.16, 0.16, 0.5))
    box.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.45, 0.85)])
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    target = UsdGeom.Sphere.Define(stage, "/World/Target")
    target.CreateRadiusAttr(0.035)
    target.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.6))
    target.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.85, 0.1)])

    camera = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    eye, look_at = Gf.Vec3d(4.3, -2.9, 2.6), Gf.Vec3d(0.75, 0.75, 0.55)
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, look_at, Gf.Vec3d(0, 0, 1)).GetInverse())
    camera.CreateFocalLengthAttr(24.0)
    return stage


def use_demo_camera() -> None:
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = "/World/DemoCamera"
    except Exception as exc:
        log(f"カメラを切り替えられなかった: {exc}")


def six_axis_arm() -> MDHParameters:
    """肩・肘・手首を持つ 6 軸アームの修正 DH パラメータ(長さはメートル)。"""
    pi = np.pi
    limits = dict(lower_limit=-pi, upper_limit=pi)
    return MDHParameters(
        rows=[
            MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.40, **limits),
            MDHRow(alpha=-pi / 2, a=0.05, theta=-pi / 2, d=0.0, **limits),
            MDHRow(alpha=0.0, a=0.45, theta=0.0, d=0.0, **limits),
            MDHRow(alpha=-pi / 2, a=0.04, theta=0.0, d=0.42, **limits),
            MDHRow(alpha=pi / 2, a=0.0, theta=0.0, d=0.0, **limits),
            MDHRow(alpha=-pi / 2, a=0.0, theta=0.0, d=0.10, **limits),
        ],
        tool=tf.translation(z=0.08),
        name="six_axis",
    )


# ---------------------------------------------------------------------- 左: パラメータから組んだアーム


class TrackingArm:
    """修正 DH から組んだアーム。目標の球を IK で追い、箱との干渉を色で見せる。"""

    def __init__(self, stage) -> None:
        self.stage = stage
        # from_mdh: 簡易形状の prim を組み、Sublayer を付けて幾何モードにする
        self.robot = GeometricRobot.from_mdh(stage, six_axis_arm(), "/World/Arm")
        # 簡易形状の現在の基準姿勢で重なっている非隣接組を無視フィルタへ登録する
        ignored = self.robot.ignore_touching_pairs()
        log(f"左のアーム: DOF {self.robot.dof_names} / 初期姿勢で重なっているため無視する組 {ignored}")
        # IK ソルバーは名前で選ぶ("jacobian" = 自前、"ikpy" = IKPy)。位置だけ合わせる
        self.solver = self.robot.create_ik_solver("jacobian", "link0", "tool", IKSolverConfig(orientation_mode="none", num_restarts=2))
        self.robot.set_dof_positions([0.0, 0.4, 0.6, 0.0, 0.5, 0.0])
        self.target_op = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Target")).GetOrderedXformOps()[0]
        self.red_links: set[str] = set()
        self.times = {"ik": [], "write": [], "collision": []}

    def step(self, t: float) -> None:
        target = np.array([0.52 + 0.10 * np.cos(0.7 * t), 0.42 * np.sin(0.9 * t), 0.55 + 0.22 * np.sin(0.5 * t)])
        self.target_op.Set(Gf.Vec3d(*target.tolist()))
        t0 = time.perf_counter()
        result = self.robot.solve_ik(self.solver, target, apply=False)  # ワールド座標の目標。seed はいまの関節値
        t1 = time.perf_counter()
        if result.success:
            self.robot.set_dof_positions(result.dof_positions, dof_indices=self.solver.chain.dof_indices)
        t2 = time.perf_counter()
        pairs = self.robot.check_collisions("all")  # Newton GeometricScene: Stop / Play=ON ともに 0 フレーム遅延
        t3 = time.perf_counter()
        self.times["ik"].append(t1 - t0)
        self.times["write"].append(t2 - t1)
        self.times["collision"].append(t3 - t2)
        depths = [p.separation * 1e3 for p in pairs if p.separation is not None]
        self._paint({p.link_a for p in pairs} | {p.link_b for p in pairs if p.link_b}, min(depths) if depths else None)

    def _paint(self, hit_links: set[str], min_sep_mm: float | None = None) -> None:
        if hit_links == self.red_links:
            return
        # 色も Sublayer に書く(幾何モードを外すと元の色に戻る)
        with self.robot.layer.edit_context():
            for link in hit_links ^ self.red_links:
                color = RED if link in hit_links else GRAY
                visuals = self.stage.GetPrimAtPath(self.robot.link_path_map[link] + "/visuals")
                for prim in visuals.GetChildren():
                    if prim.GetName().startswith("bone"):
                        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([color])
        if hit_links:
            depth_info = f" (最大めり込み {min_sep_mm:.1f} mm)" if min_sep_mm is not None else ""
            log(f"左のアーム: 干渉 {sorted(hit_links)}{depth_info}")
        self.red_links = set(hit_links)

    def report(self) -> str:
        out = "  ".join(f"{k} {1e3 * float(np.median(v)):.2f} ms" for k, v in self.times.items() if v)
        for v in self.times.values():
            v.clear()
        return out


# ---------------------------------------------------------------------- 右: 既存の Articulation


class ExistingRobot:
    """既存のロボット(Articulation を持つ USD)を幾何モードで動かし、ときどき元の構成に戻して物理で動かす。"""

    PATH = "/World/Robot"

    def __init__(self, stage) -> None:
        self.stage = stage
        prim = stage.DefinePrim(self.PATH, "Xform")
        prim.GetReferences().AddReference(EXISTING_ROBOT_USD)
        prim.GetVariantSet("Gripper").SetVariantSelection("Robotiq_2F_85")
        UsdGeom.Xformable(prim).ClearXformOpOrder()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(1.4, 1.5, 0.0))
        app.update()
        # from_articulation: Joint prim からつながり・軸・可動範囲・mimic を読み、Sublayer を付けて幾何モードにする。
        # 元の USD は書き換えない。components で「アーム」と「グリッパ」を分けておく
        source = UsdArticulationSource(self.PATH, components={"arm": self.PATH, "gripper": f"{self.PATH}/Robotiq_2F_85"})
        self.robot = GeometricRobot.from_articulation(stage, source)
        model = self.robot.model
        followers = [j.name for j in model.dof_joints if j.mimic is not None]
        log(f"右のロボット: DOF {self.robot.num_dofs} 個 / mimic の従動軸 {followers}")
        self.arm = self.robot.get_component("arm")
        self.finger = self.robot.joints["finger_joint"]
        self.geometric = True

    def step(self, t: float) -> None:
        if not self.geometric:
            return
        q = np.array([0.8 * np.sin(0.5 * t), -1.7 + 0.3 * np.sin(0.7 * t), 1.3 + 0.5 * np.sin(0.9 * t), -1.2 + 0.4 * np.sin(0.6 * t), 0.8 * np.sin(0.8 * t), t * 0.5 % (2 * np.pi) - np.pi])
        self.arm.set_dof_positions(q)  # アームの 6 軸
        self.finger.set_position(0.35 + 0.35 * np.sin(1.5 * t))  # 指は 1 軸だけ書く。残りは mimic で従う

    def to_physics(self) -> None:
        """Sublayer を外して Play する。元の Articulation に戻り、位置ドライブと重力で動く。"""
        self.robot.layer.remove()
        self.geometric = False
        omni.timeline.get_timeline_interface().play()
        log("右のロボット: Sublayer を外して Play した(元の Articulation。物理で動く)")

    def to_geometric(self) -> None:
        """Stop して Sublayer を付け直す。"""
        omni.timeline.get_timeline_interface().stop()
        app.update()
        self.robot.layer.apply()
        self.geometric = True
        log("右のロボット: Stop して Sublayer を付けた(幾何モード。関節値を直接書く)")


# ---------------------------------------------------------------------- 実行


def save_screenshot(path: str) -> None:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    capture_viewport_to_file(get_active_viewport(), os.path.abspath(path))
    for _ in range(8):
        app.update()
    log(f"画像を保存した: {os.path.abspath(path)}")


def main() -> None:
    stage = build_scene()
    arm = TrackingArm(stage)
    existing = None
    if args.no_existing_robot:
        log("右の既存ロボットは出さない(--no-existing-robot)")
    elif not os.path.exists(EXISTING_ROBOT_USD):
        log(f"右の既存ロボットのアセットが無いので出さない: {EXISTING_ROBOT_USD}")
    else:
        existing = ExistingRobot(stage)
    for _ in range(10):
        app.update()
    use_demo_camera()
    shot_frames = {int(v) for v in args.screenshot_frames.split(",") if v.strip()}

    cycle, physics_from = 600, 420  # 600 フレームのうち、420 フレーム目から物理(元の Articulation)
    frame = 0
    log("開始。ウィンドウを閉じると終わる" if not args.frames else f"開始。{args.frames} フレームで終わる")
    while app.is_running() and (not args.frames or frame < args.frames):
        t = frame / 60.0
        arm.step(t)
        if existing is not None:
            phase = frame % cycle
            if phase == physics_from:
                existing.to_physics()
            elif phase == 0 and frame > 0:
                existing.to_geometric()
            existing.step(t)
        app.update()
        frame += 1
        if frame in shot_frames and args.screenshot:
            base, ext = os.path.splitext(args.screenshot)
            save_screenshot(f"{base}_{frame:04d}{ext}")
        if frame % 120 == 0:
            log(f"frame {frame}: 左のアームの処理時間(中央値) {arm.report()}")
    if args.screenshot:
        save_screenshot(args.screenshot)
    log("終了")


main()
app.close()
