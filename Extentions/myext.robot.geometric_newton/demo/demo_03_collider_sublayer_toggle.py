"""Demo 03: カスタムコライダー用 `.usda` Sublayer の実行時 ON/OFF 切替（非破壊・0 フレーム遅延）。

実行方法（どちらも事前準備なし・コピペまたはそのまま実行）:
  1. 起動中の Isaac Sim 6.2.0 の Script Editor にこのファイル全体を貼って実行
     （開いているステージは新規ステージに置き換わります）
  2. コマンドラインから単独起動:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_03_collider_sublayer_toggle.py
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat demo\\demo_03_collider_sublayer_toggle.py --headless --frames 180

見どころ:
  - 元の UR10e USD ファイルを一切書き換えずに、カスタムコライダー `.usda` を実行時に一時生成
  - `robot.attach_collider_layer(usda_path)` で `GeometricLayer` 直下（Session Layer）へ差し込み
  - 60 フレームごとに `robot.set_collider_layer_enabled(layer_id, enabled)` で Mute / Unmute を切替
  - カスタム安全球コライダー（半径 0.22 m）が ON の瞬間に即座に干渉検出、OFF の瞬間に安全へ復帰（0 フレーム遅延）
  - 終了時に `robot.detach_collider_layer()` と `robot.close()` で元の構成へ完全復帰
"""

from __future__ import annotations

import argparse
import asyncio
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

# インストールフォルダ（別パスに置いた場合はここだけ書き換えてください）
ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"
DEFAULT_SCRIPT_EDITOR_FRAMES = 300


def _is_kit_running() -> bool:
    try:
        import omni.kit.app

        return omni.kit.app.get_app() is not None
    except Exception:
        return False


def _enable_extension() -> None:
    import omni.kit.app

    ext_root = (ISAAC_ROOT / "extsUser" / "Extentions").as_posix()
    mgr = omni.kit.app.get_app().get_extension_manager()
    if not mgr.is_extension_enabled("myext.robot.geometric_newton"):
        mgr.add_path(ext_root)
        mgr.set_extension_enabled_immediate("myext.robot.geometric_newton", True)


def _setup_stage(stage):
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(850.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2100.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 25.0, 0.0))

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
    floor.AddScaleOp().Set(Gf.Vec3d(6.0, 6.0, 0.04))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.24, 0.26, 0.30)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    # 判定対象の環境障害物（ロボット初期化後に手先近傍へ配置する）
    probe = UsdGeom.Cube.Define(stage, "/World/SafetyProbe")
    probe.CreateSizeAttr(0.16)
    probe_op = probe.AddTranslateOp()
    probe_op.Set(Gf.Vec3d(0.55, 0.30, 0.70))
    probe.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.55, 0.92)])
    UsdPhysics.CollisionAPI.Apply(probe.GetPrim())

    if not UR10E_USD.exists():
        raise FileNotFoundError(f"UR10e アセットが見つかりません: {UR10E_USD}")
    robot_prim = stage.DefinePrim("/World/UR10e", "Xform")
    robot_prim.GetReferences().AddReference(UR10E_USD.as_posix())

    cam = UsdGeom.Camera.Define(stage, "/World/DemoCamera")
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.1, -1.7, 1.45), Gf.Vec3d(0.1, 0.0, 0.55), Gf.Vec3d(0, 0, 1)).GetInverse())
    cam.CreateFocalLengthAttr(24.0)
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/DemoCamera"
    except Exception:
        pass
    return probe, probe_op


def _build_custom_collider_usda(
    usda_path: Path,
    orig_colliders: dict[str, list[str]],
    link_path_map: dict[str, str],
) -> Path:
    """元 USD のコライダーを無効化し、手先リンクへ安全マージン付きの簡易球コライダーを追加する `.usda` を生成する。"""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    sub_stage = Usd.Stage.CreateNew(str(usda_path))
    UsdGeom.SetStageUpAxis(sub_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(sub_stage, 1.0)

    def _override_ancestors(prim_path: str) -> None:
        p = Sdf.Path(prim_path)
        for anc in p.GetPrefixes()[:-1]:
            if anc != Sdf.Path.absoluteRootPath:
                sub_stage.OverridePrim(anc)

    # 1. 元のコライダー prim を `over` して `physics:collisionEnabled = false` に上書き
    for cpaths in orig_colliders.values():
        for cpath in cpaths:
            _override_ancestors(cpath)
            over_prim = sub_stage.OverridePrim(cpath)
            UsdPhysics.CollisionAPI.Apply(over_prim).CreateCollisionEnabledAttr(False)

    # 2. 対象リンクへ安全マージン付きの簡易 Sphere コライダーを新規定義
    target_links = [
        name for name in ("forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
        if name in link_path_map
    ]
    if not target_links:
        target_links = list(link_path_map.keys())[-2:]

    for lname in target_links:
        lpath = link_path_map[lname]
        _override_ancestors(lpath)
        sub_stage.OverridePrim(lpath)
        bubble = UsdGeom.Sphere.Define(sub_stage, f"{lpath}/CustomSafetyBubble")
        bubble.CreateRadiusAttr(0.22 if "wrist" in lname else 0.14)
        bubble.CreateDisplayColorAttr([Gf.Vec3f(0.92, 0.30, 0.72)])
        UsdPhysics.CollisionAPI.Apply(bubble.GetPrim()).CreateCollisionEnabledAttr(True)

    sub_stage.GetRootLayer().Save()
    return usda_path


class DemoRunner:
    def __init__(self, stage, probe_prim, probe_op) -> None:
        import numpy as np
        from pxr import Gf
        from myext.robot.geometric_newton import GeometricRobot

        self.np = np
        self.probe_color = probe_prim.GetDisplayColorAttr()
        self.temp_dir = Path(tempfile.mkdtemp(prefix="geom_newton_demo03_"))
        self.robot = GeometricRobot.from_articulation(
            stage,
            "/World/UR10e",
            write_target="fabric",
            collision_backend="newton",
        )
        # 基準姿勢へ設定し、手先リンクからちょうど「元コライダーは安全・安全球は貫通」する位置へ障害物を配置
        self.q_base = np.array([0.0, -1.15, 1.35, -1.57, -1.57, 0.0])
        self.robot.set_dof_positions(self.q_base)
        self.robot.flush_to_usd()

        anchor_link = "wrist_1_link" if "wrist_1_link" in self.robot.links else self.robot.link_names[-1]
        anchor_pos, _ = self.robot.links[anchor_link].get_world_pose()
        probe_pos = anchor_pos + np.array([0.0, 0.24, 0.0])
        probe_op.Set(Gf.Vec3d(*probe_pos.tolist()))
        self.robot.scene.sync_environment()

        # 元コライダー構成を取得し、カスタムコライダー `.usda` Sublayer を動的生成
        orig_colliders = {k: list(v) for k, v in self.robot.collider_paths.items()}
        orig_count = sum(len(v) for v in orig_colliders.values())
        self.usda_path = _build_custom_collider_usda(
            self.temp_dir / "ur10e_safety_colliders.usda",
            orig_colliders,
            dict(self.robot.link_path_map),
        )

        # 初回状態（カスタム Sublayer 未適用）の確認
        base_pairs = self.robot.compute_clearances("environment", max_distance=0.25)
        base_hits = [p for p in base_pairs if p.separation is not None and p.separation <= 0.0]
        print(
            f"[demo_03] 元のコライダー構成: {orig_count} 個, "
            f"環境干渉={len(base_hits)}組, collider_version={self.robot.collider_version}",
            flush=True,
        )

        # カスタムコライダー Sublayer を Session Layer（GeometricLayer 直下）へ差し込み
        self.layer_id = self.robot.attach_collider_layer(self.usda_path, target="session")
        self.custom_enabled = True
        custom_count = sum(len(v) for v in self.robot.collider_paths.values())
        custom_pairs = self.robot.compute_clearances("environment", max_distance=0.25)
        custom_hits = [p for p in custom_pairs if p.separation is not None and p.separation <= 0.0]
        min_sep = min((p.separation for p in custom_hits if p.separation is not None), default=0.0)
        print(
            f"[demo_03] attach_collider_layer 適用直後: {custom_count} 個, "
            f"環境干渉={len(custom_hits)}組 (最小sep={min_sep * 1e3:.1f}mm), "
            f"collider_version={self.robot.collider_version}",
            flush=True,
        )

        self.toggle_interval = 60
        self.step_ms: list[float] = []

    def step(self, frame: int) -> None:
        from pxr import Gf

        np = self.np
        # 60 フレームごとにカスタムコライダー Sublayer を Mute / Unmute 切替
        toggled = False
        if frame > 0 and frame % self.toggle_interval == 0:
            self.custom_enabled = not self.custom_enabled
            self.robot.set_collider_layer_enabled(self.layer_id, self.custom_enabled)
            toggled = True

        # Play=ON 中も微小に腕を動かし、Fabric 駆動とカスタム Sublayer 切替の共存を確認
        t = frame / 60.0
        q = self.q_base.copy()
        q[0] += 0.04 * np.sin(1.2 * t)
        q[3] += 0.06 * np.cos(1.5 * t)

        t0 = time.perf_counter()
        self.robot.set_dof_positions(q)
        pairs = self.robot.compute_clearances("environment", max_distance=0.25)
        hits = [p for p in pairs if p.separation is not None and p.separation <= 0.0]
        near = [p for p in pairs if p.separation is not None and p.separation > 0.0]
        t1 = time.perf_counter()
        self.step_ms.append((t1 - t0) * 1e3)

        if hits:
            self.probe_color.Set([Gf.Vec3f(0.95, 0.18, 0.15)])
        elif near:
            self.probe_color.Set([Gf.Vec3f(0.98, 0.82, 0.15)])
        else:
            self.probe_color.Set([Gf.Vec3f(0.20, 0.55, 0.92)])

        if toggled or frame % 30 == 0:
            mode = "カスタム安全球 Sublayer=ON " if self.custom_enabled else "元コライダー (Sublayer=OFF)"
            n_col = sum(len(v) for v in self.robot.collider_paths.values())
            if hits:
                sep = min(p.separation for p in hits if p.separation is not None) * 1e3
                status = f"干渉={len(hits)}組 (最小sep={sep:.1f}mm)"
            elif near and near[0].separation is not None:
                status = f"安全 (最近接距離={near[0].separation * 1e3:.1f}mm)"
            else:
                status = "安全 (>250mm)"
            tag = " [切替直後・0フレーム遅延]" if toggled else ""
            print(
                f"[demo_03] frame={frame:4d} {mode} col={n_col:2d} ver={self.robot.collider_version} "
                f"{status} / {self.step_ms[-1]:.2f}ms{tag}",
                flush=True,
            )

    def close(self) -> None:
        removed = self.robot.detach_collider_layer(self.layer_id)
        restored_count = sum(len(v) for v in self.robot.collider_paths.values())
        med = float(self.np.median(self.step_ms)) if self.step_ms else 0.0
        print(
            f"[demo_03] 終了 (detach={removed}, 復帰後コライダー数={restored_count}, "
            f"collider_layers={self.robot.collider_layers}, 中央値={med:.2f}ms/frame)",
            flush=True,
        )
        self.robot.close()
        shutil.rmtree(self.temp_dir, ignore_errors=True)


async def _run_script_editor(frames: int) -> None:
    import omni.kit.app
    import omni.timeline
    import omni.usd

    _enable_extension()
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    stage = ctx.get_stage()
    probe, probe_op = _setup_stage(stage)
    await app.next_update_async()
    await app.next_update_async()

    runner = DemoRunner(stage, probe, probe_op)
    await app.next_update_async()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for frame in range(frames):
        runner.step(frame)
        await app.next_update_async()
    tl.stop()
    await app.next_update_async()
    runner.close()


def _run_standalone() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="0 ならウィンドウを閉じるまで実行")
    args, _ = ap.parse_known_args()

    sys.path.insert(0, str(ISAAC_ROOT / "tools" / "myext"))
    from sim_app_launcher import create_simulation_app

    sim_app = create_simulation_app(
        {"headless": args.headless, "width": 1280, "height": 720},
        extra_extensions=("myext.robot.geometric_newton",),
    )

    import omni.timeline
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    sim_app.update()
    stage = ctx.get_stage()
    probe, probe_op = _setup_stage(stage)
    for _ in range(3):
        sim_app.update()

    runner = DemoRunner(stage, probe, probe_op)
    sim_app.update()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    frame = 0
    while sim_app.is_running() and (not args.frames or frame < args.frames):
        runner.step(frame)
        sim_app.update()
        frame += 1
    tl.stop()
    sim_app.update()
    runner.close()
    sim_app.close()


if _is_kit_running():
    task = asyncio.ensure_future(_run_script_editor(DEFAULT_SCRIPT_EDITOR_FRAMES))
else:
    _run_standalone()
