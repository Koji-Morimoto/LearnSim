"""Isaac Sim 6.2.0 バックエンド原理・干渉遅延の 1 コマ確認スクリプト。

このスクリプトは、以下の 4 つの現象を 1 コマずつ一時停止しながら自分の目と数値で確認するための
ハンズオン検証ツールです。

  実験 1: 姿勢置き場（USD Stage / Fabric）のズレを目撃する
          - Play=ON で重力落下する DynamicCube は Fabric だけ更新され USD Stage は初期値のまま
          - write_target="fabric" のロボットは set_dof_positions() 直後に Fabric だけ更新され USD Stage は古いまま
  実験 2: PhysX overlap_shape の 1 フレーム遅れ（2/5 -> 5/5）と
          PhysX ContactReport（kinematic）の 2 update 遅れ（0 -> 0 -> ヒット）を 1 コマずつ目撃する
  実験 3: Newton バックエンドの 0 フレーム遅延（Frame 0 で 5/5＋めり込み深さ）と
          sync_environment(use_fabric="auto") が静的障害物の USD 移動を即拾う仕組みを目撃する
  実験 4: Warp GPU カーネル（wp.launch）の非同期実行と wp.synchronize_device() の違いを実測する

実行方法（AGENTS.md の SimulationApp 起動ルール準拠）:
  GUI + Enter キーで 1 コマ送り（既定）:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat verify_backend_lag_step_by_step.py
  特定の実験だけ実行（例: 実験 2 のみ）:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat verify_backend_lag_step_by_step.py --exp 2
  画面なし・自動進行で全実験を一括確認:
    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat verify_backend_lag_step_by_step.py --headless --auto
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# SimulationApp 起動ルール（AGENTS.md）: 必ず tools/myext/sim_app_launcher.py を使用する
ISAAC_ROOT = Path(r"C:/isaacsim-6.2.0")
sys.path.insert(0, str(ISAAC_ROOT / "tools" / "myext"))
from sim_app_launcher import create_simulation_app  # noqa: E402

UR10E_USD = str(ISAAC_ROOT / "exts" / "isaacsim.asset.transformer.rules" / "data" / "tests" / "ur10e" / "ur10e.usd")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Isaac Sim バックエンド原理と干渉遅延の 1 コマ確認スクリプト")
    parser.add_argument("--headless", action="store_true", help="GUI を開かずに実行する")
    parser.add_argument("--auto", action="store_true", help="Enter 待ちをせず自動で全ステップを進める")
    parser.add_argument(
        "--exp",
        choices=("all", "1", "2", "3", "4"),
        default="all",
        help="実行する実験番号（既定: all）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = create_simulation_app(
        {"headless": args.headless, "width": 1280, "height": 720},
        extra_extensions=(
            "omni.physx.fabric",
            "omni.warp.core",
            "isaacsim.pip.newton",
            "myext.robot.geometric_newton",
        ),
    )

    import numpy as np
    import omni.timeline
    import omni.usd
    import usdrt
    import warp as wp
    from omni.physx import get_physx_scene_query_interface
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils

    from myext.robot.geometric_newton import (
        GeometricContactMonitor,
        GeometricLayerOptions,
        GeometricRobot,
    )
    from myext.robot.geometric_newton.usd_reader import _np, split_scale
    from myext.robot.geometric_newton.writer import GeometricFabricWriter

    def step_app(n: int = 1) -> None:
        for _ in range(n):
            app.update()

    def pause(label: str) -> None:
        if args.auto:
            print(f"  [AUTO] {label}", flush=True)
            return
        print(f"\n  >>> {label}", flush=True)
        try:
            input("      [Enter] で次のコマへ進む (Ctrl+C で終了)... ")
        except EOFError:
            pass

    def banner(title: str) -> None:
        bar = "=" * 76
        print(f"\n{bar}\n  {title}\n{bar}", flush=True)

    def usd_world_pos(stage: Usd.Stage, prim_path: str) -> np.ndarray:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        prim = stage.GetPrimAtPath(prim_path)
        return split_scale(_np(cache.GetLocalToWorldTransform(prim)))[0][:3, 3]

    def fabric_world_pos(stage: Usd.Stage, prim_path: str) -> np.ndarray:
        reader = GeometricFabricWriter(stage, [prim_path], batch=False)
        return split_scale(reader.read_world()[0])[0][:3, 3]

    def set_camera_view(eye: tuple[float, float, float], target: tuple[float, float, float]) -> None:
        if args.headless:
            return
        try:
            from isaacsim.core.utils.viewports import set_camera_view as _set_cam

            _set_cam(eye=np.array(eye), target=np.array(target))
        except Exception:
            pass

    def new_demo_stage(gravity: float = 9.81) -> Usd.Stage:
        tl = omni.timeline.get_timeline_interface()
        if not tl.is_stopped():
            tl.stop()
            step_app(2)
        ctx = omni.usd.get_context()
        ctx.new_stage()
        step_app(2)
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")

        # 照明（GUI で見やすくするため）
        dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome.CreateIntensityAttr(1200.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
        sun.CreateIntensityAttr(2000.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 25.0, 30.0))

        # PhysicsScene（PhysX CPU / MBP）
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(gravity)
        ps_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        ps_api.CreateEnableGPUDynamicsAttr(False)
        ps_api.CreateBroadphaseTypeAttr("MBP")

        step_app(2)
        return stage

    def add_ur10e(stage: Usd.Stage, path: str = "/World/UR10e") -> Usd.Prim:
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(UR10E_USD)
        step_app(3)
        return prim

    def add_box(
        stage: Usd.Stage,
        path: str,
        position: tuple[float, float, float],
        size: float = 0.25,
        kind: str = "static",
        color: tuple[float, float, float] = (0.9, 0.4, 0.1),
    ) -> Usd.Prim:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(size)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if kind != "static":
            body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            if kind == "kinematic":
                body.CreateKinematicEnabledAttr(True)
            else:
                UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
        return cube.GetPrim()

    def raw_physx_overlap_within(robot: GeometricRobot) -> list[tuple[str, str]]:
        """flush_to_usd() を呼ばずに PhysX overlap_shape を直接実行し、自己干渉ペアを返す。"""
        query = get_physx_scene_query_interface()
        path_to_link = {path: name for name, path in robot.link_path_map.items()}
        flt = robot.collision_filter

        def _owner(hit_path: str) -> str | None:
            p = hit_path
            while p and p != "/":
                if p in path_to_link:
                    return path_to_link[p]
                p = p.rsplit("/", 1)[0]
            return None

        found: set[tuple[str, str]] = set()
        for link_name, colliders in robot.collider_paths.items():
            for col_path in colliders:
                hits: list[str] = []

                def _cb(hit) -> bool:
                    hits.append(str(hit.collision))
                    return True

                id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(col_path))
                query.overlap_shape(id0, id1, _cb, False)
                for hp in hits:
                    other = _owner(hp)
                    if other is None or other == link_name or flt.is_ignored(link_name, other):
                        continue
                    found.add(tuple(sorted((link_name, other))))
        return sorted(found)

    q_safe = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0], dtype=float)
    q_self = np.array([1.57, -0.5, 2.95, -np.pi / 2, -np.pi / 2, 0.0], dtype=float)
    q_hit = np.array([0.35, -0.35, 0.65, -0.3, 0.5, 0.0], dtype=float)

    summary: dict[str, object] = {}

    # =========================================================================
    # 実験 1: 姿勢置き場（USD Stage vs Fabric）のズレを目撃する
    # =========================================================================
    if args.exp in ("all", "1"):
        banner("実験 1: USD Stage と Fabric のズレを目撃する（Source of Truth は 1 つではない）")
        stage = new_demo_stage(gravity=9.81)
        add_ur10e(stage)
        add_box(stage, "/World/DynamicCube", position=(-0.9, 0.0, 1.5), size=0.22, kind="dynamic", color=(0.2, 0.6, 0.95))
        robot = GeometricRobot.from_articulation(stage, "/World/UR10e", write_target="fabric", collision_backend="newton")
        robot.set_dof_positions(q_safe)
        robot.flush_to_usd()
        step_app(2)
        set_camera_view((2.4, -2.2, 1.5), (-0.2, 0.0, 0.6))

        tl = omni.timeline.get_timeline_interface()
        tl.play()
        step_app(3)  # Timeline の Frame Integrity と PhysX 初期ロード完了待ち

        print("\n[1-A] Play=ON 中の動的剛体 (/World/DynamicCube) の重力落下:", flush=True)
        print(f"  {'コマ':<16} | {'USD Stage Z [m]':>16} | {'Fabric Z [m]':>14} | {'差 (USD - Fabric)':>18}", flush=True)
        print("  " + "-" * 72, flush=True)
        dyn_rows = []
        for tag, n_steps in (("Play 直後", 0), ("+1 update", 1), ("+5 update", 5), ("+15 update", 10)):
            if n_steps > 0:
                step_app(n_steps)
            uz = float(usd_world_pos(stage, "/World/DynamicCube")[2])
            fz = float(fabric_world_pos(stage, "/World/DynamicCube")[2])
            dyn_rows.append((tag, uz, fz))
            print(f"  {tag:<16} | {uz:16.4f} | {fz:14.4f} | {uz - fz:18.4f}", flush=True)
        assert abs(dyn_rows[0][1] - dyn_rows[-1][1]) < 1e-6, "Play 中に DynamicCube の USD Z が変化した"
        assert dyn_rows[0][2] - dyn_rows[-1][2] > 0.05, "DynamicCube が Fabric 上で落下していない"
        pause("実験 1-A 確認: Viewport では青い箱が落下し Fabric Z は減少しますが、USD Stage Z は 1.5000 m のまま止まっています。")

        print("\n[1-B] write_target='fabric' で set_dof_positions(q_hit) した直後（app.update() 前）:", flush=True)
        w1_path = robot.link_path_map["wrist_1_link"]
        robot.set_dof_positions(q_hit)
        u_pos = usd_world_pos(stage, w1_path)
        f_pos = fabric_world_pos(stage, w1_path)
        diff_m = float(np.linalg.norm(u_pos - f_pos))
        print(f"  USD Stage wrist_1_link : {np.round(u_pos, 4).tolist()}", flush=True)
        print(f"  Fabric    wrist_1_link : {np.round(f_pos, 4).tolist()}", flush=True)
        print(f"  距離の差               : {diff_m:.4f} m  (Fabric だけ更新され、USD Stage は旧姿勢のまま)", flush=True)
        assert diff_m > 0.5, f"USD と Fabric のズレが想定より小さい: {diff_m}"
        step_app(1)  # GUI 表示も更新して確認
        pause("実験 1-B 確認: 1 回 app.update() して Viewport が新姿勢へ動いても、USD Stage の値はまだ旧姿勢のままです。")

        tl.stop()
        step_app(2)
        robot.close()
        summary["exp1"] = {"dyn_usd_z_end": round(dyn_rows[-1][1], 4), "dyn_fabric_z_end": round(dyn_rows[-1][2], 4), "robot_usd_vs_fabric_m": round(diff_m, 4)}

    # =========================================================================
    # 実験 2: PhysX overlap_shape (1F 遅れ) & ContactReport (2 update 遅れ)
    # =========================================================================
    if args.exp in ("all", "2"):
        banner("実験 2: PhysX overlap_shape (1F 遅れ) と ContactReport (2 update 遅れ) を 1 コマずつ目撃する")
        stage = new_demo_stage(gravity=9.81)
        add_ur10e(stage)
        robot = GeometricRobot.from_articulation(stage, "/World/UR10e", write_target="fabric", collision_backend="newton")
        robot.set_dof_positions(q_safe)
        robot.flush_to_usd()
        step_app(2)
        set_camera_view((2.0, -2.0, 1.3), (0.0, 0.0, 0.5))

        tl = omni.timeline.get_timeline_interface()
        tl.play()
        step_app(3)

        print("\n[2-A] Play=ON × Fabric のみ更新で q_safe -> q_self へジャンプ（自己干渉 5 組の姿勢）:", flush=True)
        robot.set_dof_positions(q_self)  # Fabric のみ書込（USD 未更新、app.update() まだ 0 回）
        f0_physx = raw_physx_overlap_within(robot)
        f0_newton = sorted({tuple(sorted((p.link_a, p.link_b))) for p in robot.check_collisions("within")})
        print(f"  Frame 0 (app.update() 前) PhysX overlap_shape : {len(f0_physx)}/5 組 -> {f0_physx}", flush=True)
        print(f"  Frame 0 (app.update() 前) Newton check        : {len(f0_newton)}/5 組 -> {f0_newton}", flush=True)
        pause("実験 2-A Frame 0 確認: 動いていない base_link / shoulder_link 相手の 2 組だけ当たり、両方動いた 3 組が抜けています。")

        step_app(1)  # Frame +1: Sync In で Fabric 姿勢が PhysX BVH へ取り込まれる
        f1_physx = raw_physx_overlap_within(robot)
        print(f"  Frame +1 (app.update() 1回後) PhysX overlap_shape : {len(f1_physx)}/5 組 -> {f1_physx}", flush=True)
        assert len(f0_physx) == 2, f"Frame 0 PhysX 想定 2 組だが {len(f0_physx)} 組: {f0_physx}"
        assert len(f1_physx) == 5, f"Frame +1 PhysX 想定 5 組だが {len(f1_physx)} 組: {f1_physx}"
        assert len(f0_newton) == 5, f"Frame 0 Newton 想定 5 組だが {len(f0_newton)} 組: {f0_newton}"
        pause("実験 2-A Frame +1 確認: app.update() を 1 回呼んで BVH が追いついた瞬間、5/5 組すべて検出されました。")

        # 定常遅延の確認（初回だけでなく、戻して再ジャンプしても毎回 1 フレーム遅れる）
        robot.set_dof_positions(q_safe)
        step_app(1)
        robot.set_dof_positions(q_self)
        repeat_f0 = raw_physx_overlap_within(robot)
        step_app(1)
        repeat_f1 = raw_physx_overlap_within(robot)
        print(f"  [再ジャンプ確認] 2 回目も Frame 0 = {len(repeat_f0)}/5 組 -> Frame +1 = {len(repeat_f1)}/5 組（初回だけでなく毎回 1F 遅れ）", flush=True)
        assert len(repeat_f0) == 2 and len(repeat_f1) == 5

        tl.stop()
        step_app(2)
        robot.close()

        # [2-B] Kinematic モード + ContactReport の 2 update 遅れ
        print("\n[2-B] Play=ON × collision_mode='kinematic' の ContactReport（2 update 遅れ）:", flush=True)
        stage = new_demo_stage(gravity=9.81)
        add_ur10e(stage)
        robot_kin = GeometricRobot.from_articulation(
            stage,
            "/World/UR10e",
            write_target="usd",
            layer_options=GeometricLayerOptions(collision_mode="kinematic"),
        )
        # q_hit の wrist_1_link 位置へ静的障害物を配置
        robot_kin.set_dof_positions(q_hit)
        hit_pos = tuple(float(v) for v in robot_kin.links["wrist_1_link"].get_world_pose()[0])
        add_box(stage, "/World/Obstacle", position=hit_pos, size=0.30, kind="static", color=(0.95, 0.25, 0.25))
        robot_kin.set_dof_positions(q_safe)
        step_app(2)
        set_camera_view((2.2, -2.0, 1.3), (0.3, 0.2, 0.4))

        tl = omni.timeline.get_timeline_interface()
        tl.play()
        step_app(3)
        monitor = GeometricContactMonitor(robot_kin)

        robot_kin.set_dof_positions(q_hit)
        c_f0 = sorted({p.link_a for p in monitor.penetrating() if p.kind == "environment"})
        print(f"  Frame 0 (app.update() 0 回) ContactReport penetrating : {c_f0}", flush=True)
        pause("実験 2-B Frame 0 確認: ステップ 0 回のため ContactReport は空です。")

        step_app(1)
        c_f1 = sorted({p.link_a for p in monitor.penetrating() if p.kind == "environment"})
        print(f"  Frame +1 (app.update() 1 回) ContactReport penetrating : {c_f1}", flush=True)
        pause("実験 2-B Frame +1 確認: Viewport ではロボットが赤い箱にめり込んで見えますが、1 回目の衝突検出は移動前位置で走ったため ContactReport はまだ空です。")

        step_app(1)
        c_f2 = sorted({p.link_a for p in monitor.penetrating() if p.kind == "environment"})
        print(f"  Frame +2 (app.update() 2 回) ContactReport penetrating : {c_f2}", flush=True)
        assert c_f0 == [] and c_f1 == [] and len(c_f2) >= 1, (c_f0, c_f1, c_f2)
        pause("実験 2-B Frame +2 確認: 2 回目の app.update() でようやく ContactReport にめり込みが届きました（2 update 遅れ）。")

        monitor.close()
        tl.stop()
        step_app(2)
        robot_kin.close()
        summary["exp2"] = {
            "overlap_f0": len(f0_physx),
            "overlap_f1": len(f1_physx),
            "overlap_repeat_f0": len(repeat_f0),
            "contact_f0": len(c_f0),
            "contact_f1": len(c_f1),
            "contact_f2": len(c_f2),
        }

    # =========================================================================
    # 実験 3: Newton の 0 フレーム遅延 & sync_environment(use_fabric="auto")
    # =========================================================================
    if args.exp in ("all", "3"):
        banner("実験 3: Newton の 0 フレーム遅延（めり込み深さつき）と use_fabric='auto' の切り分け")
        stage = new_demo_stage(gravity=9.81)
        add_ur10e(stage)
        add_box(stage, "/World/MovingObstacle", position=(3.0, 3.0, 1.0), size=0.24, kind="static", color=(0.15, 0.8, 0.45))
        obs_op = UsdGeom.Xformable(stage.GetPrimAtPath("/World/MovingObstacle")).GetOrderedXformOps()[0]

        robot = GeometricRobot.from_articulation(stage, "/World/UR10e", write_target="fabric", collision_backend="newton")
        robot.set_dof_positions(q_safe)
        robot.flush_to_usd()
        step_app(2)
        set_camera_view((2.0, -2.0, 1.3), (0.0, 0.0, 0.5))

        tl = omni.timeline.get_timeline_interface()
        tl.play()
        step_app(3)

        # [3-A] 自己干渉 Frame 0（flush_to_usd なし、めり込み深さまで取得）
        robot.set_dof_positions(q_self)
        assert robot._usd_dirty is True
        pairs_self = robot.check_collisions("within")
        assert robot._usd_dirty is True  # Newton は USD flush を呼ばない
        print("\n[3-A] Newton Frame 0 自己干渉（Fabric のみ・flush_to_usd なし・app.update() 前）:", flush=True)
        for p in pairs_self:
            print(f"  - {p.link_a:<16} <-> {p.link_b:<16} | separation = {p.separation:+.5f} m", flush=True)
        step_app(1)
        pause("実験 3-A 確認: Frame 0 で 5 組すべてと負の separation（めり込み深さ）が即座に得られました。")

        # [3-B] 静的障害物を USD で動かした直後の use_fabric=True vs use_fabric="auto"
        robot.set_dof_positions(q_safe)
        step_app(1)
        w3_pos = robot.links["wrist_3_link"].get_world_pose()[0]
        print("\n[3-B] Play=ON 中に静的障害物 /World/MovingObstacle を USD xformOp で wrist_3_link 位置へ移動（app.update() 前）:", flush=True)
        obs_op.Set(Gf.Vec3d(*w3_pos.tolist()))

        robot.scene.sync_environment(use_fabric=True)
        hits_fabric_forced = robot.scene.check_environment(robot.link_set, sync_env=False)
        robot.scene.sync_environment(use_fabric="auto")
        hits_auto = robot.scene.check_environment(robot.link_set, sync_env=False)

        print(f"  use_fabric=True  (古い Fabric を読む) : {len(hits_fabric_forced)} 件 -> {[p.names for p in hits_fabric_forced]}", flush=True)
        print(f"  use_fabric='auto' (静的は USD を読む) : {len(hits_auto)} 件 -> {[(p.link_a, round(p.separation, 5)) for p in hits_auto]}", flush=True)
        step_app(1)
        assert len(hits_fabric_forced) == 0, f"use_fabric=True で想定外ヒット: {hits_fabric_forced}"
        assert len(hits_auto) >= 1 and any(p.separation is not None and p.separation < 0.0 for p in hits_auto)
        pause("実験 3-B 確認: 静的障害物を USD で動かした直後は Fabric が古いため use_fabric=True は見逃し、use_fabric='auto' は即検出します。")

        tl.stop()
        step_app(2)
        robot.close()
        summary["exp3"] = {
            "newton_self_f0": len(pairs_self),
            "min_self_sep_m": round(min(p.separation for p in pairs_self if p.separation is not None), 5),
            "static_obs_use_fabric_true": len(hits_fabric_forced),
            "static_obs_use_fabric_auto": len(hits_auto),
        }

    # =========================================================================
    # 実験 4: GPU 非同期実行 (wp.launch) と wp.synchronize_device() の体感
    # =========================================================================
    if args.exp in ("all", "4"):
        banner("実験 4: Warp GPU 非同期実行（wp.launch）と wp.synchronize_device() の時間差")
        device = "cuda:0" if wp.is_cuda_available() else "cpu"
        print(f"  Warp device: {device}", flush=True)

        @wp.kernel(enable_backward=False)
        def _heavy_work(buf: wp.array(dtype=wp.float32), iters: int):
            i = wp.tid()
            x = buf[i]
            for _ in range(iters):
                x = wp.sin(x) * wp.cos(x) + 0.123456
            buf[i] = x

        n_elems = 1_000_000
        iters = 400
        buf = wp.ones(n_elems, dtype=wp.float32, device=device)

        # 初回 JIT コンパイルのウォームアップ
        wp.launch(_heavy_work, dim=n_elems, inputs=[buf, 1], device=device)
        wp.synchronize_device(device)

        t0 = time.perf_counter()
        wp.launch(_heavy_work, dim=n_elems, inputs=[buf, iters], device=device)
        t_launch_only_ms = (time.perf_counter() - t0) * 1e3

        t1 = time.perf_counter()
        wp.synchronize_device(device)
        t_sync_wait_ms = (time.perf_counter() - t1) * 1e3

        print(f"  wp.launch() が Python へ戻るまでの時間 (非同期キュー投入のみ) : {t_launch_only_ms:8.3f} ms", flush=True)
        print(f"  直後の wp.synchronize_device() が完了を待った実計算時間       : {t_sync_wait_ms:8.3f} ms", flush=True)
        print("  -> synchronize せずに戻ると、GPU がまだ計算中のメモリを後続処理・Hydra が読む危険があります。", flush=True)
        pause("実験 4 確認: wp.launch() はほぼ一瞬で戻り、実際の GPU 計算は wp.synchronize_device() の間に完了しています。")
        summary["exp4"] = {
            "device": device,
            "launch_only_ms": round(t_launch_only_ms, 3),
            "sync_wait_ms": round(t_sync_wait_ms, 3),
        }

    banner(f"全検証完了: {summary}")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
