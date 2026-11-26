"""任意ロボットに対して myext.robot.geometric_newton の全公開 API を一通り実行・print 確認するスクリプト。

特徴:
  - 任意のロボット USD / prim パスに対応 (`num_dofs` やリンク名・Component・VariantSet を API で動的取得)。
  - 関節角度は回転関節に [1, 2, 3, ...] [deg]、直動関節には可動範囲内の小さな値 [m] を自動生成してセット。
  - デバッガで 1 クリック確認しやすいよう、全 API の戻り値を浅いフラットなローカル変数へ格納。
    各セクション末尾や最後の `# <<< BREAKPOINT >>>` 行にブレークポイントを置けば、深い階層を辿らずに全結果を閲覧可能。

実行方法:
  1. コマンドラインから既定 UR10e で実行:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\demo\\demo_all_api_inspector.py --headless
  2. 任意の USD / prim パスを指定して実行:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\demo\\demo_all_api_inspector.py --usd "C:/path/to/my_robot.usd" --prim "/World/MyRobot" --headless
  3. GUI 付きで DebugDraw コライダー可視化を見ながら実行:
       C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat extsUser\\Extentions\\myext.robot.geometric_newton\\demo\\demo_all_api_inspector.py
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any

ISAAC_ROOT = Path(os.environ.get("ISAAC_PATH") or r"C:\isaacsim-6.2.0")
DEFAULT_UR10E_USD = ISAAC_ROOT / "exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"


def _is_kit_running() -> bool:
    try:
        import omni.kit.app

        return omni.kit.app.get_app() is not None
    except Exception:
        return False


def _find_first_articulation_prim(stage) -> str:
    """ステージ上から最初の ArticulationRootAPI を持つロボット prim パスを探す。"""
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            # `/World/Robot/root_joint` や `/World/Robot/base_link` なら親の `/World/Robot` を優先検討
            parent = prim.GetParent()
            if parent and not parent.IsPseudoRoot() and str(parent.GetPath()) != "/World":
                return str(parent.GetPath())
            return str(prim.GetPath())
    raise RuntimeError("ステージ上に UsdPhysics.ArticulationRootAPI を持つ prim が見つかりません。--prim で明示指定してください。")


def _prepare_stage_with_usd(usd_path: Path, prim_path: str):
    """新規ステージを作成し、指定 USD をリファレンス配置して環境障害物を 1 個追加する。"""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800.0)

    robot_prim = stage.DefinePrim(prim_path, "Xform")
    robot_prim.GetReferences().AddReference(usd_path.as_posix())

    # 環境干渉チェック・クリアランス計測・set_obstacle_pose 確認用のテスト障害物
    obs = UsdGeom.Cube.Define(stage, "/World/InspectorObstacle")
    obs.CreateSizeAttr(0.15)
    obs.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.0, 0.35))
    UsdPhysics.CollisionAPI.Apply(obs.GetPrim())
    return stage


def inspect_all_apis(
    stage,
    robot_prim_path: str,
    *,
    merge_variant_components: bool = False,
    app=None,
) -> dict[str, Any]:
    """任意ロボットに対して全公開 API を実行し、フラット変数へ格納しながら print する。"""
    import numpy as np

    from myext.robot.geometric_newton import (
        GeometricRobot,
        IKSolverConfig,
        LinearCommandConverter,
    )

    def _step(n: int = 1) -> None:
        if app is not None:
            for _ in range(n):
                app.update()

    def _sec(title: str) -> None:
        print(f"\n{'=' * 72}\n[{title}]\n{'=' * 72}")

    # ==========================================================================
    # 1. 生成・基本情報・describe (JSON)
    # ==========================================================================
    _sec("1. 生成・構造情報・describe() [JSON]")
    robot = GeometricRobot.from_articulation(
        stage,
        robot_prim_path,
        angle_unit="deg",
        merge_variant_components=merge_variant_components,
    )
    _step(2)

    robot_name = robot.name
    root_prim_paths = robot.root_prim_paths
    num_dofs = robot.num_dofs
    dof_names = robot.dof_names
    dof_types = robot.dof_types
    dof_paths = robot.dof_paths
    indep_dof_indices = robot.independent_dof_indices.tolist()
    indep_dof_names = robot.independent_dof_names
    num_joints = robot.num_joints
    joint_names = robot.joint_names
    joint_types = robot.joint_types
    num_links = robot.num_links
    link_names = robot.link_names
    link_paths = robot.link_paths
    link_path_map = robot.link_path_map
    component_names = robot.component_names
    comp_dof_map = {c: robot.get_component(c).dof_names for c in component_names}
    structure_version = robot.structure_version
    collider_version = robot.collider_version
    collision_mode = robot.layer.options.collision_mode
    layer_is_applied = robot.layer.is_applied()
    layer_change_count = len(robot.layer.report.changes)
    describe_dict = robot.describe_dict()
    describe_json = robot.describe(indent=2)

    print(f"robot_name         = {robot_name!r}")
    print(f"root_prim_paths    = {root_prim_paths}")
    print(f"num_links / joints / dofs = {num_links} / {num_joints} / {num_dofs} (independent={len(indep_dof_indices)})")
    print(f"dof_names          = {dof_names}")
    print(f"indep_dof_names    = {indep_dof_names}")
    print(f"component_names    = {component_names}")
    print(f"comp_dof_map       = {comp_dof_map}")
    print(f"collision_mode     = {collision_mode!r}, layer_is_applied={layer_is_applied}, layer_changes={layer_change_count}")
    print(f"describe() JSON (先頭 16 行):\n" + "\n".join(describe_json.splitlines()[:16]))

    # ==========================================================================
    # 2. 関節角度の読み書き ([1, 2, 3, ...] deg) ・単位・バッチ更新・単体ハンドル
    # ==========================================================================
    _sec("2. 関節値の読み書き ([1, 2, 3, ...] deg)・辞書/名前指定・batch_update・Joint/Link ハンドル")
    lower_limits_deg, upper_limits_deg = robot.get_dof_limits(unit="deg")
    lower_limits_list = np.round(lower_limits_deg, 4).tolist()
    upper_limits_list = np.round(upper_limits_deg, 4).tolist()

    # 回転関節には [1, 2, 3, ...] deg、直動関節には可動範囲内の小さな値 (0.005 * (i+1) m) を作る
    q_target_list: list[float] = []
    for i, jtype in enumerate(dof_types):
        if jtype == "revolute":
            val = float(i + 1)
        else:
            lo_i, hi_i = float(lower_limits_deg[i]), float(upper_limits_deg[i])
            if np.isfinite(lo_i) and np.isfinite(hi_i):
                val = lo_i + (hi_i - lo_i) * 0.25
            else:
                val = 0.005 * (i + 1)
        q_target_list.append(val)
    q_target_deg = np.array(q_target_list, dtype=float)

    robot.set_dof_positions(q_target_deg, unit="deg")
    q_actual_deg = np.round(robot.get_dof_positions(unit="deg"), 5).tolist()
    q_actual_rad = np.round(robot.get_dof_positions(unit="rad"), 5).tolist()
    q_dict_deg = {k: round(v, 5) for k, v in robot.get_dof_positions_dict(unit="deg").items()}
    q_dict_indep_deg = {k: round(v, 5) for k, v in robot.get_dof_positions_dict(independent_only=True, unit="deg").items()}

    # set_dof_position_targets / get_dof_position_targets 互換別名
    robot.set_dof_position_targets(q_target_deg, unit="deg")
    q_targets_deg = np.round(robot.get_dof_position_targets(unit="deg"), 5).tolist()

    # 辞書指定 & dof_names 指定 & ignore_unknown=True (未知軸混在でもエラーにしない)
    if dof_names:
        first_dof = dof_names[0]
        robot.set_dof_positions({first_dof: 5.0, "__nonexistent_variant_joint__": 99.0}, ignore_unknown=True)
        q_after_dict_first = float(robot.get_dof_positions(dof_names=[first_dof], unit="deg")[0])
        robot.set_dof_positions([7.0, 99.0], dof_names=[first_dof, "__nonexistent_variant_joint__"], ignore_unknown=True)
        q_after_names_first = float(robot.get_dof_positions(dof_names=[first_dof], unit="deg")[0])
        # 単体 Joint ハンドル
        joint0 = robot.joints[first_dof]
        joint0_limits = joint0.get_limits(unit="deg")
        joint0.set_position(1.0, unit="deg")
        joint0_pos = joint0.get_position(unit="deg")
    else:
        first_dof = None
        q_after_dict_first = None
        q_after_names_first = None
        joint0_limits = None
        joint0_pos = None

    # batch_update() で一括更新
    with robot.batch_update():
        robot.set_dof_positions(q_target_deg, unit="deg")
        if component_names:
            first_comp = robot.get_component(component_names[0])
            first_comp_dofs = first_comp.dof_names
            first_comp_q_dict = first_comp.get_dof_positions_dict(unit="deg")
        else:
            first_comp_dofs = []
            first_comp_q_dict = {}

    print(f"q_target_deg       = {q_target_deg.tolist()}")
    print(f"q_actual_deg       = {q_actual_deg}")
    print(f"q_actual_rad       = {q_actual_rad}")
    print(f"q_dict_deg         = {q_dict_deg}")
    print(f"q_dict_indep_deg   = {q_dict_indep_deg}")
    print(f"ignore_unknown OK  : dict->{q_after_dict_first}, dof_names->{q_after_names_first}, joint0->{joint0_pos}")

    # ==========================================================================
    # 3. Variant 異軸数対応 GeometricDofBinder
    # ==========================================================================
    _sec("3. GeometricDofBinder (異軸数 Variant 対応の高速配列バインダ)")
    probe_names = [*dof_names, "__phantom_tool_axis__"]
    binder = robot.bind_dofs(probe_names, ignore_unknown=True)
    binder_active = binder.active_names
    binder_missing = binder.missing_names
    binder_input = [*q_target_list, 123.0]
    binder.set_positions(binder_input, unit="deg")
    binder_readback_deg = np.round(binder.get_positions(fill_value=-999.0, unit="deg"), 5).tolist()
    print(f"binder_active      = {binder_active}")
    print(f"binder_missing     = {binder_missing}")
    print(f"binder_readback    = {binder_readback_deg}")

    # ==========================================================================
    # 4. 軸リミット制御 (clip / reject) と超過リンク取得 API
    # ==========================================================================
    _sec("4. 軸リミット制御 (clip / reject) & get_limit_violations / get_limit_violation_links")
    limit_policy_default = robot.limit_policy
    finite_idx = [i for i in range(num_dofs) if np.isfinite(upper_limits_deg[i])]
    if finite_idx:
        lim_i = finite_idx[0]
        lim_name = dof_names[lim_i]
        lim_over_val = float(upper_limits_deg[lim_i]) + 25.0

        # 4a) clip モード: 上限を超えないように切り詰め + リンク取得
        robot.limit_policy = "clip"
        robot.set_dof_positions({lim_name: lim_over_val}, unit="deg")
        is_at_limit_clip = robot.is_at_limit()
        limit_violations_clip = robot.get_limit_violations(unit="deg")
        limit_violation_links_clip = robot.get_limit_violation_links()
        q_after_clip_val = float(robot.get_dof_positions(dof_indices=[lim_i], unit="deg")[0])

        # 4b) reject モード: 範囲外指令を拒否し直前姿勢を維持 + 超過した軸/リンクを報告
        robot.set_dof_positions(q_target_deg, unit="deg")
        q_before_reject = float(robot.get_dof_positions(dof_indices=[lim_i], unit="deg")[0])
        robot.limit_policy = "reject"
        robot.set_dof_positions({lim_name: lim_over_val}, unit="deg")
        q_after_reject = float(robot.get_dof_positions(dof_indices=[lim_i], unit="deg")[0])
        limit_violations_reject = robot.get_limit_violations(include_at_limit=False, unit="deg")
        limit_violation_links_reject = robot.get_limit_violation_links(include_at_limit=False)

        # 元に戻す
        robot.limit_policy = "clip"
        robot.set_dof_positions(q_target_deg, unit="deg")
    else:
        lim_name = None
        is_at_limit_clip = False
        limit_violations_clip = []
        limit_violation_links_clip = {}
        q_after_clip_val = None
        q_before_reject = None
        q_after_reject = None
        limit_violations_reject = []
        limit_violation_links_reject = {}

    print(f"limit_policy_default       = {limit_policy_default!r}, tested_axis={lim_name!r}")
    print(f"clip   : actual={q_after_clip_val}, is_at_limit={is_at_limit_clip}, links={limit_violation_links_clip}")
    print(f"reject : before={q_before_reject} -> after={q_after_reject}, links={limit_violation_links_reject}")

    # ==========================================================================
    # 5. リンク姿勢・ベース姿勢・ヤコビアン・既定状態
    # ==========================================================================
    _sec("5. ベース姿勢・全リンク姿勢・ヤコビアン・既定状態リセット")
    base_pos, base_quat = robot.get_world_poses()
    local_pos, local_quat = robot.get_local_poses()
    link_positions, link_quats = robot.get_link_world_poses()
    link_local_pos, link_local_quat = robot.get_link_local_poses()
    link_matrices = robot.get_link_world_matrices()
    link_matrices_shape = tuple(link_matrices.shape)
    jacobian = robot.get_jacobian_matrices()
    jacobian_shape = tuple(jacobian.shape)
    tip_link_name = link_names[-1]
    tip_pos, tip_quat = robot.links[tip_link_name].get_world_pose()
    default_pos, default_quat, default_q_deg = robot.get_default_state(unit="deg")

    print(f"base_pos           = {np.round(base_pos, 4).tolist()}, base_quat={np.round(base_quat, 4).tolist()}")
    print(f"tip_link           = {tip_link_name!r}, tip_pos={np.round(tip_pos, 4).tolist()}")
    print(f"link_matrices      = shape {link_matrices_shape}, jacobian = shape {jacobian_shape}")

    # ==========================================================================
    # 6. 逆運動学 (IK)
    # ==========================================================================
    _sec("6. 逆運動学 (get_kinematic_chain / create_ik_solver / solve_ik)")
    root_link = robot.model.root_link or link_names[0]
    ik_tip = tip_link_name
    ik_chain = robot.get_kinematic_chain(root_link, ik_tip)
    if len(ik_chain.dof_indices) > 0:
        ik_solver = robot.create_ik_solver("jacobian", root_link, ik_tip, IKSolverConfig(orientation_mode="none"))
        ik_target_pos = tip_pos + np.array([0.02, -0.02, 0.02])
        ik_result = robot.solve_ik(ik_solver, ik_target_pos, apply=False)
        ik_success = bool(ik_result.success)
        ik_pos_error = float(ik_result.position_error)
        ik_dof_rad = np.round(ik_result.dof_positions, 5).tolist()
    else:
        ik_success = None
        ik_pos_error = None
        ik_dof_rad = []
    print(f"ik_chain           = {root_link} -> {ik_tip} (dofs={len(ik_chain.dof_indices)})")
    print(f"ik_success         = {ik_success}, ik_pos_error={ik_pos_error}")

    # ==========================================================================
    # 7. コライダー・Newton 干渉チェック・クリアランス・フィルタ・DebugDraw・障害物移動
    # ==========================================================================
    _sec("7. コライダー・Newton 干渉・クリアランス・BroadPhase フィルタ・DebugDraw")
    collider_paths = robot.collider_paths
    collider_counts = {k: len(v) for k, v in collider_paths.items() if v}
    collision_within = robot.check_collisions("within")
    collision_env = robot.check_collisions("environment")
    collision_all = robot.check_collisions("all")
    is_in_collision_all = robot.is_in_collision("all")
    touching_pairs_list = [p.names for p in robot.touching_pairs()]
    clearances = robot.compute_clearances("all", max_distance=0.5)
    clearances_top3 = [
        (p.link_a, p.link_b or p.path_b, round(p.separation, 5) if p.separation is not None else None)
        for p in clearances[:3]
    ]
    filter_pair_count = len(robot.collision_filter.pairs)
    broad_phase_filter_count = int(robot.scene.newton_model.shape_collision_filter_pairs_array().shape[0])
    convex_meshes_flag = robot.convex_meshes

    # 障害物移動 API (set_obstacle_pose)
    if stage.GetPrimAtPath("/World/InspectorObstacle").IsValid():
        robot.scene.set_obstacle_pose("/World/InspectorObstacle", position=tip_pos)
        collision_env_after_move = [p.names for p in robot.check_collisions("environment")]
        robot.scene.set_obstacle_pose("/World/InspectorObstacle", position=(0.45, 0.0, 0.35))
    else:
        collision_env_after_move = []

    # DebugDraw 単発描画と解除 (ヘッドレスでも安全に実行可)
    debug_draw_lines = robot.draw_colliders()
    robot.clear_collider_debug_draw()

    print(f"collider_counts    = {collider_counts}")
    print(f"within / env / all = {len(collision_within)} / {len(collision_env)} / {len(collision_all)} (is_in_collision={is_in_collision_all})")
    print(f"touching_pairs     = {touching_pairs_list[:5]} (計 {len(touching_pairs_list)} 組)")
    print(f"clearances_top3    = {clearances_top3}")
    print(f"filter_pairs / broad_phase_baked = {filter_pair_count} / {broad_phase_filter_count}")
    print(f"set_obstacle_pose  -> env hits={collision_env_after_move[:3]}")
    print(f"debug_draw_lines   = {debug_draw_lines}")

    # ==========================================================================
    # 8. コマンド変換・Render Ready フラグ・VariantSet・reset
    # ==========================================================================
    _sec("8. コマンド変換・is_render_ready・VariantSet・reset_to_default_state")
    if dof_names:
        conv = LinearCommandConverter(
            input_names=("cmd_0",),
            output_dofs=(dof_names[0],),
            matrix=np.array([[1.0]]),
            output_angle_unit="deg",
        )
        robot.set_command_converter("inspector_conv", conv)
        converter_active_names = robot.active_command_converters()
        robot.apply_command({"cmd_0": 12.0})
        converter_readback = robot.read_command()
        robot.remove_command_converter("inspector_conv")
    else:
        converter_active_names = []
        converter_readback = {}

    variants_current = robot.get_variants()
    variant_choices = robot.get_variant_choices()
    _step(4)
    is_render_ready = robot.is_render_ready
    pending_render_sync_frames = robot.pending_render_sync_frames

    # VariantSet があれば 1 つ切り替えて元に戻し、切替直後の is_render_ready も確認
    variant_switched_demo = None
    if variant_choices:
        for vp, vsets in variant_choices.items():
            for vs_name, choices in vsets.items():
                cur = variants_current.get(vp, {}).get(vs_name)
                alt = next((c for c in choices if c != cur), None)
                if alt is not None:
                    robot.set_variant(vs_name, alt, prim=vp)
                    variant_switched_demo = {
                        "prim": vp,
                        "variant_set": vs_name,
                        "from": cur,
                        "to": alt,
                        "dofs_after_switch": robot.num_dofs,
                        "render_ready_immediately": robot.is_render_ready,
                        "pending_frames_immediately": robot.pending_render_sync_frames,
                    }
                    _step(4)
                    variant_switched_demo["render_ready_after_4_frames"] = robot.is_render_ready
                    if cur:
                        robot.set_variant(vs_name, cur, prim=vp)
                        _step(4)
                    break
            if variant_switched_demo is not None:
                break

    robot.reset_to_default_state()
    q_after_reset_deg = np.round(robot.get_dof_positions(unit="deg"), 5).tolist()

    print(f"converter          = active={converter_active_names}, readback={converter_readback}")
    print(f"is_render_ready    = {is_render_ready}, pending_render_sync_frames={pending_render_sync_frames}")
    print(f"variants_current   = {variants_current}")
    print(f"variant_choices    = {variant_choices}")
    print(f"variant_switched   = {variant_switched_demo}")
    print(f"q_after_reset_deg  = {q_after_reset_deg}")

    # --------------------------------------------------------------------------
    # <<< BREAKPOINT HERE: 上記の全フラット変数をデバッガでそのまま確認できます >>>
    # --------------------------------------------------------------------------
    summary = {
        "robot_name": robot_name,
        "num_dofs": num_dofs,
        "num_links": num_links,
        "component_names": component_names,
        "collision_mode": collision_mode,
        "q_actual_deg": q_actual_deg,
        "limit_violation_links_clip": limit_violation_links_clip,
        "limit_violation_links_reject": limit_violation_links_reject,
        "ik_success": ik_success,
        "collision_counts": {
            "within": len(collision_within),
            "environment": len(collision_env),
            "all": len(collision_all),
        },
        "broad_phase_filter_count": broad_phase_filter_count,
        "is_render_ready": is_render_ready,
        "variant_switched_demo": variant_switched_demo,
    }

    robot.close()
    print("\n[DONE] 全 API の実行と close() が正常に完了しました。")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="任意ロボットの全 API 実機検証・デバッガ確認スクリプト")
    parser.add_argument("--usd", type=str, default=None, help="読み込むロボット USD ファイルパス (省略時は同梱 UR10e)")
    parser.add_argument("--prim", type=str, default=None, help="ロボットの prim パス (省略時は /World/Robot または自動検出)")
    parser.add_argument("--merge-variants", action="store_true", help="VariantSet 配下の子 IsaacRobotAPI を親 Component へ統合")
    parser.add_argument("--headless", action="store_true", help="ヘッドレスで起動して終了する")
    args, _ = parser.parse_known_args()

    running_in_kit = _is_kit_running()
    app = None
    if not running_in_kit:
        sys.path.insert(0, (ISAAC_ROOT / "tools/myext").as_posix())
        from sim_app_launcher import create_simulation_app

        app = create_simulation_app(
            {"headless": bool(args.headless)},
            extra_extensions=("myext.robot.geometric_newton",),
        )

    try:
        import omni.usd

        if running_in_kit and args.usd is None:
            stage = omni.usd.get_context().get_stage()
            prim_path = args.prim or _find_first_articulation_prim(stage)
        else:
            usd_path = Path(args.usd) if args.usd else DEFAULT_UR10E_USD
            if not usd_path.exists():
                raise FileNotFoundError(f"USD ファイルが見つかりません: {usd_path}")
            prim_path = args.prim or "/World/Robot"
            stage = _prepare_stage_with_usd(usd_path, prim_path)
            if app is not None:
                for _ in range(4):
                    app.update()

        inspect_all_apis(
            stage,
            prim_path,
            merge_variant_components=bool(args.merge_variants),
            app=app,
        )
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
