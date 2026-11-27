r"""公式 Isaac Sim アセットと assembly_pipeline 結合アセットの定量的な PhysX 動作一致テスト（全 6 種）。

テスト方針:
  - 「結合して見た目が合う」だけでなく、実際に PhysX 上で各関節へ複数の非ゼロ目標姿勢や車輪速度指令を
    与えてシミュレーションを実行し、**公式アセットと v2 結合アセットの物理挙動が定量的に一致すること**を検証する。
  - 固定ベース機（UR10e / Cobotta Pro 900 / OpenArm Bimanual）は全リンクのワールド姿勢・DOF・mimic 追従を
    比較し、重力下（`gravity=9.81`）での一致も確認する。
  - 浮遊ベース機・移動ロボット（Unitree G1 / Kaya / Carter v1）は、無重力下でのルート基準相対姿勢一致に加え、
    車輪ロボットでは `gravity=9.81` + `GroundPlane` 接地状態での並進・旋回走行軌跡（平面位置・ヨー角・車輪回転角・
    キャスター追従角）の一致まで検証する。
  - VariantSet を持つロボットでは、全バリアント切替（`None` 含む）や複数 VariantSet 同時切替時の
    DOF 数・RB 数・ArticulationRoot 単一性・非アクティブ Variant 検出（`discover_components`）も確認する。

対象ロボット（全 6 種）:
  1. UR10e + Robotiq 2F-85 / 2F-140（Gripper = None / Robotiq_2F_85 / Robotiq_2F_140、無重力 + 重力 9.81）
  2. Unitree G1 29DOF（4 VariantSet 組み合わせ: ThreeFingerx2, Inspirex2, None+Thor, ThreeFinger+Inspire+Thor）
  3. NVIDIA Kaya（3x 同一 `kaya_wheel_unit.urdf`: 無重力 33-DOF + 重力接地並進・その場旋回走行）
  4. NVIDIA Carter v1（2x 同一 `carter_wheel.urdf` + `carter_caster.urdf`: 無重力 4-DOF + 重力接地直進・差動旋回）
  5. Denso Cobotta Pro 900 + OnRobot RG6（6-DOF + 1 leader + 5 mimic + Gripper=None バリアント）
  6. OpenArm Bimanual（台座 + 2x 同一 `openarm_unimanual.usd`: 22-DOF + 両手 mimic、無重力 + 重力 9.81 + 片腕 Variant）

実行方法:
  # 全 6 ロボットを実行（AGENTS.md 必須の UTF-8 ラッパー経由）
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline\tests_kit\test_official_motion_match.py

  # 特定ロボットのみ実行（関数名の部分一致、複数指定可）
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py --only=openarm
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\test_official_motion_match.py --only=ur10e --only=g1

出力と終了コード:
  - 実行結果 JSON: `tests_kit/last_official_motion_run.json`（全ロボットの最大位置誤差・回転誤差・DOF 誤差・mimic 誤差）
  - 終了コード: 0 = 全件成功、1 = テスト失敗あり、2 = Isaac Sim 起動・読み込み失敗、3 = タイムアウト

許容誤差の設計方針:
  - 無重力での位置制御比較: リンク位置 `< 2〜5 mm`、姿勢 `< 0.5〜1.5 deg`、DOF `< 0.01〜0.03 rad`
    （PhysX TGS ソルバの収束残差・単精度変換・mimic 制約追従誤差を吸収しつつ、結合ズレや軸反転を確実に検出できる閾値）。
  - 重力下での位置制御・接地走行比較: 慣性・質量差やタイヤ接触積分の累積差を考慮し、各テストの docstring に根拠を明記。

新しいロボットテストを追加する手順:
  1. `examples/official_assets/<ロボット名>/` に TOML（および必要なら部品 URDF/USDA）を配置する。
  2. 本ファイル先頭に公式アセットのパス定数を追加する。
  3. `@test` デコレータ付きの `test_N_<ロボット名>()` を定義し、`_build_official_example()` で結合 USD を生成後、
     `_simulate_position_trajectory()` / `_simulate_grounded_velocity()` と `_compare_trajectories()` で比較し、
     メトリクス辞書を `return` する（自動的に `last_official_motion_run.json` へ保存される）。
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import shutil
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EX = os.path.join(ROOT, "examples")
OFFICIAL_EX = os.path.join(EX, "official_assets")
OUT = os.path.join(HERE, "out", "official_motion")

# 公式アセットの絶対パス（Isaac Sim 6.2.0 同梱 + ローカル S3 キャッシュ）
OFFICIAL_UR10E_USD = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd"
)
OFFICIAL_G1_USD = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.transformer.rules/data/tests/G1/g1.usda"
)
OFFICIAL_KAYA_URDF = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/kaya/urdf/kaya.urdf"
)
OFFICIAL_KAYA_MESHES = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/kaya/meshes"
)
OFFICIAL_CARTER_URDF = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter/urdf/carter.urdf"
)
OFFICIAL_CARTER_PKG = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter"
)
OFFICIAL_COBOTTA_URDF = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf"
)
OFFICIAL_COBOTTA_VIS = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/cobotta_pro_900/cobotta_pro_900_visualization"
)
OFFICIAL_RG6_VIS = (
    "C:/isaacsim-6.2.0/exts/isaacsim.asset.importer.urdf/data/urdf/robots/cobotta_pro_900/onrobot_rg6_visualization"
)
OFFICIAL_OPENARM_BIMANUAL_USD = os.path.join(
    OFFICIAL_EX,
    "openarm",
    "_s3_cache",
    "Isaac",
    "Robots",
    "OpenArm",
    "openarm_bimanual",
    "openarm_bimanual.usd",
)

# ハング時の無限待機を防ぐウォッチドッグ（60 分で強制終了コード 3）
timer = threading.Timer(3600, lambda: (print("[WATCHDOG]", flush=True), os._exit(3)))
timer.daemon = True
timer.start()

# AGENTS.md 必須ルール: SimulationApp は sim_app_launcher.create_simulation_app のみ経由で起動する
sys.path.insert(0, str(Path("C:/isaacsim-6.2.0/tools/myext")))
from sim_app_launcher import create_simulation_app  # noqa: E402

app = create_simulation_app({"headless": True})
try:
    import numpy as np
    import omni.kit.app
    import omni.physics.tensors as tensors
    import omni.usd
    from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    mgr = omni.kit.app.get_app().get_extension_manager()
    mgr.add_path(os.path.dirname(ROOT))
    mgr.set_extension_enabled_immediate("myext.robot.assembly_pipeline", True)

    from myext.robot.assembly_pipeline import (
        build_assembly,
        discover_components,
        load_spec,
        runtime,
    )
    from myext.robot.assembly_pipeline.usd_util import (
        ALL_PRIMS,
        articulation_roots,
        has_api,
        physics_variant_choice,
        rigid_bodies,
    )
except Exception:  # noqa: BLE001
    traceback.print_exc()
    print("[SUMMARY] passed 0 / failed 1 (起動・読み込みで失敗)", flush=True)
    os._exit(2)


results: list[dict[str, Any]] = []
only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--only=")]


def test(fn):
    """テスト関数を即時実行し、合否・所要時間・メトリクス辞書を `results` へ記録するデコレータ。

    `--only=<文字列>` 指定時は、関数名に指定文字列を含むテストだけを実行する。
    """
    if only and not any(o in fn.__name__ for o in only):
        return fn
    t0 = time.perf_counter()
    print(f"\n==================== {fn.__name__} ====================", flush=True)
    try:
        metrics = fn() or {}
        dt = round(time.perf_counter() - t0, 2)
        results.append({"name": fn.__name__, "ok": True, "s": dt, "metrics": metrics})
        print(f"[PASS] {fn.__name__} ({dt:.1f}s)", flush=True)
    except Exception:  # noqa: BLE001
        dt = round(time.perf_counter() - t0, 2)
        tb = traceback.format_exc()
        results.append({"name": fn.__name__, "ok": False, "s": dt, "traceback": tb})
        print(f"[FAIL] {fn.__name__} ({dt:.1f}s)\n{tb}", flush=True)
    return fn


# ====================================================================== 共通ヘルパ
def _wrap_angle(rad: float) -> float:
    """角度 [rad] を `[-pi, pi)` の範囲へ正規化する（ヨー角差・キャスター角差の比較用）。"""
    return float((rad + math.pi) % (2.0 * math.pi) - math.pi)


def _pose_from_rb_row(row: np.ndarray) -> np.ndarray:
    """`omni.physics.tensors` の RigidBodyView 行 `[x, y, z, qx, qy, qz, qw]` を列ベクトル 4x4 同次変換行列へ変換する。"""
    x, y, z, qx, qy, qz, qw = (float(v) for v in row[:7])
    m = np.eye(4, dtype=float)
    # Gf.Matrix3d は行ベクトル規約のため転置して NumPy 列ベクトル規約に揃える
    m[:3, :3] = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(qw, qx, qy, qz))), dtype=float).T
    m[:3, 3] = [x, y, z]
    return m


def _pose_err(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """2 つの 4x4 剛体姿勢の `(位置誤差 [m], 回転誤差 [deg])` を返す。"""
    pos = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    r_diff = a[:3, :3].T @ b[:3, :3]
    cos_val = float(np.clip((np.trace(r_diff) - 1.0) / 2.0, -1.0, 1.0))
    deg = float(np.degrees(math.acos(cos_val)))
    return pos, deg


def _rot_err_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """2 つの 3x3 回転行列間の最小回転角誤差 [deg] を返す。"""
    r_diff = r_a.T @ r_b
    cos_val = float(np.clip((np.trace(r_diff) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(math.acos(cos_val)))


def _yaw_from_pose(t: np.ndarray) -> float:
    """4x4 同次変換行列からワールド Z 軸回りのヨー角 [rad] を抽出する。"""
    return float(math.atan2(t[1, 0], t[0, 0]))


def _is_mimic_joint(prim: Usd.Prim) -> bool:
    """`prim` が mimic follower（`NewtonMimicAPI` または `PhysxMimicJointAPI:*` 付き）かを判定する。"""
    if not prim or not prim.IsValid():
        return False
    if has_api(prim, "NewtonMimicAPI"):
        return True
    return any(s.startswith("PhysxMimicJointAPI:") for s in prim.GetAppliedSchemas())


def _joint_limits_rad(prim: Usd.Prim) -> tuple[float | None, float | None]:
    """Revolute / Prismatic joint の可動域 `(lower, upper)` を返す（revolute は度から rad へ変換）。"""
    is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
    lo_attr = prim.GetAttribute("physics:lowerLimit")
    hi_attr = prim.GetAttribute("physics:upperLimit")
    lo = lo_attr.Get() if lo_attr else None
    hi = hi_attr.Get() if hi_attr else None

    def _clean(val: Any) -> float | None:
        if val is None or not math.isfinite(float(val)) or abs(float(val)) > 1e20:
            return None
        return math.radians(float(val)) if is_rev else float(val)

    return _clean(lo), _clean(hi)


def _clamp_target(prim: Usd.Prim, target: float, margin: float = 0.03) -> float:
    """目標関節角・変位を関節可動域の内側（端から `margin` 余裕を残した範囲）へ安全にクランプする。"""
    lo, hi = _joint_limits_rad(prim)
    if lo is not None and hi is not None and hi > lo:
        m = min(margin, 0.1 * (hi - lo))
        return float(np.clip(target, lo + m, hi - m))
    return float(target)


def _import_official_urdf(
    urdf_path: str,
    cache_dir: str,
    *,
    fix_base: bool | None,
    ros_package_paths: list[dict[str, str]] | None = None,
) -> str:
    """公式 URDF を `URDFImporter` で直接 import して基準比較用 USD パスを返す（キャッシュ付き）。

    引数:
      urdf_path: 公式 URDF ファイルの絶対パス。
      cache_dir: 変換済み USD を保存するキャッシュディレクトリ。
      fix_base: `URDFImporterConfig.fix_base` へ渡す値（`None` なら Importer 既定値を使用）。
      ros_package_paths: `package://` 解決用マッピング一覧。
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    os.makedirs(cache_dir, exist_ok=True)
    marker = os.path.join(cache_dir, "official_import.json")
    if os.path.exists(marker):
        try:
            meta = json.loads(Path(marker).read_text(encoding="utf-8"))
            if (
                meta.get("urdf_path") == urdf_path
                and meta.get("fix_base") == fix_base
                and meta.get("usd")
                and os.path.exists(meta["usd"])
            ):
                return str(meta["usd"])
        except OSError:
            pass

    staging = os.path.join(cache_dir, "_staging")
    target = os.path.join(cache_dir, "imported")
    shutil.rmtree(staging, ignore_errors=True)
    shutil.rmtree(target, ignore_errors=True)
    os.makedirs(staging, exist_ok=True)

    cfg_kw: dict[str, Any] = {
        "urdf_path": urdf_path,
        "usd_path": staging,
        "ros_package_paths": ros_package_paths or [],
    }
    if fix_base is not None:
        cfg_kw["fix_base"] = fix_base
    out = URDFImporter(URDFImporterConfig(**cfg_kw)).import_urdf()
    shutil.move(os.path.dirname(out), target)
    shutil.rmtree(staging, ignore_errors=True)
    generated = os.path.join(target, os.path.basename(out))
    Path(marker).write_text(
        json.dumps({"urdf_path": urdf_path, "fix_base": fix_base, "usd": generated}, indent=1),
        encoding="utf-8",
    )
    return generated


def _build_official_example(toml_rel: str, out_subdir: str):
    """`examples/official_assets/<toml_rel>` を `OUT/<out_subdir>` へビルドし `(spec, built, final_usda)` を返す。

    `build_assembly()` でエラーが 1 件でもあれば即座に `AssertionError` とする。
    """
    toml_path = os.path.join(OFFICIAL_EX, toml_rel)
    assert os.path.exists(toml_path), f"TOML が見つからない: {toml_path}"
    spec = dataclasses.replace(load_spec(toml_path), output_dir=os.path.join(OUT, out_subdir))
    built = build_assembly(spec, check_idempotency=False, log=lambda *_: None)
    final_jf = spec.join_files[-1]
    comp = built["compositions"][final_jf.usda]
    errs = {r.jid: r.error for r in comp.joins if r.error}
    assert not errs, f"build_assembly で結合エラー ({toml_rel}): {errs}"
    final_usda = os.path.join(str(spec.output_dir), final_jf.usda)
    assert os.path.exists(final_usda), f"結合 USDA が生成されていない: {final_usda}"
    return spec, built, final_usda


def _setup_robot_stage(
    usd_path: str,
    robot_path: str,
    *,
    gravity: float = 0.0,
    variant_selections: dict[str, str] | None = None,
    spawn_pos: tuple[float, float, float] | None = None,
    with_ground: bool = False,
) -> tuple[Usd.Stage, str]:
    """クリーンなプローブステージへロボットを参照ロードし、`(stage, articulation_root_path)` を返す。

    処理内容:
      1. `upAxis="Z"`, `metersPerUnit=1.0`, 指定重力で新規ステージを作成（必要なら高摩擦 `GroundPlane` 追加）。
      2. `robot_path` に `usd_path` を Reference 追加し、`Physics` を優先した順序で VariantSet を選択する。
      3. `Physics` VariantSet が未選択のまま残るアセットでは自動的に PhysX 系バリアントを選択する。
      4. `ArticulationRootAPI` が厳密に 1 箇所だけ存在することをアサートして返す。
    """
    stage = runtime.new_probe_stage(up_axis="Z", mpu=1.0, gravity=gravity)
    if with_ground:
        PhysicsSchemaTools.addGroundPlane(
            stage, "/GroundPlane", "Z", 100.0, Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.5)
        )
        gp = stage.GetPrimAtPath("/GroundPlane")
        if gp and gp.IsValid():
            mat = UsdPhysics.MaterialAPI.Apply(gp)
            mat.CreateStaticFrictionAttr(1.0)
            mat.CreateDynamicFrictionAttr(1.0)
            mat.CreateRestitutionAttr(0.0)

    prim = stage.DefinePrim(robot_path, "Xform")
    prim.GetReferences().AddReference(usd_path)

    vs_sets = prim.GetVariantSets()
    selected_vs = dict(variant_selections or {})
    # Physics Variant を先に確定させないと配下の物理 prim が展開されないアセットがあるため優先適用する
    ordered_vs_items = sorted(selected_vs.items(), key=lambda kv: (0 if kv[0] == "Physics" else 1, kv[0]))
    for vs_name, choice in ordered_vs_items:
        assert vs_sets.HasVariantSet(vs_name), (
            f"VariantSet {vs_name!r} が無い ({usd_path}): available={vs_sets.GetNames()}"
        )
        vs = prim.GetVariantSet(vs_name)
        names = vs.GetVariantNames()
        assert choice in names, f"VariantSet {vs_name!r} に {choice!r} が無い: {names}"
        vs.SetVariantSelection(choice)

    if "Physics" not in selected_vs and vs_sets.HasVariantSet("Physics"):
        pvs = prim.GetVariantSet("Physics")
        cur = pvs.GetVariantSelection()
        if not cur or cur == "None":
            choice = physics_variant_choice(prim, "auto")
            if choice:
                pvs.SetVariantSelection(choice)

    if spawn_pos is not None:
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*spawn_pos))

    runtime.step(2)
    roots = articulation_roots(stage, robot_path)
    assert len(roots) == 1, f"ArticulationRoot が 1 つではない ({usd_path}, {selected_vs}): {roots}"
    return stage, roots[0]


def _configure_position_drive(
    stage: Usd.Stage,
    joint_path: str,
    *,
    stiffness: float = 1e4,
    damping: float = 1e3,
    max_force: float = float("inf"),
) -> None:
    """関節 prim に位置制御用の `UsdPhysics.DriveAPI`（stiffness / damping / maxForce）を設定する。"""
    prim = stage.GetPrimAtPath(joint_path)
    assert prim and prim.IsValid(), f"Joint prim が無い: {joint_path}"
    kind = "linear" if prim.IsA(UsdPhysics.PrismaticJoint) else "angular"
    d = UsdPhysics.DriveAPI.Apply(prim, kind)
    d.CreateStiffnessAttr(float(stiffness))
    d.CreateDampingAttr(float(damping))
    d.CreateMaxForceAttr(float(max_force))


def _configure_velocity_drive(
    stage: Usd.Stage,
    joint_path: str,
    *,
    damping: float = 100.0,
    max_force: float = float("inf"),
) -> None:
    """関節 prim に速度制御用の `UsdPhysics.DriveAPI`（`stiffness=0`、指定 `damping`）を設定する。"""
    prim = stage.GetPrimAtPath(joint_path)
    assert prim and prim.IsValid(), f"Joint prim が無い: {joint_path}"
    kind = "linear" if prim.IsA(UsdPhysics.PrismaticJoint) else "angular"
    d = UsdPhysics.DriveAPI.Apply(prim, kind)
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(float(damping))
    d.CreateMaxForceAttr(float(max_force))


def _zero_drive(stage: Usd.Stage, joint_path: str) -> None:
    """受動関節や mimic follower が余計な復元力を出さないよう既存 drive の剛性・減衰を 0 にする。"""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim or not prim.IsValid():
        return
    for kind in ("angular", "linear"):
        if prim.HasAPI(UsdPhysics.DriveAPI, kind):
            d = UsdPhysics.DriveAPI(prim, kind)
            d.CreateStiffnessAttr(0.0)
            d.CreateDampingAttr(0.0)


def _simulate_position_trajectory(
    usd_path: str,
    robot_path: str,
    *,
    gravity: float,
    variant_selections: dict[str, str] | None,
    rb_key_fn,
    dof_key_fn,
    pose_targets_fn,
    drive_params_fn,
    steps_per_pose: int = 60,
    ramp_steps: int = 0,
    zero_uncommanded_mimic: bool = False,
) -> dict[str, Any]:
    """ロボットを PhysX 上で複数ポーズの位置制御軌道に沿って駆動し、各ポーズ収束後の DOF・全剛体姿勢を返す。

    引数:
      usd_path: テスト対象の USD ファイルパス（公式または結合版）。
      robot_path: ステージ上の配置 prim パス。
      gravity: 重力加速度 [m/s^2]（`0.0` で純粋な運動学・制約比較、`9.81` で重力負荷下比較）。
      variant_selections: 選択する VariantSet 辞書。
      rb_key_fn / dof_key_fn: 公式と結合版で異なる階層パスを共通の比較キーへ正規化する関数（`None` 返却で比較除外）。
      pose_targets_fn: `(stage, active_joints, mimic_keys) -> list[dict[dof_key, target]]` を返す目標ポーズ生成関数。
      drive_params_fn: `(dof_key, joint_path) -> (stiffness, damping, max_force)` を返すゲイン決定関数。
      steps_per_pose: 各目標ポーズで進める PhysX ステップ数。
      ramp_steps: `> 0` のとき目標値へ線形補間するステップ数（高自由度浮遊ベースの衝撃抑制用）。
      zero_uncommanded_mimic: `True` のとき非指令 mimic follower の drive を 0 クリアして公式/結合条件を揃える。
    """
    stage, art_root = _setup_robot_stage(
        usd_path,
        robot_path,
        gravity=gravity,
        variant_selections=variant_selections,
    )

    # 非アクティブ prim（sublayer 上で active=false にした旧リンク等）を除外してアクティブ剛体のみ収集する
    rb_paths = [p for p in rigid_bodies(stage, robot_path) if stage.GetPrimAtPath(p).IsActive()]
    rb_key_to_path: dict[str, str] = {}
    for p in rb_paths:
        k = rb_key_fn(stage, robot_path, p)
        if k is None:
            continue
        assert k not in rb_key_to_path, f"RB キー衝突 {k!r}: {rb_key_to_path[k]} vs {p}"
        rb_key_to_path[k] = p

    # Play 前にステージ上のアクティブ可動 Joint を走査して drive を設定する
    active_joints: dict[str, str] = {}
    mimic_keys: set[str] = set()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path), ALL_PRIMS):
        if not prim.IsActive() or not prim.IsA(UsdPhysics.Joint):
            continue
        if prim.IsA(UsdPhysics.FixedJoint):
            continue
        j = UsdPhysics.Joint(prim)
        en = j.GetJointEnabledAttr().Get()
        if en is not None and not bool(en):
            continue
        if j.GetExcludeFromArticulationAttr().Get():
            continue
        jpath = str(prim.GetPath())
        jk = dof_key_fn(stage, robot_path, jpath)
        if jk is None:
            continue
        active_joints[jk] = jpath
        if _is_mimic_joint(prim):
            mimic_keys.add(jk)

    pose_list: list[dict[str, float]] = pose_targets_fn(stage, active_joints, mimic_keys)
    commanded_keys = {k for pose in pose_list for k in pose}

    for jk, jpath in active_joints.items():
        if jk in commanded_keys:
            stiff, damp, max_f = drive_params_fn(jk, jpath)
            _configure_position_drive(stage, jpath, stiffness=stiff, damping=damp, max_force=max_f)
        elif zero_uncommanded_mimic and jk in mimic_keys:
            _zero_drive(stage, jpath)

    runtime.step(1)
    runtime.timeline().play()
    runtime.step(2)

    sv = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
    av = sv.create_articulation_view(art_root)
    assert av is not None, f"ArticulationView 作成失敗: {art_root}"

    px_dof_paths = list(av.dof_paths[0])
    px_dof_keys: list[str] = []
    key_to_dof_idx: dict[str, int] = {}
    for idx, jpath in enumerate(px_dof_paths):
        jk = dof_key_fn(stage, robot_path, jpath)
        assert jk is not None, f"PhysX DOF のキー解決失敗: {jpath}"
        assert jk not in key_to_dof_idx, f"PhysX DOF キー衝突 {jk!r}: {jpath}"
        px_dof_keys.append(jk)
        key_to_dof_idx[jk] = idx

    ordered_rb_keys = sorted(rb_key_to_path)
    ordered_rb_paths = [rb_key_to_path[k] for k in ordered_rb_keys]
    rb_view = sv.create_rigid_body_view(ordered_rb_paths)
    indices = np.array([0], dtype=np.uint32)

    # 初期姿勢（ステップ前）を記録（定数オフセット回転を持つリンクの相対回転比較に使用）
    t_init = rb_view.get_transforms().copy()
    init_rb_poses = {k: _pose_from_rb_row(t_init[i]) for i, k in enumerate(ordered_rb_keys)}
    q_prev = av.get_dof_positions().astype(np.float32, copy=True)

    snapshots: list[dict[str, Any]] = []
    for pose in pose_list:
        q_target = np.zeros((1, len(px_dof_paths)), dtype=np.float32)
        for jk, val in pose.items():
            if jk in key_to_dof_idx:
                q_target[0, key_to_dof_idx[jk]] = float(val)
        if ramp_steps > 0:
            n_ramp = min(ramp_steps, steps_per_pose)
            for s_idx in range(1, n_ramp + 1):
                alpha = float(s_idx) / float(n_ramp)
                q_interp = q_prev + alpha * (q_target - q_prev)
                av.set_dof_position_targets(q_interp.astype(np.float32), indices)
                runtime.step(1)
            hold = steps_per_pose - n_ramp
            if hold > 0:
                av.set_dof_position_targets(q_target, indices)
                runtime.step(hold)
        else:
            av.set_dof_position_targets(q_target, indices)
            runtime.step(steps_per_pose)
        q_prev = q_target.copy()

        q_actual = av.get_dof_positions()[0].copy()
        t_cur = rb_view.get_transforms().copy()
        snapshots.append(
            {
                "dofs": {px_dof_keys[i]: float(q_actual[i]) for i in range(len(px_dof_keys))},
                "rbs": {k: _pose_from_rb_row(t_cur[i]) for i, k in enumerate(ordered_rb_keys)},
            }
        )

    runtime.stop()
    return {
        "dof_count": len(px_dof_paths),
        "dof_keys": px_dof_keys,
        "mimic_keys": sorted(mimic_keys),
        "rb_count": len(ordered_rb_keys),
        "rb_keys": ordered_rb_keys,
        "init_rbs": init_rb_poses,
        "snapshots": snapshots,
    }


def _compare_trajectories(
    off: dict[str, Any],
    asm: dict[str, Any],
    *,
    root_link_key: str | None = None,
    dof_sign_map: dict[str, float] | None = None,
    dof_offset_map: dict[str, float] | None = None,
    rb_relative_orient_keys: set[str] | None = None,
) -> dict[str, Any]:
    """公式と結合版の複数ポーズスナップショットを比較し、最大 DOF 誤差・最大リンク位置/回転誤差を返す。

    引数:
      off / asm: `_simulate_position_trajectory()` の戻り値。
      root_link_key:
        - `None` のとき: 全リンクのワールド姿勢を直接比較（固定ベース機）。
        - 指定時: `T_root^-1 @ T_link` の相対姿勢を比較（浮遊ベース機）。
      dof_sign_map: 特定 DOF の符号反転マップ（例: Carter 右輪・OpenArm 右 joint7 = `-1.0`）。
      dof_offset_map: 特定 DOF の定数ゼロ点オフセットマップ `q_off = sign * q_asm + offset`
        （例: OpenArm Bimanual は `q2=0` が真下垂下、Unimanual は `q2=0` が水平展開のため `±pi/2`）。
      rb_relative_orient_keys: 定数オフセット回転を持つリンク（例: Carter 右輪の `Rz(pi)`）について、
        初期姿勢からの相対回転 `R(t) @ R(0)^T` で姿勢一致を検証する対象キー集合。
    """
    assert off["dof_count"] == asm["dof_count"], (
        f"DOF 数不一致: official={off['dof_count']} ({off['dof_keys']}) vs assembled={asm['dof_count']} ({asm['dof_keys']})"
    )
    assert set(off["dof_keys"]) == set(asm["dof_keys"]), (
        f"DOF キー集合不一致: diff={set(off['dof_keys']) ^ set(asm['dof_keys'])}"
    )
    assert set(off["rb_keys"]) == set(asm["rb_keys"]), (
        f"RB キー集合不一致: only_off={set(off['rb_keys']) - set(asm['rb_keys'])}, "
        f"only_asm={set(asm['rb_keys']) - set(off['rb_keys'])}"
    )
    assert len(off["snapshots"]) == len(asm["snapshots"])

    signs = dof_sign_map or {}
    offsets = dof_offset_map or {}
    rel_ori_keys = rb_relative_orient_keys or set()

    max_dof_err = 0.0
    worst_dof = ""
    max_pos_err = 0.0
    worst_pos_rb = ""
    max_rot_err = 0.0
    worst_rot_rb = ""

    def _rel_pose(rbs: dict[str, np.ndarray], key: str) -> np.ndarray:
        if root_link_key is None:
            return rbs[key]
        return np.linalg.inv(rbs[root_link_key]) @ rbs[key]

    init_off = {k: _rel_pose(off["init_rbs"], k) for k in off["rb_keys"]}
    init_asm = {k: _rel_pose(asm["init_rbs"], k) for k in asm["rb_keys"]}

    for p_idx, (s_off, s_asm) in enumerate(zip(off["snapshots"], asm["snapshots"])):
        for dk in off["dof_keys"]:
            sgn = float(signs.get(dk, 1.0))
            off_val = float(offsets.get(dk, 0.0))
            err = abs(s_off["dofs"][dk] - (sgn * s_asm["dofs"][dk] + off_val))
            if err > max_dof_err:
                max_dof_err = err
                worst_dof = f"pose{p_idx}:{dk}"

        for rk in off["rb_keys"]:
            t_off = _rel_pose(s_off["rbs"], rk)
            t_asm = _rel_pose(s_asm["rbs"], rk)
            pos_e = float(np.linalg.norm(t_off[:3, 3] - t_asm[:3, 3]))
            if rk in rel_ori_keys:
                r_off_delta = t_off[:3, :3] @ init_off[rk][:3, :3].T
                r_asm_delta = t_asm[:3, :3] @ init_asm[rk][:3, :3].T
                rot_e = _rot_err_deg(r_off_delta, r_asm_delta)
            else:
                rot_e = _rot_err_deg(t_off[:3, :3], t_asm[:3, :3])
            if pos_e > max_pos_err:
                max_pos_err = pos_e
                worst_pos_rb = f"pose{p_idx}:{rk}"
            if rot_e > max_rot_err:
                max_rot_err = rot_e
                worst_rot_rb = f"pose{p_idx}:{rk}"

    return {
        "dof_count": off["dof_count"],
        "rb_count": off["rb_count"],
        "poses_tested": len(off["snapshots"]),
        "max_dof_err_rad": round(max_dof_err, 6),
        "worst_dof": worst_dof,
        "max_link_pos_err_m": round(max_pos_err, 6),
        "worst_pos_rb": worst_pos_rb,
        "max_link_rot_err_deg": round(max_rot_err, 4),
        "worst_rot_rb": worst_rot_rb,
    }


def _simulate_grounded_velocity(
    usd_path: str,
    robot_path: str,
    *,
    spawn_z: float,
    chassis_key: str,
    rb_key_fn,
    dof_key_fn,
    wheel_keys: list[str],
    wheel_damping: float,
    phases: list[tuple[str, dict[str, float], int]],
    settle_steps: int = 25,
) -> dict[str, Any]:
    """重力 9.81 + `GroundPlane` 上へロボットをスポーンし、車輪速度指令で接地走行した軌跡を返す。

    手順:
      1. `spawn_z` の高さへスポーンし、駆動輪には速度ドライブ・受動輪/ローラーにはゼロドライブを設定する。
      2. ゼロ速度で `settle_steps` ステップ接地安定化させ、初期位置 `(x0, y0, yaw0)` と初期 DOF `q0` を記録する。
      3. `phases` の各走行フェーズ `(フェーズ名, {車輪キー: 目標角速度 [rad/s]}, ステップ数)` を順に実行し、
         基準姿勢からの変位 `(dx, dy, dyaw)` と各 DOF の積算変位を記録して返す。
    """
    stage, art_root = _setup_robot_stage(
        usd_path,
        robot_path,
        gravity=9.81,
        spawn_pos=(0.0, 0.0, spawn_z),
        with_ground=True,
    )

    rb_paths = [p for p in rigid_bodies(stage, robot_path) if stage.GetPrimAtPath(p).IsActive()]
    chassis_path = None
    for p in rb_paths:
        if rb_key_fn(stage, robot_path, p) == chassis_key:
            chassis_path = p
            break
    assert chassis_path is not None, f"chassis RB {chassis_key!r} が見つからない: {rb_paths}"

    active_joints: dict[str, str] = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path), ALL_PRIMS):
        if not prim.IsActive() or not prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.FixedJoint):
            continue
        j = UsdPhysics.Joint(prim)
        en = j.GetJointEnabledAttr().Get()
        if en is not None and not bool(en):
            continue
        if j.GetExcludeFromArticulationAttr().Get():
            continue
        jk = dof_key_fn(stage, robot_path, str(prim.GetPath()))
        if jk is not None:
            active_joints[jk] = str(prim.GetPath())

    wheel_set = set(wheel_keys)
    for jk, jpath in active_joints.items():
        if jk in wheel_set:
            _configure_velocity_drive(stage, jpath, damping=wheel_damping, max_force=float("inf"))
        else:
            _zero_drive(stage, jpath)

    runtime.step(1)
    runtime.timeline().play()
    runtime.step(2)

    sv = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
    av = sv.create_articulation_view(art_root)
    assert av is not None, f"ArticulationView 作成失敗: {art_root}"
    px_dof_paths = list(av.dof_paths[0])
    px_dof_keys = [dof_key_fn(stage, robot_path, p) for p in px_dof_paths]
    key_to_idx = {k: i for i, k in enumerate(px_dof_keys)}
    for wk in wheel_keys:
        assert wk in key_to_idx, f"車輪 DOF {wk!r} が PhysX に無い: {px_dof_keys}"

    rb_view = sv.create_rigid_body_view([chassis_path])
    indices = np.array([0], dtype=np.uint32)

    # 接地安定化（落下直後のバウンドが収まってから走行基準姿勢を記録）
    zero_v = np.zeros((1, len(px_dof_paths)), dtype=np.float32)
    av.set_dof_velocity_targets(zero_v, indices)
    runtime.step(settle_steps)

    t0 = _pose_from_rb_row(rb_view.get_transforms()[0])
    q0 = av.get_dof_positions()[0].copy()
    x0, y0, yaw0 = float(t0[0, 3]), float(t0[1, 3]), _yaw_from_pose(t0)

    phase_records: dict[str, dict[str, Any]] = {}
    for pname, v_map, n_steps in phases:
        v_vec = np.zeros((1, len(px_dof_paths)), dtype=np.float32)
        for jk, vel in v_map.items():
            v_vec[0, key_to_idx[jk]] = float(vel)
        av.set_dof_velocity_targets(v_vec, indices)
        runtime.step(n_steps)
        t_now = _pose_from_rb_row(rb_view.get_transforms()[0])
        q_now = av.get_dof_positions()[0].copy()
        dx = float(t_now[0, 3]) - x0
        dy = float(t_now[1, 3]) - y0
        dyaw = _wrap_angle(_yaw_from_pose(t_now) - yaw0)
        phase_records[pname] = {
            "x": dx,
            "y": dy,
            "z": float(t_now[2, 3]),
            "yaw": dyaw,
            "dofs": {px_dof_keys[i]: float(q_now[i]) for i in range(len(px_dof_keys))},
            "delta_dofs": {px_dof_keys[i]: float(q_now[i] - q0[i]) for i in range(len(px_dof_keys))},
        }

    runtime.stop()
    return {"settled_z": float(t0[2, 3]), "phases": phase_records}


# ====================================================================== Test 1: UR10e + Robotiq 2F-85 / 2F-140
@test
def test_1_ur10e_all_gripper_variants():
    """UR10e + Robotiq 2F-85 / 2F-140（USD+USD 結合 + Gripper VariantSet）の動作一致を検証する。

    テストの目的:
      - 公式 USD 同士の結合および `Gripper` VariantSet（`None` / `Robotiq_2F_85` / `Robotiq_2F_140`）が、
        公式 `ur10e.usd` と完全に同じ関節自由度・リンク姿勢・mimic 動作を再現することを確認する。
    セットアップ:
      - `examples/official_assets/ur10e/ur10e.toml` をビルドし、公式 `ur10e.usd` と比較する。
      - アーム 6 軸 + グリッパ代表軸 `finger_joint` を動かす 3 ポーズ軌道（`gravity=0.0`）と、
        `Robotiq_2F_85` での重力負荷下（`gravity=9.81`）1 ポーズ軌道を実行する。
    検証内容と許容誤差の根拠:
      - `discover_components(..., include_inactive_variants=True)` で本体とグリッパを検出できること。
      - 各バリアントの DOF 数・RB 数一致（`None`: 6/7, `2F_85`: 12/16, `2F_140`: 14/16）、mimic 追従量 `> 0.1 rad`。
      - `None` / `2F_85`: 位置 `< 3 mm`・姿勢 `< 1.0 deg`・DOF `< 0.02 rad`。
      - `2F_140` および `gravity=9.81`: 並列リンク機構の閉ループ制約収束残差・重力たわみ差を考慮し
        位置 `< 5 mm`・姿勢 `< 1.5 deg`・DOF `< 0.03 rad`（重力下 `0.02 rad`）。
    """
    spec, _built, asm_usda = _build_official_example(
        os.path.join("ur10e", "ur10e.toml"), "ur10e"
    )

    # discover_components の確認（非アクティブ variant 含む）
    st = Usd.Stage.Open(asm_usda)
    dp_path = str(st.GetDefaultPrim().GetPath())
    comps = discover_components(st, dp_path, include_inactive_variants=True)
    assert len(comps) >= 2, f"discover_components がグリッパを検出できていない: {comps}"

    # TOML が定義した Gripper 用 VariantSet 名を取得（既定 "Gripper"）
    jf = spec.join_files[-1]
    vs_candidates = {j.variant_set for j in jf.joins if j.variant_set}
    asm_gripper_vs = "Gripper" if "Gripper" in vs_candidates else next(iter(vs_candidates))
    gripper_child_names = {j.child_prim_name or j.name for j in jf.joins}

    def _ur10e_rb_key(_stage: Usd.Stage, robot_path: str, p: str) -> str:
        rel = p[len(robot_path) :].lstrip("/")
        leaf = p.rsplit("/", 1)[-1]
        parts = rel.split("/")
        # UR10e 本体にも Robotiq 2F-85 にも "base_link" があるためグリッパ配下を区別する
        in_gripper = any(
            seg.startswith("Robotiq") or seg in gripper_child_names for seg in parts[:-1]
        )
        return f"gripper/{leaf}" if in_gripper else leaf

    def _ur10e_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        return jpath.rsplit("/", 1)[-1]

    arm_poses = [
        {
            "shoulder_pan_joint": 0.4,
            "shoulder_lift_joint": -0.6,
            "elbow_joint": 0.8,
            "wrist_1_joint": -0.5,
            "wrist_2_joint": 0.6,
            "wrist_3_joint": -0.4,
            "finger_joint": 0.25,
        },
        {
            "shoulder_pan_joint": -0.7,
            "shoulder_lift_joint": -1.1,
            "elbow_joint": 1.3,
            "wrist_1_joint": 0.4,
            "wrist_2_joint": -0.7,
            "wrist_3_joint": 0.9,
            "finger_joint": 0.55,
        },
        {
            "shoulder_pan_joint": 0.2,
            "shoulder_lift_joint": -0.3,
            "elbow_joint": -0.6,
            "wrist_1_joint": -0.9,
            "wrist_2_joint": 0.3,
            "wrist_3_joint": -0.8,
            "finger_joint": 0.15,
        },
    ]

    expected_counts = {
        "None": (6, 7),
        "Robotiq_2F_85": (12, 16),
        "Robotiq_2F_140": (14, 16),
    }

    metrics: dict[str, Any] = {"discover_components": comps, "variants": {}}

    for variant in ("None", "Robotiq_2F_85", "Robotiq_2F_140"):
        def _poses_for(stage: Usd.Stage, active_joints: dict[str, str], _mimics: set[str], v=variant):
            out = []
            for p in arm_poses:
                row = {}
                for k, val in p.items():
                    if k == "finger_joint" and v == "None":
                        continue
                    if k in active_joints:
                        row[k] = _clamp_target(stage.GetPrimAtPath(active_joints[k]), val)
                out.append(row)
            return out

        def _drive_for(jk: str, _jpath: str):
            if jk == "finger_joint":
                return (500.0, 50.0, float("inf"))
            return (1e4, 1e3, float("inf"))

        off = _simulate_position_trajectory(
            OFFICIAL_UR10E_USD,
            "/ur10e_official",
            gravity=0.0,
            variant_selections={"Gripper": variant, "Physics": "PhysX"},
            rb_key_fn=_ur10e_rb_key,
            dof_key_fn=_ur10e_dof_key,
            pose_targets_fn=_poses_for,
            drive_params_fn=_drive_for,
            steps_per_pose=60,
        )
        asm = _simulate_position_trajectory(
            asm_usda,
            "/ur10e_assembled",
            gravity=0.0,
            variant_selections={asm_gripper_vs: variant},
            rb_key_fn=_ur10e_rb_key,
            dof_key_fn=_ur10e_dof_key,
            pose_targets_fn=_poses_for,
            drive_params_fn=_drive_for,
            steps_per_pose=60,
        )

        cmp = _compare_trajectories(off, asm)
        exp_dof, exp_rb = expected_counts[variant]
        assert cmp["dof_count"] == exp_dof, f"UR10e[{variant}] DOF 数: {cmp['dof_count']} != {exp_dof}"
        assert cmp["rb_count"] == exp_rb, f"UR10e[{variant}] RB 数: {cmp['rb_count']} != {exp_rb}"

        # mimic follower が実際に追従していることを確認
        if variant != "None":
            mimic_max = max(
                abs(s["dofs"][mk]) for s in asm["snapshots"] for mk in asm["mimic_keys"]
            ) if asm["mimic_keys"] else 0.0
            cmp["mimic_count"] = len(asm["mimic_keys"])
            cmp["max_mimic_motion_rad"] = round(mimic_max, 4)
            assert mimic_max > 0.1, f"UR10e[{variant}] mimic follower が動いていない: {mimic_max}"

        pos_tol = 0.005 if variant == "Robotiq_2F_140" else 0.003
        rot_tol = 1.5 if variant == "Robotiq_2F_140" else 1.0
        dof_tol = 0.03 if variant == "Robotiq_2F_140" else 0.02

        print(
            f"  UR10e[{variant}] gravity=0: dofs={cmp['dof_count']} rbs={cmp['rb_count']} "
            f"max_pos={cmp['max_link_pos_err_m']:.6f}m max_rot={cmp['max_link_rot_err_deg']:.4f}deg "
            f"max_dof={cmp['max_dof_err_rad']:.6f}rad",
            flush=True,
        )
        assert cmp["max_link_pos_err_m"] < pos_tol, f"UR10e[{variant}] 位置誤差超過: {cmp}"
        assert cmp["max_link_rot_err_deg"] < rot_tol, f"UR10e[{variant}] 姿勢誤差超過: {cmp}"
        assert cmp["max_dof_err_rad"] < dof_tol, f"UR10e[{variant}] DOF 誤差超過: {cmp}"
        metrics["variants"][variant] = cmp

    # 重力下（gravity=9.81）でのドライブ追従一致も Robotiq_2F_85 で追加検証
    def _grav_poses(stage: Usd.Stage, active_joints: dict[str, str], _mimics: set[str]):
        return [
            {
                k: _clamp_target(stage.GetPrimAtPath(active_joints[k]), val)
                for k, val in arm_poses[1].items()
                if k in active_joints
            }
        ]

    off_g = _simulate_position_trajectory(
        OFFICIAL_UR10E_USD,
        "/ur10e_off_g",
        gravity=9.81,
        variant_selections={"Gripper": "Robotiq_2F_85", "Physics": "PhysX"},
        rb_key_fn=_ur10e_rb_key,
        dof_key_fn=_ur10e_dof_key,
        pose_targets_fn=_grav_poses,
        drive_params_fn=lambda jk, _: (500.0, 50.0, float("inf")) if jk == "finger_joint" else (2e4, 2e3, float("inf")),
        steps_per_pose=60,
    )
    asm_g = _simulate_position_trajectory(
        asm_usda,
        "/ur10e_asm_g",
        gravity=9.81,
        variant_selections={asm_gripper_vs: "Robotiq_2F_85"},
        rb_key_fn=_ur10e_rb_key,
        dof_key_fn=_ur10e_dof_key,
        pose_targets_fn=_grav_poses,
        drive_params_fn=lambda jk, _: (500.0, 50.0, float("inf")) if jk == "finger_joint" else (2e4, 2e3, float("inf")),
        steps_per_pose=60,
    )
    cmp_g = _compare_trajectories(off_g, asm_g)
    print(
        f"  UR10e[Robotiq_2F_85] gravity=9.81: max_pos={cmp_g['max_link_pos_err_m']:.6f}m "
        f"max_rot={cmp_g['max_link_rot_err_deg']:.4f}deg max_dof={cmp_g['max_dof_err_rad']:.6f}rad",
        flush=True,
    )
    assert cmp_g["max_link_pos_err_m"] < 0.005, f"UR10e gravity=9.81 位置誤差超過: {cmp_g}"
    assert cmp_g["max_link_rot_err_deg"] < 1.5, f"UR10e gravity=9.81 姿勢誤差超過: {cmp_g}"
    assert cmp_g["max_dof_err_rad"] < 0.02, f"UR10e gravity=9.81 DOF 誤差超過: {cmp_g}"
    metrics["gravity_9_81_2F_85"] = cmp_g
    return metrics


# ====================================================================== Test 2: Unitree G1 29DOF Humanoid
@test
def test_2_g1_variant_combinations():
    """Unitree G1 29DOF ヒューマノイド（3 系統の独立 VariantSet + compound 部品）の動作一致を検証する。

    テストの目的:
      - 浮遊ベース高自由度（最大 53-DOF / 56 RB）ロボットに対し、`left_hand` / `right_hand` / `Thor` の
        3 つの独立 VariantSet 組み合わせ、Inspire 指の mimic 駆動、および剛体なし compound 部品
        （`Thor` バックパック: `torso_link` 配下で `MassAPI` + `CollisionAPI` のみ付加し DOF を増やさない）を検証する。
    セットアップ:
      - `examples/official_assets/g1/g1.toml` をビルドし、公式 `g1.usda` と 4 組み合わせで比較する:
        `ThreeFinger_x2` (43 DOF), `Inspire_x2` (53 DOF), `None_ThorAttached` (29 DOF), `Mixed_3F_Inspire_Thor` (48 DOF)。
      - 体幹・両脚・両腕・両手指を動かす 2 ポーズを `ramp_steps=30` で滑らかに駆動する（超軽量な Inspire 指の
        TGS ソルバ発散を防ぐため、指・手・体幹で実機相当の 3 段階ゲインを使用）。
    検証内容と許容誤差の根拠:
      - `pelvis` 基準の相対姿勢比較で位置 `< 5 mm`・姿勢 `< 1.5 deg`・DOF `< 0.02 rad`
        （最大 53-DOF 浮遊ベース + 12 本の弱減衰 mimic 指の TGS 反復収束残差を許容しつつ、手首オフセットや
        回転ズレを確実に検出できる基準）。
      - `Thor="Attached"` 時は `backpack_link` が `torso_link` 配下にあり `RigidBodyAPI` なし・`MassAPI` / `CollisionAPI` ありを確認。
    """
    spec, _built, asm_usda = _build_official_example(os.path.join("g1", "g1.toml"), "g1")

    # g1_base が選択している Physics variant（例: "PhysX" または "SimplifiedPhysX"）を公式側にも揃える
    g1_physics_variant = "PhysX"
    for us in getattr(spec, "usds", ()):
        if "g1_base" in us.usda:
            vs_dict = dict(us.variant_selections)
            if "Physics" in vs_dict:
                g1_physics_variant = vs_dict["Physics"]

    # TOML の VariantSet 名を解決（通常 left_hand / right_hand / Thor）
    jf = spec.join_files[-1]
    vs_names = {j.variant_set for j in jf.joins if j.variant_set}

    def _find_vs(default_name: str, token: str) -> str:
        if default_name in vs_names:
            return default_name
        for n in sorted(vs_names):
            if token in n.lower():
                return n
        raise AssertionError(f"G1 TOML に {default_name!r} 相当の VariantSet が無い: {vs_names}")

    vs_left = _find_vs("left_hand", "left")
    vs_right = _find_vs("right_hand", "right")
    vs_thor = _find_vs("Thor", "thor")

    def _g1_rb_key(_stage: Usd.Stage, _robot_path: str, p: str) -> str:
        return p.rsplit("/", 1)[-1]

    def _g1_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        return jpath.rsplit("/", 1)[-1]

    # 代表的な体幹・脚・両腕の 2 ポーズ（関節リミット内に自動クランプ）
    body_poses = [
        {
            "left_hip_pitch_joint": -0.25,
            "left_knee_joint": 0.5,
            "left_ankle_pitch_joint": -0.25,
            "right_hip_pitch_joint": 0.2,
            "right_knee_joint": 0.35,
            "right_ankle_pitch_joint": -0.15,
            "waist_yaw_joint": 0.25,
            "waist_roll_joint": 0.1,
            "waist_pitch_joint": 0.1,
            "left_shoulder_pitch_joint": -0.5,
            "left_shoulder_roll_joint": 0.4,
            "left_elbow_joint": 0.7,
            "left_wrist_roll_joint": 0.3,
            "left_wrist_pitch_joint": 0.2,
            "left_wrist_yaw_joint": -0.3,
            "right_shoulder_pitch_joint": 0.4,
            "right_shoulder_roll_joint": -0.4,
            "right_elbow_joint": 0.6,
            "right_wrist_roll_joint": -0.3,
            "right_wrist_pitch_joint": -0.2,
            "right_wrist_yaw_joint": 0.3,
        },
        {
            "left_hip_pitch_joint": 0.15,
            "left_knee_joint": 0.25,
            "left_ankle_pitch_joint": -0.1,
            "right_hip_pitch_joint": -0.3,
            "right_knee_joint": 0.55,
            "right_ankle_pitch_joint": -0.25,
            "waist_yaw_joint": -0.3,
            "waist_roll_joint": -0.1,
            "waist_pitch_joint": -0.08,
            "left_shoulder_pitch_joint": 0.3,
            "left_shoulder_roll_joint": 0.2,
            "left_elbow_joint": 0.4,
            "left_wrist_roll_joint": -0.4,
            "left_wrist_pitch_joint": -0.2,
            "left_wrist_yaw_joint": 0.4,
            "right_shoulder_pitch_joint": -0.6,
            "right_shoulder_roll_joint": -0.25,
            "right_elbow_joint": 0.85,
            "right_wrist_roll_joint": 0.35,
            "right_wrist_pitch_joint": 0.25,
            "right_wrist_yaw_joint": -0.35,
        },
    ]

    combos = [
        ("ThreeFinger_x2", "ThreeFinger", "ThreeFinger", "None", 43, 46),
        ("Inspire_x2", "Inspire", "Inspire", "None", 53, 56),
        ("None_ThorAttached", "None", "None", "Attached", 29, 30),
        ("Mixed_3F_Inspire_Thor", "ThreeFinger", "Inspire", "Attached", 48, 51),
    ]

    metrics: dict[str, Any] = {"physics_variant": g1_physics_variant, "combos": {}}

    for label, lh, rh, thor, exp_dof, exp_rb in combos:
        off_vs = {"left_hand": lh, "right_hand": rh, "Thor": thor, "Physics": g1_physics_variant}
        asm_vs = {vs_left: lh, vs_right: rh, vs_thor: thor}

        # Thor="Attached" のときは compound child（torso_link 配下の backpack_link）に
        # PhysicsMassAPI と Collider があり、余分な RigidBodyAPI / DOF が無いことを検証
        thor_check: dict[str, Any] = {}
        if thor == "Attached":
            st_check, _ = _setup_robot_stage(asm_usda, "/g1_thor_check", gravity=0.0, variant_selections=asm_vs)
            bp_prims = [
                p
                for p in Usd.PrimRange(st_check.GetPrimAtPath("/g1_thor_check"), ALL_PRIMS)
                if p.IsActive() and p.GetName() == "backpack_link"
            ]
            assert len(bp_prims) >= 1, "Thor=Attached なのに backpack_link が見つからない"
            bp = bp_prims[0]
            bp_path = str(bp.GetPath())
            assert "/torso_link/" in bp_path, f"backpack_link が torso_link 配下に無い: {bp_path}"
            assert not bp.HasAPI(UsdPhysics.RigidBodyAPI), (
                f"backpack_link に RigidBodyAPI が付いている (compound child であるべき): {bp_path}"
            )
            has_mass = bp.HasAPI(UsdPhysics.MassAPI) or any(
                p.HasAPI(UsdPhysics.MassAPI) for p in Usd.PrimRange(bp, ALL_PRIMS) if p.IsActive()
            )
            has_col = bp.HasAPI(UsdPhysics.CollisionAPI) or any(
                p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(bp, ALL_PRIMS) if p.IsActive()
            )
            assert has_mass, f"backpack_link に PhysicsMassAPI が無い: {bp_path}"
            assert has_col, f"backpack_link に CollisionAPI が無い: {bp_path}"
            thor_check = {"backpack_path": bp_path, "has_mass": has_mass, "has_collider": has_col}
            runtime.stop()

        body_key_set = set(body_poses[0].keys())

        def _g1_poses(stage: Usd.Stage, active_joints: dict[str, str], mimics: set[str]):
            out = []
            hand_fracs = (0.35, 0.70)
            for idx, bp in enumerate(body_poses):
                row: dict[str, float] = {}
                for jk, jpath in active_joints.items():
                    if jk in mimics:
                        continue
                    prim = stage.GetPrimAtPath(jpath)
                    if jk in bp:
                        row[jk] = _clamp_target(prim, bp[jk])
                    elif jk not in body_key_set:
                        # 手の非 mimic 関節は実可動域から安全に非ゼロ目標を生成
                        lo, hi = _joint_limits_rad(prim)
                        if lo is not None and hi is not None and hi > lo:
                            row[jk] = float(lo + hand_fracs[idx] * (hi - lo))
                        else:
                            row[jk] = 0.25 if idx == 0 else 0.5
                out.append(row)
            return out

        def _g1_drive(jk: str, _jpath: str):
            # Inspire 指は 1.8-4.5g（I ~ 5e-8..7e-7, 原仕様 stiffness=0.05..0.25, maxForce=1.0）のため
            # 過大なゲインや体幹の瞬間ステップ入力は 53-DOF 浮遊ベース TGS ソルバおよび弱減衰 mimic を不安定化する
            if jk.startswith("L_") or jk.startswith("R_"):
                return (0.5, 0.05, 1.0)
            if "_hand_" in jk:
                return (50.0, 5.0, 20.0)
            return (500.0, 100.0, 200.0)

        off = _simulate_position_trajectory(
            OFFICIAL_G1_USD,
            "/g1_official",
            gravity=0.0,
            variant_selections=off_vs,
            rb_key_fn=_g1_rb_key,
            dof_key_fn=_g1_dof_key,
            pose_targets_fn=_g1_poses,
            drive_params_fn=_g1_drive,
            steps_per_pose=60,
            ramp_steps=30,
        )
        asm = _simulate_position_trajectory(
            asm_usda,
            "/g1_assembled",
            gravity=0.0,
            variant_selections=asm_vs,
            rb_key_fn=_g1_rb_key,
            dof_key_fn=_g1_dof_key,
            pose_targets_fn=_g1_poses,
            drive_params_fn=_g1_drive,
            steps_per_pose=60,
            ramp_steps=30,
        )

        cmp = _compare_trajectories(off, asm, root_link_key="pelvis")
        assert cmp["dof_count"] == exp_dof, f"G1[{label}] DOF 数: {cmp['dof_count']} != {exp_dof}"
        assert cmp["rb_count"] == exp_rb, f"G1[{label}] RB 数: {cmp['rb_count']} != {exp_rb}"

        if "Inspire" in (lh, rh):
            mimic_max = max(
                abs(s["dofs"][mk]) for s in asm["snapshots"] for mk in asm["mimic_keys"]
            ) if asm["mimic_keys"] else 0.0
            cmp["mimic_count"] = len(asm["mimic_keys"])
            cmp["max_mimic_motion_rad"] = round(mimic_max, 4)
            assert mimic_max > 0.05, f"G1[{label}] Inspire mimic が動いていない: {mimic_max}"

        if thor_check:
            cmp["thor_compound_check"] = thor_check

        print(
            f"  G1[{label}]: dofs={cmp['dof_count']} rbs={cmp['rb_count']} "
            f"max_pos={cmp['max_link_pos_err_m']:.6f}m max_rot={cmp['max_link_rot_err_deg']:.4f}deg "
            f"max_dof={cmp['max_dof_err_rad']:.6f}rad",
            flush=True,
        )
        assert cmp["max_link_pos_err_m"] < 0.005, f"G1[{label}] 相対位置誤差超過: {cmp}"
        assert cmp["max_link_rot_err_deg"] < 1.5, f"G1[{label}] 相対姿勢誤差超過: {cmp}"
        assert cmp["max_dof_err_rad"] < 0.02, f"G1[{label}] DOF 誤差超過: {cmp}"
        metrics["combos"][label] = cmp

    return metrics


# ====================================================================== Test 3: NVIDIA Kaya (3x kaya_wheel_unit)
@test
def test_3_kaya_kinematic_and_grounded_rolling():
    """NVIDIA Kaya 全方向 3 輪台車（3x 同一 `kaya_wheel_unit.urdf` 再利用）の運動学・接地走行一致を検証する。

    テストの目的:
      - 1 つの車輪ユニット URDF（オムニホイール + 10 個の受動ローラー）を 120° 間隔で 3 回再利用結合しても、
        公式一体型 `kaya.urdf` と同一の 33-DOF・45 RB 構造およびオムニホイール接地転がり物理が再現されることを確認する。
    セットアップ:
      - 公式 `kaya.urdf`（`fix_base=False`）を直接 import した USD と、`examples/official_assets/kaya/kaya.toml` を比較。
      - Subtest 3a: 無重力で 3 車軸を非ゼロ角度へ駆動（2 ポーズ）し、全 45 リンクの `base_link` 相対姿勢を比較。
      - Subtest 3b: `gravity=9.81` + `GroundPlane` 上で並進フェーズ（`translate`）と旋回フェーズ（`spin`）を連続実行。
    検証内容と許容誤差の根拠:
      - 3a（無重力）: DOF=33, RB=45、位置 `< 3 mm`・姿勢 `< 1.0 deg`・DOF `< 0.02 rad`。
      - 3b（接地走行）: オムニローラーの多数点接触を含む動的走行のため、平面位置 `< 30 mm`・ヨー `< 5 deg`・
        車軸積算回転 `< 0.15 rad`（並進距離 `> 5 cm`・旋回量 `> 0.1 rad` で実際に走行していることも担保）。
    """
    off_usd = _import_official_urdf(
        OFFICIAL_KAYA_URDF,
        os.path.join(OUT, "_official_kaya"),
        fix_base=False,
        ros_package_paths=[{"name": "meshes", "path": OFFICIAL_KAYA_MESHES}],
    )
    _spec, _built, asm_usda = _build_official_example(os.path.join("kaya", "kaya.toml"), "kaya")

    # wheel_0 / wheel_1 / wheel_2 の正規化キー対応（公式 mx12w_i / axle_i / wheel_i / roller_i_s_n）
    def _kaya_off_rb_key(_stage: Usd.Stage, _robot_path: str, p: str) -> str:
        return p.rsplit("/", 1)[-1]

    def _kaya_asm_rb_key(_stage: Usd.Stage, robot_path: str, p: str) -> str:
        rel = p[len(robot_path) :].lstrip("/")
        leaf = p.rsplit("/", 1)[-1]
        for idx in (0, 1, 2):
            tag = f"wheel_{idx}"
            if f"/{tag}/" in f"/{rel}":
                if leaf in ("mx12w", "axle", "wheel"):
                    return f"{leaf}_{idx}"
                if leaf.startswith("roller_"):
                    return f"roller_{idx}_{leaf[len('roller_'):]}"
        return leaf

    def _kaya_off_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        return jpath.rsplit("/", 1)[-1]

    def _kaya_asm_dof_key(_stage: Usd.Stage, robot_path: str, jpath: str) -> str:
        rel = jpath[len(robot_path) :].lstrip("/")
        leaf = jpath.rsplit("/", 1)[-1]
        for idx in (0, 1, 2):
            tag = f"wheel_{idx}"
            if f"/{tag}/" in f"/{rel}":
                if leaf == "axle_joint":
                    return f"axle_{idx}_joint"
                if leaf.startswith("roller_"):
                    return f"roller_{idx}_{leaf[len('roller_'):]}"
        return leaf

    # --- Subtest 3a: 無重力 33-DOF + 45 RB 相対姿勢一致 ---
    kaya_poses = [
        {"axle_0_joint": 0.8, "axle_1_joint": -1.2, "axle_2_joint": 0.5},
        {"axle_0_joint": -0.6, "axle_1_joint": 0.9, "axle_2_joint": -1.4},
    ]

    off_3a = _simulate_position_trajectory(
        off_usd,
        "/kaya_official",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_kaya_off_rb_key,
        dof_key_fn=_kaya_off_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: kaya_poses,
        drive_params_fn=lambda _jk, _jp: (500.0, 50.0, float("inf")),
        steps_per_pose=50,
    )
    asm_3a = _simulate_position_trajectory(
        asm_usda,
        "/kaya_assembled",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_kaya_asm_rb_key,
        dof_key_fn=_kaya_asm_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: kaya_poses,
        drive_params_fn=lambda _jk, _jp: (500.0, 50.0, float("inf")),
        steps_per_pose=50,
    )

    cmp_3a = _compare_trajectories(off_3a, asm_3a, root_link_key="base_link")
    assert cmp_3a["dof_count"] == 33, f"Kaya DOF 数不一致: {cmp_3a['dof_count']}"
    assert cmp_3a["rb_count"] == 45, f"Kaya RB 数不一致: {cmp_3a['rb_count']}"
    print(
        f"  Kaya[3a zero-g]: dofs={cmp_3a['dof_count']} rbs={cmp_3a['rb_count']} "
        f"max_pos={cmp_3a['max_link_pos_err_m']:.6f}m max_rot={cmp_3a['max_link_rot_err_deg']:.4f}deg "
        f"max_dof={cmp_3a['max_dof_err_rad']:.6f}rad",
        flush=True,
    )
    assert cmp_3a["max_link_pos_err_m"] < 0.003, f"Kaya 3a 位置誤差超過: {cmp_3a}"
    assert cmp_3a["max_link_rot_err_deg"] < 1.0, f"Kaya 3a 姿勢誤差超過: {cmp_3a}"
    assert cmp_3a["max_dof_err_rad"] < 0.02, f"Kaya 3a DOF 誤差超過: {cmp_3a}"

    # --- Subtest 3b: 重力 9.81 + GroundPlane での動的転がり走行一致 ---
    axle_keys = ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
    kaya_drive_phases = [
        ("translate", {"axle_0_joint": 0.0, "axle_1_joint": 5.0, "axle_2_joint": -5.0}, 60),
        ("spin", {"axle_0_joint": 4.0, "axle_1_joint": 4.0, "axle_2_joint": 4.0}, 60),
    ]
    off_3b = _simulate_grounded_velocity(
        off_usd,
        "/kaya_off_ground",
        spawn_z=0.10,
        chassis_key="base_link",
        rb_key_fn=_kaya_off_rb_key,
        dof_key_fn=_kaya_off_dof_key,
        wheel_keys=axle_keys,
        wheel_damping=10.0,
        phases=kaya_drive_phases,
        settle_steps=25,
    )
    asm_3b = _simulate_grounded_velocity(
        asm_usda,
        "/kaya_asm_ground",
        spawn_z=0.10,
        chassis_key="base_link",
        rb_key_fn=_kaya_asm_rb_key,
        dof_key_fn=_kaya_asm_dof_key,
        wheel_keys=axle_keys,
        wheel_damping=10.0,
        phases=kaya_drive_phases,
        settle_steps=25,
    )

    grounded_metrics: dict[str, Any] = {}
    for pname, _, _ in kaya_drive_phases:
        po = off_3b["phases"][pname]
        pa = asm_3b["phases"][pname]
        xy_err = float(math.hypot(po["x"] - pa["x"], po["y"] - pa["y"]))
        yaw_err_deg = float(abs(math.degrees(_wrap_angle(po["yaw"] - pa["yaw"]))))
        axle_err = max(abs(po["delta_dofs"][k] - pa["delta_dofs"][k]) for k in axle_keys)
        off_disp = float(math.hypot(po["x"], po["y"]))
        grounded_metrics[pname] = {
            "official_xy_yaw": [round(po["x"], 4), round(po["y"], 4), round(math.degrees(po["yaw"]), 2)],
            "assembled_xy_yaw": [round(pa["x"], 4), round(pa["y"], 4), round(math.degrees(pa["yaw"]), 2)],
            "planar_pos_err_m": round(xy_err, 5),
            "yaw_err_deg": round(yaw_err_deg, 3),
            "max_axle_rot_err_rad": round(axle_err, 5),
        }
        print(
            f"  Kaya[3b {pname}]: off=({po['x']:.3f},{po['y']:.3f},{math.degrees(po['yaw']):.1f}deg) "
            f"asm=({pa['x']:.3f},{pa['y']:.3f},{math.degrees(pa['yaw']):.1f}deg) "
            f"xy_err={xy_err:.5f}m yaw_err={yaw_err_deg:.3f}deg axle_err={axle_err:.5f}rad",
            flush=True,
        )
        if pname == "translate":
            assert off_disp > 0.05, f"Kaya translate で十分移動していない: {off_disp}"
        else:
            assert abs(po["yaw"]) > 0.1, f"Kaya spin で十分回転していない: {po['yaw']}"
        assert xy_err < 0.03, f"Kaya 3b[{pname}] 平面位置誤差超過: {grounded_metrics[pname]}"
        assert yaw_err_deg < 5.0, f"Kaya 3b[{pname}] ヨー誤差超過: {grounded_metrics[pname]}"
        assert axle_err < 0.15, f"Kaya 3b[{pname}] 車軸回転誤差超過: {grounded_metrics[pname]}"

    return {"subtest_3a_zero_g": cmp_3a, "subtest_3b_grounded": grounded_metrics}


# ====================================================================== Test 4: NVIDIA Carter v1
@test
def test_4_carter_kinematic_and_grounded_driving():
    """NVIDIA Carter v1 差動 2 輪 + 後輪キャスター台車（2x 同一車輪 URDF 再利用）の動作一致を検証する。

    テストの目的:
      - 同一の `carter_wheel.urdf` を左右へ再利用（右輪は `Rz(pi)` 反転配置のため回転軸符号が逆転）し、
        さらに受動キャスター `carter_caster.urdf` を結合したアセンブリが公式 `carter.urdf` と一致することを確認する。
    セットアップ:
      - 公式 `carter.urdf`（`fix_base=False`）を直接 import した USD と、`examples/official_assets/carter/carter.toml` を比較。
      - Subtest 4a: 無重力 4-DOF（左右車輪・キャスター旋回・後輪軸）の位置制御比較（右輪は符号反転・相対回転比較）。
      - Subtest 4b: `gravity=9.81` + `GroundPlane` で直進（`straight`）→ 差動旋回（`turn`）を連続実行し、
        車体軌跡と受動キャスター追従角（`rear_pivot`）を比較。
    検証内容と許容誤差の根拠:
      - 4a（無重力）: DOF=4, RB=7（補助マウント剛体 2 個は除外）、位置 `< 3 mm`・姿勢 `< 1.0 deg`・DOF `< 0.02 rad`。
      - 4b（接地走行）: 公式 `carter.urdf` が `pi/2` を `1.57057` に丸めているため、同一車輪 URDF の `Rz(pi)` 再利用で
        右輪円柱に 0.026° のトー角差が生じ、120 ステップ累積旋回（約 1.35 m, -92°）で ~5 cm / ~5.5° の差が出る。
        このため `straight` は `30 mm / 3 deg`、`turn` は `60 mm / 7 deg`、車輪角 `< 0.05 rad`・キャスター角 `< 0.10 rad` とする。
    """
    off_usd = _import_official_urdf(
        OFFICIAL_CARTER_URDF,
        os.path.join(OUT, "_official_carter"),
        fix_base=False,
        ros_package_paths=[{"name": "carter", "path": OFFICIAL_CARTER_PKG}],
    )
    _spec, _built, asm_usda = _build_official_example(os.path.join("carter", "carter.toml"), "carter")

    def _carter_off_rb_key(_stage: Usd.Stage, _robot_path: str, p: str) -> str:
        return p.rsplit("/", 1)[-1]

    def _carter_asm_rb_key(_stage: Usd.Stage, robot_path: str, p: str) -> str | None:
        rel = p[len(robot_path) :].lstrip("/")
        leaf = p.rsplit("/", 1)[-1]
        # 結合用に設けた 1g マウント剛体は公式 URDF には存在しない補助リンクのため比較対象から除外
        if leaf in ("wheel_mount", "caster_mount"):
            return None
        if "/left_wheel/" in f"/{rel}" and leaf == "wheel_link":
            return "left_wheel_link"
        if "/right_wheel/" in f"/{rel}" and leaf == "wheel_link":
            return "right_wheel_link"
        return leaf

    def _carter_off_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        return jpath.rsplit("/", 1)[-1]

    def _carter_asm_dof_key(_stage: Usd.Stage, robot_path: str, jpath: str) -> str:
        rel = jpath[len(robot_path) :].lstrip("/")
        leaf = jpath.rsplit("/", 1)[-1]
        if "/left_wheel/" in f"/{rel}" and leaf == "wheel_joint":
            return "left_wheel"
        if "/right_wheel/" in f"/{rel}" and leaf == "wheel_joint":
            return "right_wheel"
        return leaf

    # --- Subtest 4a: 無重力 4-DOF + 7 リンク相対姿勢一致（右輪の回転軸符号反転を考慮） ---
    carter_off_poses = [
        {"left_wheel": 0.9, "right_wheel": -0.7, "rear_pivot": 0.6, "rear_axle": 1.1},
        {"left_wheel": -0.5, "right_wheel": 1.2, "rear_pivot": -0.8, "rear_axle": -0.6},
    ]
    carter_asm_poses = [
        {k: (-v if k == "right_wheel" else v) for k, v in p.items()} for p in carter_off_poses
    ]

    off_4a = _simulate_position_trajectory(
        off_usd,
        "/carter_official",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_carter_off_rb_key,
        dof_key_fn=_carter_off_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: carter_off_poses,
        drive_params_fn=lambda _jk, _jp: (2e3, 2e2, float("inf")),
        steps_per_pose=50,
    )
    asm_4a = _simulate_position_trajectory(
        asm_usda,
        "/carter_assembled",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_carter_asm_rb_key,
        dof_key_fn=_carter_asm_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: carter_asm_poses,
        drive_params_fn=lambda _jk, _jp: (2e3, 2e2, float("inf")),
        steps_per_pose=50,
    )

    # right_wheel_link は Rz(pi) 配置のため定数フレーム回転を差し引いた相対回転で比較
    cmp_4a = _compare_trajectories(
        off_4a,
        asm_4a,
        root_link_key="chassis_link",
        dof_sign_map={"right_wheel": -1.0},
        rb_relative_orient_keys={"right_wheel_link"},
    )
    assert cmp_4a["dof_count"] == 4, f"Carter DOF 数不一致: {cmp_4a['dof_count']}"
    assert cmp_4a["rb_count"] == 7, f"Carter RB 数不一致: {cmp_4a['rb_count']}"
    print(
        f"  Carter[4a zero-g]: dofs={cmp_4a['dof_count']} rbs={cmp_4a['rb_count']} "
        f"max_pos={cmp_4a['max_link_pos_err_m']:.6f}m max_rot={cmp_4a['max_link_rot_err_deg']:.4f}deg "
        f"max_dof={cmp_4a['max_dof_err_rad']:.6f}rad",
        flush=True,
    )
    assert cmp_4a["max_link_pos_err_m"] < 0.003, f"Carter 4a 位置誤差超過: {cmp_4a}"
    assert cmp_4a["max_link_rot_err_deg"] < 1.0, f"Carter 4a 姿勢誤差超過: {cmp_4a}"
    assert cmp_4a["max_dof_err_rad"] < 0.02, f"Carter 4a DOF 誤差超過: {cmp_4a}"

    # --- Subtest 4b: 重力 9.81 + GroundPlane での直進 + 差動旋回（後輪キャスター追従含む） ---
    wheel_keys = ["left_wheel", "right_wheel"]
    off_phases = [
        ("straight", {"left_wheel": 4.0, "right_wheel": 4.0}, 60),
        ("turn", {"left_wheel": 5.0, "right_wheel": 2.0}, 60),
    ]
    asm_phases = [
        (name, {"left_wheel": v["left_wheel"], "right_wheel": -v["right_wheel"]}, steps)
        for name, v, steps in off_phases
    ]

    off_4b = _simulate_grounded_velocity(
        off_usd,
        "/carter_off_ground",
        spawn_z=0.25,
        chassis_key="chassis_link",
        rb_key_fn=_carter_off_rb_key,
        dof_key_fn=_carter_off_dof_key,
        wheel_keys=wheel_keys,
        wheel_damping=500.0,
        phases=off_phases,
        settle_steps=25,
    )
    asm_4b = _simulate_grounded_velocity(
        asm_usda,
        "/carter_asm_ground",
        spawn_z=0.25,
        chassis_key="chassis_link",
        rb_key_fn=_carter_asm_rb_key,
        dof_key_fn=_carter_asm_dof_key,
        wheel_keys=wheel_keys,
        wheel_damping=500.0,
        phases=asm_phases,
        settle_steps=25,
    )

    grounded_metrics: dict[str, Any] = {}
    for pname, _, _ in off_phases:
        po = off_4b["phases"][pname]
        pa = asm_4b["phases"][pname]
        xy_err = float(math.hypot(po["x"] - pa["x"], po["y"] - pa["y"]))
        yaw_err_deg = float(abs(math.degrees(_wrap_angle(po["yaw"] - pa["yaw"]))))
        lw_err = abs(po["delta_dofs"]["left_wheel"] - pa["delta_dofs"]["left_wheel"])
        rw_err = abs(po["delta_dofs"]["right_wheel"] - (-pa["delta_dofs"]["right_wheel"]))
        caster_err = abs(_wrap_angle(po["dofs"]["rear_pivot"] - pa["dofs"]["rear_pivot"]))
        grounded_metrics[pname] = {
            "official_xy_yaw": [round(po["x"], 4), round(po["y"], 4), round(math.degrees(po["yaw"]), 2)],
            "assembled_xy_yaw": [round(pa["x"], 4), round(pa["y"], 4), round(math.degrees(pa["yaw"]), 2)],
            "planar_pos_err_m": round(xy_err, 5),
            "yaw_err_deg": round(yaw_err_deg, 3),
            "max_wheel_rot_err_rad": round(max(lw_err, rw_err), 5),
            "caster_swivel_err_rad": round(caster_err, 5),
            "official_caster_rad": round(po["dofs"]["rear_pivot"], 4),
            "assembled_caster_rad": round(pa["dofs"]["rear_pivot"], 4),
        }
        print(
            f"  Carter[4b {pname}]: off=({po['x']:.3f},{po['y']:.3f},{math.degrees(po['yaw']):.1f}deg) "
            f"asm=({pa['x']:.3f},{pa['y']:.3f},{math.degrees(pa['yaw']):.1f}deg) "
            f"xy_err={xy_err:.5f}m yaw_err={yaw_err_deg:.3f}deg caster_err={caster_err:.5f}rad",
            flush=True,
        )
        # 公式 carter.urdf が pi/2 を 1.57057 に丸めているため（0.0002264 rad = 0.013 deg）、
        # 同一 carter_wheel.urdf を Rz(pi) で右輪に再利用すると右輪円柱の微小トー角符号が反転し（0.026 deg 差）、
        # 120 ステップ累積旋回走行（~1.35 m, -92.6 deg）で ~0.05 m / ~5.5 deg の累積差が生じる
        if pname == "straight":
            assert math.hypot(po["x"], po["y"]) > 0.3, f"Carter straight で十分前進していない: {po}"
            xy_tol, yaw_tol = 0.03, 3.0
        else:
            assert abs(po["yaw"]) > 0.15, f"Carter turn で十分旋回していない: {po['yaw']}"
            xy_tol, yaw_tol = 0.06, 7.0
        assert xy_err < xy_tol, f"Carter 4b[{pname}] 平面位置誤差超過: {grounded_metrics[pname]}"
        assert yaw_err_deg < yaw_tol, f"Carter 4b[{pname}] ヨー誤差超過: {grounded_metrics[pname]}"
        assert max(lw_err, rw_err) < 0.05, f"Carter 4b[{pname}] 車輪回転誤差超過: {grounded_metrics[pname]}"
        assert caster_err < 0.10, f"Carter 4b[{pname}] キャスター角誤差超過: {grounded_metrics[pname]}"

    return {"subtest_4a_zero_g": cmp_4a, "subtest_4b_grounded": grounded_metrics}


# ====================================================================== Test 5: Denso Cobotta Pro 900 + OnRobot RG6
@test
def test_5_cobotta_pro_900_and_rg6():
    """Denso Cobotta Pro 900 + OnRobot RG6（URDF+URDF 結合 + 5 本 mimic 連動 + Gripper=None）を検証する。

    テストの目的:
      - 公式一体型 URDF をアーム単体 + RG6 グリッパ単体へ分割して v2 パイプラインで再結合した際、
        6 軸アームと 5 本の平行リンク mimic 関節（乗数 `+1 / -1`）が公式と厳密に一致すること、
        および `Gripper="None"` バリアントで素の 6-DOF アームへクリーンに切り替わることを確認する。
    セットアップ:
      - 公式 `cobotta_pro_900.urdf` と `examples/official_assets/cobotta_pro_900/cobotta_pro_900.toml` を比較。
      - Subtest 5a (`OnRobot_RG6`): 全 6 軸 + `finger_joint` の 3 ポーズ軌道。
      - Subtest 5b (`None`): `cobotta_pro_900_arm.usda` 単体と `Gripper="None"` バリアントの 2 ポーズ比較。
    検証内容と許容誤差の根拠:
      - 固定ベース無重力比較でありモデル形状・質量が完全一致するため厳格な閾値を適用:
        位置 `< 2 mm`・姿勢 `< 0.5 deg`・DOF `< 0.01 rad`・5 本 mimic 関係誤差 `< 0.02 rad`。
    """
    cobotta_pkgs = [
        {"name": "cobotta_pro_900_visualization", "path": OFFICIAL_COBOTTA_VIS},
        {"name": "onrobot_rg6_visualization", "path": OFFICIAL_RG6_VIS},
    ]
    off_usd = _import_official_urdf(
        OFFICIAL_COBOTTA_URDF,
        os.path.join(OUT, "_official_cobotta"),
        fix_base=None,
        ros_package_paths=cobotta_pkgs,
    )
    spec, _built, asm_usda = _build_official_example(
        os.path.join("cobotta_pro_900", "cobotta_pro_900.toml"), "cobotta_pro_900"
    )
    jf = spec.join_files[-1]
    vs_candidates = {j.variant_set for j in jf.joins if j.variant_set}
    gripper_vs = "Gripper" if "Gripper" in vs_candidates else next(iter(vs_candidates))

    def _cobotta_rb_key(_stage: Usd.Stage, _robot_path: str, p: str) -> str:
        return p.rsplit("/", 1)[-1]

    def _cobotta_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        return jpath.rsplit("/", 1)[-1]

    cobotta_poses = [
        {
            "joint_1": 0.5,
            "joint_2": 0.4,
            "joint_3": 0.8,
            "joint_4": -0.6,
            "joint_5": 0.7,
            "joint_6": -0.5,
            "finger_joint": 0.35,
        },
        {
            "joint_1": -0.8,
            "joint_2": -0.35,
            "joint_3": 1.1,
            "joint_4": 0.7,
            "joint_5": -0.6,
            "joint_6": 0.9,
            "finger_joint": -0.30,
        },
        {
            "joint_1": 0.25,
            "joint_2": 0.55,
            "joint_3": -0.7,
            "joint_4": -0.4,
            "joint_5": 0.5,
            "joint_6": -1.0,
            "finger_joint": 0.50,
        },
    ]

    # --- Subtest 5a: OnRobot_RG6 付き（6 arm + 1 leader + 5 mimic = 12 DOF, 14 RB） ---
    def _drive_cobotta(jk: str, _jp: str):
        if jk == "finger_joint":
            return (500.0, 50.0, float("inf"))
        # URDF の effort="1" のままだと慣性で失速するため maxForce=inf を与えて純粋な姿勢収束を比較する
        return (1e4, 1e3, float("inf"))

    off_5a = _simulate_position_trajectory(
        off_usd,
        "/cobotta_official",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_cobotta_rb_key,
        dof_key_fn=_cobotta_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: cobotta_poses,
        drive_params_fn=_drive_cobotta,
        steps_per_pose=60,
        zero_uncommanded_mimic=True,
    )
    asm_5a = _simulate_position_trajectory(
        asm_usda,
        "/cobotta_assembled",
        gravity=0.0,
        variant_selections={gripper_vs: "OnRobot_RG6"},
        rb_key_fn=_cobotta_rb_key,
        dof_key_fn=_cobotta_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: cobotta_poses,
        drive_params_fn=_drive_cobotta,
        steps_per_pose=60,
        zero_uncommanded_mimic=True,
    )

    cmp_5a = _compare_trajectories(off_5a, asm_5a)
    assert cmp_5a["dof_count"] == 12, f"Cobotta DOF 数不一致: {cmp_5a['dof_count']}"
    assert cmp_5a["rb_count"] >= 13, f"Cobotta RB 数不足: {cmp_5a['rb_count']}"
    expected_mimics = {
        "left_inner_knuckle_joint": -1.0,
        "left_inner_finger_joint": 1.0,
        "right_outer_knuckle_joint": -1.0,
        "right_inner_knuckle_joint": -1.0,
        "right_inner_finger_joint": 1.0,
    }
    max_mimic_tracking_err = 0.0
    for s in asm_5a["snapshots"]:
        q_lead = s["dofs"]["finger_joint"]
        for mname, mult in expected_mimics.items():
            assert mname in s["dofs"], f"mimic joint {mname!r} が無い"
            max_mimic_tracking_err = max(max_mimic_tracking_err, abs(s["dofs"][mname] - mult * q_lead))
    cmp_5a["mimic_tracking_err_rad"] = round(max_mimic_tracking_err, 6)

    print(
        f"  Cobotta[5a OnRobot_RG6]: dofs={cmp_5a['dof_count']} rbs={cmp_5a['rb_count']} "
        f"max_pos={cmp_5a['max_link_pos_err_m']:.6f}m max_rot={cmp_5a['max_link_rot_err_deg']:.4f}deg "
        f"max_dof={cmp_5a['max_dof_err_rad']:.6f}rad mimic_err={max_mimic_tracking_err:.6f}rad",
        flush=True,
    )
    assert cmp_5a["max_link_pos_err_m"] < 0.002, f"Cobotta 5a 位置誤差超過: {cmp_5a}"
    assert cmp_5a["max_link_rot_err_deg"] < 0.5, f"Cobotta 5a 姿勢誤差超過: {cmp_5a}"
    assert cmp_5a["max_dof_err_rad"] < 0.01, f"Cobotta 5a DOF 誤差超過: {cmp_5a}"
    assert max_mimic_tracking_err < 0.02, f"Cobotta 5a mimic 追従誤差超過: {max_mimic_tracking_err}"

    # --- Subtest 5b: Gripper = "None" -> 6-DOF 素のアーム（cobotta_pro_900_arm.usda と一致） ---
    arm_only_usda = os.path.join(str(spec.output_dir), jf.parent_usda)
    arm_only_poses = [{k: v for k, v in p.items() if k != "finger_joint"} for p in cobotta_poses[:2]]

    off_5b = _simulate_position_trajectory(
        arm_only_usda,
        "/cobotta_arm_ref",
        gravity=0.0,
        variant_selections=None,
        rb_key_fn=_cobotta_rb_key,
        dof_key_fn=_cobotta_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: arm_only_poses,
        drive_params_fn=_drive_cobotta,
        steps_per_pose=50,
    )
    asm_5b = _simulate_position_trajectory(
        asm_usda,
        "/cobotta_variant_none",
        gravity=0.0,
        variant_selections={gripper_vs: "None"},
        rb_key_fn=_cobotta_rb_key,
        dof_key_fn=_cobotta_dof_key,
        pose_targets_fn=lambda _s, _aj, _m: arm_only_poses,
        drive_params_fn=_drive_cobotta,
        steps_per_pose=50,
    )

    cmp_5b = _compare_trajectories(off_5b, asm_5b)
    assert cmp_5b["dof_count"] == 6, f"Cobotta[None] DOF 数不一致: {cmp_5b['dof_count']}"
    assert cmp_5b["rb_count"] == 7, f"Cobotta[None] RB 数不一致: {cmp_5b['rb_count']}"
    print(
        f"  Cobotta[5b Gripper=None]: dofs={cmp_5b['dof_count']} rbs={cmp_5b['rb_count']} "
        f"max_pos={cmp_5b['max_link_pos_err_m']:.6f}m max_rot={cmp_5b['max_link_rot_err_deg']:.4f}deg "
        f"max_dof={cmp_5b['max_dof_err_rad']:.6f}rad",
        flush=True,
    )
    assert cmp_5b["max_link_pos_err_m"] < 0.002, f"Cobotta 5b 位置誤差超過: {cmp_5b}"
    assert cmp_5b["max_link_rot_err_deg"] < 0.5, f"Cobotta 5b 姿勢誤差超過: {cmp_5b}"
    assert cmp_5b["max_dof_err_rad"] < 0.01, f"Cobotta 5b DOF 誤差超過: {cmp_5b}"

    return {"subtest_5a_rg6": cmp_5a, "subtest_5b_gripper_none": cmp_5b}


# ====================================================================== Test 6: OpenArm Bimanual (pedestal + 2x openarm_unimanual)
@test
def test_6_openarm_bimanual():
    """OpenArm Bimanual（台座 + 2x 同一 `openarm_unimanual.usd` 再利用 USD 結合）の動作一致を検証する。

    テストの目的:
      - 同一の単腕公式 USD（`openarm_unimanual.usd`）を台座（`openarm_body_base.usda`）の左右へ
        公式マウント変換（`y = ±0.031, z = 0.698, Rx = ∓1.5708`）で 2 本結合した双腕ロボットが、
        公式一体型 `openarm_bimanual.usd` と全 22-DOF・23 リンク・両手グリッパ mimic で一致することを確認する。
      - さらに `LeftArm` / `RightArm` VariantSet による片腕構成（11-DOF）への切替も検証する。

    セットアップと関節座標対応:
      - `examples/official_assets/openarm/openarm_bimanual.toml` をビルドし、公式 `openarm_bimanual.usd` と比較する。
      - 公式 `openarm_bimanual.usd` と単腕 `openarm_unimanual.usd` の間には次の既知の関節規約差がある:
          1. `joint2`: 公式双腕は `q2=0` で腕が真下へ垂下する（左 `[-190°, +10°]`、右 `[-10°, +190°]`）のに対し、
             単腕は `q2=0` でリンク軸沿いに直線展開する（`[-100°, +100°]`）。よって厳密に
             `q2_left_off = q2_left_asm - pi/2`、`q2_right_off = q2_right_asm + pi/2` で 1:1 対応する。
          2. `right_joint7`: 公式双腕の右 `joint7` は単位回転フレーム（`+Y` 軸）だが、単腕 `joint7` は
             `Rx(pi)` 回転フレーム（`-Y` 軸）のため、`q7_right_off = -q7_right_asm` で 1:1 対応する。
          3. `openarm_link0`: 公式双腕では台座 `openarm_body_link` に埋め込まれた非 RB 形状だが、
             結合版では左右アームの固定ベース剛体として `JoinJoint` で剛結合されるため比較対象 RB から除外する。
      - Subtest 6a (`gravity=0.0`): 両腕 7 軸 + 両手グリッパ `finger_joint1` の 3 ポーズ軌道と両手 mimic 追従を比較。
      - Subtest 6b (`gravity=9.81`): 固定ベース重力下で同一 3 ポーズ軌道を比較。
      - Subtest 6c (`LeftArm` / `RightArm` 片腕 Variant): 片腕のみ装着時に 11-DOF・12 リンク・単一 ArticulationRoot になることを確認。

    検証内容と許容誤差の根拠:
      - 6a（無重力）: 運動学構造が同一のため厳格な閾値（位置 `< 2 mm`・姿勢 `< 0.5 deg`・DOF `< 0.01 rad`・
        左右グリッパ mimic 追従誤差 `< 0.002 m`、実変位 `> 0.02 m`）を課す。
      - 6b（重力 9.81）: 公式単腕と公式双腕の USD で link4〜7 の質量定義に若干の差（例: link4 が 1.37 kg vs 0.635 kg）が
        あるため、位置制御の定常重力たわみに ~2.8 mm / ~0.9° の差が生じる。これを見込んで位置 `< 5 mm`・
        姿勢 `< 1.5 deg`・DOF `< 0.02 rad` を閾値とする。
    """
    assert os.path.exists(OFFICIAL_OPENARM_BIMANUAL_USD), (
        f"公式 OpenArm Bimanual USD が見つからない: {OFFICIAL_OPENARM_BIMANUAL_USD}"
    )
    spec, _built, asm_usda = _build_official_example(
        os.path.join("openarm", "openarm_bimanual.toml"), "openarm"
    )

    # discover_components で台座と左右アームが検出できることを確認（非アクティブ variant 含む）
    st = Usd.Stage.Open(asm_usda)
    dp_path = str(st.GetDefaultPrim().GetPath())
    comps = discover_components(st, dp_path, include_inactive_variants=True)
    assert {"openarm_bimanual_assembled", "left_arm", "right_arm"}.issubset(set(comps.keys())), (
        f"discover_components が OpenArm の構成要素を検出できていない: {comps}"
    )

    def _openarm_off_rb_key(_stage: Usd.Stage, _robot_path: str, p: str) -> str:
        return p.rsplit("/", 1)[-1]

    def _openarm_asm_rb_key(_stage: Usd.Stage, robot_path: str, p: str) -> str | None:
        rel = p[len(robot_path) :].lstrip("/")
        leaf = p.rsplit("/", 1)[-1]
        # openarm_link0 は公式双腕では openarm_body_link に統合された非 RB のため比較対象から除外
        if leaf == "openarm_link0":
            return None
        for side in ("left", "right"):
            if f"/{side}_arm/" in f"/{rel}" and leaf.startswith("openarm_"):
                return f"openarm_{side}_{leaf[len('openarm_'):]}"
        return leaf

    def _openarm_off_dof_key(_stage: Usd.Stage, _robot_path: str, jpath: str) -> str:
        leaf = jpath.rsplit("/", 1)[-1]
        # 公式双腕の固定相当手首関節名は openarm_left_hand / openarm_right_hand（単腕は _hand_joint）のため揃える
        if leaf == "openarm_left_hand":
            return "openarm_left_hand_joint"
        if leaf == "openarm_right_hand":
            return "openarm_right_hand_joint"
        return leaf

    def _openarm_asm_dof_key(_stage: Usd.Stage, robot_path: str, jpath: str) -> str:
        rel = jpath[len(robot_path) :].lstrip("/")
        leaf = jpath.rsplit("/", 1)[-1]
        for side in ("left", "right"):
            if f"/{side}_arm/" in f"/{rel}" and leaf.startswith("openarm_"):
                return f"openarm_{side}_{leaf[len('openarm_'):]}"
        return leaf

    # 公式双腕座標系での 3 ポーズ（pose0: 双腕ゼロ腕姿勢+グリッパ開、pose1/2: 左右非対称フル可動姿勢）
    off_poses = [
        {
            "openarm_left_joint1": 0.0,
            "openarm_left_joint2": 0.0,
            "openarm_left_joint3": 0.0,
            "openarm_left_joint4": 0.0,
            "openarm_left_joint5": 0.0,
            "openarm_left_joint6": 0.0,
            "openarm_left_joint7": 0.0,
            "openarm_left_finger_joint1": 0.020,
            "openarm_right_joint1": 0.0,
            "openarm_right_joint2": 0.0,
            "openarm_right_joint3": 0.0,
            "openarm_right_joint4": 0.0,
            "openarm_right_joint5": 0.0,
            "openarm_right_joint6": 0.0,
            "openarm_right_joint7": 0.0,
            "openarm_right_finger_joint1": 0.030,
        },
        {
            "openarm_left_joint1": 0.4,
            "openarm_left_joint2": -0.5,
            "openarm_left_joint3": 0.6,
            "openarm_left_joint4": 0.9,
            "openarm_left_joint5": -0.5,
            "openarm_left_joint6": 0.35,
            "openarm_left_joint7": 0.5,
            "openarm_left_finger_joint1": 0.035,
            "openarm_right_joint1": -0.4,
            "openarm_right_joint2": 0.5,
            "openarm_right_joint3": -0.6,
            "openarm_right_joint4": 1.1,
            "openarm_right_joint5": 0.4,
            "openarm_right_joint6": -0.35,
            "openarm_right_joint7": -0.5,
            "openarm_right_finger_joint1": 0.015,
        },
        {
            "openarm_left_joint1": -0.8,
            "openarm_left_joint2": -1.1,
            "openarm_left_joint3": -0.7,
            "openarm_left_joint4": 1.4,
            "openarm_left_joint5": 0.7,
            "openarm_left_joint6": -0.4,
            "openarm_left_joint7": -0.7,
            "openarm_left_finger_joint1": 0.010,
            "openarm_right_joint1": 0.8,
            "openarm_right_joint2": 1.1,
            "openarm_right_joint3": 0.7,
            "openarm_right_joint4": 0.6,
            "openarm_right_joint5": -0.7,
            "openarm_right_joint6": 0.4,
            "openarm_right_joint7": 0.7,
            "openarm_right_finger_joint1": 0.040,
        },
    ]

    half_pi = math.pi / 2.0
    dof_sign_map = {"openarm_right_joint7": -1.0}
    dof_offset_map = {
        "openarm_left_joint2": -half_pi,
        "openarm_right_joint2": half_pi,
    }

    def _to_asm_pose(pose: dict[str, float]) -> dict[str, float]:
        out = dict(pose)
        out["openarm_left_joint2"] = pose["openarm_left_joint2"] + half_pi
        out["openarm_right_joint2"] = pose["openarm_right_joint2"] - half_pi
        out["openarm_right_joint7"] = -pose["openarm_right_joint7"]
        return out

    asm_poses = [_to_asm_pose(p) for p in off_poses]

    def _drive_openarm(jk: str, _jp: str):
        if "finger_joint" in jk:
            return (1e5, 1e3, float("inf"))
        return (2e4, 2e3, float("inf"))

    def _mimic_stats(sim: dict[str, Any]) -> tuple[float, float]:
        max_err = 0.0
        max_motion = 0.0
        for s in sim["snapshots"]:
            for side in ("left", "right"):
                f1 = s["dofs"][f"openarm_{side}_finger_joint1"]
                f2 = s["dofs"][f"openarm_{side}_finger_joint2"]
                max_err = max(max_err, abs(f2 - f1))
                max_motion = max(max_motion, abs(f2))
        return max_err, max_motion

    metrics: dict[str, Any] = {"discover_components": comps}

    # --- Subtest 6a (gravity=0.0) & Subtest 6b (gravity=9.81) ---
    for tag, grav, pos_tol, rot_tol, dof_tol in (
        ("subtest_6a_zero_g", 0.0, 0.002, 0.5, 0.01),
        ("subtest_6b_gravity_9_81", 9.81, 0.005, 1.5, 0.02),
    ):
        off_sim = _simulate_position_trajectory(
            OFFICIAL_OPENARM_BIMANUAL_USD,
            f"/openarm_off_{tag}",
            gravity=grav,
            variant_selections={"Physics": "PhysX"},
            rb_key_fn=_openarm_off_rb_key,
            dof_key_fn=_openarm_off_dof_key,
            pose_targets_fn=lambda _s, _aj, _m: off_poses,
            drive_params_fn=_drive_openarm,
            steps_per_pose=80,
        )
        asm_sim = _simulate_position_trajectory(
            asm_usda,
            f"/openarm_asm_{tag}",
            gravity=grav,
            variant_selections={"LeftArm": "Attached", "RightArm": "Attached"},
            rb_key_fn=_openarm_asm_rb_key,
            dof_key_fn=_openarm_asm_dof_key,
            pose_targets_fn=lambda _s, _aj, _m: asm_poses,
            drive_params_fn=_drive_openarm,
            steps_per_pose=80,
        )

        cmp = _compare_trajectories(
            off_sim,
            asm_sim,
            dof_sign_map=dof_sign_map,
            dof_offset_map=dof_offset_map,
        )
        assert cmp["dof_count"] == 22, f"OpenArm[{tag}] DOF 数不一致: {cmp['dof_count']}"
        assert cmp["rb_count"] == 23, f"OpenArm[{tag}] RB 数不一致: {cmp['rb_count']}"

        mimic_err, mimic_motion = _mimic_stats(asm_sim)
        cmp["mimic_count"] = len(asm_sim["mimic_keys"])
        cmp["max_mimic_tracking_err_m"] = round(mimic_err, 6)
        cmp["max_mimic_motion_m"] = round(mimic_motion, 5)
        assert len(asm_sim["mimic_keys"]) == 2, f"OpenArm[{tag}] mimic 関節数不一致: {asm_sim['mimic_keys']}"
        assert mimic_motion > 0.02, f"OpenArm[{tag}] グリッパ mimic が十分動いていない: {mimic_motion}"
        assert mimic_err < 0.002, f"OpenArm[{tag}] グリッパ mimic 追従誤差超過: {mimic_err}"

        print(
            f"  OpenArm[{tag} g={grav}]: dofs={cmp['dof_count']} rbs={cmp['rb_count']} "
            f"max_pos={cmp['max_link_pos_err_m']:.6f}m max_rot={cmp['max_link_rot_err_deg']:.4f}deg "
            f"max_dof={cmp['max_dof_err_rad']:.6f}rad mimic_err={mimic_err:.6f}m",
            flush=True,
        )
        assert cmp["max_link_pos_err_m"] < pos_tol, f"OpenArm[{tag}] 位置誤差超過: {cmp}"
        assert cmp["max_link_rot_err_deg"] < rot_tol, f"OpenArm[{tag}] 姿勢誤差超過: {cmp}"
        assert cmp["max_dof_err_rad"] < dof_tol, f"OpenArm[{tag}] DOF 誤差超過: {cmp}"
        metrics[tag] = cmp

    # --- Subtest 6c: 片腕 VariantSet 切替検証 (LeftArm / RightArm の単独装着) ---
    single_arm_metrics: dict[str, Any] = {}
    for label, vs_sel, side in (
        ("LeftOnly", {"LeftArm": "Attached", "RightArm": "None"}, "left"),
        ("RightOnly", {"LeftArm": "None", "RightArm": "Attached"}, "right"),
    ):
        pose_1 = [{k: v for k, v in asm_poses[1].items() if k.startswith(f"openarm_{side}_")}]
        sim_s = _simulate_position_trajectory(
            asm_usda,
            f"/openarm_{label}",
            gravity=0.0,
            variant_selections=vs_sel,
            rb_key_fn=_openarm_asm_rb_key,
            dof_key_fn=_openarm_asm_dof_key,
            pose_targets_fn=lambda _s, _aj, _m, p=pose_1: p,
            drive_params_fn=_drive_openarm,
            steps_per_pose=40,
        )
        assert sim_s["dof_count"] == 11, f"OpenArm[{label}] DOF 数不一致: {sim_s['dof_count']}"
        assert sim_s["rb_count"] == 12, f"OpenArm[{label}] RB 数不一致: {sim_s['rb_count']}"
        f1 = sim_s["snapshots"][0]["dofs"][f"openarm_{side}_finger_joint1"]
        f2 = sim_s["snapshots"][0]["dofs"][f"openarm_{side}_finger_joint2"]
        assert abs(f2 - f1) < 0.002, f"OpenArm[{label}] mimic 追従誤差超過: f1={f1}, f2={f2}"
        single_arm_metrics[label] = {
            "dof_count": sim_s["dof_count"],
            "rb_count": sim_s["rb_count"],
            "finger_mimic_err_m": round(abs(f2 - f1), 6),
        }
        print(
            f"  OpenArm[6c {label}]: dofs={sim_s['dof_count']} rbs={sim_s['rb_count']} "
            f"mimic_err={abs(f2 - f1):.6f}m",
            flush=True,
        )

    metrics["subtest_6c_single_arm_variants"] = single_arm_metrics
    return metrics


# ====================================================================== 集計・出力
summary = {
    "passed": sum(1 for r in results if r["ok"]),
    "failed": sum(1 for r in results if not r["ok"]),
    "results": results,
}
out_json = os.path.join(HERE, "last_official_motion_run.json")
with open(out_json, "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=2)
print(
    f"\n[SUMMARY] official_motion passed {summary['passed']} / failed {summary['failed']} -> {out_json}",
    flush=True,
)
sys.stdout.flush()
os._exit(1 if summary["failed"] else 0)
