r"""`myext.robot.assembly_pipeline` のコマンドライン実行エントリポイント。

責務:
  - TOML 仕様ファイルを受け取り、4 つの実行モード（`--check` / `--gui` / `--build-only` / フル検証）を切り替える。
  - Isaac Sim (`SimulationApp`) は `AGENTS.md` 必須ルールに従い、必ず
    `C:\isaacsim-6.2.0\tools\myext\sim_app_launcher.py` の `create_simulation_app` 経由で起動する。
  - `--gui` モードでは `build_assembly` による非破壊生成のみを行い（公式 `RobotAssembler` の不具合再現を行わない）、
    組み立てたロボットを `/World/<stem>` にスポーンして操作可能な状態で GUI を維持する。

使い方:
    # 1. GUI で組み立てたロボットを確認・操作する（おすすめ）
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py <仕様.toml> --gui
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py <仕様.toml> --gui --target arm_unit.usda

    # 2. 仕様の静的検査だけ行う（Isaac Sim を起動しない・数秒）
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py <仕様.toml> --check

    # 3. Headless で .usda の生成だけ行う（高速・公式比較なし・エラーログなし）
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py <仕様.toml> --build-only

    # 4. Headless で .usda 生成 + 物理 / RobotPoser / 順運動学検証 + レポート出力を行う
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py <仕様.toml>

    # 5. TOML 補完・検証用の JSON Schema を再生成する（Isaac Sim を起動しない）
    C:\isaacsim-6.2.0\tools\myext\run_sim.bat run_pipeline.py --export-schema

終了コード:
    0 = 正常終了
    1 = 検証で問題あり（修正後も残った症状、判定外の症状、または順運動学の不一致）
    2 = 事前チェックの規則判定（`on_violation = "stop"`）で停止
    3 = 実行時例外
    4 = TOML 仕様またはコマンドライン引数の誤り
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import traceback
from pathlib import Path
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_ROOT = Path(os.environ.get("ISAAC_SIM_DIR", "C:/isaacsim-6.2.0"))
LAUNCHER_DIR = ISAAC_ROOT / "tools" / "myext"


def _bootstrap_python_paths() -> None:
    """Isaac Sim 起動前（`--check` 含む）でも仕様検査が動くよう `sys.path` を整える。

    環境変数 `PYTHONPATH` に外部の USD パッケージ（`BuiltOpenUSD` 等）や別環境の
    `sitecustomize.py` が入っていると、Isaac Sim 標準の `numpy` や `pxr` が隠されることがある。
    そのため、この拡張のルートを先頭に追加しつつ、必要なら Isaac Sim 同梱の
    `omni.kit.pip_archive*/pip_prebundle` を補う。
    """
    # BuiltOpenUSD が入っていると Kit 起動後に Isaac Sim の pxr と衝突するため除外する
    sys.path[:] = [p for p in sys.path if "builtopenusd" not in p.lower()]
    if HERE not in sys.path:
        sys.path.insert(0, HERE)

    try:
        import numpy  # noqa: F401
    except ModuleNotFoundError:
        pattern = str(ISAAC_ROOT / "extscache" / "omni.kit.pip_archive*" / "pip_prebundle")
        for prebundle in sorted(glob.glob(pattern)):
            if os.path.isdir(prebundle) and prebundle not in sys.path:
                sys.path.append(prebundle)


def _build_parser() -> argparse.ArgumentParser:
    """コマンドライン引数パーサを構築する。"""
    parser = argparse.ArgumentParser(
        prog="run_pipeline.py",
        description=(
            "URDF / USD ロボット結合パイプライン (`myext.robot.assembly_pipeline`) の CLI。\n"
            "GUI で組み立てたロボットを確認したい場合は `--gui` を指定してください。"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "spec",
        nargs="?",
        default=None,
        help="入力する TOML 仕様ファイルのパス（例: examples\\multi_arm.toml）。--export-schema 時は不要。",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--check",
        action="store_true",
        help="Isaac Sim を起動せず、TOML 仕様とリンク・VariantSet の整合性だけを静的検査して終了する。",
    )
    mode.add_argument(
        "--export-schema",
        action="store_true",
        help="Isaac Sim を起動せず、schema\\assembly_pipeline.schema.json を最新仕様から再生成して終了する。",
    )
    mode.add_argument(
        "--gui",
        action="store_true",
        help=(
            "Isaac Sim GUI を起動して .usda を非破壊生成し、組み立て済みロボットを /World/<stem> に\n"
            "配置して画面を開いたまま維持する（公式比較の不具合再現は実行しない）。"
        ),
    )
    mode.add_argument(
        "--build-only",
        action="store_true",
        help="Headless で .usda の非破壊生成のみを実行して終了する（検証プローブ・公式比較なし）。",
    )
    parser.add_argument(
        "--target",
        metavar="USDA",
        default=None,
        help=(
            "`--gui` モード時にステージへ配置する .usda のファイル名（例: arm_unit.usda）。\n"
            "省略時は依存順で最後の結合ファイル（`join_files` が無ければ最後の部品 .usda）を使う。"
        ),
    )
    return parser


def resolve_target_usda(spec: Any, target_usda: str | None = None) -> str:
    """`--gui` モードでステージに配置する `.usda` ファイル名を決定・検証する。

    引数:
      spec: `load_spec` が返した `PipelineSpec`。
      target_usda: `--target` で指定された `.usda` ファイル名。`None` なら既定の最終出力を選ぶ。

    戻り値:
      `spec.all_usda_names` に含まれる `.usda` ファイル名。

    例外:
      ValueError: 指定されたファイル名が仕様内に存在しない場合。
    """
    available = list(spec.all_usda_names)
    if not available:
        raise ValueError("仕様に生成対象の .usda が定義されていません")

    if target_usda is not None:
        if target_usda not in available:
            raise ValueError(
                f"--target {target_usda!r} は仕様に存在しません。指定できる候補: {available}"
            )
        return target_usda

    ordered_joins = spec.file_order()
    if ordered_joins:
        return ordered_joins[-1].usda
    return available[-1]


def check_only(spec_path: str, target_usda: str | None = None):
    """Isaac Sim を起動せずに TOML 仕様とリンク・VariantSet の整合性を静的検査する。

    引数:
      spec_path: TOML 仕様ファイルの絶対パス。
      target_usda: `--target` の指定値（指定時のみ仕様内の存在チェックを行う）。

    戻り値:
      検査に合格した場合は `PipelineSpec`、仕様に誤りがある場合は `None`。
    """
    _bootstrap_python_paths()
    from myext.robot.assembly_pipeline.spec import load_spec
    from myext.robot.assembly_pipeline.spec_check import check_links, describe

    try:
        spec = load_spec(spec_path)
    except ValueError as exc:
        print(f"[SPEC ERROR] {exc}", flush=True)
        return None
    except OSError as exc:
        print(f"[SPEC ERROR] TOML ファイルを開けません: {exc}", flush=True)
        return None

    problems = check_links(spec)
    for p in problems:
        print(f"[SPEC ERROR] {p}", flush=True)
    if problems:
        return None

    if target_usda is not None:
        try:
            resolve_target_usda(spec, target_usda)
        except ValueError as exc:
            print(f"[SPEC ERROR] {exc}", flush=True)
            return None

    desc = describe(spec)
    print(desc if isinstance(desc, str) else "\n".join(desc), flush=True)
    return spec


def _enable_pipeline_extension(*, enable_physics_ui: bool = False) -> None:
    """Kit の拡張マネージャで `myext.robot.assembly_pipeline` を有効化する。

    GUI モード（`enable_physics_ui=True`）では、`SimulationApp` 既定の軽量アプリ
    `isaacsim.exp.base.python.kit` に含まれない `omni.kit.property.physics` も有効化し、
    Property パネルに関節の Physics > Drive（Angular / Linear）等の専用 UI を表示する。
    """
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    mgr.add_path(os.path.dirname(HERE))
    if not mgr.is_extension_enabled("myext.robot.assembly_pipeline"):
        mgr.set_extension_enabled_immediate("myext.robot.assembly_pipeline", True)
    if enable_physics_ui and not mgr.is_extension_enabled("omni.kit.property.physics"):
        mgr.set_extension_enabled_immediate("omni.kit.property.physics", True)


def _collect_variants(stage: Any, root_prim_path: str) -> dict[str, dict[str, dict[str, Any]]]:
    """`root_prim_path` 配下の全 prim から VariantSet 名・現在選択・全候補を収集する。"""
    from pxr import Usd

    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim or not root_prim.IsValid():
        return {}

    variants_by_prim: dict[str, dict[str, dict[str, Any]]] = {}
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsDefined():
            continue
        vsets = prim.GetVariantSets()
        names = vsets.GetNames()
        if not names:
            continue
        entry: dict[str, dict[str, Any]] = {}
        for vs_name in names:
            vs = vsets.GetVariantSet(vs_name)
            entry[vs_name] = {
                "selected": vs.GetVariantSelection(),
                "choices": list(vs.GetVariantNames()),
            }
        if entry:
            variants_by_prim[str(prim.GetPath())] = entry
    return variants_by_prim


def setup_gui_stage(
    spec: Any,
    built: dict[str, Any],
    target_usda: str | None = None,
    *,
    frame_camera: bool = True,
) -> dict[str, Any]:
    """GUI 表示用の新しい USD Stage を用意し、組み立て済みロボットを `/World/<stem>` にスポーンする。

    `--gui` モード本体だけでなく、Headless 環境でのスモークテストからも直接呼び出せるよう
    独立した関数として分離している。

    引数:
      spec: `load_spec` が返した `PipelineSpec`。
      built: `build_assembly(spec)` の戻り値辞書。
      target_usda: スポーンする `.usda` のファイル名（省略時は最終結合ファイル）。
      frame_camera: True の場合、アクティブな Viewport カメラをロボット全体に合わせてフレーミングする。

    戻り値:
      `target_usda`, `usda_path`, `spawn_path`, `descriptor`, `variants_by_prim` を格納した辞書。

    副作用:
      現在の `omni.usd.get_context()` に新規ステージを作成し、PhysicsScene / GroundPlane /
      DistantLight / DomeLight / スポーン済みロボットを配置して選択状態にする。
    """
    import omni.usd
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    from myext.robot.assembly_pipeline import runtime, spawn_assembled_robot

    target_name = resolve_target_usda(spec, target_usda)
    usda_path = built["outputs"][target_name]
    stem = target_name[: -len(".usda")]
    spawn_path = f"/World/{stem}"

    # 既存の検証用・作業用ステージをリセットしてクリーンな GUI 用ステージを作る
    runtime.stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    runtime.step(2)
    stage = usd_ctx.get_stage()

    # ステージ単位を Isaac Sim 標準の Z-up / 1.0 m/unit に揃える
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world_prim)

    # 物理シーン（CPU PhysX・重力 9.81 m/s^2）
    physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_api.CreateEnableGPUDynamicsAttr(False)
    physx_api.CreateBroadphaseTypeAttr("MBP")

    # 地面（物理衝突 + 可視メッシュ）
    PhysicsSchemaTools.addGroundPlane(
        stage,
        "/World/GroundPlane",
        "Z",
        50.0,
        Gf.Vec3f(0.0, 0.0, 0.0),
        Gf.Vec3f(0.45, 0.45, 0.45),
    )

    # 照明（主光源の DistantLight + 影が真っ暗にならないよう補助の DomeLight）
    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(distant.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 30.0))

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(600.0)

    # 組み立て済み .usda を Reference として /World/<stem> にスポーン
    descriptor = spawn_assembled_robot(stage, usda_path, spawn_path)
    runtime.step(4)

    variants_by_prim = _collect_variants(stage, spawn_path)

    # Stage 上でロボットルートを選択しておき、Property パネルからすぐ VariantSet 等を触れるようにする
    try:
        usd_ctx.get_selection().set_selected_prim_paths([spawn_path], True)
    except Exception:  # noqa: BLE001
        pass

    # GUI 起動時はロボット全体が画面に収まるようにカメラを合わせる
    if frame_camera:
        try:
            import omni.kit.viewport.utility as vp_util

            viewport_api = vp_util.get_active_viewport()
            if viewport_api is not None:
                vp_util.frame_viewport_prims(viewport_api=viewport_api, prims=[spawn_path])
                runtime.step(2)
        except Exception:  # noqa: BLE001
            # Headless スモークテストなど Viewport が存在しない環境ではカメラ調整だけ飛ばす
            pass

    return {
        "target_usda": target_name,
        "usda_path": usda_path,
        "spawn_path": spawn_path,
        "descriptor": descriptor,
        "variants_by_prim": variants_by_prim,
    }


def _clean_components(
    stage: Any | None,
    root_prim_path: str,
    descriptor: Any,
    usda_path: str | None,
) -> list[tuple[str, str, bool]]:
    """表示用に構成要素一覧を `(コンポーネント名, prim パス, 現在アクティブか)` の順で整える。

    `descriptor.components` は参照先レイヤの `customLayerData` を再帰的にマージするため、
    親レイヤに上書きを持つ子 prim があると重複パスが混ざることがある。
    ここでは「ステージ上で現在存在する構成要素」と「対象 `.usda` の `customLayerData` に
    記録された非アクティブ Variant の構成要素」を整理して返す。
    """
    from pxr import Sdf

    from myext.robot.assembly_pipeline import discover_components
    from myext.robot.assembly_pipeline.descriptor import CUSTOM_LAYER_KEY

    active: dict[str, str] = {}
    if stage is not None:
        active = discover_components(stage, root_prim_path, include_inactive_variants=False)

    items: dict[str, tuple[str, bool]] = {name: (path, True) for name, path in active.items()}

    if usda_path:
        layer = Sdf.Layer.FindOrOpen(usda_path)
        cld = (layer.customLayerData or {}) if layer else {}
        block = cld.get(CUSTOM_LAYER_KEY)
        if isinstance(block, dict) and isinstance(block.get("components"), dict):
            for name, rel in block["components"].items():
                if not isinstance(name, str) or not isinstance(rel, str):
                    continue
                if rel == ".":
                    full_path = root_prim_path
                else:
                    full_path = f"{root_prim_path}/{rel.lstrip('/')}"
                if name not in items:
                    exists = bool(stage is not None and stage.GetPrimAtPath(full_path).IsValid())
                    items[name] = (full_path, exists)

    if not items and descriptor is not None:
        for name, path in descriptor.components.items():
            items[name] = (path, True)

    return [(name, path, is_active) for name, (path, is_active) in items.items()]


def _print_gui_summary(spec: Any, info: dict[str, Any]) -> None:
    """`--gui` モード起動後のスポーン情報・コンポーネント・VariantSet・操作ヒントを表示する。"""
    import omni.usd

    desc = info["descriptor"]
    variants_by_prim: dict[str, dict[str, dict[str, Any]]] = info["variants_by_prim"]
    stage = omni.usd.get_context().get_stage()
    components = _clean_components(stage, info["spawn_path"], desc, info["usda_path"])

    print("", flush=True)
    print("========================================================================", flush=True)
    print("[GUI] 組み立て済みロボットを Isaac Sim GUI に配置しました", flush=True)
    print("========================================================================", flush=True)
    print(f"  出力先フォルダ      : {spec.output_dir}", flush=True)
    print(f"  配置した usda       : {info['target_usda']} ({info['usda_path']})", flush=True)
    print(f"  スポーン先 prim     : {info['spawn_path']}", flush=True)
    print(f"  Articulation Root   : {desc.articulation_root_path or '(なし)'}", flush=True)

    print("", flush=True)
    print("  [コンポーネント一覧]", flush=True)
    if components:
        for comp_name, comp_path, is_active in components:
            suffix = "" if is_active else "  (現在は非アクティブ Variant)"
            print(f"    - {comp_name:<22} -> {comp_path}{suffix}", flush=True)
    else:
        print("    (なし)", flush=True)

    print("", flush=True)
    print("  [VariantSet 一覧]", flush=True)
    if variants_by_prim:
        for prim_path, vmap in variants_by_prim.items():
            for vs_name, vinfo in vmap.items():
                choices = ", ".join(vinfo["choices"])
                print(
                    f"    - {prim_path} : {vs_name} = {vinfo['selected']!r} (候補: [{choices}])",
                    flush=True,
                )
    else:
        print("    (なし)", flush=True)

    print("", flush=True)
    print("  [操作ヒント]", flush=True)
    print("    1. 左ツールの Play ボタン（または Space キー）で物理シミュレーションを開始できます。", flush=True)
    print("    2. Stage パネルで対象 prim を選び、Property パネルの Variants から選択肢を切り替えられます。", flush=True)
    print("    3. 各関節の Drive Target Position / Velocity を Property パネルから変更して動かせます。", flush=True)
    print("    4. 終了するときは Isaac Sim ウィンドウを閉じるか、この端末で Ctrl+C を押してください。", flush=True)
    print("========================================================================", flush=True)


def _print_build_only_summary(spec: Any, built: dict[str, Any], spec_path: str) -> None:
    """`--build-only` モードの生成結果と次の `--gui` 起動例を表示する。"""
    from pxr import Usd

    target_name = resolve_target_usda(spec, None)
    target_path = built["outputs"][target_name]
    print("", flush=True)
    print("========================================================================", flush=True)
    print(f"[BUILD] .usda の非破壊生成が完了しました: {spec.output_dir}", flush=True)
    print("========================================================================", flush=True)
    for name, path in built["outputs"].items():
        marker = " (最終出力)" if name == target_name else ""
        print(f"  - {name:<28} -> {path}{marker}", flush=True)

    desc = built["descriptors"].get(target_name)
    stage = Usd.Stage.Open(target_path)
    dp = stage.GetDefaultPrim() if stage else None
    root_path = str(dp.GetPath()) if dp and dp.IsValid() else f"/{target_name[:-5]}"
    components = _clean_components(stage, root_path, desc, target_path)
    if components:
        print("", flush=True)
        print(f"  [{target_name} のコンポーネント一覧]", flush=True)
        for comp_name, comp_path, is_active in components:
            suffix = "" if is_active else "  (現在は非アクティブ Variant)"
            print(f"    - {comp_name:<22} -> {comp_path}{suffix}", flush=True)

    print("", flush=True)
    print("  GUI で確認・操作したい場合:", flush=True)
    print(
        f'    C:\\isaacsim-6.2.0\\tools\\myext\\run_sim.bat "{os.path.abspath(__file__)}" "{spec_path}" --gui',
        flush=True,
    )
    print("========================================================================", flush=True)


def main(argv: list[str] | None = None) -> int:
    """CLI のメインエントリポイント。終了コードを返す。"""
    parser = _build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        # `--help` は 0、引数誤りは 4 に統一する
        return 0 if exc.code == 0 else 4

    if args.export_schema:
        _bootstrap_python_paths()
        from myext.robot.assembly_pipeline.schema import verify_schema_sync, write_json_schema

        out_path = write_json_schema()
        sync_errs = verify_schema_sync()
        if sync_errs:
            for e in sync_errs:
                print(f"[SCHEMA ERROR] {e}", flush=True)
            return 4
        print(f"[SCHEMA] 生成完了: {out_path}", flush=True)
        return 0

    if not args.spec:
        print("[SPEC ERROR] TOML 仕様ファイルのパスを指定してください（--help で使い方を確認）", flush=True)
        return 4

    if args.target is not None and not args.gui:
        print("[SPEC ERROR] --target は --gui モードでのみ指定できます", flush=True)
        return 4

    spec_path = os.path.abspath(args.spec)
    checked_spec = check_only(spec_path, target_usda=args.target)
    if checked_spec is None:
        return 4

    if args.check:
        print("[CHECK] 仕様に誤りは無い", flush=True)
        return 0

    # AGENTS.md 必須ルール: SimulationApp は必ず sim_app_launcher.create_simulation_app で起動する
    launcher_str = str(LAUNCHER_DIR)
    if launcher_str not in sys.path:
        sys.path.insert(0, launcher_str)
    from sim_app_launcher import create_simulation_app

    is_gui = bool(args.gui)
    if is_gui:
        print(
            "[MODE] GUI モードで Isaac Sim を起動します（.usda を非破壊生成し、ロボットを配置して画面を維持します）",
            flush=True,
        )
    elif args.build_only:
        print(
            "[MODE] Headless ビルド専用モード（--build-only）で実行します（公式比較・動的検証なし）",
            flush=True,
        )
    else:
        print(
            "[MODE] Headless フル検証モードで実行します（.usda 生成 + 物理 / RobotPoser / 順運動学検証 + レポート出力）",
            flush=True,
        )
        print(
            "[INFO] GUI でロボットを表示・操作したい場合は `--gui` を付けて実行してください。",
            flush=True,
        )
        if checked_spec.compare_official:
            print(
                "[INFO] この仕様は compare_official = true です。途中の公式 RobotAssembler 比較フェーズでは\n"
                "       公式ツールの既知不具合を意図的に再現するため PhysX の closed articulation エラー等が\n"
                "       出力されますが、最終行が `[RESULT] ok` であれば生成された .usda 自体は正常です。",
                flush=True,
            )

    app = create_simulation_app({"headless": not is_gui})
    code = 3
    try:
        _enable_pipeline_extension(enable_physics_ui=is_gui)
        from myext.robot.assembly_pipeline import (
            PipelineStopped,
            build_assembly,
            load_spec,
            run_pipeline,
        )

        spec = load_spec(spec_path)
        try:
            if is_gui:
                # GUI モードは公式不具合の再現を行わない非破壊ビルドのみ実行する
                built = build_assembly(spec, check_idempotency=False)
                info = setup_gui_stage(spec, built, target_usda=args.target, frame_camera=True)
                _print_gui_summary(spec, info)
                try:
                    while app.is_running():
                        app.update()
                except KeyboardInterrupt:
                    print("\n[GUI] Ctrl+C を受け取ったため終了します。", flush=True)
                code = 0
            elif args.build_only:
                built = build_assembly(spec, check_idempotency=False)
                _print_build_only_summary(spec, built, spec_path)
                code = 0
            else:
                result = run_pipeline(spec)
                rep = result["report"] if isinstance(result, dict) else result.report
                report_md = os.path.join(str(spec.output_dir), "report.md")
                print(f"[RESULT] {rep['status']}: {report_md}", flush=True)
                if rep["status"] == "ok":
                    if spec.compare_official:
                        print(
                            "[INFO] 生成された .usda の検証はすべて正常です"
                            "（途中で出た公式比較フェーズの PhysX エラーは公式不具合の再現ログです）。",
                            flush=True,
                        )
                    code = 0
                else:
                    print(
                        f"[ERROR] 検証で問題が検出されました。詳細は {report_md} を確認してください。",
                        flush=True,
                    )
                    code = 1
        except PipelineStopped as exc:
            print(f"[STOPPED] {exc}", flush=True)
            code = 2
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        code = 3
    finally:
        sys.stdout.flush()
        if is_gui:
            try:
                app.close()
            except Exception:  # noqa: BLE001
                pass
    return code


if __name__ == "__main__":
    # Headless モードでは Kit の終了待ちハングを避けるため os._exit で終了コードを確実に返す
    exit_code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
