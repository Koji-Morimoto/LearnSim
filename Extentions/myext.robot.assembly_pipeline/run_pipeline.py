"""コマンドラインから実行する。

    C:\\isaacsim-6.2.0\\python.bat run_pipeline.py <仕様.toml>            実行する(headless の Isaac Sim を起動する)
    C:\\isaacsim-6.2.0\\python.bat run_pipeline.py <仕様.toml> --check    仕様を検査して、参照関係の一覧を表示するだけ(Isaac Sim を起動しない。数秒)

どちらも、Isaac Sim を起動する前に仕様を検査する(TOML のキー、usda のファイル名の参照、リンク名、子の VariantSet の選択)。
終了コード: 0 = 問題なし、1 = 修正後も起きた症状か判定外の症状がある、2 = 判定で止めた、3 = 例外、4 = 仕様の誤り
報告は <output_dir>/report.md と report.json。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def check_only(spec_path: str):
    """Isaac Sim なしで仕様を読んで検査する。問題があれば表示して None。"""
    sys.path.insert(0, HERE)
    from myext.robot.assembly_pipeline.spec import load_spec
    from myext.robot.assembly_pipeline.spec_check import check_links, describe

    try:
        spec = load_spec(spec_path)
    except ValueError as exc:
        print(f"[SPEC ERROR] {exc}", flush=True)
        return None
    problems = check_links(spec)
    for p in problems:
        print(f"[SPEC ERROR] {p}", flush=True)
    if problems:
        return None
    print("\n".join(describe(spec)), flush=True)
    return spec


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        return 3
    spec_path = os.path.abspath(args[0])
    if check_only(spec_path) is None:
        return 4
    if "--check" in sys.argv:
        print("[CHECK] 仕様に誤りは無い", flush=True)
        return 0
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})  # noqa: F841
    code = 3
    try:
        import omni.kit.app

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.add_path(os.path.dirname(HERE))
        mgr.set_extension_enabled_immediate("myext.robot.assembly_pipeline", True)
        from myext.robot.assembly_pipeline import PipelineStopped, load_spec, run_pipeline

        spec = load_spec(spec_path)
        try:
            result = run_pipeline(spec)
            print(f"[RESULT] {result.report['status']}: {os.path.join(spec.output_dir, 'report.md')}", flush=True)
            code = 0 if result.report["status"] == "ok" else 1
        except PipelineStopped as exc:
            print(f"[STOPPED] {exc}", flush=True)
            code = 2
    except Exception:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        code = 3
    finally:
        sys.stdout.flush()
    return code


if __name__ == "__main__":
    os._exit(main())  # Kit の終了処理を待たずに、終了コードを確実に返す
