"""コマンドラインから実行する(headless の Isaac Sim を別プロセスで起動する)。

    C:\\isaacsim-6.2.0\\python.bat run_pipeline.py <仕様.toml>

終了コード: 0 = 問題なし、1 = 修正後も起きた症状か判定外の症状がある、2 = 判定で止めた、3 = 例外
報告は <output_dir>/report.md と report.json。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 3
    spec_path = os.path.abspath(sys.argv[1])
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
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
    os._exit(code)  # Kit の終了処理を待たずに、終了コードを確実に返す


if __name__ == "__main__":
    os._exit(main())
