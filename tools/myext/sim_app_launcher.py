"""SimulationApp を「必須設定」付きで起動するヘルパー.

必須設定 (AGENTS.md の "SimulationApp 起動ルール" 参照):
  1. Viewport の Heads Up Display > FPS を表示する
  2. 自作拡張 myext.viewport.axissnap / myext.coi_marker を有効化する
  3. コンソールを UTF-8 にする (Windows 既定の CP932/ShiftJIS だと日本語が化ける)

使い方 (スクリプトの先頭、他の omni / isaacsim import より前):

    import sys
    sys.path.insert(0, r"C:/isaacsim-6.2.0/tools/myext")
    from sim_app_launcher import create_simulation_app

    simulation_app = create_simulation_app({"headless": False})
    ...
    simulation_app.close()

実行は tools/myext/run_sim.bat 経由を推奨 (chcp 65001 + PYTHONUTF8=1 を設定して python.bat を呼ぶ)。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Iterable

ISAAC_ROOT = Path(__file__).resolve().parents[2]
USER_EXT_FOLDER = ISAAC_ROOT / "extsUser" / "Extentions"

REQUIRED_EXTENSIONS: tuple[str, ...] = (
    "myext.viewport.axissnap",
    "myext.coi_marker",
)

# Display メニュー > Heads Up Display > FPS が参照する設定 (omni.kit.viewport.window / menubar.display)
_VP_KEY = "/persistent/app/viewport/Viewport/Viewport0"
HUD_SETTINGS: dict[str, bool] = {
    f"{_VP_KEY}/hud/visible": True,
    f"{_VP_KEY}/hud/renderFPS/visible": True,
    "/app/viewport/forceHideFps": False,
}
_DISPLAY_OPTIONS_FPS_BIT = 1 << 0  # legacy displayOptions の bit0 = FPS


def force_utf8_console() -> None:
    """コンソール入出力を UTF-8 にする (Kit のネイティブログ出力にも効く)."""
    os.environ.setdefault("PYTHONUTF8", "1")
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    if os.name == "nt":
        try:
            import ctypes

            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
            ctypes.windll.kernel32.SetConsoleCP(65001)
        except Exception:
            pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def _bool_arg(value: bool) -> str:
    return "true" if value else "false"


def build_config(config: dict[str, Any] | None = None, extra_extensions: Iterable[str] = ()) -> dict[str, Any]:
    """SimulationApp に渡す config に必須設定をマージして返す."""
    cfg = dict(config or {})

    # SimulationApp の既定 display_options=3094 は FPS(bit0) が OFF なので必ず立てる
    cfg["display_options"] = int(cfg.get("display_options", 3094)) | _DISPLAY_OPTIONS_FPS_BIT

    args = list(cfg.get("extra_args", []))
    args += ["--ext-folder", USER_EXT_FOLDER.as_posix()]
    args += [f"--{key}={_bool_arg(val)}" for key, val in HUD_SETTINGS.items()]
    for ext in (*REQUIRED_EXTENSIONS, *extra_extensions):
        args += ["--enable", ext]
    cfg["extra_args"] = args
    return cfg


def apply_runtime_requirements(extra_extensions: Iterable[str] = ()) -> None:
    """起動後に HUD FPS 設定を再適用し、必須拡張が有効か検証する (無効なら有効化を試みる)."""
    import carb
    import omni.kit.app

    settings = carb.settings.get_settings()
    for key, val in HUD_SETTINGS.items():
        settings.set(key, val)
    legacy = settings.get("/persistent/app/viewport/displayOptions")
    if isinstance(legacy, int) and not legacy & _DISPLAY_OPTIONS_FPS_BIT:
        settings.set("/persistent/app/viewport/displayOptions", legacy | _DISPLAY_OPTIONS_FPS_BIT)

    manager = omni.kit.app.get_app().get_extension_manager()
    missing = []
    for ext in (*REQUIRED_EXTENSIONS, *extra_extensions):
        if not manager.is_extension_enabled(ext):
            manager.set_extension_enabled_immediate(ext, True)
        if not manager.is_extension_enabled(ext):
            missing.append(ext)
    if missing:
        raise RuntimeError(f"必須拡張を有効化できませんでした: {missing} (ext folder: {USER_EXT_FOLDER})")


def create_simulation_app(config: dict[str, Any] | None = None, extra_extensions: Iterable[str] = ()):
    """必須設定付きで SimulationApp を生成して返す."""
    force_utf8_console()
    from isaacsim import SimulationApp

    extra_extensions = tuple(extra_extensions)
    app = SimulationApp(build_config(config, extra_extensions))
    apply_runtime_requirements(extra_extensions)
    app.update()
    return app
