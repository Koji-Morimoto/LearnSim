"""GUI の操作で Fabric のみ出力に切り替えられるかを実測する（別プロセス・画面なしで実行）。

手順: メインメニュー Window > Simulation Output Settings をクリック → ウィンドウを撮る →
"Fabric CPU" ボタンをクリック → 設定値と Fabric 拡張の状態を読む → 撮る → "USD" ボタンで戻す。

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_gui_menu.py
"""

import asyncio
import os

from isaacsim import SimulationApp

# ユーザーの GUI と同じ Isaac Sim Full の構成で起動する
_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
app = SimulationApp(
    {"headless": True, "hide_ui": False, "width": 1600, "height": 900},
    experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"),
)
import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.kit.ui_test", True)

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.kit.ui_test as ui_test  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402

# スクリーンショットの保存は日本語を含むパスだと失敗する（carb のマルチバイト変換エラー）ため、英数字だけのパスに出す
OUT = os.environ.get("SHOT_DIR", os.path.join(os.environ.get("TEMP", "C:/Temp"), "fabric_only_shots"))
os.makedirs(OUT, exist_ok=True)
KEYS = [
    "/physics/updateToUsd",
    "/physics/updateVelocitiesToUsd",
    "/physics/fabricEnabled",
    "/physics/fabricUpdateTransformations",
    "/physics/fabricUseGPUInterop",
]


def state(label):
    s = carb.settings.get_settings()
    em = omni.kit.app.get_app().get_extension_manager()
    print(f"--- {label}")
    for k in KEYS:
        print(f"  {k} = {s.get(k)}")
    print(f"  omni.physx.fabric enabled = {em.is_extension_enabled('omni.physx.fabric')}")


async def capture(name):
    from omni.kit.renderer.capture import acquire_renderer_capture_interface

    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    path = os.path.join(OUT, name)
    cap = acquire_renderer_capture_interface()
    cap.capture_next_frame_swapchain(path)
    for _ in range(5):
        await omni.kit.app.get_app().next_update_async()
    cap.wait_async_capture()
    print("saved", path, os.path.exists(path))


def dump_viewport_physx_menu():
    """Viewport 上部の PhysX メニューの項目名を出す（読むだけ）。"""
    win = ui.Workspace.get_window("Viewport")

    def walk(w, path):
        for c in ui.Inspector.get_children(w):
            t = getattr(c, "text", "") or ""
            p = path + "/" + t if t else path
            if isinstance(c, (ui.Menu, ui.MenuItem)) and "PhysX" in p:
                print("  viewport menu:", p.split("PhysX", 1)[1] or "(PhysX)", type(c).__name__)
            walk(c, p if t else path)

    walk(win.frame, "")


async def main():
    omni.usd.get_context().new_stage()
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    dump_viewport_physx_menu()
    state("初期状態")

    await ui_test.menu_click("Window/Simulation Output Settings")
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()
    win = ui.Workspace.get_window("Simulation Output Settings")
    print("window:", win, "visible:", getattr(win, "visible", None))
    if win:
        pass  # 位置は動かさない（動かすとクリック座標がずれた）
        win.focus()
    await capture("1_menu_opened.png")

    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()
    btn = ui_test.find("Simulation Output Settings//Frame/**/Button[*].text=='Fabric CPU'")
    w = btn.widget
    print("Fabric CPU button: ref.position", btn.position, "screen_position", (w.screen_position_x, w.screen_position_y),
          "computed", (w.computed_width, w.computed_height), "window pos", (win.position_x, win.position_y),
          "dpi", ui.Workspace.get_dpi_scale())
    await btn.click()
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    state("Fabric CPU ボタンを押した後")
    await capture("2_fabric_cpu_pressed.png")

    btn = ui_test.find("Simulation Output Settings//Frame/**/Button[*].text=='USD'")
    print("USD button pos:", btn.position)
    await btn.click()
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    state("USD ボタンを押した後")
    await capture("3_usd_pressed.png")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
print("===== END =====")
app.close()
