"""Viewport の Display > Show By Type にある Physics > Colliders の項目が、どの設定を切り替えるかを実測する。

Show By Type の下の Menu / MenuItem をすべて書き出し、Colliders の下の None / Selected / All の項目を
選択状態にしたとき /persistent/physics/visualizationDisplayColliders（omni.physx の SETTING_DISPLAY_COLLIDERS）の
値が変わるか、逆に設定値を書いたとき項目の選択状態が変わるかを読む。最後に設定値を初期値に戻す
（この設定は保存される設定で、ユーザーの GUI と同じ設定ファイルに書かれるため）。

実行（このリポジトリのルートで）: <Isaac Sim のフォルダ>/python.bat 2026-11-25_FabricOnly切替/verify_show_by_type_menu.py
"""

import asyncio
import os

from isaacsim import SimulationApp

_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
app = SimulationApp(
    {"headless": True, "hide_ui": False, "width": 1600, "height": 900},
    experience=os.path.join(_ROOT, "apps", "isaacsim.exp.full.kit"),
)
import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.kit.ui_test", True)

import carb.settings  # noqa: E402
import omni.kit.ui_test as ui_test  # noqa: E402
import omni.physx.bindings._physx as pxb  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402

KEY = pxb.SETTING_DISPLAY_COLLIDERS


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def collect(widget, path, out):
    for c in ui.Inspector.get_children(widget):
        t = getattr(c, "text", "") or ""
        p = f"{path}/{t}" if t else path
        if isinstance(c, (ui.Menu, ui.MenuItem)):
            out.append((p, c))
        collect(c, p, out)


async def main():
    omni.usd.get_context().new_stage()
    await frames(30)
    print("setting key:", KEY, "初期値:", carb.settings.get_settings().get(KEY))

    win = ui.Workspace.get_window("Viewport")
    items = []
    collect(win.frame, "", items)
    sbt = [(p, w) for p, w in items if "Show By Type" in p]
    print("== Show By Type の下の項目")
    for p, w in sbt:
        print(" ", p.split("Show By Type", 1)[1] or "(Show By Type)", type(w).__name__)

    initial = carb.settings.get_settings().get(KEY)
    radio = {p.split("/Colliders/", 1)[1]: w for p, w in sbt if "/Physics/Colliders/" in p and isinstance(w, ui.MenuItem)}
    print("== Colliders の項目の model を True にして（クリックと同じ選択状態にして）設定値を読む")
    for name in ("None", "Selected", "All"):
        w = radio[name]
        w.model.set_value(True)
        await frames(5)
        print(f"  {name} を選択 -> {KEY} = {carb.settings.get_settings().get(KEY)}")
    print("== 逆向き: 設定値を書いたときに項目の選択状態が変わるか")
    for v in (0, 1, 2):
        carb.settings.get_settings().set_int(KEY, v)
        await frames(5)
        print(f"  {KEY} = {v} -> 選択状態", {n: radio[n].model.get_value_as_bool() for n in ("None", "Selected", "All")})
    carb.settings.get_settings().set(KEY, initial)  # 共有の設定ファイルに残らないよう初期値に戻す
    await frames(5)
    print("初期値に戻した:", carb.settings.get_settings().get(KEY))


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    import traceback

    traceback.print_exception(task.exception())
print("===== END =====")
app.close()
