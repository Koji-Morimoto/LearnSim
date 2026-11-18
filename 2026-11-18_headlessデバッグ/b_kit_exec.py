# 2026-11-18  やり方 B: headless で起動した Kit アプリに後から接続して止める
# 起動は同じフォルダの run_b.ps1 で行う（Isaac Sim の場所・ポートはそちらで指定する）
# 起動ログが止まったら、実行とデバッグから「B: Attach (port 3000)」を選んで F5
# このファイルは Isaac Sim（Kit）の中で --exec として実行されるので、SimulationApp は作らない
import asyncio

import carb.settings
import omni.kit.app
import omni.usd
from pxr import UsdGeom


async def main():
    app = omni.kit.app.get_app()
    await omni.usd.get_context().new_stage_async()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Cube.Define(stage, "/World/Cube")

    for frame in range(30):
        await app.next_update_async()
        prims = [str(p.GetPath()) for p in stage.Traverse()]
        print(f"frame={frame:02d} prims={prims}")  # ← ここにブレークポイントを置く

    # 立方体を足したのでステージは「未保存の変更あり」になっている。このまま post_quit() すると、
    # omni.kit.window.file 拡張が終了を取り消して「保存しますか？」の確認を出す。
    # headless では確認に答えられないので、いつまでも終了しない
    # （omni.kit.window.file の file_window.py の _on_post_quit）。
    # この設定を true にすると、未保存の変更を捨てて終了する
    carb.settings.get_settings().set("/app/file/ignoreUnsavedOnExit", True)
    app.post_quit()


asyncio.ensure_future(main())
