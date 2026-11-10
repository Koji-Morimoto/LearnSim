"""接触イベントの種類(FOUND / PERSIST / LOST)ごとの色分けを確認する。GUI の中で実行する(python send.py v06_event_colors.py)。

Mover が Target と Wall を通り抜ける間の決めた時刻で一時停止し、各コライダーの状態・実際の表示色を記録して撮影する。
最後に、組ごとのイベント数と FOUND / LOST の時刻を記録する。結果: ../results/v06_event_colors.json
"""

import importlib
import json
import os
import sys

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
OUT = os.path.join(os.path.dirname(CODE_DIR), "results")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)
import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import geo_demo  # noqa: E402

importlib.reload(geo_demo)
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import UsdGeom  # noqa: E402

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
st = omni.usd.get_context().get_stage()

try:
    _geo_driver.stop()  # type: ignore[name-defined]  # noqa: F821
except NameError:
    pass
tl.stop()
for _ in range(3):
    await app.next_update_async()
if not gm.is_enabled(st):
    gm.enable(st)
_geo_driver = geo_demo.Driver(st).start()
drv = _geo_driver

try:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[0.8, -4.5, 2.2], target=[0.9, 0.0, 0.6])
except Exception as e:  # noqa: BLE001
    print("camera:", e)

NAMES = {geo_demo.PATHS["Mover"]: "Mover", geo_demo.PATHS["Target"]: "Target", geo_demo.PATHS["Wall"] + "/Geom": "Wall"}
COLOR_NAME = {tuple(round(c, 2) for c in v): k for k, v in geo_demo.STATE_COLORS.items()}


def shown_color(path: str) -> str:
    v = UsdGeom.Gprim(st.GetPrimAtPath(path)).GetDisplayColorAttr().Get()
    key = tuple(round(c, 2) for c in v[0])
    return COLOR_NAME.get(key, "original")


R: dict = {"shots": []}
vp = get_active_viewport()
for t in (2.1, 2.6, 3.2, 4.3, 4.6, 5.0):
    if not tl.is_playing():
        tl.play()
    while tl.get_current_time() < t:
        await app.next_update_async()
    tl.pause()
    try:
        set_camera_view(eye=[0.8, -4.5, 2.2], target=[0.9, 0.0, 0.6])  # 途中で視点が動いても撮影位置をそろえる
    except Exception:  # noqa: BLE001
        pass
    for _ in range(15):
        await app.next_update_async()
    now = tl.get_current_time()
    path = os.path.join(OUT, f"event_t{t:.1f}.png")
    capture_viewport_to_file(vp, path)
    for _ in range(10):
        await app.next_update_async()
    R["shots"].append(
        {
            "t": round(now, 3),
            "mover_x": round(geo_demo.mover_x(now), 3),
            "state": {NAMES.get(p, p): s for p, s in drv.collider_states(now).items()},
            "shown": {n: shown_color(p) for p, n in NAMES.items()},
            "png": path,
        }
    )
# Mover が戻り切るまで流して、組ごとの集計を取る
tl.play()
while tl.get_current_time() < 11.0:
    await app.next_update_async()
tl.pause()
R["pairs"] = {
    " - ".join(NAMES.get(p, p.split("/")[-1]) for p in k): {"counts": s.counts, "found_lost_log": [(round(a, 3), b) for a, b in s.log]}
    for k, s in drv.pairs.items()
}
tl.stop()
with open(os.path.join(OUT, "v06_event_colors.json"), "w", encoding="utf-8") as f:
    json.dump(R, f, ensure_ascii=False, indent=1)
print(json.dumps(R, ensure_ascii=False, indent=1))
