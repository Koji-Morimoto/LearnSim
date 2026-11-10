"""GUI のビューポートを撮る(幾何モードで Play 中、Mover が Target / Wall に入った瞬間)。python send.py v03_capture.py"""

import os

import omni.kit.app
import omni.timeline
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

OUT = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\results"
app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
try:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[5.5, -10.0, 5.0], target=[0.3, -1.5, 0.6])
except Exception as e:  # カメラ設定に失敗しても撮影は続ける
    print("camera:", e)

tl.stop()
for _ in range(5):
    await app.next_update_async()
vp = get_active_viewport()
shots = {"t0_stopped": None, "t2.4_in_target": 2.4, "t4.3_in_wall": 4.3}
for name, t in shots.items():
    if t is not None:
        if not tl.is_playing():
            tl.play()
        while tl.get_current_time() < t:
            await app.next_update_async()
        tl.pause()
    for _ in range(20):  # 描画を落ち着かせる
        await app.next_update_async()
    path = os.path.join(OUT, f"shot_{name}.png")
    capture_viewport_to_file(vp, path)
    for _ in range(10):
        await app.next_update_async()
    print(name, "t=", round(tl.get_current_time(), 3), path)
tl.stop()
