"""コライダーの表示色が何で決まるかを撮り比べる。GUI の中で実行する(python send.py v05_colors.py)。

    1_physics   : 元の物理シーン(剛体は dynamic)
    2_geometric : 幾何モード(剛体は全部 kinematic)
    3_mixed     : 物理シーンのまま Target だけ kinematic にする
あわせて、コライダー表示に関係する carb 設定を列挙する。
"""

import importlib
import os
import sys

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
OUT = os.path.join(os.path.dirname(CODE_DIR), "results")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)
import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
omni.timeline.get_timeline_interface().stop()
st = omni.usd.get_context().get_stage()
s = carb.settings.get_settings()


def walk(path: str, d) -> None:
    if isinstance(d, dict):
        for k, v in d.items():
            walk(f"{path}/{k}", v)
    elif any(w in path.lower() for w in ("collider", "collision", "visualiz")):
        print(path, "=", d)


walk("/persistent/physics", s.get("/persistent/physics"))
walk("/physics", s.get("/physics"))

try:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[4.0, -7.0, 3.5], target=[0.3, -1.5, 0.6])
except Exception as e:  # noqa: BLE001
    print("camera:", e)

vp = get_active_viewport()
none = gm.Options(False, False, False, 0.0, False, False, False, False)  # 何も上書きしない空の幾何レイヤー
for name in ("1_physics", "2_geometric", "3_mixed"):
    gm.disable(st)
    if name == "2_geometric":
        gm.enable(st)
    elif name == "3_mixed":
        gm.enable(st, none)
        with gm.edit_context(st):
            UsdPhysics.RigidBodyAPI(st.GetPrimAtPath("/World/Lane1/Target")).CreateKinematicEnabledAttr(True)
    for _ in range(30):
        await app.next_update_async()
    path = os.path.join(OUT, f"color_{name}.png")
    capture_viewport_to_file(vp, path)
    for _ in range(10):
        await app.next_update_async()
    print("shot", path)

gm.disable(st)
gm.enable(st)  # 幾何モードに戻しておく
