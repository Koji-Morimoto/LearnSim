"""v01 で出た 2 つの疑問を調べる。GUI の中で実行する(python send.py v02_probe.py)。

Q1 物理モードの Play/Stop だけで Root Layer の文字列が変わる → 何が変わるのか(差分を出す)
Q2 最初からめり込んで止まっている kinematic 同士(OverlapA/B)の接触が報告されない
   → 「止まった kinematic は眠る(sleep)ので接触計算されない」仮説を、次の条件で比べる
      a: そのまま
      b: OverlapA に毎フレーム同じ位置を書く(kinematic target を与え続ける)
      c: OverlapA を 0.1 mm だけ往復させる
      d: 片方(OverlapB)を static コライダーにする(RigidBodyAPI を外す) → kinematic–static
結果: ../results/v02_probe.json
"""

import difflib
import importlib
import json
import os
import sys

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
ROOT_DIR = os.path.dirname(CODE_DIR)
DEMO_PATH = os.path.join(ROOT_DIR, "demo", "geometric_mode_demo.usda")
RESULT_PATH = os.path.join(ROOT_DIR, "results", "v02_probe.json")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)
import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import geo_demo  # noqa: E402

importlib.reload(geo_demo)
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, UsdGeom, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
ctx = omni.usd.get_context()
R: dict = {}

try:
    _geo_driver.stop()  # type: ignore[name-defined]  # noqa: F821
except NameError:
    pass


async def frames(n: int) -> None:
    for _ in range(n):
        await app.next_update_async()


async def stop() -> None:
    tl.stop()
    await frames(3)


async def open_fresh():
    """デモをディスクの内容から開き直す(キャッシュされた編集済みレイヤーを使わない)。"""
    from pxr import Sdf

    await ctx.new_stage_async()
    await frames(2)
    cached = Sdf.Layer.Find(DEMO_PATH)
    if cached is not None:
        cached.Reload(force=True)
    await ctx.open_stage_async(DEMO_PATH)
    await frames(5)
    return ctx.get_stage()


def save() -> None:
    with open(RESULT_PATH, "w", encoding="utf-8") as f:
        json.dump(R, f, ensure_ascii=False, indent=1, default=str)


# ------------------------------------------------------------------ Q1
await stop()
st = await open_fresh()
root = st.GetRootLayer()
on_disk = open(DEMO_PATH, encoding="utf-8").read()
r_open = root.ExportToString()
tl.play()
await frames(30)
await stop()
r_after = root.ExportToString()
q1 = R["Q1_root_text"] = {
    "diff_disk_to_opened": [l for l in difflib.unified_diff(on_disk.splitlines(), r_open.splitlines(), lineterm="", n=0)][:60],
    "diff_opened_to_after_play_stop": [l for l in difflib.unified_diff(r_open.splitlines(), r_after.splitlines(), lineterm="", n=0)][:60],
}
# 幾何モードの Play/Stop でも同じ種類の差分だけか
r_before_geo = root.ExportToString()
gm.enable(st)
tl.play()
await frames(30)
await stop()
gm.disable(st)
q1["diff_before_to_after_geometric_play_stop"] = [
    l for l in difflib.unified_diff(r_before_geo.splitlines(), root.ExportToString().splitlines(), lineterm="", n=0)
][:60]
save()


# ------------------------------------------------------------------ Q2
class Log:
    def __init__(self) -> None:
        self.pairs: dict = {}
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(self._cb)

    def _cb(self, headers, data) -> None:
        for h in headers:
            k = " - ".join(sorted(str(PhysicsSchemaTools.intToSdfPath(x)).split("/")[-1] for x in (h.collider0, h.collider1)))
            self.pairs[k] = self.pairs.get(k, 0) + 1


A, B = geo_demo.PATHS["OverlapA"], geo_demo.PATHS["OverlapB"]
q2 = R["Q2_stationary_overlap"] = {}
for case in ("a_as_is", "b_rewrite_same_pos", "c_jitter_0.1mm", "d_B_static"):
    await stop()
    st = await open_fresh()
    if case == "d_B_static":
        # 元シーン側(Root)を一時的に変える。検証後はファイルを開き直すので保存はしない
        st.GetPrimAtPath(B).RemoveAPI(UsdPhysics.RigidBodyAPI)
        st.GetPrimAtPath(B).RemoveAPI(UsdPhysics.MassAPI)
    gm.enable(st)
    op = UsdGeom.Xformable(st.GetPrimAtPath(A)).GetOrderedXformOps()[0]
    x0 = op.Get()
    log = Log()
    tl.play()
    for i in range(90):
        if case.startswith("b"):
            with gm.edit_context(st):
                op.Set(x0)
        elif case.startswith("c"):
            with gm.edit_context(st):
                op.Set(x0 + Gf.Vec3d(1e-4 if i % 2 else 0.0, 0, 0))
        await frames(1)
    await stop()
    q2[case] = dict(log.pairs)
    log.sub = None
    gm.disable(st)
save()

# 後片付け: デモを開き直して幾何モード + ドライバを有効にしておく
st = await open_fresh()
gm.enable(st)
_geo_driver = geo_demo.Driver(st).start()
print(json.dumps(R, ensure_ascii=False, indent=1))
