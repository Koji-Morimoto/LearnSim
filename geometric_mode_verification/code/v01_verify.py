"""幾何モードの検証。起動中の Isaac Sim GUI の中で実行する(python send.py v01_verify.py)。

手順:
    P0 検証用シーンを作って ../demo/geometric_mode_demo.usda に保存し、開き直す。元の状態を記録
    P1 物理のまま Play            → 落下・押し離し・腕の垂れ下がりが起きることを確認(対照)
    P2 幾何モードで Play          → 何も落ちず、Mover だけが動き、通り抜けた相手との接触が報告されるか
    P3 kinematic 組の報告フラグなし → 接触が報告されなくなるか(フラグの必要性)
    P4 幾何モード中のユーザー編集・保存 → Root Layer に幾何モードの値が混ざらないか
    P5 元に戻す                   → 合成結果・Root Layer が P0 と完全一致し、物理の挙動も戻るか
結果: ../results/v01_verify.json
"""

import importlib
import json
import os
import sys
import time

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
ROOT_DIR = os.path.dirname(CODE_DIR)
DEMO_PATH = os.path.join(ROOT_DIR, "demo", "geometric_mode_demo.usda")
RESULT_PATH = os.path.join(ROOT_DIR, "results", "v01_verify.json")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import geo_demo  # noqa: E402

importlib.reload(geo_demo)

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Sdf, Usd, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
ctx = omni.usd.get_context()
R: dict = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "phases": {}}

# 前回の実行で残ったドライバがあれば止める
try:
    _geo_driver.stop()  # type: ignore[name-defined]  # noqa: F821
except NameError:
    pass


def world_text(layer) -> str:
    """Root Layer のうち /World 以下だけを文字列にする。

    Kit は GUI のカメラ(OmniverseKit_*)と描画設定(customLayerData)を Root Layer に書き足すので、
    それを除いて「シーンの中身」だけを比べる(v02_probe Q1 で確認)。
    """
    tmp = Sdf.Layer.CreateAnonymous()
    tmp.TransferContent(layer)
    for name in [n for n in tmp.rootPrims.keys() if n != "World"]:
        del tmp.rootPrims[name]
    tmp.customLayerData = {}
    return tmp.ExportToString()


def save() -> None:
    os.makedirs(os.path.dirname(RESULT_PATH), exist_ok=True)
    with open(RESULT_PATH, "w", encoding="utf-8") as f:
        json.dump(R, f, ensure_ascii=False, indent=1, default=str)


async def frames(n: int) -> None:
    for _ in range(n):
        await app.next_update_async()


async def play_for(seconds: float) -> dict:
    """seconds だけ(タイムライン時刻で)Play し、監視対象の位置を返す。Stop は呼び出し側。"""
    tl.play()
    await frames(2)
    n = 0
    while tl.get_current_time() < seconds and n < 5000:
        await frames(1)
        n += 1
    st = ctx.get_stage()
    pos = {k: [round(v, 4) for v in geo_demo.world_pos(st, p)] for k, p in geo_demo.PATHS.items() if k not in ("Ground", "LinkGeom")}
    return {"timeline_time": round(tl.get_current_time(), 3), "app_frames": n, "pos": pos}


async def stop() -> None:
    tl.stop()
    await frames(3)


def moved(a: dict, b: dict) -> dict:
    """2 つの位置辞書の差(各物体の移動量 [m])。"""
    out = {}
    for k in a:
        d = [b[k][i] - a[k][i] for i in range(3)]
        out[k] = round((d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5, 4)
    return out


def pair_table(drv) -> dict:
    return {
        f"{a.split('/')[-1] if not a.endswith('Geom') else a.split('/')[-2] + '/Geom'} - {b.split('/')[-1] if not b.endswith('Geom') else b.split('/')[-2] + '/Geom'}": {
            "first_t": round(s.first_time, 3),
            "last_t": round(s.last_time, 3),
            "events": s.events,
            "min_sep": round(s.min_separation, 4),
            "max_sep": round(s.max_separation, 4),
            "mover_x_at_first": None if s.first_mover_x is None else round(s.first_mover_x, 3),
        }
        for (a, b), s in sorted(drv.pairs.items())
    }


def initial_pos(st) -> dict:
    return {k: [round(v, 4) for v in geo_demo.world_pos(st, p)] for k, p in geo_demo.PATHS.items() if k not in ("Ground", "LinkGeom")}


# ------------------------------------------------------------------ P0
await stop()
await ctx.new_stage_async()
await frames(2)
st = ctx.get_stage()
geo_demo.build_original_scene(st)
os.makedirs(os.path.dirname(DEMO_PATH), exist_ok=True)
st.GetRootLayer().Export(DEMO_PATH)
await ctx.open_stage_async(DEMO_PATH)
await frames(5)
st = ctx.get_stage()
root = st.GetRootLayer()
S0 = gm.snapshot(st)
R0 = world_text(root)
P0 = initial_pos(st)
R["phases"]["P0"] = {"demo": DEMO_PATH, "root_dirty": root.dirty, "session_sublayers": list(st.GetSessionLayer().subLayerPaths), "initial_pos": P0}
save()

_geo_driver = geo_demo.Driver(st).start()
drv = _geo_driver

# ------------------------------------------------------------------ P1 物理のまま
drv.reset_log()
r1 = await play_for(2.0)
await stop()
r1["moved"] = moved(P0, r1["pos"])
r1["contacts"] = pair_table(drv)  # 元のシーンには ContactReportAPI が無いので空のはず
r1["pos_after_stop_equals_initial"] = initial_pos(st) == P0
r1["root_unchanged_after_stop"] = world_text(root) == R0
R["phases"]["P1_physics"] = r1
save()

# ------------------------------------------------------------------ P2 幾何モード
rep = gm.enable(st)
S2 = gm.snapshot(st)
R["phases"]["P2_geometric"] = p2 = {
    "report": rep.to_dict(),
    "snapshot_diff_from_original": gm.diff(S0, S2),
    "root_unchanged_after_enable": world_text(root) == R0,
    "root_dirty": root.dirty,
}
save()
drv.reset_log()
r2 = await play_for(11.0)  # Mover が 1 往復(-2.5 → 3.0 → -2.5)する時間
r2["moved"] = moved(P0, r2["pos"])
r2["contacts"] = pair_table(drv)
await stop()
r2["pos_after_stop"] = initial_pos(st)
r2["root_unchanged_after_stop"] = world_text(root) == R0
p2.update(r2)
save()

# ------------------------------------------------------------------ P3 kinematic 組の報告フラグなし(対照)
gm.enable(st, gm.Options(report_kinematic_pairs=False))
drv.reset_log()
r3 = await play_for(5.5)
r3["moved"] = moved(P0, r3["pos"])
r3["contacts"] = pair_table(drv)
await stop()
R["phases"]["P3_no_kinematic_pair_flags"] = r3
save()

# ------------------------------------------------------------------ P4 幾何モード中のユーザー編集と保存
gm.enable(st)
note = st.GetPrimAtPath("/World").CreateAttribute("userNote", Sdf.ValueTypeNames.String)  # 既定の編集先 = Root Layer
note.Set("edited while geometric mode")
root_text_on = world_text(root)
R["phases"]["P4_edit_and_save"] = p4 = {
    "edit_target_is_root": st.GetEditTarget().GetLayer() == root,
    "root_has_user_edit": "userNote" in root_text_on,
    "root_has_geometric_values": any(s in root_text_on for s in ("kinematicEnabled", "reportKinematic", "PhysxContactReportAPI", "jointEnabled")),
    "root_gravity_line": [l.strip() for l in root_text_on.splitlines() if "gravityMagnitude" in l],
}
# 幾何モード中に重力を変えても(Root に書かれるので)幾何レイヤーの 0 が勝つことの確認
UsdPhysics.Scene(st.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Set(5.0)
p4["composed_gravity_after_user_sets_5_in_root"] = UsdPhysics.Scene(st.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Get()
gm.disable(st)
p4["user_edit_survives_disable"] = st.GetPrimAtPath("/World").GetAttribute("userNote").Get()
p4["gravity_after_disable"] = UsdPhysics.Scene(st.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Get()
# ユーザー編集を取り消して P0 の状態に戻す
UsdPhysics.Scene(st.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Set(9.81)
st.GetPrimAtPath("/World").RemoveProperty("userNote")
save()

# ------------------------------------------------------------------ P5 元に戻す
gm.enable(st)
await frames(2)
gm.disable(st)
await frames(2)
S5 = gm.snapshot(st)
R["phases"]["P5_restored"] = p5 = {
    "geometric_layer_remaining": gm.is_enabled(st),
    "session_sublayers": list(st.GetSessionLayer().subLayerPaths),
    "snapshot_diff_from_original": gm.diff(S0, S5),
    "root_text_equals_original": world_text(root) == R0,
}
drv.reset_log()
r5 = await play_for(2.0)
await stop()
r5["moved"] = moved(P0, r5["pos"])
r5["contacts"] = pair_table(drv)
p5.update(r5)
p5["root_text_equals_original_after_play"] = world_text(root) == R0
save()

# ------------------------------------------------------------------ 最後に: GUI で見られるよう幾何モードにしておく
final = gm.enable(st)
R["left_in_gui"] = {"geometric_mode": gm.is_enabled(st), "driver_running": True}
R["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
save()
print(json.dumps(R, ensure_ascii=False, indent=1, default=str)[:20000])
