# -*- coding: utf-8 -*-
"""検証結果を見せる静止画・動画を headless の Isaac Sim で撮る（isaac-sim-visual-explainer を使う）。

出力: ..\\media\\*.png / *.mp4 / *.gif
実行:  $env:PYTHONPATH=""; C:\\isaacsim-6.2.0\\python.bat make_media.py --no-ros-env
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = r"C:\isaacsim-6.2.0\CodeByClaude\LearnOmni_repo\RobotRepresentation"
sys.path.insert(0, r"C:\isaacsim-6.2.0\skills\isaac-sim-visual-explainer\scripts")
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "examples"))

from explainer import Explainer, Marker  # noqa: E402
import argparse  # noqa: E402

_p = argparse.ArgumentParser()
_p.add_argument("--scene", type=int, required=True)
ARGS, _ = _p.parse_known_args()

MEDIA = os.path.join(ROOT, "media_v2")
ex = Explainer(out_dir=MEDIA, name="tmp", render_size=(960, 600), canvas_size=(1600, 900), fps=20)
app = ex.start_app()

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, Vt  # noqa: E402

from robot_repr import DHRobot, ScrewRobot, Style, collision_segments  # noqa: E402
from robot_repr.collision import CollisionChecker, ContactMonitor  # noqa: E402
from robot_repr.scene import PALETTE, build_robot  # noqa: E402
from sample_robots import all_three  # noqa: E402

EXAMPLES = json.load(open(os.path.join(ROOT, "results_v2", "isaac_verify.json"), encoding="utf-8"))["examples"]
ROBOTS = all_three()
CHAIN = ROBOTS["screw"].to_chain()
OBST_C, OBST_R = np.array([0.35, 0.30, 0.45]), 0.08
RED = (0.95, 0.12, 0.10)
LABEL = {"dh": "DH（入れ子）", "screw": "スクリュー（平ら）", "swing": "スイング（入れ子）"}


def new_stage():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(350.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1200.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(40, 0, 25))
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.006))
    floor.AddScaleOp().Set(Gf.Vec3f(8, 8, 0.01))
    floor.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.12, 0.13, 0.15)]))
    return stage


def place(stage, path, offset):
    UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*offset))


def obstacle(stage, path, offset, collide=True):
    g = UsdGeom.Sphere.Define(stage, path)
    g.CreateRadiusAttr(OBST_R)
    g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(OBST_C + np.array(offset))))
    g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.85, 0.85, 0.85)]))
    if collide:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return g


def color_bodies(stage, sr, hot):
    for k, (body, paths) in enumerate(sr.visual_paths().items()):
        c = RED if body in hot else PALETTE[k % len(PALETTE)]
        for p in paths:
            UsdGeom.Gprim(stage.GetPrimAtPath(p)).GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*c)]))


def usd_endpoints(stage, sr):
    cache = UsdGeom.XformCache()
    root_inv = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sr.path)).GetInverse()
    out = []
    for group in (sr.collider_paths(), sr.visual_paths()):
        pts = {}
        for paths in group.values():
            for p in paths:
                prim = stage.GetPrimAtPath(p)
                m = cache.GetLocalToWorldTransform(prim) * root_inv
                if prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Cylinder):
                    h = prim.GetAttribute("height").Get()
                    pts[int(p.rsplit("_", 1)[1])] = (np.array(m.Transform(Gf.Vec3d(0, 0, -h / 2))), np.array(m.Transform(Gf.Vec3d(0, 0, h / 2))))
                else:
                    c = np.array(m.Transform(Gf.Vec3d(0, 0, 0)))
                    pts[int(p.rsplit("_", 1)[1])] = (c, c)
        out += [pts[i] for i in range(len(pts))]
    return out


def tip(sr, offset):
    t = sr.tool_pose()
    return tuple(t[:3, 3] + np.array(offset))


def fmt_q(q, kinds):
    return "  ".join(f"{math.degrees(v):+6.1f}°" if k == "R" else f"{v * 1000:+5.0f}mm" for v, k in zip(q, kinds))


def warm(n=60):
    """新しいステージとカメラで RTX の準備（マテリアルのコンパイル）を待つ。"""
    for _ in range(n):
        app.update()
    for _ in range(3):
        ex.capture(settle=1)


def begin(name):
    ex.name = name
    ex.frames = []


def finish():
    print("[MEDIA]", ex.write(gif_width=960), flush=True)


def smooth_traj(n, seed=3):
    rng = np.random.default_rng(seed)
    lim = ROBOTS["screw"].limits
    mids = [0.5 * (lo + hi) for lo, hi in lim]
    amps = [0.45 * (hi - lo) for lo, hi in lim]
    w = rng.uniform(0.6, 1.4, size=len(lim))
    ph = rng.uniform(0, 2 * math.pi, size=len(lim))
    return [[m + a * math.sin(wi * 2 * math.pi * t / n + p) * min(1.0, t / 15.0) for m, a, wi, p in zip(mids, amps, w, ph)]
            for t in range(n)]


# ============================================================
# 1. 3 表現で組んだ同じロボット（静止画 + 動画）
# ============================================================
def scene_same_robot():
    stage = new_stage()
    offs = {"dh": (-0.95, 0, 0), "screw": (0.0, 0, 0), "swing": (0.95, 0, 0)}
    srs = {}
    for rep, robot in ROBOTS.items():
        srs[rep] = build_robot(stage, f"/World/Robot_{rep}", robot, collision="none")
        place(stage, f"/World/Robot_{rep}", offs[rep])
    ex.add_camera(eye=(0.3, -2.25, 1.25), target=(0.3, 0.05, 0.3))
    warm()

    begin("01_same_robot_three_representations")
    ex.title_card("同じロボットを 3 つの表現で USD に直接組む",
                  ["URDF・MJCF を通さず、ArticulationRoot も使わない（Xform と Capsule / Sphere だけ）",
                   "左から DH（入れ子の Xform）・スクリュー（平らに並べた Xform）・スイング（入れ子の Xform）",
                   "どれも robot.set_joint_positions(q) で同じ関節値を与えて動かす"], seconds=3.5)
    labels = [Marker((offs[r][0], 0, 0.8), LABEL[r], "#ffd040", "square", 4) for r in ROBOTS]
    nparams = {r: len(ROBOTS[r].flat()[1]) for r in ROBOTS}
    rows = [(f"{LABEL[r]}", f"連続値 {nparams[r]} 個") for r in ROBOTS]
    frame = ex.compose(ex.capture(), title="関節値 0：3 つとも同じ形", caption=(
        "6 軸 R R P R R R。第 1・2 軸はねじれの位置、第 2・4 軸は逆向きに平行、第 3 軸は伸縮、第 4〜6 軸は順に交わる。\n"
        "見た目はリンクが円柱（半径 25 mm）、関節は回転軸に沿った円柱。形は軸の直線から決まる規則で作るので、表現が違っても同じになる。"),
        rows=rows, markers=labels, panel_title="パラメータの数（可動範囲を含む）")
    ex.add(frame, 3.0)
    ex.save_still(frame, "home")

    traj = smooth_traj(140)
    kinds = ROBOTS["screw"].kinds
    worst = 0.0
    for t, q in enumerate(traj):
        for sr in srs.values():
            sr.set_joint_positions(q, clip=True)
        e = {r: usd_endpoints(stage, sr) for r, sr in srs.items()}
        diff = max(float(np.max(np.abs(np.array(a) - np.array(b)))) for r in ("dh", "swing")
                   for a, b in zip(e[r], e["screw"]))
        worst = max(worst, diff)
        marks = labels + [Marker(tip(srs[r], offs[r]), "", "#40ff80", "circle", 7) for r in ROBOTS]
        frame = ex.compose(ex.capture(), title="同じ関節値の列を 3 体に与える", caption=(
            "関節値 q（上から順に 1〜6 軸）: " + fmt_q(q, kinds) + "\n"
            "右の値は、USD の xform を合成して求めた全形状（見た目と干渉判定）の端点の、スクリュー表現との最大の差。緑の丸は工具の原点。"),
            rows=[("フレーム", f"{t + 1} / {len(traj)}"), ("DH・スイングとの差（このフレーム）", f"{diff:.1e} m"),
                  ("ここまでの最大", f"{worst:.1e} m")], markers=marks, badge="Stop 中（物理なし）")
        ex.add(frame, 1 / ex.fps)
        if t == 90:
            ex.save_still(frame, "moving")
    finish()
    return worst


# ============================================================
# 2. 干渉検知（静的コライダー + overlap）
# ============================================================
def seg_dist(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-15 and e < 1e-15:
        return float(np.linalg.norm(r))
    if a < 1e-15:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e < 1e-15:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0.0, 1.0) if den > 1e-15 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def truth_pairs(q):
    names = ["base"] + [f"link{k}" for k in range(1, CHAIN.dof + 1)]
    mot = CHAIN.body_motions(q)
    segs = []
    for s in collision_segments(CHAIN, Style()):
        m = mot[s.body]
        segs.append((s, m[:3, :3] @ np.array(s.a) + m[:3, 3], m[:3, :3] @ np.array(s.b) + m[:3, 3]))
    hit = set()
    for i in range(len(segs)):
        si, ai, bi = segs[i]
        if seg_dist(ai, bi, OBST_C, OBST_C) < si.radius + OBST_R:
            hit.add((names[si.body], "障害物"))
        for j in range(i + 1, len(segs)):
            sj, aj, bj = segs[j]
            if abs(si.body - sj.body) <= 1:
                continue
            if seg_dist(ai, bi, aj, bj) < si.radius + sj.radius:
                hit.add((names[min(si.body, sj.body)], names[max(si.body, sj.body)]))
    return hit


def lerp(a, b, s):
    return [x + (y - x) * s for x, y in zip(a, b)]


def scene_collision():
    stage = new_stage()
    sr = build_robot(stage, "/World/Robot", ROBOTS["dh"], collision="static")
    obs = obstacle(stage, "/World/Obstacle", (0, 0, 0))
    checker = CollisionChecker(sr)
    ex.add_camera(eye=(1.0, -0.95, 0.85), target=(0.22, 0.08, 0.3))
    warm()
    begin("02_collision_check_static")
    ex.title_card("物理演算なしの干渉検知（RigidBody なし）",
                  ["形状に CollisionAPI だけを付けた静的コライダー。重力も力も無い",
                   "CollisionChecker が PhysX の overlap 問い合わせで重なりを調べる（タイムラインは Stop のまま）",
                   "干渉したボディを赤く塗る。右の表は、カプセル間距離の解析計算との比較"], seconds=3.5)
    q_free, q_env, q_self = EXAMPLES["free"], EXAMPLES["env"], EXAMPLES["self"]
    path = ([lerp(q_free, q_env, s / 50) for s in range(51)] + [q_env] * 12 +
            [lerp(q_env, q_self, s / 50) for s in range(51)] + [q_self] * 12 +
            [lerp(q_self, q_free, s / 50) for s in range(51)])
    agree = 0
    shots = {}
    for t, q in enumerate(path):
        sr.set_joint_positions(q)
        found = checker.check()
        got = {(a, "障害物" if b == str(obs.GetPath()) else b) for a, b in found}
        want = truth_pairs(q)
        agree += got == want
        hot = {a for a, _ in got} | {b for _, b in got}
        color_bodies(stage, sr, hot)
        obs.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*(RED if "障害物" in hot else (0.85, 0.85, 0.85)))]))
        txt = "、".join(f"{a}–{b}" for a, b in sorted(got)) or "なし"
        mot = CHAIN.body_motions(q)
        marks = []
        for k, body in enumerate(sr.body_names):
            if body in hot:
                pts = [mot[k][:3, :3] @ np.array(p) + mot[k][:3, 3] for sg in collision_segments(CHAIN, Style()) if sg.body == k for p in (sg.a, sg.b)]
                marks.append(Marker(tuple(np.mean(pts, axis=0)), body, "#ff7070", "cross", 6))
        frame = ex.compose(ex.capture(), title="関節値を変えた直後に checker.check()", markers=marks, caption=(
            "checker.check() の結果: " + txt + "\n"
            "隣り合うボディ（関節で接する組）は既定で除外。障害物は RigidBody なしの球のコライダー。"),
            rows=[("フレーム", f"{t + 1} / {len(path)}"), ("check() の結果", txt),
                  ("解析計算と一致したフレーム", f"{agree} / {t + 1}")], badge="Stop 中（物理なし）")
        ex.add(frame, 1 / ex.fps)
        if t in (62, 125):
            shots[t] = frame
    ex.save_still(shots[62], "obstacle")
    ex.save_still(shots[125], "self")
    finish()
    return agree, len(path)


# ============================================================
# 3. 正準形: 別の書き方でも同じロボット
# ============================================================
def scene_canonical():
    stage = new_stage()
    dh = ROBOTS["dh"]
    raw_axes = [dict(kind=a.kind, alpha=-a.alpha, a=-a.a, theta=a.theta, d=a.d, lower=a.lower, upper=a.upper) for a in dh.axes]
    raw_base = dict(theta=dh.base.theta + math.pi, d=dh.base.d)
    raw_tool = dict(alpha=-dh.tool.alpha, a=-dh.tool.a, theta=dh.tool.theta + math.pi, d=dh.tool.d)
    raw_chain = DHRobot._raw_chain(raw_axes, raw_tool, raw_base)
    canon = DHRobot.canonicalize(raw_axes, raw_tool, raw_base)
    try:
        DHRobot.from_dict({"representation": "dh", "base": raw_base, "axes": raw_axes, "tool": raw_tool})
        rejected = "受け付けた"
    except ValueError as e:
        rejected = str(e).split("（")[0]
    left = build_robot(stage, "/World/Canonical", dh, collision="none")
    place(stage, "/World/Canonical", (-0.45, 0, 0))
    right = build_robot(stage, "/World/Raw", ScrewRobot.from_chain(raw_chain), collision="none")
    place(stage, "/World/Raw", (0.55, 0, 0))
    for sr in (left, right):
        sr.set_joint_positions(smooth_traj(140)[90], clip=True)
    ex.add_camera(eye=(0.3, -1.9, 1.05), target=(0.3, 0.05, 0.33))
    warm()
    begin("03_canonical_form")
    rows = [("正準形の第 2 軸 (alpha, a)", f"({dh.axes[1].alpha:+.4f}, {dh.axes[1].a:+.3f})"),
            ("別の書き方の第 2 軸 (alpha, a)", f"({raw_axes[1]['alpha']:+.4f}, {raw_axes[1]['a']:+.3f})"),
            ("別の書き方を DHRobot に渡すと", rejected),
            ("canonicalize() の結果", "正準形と一致" if np.allclose(canon.flat()[1], dh.flat()[1], atol=1e-9) else "不一致")]
    frame = ex.compose(ex.capture(), title="DH の別の書き方（垂線の向きを反転）でも同じロボットになる", caption=(
        "左: 正準形のパラメータ。右: 全部の共通垂線の向きを反転した書き方（a < 0、alpha の符号反転、theta + pi）。\n"
        "形も動きも同じなので、正準形は a ≥ 0 などの条件でこの書き方を締め出し、1 つのロボットに 1 つの値だけを残す。"),
        rows=rows, markers=[Marker((-0.45, 0, 0.8), "正準形", "#ffd040", "square", 4),
                            Marker((0.55, 0, 0.8), "別の書き方", "#ffd040", "square", 4)], panel_title="値")
    ex.add(frame, 4.0)
    ex.save_still(frame, "still")
    finish()


# ============================================================
# 4. リンクの生やし方の規則
# ============================================================
def scene_link_rule():
    from robot_repr import ScrewAxis, ScrewTool

    stage = new_stage()
    # 軸どうしがねじれの位置にある 3 軸（規則の違いが見やすいもの）
    robot = ScrewRobot([ScrewAxis.through("R", (0, 0, 0), (0, 0, 1)),
                        ScrewAxis.through("R", (0, 0.15, 0.35), (1, 0, 0)),
                        ScrewAxis.through("R", (0.35, 0, 0.6), (0, 1, 0))],
                       ScrewTool((0.35, -0.12, 0.6), ((1, 0, 0), (0, 0, 1), (0, -1, 0))))
    a = build_robot(stage, "/World/CommonNormal", robot, Style(link_rule="common_normal"), collision="none")
    place(stage, "/World/CommonNormal", (-0.45, 0, 0))
    b = build_robot(stage, "/World/Projection", robot, Style(link_rule="projection"), collision="none")
    place(stage, "/World/Projection", (0.55, 0, 0))
    ex.add_camera(eye=(0.3, -1.9, 1.05), target=(0.3, 0.05, 0.3))
    warm()
    begin("04_link_rule")
    frame = ex.compose(ex.capture(), title="形状は軸の直線から規則で一意に決める（Style.link_rule）", caption=(
        "左 common_normal: 隣の軸との共通垂線に沿ってリンクを張る（DH の座標系の原点と一致）。\n"
        "右 projection: 前の軸に入ってきた点から次の軸へ垂線を下ろす。3 軸はねじれの位置。軸の直線は同じなので動きは同じ。"),
        rows=[("左", "common_normal（既定）"), ("右", "projection")],
        markers=[Marker((-0.45, 0, 0.8), "common_normal", "#ffd040", "square", 4),
                 Marker((0.55, 0, 0.8), "projection", "#ffd040", "square", 4)], panel_title="Style.link_rule")
    ex.add(frame, 4.0)
    ex.save_still(frame, "still")
    finish()


# ============================================================
# 5. キネマティック剛体: 入れ子と平らの違い
# ============================================================
def scene_kinematic_layout():
    stage = new_stage()
    flat = build_robot(stage, "/World/Flat", ROBOTS["dh"], collision="kinematic", layout="flat")
    place(stage, "/World/Flat", (-0.45, 0, 0))
    nested = build_robot(stage, "/World/Nested", ROBOTS["dh"], collision="kinematic", layout="nested")
    place(stage, "/World/Nested", (0.55, 0, 0))
    o1 = obstacle(stage, "/World/ObsFlat", (-0.45, 0, 0))
    o2 = obstacle(stage, "/World/ObsNested", (0.55, 0, 0))
    q_free, q_env = EXAMPLES["free"], EXAMPLES["env"]
    for sr in (flat, nested):
        sr.set_joint_positions(q_free)
    mon = ContactMonitor(flat)
    nested.layout = "flat"  # 比較のためだけに ContactMonitor の拒否を外す（実際の購読は同じ PhysX の報告）
    mon_n = ContactMonitor(nested)
    nested.layout = "nested"
    chk = {"flat": CollisionChecker(flat), "nested": CollisionChecker(nested)}
    ex.add_camera(eye=(0.3, -1.95, 1.1), target=(0.3, 0.12, 0.36))
    warm()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(10):
        app.update()
    begin("05_kinematic_nested_vs_flat")
    ex.title_card("RigidBody（kinematicEnabled）を付けた場合",
                  ["左: ボディを平らに並べる（collision='kinematic' の既定）",
                   "右: ボディを入れ子にする（layout='nested' を強制）",
                   "同じ動きで障害物の球に触れる。Contact Report（ContactMonitor）と overlap（CollisionChecker）を並べて見る"],
                  seconds=4.0)
    path = [lerp(q_free, q_env, s / 50) for s in range(51)] + [q_env] * 25
    for t, q in enumerate(path):
        for sr in (flat, nested):
            sr.set_joint_positions(q)
        app.update()
        m_f = any(b == str(o1.GetPath()) for _, b in mon.touching())
        m_n = any(b == str(o2.GetPath()) for _, b in mon_n.touching())
        c_f = any(b == str(o1.GetPath()) for _, b in chk["flat"].check())
        c_n = any(b == str(o2.GetPath()) for _, b in chk["nested"].check())
        for o, hit in ((o1, m_f), (o2, m_n)):
            o.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*(RED if hit else (0.85, 0.85, 0.85)))]))
        yes = lambda v: "接触あり" if v else "なし"  # noqa: E731
        frame = ex.compose(ex.capture(), title="静的な障害物との Contact Report は、入れ子のキネマティック剛体では来ない", caption=(
            "球が赤 = ContactMonitor（Contact Report）が接触を報告した。overlap（CollisionChecker）は両方とも正しく検知する。\n"
            "入れ子では、自分の xformOp が変わったボディについて PhysX の接触ペアが作られない（Isaac Sim 6.2.0 で測定）。"),
            rows=[("左 平ら: Contact Report", yes(m_f)), ("左 平ら: overlap", yes(c_f)),
                  ("右 入れ子: Contact Report", yes(m_n)), ("右 入れ子: overlap", yes(c_n))],
            markers=[Marker((-0.45, 0, 0.8), "平ら（flat）", "#ffd040", "square", 4),
                     Marker((0.55, 0, 0.8), "入れ子（nested）", "#ffd040", "square", 4)], badge="Play 中")
        ex.add(frame, 1 / ex.fps)
        if t == len(path) - 1:
            ex.save_still(frame, "hit")
    tl.stop()
    app.update()
    mon.close()
    mon_n.close()
    finish()


# ============================================================
# 6. 見た目と干渉判定の形状
# ============================================================
def scene_visual_vs_collision():
    stage = new_stage()
    vis = build_robot(stage, "/World/Visual", ROBOTS["dh"], collision="static")
    place(stage, "/World/Visual", (-0.45, 0, 0))
    col = build_robot(stage, "/World/Collision", ROBOTS["dh"], collision="static")
    place(stage, "/World/Collision", (0.55, 0, 0))
    q = smooth_traj(140)[90]
    for sr in (vis, col):
        sr.set_joint_positions(q, clip=True)
    # 右のロボットだけ、見た目を隠して干渉判定の形状（purpose = guide）を描く
    for bp in col.body_paths:
        UsdGeom.Imageable(stage.GetPrimAtPath(bp.AppendChild("visuals"))).MakeInvisible()
        UsdGeom.Imageable(stage.GetPrimAtPath(bp.AppendChild("collisions"))).CreatePurposeAttr(UsdGeom.Tokens.default_)
    ex.add_camera(eye=(0.3, -1.9, 1.05), target=(0.3, 0.05, 0.33))
    warm()
    begin("06_visual_vs_collision")
    n_vis = sum(len(v) for v in vis.visual_paths().values())
    n_col = sum(len(v) for v in vis.collider_paths().values())
    frame = ex.compose(ex.capture(), title="見た目（visuals）と干渉判定（collisions）は別の形状", caption=(
        "左: 見た目。リンクは円柱、関節は回転軸に沿った円柱、折れ目は球（どれも半径 25 mm）。干渉判定には使わない。\n"
        "右: 干渉判定の形状。リンクの折れ線の各区間のカプセル（円柱 + 両端の球と同じ形）。ふだんは purpose = guide で描かない。"),
        rows=[("見た目の形状の数", f"{n_vis} 個（円柱と球）"), ("干渉判定の形状の数", f"{n_col} 個（カプセルと球）"),
              ("CollisionAPI", "干渉判定の形状だけ")],
        markers=[Marker((-0.45, 0, 0.8), "visuals", "#ffd040", "square", 4),
                 Marker((0.55, 0, 0.8), "collisions", "#ffd040", "square", 4)], panel_title="同じロボット・同じ関節値")
    ex.add(frame, 4.0)
    ex.save_still(frame, "still")
    finish()


SCENES = {1: scene_same_robot, 2: scene_collision, 3: scene_canonical, 4: scene_link_rule, 5: scene_kinematic_layout,
          6: scene_visual_vs_collision}
res = SCENES[ARGS.scene]()
print("[RESULT]", json.dumps({"scene": ARGS.scene, "result": res}, default=float), flush=True)
print("[DONE]", flush=True)
app.close()
