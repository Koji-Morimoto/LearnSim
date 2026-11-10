"""R01: 描画(RTX)は USD / Fabric / PhysX のどの値を映すか。

描画ありのエクスペリエンス(既定の isaacsim.exp.base.python.kit。headless)で起動し、
真上から見下ろすカメラで赤い立方体を撮影する。画像の赤い画素の重心から、立方体の xy 位置を逆算する。
各層に別々の値を入れ、描画された位置がどの層の値と一致するかを見る。

撮影は replicator の rgb アノテータで行う。app.update だけではアノテータにデータが来ない(Play 中の captureOnPlay を除く)ので、
rep.orchestrator.step(pause_timeline=False, delta_time=0.0) で「タイムラインを進めずに今の状態を 1 枚描く」。
Play 中にこれを呼んでも時刻と PhysX の状態が変わらないことは事前に確かめた。captureOnPlay は無効にして、撮影の経路を 1 つにそろえる。

--physx-fabric / --gpu 付きで実行すると、その構成で同じ測定をする。

ケース:
    R-01  Stop 中に USD の xformOp:translate を書く
    R-02  Stop 中に Fabric の local 行列だけを書く(IFabricHierarchy.set_local_xform。world 行列は update まで古い)
    R-03  Stop 中に Fabric の omni:fabric:worldMatrix 属性だけを直接書く(local 行列はそのまま)
    R-04  Play 中、物理で動かす(速度 0.6)。描画は PhysX に追従するか
    R-05  Play 中に tensor で位置を書く
    R-06  Play 中に usd で位置を書き、Stop。Stop 後の描画は USD と Fabric のどちらか(physx_fabric で両者が食い違う場面)
    R-07  Pause 中に update を回してから再開(Q-92 で GPU 構成の Fabric が止まった手順)。描画は止まるか
    R-08  R-06 の Stop 後を 1 update ごとに 8 回撮影し、撮影の前後で各層を読む(3 回繰り返す)。
          physx_fabric 構成で R-06 の Stop 後の描画が実行ごとに USD 側 / Fabric 側と食い違ったので、時間変化を追う
"""

from __future__ import annotations

import bplib

app = bplib.start(render=True)

from typing import Any

import carb.settings
import numpy as np
import omni.replicator.core as rep
from isaacsim.core.experimental.prims import RigidPrim
from pxr import Gf, Usd, UsdGeom, UsdLux

rec = bplib.Recorder("r01_render")
carb.settings.get_settings().set_bool("/omni/replicator/captureOnPlay", False)

RES: tuple[int, int] = (320, 320)  # 画像サイズ (幅, 高さ)
CAMERA: str = "/World/Camera"
CUBE_SIZE: float = 0.4  # 撮影用に M01 より大きい立方体 [m]
WARMUP_UPDATES: int = 60  # 立方体が赤く写るまで(マテリアルの準備など)撮影を繰り返す上限回数
SETTLE_UPDATES: int = 3  # 書き込みから撮影までに回す update 数(描画は数フレーム遅れうる)

_annotator: Any = None
_render_product: Any = None
_calib: dict[str, np.ndarray] = {}  # 画素重心 → 世界 xy の線形変換


def build_scene() -> Usd.Stage:
    """重力 0 のステージに、赤い立方体(剛体)、真上のカメラ、平行光源を置く。"""
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    cube = UsdGeom.Cube(stage.GetPrimAtPath(bplib.BODY))
    cube.GetSizeAttr().Set(CUBE_SIZE)
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])
    cam = UsdGeom.Camera.Define(stage, CAMERA)
    cam.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 10.0))  # 既定で -Z を向くので真下を見る
    cam.CreateFocalLengthAttr(24.0)
    cam.CreateHorizontalApertureAttr(36.0)
    cam.CreateVerticalApertureAttr(36.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1000.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 0.0))  # 真上からだと上面の鏡面反射がカメラに入って白飛びするので傾ける
    bplib.update()
    return stage


def attach_camera() -> None:
    """カメラに rgb アノテータを付ける(ステージを作り直すたびに呼ぶ)。"""
    global _annotator, _render_product
    if _annotator is not None:
        _annotator.detach()
    if _render_product is not None:
        _render_product.destroy()
    _render_product = rep.create.render_product(CAMERA, RES)
    _annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    _annotator.attach([_render_product])


def red_centroid(n_updates: int = SETTLE_UPDATES) -> list[float] | None:
    """n_updates 回 update してから撮影し、赤い画素の重心 (u, v) を返す。赤い画素が無ければ None。"""
    bplib.update(n_updates)
    rep.orchestrator.step(rt_subframes=1, pause_timeline=False, delta_time=0.0)
    img = np.asarray(_annotator.get_data())
    if img.ndim != 3 or img.size == 0:
        return None
    r, g, b = img[..., 0].astype(int), img[..., 1].astype(int), img[..., 2].astype(int)
    mask = (r > 60) & (r - g > 40) & (r - b > 40)
    if mask.sum() < 5:
        return None
    vs, us = np.nonzero(mask)
    return [float(us.mean()), float(vs.mean())]


def to_world(c: list[float] | None) -> list[float] | None:
    """画素重心を、キャリブレーションで求めた線形変換で世界 xy に直す。"""
    if c is None or not _calib:
        return None
    rel = np.asarray(c) - _calib["origin"]
    xy = np.linalg.solve(_calib["basis"].T, rel)
    return [round(float(xy[0]), 3), round(float(xy[1]), 3)]


def rendered(n_updates: int = SETTLE_UPDATES) -> dict[str, Any]:
    """撮影して、画素重心と推定した世界 xy を返す。"""
    c = red_centroid(n_updates)
    return {"pixel": None if c is None else [round(v, 1) for v in c], "world_xy": to_world(c)}


def layers(stage: Usd.Stage) -> dict[str, Any]:
    """比較用に、各層の xy を返す(Fabric は CPU 側と cuda 側の両方)。"""
    s = bplib.snapshot(stage)
    dev = bplib.fabric_device_state(bplib.BODY, "cuda:0")
    xy = lambda v: None if not isinstance(v, list) else [round(v[0], 3), round(v[1], 3)]  # noqa: E731
    return {
        "usd": xy(s["usd"]["translate"]),
        "fabric_local": xy(s["fabric"].get("local")),
        "fabric_world_cache": xy(s["fabric"].get("world_cache")),
        "fabric_cuda_world": xy(dev.get("world")),
        "tensor": xy(s["tensor"].get("position")),
    }


def observe(stage: Usd.Stage, n_updates: int = SETTLE_UPDATES) -> dict[str, Any]:
    out = rendered(n_updates)
    out["layers"] = layers(stage)
    return out


def fresh() -> Usd.Stage:
    stage = build_scene()
    attach_camera()
    bplib.update(10)
    return stage


@bplib.guarded(rec, "R-00")
def calibrate() -> None:
    """USD で既知の位置に置いて撮影し、画素 → 世界 xy の変換を作る(Stop 中なので USD = Fabric = 描画のはず)。"""
    stage = fresh()
    warmup = 0
    while red_centroid(10) is None and warmup < WARMUP_UPDATES:  # マテリアルが効いて赤く写るまで待つ
        warmup += 1
    pts: dict[str, list[float] | None] = {}
    for name, pos in [("origin", (0.0, 0.0, 1.0)), ("x1", (1.0, 0.0, 1.0)), ("y1", (0.0, 1.0, 1.0))]:
        bplib.usd_set_translate(stage, bplib.BODY, pos)
        pts[name] = red_centroid(10)
    if all(v is not None for v in pts.values()):
        o = np.asarray(pts["origin"])
        _calib["origin"] = o
        _calib["basis"] = np.stack([np.asarray(pts["x1"]) - o, np.asarray(pts["y1"]) - o])  # 行: x 方向, y 方向 [px/m]
    check: dict[str, Any] = {}
    bplib.usd_set_translate(stage, bplib.BODY, (-0.5, 0.7, 1.0))
    check["usd(-0.5,0.7)"] = rendered(10)
    rec.add(
        "R-00", "キャリブレーション(USD で既知の位置に置いて撮影)", environment=bplib.environment(), warmup_captures=warmup, pixels=pts, check=check
    )


@bplib.guarded(rec, "R-01")
def stop_usd() -> None:
    stage = fresh()
    bplib.usd_set_translate(stage, bplib.BODY, (1.0, 0.5, 1.0))
    rec.add("R-01", "Stop 中に USD に (1, 0.5) を書く", result=observe(stage))


@bplib.guarded(rec, "R-02")
def stop_fabric_local() -> None:
    stage = fresh()
    bplib.fabric_set_translate(bplib.BODY, (-1.0, 0.5, 1.0))
    rec.add("R-02", "Stop 中に Fabric の local 行列だけ (-1, 0.5) にする", after_1_update=observe(stage, 1), after_3_updates=observe(stage, 2))


@bplib.guarded(rec, "R-03")
def stop_fabric_world_attr() -> None:
    import usdrt

    stage = fresh()
    prim = bplib._rt_stage().GetPrimAtPath(bplib.BODY)
    attr = prim.GetAttribute("omni:fabric:worldMatrix")
    m = attr.Get()
    m.SetTranslateOnly(usdrt.Gf.Vec3d(0.5, -1.0, 1.0))
    attr.Set(m)
    rec.add("R-03", "Stop 中に omni:fabric:worldMatrix 属性だけ (0.5, -1) にする(local 行列はそのまま)", after_1_update=observe(stage, 1), after_3_updates=observe(stage, 2))


@bplib.guarded(rec, "R-04")
def play_physics_motion() -> None:
    stage = fresh()
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    rp.set_velocities(linear_velocities=[[0.6, 0.0, 0.0]])
    trace = [observe(stage, 10) for _ in range(3)]  # 10 update(= 1/6 s, 0.1 m)ごとに撮影
    rec.add("R-04", "Play 中、速度 0.6 m/s で x 方向に動かす", trace=trace)


@bplib.guarded(rec, "R-05")
def play_tensor_write() -> None:
    stage = fresh()
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    rp.set_world_poses(positions=[[1.0, -0.5, 1.0]])
    rec.add("R-05", "Play 中に tensor で (1, -0.5) に置く", result=observe(stage))


@bplib.guarded(rec, "R-06")
def usd_write_then_stop() -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils

    stage = fresh()
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    with backend_utils.use_backend("usd"):
        rp.set_world_poses(positions=[[-1.0, -0.5, 1.0]])
    playing = observe(stage)
    bplib.stop(1)
    rec.add("R-06", "Play 中に usd で (-1, -0.5) に置き、Stop", playing=playing, stopped=observe(stage))


@bplib.guarded(rec, "R-07")
def pause_resume() -> None:
    stage = fresh()
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    rp.set_velocities(linear_velocities=[[0.6, 0.0, 0.0]])
    before = observe(stage, 10)
    bplib.timeline().pause()
    paused = observe(stage, 5)
    bplib.timeline().play()
    after = [observe(stage, 10) for _ in range(3)]
    rec.add("R-07", "速度 0.6 で動かし、Pause(update 5 回)→ 再開", before_pause=before, paused=paused, after_resume=after)


@bplib.guarded(rec, "R-08")
def usd_write_then_stop_trace() -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils

    out: dict[str, Any] = {}
    for trial in range(3):
        stage = fresh()
        rp = RigidPrim(bplib.BODY)
        bplib.play(3)
        with backend_utils.use_backend("usd"):
            rp.set_world_poses(positions=[[-1.0, -0.5, 1.0]])
        bplib.update(3)
        bplib.timeline().stop()  # update を回さずに撮影を始める
        trace: list[dict[str, Any]] = []
        for _ in range(8):
            before = layers(stage)
            shot = rendered(1)
            trace.append({"layers_before": before, "rendered_xy": shot["world_xy"], "layers_after": layers(stage)})
        out[f"trial{trial}"] = trace
    rec.add("R-08", "Play 中に usd で (-1, -0.5) に置き、Stop 後を 1 update ごとに撮影(3 回繰り返し)", **out)


calibrate()
for _case in [stop_usd, stop_fabric_local, stop_fabric_world_attr, play_physics_motion, play_tensor_write, usd_write_then_stop, pause_resume, usd_write_then_stop_trace]:
    _case()
bplib.finish(rec)
