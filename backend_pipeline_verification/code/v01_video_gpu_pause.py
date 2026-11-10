"""V01: 説明動画「GPU 構成で Pause → 再開すると描画が止まる」(§4.3 / Q-92 / X-A)。

skills/isaac-sim-visual-explainer の explainer.py で撮影・合成・書き出しをする。--gpu 付きで実行する:
    powershell -File run.ps1 -ScriptArgs '--gpu' v01_video_gpu_pause.py
出力: ../media/v01_gpu_pause.mp4 / .gif / 要所の .png
"""

from __future__ import annotations

import os
import sys

import bplib

app = bplib.start(render=True)

sys.path.insert(0, r"C:\isaacsim-6.2.0\skills\isaac-sim-visual-explainer\scripts")
from typing import Any

import carb.settings
from explainer import Explainer, Marker
from isaacsim.core.experimental.prims import RigidPrim
from pxr import Gf, Usd, UsdGeom, UsdLux

carb.settings.get_settings().set_bool("/omni/replicator/captureOnPlay", False)
MEDIA = os.path.join(os.path.dirname(bplib.HERE), "media")
ex = Explainer(out_dir=MEDIA, name="v01_gpu_pause" + bplib.SUFFIX)

START_X: float = -1.5
SPEED: float = 0.6  # [m/s]


def build() -> tuple[Usd.Stage, RigidPrim]:
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage, pos=(START_X, 0.0, 1.0))
    cube = UsdGeom.Cube(stage.GetPrimAtPath(bplib.BODY))
    cube.GetSizeAttr().Set(0.3)
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.1, 0.1)])
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")  # 物理なしの床(見た目だけ)
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))
    floor.AddScaleOp().Set(Gf.Vec3d(5.0, 2.0, 0.02))
    floor.CreateDisplayColorAttr().Set([Gf.Vec3f(0.35, 0.37, 0.42)])
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1500.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 20.0))
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(300.0)
    bplib.update(2)
    ex.add_camera(eye=(0.0, -2.8, 2.3), target=(-0.2, 0.0, 0.95))
    for _ in range(30):  # マテリアルとシェーダの準備
        bplib.update()
    ex.capture()
    return stage, RigidPrim(bplib.BODY)


def values(stage: Usd.Stage) -> dict[str, Any]:
    s = bplib.snapshot(stage)
    dev = bplib.fabric_device_state(bplib.BODY, "cuda:0")
    return {
        "physx": (s["tensor"].get("position") or [None] * 3),
        "fabric": dev.get("world") if isinstance(dev.get("world"), list) else [None] * 3,
        "usd": s["usd"]["translate"],
        "time": s["time"],
    }


def fmt(v: Any) -> str:
    return "−" if v is None else f"{v:+.2f} m"


def shoot(stage: Usd.Stage, title: str, caption: str, badge: str, hold: float = 0.0) -> Any:
    """今の状態を撮って、PhysX の位置の印と値の表を重ねた 1 フレームを足す。"""
    v = values(stage)
    markers = []
    if v["fabric"][0] is not None:
        markers.append(Marker((v["fabric"][0], 0.0, 1.25), "Fabric", "#ffc040", "square", 8))
    if v["physx"][0] is not None:
        markers.append(Marker((v["physx"][0], 0.0, 1.0), "PhysX", "#40ff80", "circle", 16))
    frame = ex.compose(
        ex.capture(),
        title=title,
        caption=caption,
        badge=badge,
        panel_title="x 座標（物体の中心）",
        rows=[
            ("PhysX（tensor で読む）", fmt(v["physx"][0])),
            ("Fabric（描画が読む、GPU 側）", fmt(v["fabric"][0])),
            ("USD", fmt(v["usd"][0])),
            ("タイムライン時刻", f"{v['time']:.2f} s"),
        ],
        markers=markers,
    )
    ex.add(frame, hold if hold else 1.0 / ex.fps)
    return frame


def run(stage: Usd.Stage, rp: RigidPrim, title: str, pause_updates: int, resume_note: str) -> dict[str, Any]:
    """Play → Pause(update を pause_updates 回)→ 再開 の流れを撮る。要所のフレームを返す。"""
    keys: dict[str, Any] = {}
    bplib.timeline().play()
    bplib.update(1)
    rp.set_velocities(linear_velocities=[[SPEED, 0.0, 0.0]])
    for i in range(30):
        bplib.update(1)
        f = shoot(stage, title, "Play 中：物理が進み、Fabric（＝描画）も PhysX に追従している。", "Play 中")
    keys["play"] = f
    bplib.timeline().pause()
    if pause_updates:
        for i in range(pause_updates):
            bplib.update(1)
            f = shoot(stage, title, f"Pause 中に update を回している（{i + 1} 回目）。\n物理は止まっている。", "Pause 中")
    else:
        # 撮影(rep.orchestrator.step)も app の update を回すので、Pause 中は撮らない。直前の絵に説明だけ付ける
        f = keys["play"]
        ex.title_card("Pause → すぐ再開", ["Pause 中に update を回さない（撮影もしない）。"], seconds=1.5)
    keys["pause"] = f
    bplib.timeline().play()
    for i in range(45):
        bplib.update(1)
        f = shoot(stage, title, resume_note, "再開後")
    keys["resumed"] = f
    return keys


ex.title_card(
    "GPU 構成で Pause → 再開すると描画が止まる",
    [
        "SimulationManager.set_device('cuda:0') の構成（GPU 物理 + omni.physx.fabric）",
        "赤い立方体を 0.6 m/s で動かし、Pause → 再開する。",
        "緑の丸＝PhysX の実際の位置、黄の四角（立方体の上）＝Fabric の位置。右の表は各層の x 座標。",
    ],
    seconds=4.0,
)
stage, rp = build()
k1 = run(
    stage,
    rp,
    "① Pause 中に update を回してから再開",
    pause_updates=8,
    resume_note="再開後：PhysX（緑の丸）は進み続けるが、Fabric と描画は Pause 時の位置で止まったまま。",
)
ex.save_still(k1["resumed"], "frozen")
bplib.stop(1)
ex.title_card("② Stop → Play し直すと戻る", ["Stop で omni.physx.fabric の記録が消え、次の Play で剛体が登録し直される。"], seconds=2.5)
bplib.timeline().play()
bplib.update(1)
rp.set_velocities(linear_velocities=[[SPEED, 0.0, 0.0]])
for _ in range(30):
    bplib.update(1)
    f = shoot(stage, "② Stop → Play し直した後", "描画が PhysX に追従している。", "Play 中")
ex.save_still(f, "recovered")
bplib.stop(1)
ex.title_card("③ 比較：Pause 中に update を回さない場合", ["PhysX の PAUSED / RESUMED イベント自体が出ないので、止まらない。"], seconds=2.5)
k3 = run(
    stage,
    rp,
    "③ Pause 中に update を回さずに再開",
    pause_updates=0,
    resume_note="再開後：描画は PhysX に追従し続ける。",
)
ex.save_still(k3["resumed"], "not_frozen")
bplib.stop(1)
ex.title_card(
    "原因（omni.physx.fabric のソース、GitHub NVIDIA-Omniverse/PhysX）",
    [
        "1. PAUSED で FabricManager::pause() が DirectGpuHelper の剛体一覧を空にする",
        "2. RESUMED で resume() が登録し直そうとするが、",
        "    Play 開始時の記録（mInitialTransformation）がある剛体は「初期化済み」で飛ばす",
        "3. 一覧が空のままなので、GPU から Fabric に何も書かれない（記録は stop() でだけ消える）",
    ],
    seconds=7.0,
)
print("[VIDEO]", ex.write(), flush=True)
bplib._app.close()
