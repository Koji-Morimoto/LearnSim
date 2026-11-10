"""D01: DeformablePrim(体積変形体)で、Play ON / OFF × バックエンド指定 × 書き換え系メソッドを測る(M01 の変形体版)。

変形体は GPU 物理でしか動かないので --gpu 付きでだけ実行する(付けなければ何もせず記録だけ残す)。

変形体(重力 0):
    /World/Def  TetMesh(四面体 1 個、頂点は ±1 の立方体の 4 隅)。xformOp translate=(0,0,3) / orient / scale
                deformableUtils.set_physics_volume_deformable_body で体積変形体にする

マトリクス:
    開始状態: stopped / playing
    メソッド × バックエンド:
        set_world_poses(positions=(1,2,3))      : 既定 / tensor / usd / usdrt / fabric   (tensor は非対応 → フォールバック)
        set_nodal_positions(全節点を +x に 0.5) : 既定 / usd   (tensor 専用)
        set_nodal_velocities((0,0.6,0))         : 既定 / usd   (tensor 専用)
手順は M01 と同じ。

追加:
    D-90  Play 中に usd で変形体の姿勢を書くと simulation view(全 tensor ビューの親)が無効になるか。
          同じステージの剛体(RigidPrim)の tensor ビューも巻き込まれるか、物理は続くか

観測(ラッパーを通さない):
    USD    : prim のワールド位置、points 属性の平均(prim のローカル座標)
    Fabric : prim のワールド位置(IFabricHierarchy)、Fabric 上の points の平均
    tensor : 検証用 VolumeDeformableBodyView のシミュレーション節点位置の平均(ワールド座標)と節点速度の平均
"""

from __future__ import annotations

import bplib

app = bplib.start()

from typing import Any, Callable

import numpy as np
from pxr import Gf, Usd, UsdGeom, Vt

rec = bplib.Recorder("d01_deformable")

DEF: str = "/World/Def"
START: tuple[float, float, float] = (0.0, 0.0, 3.0)
TARGET_POS: list[float] = [1.0, 2.0, 3.0]
NODE_SHIFT_X: float = 0.5  # set_nodal_positions で全節点を +x に動かす量 [m]
NODE_VEL: list[float] = [0.0, 0.6, 0.0]  # set_nodal_velocities で全節点に与える速度 [m/s]


def build() -> Usd.Stage:
    from omni.physx.scripts import deformableUtils

    stage = bplib.new_stage(gravity=0.0)
    tet = UsdGeom.TetMesh.Define(stage, DEF)
    tet.GetPointsAttr().Set(Vt.Vec3fArray([[1.0, 1.0, 1.0], [-1.0, 1.0, -1.0], [1.0, -1.0, -1.0], [-1.0, -1.0, 1.0]]))
    tet.GetTetVertexIndicesAttr().Set(Vt.Vec4iArray([[0, 1, 2, 3]]))
    tet.GetSurfaceFaceVertexIndicesAttr().Set(UsdGeom.TetMesh.ComputeSurfaceFaces(tet, Usd.TimeCode.Default()))
    tet.GetExtentAttr().Set(Vt.Vec3fArray([[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]]))
    tet.AddTranslateOp().Set(Gf.Vec3d(*START))
    tet.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    tet.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    assert deformableUtils.set_physics_volume_deformable_body(stage, DEF), "体積変形体の設定に失敗"
    bplib.update()
    return stage


_view_cache: list[Any] = [None, None]


def def_view() -> Any | None:
    """検証用の VolumeDeformableBodyView。simulation view が無効(Stop 中)なら None。"""
    from isaacsim.core.simulation_manager import SimulationManager

    sv = SimulationManager._physics_sim_view__warp
    if sv is None or not sv.is_valid:
        return None
    if _view_cache[0] is not sv:
        _view_cache[0], _view_cache[1] = sv, sv.create_volume_deformable_body_view(DEF)
    return _view_cache[1]


def _mean(points: Any) -> list[float] | None:
    if points is None or len(points) == 0:
        return None
    return bplib._r(np.asarray(points, dtype=float).reshape(-1, 3).mean(axis=0))


def snap(stage: Usd.Stage) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(DEF)
    out: dict[str, Any] = {
        "timeline": bplib.timeline_state(),
        "usd": {
            "world": bplib._r(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()),
            "points_mean_local": _mean(prim.GetAttribute("points").Get()),
        },
    }
    try:
        rt_prim = bplib._rt_stage().GetPrimAtPath(DEF)
        attr = rt_prim.GetAttribute("points")
        out["fabric"] = {
            "world": bplib.fabric_state(DEF).get("world"),
            "points_mean_local": _mean([list(p) for p in attr.Get()]) if attr and attr.HasValue() else None,
        }
    except Exception as e:
        out["fabric"] = {"error": f"{type(e).__name__}: {e}"}
    v = def_view()
    if v is None:
        out["tensor"] = {"valid": False}
    else:
        try:
            out["tensor"] = {
                "valid": True,
                "nodes_mean_world": _mean(v.get_simulation_nodal_positions().numpy()[0]),
                "nodes_vel_mean": _mean(v.get_simulation_nodal_velocities().numpy()[0]),
            }
        except Exception as e:  # GPU 物理では最初のステップ前に読めない
            out["tensor"] = {"valid": True, "error": f"{type(e).__name__}: {e}"}
    return out


def shift_nodes(dp: Any) -> None:
    """現在の節点位置を tensor で読み、+x に NODE_SHIFT_X ずらして書く。Stop 中は読めないので例外になる。

    get_nodal_positions は (シミュレーション, 衝突, 静止) メッシュの 3 つ組を返すので、先頭を使う。
    """
    pos = dp.get_nodal_positions()[0].numpy()
    pos[..., 0] += NODE_SHIFT_X
    dp.set_nodal_positions(pos)


def set_vel(dp: Any) -> None:
    dp.set_nodal_velocities(np.broadcast_to(np.asarray(NODE_VEL, dtype=np.float32), (1, dp.num_nodes_per_body[0], 3)).copy())


OPS: dict[str, tuple[Callable[[Any], None], list[str | None]]] = {
    "set_world_poses": (lambda dp: dp.set_world_poses(positions=[TARGET_POS]), [None, "tensor", "usd", "usdrt", "fabric"]),
    "set_nodal_positions": (shift_nodes, [None, "usd"]),
    "set_nodal_velocities": (set_vel, [None, "usd"]),
}


def run_case(start: str, op: str, backend: str | None) -> dict[str, Any]:
    from isaacsim.core.experimental.prims import DeformablePrim

    stage = build()
    dp = DeformablePrim(DEF, deformable_type="volume")
    fn = OPS[op][0]
    res: dict[str, Any] = {}
    if start == "playing":
        bplib.play(3)
    res["0_before"] = snap(stage)
    res["call"] = bplib.call_with_backend([dp], lambda: fn(dp), backend)
    res["1_after_call"] = snap(stage)
    bplib.update()
    res["2_after_update1"] = snap(stage)
    if start == "stopped":
        bplib.play(1)
        res["3_toggled_play_update1"] = snap(stage)
        bplib.update(9)
        res["4_play_update10"] = snap(stage)
        bplib.stop(1)
        res["5_stopped_again"] = snap(stage)
    else:
        bplib.stop(1)
        res["3_toggled_stop"] = snap(stage)
        bplib.play(1)
        res["4_replay_update1"] = snap(stage)
        bplib.update(9)
        res["5_replay_update10"] = snap(stage)
    return res


@bplib.guarded(rec, "D-00")
def env() -> None:
    rec.add("D-00", "バージョンと設定", **bplib.environment())


def make_case(start: str, op: str, backend: str | None) -> Callable[[], None]:
    test_id = f"D/{start}/{op}/{backend or '(既定)'}"

    @bplib.guarded(rec, test_id)
    def case() -> None:
        rec.add(test_id, f"開始={start} メソッド={op} use_backend={backend}", **run_case(start, op, backend))

    return case


@bplib.guarded(rec, "D-90")
def usd_pose_write_invalidates_view() -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils
    from isaacsim.core.experimental.prims import DeformablePrim, RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager

    stage = build()
    bplib.make_body(stage)  # 剛体 /World/Body(z=1)
    bplib.update()
    dp = DeformablePrim(DEF, deformable_type="volume")
    rp = RigidPrim(bplib.BODY)
    bplib.play(3)
    rp.set_velocities(linear_velocities=[[0.0, 0.6, 0.0]])
    bplib.update(3)

    def state(tag: str) -> dict[str, Any]:
        sv = SimulationManager._physics_sim_view__warp
        out: dict[str, Any] = {
            "sim_view_valid": bool(sv is not None and sv.is_valid),
            "deformable_wrapper_tensor_valid": dp.is_physics_tensor_entity_valid(),
            "rigid_wrapper_tensor_valid": rp.is_physics_tensor_entity_valid(),
            "rigid_fabric": bplib.fabric_state(bplib.BODY).get("local"),
            "time": round(bplib.timeline().get_current_time(), 4),
        }
        with bplib.LogCapture():
            try:
                out["rigid_readback_position"] = bplib._r(rp.get_world_poses()[0].numpy()[0])
            except Exception as e:
                out["rigid_readback_position"] = f"{type(e).__name__}: {e}"
        return out

    res: dict[str, Any] = {"0_before": state("before")}
    with bplib.LogCapture() as lc:
        with backend_utils.use_backend("usd"):
            dp.set_world_poses(positions=[TARGET_POS])
    res["1_after_call"] = state("after_call")
    bplib.update(1)
    res["2_after_update1"] = state("update1")
    bplib.update(5)
    res["3_after_update6"] = state("update6")
    res["warnings"] = lc.messages
    rec.add("D-90", "Play 中に usd で変形体の姿勢を書く → simulation view と剛体の tensor ビューはどうなるか", **res)


env()
if not bplib.GPU:
    rec.add("D-SKIP", "変形体は GPU 物理でしか動かないので --gpu なしでは測らない")
else:
    for _start in ["stopped", "playing"]:
        for _op, (_fn, _backends) in OPS.items():
            for _b in _backends:
                make_case(_start, _op, _b)()
    usd_pose_write_invalidates_view()
bplib.finish(rec)
