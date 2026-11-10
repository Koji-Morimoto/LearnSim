"""isaacsim.core.experimental の「バックエンド」(usd / usdrt / fabric / tensor)を調べる共通ライブラリ。

使い方:
    import bplib
    app = bplib.start()      # pxr / omni.* / isaacsim.* の import より先に必ず呼ぶ
    ...
    bplib.finish(rec)

実行時オプション:
    --physx-fabric   omni.physx.fabric を有効にして実行する(PhysX の結果の書き込み先が USD から Fabric に変わる構成)。
                     結果ファイル名に "__physx_fabric" が付く。
    --gpu            GPU 物理で実行する(SimulationManager.set_device("cuda:0")。omni.physx.fabric も自動で有効になる)。
                     結果ファイル名に "__gpu" が付く。
    --no-fsd         Fabric Scene Delegate を無効にして起動する(--/app/useFabricSceneDelegate=false)。
                     結果ファイル名に "__no_fsd" が付く。

観測の方針:
    検証対象のラッパー(RigidPrim など)を通さず、各層を下位 API で直接読む。
        USD     : pxr.Usd       (xformOp:translate / physics:velocity / physics:mass の属性値)
        Fabric  : usdrt         (IFabricHierarchy の local / world 行列、Fabric 上の属性値)
        PhysX   : omni.physx    (get_rigidbody_transformation。PhysX 内部の剛体の姿勢)
        tensor  : omni.physics.tensors (検証用に別途作る RigidBodyView。PhysX 内部の値を配列で読む)
    これで「ラッパーで書いた値が、どの層に、いつ届いたか」を、ラッパー自身の読み出しに頼らずに確かめる。
"""

from __future__ import annotations

import contextlib
import json
import os
import sys
import threading
import time
import traceback
import types
from collections.abc import Iterator
from typing import TYPE_CHECKING, Any, Callable

if TYPE_CHECKING:
    # 型ヒント専用。実行時の pxr import は SimulationApp 起動後でないとできない。
    from pxr import Usd

_app: Any = None  # isaacsim.SimulationApp。start() で設定する

HERE: str = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR: str = os.path.join(os.path.dirname(HERE), "results")

PHYSX_FABRIC: bool = "--physx-fabric" in sys.argv  # omni.physx.fabric を有効にする構成で走らせるか
GPU: bool = "--gpu" in sys.argv  # GPU 物理で走らせるか
NO_FSD: bool = "--no-fsd" in sys.argv  # Fabric Scene Delegate を無効にして起動するか
SUFFIX: str = ("__physx_fabric" if PHYSX_FABRIC else "") + ("__gpu" if GPU else "") + ("__no_fsd" if NO_FSD else "")

BODY: str = "/World/Body"  # 動的剛体(立方体)
MARKER: str = "/World/Marker"  # 物理を持たない Xform
BODY_START: tuple[float, float, float] = (0.0, 0.0, 1.0)  # 剛体の初期位置 [m]
BODY_MASS: float = 1.0  # 剛体の初期質量 [kg]

Vec3 = tuple[float, float, float]


# ------------------------------------------------------------------ 起動・記録


def start(timeout_s: float = 1200.0, render: bool = False) -> Any:
    """headless の SimulationApp を起動して返す。

    ハングしても残らないよう、timeout_s 経過で自プロセスを終了するウォッチドッグを付ける。
    --physx-fabric 指定時は、起動後に SimulationManager.enable_fabric(True) で omni.physx.fabric を有効にする。
    --gpu 指定時は、起動後に SimulationManager.set_device("cuda:0") で GPU 物理にする。

    Args:
        timeout_s: ウォッチドッグの秒数。
        render: True なら描画あり(RTX と replicator を含む既定のエクスペリエンス)で起動する。
            False なら描画なしの軽量エクスペリエンス bp_verify.kit で起動する。
    """
    global _app

    def _die() -> None:
        print(f"[WATCHDOG] {timeout_s}s 経過のため終了", flush=True)
        os._exit(3)

    t = threading.Timer(timeout_s, _die)
    t.daemon = True
    t.start()

    from isaacsim import SimulationApp

    config: dict[str, Any] = {"headless": True}
    if NO_FSD:
        config["extra_args"] = ["--/app/useFabricSceneDelegate=false"]
    if render:
        _app = SimulationApp(config)
    else:
        _app = SimulationApp(config, experience=os.path.join(HERE, "bp_verify.kit"))
    from isaacsim.core.simulation_manager import SimulationManager

    if PHYSX_FABRIC:
        SimulationManager.enable_fabric(True)
        _app.update()
    if GPU:
        SimulationManager.set_device("cuda:0")
        _app.update()
    return _app


def update(n: int = 1) -> None:
    """Kit のメインループを n 回まわす(タイムラインが Play 中なら、そのたびに物理が 1 ステップ進む)。"""
    for _ in range(n):
        _app.update()


class Recorder:
    """測定結果を results/<group><SUFFIX>.json に逐次保存する。"""

    def __init__(self, group: str) -> None:
        self.group: str = group + SUFFIX
        self.items: list[dict[str, Any]] = []
        os.makedirs(RESULT_DIR, exist_ok=True)
        self.path: str = os.path.join(RESULT_DIR, f"{self.group}.json")

    def add(self, test_id: str, description: str, **data: Any) -> None:
        item = {"id": test_id, "description": description, **data}
        self.items.append(item)
        print(f"[RESULT] {test_id}: " + json.dumps(data, ensure_ascii=False, default=str)[:1500], flush=True)
        self.save()

    def save(self) -> None:
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"group": self.group, "items": self.items}, f, ensure_ascii=False, indent=1, default=str)


def guarded(rec: Recorder, test_id: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
    """テスト関数の例外を記録して握りつぶすデコレータ。1 件の失敗で残りを止めない。

    終了時は必ず cleanup() を呼び、次のテストへタイムラインの状態を持ち越さない。
    """

    def deco(fn: Callable[[], None]) -> Callable[[], None]:
        def wrapper() -> None:
            print(f"[START] {test_id}", flush=True)
            t0 = time.perf_counter()
            try:
                fn()
            except Exception:
                rec.add(test_id + "!EXC", "例外", exception=traceback.format_exc())
            finally:
                cleanup()
                print(f"[END] {test_id} {time.perf_counter() - t0:.1f}s", flush=True)

        return wrapper

    return deco


def finish(rec: Recorder) -> None:
    rec.save()
    print("[DONE]", rec.group, len(rec.items), flush=True)
    sys.stdout.flush()
    _app.close()


def environment() -> dict[str, Any]:
    """記録用に、関係する拡張のバージョンと設定値を返す。"""
    import carb.settings
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    exts = [
        "omni.physx",
        "omni.physx.fabric",
        "omni.physics.tensors",
        "usdrt.scenegraph",
        "omni.hydra.usdrt_delegate",
        "isaacsim.core.experimental.prims",
        "isaacsim.core.experimental.utils",
        "isaacsim.core.simulation_manager",
    ]
    out: dict[str, Any] = {n: str(mgr.get_enabled_extension_id(n) or "(disabled)") for n in exts}
    with open(r"C:\isaacsim-6.2.0\VERSION", encoding="utf-8") as f:
        out["isaac_sim_VERSION"] = f.read().strip()
    s = carb.settings.get_settings()
    for key in [
        "/app/useFabricSceneDelegate",
        "/physics/resetOnStop",
        "/physics/updateToUsd",
        "/physics/updateVelocitiesToUsd",
        "/physics/fabricEnabled",
        "/physics/suppressReadback",
        "/physics/cudaDevice",
    ]:
        out[key] = s.get(key)
    out["config"] = ("gpu" if GPU else ("physx_fabric" if PHYSX_FABRIC else "default")) + ("+no_fsd" if NO_FSD else "")
    from isaacsim.core.simulation_manager import SimulationManager

    out["simulation_device"] = str(SimulationManager.get_device())
    return out


# ------------------------------------------------------------------ carb ログの捕捉


class LogCapture:
    """with ブロックの間に carb.log_warn / carb.log_error へ渡されたメッセージを記録する。

    isaacsim.core.experimental の各モジュールは `carb.log_warn(...)` を属性参照で呼ぶので、
    carb モジュールの属性を差し替えれば捕まえられる(元の関数にも転送するのでログ出力は変わらない)。
    """

    def __init__(self) -> None:
        self.messages: list[str] = []
        self._orig: dict[str, Callable[..., Any]] = {}

    def __enter__(self) -> LogCapture:
        import carb

        for level in ["log_warn", "log_error"]:
            orig = getattr(carb, level)
            self._orig[level] = orig

            def hook(msg: str, *a: Any, _lv: str = level, _orig: Callable[..., Any] = orig, **k: Any) -> Any:
                self.messages.append(f"{_lv[4:].upper()}: {msg}")
                return _orig(msg, *a, **k)

            setattr(carb, level, hook)
        return self

    def __exit__(self, *exc: Any) -> None:
        import carb

        for level, orig in self._orig.items():
            setattr(carb, level, orig)


# ------------------------------------------------------------------ タイムライン


def timeline() -> Any:
    import omni.timeline

    return omni.timeline.get_timeline_interface()


def timeline_state() -> str:
    tl = timeline()
    if tl.is_playing():
        return "playing"
    if tl.is_stopped():
        return "stopped"
    return "paused"


def play(n_updates: int = 1) -> None:
    """Play して n_updates 回 update する(1 回目の update で物理の初期化と tensor ビューの作成が走る)。"""
    timeline().play()
    update(n_updates)


def stop(n_updates: int = 1) -> None:
    timeline().stop()
    update(n_updates)


def cleanup() -> None:
    """タイムラインを止める。ステージは次のテストの new_stage() で作り直す。"""
    if not timeline().is_stopped():
        stop()


# ------------------------------------------------------------------ ステージ構築


def new_stage(gravity: float = 0.0, gpu: bool | None = None) -> Usd.Stage:
    """Z-up、1 unit = 1 m の空ステージに PhysicsScene を作って返す(--gpu なら GPU 実行、それ以外は CPU 実行)。

    Args:
        gravity: 重力加速度の大きさ [m/s^2]。0 なら剛体は外から速度を与えない限り動かない。
        gpu: PhysicsScene を GPU 実行にするか。None なら --gpu の指定に従う。
    """
    if gpu is None:
        gpu = GPU
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    ctx = omni.usd.get_context()
    ctx.new_stage()
    update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(gpu)
    api.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    update()
    return stage


def make_body(stage: Usd.Stage, path: str = BODY, pos: Vec3 = BODY_START, mass: float = BODY_MASS) -> None:
    """一辺 0.2 m の動的剛体(立方体)を作る。xformOp は translate / orient / scale の 3 つ。"""
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(0.2)
    cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
    cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    cube.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)


def make_marker(stage: Usd.Stage, path: str = MARKER, pos: Vec3 = (0.0, 0.0, 0.0)) -> None:
    """物理を持たない Xform を作る(USD ↔ Fabric の同期だけを見るため)。

    xformOp は剛体と同じ translate / orient / scale の 3 つにする。
    XformPrim の usd バックエンドは orient が無いと set_world_poses で AssertionError になるため
    (usdrt バックエンドは xformOp を見ないので影響しない。N-03 の初回実行で確認)。
    """
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    xf.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))


def usd_set_translate(stage: Usd.Stage, path: str, pos: Vec3) -> None:
    """pxr で xformOp:translate を直接書く(ラッパーを通さない USD 書き込み)。"""
    from pxr import Gf

    stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))


def fabric_set_translate(path: str, pos: Vec3) -> None:
    """IFabricHierarchy で Fabric 上の local 行列の平行移動成分だけを書く(ラッパーを通さない Fabric 書き込み)。"""
    import usdrt

    h = _fabric_hierarchy()
    p = usdrt.Sdf.Path(path)
    m = h.get_local_xform(p)
    m.SetTranslateOnly(usdrt.Gf.Vec3d(*pos))
    h.set_local_xform(p, m)


# ------------------------------------------------------------------ 観測


def _r(v: Any, nd: int = 4) -> Any:
    """ベクトル・スカラーを丸めて JSON にしやすい形にする。None はそのまま。"""
    if v is None:
        return None
    try:
        return [round(float(x), nd) for x in v]
    except TypeError:
        return round(float(v), nd)


def _stage_id(stage: Usd.Stage) -> int:
    from pxr import UsdUtils

    cache = UsdUtils.StageCache.Get()
    sid = cache.GetId(stage).ToLongInt()
    if sid < 0:
        sid = cache.Insert(stage).ToLongInt()
    return sid


def _rt_stage() -> Any:
    """現在の USD ステージに対応する usdrt.Usd.Stage(= Fabric 上のステージ)を返す。"""
    import omni.usd
    import usdrt

    return usdrt.Usd.Stage.Attach(_stage_id(omni.usd.get_context().get_stage()))


def _fabric_hierarchy() -> Any:
    import usdrt

    rt = _rt_stage()
    return usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(rt.GetFabricId(), rt.GetStageIdAsStageId())


def usd_state(stage: Usd.Stage, path: str) -> dict[str, Any]:
    """USD(pxr)上の値。translate は xformOp:translate の値、world は親を含めて計算したワールド位置。"""
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    out: dict[str, Any] = {
        "translate": _r(prim.GetAttribute("xformOp:translate").Get()),
        "world": _r(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()),
    }
    for name, key in [("physics:velocity", "velocity"), ("physics:mass", "mass")]:
        attr = prim.GetAttribute(name)
        out[key] = _r(attr.Get()) if attr and attr.HasAuthoredValue() else None
    return out


def fabric_state(path: str) -> dict[str, Any]:
    """Fabric(usdrt)上の値。

    local       : IFabricHierarchy.get_local_xform(= omni:fabric:localMatrix)の平行移動成分
    world       : IFabricHierarchy.get_world_xform(その場で計算した正しいワールド行列)の平行移動成分
    world_cache : omni:fabric:worldMatrix 属性の値(update_world_xforms まで更新されないキャッシュ。描画はこれを使う)
    velocity / mass / translate_attr : Fabric に載っている同名属性の値(無ければ None)
    """
    import usdrt

    try:
        rt = _rt_stage()
        h = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(rt.GetFabricId(), rt.GetStageIdAsStageId())
        p = usdrt.Sdf.Path(path)
        prim = rt.GetPrimAtPath(path)
        out: dict[str, Any] = {
            "local": _r(h.get_local_xform(p).ExtractTranslation()),
            "world": _r(h.get_world_xform(p).ExtractTranslation()),
        }
        cache = prim.GetAttribute("omni:fabric:worldMatrix")
        out["world_cache"] = _r(cache.Get().ExtractTranslation()) if cache and cache.HasValue() else None
        for name, key in [("physics:velocity", "velocity"), ("physics:mass", "mass"), ("xformOp:translate", "translate_attr")]:
            attr = prim.GetAttribute(name)
            out[key] = _r(attr.Get()) if attr and attr.HasValue() else None
        return out
    except Exception as e:  # Fabric にまだ prim が無いときなど
        return {"error": f"{type(e).__name__}: {e}"}


_PROBE_ATTR: str = "bpProbe"  # fabric_device_state が対象 prim を選ぶための目印属性(Fabric にだけ作る)
_translation_kernel: Any = None


def _get_translation_kernel() -> Any:
    """Fabric の 4x4 行列(行優先、平行移動は 4 行目)から平行移動成分を取り出す warp カーネルを返す。"""
    global _translation_kernel
    if _translation_kernel is None:
        import warp as wp

        @wp.kernel(enable_backward=False)
        def _wk_translation(m: wp.fabricarray(dtype=wp.mat44d), out: wp.array(dtype=wp.vec3d)) -> None:
            i = wp.tid()
            out[i] = wp.vec3d(m[i][3, 0], m[i][3, 1], m[i][3, 2])

        _translation_kernel = _wk_translation
    return _translation_kernel


def fabric_device_state(path: str, device: str = "cuda:0") -> dict[str, Any]:
    """Fabric 上の行列を、指定デバイス側のデータとして warp の fabricarray で読む。

    fabric_state は usdrt の CPU 側の読み出し。GPU 物理 + omni.physx.fabric では PhysX の結果が
    GPU 側の Fabric に書かれるので、描画(GPU)が見る値は device="cuda:0" で読んで確かめる。

    Returns:
        world : omni:fabric:worldMatrix の平行移動成分
        local : omni:fabric:localMatrix の平行移動成分
    """
    import usdrt
    import warp as wp

    try:
        rt = _rt_stage()
        prim = rt.GetPrimAtPath(path)
        if not prim.HasAttribute(_PROBE_ATTR):
            prim.CreateAttribute(_PROBE_ATTR, usdrt.Sdf.ValueTypeNames.UInt, True).Set(0)
        out: dict[str, Any] = {"device": device}
        for key, attr in [("world", "omni:fabric:worldMatrix"), ("local", "omni:fabric:localMatrix")]:
            sel = rt.SelectPrims(
                require_attrs=[
                    (usdrt.Sdf.ValueTypeNames.UInt, _PROBE_ATTR, usdrt.Usd.Access.Read),
                    (usdrt.Sdf.ValueTypeNames.Matrix4d, attr, usdrt.Usd.Access.Read),
                ],
                device=device,
            )
            if sel.GetCount() != 1:
                out[key] = f"selection count={sel.GetCount()}"
                continue
            buf = wp.zeros(1, dtype=wp.vec3d, device=device)
            wp.launch(_get_translation_kernel(), dim=1, inputs=[wp.fabricarray(sel, attrib=attr), buf], device=device)
            out[key] = _r(buf.numpy()[0])
        return out
    except Exception as e:
        return {"error": f"{type(e).__name__}: {e}"}


def phys_sim() -> Any:
    """IPhysxSimulation(pause_change_tracking / is_change_tracking_paused などを持つ)。"""
    from omni.physx import get_physx_simulation_interface

    return get_physx_simulation_interface()


def physx_state(path: str) -> dict[str, Any]:
    """PhysX 内部の剛体姿勢。PhysX にオブジェクトが無ければ found=False。"""
    from omni.physx import get_physx_interface

    t = get_physx_interface().get_rigidbody_transformation(path)
    return {"found": bool(t["ret_val"]), "position": _r(t["position"]) if t["ret_val"] else None}


# 検証用 RigidBodyView のキャッシュ: path → (作成元の simulation view, RigidBodyView)。
# simulation view は Stop のたびに作り直されるので、作成元が変わったら作り直す。
_tensor_view_cache: dict[str, tuple[Any, Any]] = {}


def tensor_view(path: str) -> Any | None:
    """検証用の RigidBodyView を返す。simulation view が無効(Stop 中)なら None。

    ビューは SimulationManager が Play 時に作る simulation view から、検証用に別途作る
    (検証対象の RigidPrim が持つビューは使わない)。
    """
    from isaacsim.core.simulation_manager import SimulationManager

    sim_view = SimulationManager._physics_sim_view__warp
    if sim_view is None or not sim_view.is_valid:
        return None
    cached = _tensor_view_cache.get(path)
    if cached is None or cached[0] is not sim_view:
        _tensor_view_cache[path] = (sim_view, sim_view.create_rigid_body_view(path))
    return _tensor_view_cache[path][1]


def tensor_set_position(path: str, pos: Vec3) -> None:
    """検証用 RigidBodyView で PhysX 内部の位置を直接書く(ラッパーを通さない tensor 書き込み)。"""
    import warp as wp

    view = tensor_view(path)
    assert view is not None, "simulation view が無効(Play 中でない)"
    current = view.get_transforms()
    data = current.numpy().copy()  # (1, 7): px, py, pz, qx, qy, qz, qw
    data[0, :3] = pos
    dev = current.device  # GPU 物理なら cuda:0
    view.set_transforms(wp.from_numpy(data, dtype=wp.float32, device=dev), wp.array([0], dtype=wp.int32, device=dev))


def tensor_state(path: str) -> dict[str, Any]:
    """omni.physics.tensors の RigidBodyView で PhysX 内部の値を読む。Stop 中なら valid=False。"""
    view = tensor_view(path)
    if view is None:
        return {"valid": False}
    return {
        "valid": True,
        "position": _r(view.get_transforms().numpy()[0, :3]),
        "velocity": _r(view.get_velocities().numpy()[0, :3]),
        "mass": _r(view.get_masses().numpy()[0, 0]),
    }


def snapshot(stage: Usd.Stage, path: str = BODY) -> dict[str, Any]:
    """全層の値をまとめて返す。読むだけで、どの層にも書き込まない(Fabric は prim が未投入なら投入される)。"""
    return {
        "timeline": timeline_state(),
        "time": round(timeline().get_current_time(), 4),
        "usd": usd_state(stage, path),
        "fabric": fabric_state(path),
        "physx": physx_state(path),
        "tensor": tensor_state(path),
    }


# ------------------------------------------------------------------ 名前空間スパイ


class _Spy:
    """モジュールやオブジェクトの代わりに置き、属性アクセスを "<名前空間>.<属性名>" として記録するプロキシ。

    属性がモジュールなら、それもスパイで包んで返す(usdrt.Sdf.Path のような 2 段アクセスを記録するため)。
    それ以外は本物をそのまま返すので、動作は変わらない。
    """

    def __init__(self, real: Any, name: str, log: list[str]) -> None:
        object.__setattr__(self, "_real", real)
        object.__setattr__(self, "_name", name)
        object.__setattr__(self, "_log", log)

    def __getattr__(self, attr: str) -> Any:
        qual = f"{self._name}.{attr}"
        self._log.append(qual)
        val = getattr(self._real, attr)
        if isinstance(val, types.ModuleType):
            return _Spy(val, val.__name__, self._log)
        return val


def _prims_impl_modules() -> list[types.ModuleType]:
    """スパイを仕掛ける isaacsim.core.experimental.prims の実装モジュール。"""
    from isaacsim.core.experimental.prims.impl import (
        _fabric,
        _transform,
        _usd_articulation,
        articulation,
        deformable_prim,
        prim,
        rigid_prim,
        xform_prim,
    )

    return [xform_prim, rigid_prim, articulation, _usd_articulation, deformable_prim, prim, _fabric, _transform]


# ラッパーが tensor ビューを持つ属性名(RigidPrim / Articulation / DeformablePrim)
_VIEW_ATTRS: tuple[str, ...] = ("_physics_rigid_body_view", "_physics_articulation_view", "_physics_deformable_body_view")


@contextlib.contextmanager
def spy_namespaces(wrappers: list[Any]) -> Iterator[list[str]]:
    """with ブロックの間、ラッパー実装が「どの名前空間のものに触ったか」を記録する。

    - 実装モジュールのグローバル変数のうちモジュール型のもの(pxr.UsdGeom, usdrt, warp, _fabric など)をスパイに差し替える
    - ラッパーが持つ tensor ビュー(_VIEW_ATTRS)もスパイに差し替える
    with を抜けると元に戻す。yield するリストに "<名前空間>.<属性名>" が呼ばれた順に入る。
    """
    log: list[str] = []
    patched: list[tuple[types.ModuleType, str, Any]] = []
    for mod in _prims_impl_modules():
        for name, val in list(vars(mod).items()):
            if isinstance(val, types.ModuleType):
                patched.append((mod, name, val))
                setattr(mod, name, _Spy(val, val.__name__, log))
    views: list[tuple[Any, str, Any, _Spy]] = []  # (ラッパー, 属性名, 本物のビュー, スパイ)
    for w in wrappers:
        for attr in _VIEW_ATTRS:
            v = getattr(w, attr, None)
            if v is not None:
                spy = _Spy(v, f"{type(v).__module__}.{type(v).__name__}", log)
                setattr(w, attr, spy)
                views.append((w, attr, v, spy))
    try:
        yield log
    finally:
        for mod, name, val in patched:
            setattr(mod, name, val)
        for w, attr, v, spy in views:
            # Stop などでラッパー側がビューを None にしていたら、そのままにする
            if getattr(w, attr) is spy:
                setattr(w, attr, v)


# 名前空間の接頭辞 → 層の名前。spy_namespaces の記録をこの表で層に分類する。
NAMESPACE_FAMILIES: list[tuple[str, str]] = [
    ("pxr.", "USD(pxr)"),
    ("usdrt", "USDRT(usdrt)"),
    ("isaacsim.core.experimental.prims.impl._fabric", "Fabric直接(warp.fabricarray)"),
    ("warp.fabricarray", "Fabric直接(warp.fabricarray)"),
    ("omni.physics.tensors", "tensor(omni.physics.tensors)"),
]


def classify(log: list[str]) -> list[str]:
    """spy_namespaces の記録から、触った層の一覧(重複なし、出現順)を返す。"""
    out: list[str] = []
    for item in log:
        for prefix, family in NAMESPACE_FAMILIES:
            if item.startswith(prefix) and family not in out:
                out.append(family)
    return out


def touched(log: list[str], prefixes: tuple[str, ...] = ("pxr.", "usdrt", "omni.physics.tensors", "warp.fabricarray", "isaacsim.core.experimental.prims.impl._fabric")) -> list[str]:
    """spy_namespaces の記録のうち、層の判定に効くものだけを重複なしで返す(記録用)。"""
    seen: list[str] = []
    for item in log:
        if item.startswith(prefixes) and item not in seen:
            seen.append(item)
    return seen


# ------------------------------------------------------------------ バックエンドを指定した呼び出し


def call_with_backend(wrappers: list[Any], fn: Callable[[], Any], backend: str | None) -> dict[str, Any]:
    """use_backend(backend) の中で fn を呼び、触った層・警告・例外を返す。backend が None なら use_backend を使わない。

    Returns:
        touched : 実際に触った層(spy_namespaces の記録を classify したもの)
        warnings: carb の警告・エラー(重複を除いて順序は保つ)
        error   : 例外("型名: メッセージ")。無ければ None
    """
    import isaacsim.core.experimental.utils.backend as backend_utils

    with LogCapture() as lc, spy_namespaces(wrappers) as log:
        try:
            with backend_utils.use_backend(backend) if backend else contextlib.nullcontext():
                fn()
            err = None
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
    return {"touched": classify(log), "warnings": list(dict.fromkeys(lc.messages)), "error": err}


def rigid_readback(rp: Any) -> dict[str, Any]:
    """RigidPrim をユーザーが既定のバックエンドで読み返したら何が見えるか(位置・速度・質量)。"""
    out: dict[str, Any] = {}
    with LogCapture():
        for key, fn in [
            ("position", lambda: rp.get_world_poses()[0]),
            ("velocity", lambda: rp.get_velocities()[0]),
            ("mass", lambda: rp.get_masses()),
        ]:
            try:
                out[key] = _r(fn().numpy().reshape(-1))
            except Exception as e:
                out[key] = f"{type(e).__name__}: {e}"
    return out
