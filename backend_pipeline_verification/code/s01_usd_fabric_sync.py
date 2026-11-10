"""S01: USD に書いた値は、いつ Fabric に反映されるか(Stop 中、物理なし)。

usdrt の文書は 2 か所で食い違って見える:
    fabricsd/fabric_hierarchy.rst   「USD の transform 変更は即座に omni:fabric:localMatrix に反映される」
    fabricsd/fsd_vs_omnihydra.md    「即時同期は OmniHydra(FSD 無効)の動作。FSD 有効時は変更をバッファし、
                                     SynchronizeToFabric() の明示呼び出し(Kit では描画直前に自動)でまとめて反映する」
    fabricsd/integration.md         「GetPrimAtPath() / GetAttributeAtPath() は暗黙に同期する(1 回限り)」
これを実測で確かめる。描画ありのエクスペリエンス(Kit の通常構成)で起動する。--no-fsd で FSD 無効の構成も測る。

ケース(毎回ステージを作り直す。剛体 /World/Body を z=1 に置き、Stop のまま):
    S-01  pxr で translate を書く → 何も呼ばずに IFabricHierarchy で local 行列を読む
    S-02  pxr で書く → usdrt の GetPrimAtPath を呼ぶ → local 行列を読む
    S-03  pxr で書く → SynchronizeToFabric() → local 行列を読む
    S-04  pxr で書く → app.update() → local 行列を読む
    S-05  pxr で質量を書く → 書く前に取得した usdrt の属性オブジェクトで読む / GetAttributeAtPath で読む / Sync 後に読む
    S-06  暗黙同期の罠: GetPrimAtPath → pxr で 2 回目を書く → 同じ prim オブジェクトで読む → Sync 後に読む
    S-07  ラッパー(XformPrim)を fabric バックエンドで読むと、書いた直後の値が見えるか
    S-08  FSD 無効時に use_backend("usdrt") / ("fabric") を指定するとどうなるか(--no-fsd のときだけ意味がある)
    S-09  OmniHydra(FSD 無効)の transform 属性 _worldPosition は、pxr で書いた直後に変わるか
          (omnihydra_xforms.md: OmniHydra は transform を omni:fabric:localMatrix ではなく _worldPosition 等に持つ)
"""

from __future__ import annotations

import bplib

app = bplib.start(render=True)

from typing import Any, Callable

import usdrt
from pxr import Usd

rec = bplib.Recorder("s01_usd_fabric_sync")

POS1: tuple[float, float, float] = (2.0, 0.0, 1.0)
POS2: tuple[float, float, float] = (3.0, 0.0, 1.0)


def fresh() -> tuple[Usd.Stage, Any, Any]:
    """ステージを作り、update で Fabric を最新にしてから、usdrt ステージと IFabricHierarchy を先に取得して返す。"""
    stage = bplib.new_stage(gravity=0.0)
    bplib.make_body(stage)
    bplib.update(3)
    rt = bplib._rt_stage()
    h = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(rt.GetFabricId(), rt.GetStageIdAsStageId())
    return stage, rt, h


def local_x(h: Any) -> float:
    """IFabricHierarchy で Body の local 行列の x を読む(usdrt の prim 取得は呼ばない)。"""
    return round(h.get_local_xform(usdrt.Sdf.Path(bplib.BODY)).ExtractTranslation()[0], 4)


def case_translate(after_write: Callable[[Any], None]) -> dict[str, Any]:
    stage, rt, h = fresh()
    before = local_x(h)
    bplib.usd_set_translate(stage, bplib.BODY, POS1)
    after_write(rt)
    return {"before": before, "after": local_x(h), "usd_written_x": POS1[0]}


@bplib.guarded(rec, "S-00")
def env() -> None:
    rec.add("S-00", "バージョンと設定", **bplib.environment())


@bplib.guarded(rec, "S-01..04")
def translate_cases() -> None:
    out = {
        "S-01 何も呼ばない": case_translate(lambda rt: None),
        "S-02 GetPrimAtPath": case_translate(lambda rt: rt.GetPrimAtPath(bplib.BODY)),
        "S-03 SynchronizeToFabric": case_translate(lambda rt: rt.SynchronizeToFabric()),
        "S-04 app.update": case_translate(lambda rt: bplib.update(1)),
    }
    rec.add("S-01..04", "pxr で translate.x=2 を書いた後、何をすれば Fabric の local 行列に反映されるか", **out)


@bplib.guarded(rec, "S-05")
def mass_cases() -> None:
    stage, rt, _ = fresh()
    attr_before = rt.GetPrimAtPath(bplib.BODY).GetAttribute("physics:mass")  # 書く前に取得した属性オブジェクト
    stage.GetPrimAtPath(bplib.BODY).GetAttribute("physics:mass").Set(5.0)
    out: dict[str, Any] = {"usd_written": 5.0, "1_old_attr_object": attr_before.Get()}
    out["2_GetAttributeAtPath"] = rt.GetAttributeAtPath(usdrt.Sdf.Path(bplib.BODY).AppendProperty("physics:mass")).Get()
    rt.SynchronizeToFabric()
    out["3_after_SynchronizeToFabric"] = attr_before.Get()
    rec.add("S-05", "pxr で質量 5 を書いた後の Fabric の値", **out)


@bplib.guarded(rec, "S-06")
def implicit_trap() -> None:
    stage, rt, h = fresh()
    bplib.usd_set_translate(stage, bplib.BODY, POS1)
    rt.GetPrimAtPath(bplib.BODY)  # 1 回目: 暗黙同期
    out: dict[str, Any] = {"1_after_implicit_sync": local_x(h)}
    bplib.usd_set_translate(stage, bplib.BODY, POS2)  # 2 回目の書き込み
    out["2_after_second_write"] = local_x(h)
    rt.GetPrimAtPath(bplib.BODY)  # もう一度 GetPrimAtPath
    out["3_after_second_GetPrimAtPath"] = local_x(h)
    rt.SynchronizeToFabric()
    out["4_after_SynchronizeToFabric"] = local_x(h)
    rec.add("S-06", "暗黙同期は 1 回限りか(x=2 → x=3 と 2 回書く)", **out)


@bplib.guarded(rec, "S-07")
def wrapper_read() -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils
    from isaacsim.core.experimental.prims import XformPrim

    out: dict[str, Any] = {}
    for backend in ["usd", "usdrt", "fabric"]:
        stage, rt, h = fresh()
        xp = XformPrim(bplib.BODY)
        bplib.update(1)
        bplib.usd_set_translate(stage, bplib.BODY, POS1)
        with bplib.LogCapture() as lc:
            try:
                with backend_utils.use_backend(backend):
                    out[backend] = round(float(xp.get_world_poses()[0].numpy()[0, 0]), 4)
            except Exception as e:
                out[backend] = f"{type(e).__name__}: {e}"
        out[backend + "_warnings"] = lc.messages
    rec.add("S-07", "pxr で x=2 を書いた直後に、XformPrim.get_world_poses を各バックエンドで読む", usd_written_x=POS1[0], **out)


@bplib.guarded(rec, "S-08")
def fsd_off_backend() -> None:
    import isaacsim.core.experimental.utils.backend as backend_utils

    out: dict[str, Any] = {}
    for backend in ["usdrt", "fabric"]:
        try:
            with backend_utils.use_backend(backend):
                out[backend] = "入れた(例外なし)"
        except Exception as e:
            out[backend] = f"{type(e).__name__}: {e}"
    rec.add("S-08", "use_backend('usdrt' / 'fabric') に入れるか(FSD 無効だと RuntimeError になる実装)", **out)


@bplib.guarded(rec, "S-09")
def omnihydra_world_position() -> None:
    stage, rt, _ = fresh()
    prim = rt.GetPrimAtPath(bplib.BODY)

    def read() -> Any:
        attr = prim.GetAttribute("_worldPosition")
        return bplib._r(attr.Get()) if attr and attr.HasValue() else None

    out: dict[str, Any] = {"before": read()}
    bplib.usd_set_translate(stage, bplib.BODY, POS1)
    out["immediately"] = read()
    bplib.update(1)
    out["after_update1"] = read()
    rec.add("S-09", "pxr で x=2 を書いた直後の Fabric の _worldPosition(OmniHydra の transform 属性)", **out)


env()
translate_cases()
mass_cases()
implicit_trap()
wrapper_read()
fsd_off_backend()
omnihydra_world_position()
bplib.finish(rec)
