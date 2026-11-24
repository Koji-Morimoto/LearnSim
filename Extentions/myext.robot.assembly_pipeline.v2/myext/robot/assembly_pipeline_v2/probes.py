"""部品の静的検査と、結合ロボットの動的プローブ（事後確認）を担うモジュール。

責務:
  1. 部品単体の静的検査（``static_part``）: 未修正生成物と正規化層の USD を開き、
     ワールド固定 joint・Articulation Root・instanceable リンク・末端リンク座標ずれ・
     mimic follower の drive・重複衝突メッシュ（R01〜R10 の事前判定・事後確認材料）を収集する。
  2. mimic 関節の動的プローブ（``probe_mimic`` / ``probe_mimic_usd``）: 無重力で leader 関節を
     柔らかい drive で動かし、follower 側の不要 drive による遅延・未到達（R03）を実測する。
  3. 結合ステージの動的プローブ（``probe_stage``）: 静的構造検査 → Play 中の関節駆動・結合ドリフト・
     親との接触イベント・閉ループ警告収集 → Stop 中の ``RobotPoser.apply_joint_state``（teleport）
     を 1 本の関数で計測し、公式ステージ（``official.py``）と修正後ステージ（``compose.py``）の両方に適用する。

主な公開 API:
  - ``LOG_KEYS``: ``carb`` 警告ログから PhysX 関連メッセージを抽出するキーワード一覧
  - ``static_part(...)``: 部品単体の静的 USD 検査（Stage/Timeline の副作用なし）
  - ``probe_mimic(...)``: URDF 部品の mimic 動特性プローブ
  - ``probe_mimic_usd(...)``: USD 部品の mimic 動特性プローブ（``UrdfModel`` 不要）
  - ``join_present(stage, top, res)``: 現在の VariantSet 選択状態で結合 ``res`` が存在するか判定する
  - ``probe_stage(...)``: 結合ステージ全体の静的検査 + Play プローブ + RobotPoser teleport 検査

パイプライン全体での位置づけ:
  - ``static_part`` は ``pipeline.build_assembly`` の Step 2 で呼ばれる（非破壊・Timeline 未使用）。
  - ``probe_mimic*`` と ``probe_stage`` は ``pipeline.run_pipeline`` の Step 5〜7 と
    ``official.probe_official`` から呼ばれ、測定値を ``rules.Context`` に供給する。

手を入れやすい箇所:
  - 新しい静的検査項目を足す場合は ``static_part``、新しい動的測定（例: 外力応答・別ログ監視）を足す場合は
    ``_probe_stage_impl`` と ``LOG_KEYS`` を拡張する。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from . import runtime
from . import usd_util as _u
from .compose import JoinResult
from .sources import Owner, UsdaInfo
from .spec import JoinFileSpec, PipelineSpec, ProbeSettings, UrdfSource, join_id
from .urdf_model import UrdfModel, pose_matrix

LOG_KEYS = ("closed articulation", "nested articulation", "was skipped, since body1", "mimic", "articulation")


# ---- usd_util ヘルパーの薄いラッパー群（関数存在チェック付きでフォールバックを提供） ----
def _all_prims():
    return getattr(_u, "ALL_PRIMS", Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))


def _has_api(prim: Usd.Prim, schema_name: str) -> bool:
    fn = getattr(_u, "has_api", None)
    if fn is not None:
        return fn(prim, schema_name)
    if not prim or not prim.IsValid():
        return False
    if schema_name in prim.GetAppliedSchemas():
        return True
    try:
        return bool(prim.HasAPI(schema_name))
    except Exception:  # noqa: BLE001
        return False


def _rigid(m: np.ndarray) -> np.ndarray:
    fn = getattr(_u, "rigid", getattr(_u, "rigid_part", None))
    if fn is not None:
        return fn(m)
    out = np.eye(4)
    u, _, vt = np.linalg.svd(m[:3, :3])
    out[:3, :3] = u @ vt
    out[:3, 3] = m[:3, 3]
    return out


def _world(stage: Usd.Stage, path: str, cache: UsdGeom.XformCache | None = None) -> np.ndarray:
    fn = getattr(_u, "world", getattr(_u, "world_xform", None))
    if fn is not None:
        return fn(stage, path, cache)
    prim = stage.GetPrimAtPath(str(path))
    cache = cache or UsdGeom.XformCache()
    return np.array(cache.GetLocalToWorldTransform(prim), dtype=float).T


def _pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    fn = getattr(_u, "pose_error", None)
    if fn is not None:
        return fn(a, b)
    d = np.linalg.inv(_rigid(a)) @ _rigid(b)
    c = max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1.0) / 2.0))
    return float(np.linalg.norm(d[:3, 3])), float(np.degrees(np.arccos(c)))


def _physics_variant_choice(prim: Usd.Prim, wanted: str, *, keep_existing: bool = False) -> str | None:
    fn = getattr(_u, "physics_variant_choice", getattr(_u, "select_physics_variant", None))
    if fn is not None:
        try:
            return fn(prim, wanted, keep_existing=keep_existing)
        except TypeError:
            return fn(prim, wanted)
    vs = prim.GetVariantSets()
    if not vs.HasVariantSet("Physics"):
        return None
    vset = vs.GetVariantSet("Physics")
    names = vset.GetVariantNames()
    if keep_existing and wanted == "auto":
        cur = vset.GetVariantSelection()
        if cur and cur != "None" and cur in names:
            return cur
    if wanted == "auto":
        for n in ("physx", "physics"):
            if n in names:
                return n
        lower_map = {n.lower(): n for n in names}
        for key in ("physx", "physics", "physx_mimic", "simplifiedphysx"):
            if key in lower_map:
                return lower_map[key]
        return None
    return wanted if wanted in names else None


def _articulation_roots(stage: Usd.Stage, root: str) -> list[str]:
    fn = getattr(_u, "articulation_roots", None)
    if fn is not None:
        return fn(stage, root)
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, _all_prims()) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def _rigid_bodies(stage: Usd.Stage, root: str) -> list[str]:
    fn = getattr(_u, "rigid_bodies", None)
    if fn is not None:
        return fn(stage, root)
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, _all_prims()) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def _rigid_body_at_or_above(stage: Usd.Stage, path: str) -> Usd.Prim | None:
    fn = getattr(_u, "rigid_body_at_or_above")
    return fn(stage, path)


def _resolve_body(stage: Usd.Stage, path: str | None) -> Usd.Prim | None:
    fn = getattr(_u, "resolve_body")
    return fn(stage, path)


def _joint_targets(prim: Usd.Prim) -> tuple[str | None, str | None]:
    fn = getattr(_u, "joint_targets", None)
    if fn is not None:
        return fn(prim)
    j = UsdPhysics.Joint(prim)
    t0, t1 = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
    return (str(t0[0]) if t0 else None, str(t1[0]) if t1 else None)


def _joints_under(stage: Usd.Stage, root: str) -> list[Usd.Prim]:
    fn = getattr(_u, "joints_under", None)
    if fn is not None:
        return fn(stage, root)
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [p for p in Usd.PrimRange(prim, _all_prims()) if p.IsA(UsdPhysics.Joint)]


def _world_anchors(stage: Usd.Stage, root: str, *, scope: str | None = None, skip: tuple[str, ...] = ()) -> list[dict[str, Any]]:
    fn = getattr(_u, "world_anchors")
    return fn(stage, root, scope=scope, skip=skip)


def _link_prims(stage: Usd.Stage, robot_root: str, names: list[str] | None = None) -> dict[str, str]:
    fn = getattr(_u, "link_prims")
    return fn(stage, robot_root, names)


def _usd_link_prims(stage: Usd.Stage, robot_root: str) -> dict[str, str]:
    fn = getattr(_u, "usd_link_prims", None)
    if fn is not None:
        return fn(stage, robot_root)
    return _link_prims(stage, robot_root, None)


def _usd_mimic_joints(stage: Usd.Stage, root: str) -> list[dict[str, Any]]:
    fn = getattr(_u, "usd_mimic_joints", None)
    if fn is not None:
        return fn(stage, root)
    return []


def _joint_active(prim: Usd.Prim) -> bool:
    fn = getattr(_u, "joint_active", None)
    if fn is not None:
        return bool(fn(prim))
    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    en = UsdPhysics.Joint(prim).GetJointEnabledAttr().Get()
    return en is None or bool(en)


def _is_active_mimic(prim: Usd.Prim) -> bool:
    if not prim or not prim.IsValid():
        return False
    schemas = prim.GetAppliedSchemas()
    if "NewtonMimicAPI" in schemas:
        attr = prim.GetAttribute("newton:mimicEnabled")
        enabled = attr.Get() if attr else True
        if enabled is None or bool(enabled):
            return True
    return any(s.startswith("PhysxMimicJointAPI:") or s == "PhysxMimicJointAPI" for s in schemas)


def _drive_gains(prim: Usd.Prim | None) -> dict[str, float] | None:
    if not prim or not prim.IsValid():
        return None
    for kind in ("angular", "linear"):
        if prim.HasAPI(UsdPhysics.DriveAPI, kind):
            d = UsdPhysics.DriveAPI(prim, kind)
            return {
                "stiffness": float(d.GetStiffnessAttr().Get() or 0.0),
                "damping": float(d.GetDampingAttr().Get() or 0.0),
            }
    return None


def _mimic_follower_name(item: dict[str, Any]) -> str:
    return str(item.get("follower") or item.get("name") or "")


def _mimic_leader_name(item: dict[str, Any]) -> str:
    return str(item.get("leader") or item.get("reference") or item.get("reference_joint") or "")


def _joint_prims(stage: Usd.Stage, robot_root: str) -> dict[str, str]:
    fn = getattr(_u, "joint_prims")
    return fn(stage, robot_root)


def _display_name(prim: Usd.Prim) -> str:
    fn = getattr(_u, "display_name", None)
    if fn is not None:
        return fn(prim)
    v = prim.GetMetadata("displayName")
    return v if v else prim.GetName()


def _resolve_raw_and_fixed_paths(usda_path: str | Path) -> tuple[str, str]:
    """正規化層または ``_import/`` 配下のパスから ``(生成物 USD パス, 正規化層 USD パス)`` の組を推定する。"""
    p = Path(usda_path)
    manifest = p.parent / "_import" / f"{p.stem}.manifest.json"
    if manifest.exists():
        try:
            gen = json.loads(manifest.read_text(encoding="utf-8")).get("generated_usd")
            if gen and os.path.exists(gen):
                return str(gen), str(p)
        except OSError:
            pass
    if p.parent.parent.name == "_import":
        fixed = p.parent.parent.parent / f"{p.parent.name}.usda"
        return str(p), str(fixed if fixed.exists() else p)
    return str(p), str(p)


# ================================================================ 部品単体 (静的)
def static_part(
    arg1: str | Path,
    arg2: str | Path | UrdfModel | None,
    arg3: UrdfModel | bool | str | None = False,
    arg4: str = "auto",
) -> dict[str, Any]:
    """部品単体の USD（生成物と正規化層）を静的に検査し、規則判定用の構造情報を返す。

    引数形式は 2 パターンを受け付ける:
      - ``static_part(generated_usd, fixed_usd, model, variant_hint="auto")``
      - ``static_part(usda_path, urdf_or_none, is_base=False)``
    ``model`` が ``None`` のときは USD 部品として正規化層 ``fx`` 上の構造を解析する。

    Returns:
        Physics variant 選択状態・剛体数・ワールド固定 joint・Articulation Root・
        instanceable リンク・剛体なしリンク・末端リンク座標ずれ・mimic follower drive・
        可視重複コライダーを格納した辞書。

    副作用:
        なし（``raw`` の Physics variant 一時切替も ``SessionLayer`` 内のみで行い元ファイルは変更しない）。
    """
    if isinstance(arg2, UrdfModel) or arg2 is None:
        generated_usd, fixed_usd = _resolve_raw_and_fixed_paths(arg1)
        model: UrdfModel | None = arg2
        is_base = bool(arg3) if isinstance(arg3, bool) else False
        variant_hint = arg3 if isinstance(arg3, str) else arg4
    else:
        generated_usd = str(arg1)
        fixed_usd = str(arg2)
        if arg3 is not None and not isinstance(arg3, UrdfModel):
            raise TypeError(f"static_part の第 3 引数は UrdfModel または None が必要: {type(arg3).__name__}")
        model = arg3
        is_base = False
        variant_hint = str(arg4)

    out: dict[str, Any] = {"is_base": is_base}
    raw = Usd.Stage.Open(generated_usd)
    dp = raw.GetDefaultPrim() if raw else None
    has_raw_dp = bool(dp and dp.IsValid())
    if has_raw_dp:
        root = str(dp.GetPath())
        vs = dp.GetVariantSet("Physics") if dp.GetVariantSets().HasVariantSet("Physics") else None
        out["physics_variant_selection"] = vs.GetVariantSelection() if vs else None
        out["physics_variants"] = vs.GetVariantNames() if vs else []
        # R06 事後確認用: Physics variant を選ぶ前の「生のままの剛体数」を先に数えておく
        out["raw_rigid_bodies_as_is"] = len(_rigid_bodies(raw, root))
        v = _physics_variant_choice(dp, variant_hint, keep_existing=(model is None))
        if vs and v:
            # 以降の静的検査で剛体・関節が見えるようにするため、元レイヤを汚さず SessionLayer 上で variant を選ぶ
            with Usd.EditContext(raw, raw.GetSessionLayer()):
                vs.SetVariantSelection(v)
        out["raw_variant_used"] = v
    else:
        root = ""
        out["physics_variant_selection"] = None
        out["physics_variants"] = []
        out["raw_rigid_bodies_as_is"] = 0
        out["raw_variant_used"] = None

    fx = Usd.Stage.Open(fixed_usd)
    fdp = fx.GetDefaultPrim()
    froot = str(fdp.GetPath())

    if model is None:
        # USD 部品: TOML の variant_selections が反映された正規化層 (fx) 上でリンク・関節・mimic を調べる
        out["anchors"] = _world_anchors(fx, froot)
        out["articulation_roots"] = _articulation_roots(fx, froot)
        out["newton_articulation_roots"] = [
            str(p.GetPath()) for p in Usd.PrimRange(fdp, _all_prims()) if _has_api(p, "NewtonArticulationRootAPI")
        ]
        links = _usd_link_prims(fx, froot)
        out["link_prims"] = links
        out["rigid_link_names"] = [n for n, p in links.items() if fx.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI)]
        out["site_link_names"] = [n for n, p in links.items() if _has_api(fx.GetPrimAtPath(p), "IsaacSiteAPI")]
        out["has_robot_api"] = _has_api(fdp, "IsaacRobotAPI")
        out["instanceable_links"] = [n for n, p in links.items() if fx.GetPrimAtPath(p).IsInstanceable()]
        out["links_without_body_above"] = [n for n, p in links.items() if _rigid_body_at_or_above(fx, p) is None]
        out["link_frame_shift"] = {}
        out["visible_duplicate_colliders"] = _visible_duplicates(fx, froot)
        out["fixed_rigid_bodies"] = len(_rigid_bodies(fx, froot))

        mimics = _usd_mimic_joints(fx, froot)
        out["mimic_followers"] = mimics
        raw_jp = _joint_prims(raw, root) if has_raw_dp else {}
        fjp = _joint_prims(fx, froot)
        gains: dict[str, dict[str, float]] = {}
        fgains: dict[str, dict[str, float]] = {}
        for item in mimics:
            fname = _mimic_follower_name(item)
            if not fname:
                continue
            if has_raw_dp:
                rp = raw.GetPrimAtPath(raw_jp.get(fname, ""))
                rg = _drive_gains(rp)
                if rg is not None:
                    gains[fname] = rg
            fpath = item.get("follower_path") or item.get("path") or fjp.get(fname, "")
            fp = fx.GetPrimAtPath(str(fpath)) if fpath else None
            fg = _drive_gains(fp)
            if fg is not None:
                fgains[fname] = fg
        out["raw_follower_drive"] = gains
        out["fixed_follower_drive"] = fgains
        return out

    # URDF 部品: 生成物 (raw) 上で剛体・関節・アンカー・Articulation Root・座標ずれ・mimic drive を調べる
    out["anchors"] = _world_anchors(raw, root)
    out["articulation_roots"] = _articulation_roots(raw, root)
    out["newton_articulation_roots"] = [
        str(p.GetPath()) for p in Usd.PrimRange(dp, _all_prims()) if _has_api(p, "NewtonArticulationRootAPI")
    ]
    links = _link_prims(raw, root, list(model.links))
    out["link_prims"] = links
    out["rigid_link_names"] = [n for n, p in links.items() if raw.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI)]
    out["site_link_names"] = [n for n, p in links.items() if _has_api(raw.GetPrimAtPath(p), "IsaacSiteAPI")]
    out["has_robot_api"] = _has_api(dp, "IsaacRobotAPI")
    out["instanceable_links"] = [n for n, p in links.items() if raw.GetPrimAtPath(p).IsInstanceable()]
    out["links_without_body_above"] = [n for n, p in links.items() if _rigid_body_at_or_above(raw, p) is None]
    # R04: 末端リンクの座標ずれを「URDF ゼロ姿勢 FK」と「USD prim のワールド姿勢」の差分で計測
    cache = UsdGeom.XformCache()
    top = _rigid(_world(raw, root, cache))
    shifts = {}
    for name, path in links.items():
        if name not in model.links:
            continue
        err_m, err_deg = _pose_error(top @ model.fk_zero(name), _world(raw, path, cache))
        shifts[name] = (round(err_m, 6), round(err_deg, 4))
    out["link_frame_shift"] = shifts
    # R03: 生生成物の mimic follower に付与された drive ゲインを取得
    jp = _joint_prims(raw, root)
    gains = {}
    for follower, _leader in model.mimic_pairs():
        p = raw.GetPrimAtPath(jp.get(follower.name, ""))
        rg = _drive_gains(p)
        if rg is not None:
            gains[follower.name] = rg
    out["raw_follower_drive"] = gains
    # R10: 衝突メッシュが guide 以外（default / render）で見えてしまい visual と重なっているか
    out["visible_duplicate_colliders"] = _visible_duplicates(raw, root)
    # 修正後（正規化層 fx）での剛体数と mimic follower drive
    out["fixed_rigid_bodies"] = len(_rigid_bodies(fx, froot))
    fjp = _joint_prims(fx, froot)
    fgains = {}
    for follower, _leader in model.mimic_pairs():
        p = fx.GetPrimAtPath(fjp.get(follower.name, ""))
        fg = _drive_gains(p)
        if fg is not None:
            fgains[follower.name] = fg
    out["fixed_follower_drive"] = fgains
    return out


def _visible_duplicates(stage: Usd.Stage, root: str) -> list[str]:
    """R10 用: visual と同一バウンディングボックスを持つ可視（``guide`` でない）コライダーのパス一覧を返す。"""
    gprims = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root), _all_prims()) if p.IsA(UsdGeom.Gprim)]
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide]
    )

    def key(p: Usd.Prim) -> tuple[float, ...]:
        r = bbox.ComputeWorldBound(p).ComputeAlignedRange()
        return tuple(round(v, 5) for v in list(r.GetMin()) + list(r.GetMax()))

    out = []
    visuals: dict[tuple[float, ...], list[str]] = {}
    for p in gprims:
        purpose = UsdGeom.Imageable(p).ComputePurpose()
        if not p.HasAPI(UsdPhysics.CollisionAPI) and purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
            visuals.setdefault(key(p), []).append(str(p.GetPath()))
    for p in gprims:
        purpose = UsdGeom.Imageable(p).ComputePurpose()
        if p.HasAPI(UsdPhysics.CollisionAPI) and purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
            if key(p) in visuals and str(p.GetPath()) not in visuals[key(p)]:
                out.append(str(p.GetPath()))
    return out


# ================================================================ mimic (部品単体を Play)
def _probe_mimic_single(
    usd_path: str,
    model: UrdfModel,
    settings: ProbeSettings,
    variant_hint: str = "auto",
) -> dict[str, Any]:
    """URDF 部品単体を無重力で Play し、follower の drive が leader の目標到達を妨げないか測る。

    あえて leader 側を柔らかい drive（``mimic_leader_stiffness_*``）で駆動することで、
    follower 側に不要な drive が残っている場合に leader の遅延や定常偏差が明確に現れるようにしている。
    """
    pairs = model.mimic_pairs()
    if not pairs:
        return {}
    stage = runtime.new_probe_stage(gravity=0.0)
    path = "/Part"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    v = _physics_variant_choice(prim, variant_hint)
    if v and prim.GetVariantSets().HasVariantSet("Physics") and not prim.GetVariantSet("Physics").GetVariantSelection():
        prim.GetVariantSet("Physics").SetVariantSelection(v)
    runtime.step(2)
    roots = _articulation_roots(stage, path)
    jp = _joint_prims(stage, path)
    targets = {}
    for _follower, leader in pairs:
        p = stage.GetPrimAtPath(jp.get(leader.name, ""))
        if not p or leader.name in targets:
            continue
        lo, hi = (leader.lower, leader.upper) if leader.has_limit and leader.upper > leader.lower else (0.0, 0.5)
        t = lo + 0.75 * (hi - lo)
        targets[leader.name] = t
        if leader.type == "prismatic":
            d = UsdPhysics.DriveAPI.Apply(p, "linear")
            d.CreateTargetPositionAttr(t)
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_linear)
            d.CreateDampingAttr(settings.mimic_leader_damping_linear)
        else:
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
            d.CreateTargetPositionAttr(math.degrees(t))
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_angular)
            d.CreateDampingAttr(settings.mimic_leader_damping_angular)
    out: dict[str, Any] = {"targets": targets}
    with runtime.capture_warnings() as logs:
        runtime.timeline().play()
        runtime.step(2)
        q = _run_and_read(stage, roots[0] if roots else None, settings.mimic_steps, samples=(10, 30, 60))
        runtime.stop()
    out["logs"] = [m for m in logs if any(k in m.lower() for k in LOG_KEYS)]
    if q is None:
        out["error"] = "Articulation を読めなかった"
        return out
    names, values, series = q
    pos = dict(zip(names, values))
    for follower, leader in pairs:
        if follower.name not in pos or leader.name not in pos:
            continue
        expected = follower.mimic.multiplier * pos[leader.name] + follower.mimic.offset  # type: ignore[union-attr]
        out[follower.name] = {
            "leader": leader.name,
            "leader_target": targets.get(leader.name),
            "leader_final": pos[leader.name],
            "follower_final": pos[follower.name],
            "relation_error": abs(pos[follower.name] - expected),
            "leader_error": abs(pos[leader.name] - targets.get(leader.name, pos[leader.name])),
            "type": leader.type,
            "leader_series": {str(k): dict(zip(names, v))[leader.name] for k, v in series.items()},
        }
    return out


def probe_mimic(
    arg1: str | Path | PipelineSpec,
    arg2: UrdfModel | UrdfSource,
    arg3: ProbeSettings | UrdfModel | None = None,
    variant_hint: str = "auto",
) -> dict[str, Any]:
    """URDF 部品の mimic 関節の動特性を Play して測定する（R03 の事後確認）。

    2 通りの呼び出し形式を受け付ける:
      - 単一 USD 測定: ``probe_mimic(usd_path, model, settings, variant_hint="auto")``
      - 公式・修正後一括測定: ``probe_mimic(spec, src, urdf)``

    Returns:
        follower ごとの ``leader_final`` / ``follower_final`` / ``relation_error`` / ``leader_error`` /
        ``leader_series`` と ``logs`` を格納した辞書。

    副作用:
        ``runtime.new_probe_stage(gravity=0.0)`` でグローバルステージを作り直し、Timeline を Play / Stop する。
    """
    if isinstance(arg1, (str, Path)):
        assert isinstance(arg2, UrdfModel)
        settings = arg3 if isinstance(arg3, ProbeSettings) else ProbeSettings()
        return _probe_mimic_single(str(arg1), arg2, settings, variant_hint)

    spec = arg1
    assert isinstance(arg2, UrdfSource)
    src = arg2
    urdf = arg3 if isinstance(arg3, UrdfModel) else None
    if urdf is None:
        from .urdf_model import load_urdf

        urdf = load_urdf(src.path)
    fixed_usd = os.path.join(spec.output_dir, src.usda)
    generated_usd, _ = _resolve_raw_and_fixed_paths(fixed_usd)
    official_res = _probe_mimic_single(generated_usd, urdf, spec.probe, src.physics_variant)
    fixed_res = _probe_mimic_single(fixed_usd, urdf, spec.probe, src.physics_variant)
    return {
        **fixed_res,
        "official": official_res,
        "fixed": fixed_res,
    }


def probe_mimic_usd(
    usd_path: str | Path,
    settings: ProbeSettings,
    variant_hint: str = "auto",
    *,
    prim_path: str | None = None,
    variant_selections: tuple[tuple[str, str], ...] = (),
) -> dict[str, Any]:
    """USD 部品の mimic 関節の動特性を Play して測定する（``UrdfModel`` 不要）。

    USD 内の ``NewtonMimicAPI`` / ``PhysxMimicJointAPI`` から leader・follower・倍率・オフセットを
    抽出し、``_probe_mimic_single`` と同じ形式で誤差と時系列を返す。

    副作用:
        ``runtime.new_probe_stage(gravity=0.0)`` でグローバルステージを作り直し、Timeline を Play / Stop する。
    """
    stage = runtime.new_probe_stage(gravity=0.0)
    path = "/Part"
    prim = stage.DefinePrim(path, "Xform")
    if prim_path:
        prim.GetReferences().AddReference(str(usd_path), Sdf.Path(prim_path))
    else:
        prim.GetReferences().AddReference(str(usd_path))
    var_map = dict(variant_selections)
    for vs_name, choice in var_map.items():
        if prim.GetVariantSets().HasVariantSet(vs_name):
            prim.GetVariantSet(vs_name).SetVariantSelection(choice)
    if "Physics" not in var_map:
        v = _physics_variant_choice(prim, variant_hint, keep_existing=True)
        if v and prim.GetVariantSets().HasVariantSet("Physics") and not prim.GetVariantSet("Physics").GetVariantSelection():
            prim.GetVariantSet("Physics").SetVariantSelection(v)
    runtime.step(2)
    mimics = _usd_mimic_joints(stage, path)
    if not mimics:
        return {}
    roots = _articulation_roots(stage, path)
    jp = _joint_prims(stage, path)
    targets: dict[str, float] = {}
    leader_types: dict[str, str] = {}
    for item in mimics:
        lname = _mimic_leader_name(item)
        lpath = item.get("leader_path") or item.get("reference_path") or jp.get(lname, "")
        p = stage.GetPrimAtPath(str(lpath)) if lpath else None
        if not p or not p.IsValid() or not lname or lname in targets:
            continue
        is_prismatic = p.IsA(UsdPhysics.PrismaticJoint)
        ltype = "prismatic" if is_prismatic else "revolute"
        leader_types[lname] = ltype
        lo_raw = p.GetAttribute("physics:lowerLimit").Get() if p.GetAttribute("physics:lowerLimit") else None
        hi_raw = p.GetAttribute("physics:upperLimit").Get() if p.GetAttribute("physics:upperLimit") else None
        lo_ok = lo_raw is not None and math.isfinite(lo_raw) and abs(lo_raw) < 1e30
        hi_ok = hi_raw is not None and math.isfinite(hi_raw) and abs(hi_raw) < 1e30
        if lo_ok and hi_ok and float(hi_raw) > float(lo_raw):
            lo = float(lo_raw) if is_prismatic else math.radians(float(lo_raw))
            hi = float(hi_raw) if is_prismatic else math.radians(float(hi_raw))
        else:
            lo, hi = (0.0, 0.5)
        t = lo + 0.75 * (hi - lo)
        targets[lname] = t
        if is_prismatic:
            d = UsdPhysics.DriveAPI.Apply(p, "linear")
            d.CreateTargetPositionAttr(t)
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_linear)
            d.CreateDampingAttr(settings.mimic_leader_damping_linear)
        else:
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
            d.CreateTargetPositionAttr(math.degrees(t))
            d.CreateStiffnessAttr(settings.mimic_leader_stiffness_angular)
            d.CreateDampingAttr(settings.mimic_leader_damping_angular)
    out: dict[str, Any] = {"targets": targets}
    with runtime.capture_warnings() as logs:
        runtime.timeline().play()
        runtime.step(2)
        q = _run_and_read(stage, roots[0] if roots else None, settings.mimic_steps, samples=(10, 30, 60))
        runtime.stop()
    out["logs"] = [m for m in logs if any(k in m.lower() for k in LOG_KEYS)]
    if q is None:
        out["error"] = "Articulation を読めなかった"
        return out
    names, values, series = q
    pos = dict(zip(names, values))
    for item in mimics:
        fname = _mimic_follower_name(item)
        lname = _mimic_leader_name(item)
        if not fname or not lname or fname not in pos or lname not in pos:
            continue
        mult = float(item.get("multiplier", 1.0))
        off = float(item.get("offset", 0.0))
        ltype = leader_types.get(lname, str(item.get("type") or item.get("joint_type") or "revolute"))
        if ltype == "revolute":
            off = math.radians(off)
        expected = mult * pos[lname] + off
        out[fname] = {
            "leader": lname,
            "leader_target": targets.get(lname),
            "leader_final": pos[lname],
            "follower_final": pos[fname],
            "relation_error": abs(pos[fname] - expected),
            "leader_error": abs(pos[lname] - targets.get(lname, pos[lname])),
            "type": ltype,
            "leader_series": {str(k): dict(zip(names, v))[lname] for k, v in series.items()},
        }
    return out


def _sim_view(stage: Usd.Stage):
    """現在の USD コンテキストに対する NumPy バックエンドの PhysX ``SimulationView`` を生成する。"""
    import omni.physics.tensors as tensors
    import omni.usd

    return tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())


def _run_and_read(stage: Usd.Stage, art_root: str | None, steps: int, samples: tuple[int, ...] = ()):
    """Play 中の Articulation を ``steps`` ステップ進め、途中サンプルと最終 DOF 位置を読み取る。

    ``timeline().stop()`` を呼ぶと ``ArticulationView`` が無効化されるため、
    ``dof_names`` と各ステップの DOF 位置は必ず Stop 前に Python 値へコピーして返す。
    """
    if not art_root:
        runtime.step(steps)
        return None
    sv = _sim_view(stage)
    av = sv.create_articulation_view(art_root)
    if av is None:
        runtime.step(steps)
        return None
    names = list(av.shared_metatype.dof_names)  # Stop すると view が無効になるので先に読む
    series = {}
    for i in range(steps):
        runtime.step(1)
        if i + 1 in samples:
            series[i + 1] = [float(v) for v in av.get_dof_positions()[0]]
    return names, [float(v) for v in av.get_dof_positions()[0]], series


# ================================================================ 結合 (Play と Stop 中の teleport)
def _root_art_root(stage: Usd.Stage, top: str, child_paths: list[str]) -> str | None:
    """結合した子ツリーの外側にある親側の Articulation Root パスを返す。"""
    for r in _articulation_roots(stage, top):
        if not any(r == cp or r.startswith(cp + "/") for cp in child_paths):
            return r
    return None


def _clamp_revolute_target(target: float, lo: float, hi: float) -> float:
    """回転関節の目標角 [rad] を可動域 ``[lo, hi]`` の内側（両端 10% マージン）に収める。

    0 を含まない可動域（例: Franka ``panda_joint4`` の ``[-3.0718, -0.0698]`` や ``[+0.5, +2.0]``）に
    対して ``0.8 * bound`` を使うと上限・下限の外側に出てしまうため、必ず区間幅基準で内側へ丸める。
    """
    if not (math.isfinite(lo) and math.isfinite(hi) and hi > lo):
        return target
    margin = 0.1 * (hi - lo)
    return float(np.clip(target, lo + margin, hi - margin))


def _drive_targets(
    stage: Usd.Stage,
    top: str,
    owners: list[Owner],
    prefixes: set[str] | None,
    settings: ProbeSettings,
) -> dict[str, tuple[str, float, str, str]]:
    """Play / RobotPoser プローブで駆動する関節と目標値（rad または m）を一覧化する。

    mimic follower 関節と ``excludeFromArticulation`` 関節は、直接駆動すると制約と競合するため除外する。
    """
    out = {}
    for o in owners:
        if (prefixes is not None and o.prefix not in prefixes) or not o.present(stage, top):
            continue
        if o.model is not None:
            followers = {f.name for f, _ in o.model.mimic_pairs()}
            for j in o.model.joints.values():
                if not j.movable or j.name in followers or j.name not in o.joint_rel:
                    continue
                path = o.joint_path(top, j.name)
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsActive():
                    continue
                if j.type == "prismatic":
                    lo, hi = (j.lower, j.upper) if (j.has_limit and j.upper > j.lower) else (0.0, 0.1)
                    t = lo + settings.prismatic_ratio * (hi - lo)
                else:
                    t = settings.revolute_target
                    if j.type == "revolute" and j.has_limit and j.upper > j.lower:
                        t = _clamp_revolute_target(t, j.lower, j.upper)
                out[f"{o.prefix}:{j.name}" if o.prefix else j.name] = (path, t, j.type, o.prefix)
        else:
            for jname in o.joint_rel:
                path = o.joint_path(top, jname)
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid() or not _joint_active(prim):
                    continue
                if UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Get():
                    continue
                if _is_active_mimic(prim):
                    continue
                if prim.IsA(UsdPhysics.PrismaticJoint):
                    jtype = "prismatic"
                elif prim.IsA(UsdPhysics.RevoluteJoint):
                    jtype = "revolute"
                else:
                    continue
                lo_attr = prim.GetAttribute("physics:lowerLimit")
                hi_attr = prim.GetAttribute("physics:upperLimit")
                lo_raw = lo_attr.Get() if lo_attr else None
                hi_raw = hi_attr.Get() if hi_attr else None
                lo_ok = lo_raw is not None and math.isfinite(lo_raw) and abs(lo_raw) < 1e30
                hi_ok = hi_raw is not None and math.isfinite(hi_raw) and abs(hi_raw) < 1e30
                has_limit = lo_ok and hi_ok and float(hi_raw) > float(lo_raw)
                if jtype == "prismatic":
                    lo, hi = (float(lo_raw), float(hi_raw)) if has_limit else (0.0, 0.1)
                    t = lo + settings.prismatic_ratio * (hi - lo)
                else:
                    t = settings.revolute_target
                    if has_limit:
                        t = _clamp_revolute_target(t, math.radians(float(lo_raw)), math.radians(float(hi_raw)))
                out[f"{o.prefix}:{jname}" if o.prefix else jname] = (path, t, jtype, o.prefix)
    return out


def join_present(stage: Usd.Stage, top: str, res: JoinResult) -> bool:
    """結合 ``res`` の子 prim が存在し、かつ ``res.conditions`` の VariantSet 選択がすべて一致するか返す。"""
    if not res.child_path or not stage.GetPrimAtPath(res.child_path):
        return False
    for rel, vs_name, choice in res.conditions:
        prim = stage.GetPrimAtPath(top if not rel else f"{top}/{rel}")
        vs = prim.GetVariantSets() if prim else None
        if not vs or not vs.HasVariantSet(vs_name) or vs.GetVariantSet(vs_name).GetVariantSelection() != choice:
            return False
    return True


def _reconstruct_joins_from_info(stage: Usd.Stage, top: str, f: JoinFileSpec, info: UsdaInfo) -> list[JoinResult]:
    """``probe_stage(spec, f, info)`` 形式で ``joins`` が省略された場合に ``UsdaInfo`` から ``JoinResult`` を復元する。"""
    results: list[JoinResult] = []
    for j in f.joins:
        res = JoinResult(jid=join_id(f, j), spec=j)
        if j.variant_set:
            res.conditions = (("", j.variant_set, j.variant),)  # type: ignore[assignment]
        try:
            p_owner, p_link = info.resolve(j.parent_link)
            res.parent_owner, res.parent_link = p_owner, p_link
            res.parent_link_prim = p_owner.link_path(top, p_link)
            c_owner, c_link = info.resolve(f"{j.name}/{j.child_link}")
            res.child_owner, res.child_link = c_owner, c_link
            res.child_link_prim = c_owner.link_path(top, c_link)
            top_child_owner = next((o for o in info.owners if o.prefix == j.name), c_owner)
            res.child_path = top_child_owner.top(top)
            b0 = _rigid_body_at_or_above(stage, res.parent_link_prim)
            b1 = _rigid_body_at_or_above(stage, res.child_link_prim)
            if b1 is not None and res.child_path and not (
                str(b1.GetPath()) == res.child_path or str(b1.GetPath()).startswith(res.child_path + "/")
            ):
                b1 = None
            if b1 is None and res.child_path:
                if c_owner.model is not None:
                    for d in c_owner.model.fixed_descendants(c_link):
                        if d in c_owner.link_rel:
                            rb = _rigid_body_at_or_above(stage, c_owner.link_path(top, d))
                            if rb is not None and (
                                str(rb.GetPath()) == res.child_path or str(rb.GetPath()).startswith(res.child_path + "/")
                            ):
                                b1 = rb
                                break
                else:
                    rbs = _u.rigid_descendants_via_fixed(stage, res.child_path, res.child_link_prim)
                    if rbs:
                        b1 = rbs[0]
            res.parent_body = str(b0.GetPath()) if b0 is not None else None
            res.child_body = str(b1.GetPath()) if b1 is not None else None
            if res.child_path:
                res.joint_path = f"{res.child_path}/JoinJoint"
        except Exception as exc:  # noqa: BLE001
            res.error = f"{type(exc).__name__}: {exc}"
        results.append(res)
    return results


def _probe_stage_impl(
    stage: Usd.Stage,
    top: str,
    owners: list[Owner],
    joins: list[JoinResult],
    settings: ProbeSettings,
    drive_prefixes: set[str] | None = None,
    include_offset: bool = True,
    with_poser: bool = True,
) -> dict[str, Any]:
    """結合済みステージに対して 3 段階のプローブ（静的構造 → Play 動的駆動 → Stop 中 RobotPoser）を実行する。

    段階ごとの役割:
      1. 静的構造検査: Articulation Root 残留（R07）・外部アンカー（R01/R02）・URDF 結合面ずれ（R04）・
         固定関節本数（R14）・固定関節 body1 解決先（R08）を記録する。
      2. Play 動的駆動: 全対象関節に drive 目標を与えてシミュレーションを進め、DOF 到達誤差（R02）・
         親剛体と子剛体の相対ドリフト（``mount_drift_during_play``）・親との接触（R11）・
         閉ループ警告ログ（R09）を計測する。
      3. Stop 中の teleport: ``RobotPoser.apply_joint_state`` を呼び、停止中の姿勢適用で
         子が置いて行かれないか・親が逆向きに動かないか（R12）を計測する。
    """
    out: dict[str, Any] = {"joins": {}}
    live = [r for r in joins if join_present(stage, top, r)]
    child_paths = [r.child_path for r in live if r.child_path]
    join_joints = tuple(r.joint_path for r in live if r.joint_path)
    out["articulation_roots"] = _articulation_roots(stage, top)
    out["robot_anchors"] = _world_anchors(stage, top)
    out["rigid_bodies"] = len(_rigid_bodies(stage, top))
    out["present_joins"] = [r.jid for r in live]
    # ---- Phase 1: 静的構造検査
    for r in live:
        info: dict[str, Any] = {}
        if (
            r.parent_owner
            and r.child_owner
            and r.parent_owner.model is not None
            and r.child_owner.model is not None
            and r.parent_link in r.parent_owner.model.links
            and r.child_link in r.child_owner.model.links
        ):
            offset = pose_matrix(r.spec.offset_xyz, r.spec.offset_rpy) if include_offset else np.eye(4)
            want = _rigid(_world(stage, r.parent_owner.top(top))) @ r.parent_owner.model.fk_zero(r.parent_link) @ offset
            got = _rigid(_world(stage, r.child_owner.top(top))) @ r.child_owner.model.fk_zero(r.child_link)
            info["urdf_mount_alignment"] = _pose_error(want, got)
        info["external_anchors"] = _world_anchors(stage, r.child_path, scope=r.child_path, skip=join_joints)  # type: ignore[arg-type]
        info["articulation_roots_in_child"] = _articulation_roots(stage, r.child_path)  # type: ignore[arg-type]
        if r.parent_body and r.child_body:
            pair = {r.parent_body, r.child_body}
            n = 0
            for p in _joints_under(stage, top):
                if not p.IsA(UsdPhysics.FixedJoint) or not p.IsActive():
                    continue
                t0, t1 = _joint_targets(p)
                b0, b1 = _resolve_body(stage, t0), _resolve_body(stage, t1)
                if b0 is not None and b1 is not None and {str(b0.GetPath()), str(b1.GetPath())} == pair:
                    n += 1
            info["fixed_joints_on_mount_pair"] = n
        if r.joint_path and stage.GetPrimAtPath(r.joint_path):
            _t0, t1 = _joint_targets(stage.GetPrimAtPath(r.joint_path))
            b1 = _resolve_body(stage, t1)
            info["joint_body1_resolved"] = str(b1.GetPath()) if b1 is not None else None
        out["joins"][r.jid] = info

    # ---- Phase 2: Play 動的プローブ（関節を drive で動かし、到達・結合のずれ・親との接触・警告ログを測る）
    targets = _drive_targets(stage, top, owners, drive_prefixes, settings)
    for _key, (path, t, kind, _prefix) in targets.items():
        d = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(path), "linear" if kind == "prismatic" else "angular")
        d.CreateTargetPositionAttr(t if kind == "prismatic" else math.degrees(t))
        d.CreateStiffnessAttr(settings.drive_stiffness)
        d.CreateDampingAttr(settings.drive_damping)
        d.CreateMaxForceAttr(float("inf"))
    watch = [r for r in live if r.parent_body and r.child_body]
    child_bodies = {r.jid: set(_rigid_bodies(stage, r.child_path)) for r in live}  # type: ignore[arg-type]
    parent_bodies = {}
    for r in live:
        o = r.parent_owner
        parent_bodies[r.jid] = (
            {
                str(rb.GetPath())
                for ln in (o.link_rel if o else {})
                for rb in [_rigid_body_at_or_above(stage, o.link_path(top, ln))]
                if rb is not None
            }
            if o
            else set()
        )
    # R11 接触検知のため、子の全剛体に閾値 0.0 の ContactReportAPI を付与して接触イベントを購読する
    for bodies in child_bodies.values():
        for b in bodies:
            prim = stage.GetPrimAtPath(b)
            if not prim.IsInstanceProxy():
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    contacts: list[tuple[str, str]] = []

    def on_contact(headers, data):  # noqa: ANN001
        from pxr import PhysicsSchemaTools

        for h in headers:
            contacts.append(
                (str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
            )

    import omni.physx

    sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    art_root = _root_art_root(stage, top, child_paths)
    out["root_articulation"] = art_root
    with runtime.capture_warnings() as logs:
        runtime.timeline().play()
        runtime.step(2)
        sv = _sim_view(stage)
        av = sv.create_articulation_view(art_root) if art_root else None
        body_paths = list(dict.fromkeys(p for r in watch for p in (r.parent_body, r.child_body) if p))
        body_index = {p: i for i, p in enumerate(body_paths)}
        rb = sv.create_rigid_body_view(body_paths) if body_paths else None
        dof_paths = list(av.dof_paths[0]) if av is not None and hasattr(av, "dof_paths") else []
        names = list(av.shared_metatype.dof_names) if av is not None else []
        # 初期剛体姿勢 t0 を Play 開始直後に記録し、ステップ後の t1 と親-子相対姿勢を比較して結合ドリフトを測る
        t0 = rb.get_transforms().copy() if rb is not None else None
        runtime.step(settings.steps)
        q = [float(v) for v in av.get_dof_positions()[0]] if av is not None else []
        t1 = rb.get_transforms().copy() if rb is not None else None
        runtime.stop()
    sub = None  # noqa: F841
    out["logs"] = [m for m in logs if any(k in m.lower() for k in LOG_KEYS)]
    out["dof_names"] = names
    by_path = dict(zip(dof_paths, q)) if dof_paths else {}
    by_name = dict(zip(names, q))
    reach = {}
    for key, (path, t, kind, prefix) in targets.items():
        v = by_path.get(path)
        if v is None and not dof_paths:
            v = by_name.get(key.split(":")[-1])
        if v is None:
            reach[key] = {
                "target": t,
                "final": None,
                "error": None,
                "kind": kind,
                "prefix": prefix,
                "note": "親の Articulation の DOF に無い",
            }
        else:
            reach[key] = {"target": t, "final": v, "error": abs(v - t), "kind": kind, "prefix": prefix}
    out["reach"] = reach
    for r in watch:
        if t0 is None or t1 is None or len(t0) != len(body_index):
            out["rigid_body_view_error"] = "剛体の view の数が合わない"
            break
        a, b = body_index[r.parent_body], body_index[r.child_body]  # type: ignore[index]
        m0 = [_pose_from_row(t0[a]), _pose_from_row(t0[b])]
        m1 = [_pose_from_row(t1[a]), _pose_from_row(t1[b])]
        out["joins"][r.jid]["mount_drift_during_play"] = _pose_error(
            np.linalg.inv(m0[0]) @ m0[1], np.linalg.inv(m1[0]) @ m1[1]
        )
    for r in live:
        # 固定関節で直接接続された親剛体-子剛体ペアは PhysX が自動的に衝突無視するため集計から除外する
        joined = {(r.parent_body, r.child_body), (r.child_body, r.parent_body)} if r.parent_body and r.child_body else set()
        cb, pb = child_bodies[r.jid], parent_bodies[r.jid] - child_bodies[r.jid]
        hits = sorted(
            {(x, y) for x, y in contacts if ((x in cb and y in pb) or (y in cb and x in pb)) and (x, y) not in joined}
        )
        out["joins"][r.jid]["contacts_with_parent"] = hits
        out["joins"][r.jid]["closed_loop_logs"] = [
            m for m in out["logs"] if "closed articulation" in m.lower() and (r.child_path or "") in m
        ]

    # ---- Phase 3: Stop 中の teleport 検査（R12: RobotPoser.apply_joint_state）
    if with_poser and watch and targets:
        try:
            from isaacsim.robot.poser.robot_poser import apply_joint_state

            before = {r.jid: (_world(stage, r.parent_body), _world(stage, r.child_body)) for r in watch}  # type: ignore[arg-type]
            tops_before = {o.prefix: _rigid(_world(stage, o.top(top))) for o in owners if o.present(stage, top)}
            apply_joint_state(stage, stage.GetPrimAtPath(top), {p: t for p, t, _k, _x in targets.values()})
            runtime.step(1)
            for r in watch:
                b0, b1 = before[r.jid]
                n0, n1 = _world(stage, r.parent_body), _world(stage, r.child_body)  # type: ignore[arg-type]
                info = out["joins"][r.jid]
                info["poser_drift"] = _pose_error(
                    np.linalg.inv(_rigid(b0)) @ _rigid(b1), np.linalg.inv(_rigid(n0)) @ _rigid(n1)
                )
                info["poser_parent_moved_deg"] = _pose_error(b0, n0)[1]
                o = r.parent_owner
                link = _display_name(stage.GetPrimAtPath(r.parent_body))  # type: ignore[arg-type]
                if o is not None and o.prefix == "" and o.model is not None and link in o.model.links:
                    q_o = {key.split(":")[-1]: t for key, (_p, t, _k, prefix) in targets.items() if prefix == ""}
                    top_w = tops_before[""]
                    offset = np.linalg.inv(top_w @ o.model.fk_zero(link)) @ _rigid(b0)
                    info["poser_fk_error"] = _pose_error(top_w @ o.model.fk(link, q_o) @ offset, _rigid(n0))
        except Exception as exc:  # noqa: BLE001
            out["poser_error"] = f"{type(exc).__name__}: {exc}"
    return out


def probe_stage(
    arg1: Usd.Stage | PipelineSpec,
    arg2: str | JoinFileSpec,
    arg3: list[Owner] | UsdaInfo,
    joins: list[JoinResult] | None = None,
    settings: ProbeSettings | None = None,
    drive_prefixes: set[str] | None = None,
    include_offset: bool = True,
    with_poser: bool = True,
    *,
    selection: dict[str, str] | None = None,
) -> dict[str, Any]:
    """結合済みステージにある全結合を静的・動的・RobotPoser の 3 段階で測定する。

    2 通りの呼び出し形式を受け付ける:
      - 既存ステージ測定: ``probe_stage(stage, top, owners, joins, settings, ...)``
      - 結合ファイルから新規ステージ作成して測定: ``probe_stage(spec, f, info, joins=None, selection=None)``

    Returns:
        ``joins``（結合 ID ごとの各種指標）・``reach``・``logs``・``articulation_roots`` 等を格納した辞書。

    副作用:
        対象ステージ上の関節に ``UsdPhysics.DriveAPI`` と ``PhysxContactReportAPI`` を設定し、
        Timeline を Play / Stop し、さらに ``RobotPoser.apply_joint_state`` で prim Transform を更新する。
    """
    if isinstance(arg1, Usd.Stage):
        assert isinstance(arg2, str)
        assert isinstance(arg3, list)
        assert joins is not None
        return _probe_stage_impl(
            arg1,
            arg2,
            arg3,
            joins,
            settings or ProbeSettings(),
            drive_prefixes=drive_prefixes,
            include_offset=include_offset,
            with_poser=with_poser,
        )

    spec = arg1
    assert isinstance(arg2, JoinFileSpec)
    assert isinstance(arg3, UsdaInfo)
    f, info = arg2, arg3
    stage = runtime.new_probe_stage()
    top = f"/{f.stem}"
    prim = stage.DefinePrim(top, "Xform")
    prim.GetReferences().AddReference(os.path.join(spec.output_dir, f.usda))
    for vs_name, choice in (selection or {}).items():
        if prim.GetVariantSets().HasVariantSet(vs_name):
            prim.GetVariantSet(vs_name).SetVariantSelection(choice)
    runtime.step(2)
    actual_joins = joins or getattr(info, "joins", None) or _reconstruct_joins_from_info(stage, top, f, info)
    return _probe_stage_impl(
        stage,
        top,
        info.owners,
        actual_joins,
        settings or spec.probe,
        drive_prefixes=drive_prefixes,
        include_offset=include_offset,
        with_poser=with_poser,
    )


def _pose_from_row(row) -> np.ndarray:
    """PhysX ``RigidBodyView.get_transforms()`` の 1 行 ``[x, y, z, qx, qy, qz, qw]`` を 4x4 列ベクトル行列へ変換する。"""
    x, y, z, w = (float(v) for v in row[3:7])
    m = np.eye(4)
    m[:3, :3] = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(w, x, y, z)))).T
    m[:3, 3] = [float(v) for v in row[:3]]
    return m


__all__ = [
    "LOG_KEYS",
    "static_part",
    "probe_mimic",
    "probe_mimic_usd",
    "join_present",
    "probe_stage",
]
