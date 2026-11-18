"""USD の共通処理。行列は numpy の列ベクトル形式(4x4)で扱い、Gf(行ベクトル形式)との変換はここだけで行う。"""

from __future__ import annotations

import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ROBOT_LINKS = "isaac:physics:robotLinks"
ROBOT_JOINTS = "isaac:physics:robotJoints"
ARTICULATION_ROOT_APIS = ("PhysicsArticulationRootAPI", "NewtonArticulationRootAPI", "PhysxArticulationAPI")
ALL_PRIMS = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)


# ---------------------------------------------------------------- 行列
def gf_to_np(m: Gf.Matrix4d) -> np.ndarray:
    return np.array(m, dtype=float).T


def np_to_gf(m: np.ndarray) -> Gf.Matrix4d:
    return Gf.Matrix4d(*np.asarray(m, dtype=float).T.flatten().tolist())


def rigid(m: np.ndarray) -> np.ndarray:
    """スケールを抜いた剛体変換。"""
    out = np.eye(4)
    r = m[:3, :3]
    u, _, vt = np.linalg.svd(r)
    out[:3, :3] = u @ vt
    out[:3, 3] = m[:3, 3]
    return out


def world(stage: Usd.Stage, path: str | Sdf.Path, cache: UsdGeom.XformCache | None = None) -> np.ndarray:
    prim = stage.GetPrimAtPath(str(path))
    cache = cache or UsdGeom.XformCache()
    return gf_to_np(cache.GetLocalToWorldTransform(prim))


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """2 つの姿勢の差(位置 m、角度 deg)。"""
    d = np.linalg.inv(rigid(a)) @ rigid(b)
    c = max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1.0) / 2.0))
    return float(np.linalg.norm(d[:3, 3])), float(np.degrees(np.arccos(c)))


def set_local_pose(prim: Usd.Prim, m: np.ndarray) -> None:
    """translate + orient の xformOp で置く(既存の xformOp は消す)。"""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    r = rigid(m)
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*r[:3, 3]))
    q = Gf.Matrix4d(np_to_gf(r)).ExtractRotationQuat()
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))


def joint_local(m: np.ndarray) -> tuple[Gf.Vec3f, Gf.Quatf]:
    r = rigid(m)
    q = np_to_gf(r).ExtractRotationQuat()
    return Gf.Vec3f(*r[:3, 3]), Gf.Quatf(float(q.GetReal()), Gf.Vec3f(q.GetImaginary()))


# ---------------------------------------------------------------- 剛体と関節
def resolve_body(stage: Usd.Stage, path: str | Sdf.Path | None) -> Usd.Prim | None:
    """joint の body の解決と同じ規則:祖先をたどって剛体、無ければ最初に見つけたコライダー、どちらも無ければ None(ワールド)。"""
    if not path:
        return None
    prim = stage.GetPrimAtPath(str(path))
    collider = None
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        if collider is None and prim.HasAPI(UsdPhysics.CollisionAPI):
            collider = prim
        prim = prim.GetParent()
    return collider


def rigid_body_at_or_above(stage: Usd.Stage, path: str) -> Usd.Prim | None:
    prim = stage.GetPrimAtPath(path)
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def joint_targets(prim: Usd.Prim) -> tuple[str | None, str | None]:
    j = UsdPhysics.Joint(prim)
    t0, t1 = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
    return (str(t0[0]) if t0 else None, str(t1[0]) if t1 else None)


def joints_under(stage: Usd.Stage, root: str) -> list[Usd.Prim]:
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [p for p in Usd.PrimRange(prim, ALL_PRIMS) if p.IsA(UsdPhysics.Joint)]


def joint_active(prim: Usd.Prim) -> bool:
    if not prim.IsActive():
        return False
    en = UsdPhysics.Joint(prim).GetJointEnabledAttr().Get()
    return en is None or bool(en)


def world_anchors(stage: Usd.Stage, root: str, *, scope: str | None = None, skip: tuple[str, ...] = ()) -> list[dict]:
    """root の下の、部品の外に固定する joint。名前は見ない。

    scope を省略すると「片側が剛体に解決されず(= ワールド)、もう片側が剛体」の joint。
    scope を渡すと、片側が scope の外の剛体に解決される joint も含める(部品を別の剛体の子に置いたとき、
    importer の root_joint の body0 は親の剛体に解決されるため)。
    """
    out = []
    for p in joints_under(stage, root):
        if str(p.GetPath()) in skip:
            continue
        if not joint_active(p) or UsdPhysics.Joint(p).GetExcludeFromArticulationAttr().Get():
            continue
        t0, t1 = joint_targets(p)
        b0, b1 = resolve_body(stage, t0), resolve_body(stage, t1)

        def inside(b: Usd.Prim | None) -> bool:
            if b is None:
                return False
            return scope is None or str(b.GetPath()).startswith(scope + "/")

        if inside(b0) != inside(b1):
            out.append({"path": str(p.GetPath()), "name": p.GetName(), "type": p.GetTypeName(), "body0": t0, "body1": t1,
                        "both_targets_authored": t0 is not None and t1 is not None})
    return out


def articulation_roots(stage: Usd.Stage, root: str) -> list[str]:
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, ALL_PRIMS) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def rigid_bodies(stage: Usd.Stage, root: str) -> list[str]:
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, ALL_PRIMS) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


# ---------------------------------------------------------------- 名前の対応
def display_name(prim: Usd.Prim) -> str:
    v = prim.GetMetadata("displayName")
    return v if v else prim.GetName()


def link_prims(stage: Usd.Stage, robot_root: str, names: list[str] | None = None) -> dict[str, str]:
    """URDF のリンク名 -> prim パス。ロボットスキーマの robotLinks から引き、無ければ剛体を名前で探す。

    names を渡すと、見つからなかったリンク(剛体の無い空のルートリンクなど、robotLinks に入らないもの)を、
    見つかったリンクの prim パスの途中にある同じ名前の prim から補う。
    """
    robot = stage.GetPrimAtPath(robot_root)
    out: dict[str, str] = {}
    rel = robot.GetRelationship(ROBOT_LINKS) if robot else None
    targets = [str(t) for t in rel.GetTargets()] if rel else []
    for t in targets:
        p = stage.GetPrimAtPath(t)
        # 結合で足された部品ロボットの prim(IsaacRobotAPI 付き)はリンクではないので除く
        if p and t.startswith(robot_root + "/") and not p.HasAPI("IsaacRobotAPI"):
            out.setdefault(display_name(p), t)
    if not out:
        for p in Usd.PrimRange(robot, ALL_PRIMS):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                out.setdefault(display_name(p), str(p.GetPath()))
    for name in names or []:
        if name in out:
            continue
        for known in list(out.values()):
            path = Sdf.Path(known)
            while path != Sdf.Path(robot_root) and path != Sdf.Path.absoluteRootPath:
                path = path.GetParentPath()
                prim = stage.GetPrimAtPath(path)
                if prim and display_name(prim) == name and not prim.IsA(UsdGeom.Gprim):
                    out[name] = str(path)
                    break
            if name in out:
                break
    # 空の末端リンクは site(IsaacSiteAPI 付きの Xform)として残るが、robotLinks に入る場合と入らない場合がある
    # (例: 自作の arm の flange は入り、Franka の panda_link8 は入らない)。見つかったリンクの子から名前で探す
    missing = [n for n in names or [] if n not in out]
    if missing and robot:
        known = set(out.values())
        for p in Usd.PrimRange(robot, ALL_PRIMS):
            n = display_name(p)
            if n in missing and n not in out and not p.IsA(UsdGeom.Gprim) and (
                    p.HasAPI("IsaacSiteAPI") or p.HasAPI("IsaacLinkAPI") or str(p.GetParent().GetPath()) in known):
                out[n] = str(p.GetPath())
    return out


def joint_prims(stage: Usd.Stage, robot_root: str) -> dict[str, str]:
    """URDF の関節名 -> prim パス。"""
    return {display_name(p): str(p.GetPath()) for p in joints_under(stage, robot_root)}


# ---------------------------------------------------------------- variant とレイヤー
def physics_variant_choice(prim: Usd.Prim, wanted: str) -> str | None:
    vs = prim.GetVariantSets()
    if not vs.HasVariantSet("Physics"):
        return None
    names = vs.GetVariantSet("Physics").GetVariantNames()
    if wanted == "auto":
        for n in ("physx", "physics"):
            if n in names:
                return n
        return None
    return wanted if wanted in names else None


def reload_layers_under(folder: str) -> None:
    """folder の下のファイルから読み込み済みのレイヤーを読み直す(同じプロセスで出力を作り直した後に古い内容を使わないため)。"""
    folder = os.path.normcase(os.path.abspath(folder))
    for layer in Sdf.Layer.GetLoadedLayers():
        real = layer.realPath
        if real and os.path.normcase(os.path.abspath(real)).startswith(folder):
            layer.Reload(force=True)
