"""疑似的な幾何シミュレーション(幾何モード)の切り替え。

幾何モードでは、物体を「位置を直接書いて動かす」だけにし、PhysX には干渉(接触)の検出だけをさせる。
力・質量・摩擦による運動は一切起きない。

元に戻せる仕組み:
    上書きする値はすべて専用の匿名レイヤー(以下「幾何レイヤー」)に書き、そのレイヤーを
    セッションレイヤーのサブレイヤーとして一番強い位置に差し込む。
        Session Layer
          └─ 幾何レイヤー   ← ここにだけ書く(重力 0、kinematic 化、接触レポート …)
        Root Layer        ← 元の設定。一切書き換えない
    disable() はこのサブレイヤーを外すだけなので、元の値が「保存」→「書き戻し」される訳ではなく、
    元の値は最初から一度も変わっていない。セッションレイヤーはファイル保存の対象外なので、
    幾何モードのまま保存しても上書き値はファイルに入らない。

使い方:
    import geometric_mode as gm
    mode = gm.enable(stage)          # 幾何モードへ。戻り値の report に変更一覧
    ...                              # Play して、kinematic になった物体の xform を書いて動かす
    gm.disable(stage)                # 元の物理設定に戻す

    # 幾何モード中だけの一時的な書き込み(物体を動かす、色を変える等)は edit_context() の中で行うと、
    # disable() で一緒に消える
    with gm.edit_context(stage):
        UsdGeom.XformCommonAPI(prim).SetTranslate(...)
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
from typing import Any, Iterator

import omni.timeline
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

LAYER_MARKER: str = "geometricMode"  # 幾何レイヤーの customLayerData に入れる目印
SNAPSHOT_SKIP_PREFIXES: tuple[str, ...] = ("/OmniverseKit", "/Render")  # GUI のカメラ等。比較から外す


@dataclass
class Options:
    """幾何モードで何を上書きするか。既定値は「全部やる」。

    gravity_zero          : 全 PhysicsScene の重力を 0 にする
    make_kinematic        : 全剛体を kinematic にする(力では動かず、書いた位置にだけ動く)
    report_contacts       : 全剛体に ContactReportAPI(threshold 0 = 全接触を報告)を付ける
    report_kinematic_pairs: kinematic–kinematic、kinematic–static の組の接触も報告させる。
                            PhysX は既定ではこの組の接触を作らないので、これが無いと干渉が検出されない
    disable_articulations : アーティキュレーションを無効化する(リンクを個別の kinematic 剛体として扱う)
    disable_joints        : 全ジョイントを無効化する(kinematic 同士のジョイントは効果が無いため、明示的に切る)
    disable_ccd           : 剛体の CCD を切る(kinematic には不要で、PhysX が警告を出すことがある)
    """

    gravity_zero: bool = True
    make_kinematic: bool = True
    report_contacts: bool = True
    contact_threshold: float = 0.0
    report_kinematic_pairs: bool = True
    disable_articulations: bool = True
    disable_joints: bool = True
    disable_ccd: bool = True


@dataclass
class Change:
    """幾何レイヤーで上書きした 1 つの値。before は上書き前に見えていた値(未設定なら None)。"""

    path: str
    name: str
    before: Any
    after: Any
    before_authored: bool


@dataclass
class Report:
    """enable() の結果。changes は上書き一覧、unused は「幾何モードでは効かなくなる」設定の在庫。"""

    layer_identifier: str
    changes: list[Change] = field(default_factory=list)
    unused: dict[str, list[str]] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


# ------------------------------------------------------------------ レイヤー管理


def find_layer(stage: Usd.Stage) -> Sdf.Layer | None:
    """セッションレイヤー直下にある幾何レイヤーを返す。無ければ None。"""
    for ident in stage.GetSessionLayer().subLayerPaths:
        layer = Sdf.Layer.Find(ident)
        if layer is not None and layer.customLayerData.get(LAYER_MARKER):
            return layer
    return None


def is_enabled(stage: Usd.Stage) -> bool:
    return find_layer(stage) is not None


@contextmanager
def edit_context(stage: Usd.Stage) -> Iterator[Sdf.Layer]:
    """幾何レイヤーを編集先にする。中で書いた値は disable() で消える。"""
    layer = find_layer(stage)
    if layer is None:
        raise RuntimeError("幾何モードが有効ではない(先に enable() を呼ぶ)")
    with Usd.EditContext(stage, layer):
        yield layer


def _stop_timeline() -> None:
    """物理の設定は Play 開始時に読まれるので、切り替えは停止中に行う。"""
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()


# ------------------------------------------------------------------ 有効化・無効化


def enable(stage: Usd.Stage, options: Options | None = None) -> Report:
    """幾何モードにする。既に有効なら一度外してから作り直す。"""
    opt = options or Options()
    _stop_timeline()
    if is_enabled(stage):
        disable(stage)

    layer = Sdf.Layer.CreateAnonymous("geometric_mode")
    layer.customLayerData = {LAYER_MARKER: True}
    report = Report(layer.identifier)
    report.unused = _inventory_unused(stage)

    # 空のレイヤーを差し込んでも見え方は変わらないので、put() の before は元の値になる。
    # 値は edit context の中で幾何レイヤーにだけ書く(Root Layer は触らない)
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)

    def put(prim: Usd.Prim, attr: Usd.Attribute, value: Any) -> None:
        before_authored = attr.HasAuthoredValue()
        before = attr.Get() if attr.IsValid() else None
        attr.Set(value)
        report.changes.append(Change(str(prim.GetPath()), attr.GetName(), _plain(before), _plain(value), before_authored))

    with Usd.EditContext(stage, layer):
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                scene = UsdPhysics.Scene(prim)
                if opt.gravity_zero:
                    put(prim, scene.GetGravityMagnitudeAttr(), 0.0)
                if opt.report_kinematic_pairs:
                    px = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    put(prim, px.CreateReportKinematicKinematicPairsAttr(), True)
                    put(prim, px.CreateReportKinematicStaticPairsAttr(), True)

            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rb = UsdPhysics.RigidBodyAPI(prim)
                if opt.make_kinematic:
                    put(prim, rb.CreateKinematicEnabledAttr(), True)
                if opt.report_contacts:
                    cr = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                    put(prim, cr.CreateThresholdAttr(), opt.contact_threshold)
                if opt.disable_ccd and prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                    ccd = PhysxSchema.PhysxRigidBodyAPI(prim).GetEnableCCDAttr()
                    if ccd.HasAuthoredValue() and ccd.Get():
                        put(prim, ccd, False)

            if opt.disable_articulations and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                put(prim, art.CreateArticulationEnabledAttr(), False)

            if opt.disable_joints and prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                put(prim, joint.GetJointEnabledAttr(), False)
                _keep_joint_filter(stage, joint, report)

            if any(s.startswith(("PhysxDeformable", "PhysxParticle", "OmniPhysicsDeformable")) for s in prim.GetAppliedSchemas()):
                report.warnings.append(f"{prim.GetPath()}: 変形体・粒子は幾何モードの対象外(そのまま物理で動く)")

    if not any(c.name == UsdPhysics.Tokens.physicsGravityMagnitude for c in report.changes) and opt.gravity_zero:
        report.warnings.append("PhysicsScene が無い(Play すると既定の重力つきシーンが作られる)")
    return report


def disable(stage: Usd.Stage) -> bool:
    """幾何レイヤーを外して元の物理設定に戻す。外したら True。"""
    _stop_timeline()
    layer = find_layer(stage)
    if layer is None:
        return False
    paths = stage.GetSessionLayer().subLayerPaths
    paths.remove(layer.identifier)
    return True


# ------------------------------------------------------------------ 補助


def _keep_joint_filter(stage: Usd.Stage, joint: UsdPhysics.Joint, report: Report) -> None:
    """ジョイントを無効にしても「繋がった 2 物体は衝突しない」を保つ。

    ジョイントは physics:collisionEnabled=false(既定)のとき両端の物体の衝突を除外している。
    ジョイントを切るとこの除外も消え、隣り合うリンクが常に接触として報告されるので、
    同じ除外を FilteredPairsAPI で幾何レイヤーに書く。edit context の中で呼ぶこと。
    """
    if joint.GetCollisionEnabledAttr().Get():
        return
    b0 = joint.GetBody0Rel().GetTargets()
    b1 = joint.GetBody1Rel().GetTargets()
    if not b0 or not b1:
        return  # 片側が世界なら除外する相手がいない
    prim0 = stage.GetPrimAtPath(b0[0])
    if not prim0:
        return
    rel = UsdPhysics.FilteredPairsAPI.Apply(prim0).CreateFilteredPairsRel()
    rel.AddTarget(b1[0])
    report.changes.append(Change(str(prim0.GetPath()), "physics:filteredPairs", None, f"+{b1[0]}", False))


def _inventory_unused(stage: Usd.Stage) -> dict[str, list[str]]:
    """幾何モードでは効かなくなる設定を持つ prim を種類ごとに集める(書き換えはしない)。"""
    rules: dict[str, Any] = {
        "質量・慣性 (MassAPI)": lambda p: p.HasAPI(UsdPhysics.MassAPI),
        "摩擦・反発 (MaterialAPI)": lambda p: p.HasAPI(UsdPhysics.MaterialAPI),
        "ジョイントドライブ (DriveAPI)": lambda p: any(s.startswith("PhysicsDriveAPI") for s in p.GetAppliedSchemas()),
        "減衰・速度上限・ソルバ反復 (PhysxRigidBodyAPI)": lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI),
        "初速度 (physics:velocity / angularVelocity)": lambda p: p.HasAPI(UsdPhysics.RigidBodyAPI)
        and (p.GetAttribute("physics:velocity").HasAuthoredValue() or p.GetAttribute("physics:angularVelocity").HasAuthoredValue()),
    }
    out: dict[str, list[str]] = {k: [] for k in rules}
    for prim in stage.Traverse():
        for k, f in rules.items():
            if f(prim):
                out[k].append(str(prim.GetPath()))
    return {k: v for k, v in out.items() if v}


def _plain(v: Any) -> Any:
    """JSON にできる形へ。"""
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    return str(v)


def snapshot(stage: Usd.Stage) -> dict[str, dict[str, Any]]:
    """合成後のステージの見え方(prim ごとの型・API・属性値・関係)を辞書にする。復元の比較用。"""
    out: dict[str, dict[str, Any]] = {}
    for prim in stage.TraverseAll():
        path = str(prim.GetPath())
        if path.startswith(SNAPSHOT_SKIP_PREFIXES):
            continue
        d: dict[str, Any] = {"@type": prim.GetTypeName(), "@api": list(prim.GetAppliedSchemas()), "@active": prim.IsActive()}
        for a in prim.GetAuthoredAttributes():
            d[a.GetName()] = repr(a.Get())
        for r in prim.GetAuthoredRelationships():
            d["rel:" + r.GetName()] = [str(t) for t in r.GetTargets()]
        out[path] = d
    return out


def diff(a: dict[str, dict[str, Any]], b: dict[str, dict[str, Any]]) -> list[str]:
    """snapshot 同士の差分を「パス.名前: 前 -> 後」の文字列で返す。"""
    lines: list[str] = []
    for path in sorted(set(a) | set(b)):
        pa, pb = a.get(path), b.get(path)
        if pa is None or pb is None:
            lines.append(f"{path}: {'なし' if pa is None else 'あり'} -> {'なし' if pb is None else 'あり'}")
            continue
        for k in sorted(set(pa) | set(pb)):
            if pa.get(k) != pb.get(k):
                lines.append(f"{path}.{k}: {pa.get(k)} -> {pb.get(k)}")
    return lines
