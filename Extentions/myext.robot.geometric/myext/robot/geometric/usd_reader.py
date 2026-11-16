"""ステージ上の既存ロボット(Joint prim でつながった剛体)を読んで、内部モデルを作る。pxr だけを使う。

読むもの:
    physics:body0 / body1        つながり(剛体の子 prim を指している場合は、その剛体まで親をたどる)
    physics:localPos0/1, localRot0/1   関節の位置と姿勢(剛体のスケールを掛けて、スケール抜きのリンク座標系に直す)
    physics:axis                 軸(X / Y / Z)。body0 から見た body1 の、軸まわり右ねじ方向が正
    physics:lowerLimit/upperLimit 可動範囲(回転は度 → ラジアンに直す)
    PhysxMimicJointAPI / NewtonMimicAPI   mimic(参照関節・倍率・offset)

名前と並び(Play した Articulation の link_names / dof_names に合わせてある。UR10e + グリッパで確認):
    リンク名 = prim 名。同じ名前が複数あれば 2 個目から _0, _1, ... を付ける
    リンクの並び = 根から幅優先。DOF の並び = 各リンクに入ってくる関節を同じ順に並べ、fixed を除いたもの
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Sequence, get_args

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .model import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, GeometricRobotModel
from .model import transforms as tf

__all__ = ["UsdArticulationSource", "UsdRobotInfo", "read_articulation", "UnsupportedJointPolicy", "WorldJointPolicy", "WORLD_LINK"]

UnsupportedJointPolicy = Literal["error", "fixed", "ignore"]
WorldJointPolicy = Literal["ignore", "dof"]
_LAYER_MARKER = "myext.robot.geometric"  # layer.LAYER_MARKER と同じ値(幾何モードの Sublayer の目印)
WORLD_LINK = "world"  # ワールドに対して動く関節(レール上の台車など)があるときに根として置く、prim を持たないリンク
_MIMIC_PREFIX = "PhysxMimicJointAPI:"


@dataclass(frozen=True)
class UsdArticulationSource:
    """既存ロボットの指定。

    root_prim_path      : ロボットの prim(その下に剛体と Joint がある)。複数のロボットを 1 本の木として読むときは列で渡す。
                          ArticulationRoot が付いた Joint や剛体を渡した場合は、その親を使う
    components          : 構成要素名 -> prim パス。そのパス以下のリンクがその構成要素になる(いちばん長く一致したもの)。
                          結合したロボットで「アーム」「グリッパ」を分けたいときに使う
    extra_joint_paths   : root_prim_path の外にある Joint(ロボット同士をつなぐ固定関節など)
    ignore_joint_paths  : 読まない Joint
    dof_order           : DOF の並びを関節名で指定する(省略時は上の説明の並び)
    unsupported_joints  : revolute / prismatic / fixed 以外の Joint の扱い。"error" = 例外、"fixed" = 固定として読む、"ignore" = 無視
    world_joints        : ワールドに対して動く Joint(片側に剛体が無い revolute / prismatic)の扱い。
                          "ignore" = 木に入れない(PhysX の Articulation と同じ DOF になる。根リンクは set_world_poses で動かす)
                          "dof"    = DOF として読む(prim を持たないリンク "world" が根になる)
    name                : モデルの名前(省略時は最初の root_prim_path の prim 名)
    """

    root_prim_path: str | Sequence[str]
    components: dict[str, str] | None = None
    extra_joint_paths: Sequence[str] = ()
    ignore_joint_paths: Sequence[str] = ()
    dof_order: Sequence[str] | None = None
    unsupported_joints: UnsupportedJointPolicy = "error"
    world_joints: WorldJointPolicy = "ignore"
    name: str | None = None

    def __post_init__(self) -> None:
        if self.unsupported_joints not in get_args(UnsupportedJointPolicy):
            raise ValueError(f"unsupported_joints は {get_args(UnsupportedJointPolicy)} のどれか: {self.unsupported_joints!r}")
        if self.world_joints not in get_args(WorldJointPolicy):
            raise ValueError(f"world_joints は {get_args(WorldJointPolicy)} のどれか: {self.world_joints!r}")

    @property
    def root_prim_paths(self) -> list[str]:
        return [self.root_prim_path] if isinstance(self.root_prim_path, str) else list(self.root_prim_path)


@dataclass
class UsdRobotInfo:
    """モデルに入らない、USD 側の情報(Sublayer と干渉チェックが使う)。"""

    root_prim_paths: list[str]
    articulation_root_paths: list[str] = field(default_factory=list)  # ArticulationRootAPI が付いた prim
    joint_prim_paths: list[str] = field(default_factory=list)  # 読んだ範囲の全 Joint(ワールドへの固定や、木に入らなかったものも含む)
    link_prim_paths: dict[str, str] = field(default_factory=dict)  # リンク名 -> prim パス
    self_collision_enabled: bool | None = None  # physxArticulation:enabledSelfCollisions(書かれていなければ None)
    colliding_adjacent_pairs: list[tuple[str, str]] = field(default_factory=list)  # physics:collisionEnabled = true の Joint の両端
    filtered_pairs: list[tuple[str, str]] = field(default_factory=list)  # 既存の FilteredPairsAPI(リンク名の組)
    reversed_joints: list[str] = field(default_factory=list)  # 木の親が body1 側だった関節
    warnings: list[str] = field(default_factory=list)


def _np(matrix: Gf.Matrix4d) -> np.ndarray:
    """Gf の行ベクトル形式の行列を、列ベクトル形式の numpy 4x4 にする。"""
    return np.array(matrix, dtype=float).T


def split_scale(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """列ベクトル形式の 4x4 を (スケール抜きの剛体変換 4x4, スケール (3,)) に分ける。せん断は無いものとして扱う。"""
    scale = np.linalg.norm(matrix[:3, :3], axis=0)
    scale = np.where(scale < 1e-12, 1.0, scale)
    rigid = np.eye(4)
    rigid[:3, :3] = matrix[:3, :3] / scale
    if np.linalg.det(rigid[:3, :3]) < 0:  # 鏡像(負のスケール)は x を反転して右手系に戻す
        rigid[:3, 0] *= -1.0
        scale = scale * np.array([-1.0, 1.0, 1.0])
    rigid[:3, 3] = matrix[:3, 3]
    return rigid, scale


def _unique(name: str, used: dict[str, int]) -> str:
    if name not in used:
        used[name] = 0
        return name
    out = f"{name}_{used[name]}"
    used[name] += 1
    return out


def _body_of(prim: Usd.Prim) -> Usd.Prim | None:
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def _joint_enabled(joint: UsdPhysics.Joint) -> bool:
    """physics:jointEnabled の値。幾何モードの Sublayer が書いた false は数えない(幾何モード中でも元の構成を読めるように)。"""
    attr = joint.GetJointEnabledAttr()
    if not attr or not attr.IsValid():
        return True
    for spec in attr.GetPropertyStack(Usd.TimeCode.Default()):
        if spec.layer.customLayerData.get(_LAYER_MARKER):
            continue
        if spec.HasDefaultValue():
            return bool(spec.default)
    return True


def _limit(value: float | None, to_rad: bool, sign: float) -> float:
    if value is None or not np.isfinite(value) or abs(value) > 1e30:
        return sign * np.inf
    return float(np.deg2rad(value)) if to_rad else float(value)


def read_articulation(stage: Usd.Stage, source: UsdArticulationSource | str) -> tuple[GeometricRobotModel, UsdRobotInfo]:
    """ステージからロボットを読んで (内部モデル, USD 側の情報) を返す。ステージは書き換えない。"""
    if isinstance(source, str):
        source = UsdArticulationSource(source)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    info = UsdRobotInfo(root_prim_paths=[])

    # ---- 1. 探す範囲と Joint を集める
    roots: list[Usd.Prim] = []
    for path in source.root_prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise ValueError(f"prim が無い: {path}")
        has_joint = any(p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(prim) if p != prim)
        if (prim.IsA(UsdPhysics.Joint) or not has_joint) and not prim.GetParent().IsPseudoRoot():
            info.warnings.append(f"{path} の下に Joint が無いので、親 {prim.GetParent().GetPath()} を探す範囲にした")
            prim = prim.GetParent()
        roots.append(prim)
    info.root_prim_paths = [str(p.GetPath()) for p in roots]

    ignore = set(source.ignore_joint_paths)
    joint_prims: list[Usd.Prim] = []
    seen: set[str] = set()
    candidates = [p for root in roots for p in Usd.PrimRange(root)] + [stage.GetPrimAtPath(p) for p in source.extra_joint_paths]
    for prim in candidates:
        if not prim or not prim.IsValid():
            raise ValueError("extra_joint_paths に無い prim がある")
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and path not in info.articulation_root_paths:
            info.articulation_root_paths.append(path)
            attr = prim.GetAttribute("physxArticulation:enabledSelfCollisions")
            if attr and attr.HasAuthoredValue():
                info.self_collision_enabled = bool(attr.Get())
        if prim.IsA(UsdPhysics.Joint) and path not in seen and path not in ignore:
            seen.add(path)
            joint_prims.append(prim)
    for root in roots:  # ArticulationRootAPI がロボットの prim より上に付いている場合
        parent = root.GetParent()
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.ArticulationRootAPI) and str(parent.GetPath()) not in info.articulation_root_paths:
                info.articulation_root_paths.append(str(parent.GetPath()))
            parent = parent.GetParent()
    info.joint_prim_paths = [str(p.GetPath()) for p in joint_prims]

    # ---- 2. Joint を 1 つずつ読む(まだ向きは決めない)
    raw = []  # (prim, type, body0 path | None, body1 path | None, frame0, frame1, axis, lower, upper)
    world_fixed: list[str] = []  # ワールドに固定されている剛体
    for prim in joint_prims:
        joint = UsdPhysics.Joint(prim)
        path = str(prim.GetPath())
        if not _joint_enabled(joint):
            info.warnings.append(f"{path}: jointEnabled = false なので読まない")
            continue
        if joint.GetExcludeFromArticulationAttr().Get():
            info.warnings.append(f"{path}: excludeFromArticulation = true なので木に入れない")
            continue
        if prim.IsA(UsdPhysics.RevoluteJoint):
            jtype = "revolute"
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            jtype = "prismatic"
        elif prim.IsA(UsdPhysics.FixedJoint):
            jtype = "fixed"
        elif source.unsupported_joints == "fixed":
            jtype = "fixed"
            info.warnings.append(f"{path}: {prim.GetTypeName()} を固定として読んだ")
        elif source.unsupported_joints == "ignore":
            info.warnings.append(f"{path}: {prim.GetTypeName()} は対応していないので無視した")
            continue
        else:
            raise NotImplementedError(
                f"{path}: {prim.GetTypeName()} には対応していない(revolute / prismatic / fixed のみ)。"
                'UsdArticulationSource(unsupported_joints="fixed" か "ignore") で扱いを変えられる'
            )

        bodies: list[str | None] = []
        frames: list[np.ndarray] = []
        for rel, pos_attr, rot_attr in (
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
        ):
            targets = rel.GetTargets()
            pos = np.array(pos_attr.Get() or (0.0, 0.0, 0.0), dtype=float)
            quat = rot_attr.Get()
            rot = np.eye(3) if quat is None else tf.quat_to_matrix(np.array([quat.GetReal(), *quat.GetImaginary()], dtype=float))
            target = stage.GetPrimAtPath(targets[0]) if targets else None
            body = _body_of(target) if target else None
            if target and body is None and targets:
                info.warnings.append(f"{path}: 接続先 {targets[0]} に剛体が無いので、ワールドへの接続として読んだ")
            if target and target.IsValid():
                target_world = _np(cache.GetLocalToWorldTransform(target))
                target_rigid, target_scale = split_scale(target_world)
                joint_world = target_rigid @ tf.make_transform(rot, target_scale * pos)
            else:
                joint_world = tf.make_transform(rot, pos)
            if body is not None:
                body_rigid, _ = split_scale(_np(cache.GetLocalToWorldTransform(body)))
                frames.append(tf.invert(body_rigid) @ joint_world)
                bodies.append(str(body.GetPath()))
            else:
                frames.append(joint_world)  # ワールド座標系で見た関節座標系
                bodies.append(None)
        if bodies[0] is None and bodies[1] is None:
            info.warnings.append(f"{path}: 両端とも剛体が無いので読まない")
            continue
        axis_token = prim.GetAttribute("physics:axis").Get() if jtype != "fixed" and prim.GetAttribute("physics:axis") else "X"
        axis = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[axis_token or "X"]
        lower = prim.GetAttribute("physics:lowerLimit").Get() if jtype != "fixed" and prim.GetAttribute("physics:lowerLimit") else None
        upper = prim.GetAttribute("physics:upperLimit").Get() if jtype != "fixed" and prim.GetAttribute("physics:upperLimit") else None
        if jtype == "fixed" and None in bodies:
            world_fixed.append(bodies[0] or bodies[1])  # type: ignore[arg-type]
            continue
        if None in bodies and source.world_joints == "ignore":
            # PhysX の Articulation は、ワールドに対して動く Joint を DOF に数えない(根リンクが浮いた土台になる)
            info.warnings.append(f'{path}: ワールドに対して動く Joint なので木に入れない(DOF にするなら world_joints="dof")')
            world_fixed.append(bodies[0] or bodies[1])  # type: ignore[arg-type]
            continue
        raw.append((prim, jtype, bodies[0], bodies[1], frames[0], frames[1], axis, _limit(lower, jtype == "revolute", -1.0), _limit(upper, jtype == "revolute", 1.0)))
        collision_enabled = joint.GetCollisionEnabledAttr().Get()
        if collision_enabled and bodies[0] and bodies[1]:
            info.colliding_adjacent_pairs.append((bodies[0], bodies[1]))

    # ---- 3. 根を決めて、幅優先で木にする
    body_paths: list[str] = []
    for _, _, b0, b1, *_ in raw:
        for b in (b0, b1):
            if b is not None and b not in body_paths:
                body_paths.append(b)
    for b in world_fixed:
        if b not in body_paths:
            body_paths.append(b)
    if not body_paths:
        raise ValueError(f"{info.root_prim_paths} の下に、剛体をつなぐ Joint が見つからない")
    has_world_joint = any(b0 is None or b1 is None for _, _, b0, b1, *_ in raw)
    child_side = {b1 for _, _, b0, b1, *_ in raw if b0 is not None and b1 is not None}
    if has_world_joint:
        root_body: str | None = None  # 根はワールド(prim なし)
    elif world_fixed:
        root_body = world_fixed[0]
    else:
        with_api = [b for b in body_paths if stage.GetPrimAtPath(b).HasAPI(UsdPhysics.ArticulationRootAPI)]
        never_child = [b for b in body_paths if b not in child_side]
        root_body = with_api[0] if with_api else (never_child[0] if never_child else body_paths[0])

    used_links: dict[str, int] = {}
    used_joints: dict[str, int] = {}
    link_name_of: dict[str | None, str] = {}
    links: list[GeometricLinkSpec] = []

    def add_link(body: str | None) -> None:
        name = _unique(WORLD_LINK if body is None else stage.GetPrimAtPath(body).GetName(), used_links)
        link_name_of[body] = name
        links.append(GeometricLinkSpec(name, body))
        if body is not None:
            info.link_prim_paths[name] = body

    add_link(root_body)
    queue: list[str | None] = [root_body]
    used_raw: set[int] = set()
    specs: list[GeometricJointSpec] = []
    joint_name_of_path: dict[str, str] = {}
    pending_mimic: dict[str, Usd.Prim] = {}
    while queue:
        cur = queue.pop(0)
        for k, (prim, jtype, b0, b1, f0, f1, axis, lower, upper) in enumerate(raw):
            if k in used_raw or cur not in (b0, b1):
                continue
            other = b1 if cur == b0 else b0
            path = str(prim.GetPath())
            if other in link_name_of:
                used_raw.add(k)
                info.warnings.append(f"{path}: 閉じた輪を作るので木に入れない")
                continue
            used_raw.add(k)
            add_link(other)
            queue.append(other)
            forward = cur == b0
            name = _unique(prim.GetName(), used_joints)
            joint_name_of_path[path] = name
            if not forward:
                info.reversed_joints.append(name)
            specs.append(
                GeometricJointSpec(
                    name=name,
                    joint_type=jtype,
                    parent_link=link_name_of[cur],
                    child_link=link_name_of[other],
                    parent_frame=f0 if forward else f1,
                    child_frame=f1 if forward else f0,
                    # 逆向き(木の親が body1)のときは、軸を反転すると関節値の正負が body0 → body1 の定義のまま保たれる
                    axis=axis if forward else tuple(-v for v in axis),  # type: ignore[arg-type]
                    lower_limit=lower,
                    upper_limit=upper,
                    prim_path=path,
                )
            )
            if jtype != "fixed":
                pending_mimic[name] = prim
    for k, (prim, *_rest) in enumerate(raw):
        if k not in used_raw:
            info.warnings.append(f"{prim.GetPath()}: 根 {root_body or 'ワールド'} からたどれないので木に入れない")

    # ---- 4. mimic。2 つの書き方を読む(offset の単位は、回転なら度、直動ならステージの長さ。PhysX で確認済み)
    #   NewtonMimicAPI      : 従動 = coef0 + coef1 × 参照
    #   PhysxMimicJointAPI  : 従動 + gearing × 参照 + offset = 0  →  従動 = −gearing × 参照 − offset
    type_of = {s.name: s.joint_type for s in specs}
    for i, spec in enumerate(specs):
        prim = pending_mimic.get(spec.name)
        if prim is None:
            continue
        schemas = prim.GetAppliedSchemas()
        instances = [s[len(_MIMIC_PREFIX) :] for s in schemas if s.startswith(_MIMIC_PREFIX)]
        if "NewtonMimicAPI" in schemas:
            enabled = prim.GetAttribute("newton:mimicEnabled").Get() if prim.GetAttribute("newton:mimicEnabled") else True
            if enabled is not None and not enabled:
                continue
            targets = prim.GetRelationship("newton:mimicJoint").GetTargets()
            coef0 = prim.GetAttribute("newton:mimicCoef0").Get() if prim.GetAttribute("newton:mimicCoef0") else None
            coef1 = prim.GetAttribute("newton:mimicCoef1").Get() if prim.GetAttribute("newton:mimicCoef1") else None
            multiplier, offset = float(1.0 if coef1 is None else coef1), float(coef0 or 0.0)
        elif instances:
            if len(instances) > 1:
                info.warnings.append(f"{prim.GetPath()}: mimic が複数ある({instances})。最初の 1 つだけ読んだ")
            inst = instances[0]
            targets = prim.GetRelationship(f"physxMimicJoint:{inst}:referenceJoint").GetTargets()
            gearing = prim.GetAttribute(f"physxMimicJoint:{inst}:gearing").Get()
            multiplier = -float(1.0 if gearing is None else gearing)
            offset = -float(prim.GetAttribute(f"physxMimicJoint:{inst}:offset").Get() or 0.0)
        else:
            continue
        if not targets or str(targets[0]) not in joint_name_of_path:
            info.warnings.append(f"{prim.GetPath()}: mimic の参照関節が読んだ範囲に無いので、独立な軸として扱う")
            continue
        reference = joint_name_of_path[str(targets[0])]
        if type_of.get(reference) != spec.joint_type:
            info.warnings.append(f"{prim.GetPath()}: 種類の違う関節({reference})への mimic は単位が決まらないので、独立な軸として扱う")
            continue
        if spec.joint_type == "revolute":
            offset = float(np.deg2rad(offset))
        specs[i] = spec.replaced(mimic=GeometricMimicSpec(reference, multiplier, offset))

    # ---- 5. DOF の並びの指定
    if source.dof_order is not None:
        dof_names = [s.name for s in specs if s.is_dof]
        if sorted(dof_names) != sorted(source.dof_order):
            raise ValueError(f"dof_order は DOF 名を過不足なく並べる。DOF: {dof_names} / 指定: {list(source.dof_order)}")
        by_name = {s.name: s for s in specs}
        ordered = iter([by_name[n] for n in source.dof_order])
        specs = [next(ordered) if s.is_dof else s for s in specs]

    # ---- 6. 構成要素と、既存の衝突除外
    components: dict[str, tuple[str, ...]] = {}
    if source.components:
        for link in links:
            if link.prim_path is None:
                continue
            best = max((c for c, p in source.components.items() if link.prim_path == p or link.prim_path.startswith(p.rstrip("/") + "/")), key=lambda c: len(source.components[c]), default=None)  # type: ignore[index]
            if best is not None:
                components.setdefault(best, ())
                components[best] = components[best] + (link.name,)
    info.colliding_adjacent_pairs = [(link_name_of[a], link_name_of[b]) for a, b in info.colliding_adjacent_pairs if a in link_name_of and b in link_name_of]
    path_to_link = {v: k for k, v in info.link_prim_paths.items()}
    for name, body in info.link_prim_paths.items():
        for prim in Usd.PrimRange(stage.GetPrimAtPath(body)):
            if not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                continue
            for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets():
                other_body = _body_of(stage.GetPrimAtPath(target))
                other = path_to_link.get(str(other_body.GetPath())) if other_body else None
                if other is not None and other != name and (other, name) not in info.filtered_pairs:
                    info.filtered_pairs.append((name, other))

    model = GeometricRobotModel(
        name=source.name or roots[0].GetName(),
        links=tuple(links),
        joints=tuple(specs),
        root_link=links[0].name,
        components=components,
    )
    return model, info
