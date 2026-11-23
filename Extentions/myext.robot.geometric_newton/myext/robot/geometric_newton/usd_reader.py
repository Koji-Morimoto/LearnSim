"""USD ステージ上の既存 Articulation (`UsdPhysics.Joint` で結ばれた剛体群) を読み取り、`GeometricRobotModel` を構築するモジュール。

`pxr` のみで動作し、Kit や PhysX の起動を前提としない。

読み取る USD 属性と規約:
    - `physics:body0` / `physics:body1`:
        接続先 prim。剛体の子 prim (コライダー等) を指している場合は親方向へ `RigidBodyAPI` をたどる。
    - `physics:localPos0/1`, `physics:localRot0/1`:
        各接続先ローカル座標系における関節フレーム。接続先 prim のワールドスケールを位置へ掛けたうえで、
        スケール抜きの剛体リンク座標系へ変換する。
    - `physics:axis`:
        `X` / `Y` / `Z`。`body0` から見た `body1` の右ねじ方向を正とする。
    - `physics:lowerLimit` / `physics:upperLimit`:
        可動範囲。USD の回転関節は度 (deg) で記録されるため、内部モデル用にラジアン (rad) へ変換する。
    - `NewtonMimicAPI` / `PhysxMimicJointAPI`:
        従動軸 (mimic) の参照関節・倍率・オフセット。

設計上の重要なポイント (なぜこう実装しているか):
    1. **逆向き関節 (`body1` が木の親になるケース) の正規化**:
       USD では `body0` が子リンク・`body1` が親リンクを指す関節が許容される (Newton の `builder.add_usd()` は
       これを非対応エラーとする)。本モジュールでは `parent_frame` / `child_frame` を入れ替えつつ `axis` の符号を
       反転することで、「`body0` から見た `body1` の正方向」という関節値の意味を保ったまま木構造へ正規化する。
    2. **`PhysxMimicJointAPI` の符号変換とオフセット単位**:
       PhysX の定義式は `follower + gearing * reference + offset = 0` であるため、内部表現
       `follower = multiplier * reference + offset` へ直す際に `multiplier = -gearing`、`offset = -offset` と
       符号反転する。また回転関節の `offset` は度で書かれる (直動関節はステージの長さ単位) ため `deg2rad` する。
    3. **`_LAYER_MARKERS` による幾何 Sublayer の無視**:
       `GeometricLayer` 適用中 (`jointEnabled=false` が上書きされている状態) に VariantSet 切替等で再読込しても、
       `_marker_layers` に該当する Sublayer の値を飛ばしてプロパティスタックを読むことで元の関節構成を復元できる。
    4. **ワールド接続可動関節の既定除外 (`world_joints="ignore"`)**:
       PhysX の Articulation では片側がワールドに開いた可動 Joint は Articulation の DOF に含まれない (根リンクが
       浮遊ベース扱いになる)。既定で除外することで `Articulation.dof_names` と DOF 数を一致させる。
    5. **リンク・DOF 順の一致**:
       根から幅優先 (BFS) でリンクを並べ、各リンクへ入る関節の順 (fixed を除く) に DOF を並べることで、
       Isaac Sim で Play した PhysX `Articulation` の `link_names` / `dof_names` と一致させている。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Sequence, get_args

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .model import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, GeometricRobotModel
from .model import transforms as tf

__all__ = ["UsdArticulationSource", "UsdRobotInfo", "read_articulation", "UnsupportedJointPolicy", "WorldJointPolicy", "WORLD_LINK"]

# ==============================================================================
# 型定義・定数・データクラス
# ==============================================================================

UnsupportedJointPolicy = Literal["error", "fixed", "ignore"]
WorldJointPolicy = Literal["ignore", "dof"]
# 新旧どちらの幾何モード Sublayer (`geometric_newton` / 旧 `geometric`) が付いていても透過的に無視するための目印一覧
_LAYER_MARKERS = ("myext.robot.geometric_newton", "myext.robot.geometric")
# `world_joints="dof"` のとき、ワールド固定座標系を表す prim 非保有の仮想根リンク名
WORLD_LINK = "world"
_MIMIC_PREFIX = "PhysxMimicJointAPI:"


@dataclass(frozen=True)
class UsdArticulationSource:
    """ステージ上の既存 Articulation を読み取る際の指定パラメータ。

    root_prim_path      : ロボットのルート prim パス (複数ロボットを 1 本の木として読むときはパス列)。
                          `ArticulationRootAPI` が Joint や単一剛体に直接付いている場合は親 prim を探索範囲にする。
    components          : 構成要素名 -> prim パス / `fnmatch` パターン (例 `"/World/Robot/arm0_*"`) またはその列。
                          複数に一致した場合は最長パターンを採用する。結合ロボットの部位別干渉チェックに使う。
    extra_joint_paths   : `root_prim_path` の外に置かれた追加 Joint (ロボット同士をつなぐ固定関節など)。
    ignore_joint_paths  : 読み取り対象から除外する Joint パス。
    dof_order           : DOF の並びを関節名で明示指定する (`None` なら BFS 順)。
    unsupported_joints  : `revolute` / `prismatic` / `fixed` 以外の Joint の扱い (`"error"` / `"fixed"` / `"ignore"`)。
    world_joints        : 片側に剛体が無い可動 Joint の扱い。
                          `"ignore"` = 木に入れない (PhysX Articulation と DOF 数を一致させ、根リンクは `set_world_poses` で動かす)。
                          `"dof"`    = DOF として木に含める (仮想根リンク `WORLD_LINK` を追加する)。
    name                : モデルの名前 (省略時は最初の `root_prim_path` の prim 名)。
    """

    root_prim_path: str | Sequence[str]
    components: dict[str, str | Sequence[str]] | None = None
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
    """`GeometricRobotModel` 本体には含めない USD 側メタデータ (`GeometricLayer`・干渉チェック・計測が利用)。"""

    root_prim_paths: list[str]
    articulation_root_paths: list[str] = field(default_factory=list)  # `ArticulationRootAPI` が付いた prim パス一覧
    joint_prim_paths: list[str] = field(default_factory=list)  # 探索範囲内の全 Joint (ワールド固定・除外分を含む)
    link_prim_paths: dict[str, str] = field(default_factory=dict)  # リンク名 -> 対応する USD prim パス
    self_collision_enabled: bool | None = None  # `physxArticulation:enabledSelfCollisions` (未記述なら `None`)
    colliding_adjacent_pairs: list[tuple[str, str]] = field(default_factory=list)  # `physics:collisionEnabled=true` の Joint の両端
    filtered_pairs: list[tuple[str, str]] = field(default_factory=list)  # USD 側に既存の `FilteredPairsAPI` ペア (リンク名組)
    reversed_joints: list[str] = field(default_factory=list)  # 木の親が `body1` 側だった (軸反転で正規化した) 関節名一覧
    warnings: list[str] = field(default_factory=list)
    timings_ms: dict[str, float] = field(default_factory=dict)  # 読み取り各フェーズの所要時間 [ms]


# ==============================================================================
# 行列・スケール分離・USD プロパティ読み取り補助関数
# ==============================================================================


def _np(matrix: Gf.Matrix4d) -> np.ndarray:
    """`Gf.Matrix4d` (行ベクトル形式・第 3 行が並進) を列ベクトル形式 (第 3 列が並進) の NumPy 4x4 へ転置変換する。"""
    return np.array(matrix, dtype=float).T


def split_scale(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """列ベクトル形式 4x4 行列を `(スケール抜きの剛体変換 4x4, 各軸スケール (3,))` へ分解する (せん断なし前提)。"""
    scale = np.linalg.norm(matrix[:3, :3], axis=0)
    scale = np.where(scale < 1e-12, 1.0, scale)
    rigid = np.eye(4)
    rigid[:3, :3] = matrix[:3, :3] / scale
    if np.linalg.det(rigid[:3, :3]) < 0:  # 負のスケール (鏡像反転) は X 軸符号を反転して右手系回転行列へ戻す
        rigid[:3, 0] *= -1.0
        scale = scale * np.array([-1.0, 1.0, 1.0])
    rigid[:3, 3] = matrix[:3, 3]
    return rigid, scale


def split_scale_batch(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`split_scale` のバッチ版。形状 `(..., 4, 4)` を一括で `(rigid, scale)` へ分解する。"""
    scale = np.linalg.norm(matrix[..., :3, :3], axis=-2)
    scale = np.where(scale < 1e-12, 1.0, scale)
    rigid = np.zeros(matrix.shape)
    rigid[..., :3, :3] = matrix[..., :3, :3] / scale[..., None, :]
    mirror = np.linalg.det(rigid[..., :3, :3]) < 0  # 鏡像 (負のスケール) は X 軸を反転して右手系に戻す
    rigid[..., :3, 0] = np.where(mirror[..., None], -rigid[..., :3, 0], rigid[..., :3, 0])
    scale[..., 0] = np.where(mirror, -scale[..., 0], scale[..., 0])
    rigid[..., :3, 3] = matrix[..., :3, 3]
    rigid[..., 3, 3] = 1.0
    return rigid, scale


def _unique(name: str, used: dict[str, int]) -> str:
    """同名 prim が複数存在するときに 2 個目から `_0`, `_1`, ... を付けて一意化する。"""
    if name not in used:
        used[name] = 0
        return name
    out = f"{name}_{used[name]}"
    used[name] += 1
    return out


def _body_of(prim: Usd.Prim) -> Usd.Prim | None:
    """`prim` 自身または祖先から最も近い `UsdPhysics.RigidBodyAPI` 保有 prim を探す。"""
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def _marker_layers(stage: Usd.Stage) -> set[str]:
    """`_LAYER_MARKERS` が付いた幾何モード Sublayer の識別子集合を返す (読み取り開始時に 1 回だけ評価)。"""
    return {
        layer.identifier
        for layer in stage.GetLayerStack(includeSessionLayers=True)
        if any(layer.customLayerData.get(m) for m in _LAYER_MARKERS)
    }


def _joint_enabled(joint: UsdPhysics.Joint, marker_layers: set[str]) -> bool:
    """`physics:jointEnabled` の元の値を返す。

    なぜ `GetPropertyStack` をたどるか:
        `GeometricLayer` 適用中は Sublayer に `physics:jointEnabled = false` が書かれている。
        通常の `attr.Get()` はその `false` を返してしまうため、`marker_layers` に属するレイヤーを
        飛ばして元の USD 側の定義値を取得する。
    """
    attr = joint.GetJointEnabledAttr()
    if not attr or not attr.IsValid():
        return True
    if not marker_layers:
        value = attr.Get()
        return True if value is None else bool(value)
    for spec in attr.GetPropertyStack(Usd.TimeCode.Default()):
        if spec.layer.identifier in marker_layers:
            continue
        if spec.HasDefaultValue():
            return bool(spec.default)
    return True


def _limit(value: float | None, to_rad: bool, sign: float) -> float:
    """USD の可動範囲値を変換する。未設定・非有限・PhysX の番兵値 (`> 1e30`) は `±inf` として扱う。"""
    if value is None or not np.isfinite(value) or abs(value) > 1e30:
        return sign * np.inf
    return float(np.deg2rad(value)) if to_rad else float(value)


def component_patterns(spec: str | Sequence[str]) -> list[str]:
    """単一パターン文字列または文字列列をリストへ正規化する。"""
    return [spec] if isinstance(spec, str) else list(spec)


def prim_matches(path: str, pattern: str) -> bool:
    """`path` が `pattern` (prim パスまたは `fnmatch` パターン) 自身・配下・ワイルドカードに一致するか判定する。"""
    import fnmatch

    pattern = pattern.rstrip("/")
    return path == pattern or path.startswith(pattern + "/") or fnmatch.fnmatchcase(path, pattern)


def component_of_prim(path: str, components: dict[str, str | Sequence[str]]) -> str | None:
    """`path` が属する構成要素名を返す (複数パターンに一致する場合は最長指定を優先)。"""
    best, best_len = None, -1
    for name, spec in components.items():
        for pattern in component_patterns(spec):
            if prim_matches(path, pattern) and len(pattern) > best_len:
                best, best_len = name, len(pattern)
    return best


# ==============================================================================
# メイン読み取り関数: read_articulation
# ==============================================================================


def read_articulation(stage: Usd.Stage, source: UsdArticulationSource | str) -> tuple[GeometricRobotModel, UsdRobotInfo]:
    """USD ステージから既存ロボットを読み取り、`(GeometricRobotModel, UsdRobotInfo)` を返す (ステージ非破壊)。"""
    import time

    _t = [time.perf_counter()]

    def mark(name: str) -> None:
        now = time.perf_counter()
        info.timings_ms[name] = round((now - _t[0]) * 1e3, 3)
        _t[0] = now

    if isinstance(source, str):
        source = UsdArticulationSource(source)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    info = UsdRobotInfo(root_prim_paths=[])

    # ==========================================================================
    # Phase 1: 探索範囲の確定と Joint / ArticulationRoot / FilteredPairs 収集
    # ==========================================================================
    roots: list[Usd.Prim] = []
    for path in source.root_prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise ValueError(f"prim が無い: {path}")
        has_joint = any(p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(prim) if p != prim)
        # 一部の USD アセットは `ArticulationRootAPI` をルートリンク剛体や固定 Joint 自身に付けるため、
        # 配下に Joint が見当たらない場合は 1 段親のロボットルートへ広げて探索する
        if (prim.IsA(UsdPhysics.Joint) or not has_joint) and not prim.GetParent().IsPseudoRoot():
            info.warnings.append(f"{path} の下に Joint が無いので、親 {prim.GetParent().GetPath()} を探す範囲にした")
            prim = prim.GetParent()
        roots.append(prim)
    info.root_prim_paths = [str(p.GetPath()) for p in roots]

    ignore = set(source.ignore_joint_paths)
    joint_prims: list[Usd.Prim] = []
    filtered_prims: list[Usd.Prim] = []
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
        elif prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            filtered_prims.append(prim)
    # `ArticulationRootAPI` がロボットのルートよりさらに上位の Xform に付いているケースも漏れなく拾う
    for root in roots:
        parent = root.GetParent()
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.ArticulationRootAPI) and str(parent.GetPath()) not in info.articulation_root_paths:
                info.articulation_root_paths.append(str(parent.GetPath()))
            parent = parent.GetParent()
    info.joint_prim_paths = [str(p.GetPath()) for p in joint_prims]
    mark("traverse")

    # ==========================================================================
    # Phase 2: Joint 属性の収集とバッチ座標計算 (向き確定前)
    # ==========================================================================
    # 高速化のため USD 属性値をリストへ集約し、行列の逆変換・スケール分離を NumPy で一括処理する
    world_fixed: list[str] = []  # ワールドへ固定接続されている剛体 (根リンク候補)
    marker_layers = _marker_layers(stage)
    eye = Gf.Matrix4d(1.0)
    kept: list[tuple] = []  # (prim, type, body0, body1, axis, lower, upper, collision_enabled)
    target_m: list[Gf.Matrix4d] = []  # 接続先 prim のワールド行列 (関節ごとに 2 つ)。未指定時は単位行列
    body_m: list[Gf.Matrix4d] = []  # 接続先剛体のワールド行列。剛体なし時は単位行列
    local_pos: list = []  # `Gf.Vec3f` のまま収集し、最後に `np.array` で一括変換する (要素ごとの tuple 化を避ける)
    quat_real: list[float] = []
    quat_imag: list = []
    body_cache: dict = {}  # 接続先 `Sdf.Path` -> `(target_prim, body_prim, body_path_str)`
    zero3 = Gf.Vec3f(0.0, 0.0, 0.0)
    no_imag = Gf.Vec3f(0.0, 0.0, 0.0)
    for prim in joint_prims:
        joint = UsdPhysics.Joint(prim)
        path = str(prim.GetPath())
        if not _joint_enabled(joint, marker_layers):
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
        sides = []
        for rel, pos_attr, rot_attr in (
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
        ):
            targets = rel.GetTargets()
            if targets:
                hit = body_cache.get(targets[0])
                if hit is None:
                    target = stage.GetPrimAtPath(targets[0])
                    body = _body_of(target) if target else None
                    hit = body_cache[targets[0]] = (target, body, None if body is None else str(body.GetPath()))
                target, body, body_path = hit
            else:
                target, body, body_path = None, None, None
            if target and body is None:
                info.warnings.append(f"{path}: 接続先 {targets[0]} に剛体が無いので、ワールドへの接続として読んだ")
            quat = rot_attr.Get()
            pos = pos_attr.Get()
            sides.append(
                (
                    cache.GetLocalToWorldTransform(target) if target and target.IsValid() else eye,
                    cache.GetLocalToWorldTransform(body) if body is not None else eye,
                    zero3 if pos is None else pos,
                    1.0 if quat is None else quat.GetReal(),
                    no_imag if quat is None else quat.GetImaginary(),
                )
            )
            bodies.append(body_path)
        if bodies[0] is None and bodies[1] is None:
            info.warnings.append(f"{path}: 両端とも剛体が無いので読まない")
            continue
        if jtype == "fixed" and None in bodies:
            world_fixed.append(bodies[0] or bodies[1])  # type: ignore[arg-type]
            continue
        if None in bodies and source.world_joints == "ignore":
            # PhysX の Articulation はワールドへ開いた可動 Joint を DOF に数えないため既定では除外する
            info.warnings.append(f'{path}: ワールドに対して動く Joint なので木に入れない(DOF にするなら world_joints="dof")')
            world_fixed.append(bodies[0] or bodies[1])  # type: ignore[arg-type]
            continue
        axis_token = prim.GetAttribute("physics:axis").Get() if jtype != "fixed" and prim.GetAttribute("physics:axis") else "X"
        axis = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[axis_token or "X"]
        lower = prim.GetAttribute("physics:lowerLimit").Get() if jtype != "fixed" and prim.GetAttribute("physics:lowerLimit") else None
        upper = prim.GetAttribute("physics:upperLimit").Get() if jtype != "fixed" and prim.GetAttribute("physics:upperLimit") else None
        kept.append((prim, jtype, bodies[0], bodies[1], axis, _limit(lower, jtype == "revolute", -1.0), _limit(upper, jtype == "revolute", 1.0), joint.GetCollisionEnabledAttr().Get()))
        for t, b, pos, real, imag in sides:
            target_m.append(t)
            body_m.append(b)
            local_pos.append(pos)
            quat_real.append(real)
            quat_imag.append(imag)

    mark("joints_usd")
    # 関節フレーム計算:
    #   UsdPhysics の `localPos` は接続先 prim (`target`) のローカルスケール単位で定義される。
    #   したがって `target_scale * local_pos` でワールド長さ単位に直したうえで `inv(body_rigid) @ target_rigid @ local`
    #   を計算し、スケール抜きの剛体リンク座標系における 4x4 関節フレーム (`frame0`, `frame1`) を得る。
    #   片側がワールド接続 (`None`) のときは `target_m = body_m = eye` なので同じ式がそのままワールド座標系フレームになる。
    raw = []  # (prim, type, body0 path | None, body1 path | None, frame0, frame1, axis, lower, upper)
    if kept:
        target_rigid, target_scale = split_scale_batch(np.swapaxes(np.array(target_m, dtype=float), -1, -2))
        body_rigid, _ = split_scale_batch(np.swapaxes(np.array(body_m, dtype=float), -1, -2))
        quats = np.concatenate([np.array(quat_real, dtype=float)[:, None], np.array(quat_imag, dtype=float)], axis=1)
        local = tf.make_transform(tf.quat_to_matrix(quats), target_scale * np.array(local_pos, dtype=float))
        frames = (tf.invert(body_rigid) @ target_rigid @ local).reshape(len(kept), 2, 4, 4)
        for k, (prim, jtype, b0, b1, axis, lower, upper, collision_enabled) in enumerate(kept):
            raw.append((prim, jtype, b0, b1, frames[k, 0], frames[k, 1], axis, lower, upper))
            if collision_enabled and b0 and b1:
                info.colliding_adjacent_pairs.append((b0, b1))

    mark("joints_math")
    # ==========================================================================
    # Phase 3: 根リンク決定と幅優先探索 (BFS) による木構造化・逆向き関節正規化
    # ==========================================================================
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
    # 根リンク優先順位: (1) ワールド可動関節があれば仮想根 `None`、(2) ワールド固定剛体、
    # (3) `ArticulationRootAPI` を持つ剛体、(4) どの関節の `body1` にもなっていない剛体、(5) 先頭剛体
    if has_world_joint:
        root_body: str | None = None
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
        name = _unique(WORLD_LINK if body is None else body.rsplit("/", 1)[-1], used_links)
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
    # 剛体 -> 接続されている関節番号列 (`raw` の出現順を保持することで PhysX Articulation の DOF 順と揃える)
    incident: dict[str | None, list[int]] = {}
    for k, item in enumerate(raw):
        for b in dict.fromkeys((item[2], item[3])):
            incident.setdefault(b, []).append(k)
    head = 0
    while head < len(queue):
        cur = queue[head]
        head += 1
        for k in incident.get(cur, ()):
            if k in used_raw:
                continue
            prim, jtype, b0, b1, f0, f1, axis, lower, upper = raw[k]
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
                    # 逆向き関節 (`cur == b1`) では `parent_frame`/`child_frame` を入れ替えて `axis` を反転する。
                    # これにより `X_body1 = X_body0 @ f0 @ motion(+axis, q) @ inv(f1)` の逆式
                    # `X_body0 = X_body1 @ f1 @ motion(-axis, q) @ inv(f0)` が厳密に一致し、関節値 `q` の正負が保存される。
                    parent_frame=f0 if forward else f1,
                    child_frame=f1 if forward else f0,
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

    mark("tree")
    # ==========================================================================
    # Phase 4: mimic (従動軸) の読み取り (`NewtonMimicAPI` / `PhysxMimicJointAPI`)
    # ==========================================================================
    # 2 つのスキーマを共通式 `follower = multiplier * reference + offset` へ正規化する:
    #   - `NewtonMimicAPI`:     `follower = coef0 + coef1 * reference` -> `multiplier = coef1, offset = coef0`
    #   - `PhysxMimicJointAPI`: `follower + gearing * reference + offset = 0` -> `multiplier = -gearing, offset = -offset`
    # `offset` の単位は回転関節なら度 (deg)、直動関節ならステージの長さ単位 (PhysX で実測確認済み) のため、
    # 回転関節のみ `np.deg2rad` でラジアンへ変換する。
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

    mark("mimic")
    # ==========================================================================
    # Phase 5: ユーザー指定の `dof_order` による並び替え
    # ==========================================================================
    if source.dof_order is not None:
        dof_names = [s.name for s in specs if s.is_dof]
        if sorted(dof_names) != sorted(source.dof_order):
            raise ValueError(f"dof_order は DOF 名を過不足なく並べる。DOF: {dof_names} / 指定: {list(source.dof_order)}")
        by_name = {s.name: s for s in specs}
        ordered = iter([by_name[n] for n in source.dof_order])
        # fixed 関節の位置は変えず、可動関節スロットだけを `dof_order` 順に差し替える
        specs = [next(ordered) if s.is_dof else s for s in specs]

    # ==========================================================================
    # Phase 6: 構成要素 (`components`) と既存 `FilteredPairsAPI` の集約
    # ==========================================================================
    components: dict[str, tuple[str, ...]] = {}
    if source.components:
        for link in links:
            if link.prim_path is None:
                continue
            best = component_of_prim(link.prim_path, source.components)
            if best is not None:
                components.setdefault(best, ())
                components[best] = components[best] + (link.name,)
    info.colliding_adjacent_pairs = [(link_name_of[a], link_name_of[b]) for a, b in info.colliding_adjacent_pairs if a in link_name_of and b in link_name_of]
    path_to_link = {v: k for k, v in info.link_prim_paths.items()}
    for prim in filtered_prims:
        own_body = _body_of(prim)
        name = path_to_link.get(str(own_body.GetPath())) if own_body else None
        if name is None:
            continue
        for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets():
            other_body = _body_of(stage.GetPrimAtPath(target))
            other = path_to_link.get(str(other_body.GetPath())) if other_body else None
            if other is not None and other != name and (other, name) not in info.filtered_pairs and (name, other) not in info.filtered_pairs:
                info.filtered_pairs.append((name, other))

    mark("components_filtered_pairs")
    model = GeometricRobotModel(
        name=source.name or roots[0].GetName(),
        links=tuple(links),
        joints=tuple(specs),
        root_link=links[0].name,
        components=components,
    )
    mark("model")
    return model, info
