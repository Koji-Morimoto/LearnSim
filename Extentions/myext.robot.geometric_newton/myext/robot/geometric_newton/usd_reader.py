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
                          複数に一致した場合は最長パターンを採用する。結合ロボットの部位別操作・干渉チェックに使う。
                          `None` のまま `auto_components=True` (既定) にすると、ステージ上の `IsaacRobotAPI` から
                          構成要素を自動生成する。
    auto_components     : `True` (既定) かつ `components` が未指定のとき、探索範囲内の `IsaacRobotAPI` (入れ子構成や
                          `isaac:physics:robotLinks` 関係を含む) から `GeometricRobotComponent` を自動生成する。
                          `components` を明示指定した場合はユーザー指定が優先される。
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
    auto_components: bool = True
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
    component_prim_paths: dict[str, list[str]] = field(default_factory=dict)  # 構成要素名 -> 担当 prim パス/パターン一覧 (`set_variant` 用)
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


def _has_isaac_robot_api(prim: Usd.Prim) -> bool:
    """prim に `IsaacRobotAPI` (またはその関係属性) が付与されているか判定する。

    Pure OpenUSD 環境 (Kit の `isaacsim.robot.schema` 未登録時) では `prim.HasAPI("IsaacRobotAPI")`
    および `prim.GetAppliedSchemas()` が未登録スキーマ名を返さないことがあるため、
    `apiSchemas` メタデータの `ListOp` 生項目と `isaac:physics:robotLinks` / `robotJoints` の存在も併せて確認する。
    """
    if not prim or not prim.IsValid():
        return False
    if "IsaacRobotAPI" in prim.GetAppliedSchemas():
        return True
    try:
        if prim.HasAPI("IsaacRobotAPI"):
            return True
    except Exception:
        pass
    api_op = prim.GetMetadata("apiSchemas")
    if api_op is not None:
        items = list(api_op.explicitItems) + list(api_op.prependedItems) + list(api_op.appendedItems)
        if "IsaacRobotAPI" in items:
            return True
    for rel_name in ("isaac:physics:robotLinks", "isaac:physics:robotJoints"):
        rel = prim.GetRelationship(rel_name)
        if rel and rel.GetTargets():
            return True
    return False


def _discover_isaac_robot_components(
    stage: Usd.Stage,
    roots: Sequence[Usd.Prim],
    links: Sequence[GeometricLinkSpec],
) -> tuple[dict[str, tuple[str, ...]], dict[str, list[str]]]:
    """探索範囲内の `IsaacRobotAPI` 構成から `(components, component_prim_paths)` を自動生成する。

    設計上のルール:
        1. `roots` 配下 (および `isaac:physics:robotLinks` / `robotJoints` の参照先) にある
           すべての `IsaacRobotAPI` 付き prim を収集する。
        2. 各リンク prim の所属先は、`isaac:physics:robotLinks` による明示参照 (ただし子 `IsaacRobotAPI`
           への参照は除く) を優先し、明示参照が無ければ最深の祖先 `IsaacRobotAPI` prim へ割り当てる。
           これにより、入れ子ロボット (`/Robot` 配下に `/Robot/arm0`・`/Robot/arm0/gripper` 等) でも
           最深のサブロボットへ正確に帰属する。
        3. 構成要素名は `isaac:namespace` 属性 (空でなければ優先) または prim 名を使う。
           同名衝突 (例: `/Robot/arm0/gripper` と `/Robot/arm1/gripper`) はルートからの相対パスで一意化する。
    """
    robot_prims_by_path: dict[str, Usd.Prim] = {}
    queue: list[Usd.Prim] = []
    for root in roots:
        for p in Usd.PrimRange(root):
            if _has_isaac_robot_api(p):
                ps = str(p.GetPath())
                if ps not in robot_prims_by_path:
                    robot_prims_by_path[ps] = p
                    queue.append(p)
    # `isaac:physics:robotLinks` / `robotJoints` が参照する入れ子サブロボットも漏れなく辿る
    head = 0
    while head < len(queue):
        rp = queue[head]
        head += 1
        for rel_name in ("isaac:physics:robotLinks", "isaac:physics:robotJoints"):
            rel = rp.GetRelationship(rel_name)
            if not rel:
                continue
            for tgt in rel.GetTargets():
                tp = stage.GetPrimAtPath(tgt)
                if tp and tp.IsValid() and _has_isaac_robot_api(tp):
                    tps = str(tp.GetPath())
                    if tps not in robot_prims_by_path:
                        robot_prims_by_path[tps] = tp
                        queue.append(tp)

    if not robot_prims_by_path:
        return {}, {}

    robot_paths = list(robot_prims_by_path.keys())
    # 各 `IsaacRobotAPI` prim が `isaac:physics:robotLinks` で直接指している剛体パス集合
    explicit_targets: dict[str, set[str]] = {}
    for rpath, rp in robot_prims_by_path.items():
        rel = rp.GetRelationship("isaac:physics:robotLinks")
        bodies: set[str] = set()
        if rel:
            for tgt in rel.GetTargets():
                tps = str(tgt)
                # 子サブロボット (`IsaacRobotAPI`) 自体を指しているエントリは子側へ委譲する
                if tps in robot_prims_by_path and tps != rpath:
                    continue
                tp = stage.GetPrimAtPath(tgt)
                body = _body_of(tp) if tp and tp.IsValid() else None
                bodies.add(str(body.GetPath()) if body is not None else tps)
        explicit_targets[rpath] = bodies

    # 各リンクの帰属先 `IsaacRobotAPI` パスを決定 (明示参照 -> 祖先パスの最深一致)
    owner_links: dict[str, list[str]] = {}
    for link in links:
        lpath = link.prim_path
        if not lpath:
            continue
        matched_explicit = [rp for rp, bset in explicit_targets.items() if lpath in bset]
        if matched_explicit:
            best_rp = max(matched_explicit, key=len)
        else:
            matched_anc = [rp for rp in robot_paths if lpath == rp or lpath.startswith(rp + "/")]
            if not matched_anc:
                continue
            best_rp = max(matched_anc, key=len)
        owner_links.setdefault(best_rp, []).append(link.name)

    if not owner_links:
        return {}, {}

    # 構成要素名の決定 (`isaac:namespace` 優先、衝突時はルートからの相対パスで一意化)
    root_prefixes = [str(r.GetPath()).rstrip("/") for r in roots]

    def _base_name(rp_path: str) -> str:
        prim = robot_prims_by_path[rp_path]
        ns_attr = prim.GetAttribute("isaac:namespace")
        ns = ns_attr.Get() if ns_attr else None
        if isinstance(ns, str) and ns.strip():
            return ns.strip().strip("/")
        return prim.GetName()

    raw_names = {rp_path: _base_name(rp_path) for rp_path in owner_links}
    counts: dict[str, int] = {}
    for nm in raw_names.values():
        counts[nm] = counts.get(nm, 0) + 1

    used: dict[str, int] = {}
    components: dict[str, tuple[str, ...]] = {}
    comp_prim_paths: dict[str, list[str]] = {}
    for rp_path, lnames in owner_links.items():
        nm = raw_names[rp_path]
        if counts[nm] > 1:
            rel_name = rp_path.lstrip("/")
            for pref in root_prefixes:
                if rp_path.startswith(pref + "/"):
                    rel_name = rp_path[len(pref) + 1 :]
                    break
            nm = rel_name or nm
        final_name = _unique(nm, used)
        components[final_name] = tuple(lnames)
        comp_prim_paths[final_name] = [rp_path]
    return components, comp_prim_paths


def _resolve_root_prim(
    stage: Usd.Stage,
    prim: Usd.Prim,
    info: UsdRobotInfo,
) -> tuple[Usd.Prim, set[str]]:
    """指定された prim がリンク剛体・`ArticulationRootAPI`・`Joint` の場合、接続 Joint を含むロボットルートへ広げる。

    設計上の理由 (なぜ 1 段親だけでは不十分か):
        URDF Converter v0.3.2 や `myext.robot.assembly_pipeline` (`multi_arm.toml` 等) の USD 構成では、
        `ArticulationRootAPI` が `/robot/Geometry/base_body` (2〜3 階層下) に付き、
        一方で根の関節 (`root_joint`, `turret_joint`) は兄弟スコープ `/robot/Physics/` に置かれる。
        従来の「配下に Joint が無ければ 1 段親へ広げる」だけの実装では、
        1) `/base/Geometry/base_body` は 1 段上が `/base/Geometry` で Joint が見つからず例外になり、
        2) `/robot/Geometry/base_body` は配下のアーム群に Joint があるため一度も親へ登らず、
           `/robot/Physics/turret_joint` を取りこぼして木が分断されていた。

    解決策:
        `IsaacRobotAPI` を既に持つロボットルート prim はそのまま使う。
        それ以外で `Joint` / `RigidBodyAPI` / `ArticulationRootAPI` または配下 Joint 0 個の prim が渡された場合、
        上位の祖先をたどり、元の prim 配下の剛体 (`inner_bodies`) に接続する外部 Joint を持つ
        最小のロボット祖先 (または `IsaacRobotAPI` を持つ祖先) まで探索範囲を広げる。
        広げた場合は `seed_bodies` を返し、Phase 3 で元の prim が属する連結成分だけを抽出できるようにする。
    """
    orig_path = str(prim.GetPath())
    if prim.HasAPI("IsaacRobotAPI"):
        return prim, set()

    has_inner_joint = any(p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(prim) if p != prim)
    is_body_or_joint = (
        prim.IsA(UsdPhysics.Joint)
        or prim.HasAPI(UsdPhysics.RigidBodyAPI)
        or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    )
    if not is_body_or_joint and has_inner_joint:
        return prim, set()

    inner_bodies: set[str] = {
        str(p.GetPath()) for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    if prim.IsA(UsdPhysics.Joint):
        j = UsdPhysics.Joint(prim)
        for rel in (j.GetBody0Rel(), j.GetBody1Rel()):
            for t in rel.GetTargets():
                tp = stage.GetPrimAtPath(t)
                b = _body_of(tp) if tp else None
                if b is not None:
                    inner_bodies.add(str(b.GetPath()))

    def _has_connecting_joint_outside(anc: Usd.Prim, skip_prefix: str) -> bool:
        it = iter(Usd.PrimRange(anc))
        for p in it:
            ps = str(p.GetPath())
            if ps == skip_prefix:
                it.PruneChildren()
                continue
            if not p.IsA(UsdPhysics.Joint):
                continue
            if not inner_bodies:
                return True
            j = UsdPhysics.Joint(p)
            for rel in (j.GetBody0Rel(), j.GetBody1Rel()):
                for t in rel.GetTargets():
                    tp = stage.GetPrimAtPath(t)
                    b = _body_of(tp) if tp else None
                    if b is not None and str(b.GetPath()) in inner_bodies:
                        return True
        return False

    best = prim
    prev_prefix = orig_path
    anc = prim.GetParent()
    while anc and anc.IsValid() and not anc.IsPseudoRoot():
        anc_path = str(anc.GetPath())
        is_world_root = anc.GetParent().IsPseudoRoot() and anc.GetName().lower() == "world"
        if anc.HasAPI("IsaacRobotAPI"):
            if _has_connecting_joint_outside(anc, orig_path) or (not has_inner_joint) or prim.IsA(UsdPhysics.Joint):
                best = anc
            break
        if _has_connecting_joint_outside(anc, prev_prefix):
            best = anc
            if is_world_root:
                break
        elif is_world_root:
            break
        prev_prefix = anc_path
        anc = anc.GetParent()

    # 既存の挙動互換: Joint が全く見つからず 1 段親が有効なら 1 段親へフォールバック
    if best == prim and (prim.IsA(UsdPhysics.Joint) or not has_inner_joint) and not prim.GetParent().IsPseudoRoot():
        best = prim.GetParent()

    if best != prim:
        info.warnings.append(f"{orig_path} に接続する Joint を含めるため、親 {best.GetPath()} を探す範囲にした")
        return best, inner_bodies
    return prim, set()


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
    seed_bodies: set[str] = set()
    for path in source.root_prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise ValueError(f"prim が無い: {path}")
        resolved, seeds = _resolve_root_prim(stage, prim, info)
        roots.append(resolved)
        seed_bodies.update(seeds)
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
    world_fixed_prim_of: dict[str, Usd.Prim] = {}  # 剛体パス -> ワールド固定 Joint prim (複数サブツリー統合用)
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
            wf_body = bodies[0] or bodies[1]
            assert wf_body is not None
            world_fixed.append(wf_body)
            world_fixed_prim_of.setdefault(wf_body, prim)
            continue
        if None in bodies and source.world_joints == "ignore":
            # PhysX の Articulation はワールドへ開いた可動 Joint を DOF に数えないため既定では除外する
            info.warnings.append(f'{path}: ワールドに対して動く Joint なので木に入れない(DOF にするなら world_joints="dof")')
            wf_body = bodies[0] or bodies[1]
            assert wf_body is not None
            world_fixed.append(wf_body)
            world_fixed_prim_of.setdefault(wf_body, prim)
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
    # Phase 3: 連結成分分解・根リンク決定・幅優先探索 (BFS) による木構造化
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

    # 剛体グラフの連結成分を分解する (ワールド接続 None は辺から除いて剛体同士の連結性だけを見る)
    body_adj: dict[str, list[str]] = {b: [] for b in body_paths}
    for _, _, b0, b1, *_ in raw:
        if b0 is not None and b1 is not None:
            body_adj[b0].append(b1)
            body_adj[b1].append(b0)
    visited_comp: set[str] = set()
    body_components: list[list[str]] = []
    for b in body_paths:
        if b in visited_comp:
            continue
        comp: list[str] = []
        q_comp = [b]
        visited_comp.add(b)
        for cur_b in q_comp:
            comp.append(cur_b)
            for nb in body_adj[cur_b]:
                if nb not in visited_comp:
                    visited_comp.add(nb)
                    q_comp.append(nb)
        body_components.append(comp)

    # `_resolve_root_prim` で特定の剛体/ArticulationRoot から上位へ広げた場合、
    # 兄弟ロボットなど別成分の Joint を巻き込まないよう `seed_bodies` を含む連結成分だけを残す。
    if seed_bodies and len(body_components) > 1:
        matched_comps = [c for c in body_components if any(b in seed_bodies for b in c)]
        if matched_comps:
            body_components = matched_comps
            allowed_bodies = {b for c in body_components for b in c}
            raw = [item for item in raw if (item[2] is None or item[2] in allowed_bodies) and (item[3] is None or item[3] in allowed_bodies)]
            world_fixed = [b for b in world_fixed if b in allowed_bodies]
            body_paths = [b for b in body_paths if b in allowed_bodies]

    has_world_joint = any(b0 is None or b1 is None for _, _, b0, b1, *_ in raw)
    child_side = {b1 for _, _, b0, b1, *_ in raw if b0 is not None and b1 is not None}
    wf_set = set(world_fixed)
    body_order_idx = {b: i for i, b in enumerate(body_paths)}

    def _pick_component_root(comp_bodies: list[str]) -> str:
        """連結成分内の根リンクを優先順位に従って決定する。

        優先順位:
            1. ワールド固定 (`world_fixed`) に含まれる剛体 -> その中で `ArticulationRootAPI` 保有・パス浅い順
            2. `ArticulationRootAPI` を持つ剛体 -> パス浅い順
            3. どの関節の `body1` にもなっていない剛体 (`never_child`) -> パス浅い順
            4. 成分内の先頭剛体
        なぜ単純な `world_fixed[0]` にしないか:
            URDF Converter v0.3.2 の USD 階層では `Geometry/` が `Physics/` よりアルファベット順で先に走査される。
            もし配下のサブ部品に未無効化の `root_joint` や静的固定 Joint が混ざっていると、`world_fixed[0]` が
            深い子リンクになってしまい、本来の `ArticulationRootAPI` ルート剛体より先に選ばれて木が逆転・破綻するため。
        """

        def _rank(b: str) -> tuple[int, int, int]:
            p = stage.GetPrimAtPath(b)
            has_ar = 0 if (p and p.HasAPI(UsdPhysics.ArticulationRootAPI)) else 1
            return (has_ar, b.count("/"), body_order_idx.get(b, 0))

        comp_wf = [b for b in comp_bodies if b in wf_set]
        if comp_wf:
            return min(comp_wf, key=_rank)
        with_api = [b for b in comp_bodies if stage.GetPrimAtPath(b).HasAPI(UsdPhysics.ArticulationRootAPI)]
        if with_api:
            return min(with_api, key=_rank)
        never_child = [b for b in comp_bodies if b not in child_side]
        if never_child:
            return min(never_child, key=_rank)
        return comp_bodies[0]

    # 独立した複数サブツリー (例: 複数アームが各自ワールド/静的ベースへ固定されたマルチアームセル、
    # あるいは複数ロボットを含むルート prim 指定) がある場合は、1 つ目の成分の根から他成分をたどろうとして
    # 大量の「根からたどれない」エラーを出す代わりに、仮想根 `WORLD_LINK` (`None`) 配下へ各成分の根を
    # ワールド姿勢付き固定関節で束ねて 1 本の正確な木にする。
    if has_world_joint:
        root_body: str | None = None
    elif len(body_components) == 1:
        root_body = _pick_component_root(body_components[0])
    else:
        root_body = None
        comp_roots = [_pick_component_root(c) for c in body_components]
        for comp_root in comp_roots:
            root_prim_obj = stage.GetPrimAtPath(comp_root)
            root_world_rigid = (
                split_scale(_np(cache.GetLocalToWorldTransform(root_prim_obj)))[0]
                if root_prim_obj and root_prim_obj.IsValid()
                else np.eye(4)
            )
            wf_prim = world_fixed_prim_of.get(comp_root)
            if wf_prim is None:
                info.warnings.append(f"{comp_root}: 独立した浮遊サブツリーを仮想根 {WORLD_LINK!r} 配下に固定接続した")
            raw.append((wf_prim, "fixed", None, comp_root, root_world_rigid, np.eye(4), (1.0, 0.0, 0.0), -np.inf, np.inf))

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
            path = str(prim.GetPath()) if prim is not None else None
            if other in link_name_of:
                used_raw.add(k)
                if path is not None:
                    info.warnings.append(f"{path}: 閉じた輪を作るので木に入れない")
                continue
            used_raw.add(k)
            add_link(other)
            queue.append(other)
            forward = cur == b0
            raw_jname = prim.GetName() if prim is not None else f"{link_name_of[other]}_world_mount"
            name = _unique(raw_jname, used_joints)
            if path is not None:
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
            if jtype != "fixed" and prim is not None:
                pending_mimic[name] = prim
    for k, (prim, *_rest) in enumerate(raw):
        if k not in used_raw and prim is not None:
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
        # Pure OpenUSD 環境 (Kit の `omni.usd.schema.newton` 未登録) では `GetAppliedSchemas()` に
        # `"NewtonMimicAPI"` が出ない場合があるため、`newton:mimicJoint` リレーションの存在でも判定する。
        newton_mimic_rel = prim.GetRelationship("newton:mimicJoint")
        has_newton_mimic = ("NewtonMimicAPI" in schemas) or bool(newton_mimic_rel and newton_mimic_rel.GetTargets())
        if has_newton_mimic:
            enabled = prim.GetAttribute("newton:mimicEnabled").Get() if prim.GetAttribute("newton:mimicEnabled") else True
            if enabled is not None and not enabled:
                continue
            targets = newton_mimic_rel.GetTargets() if newton_mimic_rel else []
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
        info.component_prim_paths = {name: component_patterns(spec) for name, spec in source.components.items() if name in components}
    elif source.auto_components:
        components, info.component_prim_paths = _discover_isaac_robot_components(stage, roots, links)
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
