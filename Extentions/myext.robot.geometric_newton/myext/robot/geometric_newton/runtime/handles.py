"""ロボット操作用の補助型・ハンドル・構成要素ビュー (`GeometricJoint` / `GeometricLink` / `GeometricRobotComponent` 等)。"""

from __future__ import annotations

import dataclasses
from typing import TYPE_CHECKING, Callable, Literal, Mapping, Sequence, get_args

import numpy as np

from ..collision.newton_shapes import CollisionBackend
from ..model import AngleUnit, GeometricJointSpec, GeometricLinkSpec
from ..usd.reader import component_patterns, prim_matches

if TYPE_CHECKING:
    from .robot import GeometricRobot

__all__ = [
    "LimitPolicy",
    "StructureListener",
    "GeometricStructureChangedError",
    "GeometricLimitViolation",
    "GeometricDofBinder",
    "GeometricJoint",
    "GeometricLink",
    "GeometricRobotComponent",
]


# ==============================================================================
# 定数・例外・内部ヘルパー
# ==============================================================================

LimitPolicy = Literal["clip", "reject", "warn", "error", "ignore"]
StructureListener = Callable[[list[str], list[str]], None]  # (再構築前の DOF 名一覧, 再構築後の DOF 名一覧)
# 再構築のたびにモデルと USD から自動生成し直す `GeometricCollisionFilter` の reason 一覧。
# これ以外の reason でユーザーが追加したルールは再構築後も保持する。
_AUTO_FILTER_REASONS = ("adjacent", "mount", "usd_filtered_pairs")

# コライダーの有効状態や形状寸法に直結する USD 属性名一覧。
# `GeometricUsdWriter` が毎フレーム書き込む `xformOp:transform:geometric` を含めないことで、
# 通常の姿勢更新 (`write_target="usd"` / `flush_to_usd()`) でコライダーキャッシュを誤破棄しない。
_COLLIDER_ATTRS = frozenset(
    (
        "physics:collisionEnabled",
        "physics:approximation",
        "radius",
        "size",
        "height",
        "axis",
        "points",
        "faceVertexCounts",
        "faceVertexIndices",
    )
)


class GeometricStructureChangedError(RuntimeError):
    """ロボットの運動学構造 (DOF の並び) が変わった後に、古い構造で作った `KinematicChain` / `IKSolver` / `GeometricRobotView` を使おうとした。"""


@dataclasses.dataclass(frozen=True)
class GeometricLimitViolation:
    """関節リミットに到達・超過している 1 軸の詳細情報 (リミット到達リンクの着色・UI 警告用途)。"""

    dof_index: int
    dof_name: str
    joint_name: str
    child_link: str
    child_link_path: str | None
    parent_link: str
    side: Literal["lower", "upper"]
    requested_value: float
    actual_value: float
    lower_limit: float
    upper_limit: float
    exceeded: bool
    at_limit: bool


class GeometricDofBinder:
    """VariantSet 切替で DOF 数や並びが変わっても安全かつ高速に配列書き込みできる名前バインダー。

    設計上の意図:
        - `robot.bind_dofs(["j1", "j2", ..., "gripper_l", "suction_valve"], ignore_unknown=True)` で生成。
        - 内部に `(structure_version, src_indices, dst_indices)` を NumPy 整数配列としてキャッシュする。
        - 毎フレームの `set_positions(values)` では `structure_version` の整数比較 1 回だけで済み、
          辞書引きなしで純粋 NumPy 配列速度のまま更新できる。VariantSet 切替が起きたフレームだけ自動で
          再バインドし、新 Variant に存在しない軸はスキップして存在する軸だけ正確に書き込む。
    """

    def __init__(self, robot: "GeometricRobot", dof_names: Sequence[str], *, ignore_unknown: bool = True) -> None:
        self._robot = robot
        self._requested_names: tuple[str, ...] = tuple(dof_names)
        self._ignore_unknown = bool(ignore_unknown)
        self._version: int = -1
        self._src_idx: np.ndarray = np.zeros(0, dtype=int)
        self._dst_idx: np.ndarray = np.zeros(0, dtype=int)
        self._active_names: tuple[str, ...] = ()
        self._missing_names: tuple[str, ...] = ()
        self._rebind()

    def _rebind(self) -> None:
        self._robot._ensure_current()
        ver = self._robot.structure_version
        if ver == self._version:
            return
        name_to_idx = self._robot._dof_name_to_idx
        src: list[int] = []
        dst: list[int] = []
        active: list[str] = []
        missing: list[str] = []
        for i, nm in enumerate(self._requested_names):
            idx = name_to_idx.get(nm)
            if idx is None:
                missing.append(nm)
            else:
                src.append(i)
                dst.append(idx)
                active.append(nm)
        if missing and not self._ignore_unknown:
            raise KeyError(f"現在のロボット構成に存在しない DOF 名: {missing} (利用可能: {self._robot.dof_names})")
        self._src_idx = np.asarray(src, dtype=int)
        self._dst_idx = np.asarray(dst, dtype=int)
        self._active_names = tuple(active)
        self._missing_names = tuple(missing)
        self._version = ver

    @property
    def requested_names(self) -> tuple[str, ...]:
        return self._requested_names

    @property
    def active_names(self) -> tuple[str, ...]:
        self._rebind()
        return self._active_names

    @property
    def missing_names(self) -> tuple[str, ...]:
        self._rebind()
        return self._missing_names

    def set_positions(self, values, *, unit: AngleUnit | None = None) -> None:
        """`requested_names` 順の配列 `values` から、現在の Variant に存在する軸だけを高速適用する。"""
        self._rebind()
        arr = np.asarray(values, dtype=float).reshape(-1)
        if arr.shape[0] != len(self._requested_names):
            raise ValueError(f"values の個数 {arr.shape[0]} がバインド軸数 {len(self._requested_names)} と合わない")
        if len(self._dst_idx) == 0:
            return
        self._robot.set_dof_positions(arr[self._src_idx], dof_indices=self._dst_idx, unit=unit)

    def get_positions(self, *, fill_value: float = float("nan"), unit: AngleUnit | None = None) -> np.ndarray:
        """`requested_names` 順の配列で現在値を返す (現在の Variant に無い軸は `fill_value`)。"""
        self._rebind()
        out = np.full(len(self._requested_names), float(fill_value), dtype=float)
        if len(self._dst_idx) > 0:
            out[self._src_idx] = self._robot.get_dof_positions(dof_indices=self._dst_idx, unit=unit)
        return out


def _log_warn(message: str) -> None:
    try:
        import carb

        carb.log_warn(f"[myext.robot.geometric_newton] {message}")
    except ImportError:
        print(f"[myext.robot.geometric_newton] 警告: {message}")


def _as_indices(indices, count: int) -> np.ndarray:
    if indices is None:
        return np.arange(count)
    out = np.atleast_1d(np.asarray(indices, dtype=int))
    if out.size and (out.min() < -count or out.max() >= count):
        raise IndexError(f"番号が範囲外: {out.tolist()}(個数 {count})")
    return out


# ==============================================================================
# 関節・リンク単体ハンドル (GeometricJoint / GeometricLink)
# ==============================================================================


class GeometricJoint:
    """関節 1 つに対する軽量ハンドル。`robot.joints["名前"]` で取得する。"""

    def __init__(self, robot: "GeometricRobot", spec: GeometricJointSpec) -> None:
        self._robot = robot
        self.spec = spec

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def is_dof(self) -> bool:
        return self.spec.is_dof

    @property
    def dof_index(self) -> int:
        return self._robot.model.get_dof_index(self.spec.name)

    def get_position(self, *, unit: AngleUnit | None = None) -> float:
        return float(self._robot.get_dof_positions(dof_indices=self.dof_index, unit=unit)[0])

    def set_position(self, value: float, *, unit: AngleUnit | None = None) -> None:
        """この関節の値を変更する。mimic の従動軸に書いた値は参照軸から計算された値で上書きされる。"""
        self._robot.set_dof_positions([value], dof_indices=self.dof_index, unit=unit)

    def get_limits(self, *, unit: AngleUnit | None = None) -> tuple[float, float]:
        lower, upper = self._robot.get_dof_limits(dof_indices=self.dof_index, unit=unit)
        return float(lower[0]), float(upper[0])


class GeometricLink:
    """リンク 1 つに対する軽量ハンドル。`robot.links["名前"]` で取得する。"""

    def __init__(self, robot: "GeometricRobot", spec: GeometricLinkSpec, index: int) -> None:
        self._robot = robot
        self.spec = spec
        self.index = index

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def prim_path(self) -> str | None:
        return self._robot.link_path_map.get(self.spec.name)

    def get_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        pos, quat = self._robot.get_link_world_poses(link_indices=self.index)
        return pos[0], quat[0]

    def get_world_matrix(self) -> np.ndarray:
        return self._robot.get_link_world_matrices()[self.index]



# ==============================================================================
# 干渉チェック共通ディスパッチャと結合ロボットの部分ビュー (GeometricRobotComponent)
# ==============================================================================


def _check(
    robot: GeometricRobot,
    link_set,
    scope,
    extra_filter,
    ignore_prim_paths,
    *,
    clearance: float = 0.0,
    backend: CollisionBackend | None = None,
) -> list:
    """`GeometricRobot.check_collisions` と `GeometricRobotComponent.check_collisions` の共通バックエンド分岐。

    `backend="newton"` は `robot.scene.check_collisions` (`flush_to_usd()` 不要・0 フレーム遅延・`clearance > 0` 対応)、
    `backend="physx"` は `robot.collision_checker` (`_prepare()` で `flush_to_usd()` を実行) へ振り分ける。
    """
    from ..collision.types import CollisionScope

    robot._ensure_current()
    be = robot._collision_backend if backend is None else backend
    if be not in get_args(CollisionBackend):
        raise ValueError(f"collision_backend は {get_args(CollisionBackend)} のどれか: {be!r}")
    scope = robot._default_scope() if scope is None else scope
    if scope not in get_args(CollisionScope):
        raise ValueError(f"scope は {get_args(CollisionScope)} のどれか: {scope!r}")
    if be == "newton":
        return robot.scene.check_collisions(
            robot,
            link_set=link_set,
            scope=scope,
            clearance=clearance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
        )
    if clearance > 0.0:
        raise ValueError('clearance > 0 は collision_backend="newton" でのみ使える')
    if robot.layer.is_applied() and robot.layer.options.collision_mode == "none":
        raise RuntimeError(
            'collision_mode="none" では PhysX コライダーが無効化されているため '
            'collision_backend="physx" は使えない。既定の collision_backend="newton" を使うか、'
            'GeometricLayerOptions(collision_mode="static") を指定する'
        )
    checker = robot.collision_checker
    out = []
    if scope in ("within", "all"):
        out += checker.check_within(link_set, extra_filter=extra_filter)
    if scope in ("environment", "all"):
        out += checker.check_environment(link_set, ignore_prim_paths=ignore_prim_paths)
    return out


class GeometricRobotComponent:
    """結合したロボットの構成要素を表す部分ビュー。関節値などの状態は親の `GeometricRobot` と共有する。

    設計上の特徴:
        - 担当 DOF は「子リンクがこの構成要素に属する関節」として定義する。
        - 干渉チェックの `scope="within"` はこの構成要素内のリンク同士のみを見るため、
          `attach()` による結合の前後で意味が変わらない。
        - `self._cache` に `robot.structure_version` を保持しているため、ロボットが再構築されても
          同じ構成要素名が残っていれば自動で最新リンク・DOF 番号へ追従して使い続けられる。
    """

    def __init__(self, robot: GeometricRobot, name: str) -> None:
        self._robot = robot
        self.name = name
        self._cache: tuple[int, list[str], np.ndarray] | None = None

    def _resolve(self) -> tuple[list[str], np.ndarray]:
        robot = self._robot
        version = robot.structure_version
        if self._cache is None or self._cache[0] != version:
            model = robot.model
            if self.name not in model.components:
                raise KeyError(f"構成要素 {self.name!r} は、作り直した後のロボットに無い。ある名前: {list(model.components)}")
            links = list(model.components[self.name])
            members = set(links)
            dofs = np.array([d for d, j in enumerate(model.dof_joints) if j.child_link in members], dtype=int)
            self._cache = (version, links, dofs)
        return self._cache[1], self._cache[2]

    @property
    def robot(self) -> GeometricRobot:
        return self._robot

    @property
    def link_names(self) -> list[str]:
        return list(self._resolve()[0])

    @property
    def dof_indices(self) -> np.ndarray:
        return self._resolve()[1].copy()

    @property
    def num_dofs(self) -> int:
        return len(self._resolve()[1])

    @property
    def dof_names(self) -> list[str]:
        names = self._robot.dof_names
        return [names[d] for d in self._resolve()[1]]

    @property
    def independent_dof_indices(self) -> np.ndarray:
        """この構成要素内の独立 DOF (mimic 従動軸を除く) のローカル番号配列 (`0..num_dofs-1`)。"""
        own = self._resolve()[1]
        indep_set = set(self._robot.independent_dof_indices.tolist())
        return np.array([i for i, d in enumerate(own) if int(d) in indep_set], dtype=int)

    @property
    def independent_dof_names(self) -> list[str]:
        """この構成要素内の独立 DOF 名一覧。"""
        names = self.dof_names
        return [names[i] for i in self.independent_dof_indices]

    @property
    def link_paths(self) -> list[str | None]:
        return [self._robot.link_path_map.get(n) for n in self._resolve()[0]]

    def _map(self, dof_indices, dof_names=None) -> np.ndarray:
        own = self._resolve()[1]
        if dof_names is None:
            return own[_as_indices(dof_indices, len(own))]
        if dof_indices is not None:
            raise ValueError("dof_indices と dof_names は同時に指定できない")
        own_names = self.dof_names
        name_to_global = {n: int(g) for n, g in zip(own_names, own)}
        req = [dof_names] if isinstance(dof_names, str) else list(dof_names)
        missing = [n for n in req if n not in name_to_global]
        if missing:
            raise KeyError(f"構成要素 {self.name!r} に存在しない DOF 名: {missing} (利用可能: {own_names})")
        return np.fromiter((name_to_global[n] for n in req), dtype=int, count=len(req))

    def set_dof_positions(
        self,
        positions,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
        ignore_unknown: bool = False,
    ) -> None:
        """この構成要素の関節値を更新する。配列 (最速パス)・辞書 `{dof_name: val}`・`dof_names=` に対応。

        `ignore_unknown=True` のとき、辞書または `dof_names=` にこの構成要素へ存在しない軸名が含まれていても
        スキップして存在する軸だけを更新する。
        """
        if isinstance(positions, Mapping):
            if dof_indices is not None or dof_names is not None:
                raise ValueError("positions に辞書を渡すときは dof_indices / dof_names を併用できない")
            if not positions:
                return
            if ignore_unknown:
                own = self._resolve()[1]
                name_to_global = {n: int(g) for n, g in zip(self.dof_names, own)}
                matched = [(name_to_global[k], v) for k, v in positions.items() if k in name_to_global]
                if not matched:
                    return
                g_idx = np.fromiter((i for i, _ in matched), dtype=int, count=len(matched))
                vals = np.fromiter((v for _, v in matched), dtype=float, count=len(matched))
                self._robot.set_dof_positions(vals, dof_indices=g_idx, unit=unit)
                return
            keys = list(positions.keys())
            g_idx = self._map(None, keys)
            vals = np.fromiter((positions[k] for k in keys), dtype=float, count=len(keys))
            self._robot.set_dof_positions(vals, dof_indices=g_idx, unit=unit)
            return
        if dof_names is not None and ignore_unknown:
            if dof_indices is not None:
                raise ValueError("dof_indices と dof_names は同時に指定できない")
            own = self._resolve()[1]
            name_to_global = {n: int(g) for n, g in zip(self.dof_names, own)}
            req = [dof_names] if isinstance(dof_names, str) else list(dof_names)
            raw_vals = np.asarray(positions, dtype=float).reshape(-1)
            if raw_vals.shape[0] != len(req):
                raise ValueError(f"positions の個数 {raw_vals.shape[0]} が DOF の個数 {len(req)} と合わない")
            keep_src = [i for i, n in enumerate(req) if n in name_to_global]
            if not keep_src:
                return
            g_idx = np.fromiter((name_to_global[req[i]] for i in keep_src), dtype=int, count=len(keep_src))
            self._robot.set_dof_positions(raw_vals[keep_src], dof_indices=g_idx, unit=unit)
            return
        self._robot.set_dof_positions(positions, dof_indices=self._map(dof_indices, dof_names), unit=unit)

    def get_limit_violations(
        self,
        *,
        atol: float = 1e-6,
        include_at_limit: bool = True,
        unit: AngleUnit | None = None,
    ) -> list[GeometricLimitViolation]:
        """この構成要素に属する関節のうち、リミット到達・超過しているものの詳細一覧を返す。"""
        own_set = set(int(d) for d in self._resolve()[1].tolist())
        return [
            v
            for v in self._robot.get_limit_violations(atol=atol, include_at_limit=include_at_limit, unit=unit)
            if v.dof_index in own_set
        ]

    def get_limit_violation_links(
        self,
        *,
        atol: float = 1e-6,
        include_at_limit: bool = True,
    ) -> dict[str, Literal["lower", "upper"]]:
        """この構成要素でリミット到達・超過している子リンク名 -> `"lower"` / `"upper"` の辞書を返す。"""
        return {v.child_link: v.side for v in self.get_limit_violations(atol=atol, include_at_limit=include_at_limit)}

    def get_dof_positions(
        self,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> np.ndarray:
        return self._robot.get_dof_positions(dof_indices=self._map(dof_indices, dof_names), unit=unit)

    def get_dof_positions_dict(
        self,
        *,
        dof_names: str | Sequence[str] | None = None,
        independent_only: bool = False,
        unit: AngleUnit | None = None,
    ) -> dict[str, float]:
        """この構成要素の現在関節値を `{dof_name: value}` 辞書で返す。"""
        if dof_names is not None:
            req = [dof_names] if isinstance(dof_names, str) else list(dof_names)
            vals = self.get_dof_positions(dof_names=req, unit=unit)
            return dict(zip(req, vals.tolist()))
        if independent_only:
            l_idx = self.independent_dof_indices
            names = [self.dof_names[i] for i in l_idx]
            vals = self.get_dof_positions(dof_indices=l_idx, unit=unit)
            return dict(zip(names, vals.tolist()))
        names = self.dof_names
        vals = self.get_dof_positions(unit=unit)
        return dict(zip(names, vals.tolist()))

    def get_dof_limits(
        self,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._robot.get_dof_limits(dof_indices=self._map(dof_indices, dof_names), unit=unit)

    def get_link_world_poses(self) -> tuple[np.ndarray, np.ndarray]:
        return self._robot.get_link_world_poses(link_indices=self._robot.get_link_indices(self._resolve()[0]))

    def _patterns(self) -> list[str]:
        source = self._robot._require_source("構成要素の set_variant")
        info = self._robot.info
        if info is not None and self.name in info.component_prim_paths:
            return list(info.component_prim_paths[self.name])
        spec = (source.components or {}).get(self.name)
        if spec is None:
            raise RuntimeError(
                f"構成要素 {self.name!r} は IsaacRobotAPI 自動生成または UsdArticulationSource(components=...) 由来ではないため、prim の範囲が決まらない"
            )
        return component_patterns(spec)

    def set_variant(self, variant_set: str, variant: str, *, prim: str | None = None) -> None:
        """この構成要素の範囲にある VariantSet を切り替え、ロボットを作り直す。

        prim を省略できるのは、構成要素の範囲(UsdArticulationSource の components の指定)にその名前の VariantSet が 1 つだけのとき。
        """
        self._robot.set_variants({(prim, variant_set): variant}, _patterns=None if prim is not None else self._patterns())

    def get_variants(self) -> dict[str, dict[str, str]]:
        """この構成要素の範囲にある prim の {prim パス: {VariantSet 名: いまの選択}}。"""
        patterns = self._patterns()
        return {p: v for p, v in self._robot.get_variants().items() if any(prim_matches(p, pat) for pat in patterns)}

    @property
    def link_set(self):
        from ..collision.types import GeometricLinkSet

        return GeometricLinkSet(self._robot, self._resolve()[0], self.name)

    def check_collisions(
        self,
        scope=None,
        *,
        clearance: float = 0.0,
        extra_filter=None,
        ignore_prim_paths: Sequence[str] = (),
        backend: CollisionBackend | None = None,
    ) -> list:
        """"within" はこの構成要素のリンク同士、"environment" はロボット全体以外のコライダーとの間。"""
        return _check(self._robot, self.link_set, scope, extra_filter, ignore_prim_paths, clearance=clearance, backend=backend)

    def compute_clearances(
        self,
        scope=None,
        *,
        max_distance: float = 0.05,
        extra_filter=None,
        ignore_prim_paths: Sequence[str] = (),
    ) -> list:
        """この構成要素のリンクについて max_distance [m] 以内の組と符号付き距離を返す。"""
        self._robot._ensure_current()
        return self._robot.scene.compute_clearances(
            self._robot,
            link_set=self.link_set,
            scope=scope,
            max_distance=max_distance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
        )

    def is_in_collision(self, scope=None, **kwargs) -> bool:
        return bool(self.check_collisions(scope, **kwargs))
