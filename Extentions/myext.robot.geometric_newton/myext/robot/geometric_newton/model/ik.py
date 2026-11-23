"""IK ソルバー共通インターフェース・レジストリ・標準実装 (`JacobianIKSolver`: 減衰最小二乗法 / Levenberg-Marquardt)。

すべてのソルバーは `KinematicChain` のみを対象とし、USD / Kit / Play 状態には一切依存しない。

基本の使い方:
    chain = model.get_kinematic_chain("base", "tool")
    solver = create_ik_solver("jacobian", chain, IKSolverConfig(active_dof_mask=[True, True, True, False]))
    result = solver.solve(target_position, target_orientation, seed=current)
    solver.set_active_dofs(["joint1", "joint2"])  # 後から動かす軸を名前または bool マスクで変更可能

自作ソルバーを追加したい場合は `IKSolver` を継承して `register_ik_solver("my_solver", MySolver)` を呼ぶ。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, replace
from typing import Literal, Sequence, get_args

import numpy as np

from . import transforms as tf
from .chain import KinematicChain

__all__ = [
    "OrientationMode",
    "IKSolverName",
    "IKSolverConfig",
    "IKResult",
    "IKSolver",
    "JacobianIKSolver",
    "register_ik_solver",
    "create_ik_solver",
    "available_ik_solvers",
]

# ==============================================================================
# 設定・結果データクラス・誤差ヘルパー
# ==============================================================================

OrientationMode = Literal["full", "none"]  # `"full"`: 位置と姿勢を一致 / `"none"`: 位置のみ一致
IKSolverName = Literal["jacobian", "ikpy"]


@dataclass(frozen=True)
class IKSolverConfig:
    """IK ソルバー共通設定。

    active_dof_mask       : `chain.variable_names` ごとの可動マスク (`False` の軸は `seed` の値で固定)。`None` で全軸可動。
    orientation_mode      : `"full"` = 位置と姿勢の両方を合わせる / `"none"` = 位置のみ合わせる。
    position_tolerance    : 位置誤差ノルムの収束閾値 (モデルの長さ単位)。
    orientation_tolerance : 姿勢誤差 (回転ベクトルノルム [rad]) の収束閾値。
    max_iterations        : 1 試行あたりの最大反復回数。
    num_restarts          : 局所解に陥った際にランダム初期値でやり直す最大回数 (`JacobianIKSolver` 専用)。
    damping               : Levenberg-Marquardt 減衰係数 `lambda` の初期値 (`JacobianIKSolver` 専用)。
    orientation_weight    : 姿勢誤差 1 rad を位置誤差いくつ分に重み付けするか (`JacobianIKSolver` 専用)。
    respect_limits        : `chain.lower_limits` / `chain.upper_limits` の範囲内で解くか。
    random_seed           : リスタート時の乱数シード (同じ入力に対し完全に決定的な結果を保証するため固定既定)。
    """

    active_dof_mask: Sequence[bool] | None = None
    orientation_mode: OrientationMode = "full"
    position_tolerance: float = 1e-5
    orientation_tolerance: float = 1e-4
    max_iterations: int = 200
    num_restarts: int = 20
    damping: float = 1e-2
    orientation_weight: float = 1.0
    respect_limits: bool = True
    random_seed: int = 0

    def __post_init__(self) -> None:
        if self.orientation_mode not in get_args(OrientationMode):
            raise ValueError(f"orientation_mode は {get_args(OrientationMode)} のどれか: {self.orientation_mode!r}")


@dataclass(frozen=True)
class IKResult:
    """IK の計算結果。`dof_positions` は `chain.variable_names` 順であり、`set_dof_positions(..., dof_indices=chain.dof_indices)` へそのまま渡せる。"""

    dof_positions: np.ndarray
    success: bool
    position_error: float
    orientation_error: float  # `orientation_mode="none"` または目標姿勢未指定時は `0.0`
    iterations: int = 0
    message: str = ""


def pose_error(chain: KinematicChain, variables: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`(位置誤差ベクトル, 姿勢誤差の回転ベクトル)` を `base_link` 座標系・`目標 - 現在` の向きで返す。"""
    current = chain.forward_kinematics(variables)
    pos = target[..., :3, 3] - current[..., :3, 3]
    rot = tf.rotation_vector(target[..., :3, :3] @ np.swapaxes(current[..., :3, :3], -1, -2))
    return pos, rot


# ==============================================================================
# IKSolver 抽象基底クラス
# ==============================================================================


class IKSolver(ABC):
    """全 IK ソルバーの共通基底クラス (アクティブ軸管理・seed 検証・結果組み立てを共通化)。"""

    def __init__(self, chain: KinematicChain, config: IKSolverConfig | None = None) -> None:
        self._chain = chain
        self._config = config or IKSolverConfig()
        self._active = np.ones(chain.num_variables, dtype=bool)
        if self._config.active_dof_mask is not None:
            self.set_active_dofs(self._config.active_dof_mask)

    @property
    def chain(self) -> KinematicChain:
        return self._chain

    @property
    def config(self) -> IKSolverConfig:
        return self._config

    def set_config(self, **changes: object) -> None:
        """設定の一部を差し替える (例: `solver.set_config(orientation_mode="none")`)。"""
        self._config = replace(self._config, **changes)  # type: ignore[arg-type]
        if "active_dof_mask" in changes and changes["active_dof_mask"] is not None:
            self.set_active_dofs(changes["active_dof_mask"])  # type: ignore[arg-type]

    def set_active_dofs(self, mask_or_names: Sequence[bool] | Sequence[str]) -> None:
        """探索に使う軸を指定する (`chain.variable_names` と同長の bool 列、または可動軸名の列)。"""
        items = list(mask_or_names)
        names = list(self._chain.variable_names)
        if items and all(isinstance(v, str) for v in items):
            unknown = [v for v in items if v not in names]
            if unknown:
                raise KeyError(f"鎖の変数に無い名前: {unknown}。ある名前: {names}")
            mask = np.array([n in items for n in names], dtype=bool)
        else:
            if len(items) != len(names):
                raise ValueError(f"マスクの長さは鎖の変数の数 {len(names)} に合わせる: {len(items)}")
            mask = np.array([bool(v) for v in items], dtype=bool)
        self._active = mask
        self._on_active_changed()

    def get_active_dofs(self) -> list[bool]:
        return [bool(v) for v in self._active]

    def _on_active_changed(self) -> None:
        """アクティブ軸マスクが変更された際に派生ソルバーが内部キャッシュを再構築するためのフック。"""

    def _target_matrix(self, target_position: np.ndarray, target_orientation: np.ndarray | None) -> np.ndarray:
        """目標位置 `(3,)` と目標姿勢 (四元数 `(w, x, y, z)` または 3x3 回転行列、`None` 可) を 4x4 行列へ整形する。"""
        target = np.eye(4)
        target[:3, 3] = np.asarray(target_position, dtype=float).reshape(3)
        if target_orientation is not None:
            ori = np.asarray(target_orientation, dtype=float)
            target[:3, :3] = ori if ori.shape == (3, 3) else tf.quat_to_matrix(ori.reshape(4))
        return target

    def _seed(self, seed: np.ndarray | None) -> np.ndarray:
        chain = self._chain
        if seed is not None:
            x = np.array(seed, dtype=float).reshape(-1)
            if x.shape[0] != chain.num_variables:
                raise ValueError(f"seed の長さは鎖の変数の数 {chain.num_variables}: {x.shape[0]}")
            return x
        # `seed` 省略時は 0 (0 が可動範囲外なら最も 0 に近い範囲端) を初期値にする
        return np.clip(np.zeros(chain.num_variables), chain.lower_limits, chain.upper_limits)

    def _uses_orientation(self, target_orientation: np.ndarray | None) -> bool:
        return self._config.orientation_mode == "full" and target_orientation is not None

    def _result(self, variables: np.ndarray, target: np.ndarray, iterations: int, use_orientation: bool, message: str = "") -> IKResult:
        cfg = self._config
        pos, rot = pose_error(self._chain, variables, target)
        pos_err = float(np.linalg.norm(pos))
        rot_err = float(np.linalg.norm(rot)) if use_orientation else 0.0
        ok = pos_err <= cfg.position_tolerance and rot_err <= cfg.orientation_tolerance
        return IKResult(np.array(variables, dtype=float), ok, pos_err, rot_err, iterations, message)

    @abstractmethod
    def solve(self, target_position: np.ndarray, target_orientation: np.ndarray | None = None, *, seed: np.ndarray | None = None) -> IKResult:
        """`chain.base_link` 座標系の目標姿勢に手先を一致させる最適化変数 `x` を求める。

        `target_orientation is None` または `orientation_mode="none"` のときは位置のみ一致させる。
        `active_dof_mask` が `False` の軸は `seed` の値のまま固定される。
        """


# ==============================================================================
# JacobianIKSolver (Levenberg-Marquardt / 減衰最小二乗法 + 可動範囲アクティブセット)
# ==============================================================================


class JacobianIKSolver(IKSolver):
    """減衰最小二乗法 (DLS / Levenberg-Marquardt) による汎用 IK ソルバー。

    特徴と設計上の意図:
        - `KinematicChain` が mimic を `A, b` に畳み込んでいるため、鎖の途中に mimic 従動軸があっても
          厳密に mimic 拘束を満たしたまま解ける。
        - 特異姿勢付近でも発散しないよう、適応的ダンピング `(J^T J + lambda * diag(scale)) dx = J^T e` と
          ステップ幅上限 (`max(|dx|) <= 0.5`) を併用している。
        - 可動範囲の境界で外向き勾配を持つ軸を一時凍結 (投影アクティブセット法) することで、
          限界に突き当たった軸が他の自由度の収束を阻害する現象を防いでいる。
    """

    def solve(self, target_position: np.ndarray, target_orientation: np.ndarray | None = None, *, seed: np.ndarray | None = None) -> IKResult:
        cfg, chain = self._config, self._chain
        use_rot = self._uses_orientation(target_orientation)
        target = self._target_matrix(target_position, target_orientation)
        x0 = self._seed(seed)
        if cfg.respect_limits:
            x0 = np.where(self._active, chain.clip(x0), x0)
        rng = np.random.default_rng(cfg.random_seed)
        best: tuple[float, np.ndarray] | None = None
        total_iter = 0
        # 初回は `seed` から降下し、局所解で未収束なら `num_restarts` 回までランダム初期値で再探索する
        for attempt in range(cfg.num_restarts + 1):
            start = x0 if attempt == 0 else self._random_start(x0, rng)
            x, cost, n_iter = self._descend(start, target, use_rot)
            total_iter += n_iter
            if best is None or cost < best[0]:
                best = (cost, x)
            if self._converged(x, target, use_rot):
                break
        assert best is not None
        return self._result(best[1], target, total_iter, use_rot)

    def _converged(self, x: np.ndarray, target: np.ndarray, use_rot: bool) -> bool:
        pos, rot = _single_error(self._chain.pose_and_jacobian(x, False)[0], target)
        ok = np.linalg.norm(pos) <= self._config.position_tolerance
        return bool(ok and (not use_rot or np.linalg.norm(rot) <= self._config.orientation_tolerance))

    def _random_start(self, x0: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        chain = self._chain
        lo = np.where(np.isfinite(chain.lower_limits), chain.lower_limits, x0 - np.pi)
        hi = np.where(np.isfinite(chain.upper_limits), chain.upper_limits, x0 + np.pi)
        return np.where(self._active, rng.uniform(lo, hi), x0)

    def _descend(self, start: np.ndarray, target: np.ndarray, use_rot: bool) -> tuple[np.ndarray, float, int]:
        """Levenberg-Marquardt 法で 1 回局所降下し、`(変数 x, 重み付き二乗誤差, 反復回数)` を返す。"""
        cfg, chain = self._config, self._chain
        active = self._active
        lo = chain.lower_limits if cfg.respect_limits else np.full(chain.num_variables, -np.inf)
        hi = chain.upper_limits if cfg.respect_limits else np.full(chain.num_variables, np.inf)
        rows = 6 if use_rot else 3
        weight = np.array([1.0, 1.0, 1.0] + [cfg.orientation_weight] * 3)[:rows]
        pos_tol2 = cfg.position_tolerance**2
        rot_tol2 = cfg.orientation_tolerance**2

        def evaluate(x: np.ndarray, need_jac: bool) -> tuple[np.ndarray, np.ndarray | None, bool]:
            tip, jac = chain.pose_and_jacobian(x, need_jac)
            pos, rot = _single_error(tip, target)
            done = float(pos @ pos) <= pos_tol2 and (not use_rot or float(rot @ rot) <= rot_tol2)
            err = (np.concatenate([pos, rot]) if use_rot else pos) * weight
            return err, (None if jac is None else jac[:rows] * weight[:, None]), done

        x = start.copy()
        err, jac, done = evaluate(x, True)
        cost = float(err @ err)
        lam = cfg.damping
        n_iter = 0
        if not active.any():
            return x, cost, 0
        for n_iter in range(1, cfg.max_iterations + 1):
            if done:
                break
            assert jac is not None
            grad = jac.T @ err
            # 可動範囲の境界に貼り付き、かつ外向きへ押されている軸をこの反復の自由変数から一時除外する
            free = active & ~(((x <= lo) & (grad < 0)) | ((x >= hi) & (grad > 0)))
            idx = np.nonzero(free)[0]
            if len(idx) == 0:
                break
            jf = jac[:, idx]
            hess = jf.T @ jf
            # Marquardt スケーリング: 対角要素 `diag(J^T J)` に比例したダンピングで回転軸と直動軸の単位差を吸収する
            scale = np.maximum(np.diag(hess), 1e-9)
            accepted = False
            for _ in range(10):  # 目的関数が減るまでダンピング係数 `lam` を 4 倍ずつ強めて試行する
                step = np.linalg.solve(hess + lam * np.diag(scale), grad[idx])
                norm = float(np.max(np.abs(step)))
                if norm > 0.5:  # 非線形性が強い領域で大きく飛び過ぎないよう 1 反復の変位量を 0.5 に制限する
                    step *= 0.5 / norm
                trial = x.copy()
                trial[idx] = np.clip(trial[idx] + step, lo[idx], hi[idx])
                t_err, _, t_done = evaluate(trial, False)
                t_cost = float(t_err @ t_err)
                if t_cost < cost:
                    x, cost, done = trial, t_cost, t_done
                    err, jac, _ = evaluate(x, True)
                    # 受理時は `lam` を弱めて Gauss-Newton 収束 (二次収束) に近づける
                    lam = max(lam / 3.0, 1e-7)
                    accepted = True
                    break
                lam *= 4.0
            if not accepted:
                break
        return x, cost, n_iter


def _single_error(current: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """単一姿勢用の高速 `(位置誤差, 姿勢誤差回転ベクトル)` (`目標 - 現在`, `base_link` 座標系)。"""
    pos = target[:3, 3] - current[:3, 3]
    rel = target[:3, :3] @ current[:3, :3].T
    cos = (rel[0, 0] + rel[1, 1] + rel[2, 2] - 1.0) * 0.5
    vec = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
    sin = 0.5 * float(np.sqrt(vec @ vec))
    # 回転角が `pi` (180 deg) 付近では歪対称部分 `R - R^T` が 0 へ潰れるため四元数ベースの `tf.rotation_vector` へフォールバックする
    if cos < -0.99:
        return pos, tf.rotation_vector(rel)
    angle = float(np.arctan2(sin, cos))
    rot = 0.5 * vec if sin < 1e-9 else vec * (angle / (2.0 * sin))
    return pos, rot


# ==============================================================================
# ソルバーレジストリ (差し替え・拡張ポイント)
# ==============================================================================

_IK_SOLVERS: dict[str, type[IKSolver]] = {"jacobian": JacobianIKSolver}


def register_ik_solver(name: str, solver_class: type[IKSolver]) -> None:
    """`name` でカスタム `IKSolver` サブクラスを登録する (同名があれば上書き)。"""
    _IK_SOLVERS[name] = solver_class


def available_ik_solvers() -> list[str]:
    """登録済み IK ソルバー名の一覧を昇順で返す (`"ikpy"` も遅延登録される)。"""
    _register_optional()
    return sorted(_IK_SOLVERS)


def create_ik_solver(name: IKSolverName | str, chain: KinematicChain, config: IKSolverConfig | None = None) -> IKSolver:
    """登録名から IK ソルバーを生成する (標準で `"jacobian"` と `"ikpy"` を提供)。"""
    _register_optional()
    if name not in _IK_SOLVERS:
        raise KeyError(f"IK ソルバー {name!r} は登録されていない。ある名前: {sorted(_IK_SOLVERS)}")
    return _IK_SOLVERS[name](chain, config)


def _register_optional() -> None:
    if "ikpy" not in _IK_SOLVERS:
        from .ik_ikpy import IKPySolver

        _IK_SOLVERS["ikpy"] = IKPySolver
