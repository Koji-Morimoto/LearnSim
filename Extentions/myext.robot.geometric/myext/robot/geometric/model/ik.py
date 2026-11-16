"""IK ソルバーの共通インターフェースと、自前のソルバー(減衰最小二乗法)。

ソルバーは KinematicChain だけを相手にする。Play しているかどうか、USD があるかどうかには関係しない。

    chain = model.get_kinematic_chain("base", "tool")
    solver = create_ik_solver("jacobian", chain, IKSolverConfig(active_dof_mask=[True, True, True, False]))
    result = solver.solve(target_position, target_orientation, seed=current)
    solver.set_active_dofs(["joint1", "joint2"])      # 後から、使う軸を変える(名前でもマスクでも)

別のソルバーを足すときは IKSolver を継承して register_ik_solver("名前", クラス) を呼ぶ。
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

OrientationMode = Literal["full", "none"]  # full: 位置と向きを合わせる / none: 位置だけ合わせる
IKSolverName = Literal["jacobian", "ikpy"]


@dataclass(frozen=True)
class IKSolverConfig:
    """ソルバー共通の設定。

    active_dof_mask    : 鎖の変数(chain.variable_names)ごとに、動かすなら True。None なら全部動かす。
                         False の軸は seed の値のまま固定する
    orientation_mode   : "full" = 位置と向き、"none" = 位置だけ
    position_tolerance : 位置の誤差がこれ以下なら成功(長さの単位はモデルと同じ)
    orientation_tolerance : 向きの誤差 [rad] がこれ以下なら成功
    max_iterations     : 1 回の試行の反復回数の上限
    num_restarts       : 失敗したときに、初期値を変えてやり直す回数(jacobian のみ)
    damping            : 減衰の初期値(jacobian のみ)
    orientation_weight : 向きの誤差 1 rad を、長さいくつ分として扱うか(jacobian のみ)
    respect_limits     : 可動範囲の中だけで探すか
    random_seed        : やり直しの初期値を決める乱数の種(同じ入力なら同じ答えになる)
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
    """dof_positions は鎖の変数(chain.variable_names の順)。robot.set_dof_positions(..., dof_indices=chain.dof_indices) に渡せる。"""

    dof_positions: np.ndarray
    success: bool
    position_error: float
    orientation_error: float  # orientation_mode="none" のときは 0
    iterations: int = 0
    message: str = ""


def pose_error(chain: KinematicChain, variables: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(位置の誤差ベクトル, 向きの誤差の回転ベクトル)。どちらも base 座標系、「目標 − 現在」の向き。"""
    current = chain.forward_kinematics(variables)
    pos = target[..., :3, 3] - current[..., :3, 3]
    rot = tf.rotation_vector(target[..., :3, :3] @ np.swapaxes(current[..., :3, :3], -1, -2))
    return pos, rot


class IKSolver(ABC):
    """IK ソルバーの共通インターフェース。"""

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
        """設定の一部を変える(例: solver.set_config(orientation_mode="none"))。"""
        self._config = replace(self._config, **changes)  # type: ignore[arg-type]
        if "active_dof_mask" in changes and changes["active_dof_mask"] is not None:
            self.set_active_dofs(changes["active_dof_mask"])  # type: ignore[arg-type]

    def set_active_dofs(self, mask_or_names: Sequence[bool] | Sequence[str]) -> None:
        """動かす軸を決める。bool の列(chain.variable_names と同じ長さ)か、動かす軸の名前の列。"""
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
        """マスクが変わったときに、ソルバー内部の準備をやり直すためのフック。"""

    def _target_matrix(self, target_position: np.ndarray, target_orientation: np.ndarray | None) -> np.ndarray:
        """目標の位置 (3,) と向き(四元数 (w, x, y, z) か 3x3 行列。None なら向きは問わない)を 4x4 にする。"""
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
        # seed が無いときは 0。0 が可動範囲の外なら、範囲の中でいちばん 0 に近い値
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
        """鎖の base 座標系で与えた目標に手先を合わせる変数を求める。

        target_orientation が None のとき、または orientation_mode="none" のときは位置だけ合わせる。
        seed は探索の初期値(鎖の変数)。動かさない軸は seed の値のまま返る。
        """


class JacobianIKSolver(IKSolver):
    """減衰最小二乗法(Levenberg–Marquardt)。鎖の途中に mimic があっても、複数の軸が 1 つの変数に従っていても解ける。"""

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
        """減衰最小二乗法で 1 回下る。(変数, 重み付き誤差の 2 乗, 反復回数) を返す。

        可動範囲の端に当たって外向きに押している軸は、その反復では動かさない。
        """
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
            # 端に当たっていて、さらに外へ出ようとしている軸を外す
            free = active & ~(((x <= lo) & (grad < 0)) | ((x >= hi) & (grad > 0)))
            idx = np.nonzero(free)[0]
            if len(idx) == 0:
                break
            jf = jac[:, idx]
            hess = jf.T @ jf
            scale = np.maximum(np.diag(hess), 1e-9)
            accepted = False
            for _ in range(10):  # 誤差が減るまで減衰を強める
                step = np.linalg.solve(hess + lam * np.diag(scale), grad[idx])
                norm = float(np.max(np.abs(step)))
                if norm > 0.5:  # 1 回の移動量を抑える
                    step *= 0.5 / norm
                trial = x.copy()
                trial[idx] = np.clip(trial[idx] + step, lo[idx], hi[idx])
                t_err, _, t_done = evaluate(trial, False)
                t_cost = float(t_err @ t_err)
                if t_cost < cost:
                    x, cost, done = trial, t_cost, t_done
                    err, jac, _ = evaluate(x, True)
                    lam = max(lam / 3.0, 1e-7)
                    accepted = True
                    break
                lam *= 4.0
            if not accepted:
                break
        return x, cost, n_iter


def _single_error(current: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """1 姿勢だけの (位置の誤差, 向きの誤差の回転ベクトル)。「目標 − 現在」、base 座標系。"""
    pos = target[:3, 3] - current[:3, 3]
    rel = target[:3, :3] @ current[:3, :3].T
    cos = (rel[0, 0] + rel[1, 1] + rel[2, 2] - 1.0) * 0.5
    vec = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
    sin = 0.5 * float(np.sqrt(vec @ vec))
    if cos < -0.99:  # 180° 付近は、歪対称部分が 0 に近づくので四元数経由で求める
        return pos, tf.rotation_vector(rel)
    angle = float(np.arctan2(sin, cos))
    rot = 0.5 * vec if sin < 1e-9 else vec * (angle / (2.0 * sin))
    return pos, rot


_IK_SOLVERS: dict[str, type[IKSolver]] = {"jacobian": JacobianIKSolver}


def register_ik_solver(name: str, solver_class: type[IKSolver]) -> None:
    """名前でソルバーを登録する。同じ名前があれば置き換える。"""
    _IK_SOLVERS[name] = solver_class


def available_ik_solvers() -> list[str]:
    _register_optional()
    return sorted(_IK_SOLVERS)


def create_ik_solver(name: IKSolverName | str, chain: KinematicChain, config: IKSolverConfig | None = None) -> IKSolver:
    """登録名からソルバーを作る。標準で "jacobian"(自前)と "ikpy"(IKPy を使う)がある。"""
    _register_optional()
    if name not in _IK_SOLVERS:
        raise KeyError(f"IK ソルバー {name!r} は登録されていない。ある名前: {sorted(_IK_SOLVERS)}")
    return _IK_SOLVERS[name](chain, config)


def _register_optional() -> None:
    if "ikpy" not in _IK_SOLVERS:
        from .ik_ikpy import IKPySolver

        _IK_SOLVERS["ikpy"] = IKPySolver
