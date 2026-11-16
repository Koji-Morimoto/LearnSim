"""IKPy を使う IK ソルバー。ikpy は使うときに初めて import する(入っていなければ入れ方を示して止まる)。

IKPy は自分で順運動学を持つ直列の鎖だけを扱うので、鎖の途中に mimic の従動軸がある場合は使えない
(その場合は "jacobian" を使う)。鎖の外にある mimic(グリッパなど)は影響しない。
"""

from __future__ import annotations

import numpy as np

from . import transforms as tf
from .chain import KinematicChain
from .ik import IKResult, IKSolver, IKSolverConfig

__all__ = ["IKPySolver", "import_ikpy"]


def import_ikpy():
    """ikpy を import して返す。Kit の中で無ければ omni.kit.pipapi で入れる。それも無理なら ImportError。"""
    try:
        import ikpy.chain  # noqa: F401
        import ikpy.link  # noqa: F401

        return ikpy
    except ImportError:
        pass
    try:
        import omni.kit.pipapi

        omni.kit.pipapi.install("ikpy")
        import ikpy.chain  # noqa: F401
        import ikpy.link  # noqa: F401

        return ikpy
    except Exception as exc:  # pipapi が無い(Kit の外)・ネットワークに出られない、など
        raise ImportError(
            "ikpy が入っていない。Isaac Sim の Python に `python.bat -m pip install ikpy` で入れるか、"
            'IK ソルバーに "jacobian" を使う。'
        ) from exc


class IKPySolver(IKSolver):
    def __init__(self, chain: KinematicChain, config: IKSolverConfig | None = None) -> None:
        if chain.has_mimic:
            raise ValueError(
                "IKPySolver は、鎖の途中に mimic の従動軸がある鎖を扱えない。"
                f'鎖の関節 {[j.name for j in chain.joints]} / 変数 {list(chain.variable_names)}。"jacobian" を使う'
            )
        self._ikpy = import_ikpy()
        self._ik_chain = None
        super().__init__(chain, config)
        self._build()

    def _on_active_changed(self) -> None:
        if getattr(self, "_ikpy", None) is not None:
            self._build()

    def _build(self) -> None:
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink

        cfg = self._config
        links = [OriginLink()]
        for sj in self._chain.joints:
            lo = sj.lower_limit if (cfg.respect_limits and np.isfinite(sj.lower_limit)) else -np.inf
            hi = sj.upper_limit if (cfg.respect_limits and np.isfinite(sj.upper_limit)) else np.inf
            common = dict(
                name=sj.name,
                origin_translation=sj.origin[:3, 3],
                origin_orientation=np.array(tf.matrix_to_rpy(sj.origin[:3, :3])),
                bounds=(lo, hi),
                joint_type=sj.joint_type,
            )
            if sj.joint_type == "revolute":
                links.append(URDFLink(rotation=np.array(sj.axis), **common))
            else:
                links.append(URDFLink(translation=np.array(sj.axis), **common))
        tip = self._chain.tip_transform
        links.append(URDFLink(name="tip", origin_translation=tip[:3, 3], origin_orientation=np.array(tf.matrix_to_rpy(tip[:3, :3])), joint_type="fixed"))
        mask = [False] + [bool(v) for v in self._active] + [False]
        self._ik_chain = Chain(name="chain", links=links, active_links_mask=mask)

    def solve(self, target_position: np.ndarray, target_orientation: np.ndarray | None = None, *, seed: np.ndarray | None = None) -> IKResult:
        use_rot = self._uses_orientation(target_orientation)
        target = self._target_matrix(target_position, target_orientation)
        x0 = self._seed(seed)
        initial = np.concatenate([[0.0], x0, [0.0]])
        kwargs = dict(initial_position=initial, max_iter=self._config.max_iterations)
        assert self._ik_chain is not None
        if use_rot:
            out = self._ik_chain.inverse_kinematics_frame(target, orientation_mode="all", **kwargs)
        else:
            out = self._ik_chain.inverse_kinematics(target[:3, 3], **kwargs)
        x = np.where(self._active, np.asarray(out, dtype=float)[1:-1], x0)
        return self._result(x, target, 0, use_rot)
