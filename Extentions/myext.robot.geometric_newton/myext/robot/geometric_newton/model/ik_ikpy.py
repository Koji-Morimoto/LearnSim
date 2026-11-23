"""外部ライブラリ `ikpy` をラップする任意 IK ソルバー (`IKPySolver`)。

`ikpy` は初回使用時に遅延 import する (未導入環境でも他モジュールの動作を妨げない)。

設計上の制限 (なぜ鎖内の mimic を拒否するか):
    `ikpy.chain.Chain` は「各 `URDFLink` が互いに独立な 1 自由度を持つ」前提で内部の順運動学と
    `scipy.optimize` 目的関数を組み立てるため、`KinematicChain.joint_matrix` の線形結合 (mimic 従動軸)
    を内部へ注入できない。誤った無拘束解を返してしまうのを防ぐため、`chain.has_mimic` が `True` の鎖は
    コンストラクタで即座に `ValueError` を送出し、`JacobianIKSolver` (`"jacobian"`) の使用を案内する。
    (※ グリッパなど鎖の外側にある mimic は `chain.has_mimic` に影響しないためそのまま使える。)
"""

from __future__ import annotations

import numpy as np

from . import transforms as tf
from .chain import KinematicChain
from .ik import IKResult, IKSolver, IKSolverConfig

__all__ = ["IKPySolver", "import_ikpy"]


# ==============================================================================
# ikpy 遅延ローダー
# ==============================================================================


def import_ikpy():
    """`ikpy` を import して返す。Kit 内で未導入なら `omni.kit.pipapi.install("ikpy")` を試み、不可なら `ImportError`。"""
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
    except Exception as exc:  # Kit 外 (`pipapi` 不在) やオフライン環境など
        raise ImportError(
            "ikpy が入っていない。Isaac Sim の Python に `python.bat -m pip install ikpy` で入れるか、"
            'IK ソルバーに "jacobian" を使う。'
        ) from exc


# ==============================================================================
# IKPySolver 本体
# ==============================================================================


class IKPySolver(IKSolver):
    """`ikpy.chain.Chain` バックエンドの IK ソルバー (鎖内に mimic を含まない直列鎖専用)。"""

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
        """`set_active_dofs` でアクティブマスクが変わったときに `ikpy.chain.Chain` を再構築する。"""
        if getattr(self, "_ikpy", None) is not None:
            self._build()

    def _build(self) -> None:
        """`KinematicChain` から `OriginLink + 可動 URDFLink 列 + 末尾固定 tip` の `ikpy.chain.Chain` を組む。"""
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink

        cfg = self._config
        # `ikpy` は先頭にダミーの `OriginLink` を必須とするため、先頭と末尾 `tip` のマスクを `False` にする
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
        """`ikpy` で解き、先頭ダミー (`OriginLink`) と末尾固定 (`tip`) を除いた変数ベクトルを `IKResult` で返す。"""
        use_rot = self._uses_orientation(target_orientation)
        target = self._target_matrix(target_position, target_orientation)
        x0 = self._seed(seed)
        # `OriginLink` と `tip` のダミー要素 (0.0) を前後に挟んで `ikpy` の配列長に合わせる
        initial = np.concatenate([[0.0], x0, [0.0]])
        kwargs = dict(initial_position=initial, max_iter=self._config.max_iterations)
        assert self._ik_chain is not None
        if use_rot:
            out = self._ik_chain.inverse_kinematics_frame(target, orientation_mode="all", **kwargs)
        else:
            out = self._ik_chain.inverse_kinematics(target[:3, 3], **kwargs)
        x = np.where(self._active, np.asarray(out, dtype=float)[1:-1], x0)
        return self._result(x, target, 0, use_rot)
