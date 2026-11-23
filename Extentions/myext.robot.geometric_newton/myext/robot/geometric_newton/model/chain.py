"""`KinematicChain`: ロボットの木 (`GeometricRobotModel`) から `base_link` -> `tip_link` の直列部分を取り出した鎖。

IK ソルバーへ渡す専用の自己完結データであり、生成後は `GeometricRobotModel` にも USD にも依存しない。

数式表現 (`base_link` 座標系):
    `T_tip(x) = Π_k [ origin_k @ motion_k(theta_k) ] @ tip_transform`
    `theta = joint_matrix @ x + joint_offset` (`theta = A @ x + b`)
    - `x` (`variable_names`): 鎖の姿勢を決める「独立な DOF」だけの変数ベクトル (mimic の従動軸は含めない)
    - `theta` (`joints`):     鎖上に並ぶ各可動関節の値 (mimic の従動軸も含む)

設計上の意図 (なぜ `KinematicChain` 内部に mimic を畳み込むか):
    1. **厳密な mimic 拘束の保証**:
       最適化変数 `x` を独立 DOF だけに絞り、毎回 `theta = A @ x + b` で従動軸を算出するため、
       IK の反復途中でも最終解でも mimic 関係が 1e-16 レベルで厳密に保たれる (ペナルティ項による近似誤差が出ない)。
    2. **ヤコビアン列の連鎖則合成 (`J_x = J_theta @ A`)**:
       鎖の各関節に対する 6xN 幾何ヤコビアン `J_theta` に右から `joint_matrix` (`A`) を掛けるだけで、
       自動的に `J_ref += multiplier * J_follower` の連鎖則が適用され、複数の従動軸が 1 つの参照軸に
       従う鎖でも正確な勾配が得られる。
    3. **従動軸の可動範囲の変数側への射影**:
       従動軸の制約 `lower_k <= m * x_v + b_k <= upper_k` を構築時に変数 `x_v` の `[lower_limits, upper_limits]` へ
       交差統合しているため、IK ソルバーは `x` の単純な箱制約 (clip) だけで従動軸の可動範囲も同時に守れる。
    4. **根を通らない任意 2 リンク間の鎖**:
       `path_between()` の逆向き区間 (`forward=False`) は `parent_frame` / `child_frame` を入れ替え `axis` を反転して
       直列化するため、左腕手先から右腕手先までといった非根経由の IK も同一式で解ける。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from . import transforms as tf
from .spec import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, JointType

if TYPE_CHECKING:
    from .spec import GeometricRobotModel

__all__ = ["SerialJoint", "KinematicChain"]


# ==============================================================================
# データクラス定義 (SerialJoint / KinematicChain)
# ==============================================================================


@dataclass(frozen=True, eq=False)
class SerialJoint:
    """直列鎖上の可動関節 1 つ。

    `origin` は 1 つ前の関節座標系 (先頭関節では `base_link` 座標系) から見た、この関節の座標系 (4x4)。
    間の `fixed` 関節や `inv(child_frame)` はすべて `origin` に事前合成して畳み込んである。
    """

    name: str
    joint_type: JointType
    origin: np.ndarray
    axis: tuple[float, float, float]
    lower_limit: float
    upper_limit: float
    dof_index: int  # 元の `GeometricRobotModel` における DOF インデックス


@dataclass(frozen=True, eq=False)
class KinematicChain:
    """`base_link` -> `tip_link` の直列運動学鎖 (IK 用)。"""

    base_link: str
    tip_link: str
    joints: tuple[SerialJoint, ...]
    tip_transform: np.ndarray  # 最終関節座標系から見た手先 (`tip_link @ tip_offset`) の 4x4 変換
    variable_names: tuple[str, ...]  # 最適化変数 `x` に対応する独立 DOF 名
    dof_indices: np.ndarray  # 変数 `x` に対応する元モデルの DOF インデックス (`set_dof_positions(..., dof_indices=...)` 用)
    joint_matrix: np.ndarray  # mimic 写像行列 `A` (`(num_joints, num_variables)`)
    joint_offset: np.ndarray  # mimic オフセット `b` (`(num_joints,)`)
    lower_limits: np.ndarray  # 変数 `x` の下限 (従動軸側の可動範囲も射影・統合済み)
    upper_limits: np.ndarray  # 変数 `x` の上限 (従動軸側の可動範囲も射影・統合済み)
    # 鎖を生成した `GeometricRobot` とその `structure_version` の組 (VariantSet 切替後の古い鎖の誤用を検知するため)
    source: object = None

    # ==========================================================================
    # 構築 (`GeometricRobotModel` からの切り出し)
    # ==========================================================================

    @classmethod
    def from_model(cls, model: "GeometricRobotModel", base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> "KinematicChain":
        """`model` の `base_link` -> `tip_link` 経路から `KinematicChain` を構築する。"""
        path = model.path_between(base_link, tip_link)
        mimic_matrix, mimic_offset = model.mimic_matrix, model.mimic_offset
        independent = model.independent_dof_indices
        lower_all, upper_all = model.dof_lower_limits, model.dof_upper_limits

        serial: list[SerialJoint] = []
        # `acc` は「直前の可動関節の運動直後フレーム (先頭では `base_link`) -> 現在のリンク座標系」の累積変換。
        # 中間の `fixed` 関節はここで `acc` へ吸収し、IK 反復時のループ長を可動関節数だけへ短縮する。
        acc = np.eye(4)
        for j, forward in path:
            joint = model.joints[j]
            # 逆向きにたどる関節 (`forward=False`) は `parent_frame` / `child_frame` を入れ替え `axis` を反転する
            frame_in, frame_out = (joint.parent_frame, joint.child_frame) if forward else (joint.child_frame, joint.parent_frame)
            axis = np.array(joint.axis) if forward else -np.array(joint.axis)
            acc = acc @ frame_in
            if joint.is_dof:
                serial.append(
                    SerialJoint(joint.name, joint.joint_type, acc.copy(), tuple(float(v) for v in axis), joint.lower_limit, joint.upper_limit, model.get_dof_index(joint.name))
                )
                acc = tf.invert(frame_out)
            else:
                acc = acc @ tf.invert(frame_out)
        tip_transform = acc @ (np.eye(4) if tip_offset is None else np.asarray(tip_offset, dtype=float))

        # 鎖上の各関節 (従動軸を含む) に影響を与える独立 DOF 列だけを出現順に集めて変数 `x` とする
        var_cols: list[int] = []  # `model.mimic_matrix` の列番号
        for sj in serial:
            for col in np.nonzero(mimic_matrix[sj.dof_index])[0]:
                if col not in var_cols:
                    var_cols.append(int(col))
        a = mimic_matrix[[sj.dof_index for sj in serial]][:, var_cols] if serial else np.zeros((0, 0))
        b = mimic_offset[[sj.dof_index for sj in serial]] if serial else np.zeros(0)
        dof_indices = independent[var_cols] if var_cols else np.zeros(0, dtype=int)
        lower = lower_all[dof_indices].astype(float) if var_cols else np.zeros(0)
        upper = upper_all[dof_indices].astype(float) if var_cols else np.zeros(0)
        # 従動軸の可動範囲 `lower_k <= m * x_v + b_k <= upper_k` を独立変数 `x_v` の範囲へ射影して交差を取る
        # (`m < 0` の逆倍率では不等号が反転するため `lo` と `hi` を入れ替える)
        for k, sj in enumerate(serial):
            cols = np.nonzero(a[k])[0]
            if len(cols) != 1:
                continue
            v, m = int(cols[0]), float(a[k, cols[0]])
            lo, hi = (sj.lower_limit - b[k]) / m, (sj.upper_limit - b[k]) / m
            if m < 0:
                lo, hi = hi, lo
            lower[v], upper[v] = max(lower[v], lo), min(upper[v], hi)
        dof_names = model.dof_names
        return cls(
            base_link=base_link,
            tip_link=tip_link,
            joints=tuple(serial),
            tip_transform=tip_transform,
            variable_names=tuple(dof_names[d] for d in dof_indices),
            dof_indices=np.asarray(dof_indices, dtype=int),
            joint_matrix=a,
            joint_offset=b,
            lower_limits=lower,
            upper_limits=upper,
        )

    # ==========================================================================
    # 順運動学とヤコビアン (バッチ版・IK 反復向け単一姿勢高速版)
    # ==========================================================================

    @property
    def num_variables(self) -> int:
        return len(self.variable_names)

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    @property
    def has_mimic(self) -> bool:
        """鎖の関節と最適化変数 `x` が恒等 1 対 1 (`A == I` かつ `b == 0`) でないなら `True` (`IKPySolver` の可否判定に使用)。"""
        a, b = self.joint_matrix, self.joint_offset
        return not (a.shape[0] == a.shape[1] and np.allclose(a, np.eye(a.shape[0])) and np.allclose(b, 0.0))

    def joint_values(self, variables: np.ndarray) -> np.ndarray:
        """最適化変数 `x (..., num_variables)` から鎖上の全関節値 `theta (..., num_joints)` を計算する (`x @ A^T + b`)。"""
        x = np.asarray(variables, dtype=float)
        if x.shape[-1] != self.num_variables:
            raise ValueError(f"変数の数は {self.num_variables}: {x.shape}")
        return x @ self.joint_matrix.T + self.joint_offset

    def _joint_frames(self, theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """`(各関節の運動前フレーム (..., num_joints, 4, 4), 手先姿勢 (..., 4, 4))` を `base_link` 座標系で返す。"""
        batch = theta.shape[:-1]
        frames = np.zeros(batch + (self.num_joints, 4, 4))
        cur = np.broadcast_to(np.eye(4), batch + (4, 4)).copy()
        for k, sj in enumerate(self.joints):
            cur = cur @ sj.origin
            frames[..., k, :, :] = cur
            motion = np.broadcast_to(np.eye(4), batch + (4, 4)).copy()
            if sj.joint_type == "revolute":
                motion[..., :3, :3] = tf.rotation_about_axis(np.array(sj.axis), theta[..., k])
            else:
                motion[..., :3, 3] = theta[..., k, None] * np.array(sj.axis)
            cur = cur @ motion
        return frames, cur @ self.tip_transform

    def forward_kinematics(self, variables: np.ndarray) -> np.ndarray:
        """`base_link` 座標系で見た手先の 4x4 姿勢 `(..., 4, 4)` を返す。"""
        return self._joint_frames(self.joint_values(variables))[1]

    def jacobian(self, variables: np.ndarray) -> np.ndarray:
        """`base_link` 座標系における手先の幾何ヤコビアン `(..., 6, num_variables)` (行順: `[線速度 3; 角速度 3]`)。

        関節ごとのヤコビアン `jac_theta (..., 6, num_joints)` に右から `joint_matrix` を掛けることで、
        連鎖則 `J_ref += multiplier * J_follower` を自動的に適用している。
        """
        theta = self.joint_values(variables)
        frames, tip = self._joint_frames(theta)
        if self.num_joints == 0:
            return np.zeros(theta.shape[:-1] + (6, 0))
        axes = np.einsum("...kij,kj->...ki", frames[..., :3, :3], np.array([sj.axis for sj in self.joints]))
        is_rev = np.array([sj.joint_type == "revolute" for sj in self.joints], dtype=float)[:, None]
        diff = tip[..., None, :3, 3] - frames[..., :3, 3]
        lin = np.cross(axes, diff) * is_rev + axes * (1.0 - is_rev)
        ang = axes * is_rev
        jac_theta = np.swapaxes(np.concatenate([lin, ang], axis=-1), -1, -2)  # (..., 6, num_joints)
        return jac_theta @ self.joint_matrix

    def pose_and_jacobian(self, variables: np.ndarray, need_jacobian: bool = True) -> tuple[np.ndarray, np.ndarray | None]:
        """単一姿勢専用の高速 `(手先姿勢 4x4, ヤコビアン (6, num_variables))`。

        `JacobianIKSolver` の反復ループ内で毎ステップ呼ばれるため、バッチ用ブロードキャスト配列の確保と
        `rotation_about_axis` 呼び出しを省き、Rodrigues 回転行列をスカラー展開して 1 パスで手先姿勢とヤコビアンを求める。
        """
        theta = self.joint_matrix @ np.asarray(variables, dtype=float) + self.joint_offset
        cur = np.eye(4)
        n = self.num_joints
        axes = np.empty((n, 3))
        origins = np.empty((n, 3))
        rev = np.empty(n, dtype=bool)
        for k, sj in enumerate(self.joints):
            cur = cur @ sj.origin
            ax = np.array(sj.axis)
            axes[k] = cur[:3, :3] @ ax
            origins[k] = cur[:3, 3]
            motion = np.eye(4)
            if sj.joint_type == "revolute":
                rev[k] = True
                x, y, z = ax
                s, c = np.sin(theta[k]), np.cos(theta[k])
                v = 1.0 - c
                motion[:3, :3] = (
                    (c + x * x * v, x * y * v - z * s, x * z * v + y * s),
                    (y * x * v + z * s, c + y * y * v, y * z * v - x * s),
                    (z * x * v - y * s, z * y * v + x * s, c + z * z * v),
                )
            else:
                rev[k] = False
                motion[:3, 3] = theta[k] * ax
            cur = cur @ motion
        tip = cur @ self.tip_transform
        if not need_jacobian:
            return tip, None
        jac = np.zeros((6, n))
        if n:
            lin = np.where(rev[:, None], np.cross(axes, tip[:3, 3] - origins), axes)
            jac[:3] = lin.T
            jac[3:] = (axes * rev[:, None]).T
        return tip, jac @ self.joint_matrix

    def clip(self, variables: np.ndarray) -> np.ndarray:
        """最適化変数 `variables` を `[lower_limits, upper_limits]` へ切り詰める。"""
        return np.clip(variables, self.lower_limits, self.upper_limits)

    # ==========================================================================
    # モデル・URDF への逆変換
    # ==========================================================================

    def to_model(self, name: str = "chain") -> "GeometricRobotModel":
        """この鎖だけから成る直列 `GeometricRobotModel` (根=`base`、末尾固定リンク=`tip`) を生成する。

        mimic の参照軸が鎖の外にある場合は単独の直列モデルで表現できないため `NotImplementedError` を送出する。
        """
        from .spec import GeometricRobotModel

        on_chain = {sj.dof_index: sj.name for sj in self.joints}
        specs: list[GeometricJointSpec] = []
        links = [GeometricLinkSpec("base")]
        for k, sj in enumerate(self.joints):
            cols = np.nonzero(self.joint_matrix[k])[0]
            if len(cols) != 1:
                raise NotImplementedError(f"関節 {sj.name} は複数の変数に従うので、直列モデルにできない")
            var_dof = int(self.dof_indices[cols[0]])
            mimic = None
            if var_dof != sj.dof_index:
                if var_dof not in on_chain:
                    raise NotImplementedError(f"関節 {sj.name} の mimic の参照軸が鎖の外にあるので、直列モデルにできない")
                mimic = GeometricMimicSpec(on_chain[var_dof], float(self.joint_matrix[k, cols[0]]), float(self.joint_offset[k]))
            child = f"link{k + 1}"
            specs.append(
                GeometricJointSpec(sj.name, sj.joint_type, links[-1].name, child, parent_frame=sj.origin, axis=sj.axis, lower_limit=sj.lower_limit, upper_limit=sj.upper_limit, mimic=mimic)
            )
            links.append(GeometricLinkSpec(child))
        specs.append(GeometricJointSpec("tip_fixed", "fixed", links[-1].name, "tip", parent_frame=self.tip_transform))
        links.append(GeometricLinkSpec("tip"))
        return GeometricRobotModel(name=name, links=tuple(links), joints=tuple(specs), root_link="base")

    def to_urdf(self, path: str | None = None, *, robot_name: str = "chain") -> str:
        """この鎖だけの URDF XML (リンク: `base`, `link1`, ..., `tip`) を返す (`path` 指定時はファイルにも保存)。"""
        return self.to_model(robot_name).to_urdf(path, robot_name=robot_name)
