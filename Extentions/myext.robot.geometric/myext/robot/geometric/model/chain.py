"""KinematicChain: ロボットの木から取り出した、base リンクから tip リンクまでの直列部分。

IK ソルバーに渡すためのデータで、作った後はモデルにも USD にも依存しない。

    手先の姿勢(base 座標系) = Π_k [ origin_k × 動き_k(θ_k) ] × tip_transform
    θ = A @ x + b

    x : 鎖の変数。鎖の動きを決める「独立な DOF」だけを並べたもの(mimic の従動軸は入らない)
    θ : 鎖の上にある関節の値(従動軸もここには入る)

mimic はこの A と b に畳み込んであるので、順運動学とヤコビアンは常に mimic の式を厳密に満たす。
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


@dataclass(frozen=True, eq=False)
class SerialJoint:
    """鎖の上の関節 1 つ。origin は「1 つ前の関節座標系(最初は base 座標系)で見た、この関節座標系」。"""

    name: str
    joint_type: JointType
    origin: np.ndarray
    axis: tuple[float, float, float]
    lower_limit: float
    upper_limit: float
    dof_index: int  # 元のモデルの DOF 番号


@dataclass(frozen=True, eq=False)
class KinematicChain:
    base_link: str
    tip_link: str
    joints: tuple[SerialJoint, ...]
    tip_transform: np.ndarray  # 最後の関節座標系で見た手先の姿勢
    variable_names: tuple[str, ...]  # 変数 x に対応する DOF 名
    dof_indices: np.ndarray  # 変数 x に対応する、元のモデルの DOF 番号。robot.set_dof_positions(x, dof_indices=...) に渡す
    joint_matrix: np.ndarray  # A (関節数, 変数の数)
    joint_offset: np.ndarray  # b (関節数,)
    lower_limits: np.ndarray  # 変数 x の下限(従動軸の可動範囲も反映済み)
    upper_limits: np.ndarray

    # ------------------------------------------------------------------ 構築

    @classmethod
    def from_model(cls, model: "GeometricRobotModel", base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> "KinematicChain":
        path = model.path_between(base_link, tip_link)
        mimic_matrix, mimic_offset = model.mimic_matrix, model.mimic_offset
        independent = model.independent_dof_indices
        lower_all, upper_all = model.dof_lower_limits, model.dof_upper_limits

        serial: list[SerialJoint] = []
        acc = np.eye(4)  # 1 つ前の関節座標系(最初は base)から、いま見ているリンク座標系まで
        for j, forward in path:
            joint = model.joints[j]
            # 逆向きにたどる関節は、親と子の役を入れ替え、軸の向きを反転すると同じ値 q で表せる
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

        # 鎖の関節の値を決める独立な DOF を、鎖に現れる順に変数にする
        var_cols: list[int] = []  # mimic_matrix の列番号
        for sj in serial:
            for col in np.nonzero(mimic_matrix[sj.dof_index])[0]:
                if col not in var_cols:
                    var_cols.append(int(col))
        a = mimic_matrix[[sj.dof_index for sj in serial]][:, var_cols] if serial else np.zeros((0, 0))
        b = mimic_offset[[sj.dof_index for sj in serial]] if serial else np.zeros(0)
        dof_indices = independent[var_cols] if var_cols else np.zeros(0, dtype=int)
        lower = lower_all[dof_indices].astype(float) if var_cols else np.zeros(0)
        upper = upper_all[dof_indices].astype(float) if var_cols else np.zeros(0)
        # 従動軸の可動範囲を、変数の範囲に直して重ねる: lower_k <= m x + b <= upper_k
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

    # ------------------------------------------------------------------ 基本の性質

    @property
    def num_variables(self) -> int:
        return len(self.variable_names)

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    @property
    def has_mimic(self) -> bool:
        """鎖の関節と変数が 1 対 1(倍率 1、offset 0)でないなら True。"""
        a, b = self.joint_matrix, self.joint_offset
        return not (a.shape[0] == a.shape[1] and np.allclose(a, np.eye(a.shape[0])) and np.allclose(b, 0.0))

    def joint_values(self, variables: np.ndarray) -> np.ndarray:
        """変数 x (..., 変数の数) から、鎖の関節の値 θ (..., 関節数)。"""
        x = np.asarray(variables, dtype=float)
        if x.shape[-1] != self.num_variables:
            raise ValueError(f"変数の数は {self.num_variables}: {x.shape}")
        return x @ self.joint_matrix.T + self.joint_offset

    def _joint_frames(self, theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(動かす前の各関節座標系 (..., 関節数, 4, 4), 手先の姿勢 (..., 4, 4))。どちらも base 座標系。"""
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
        """base 座標系で見た手先の姿勢 (..., 4, 4)。"""
        return self._joint_frames(self.joint_values(variables))[1]

    def jacobian(self, variables: np.ndarray) -> np.ndarray:
        """手先のヤコビアン (..., 6, 変数の数)。base 座標系。行は [手先原点の速度 3 行; 角速度 3 行]。"""
        theta = self.joint_values(variables)
        frames, tip = self._joint_frames(theta)
        if self.num_joints == 0:
            return np.zeros(theta.shape[:-1] + (6, 0))
        axes = np.einsum("...kij,kj->...ki", frames[..., :3, :3], np.array([sj.axis for sj in self.joints]))
        is_rev = np.array([sj.joint_type == "revolute" for sj in self.joints], dtype=float)[:, None]
        diff = tip[..., None, :3, 3] - frames[..., :3, 3]
        lin = np.cross(axes, diff) * is_rev + axes * (1.0 - is_rev)
        ang = axes * is_rev
        jac_theta = np.swapaxes(np.concatenate([lin, ang], axis=-1), -1, -2)  # (..., 6, 関節数)
        return jac_theta @ self.joint_matrix

    def pose_and_jacobian(self, variables: np.ndarray, need_jacobian: bool = True) -> tuple[np.ndarray, np.ndarray | None]:
        """1 姿勢だけの (手先の姿勢 4x4, ヤコビアン (6, 変数の数))。IK の反復用に、まとめ計算の準備を省いた速い版。"""
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
        return np.clip(variables, self.lower_limits, self.upper_limits)

    # ------------------------------------------------------------------ 書き出し

    def to_model(self, name: str = "chain") -> "GeometricRobotModel":
        """この鎖だけの直列ロボットのモデル(根 = base、最後に固定の "tip" リンク)。

        mimic の参照軸が鎖の外にある場合は表せないので NotImplementedError。
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
        """この鎖だけの URDF(リンクは base, link1, ..., tip)。path を渡すとファイルにも書く。"""
        return self.to_model(robot_name).to_urdf(path, robot_name=robot_name)
