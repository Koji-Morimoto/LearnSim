"""GeometricRobot: 物理を使わず、関節値からリンクの Transform を計算して書くロボット。

メソッド名・引数・単位(角度 rad、長さはステージの単位)は isaacsim.core.experimental.prims.Articulation に合わせてある。
違い: 1 台用なので配列は (num_dofs,) の numpy で、indices= 引数は無い。複数台をまとめて同じ形で扱うなら GeometricRobotView。
速度・力・ゲイン・質量などのメソッドは無い。

    robot = GeometricRobot.from_articulation(stage, "/World/Robot")   # Sublayer を付けて幾何モードにする
    robot.set_dof_positions([0.1, -0.5, 0.8, 0.0, 0.3, 0.0])
    robot.check_collisions()
    robot.layer.remove()                                             # 元の Articulation 構成に戻る
"""

from __future__ import annotations

from typing import Literal, Sequence, get_args

import numpy as np
from pxr import Usd, UsdGeom

from .layer import GeometricLayer, GeometricLayerOptions, _setting
from .model import (
    DHParameters,
    GeometricCollisionFilter,
    GeometricJointSpec,
    GeometricLinkSpec,
    GeometricRobotModel,
    IKSolver,
    IKSolverConfig,
    IKSolverName,
    KinematicChain,
    MDHParameters,
    ScrewParameters,
    UrdfSource,
    create_ik_solver,
)
from .model import transforms as tf
from .usd_reader import UsdArticulationSource, UsdRobotInfo, _np, read_articulation, split_scale
from .writer import GeometricFabricWriter, GeometricUsdWriter, LocalPoseSolver, WriteTarget

__all__ = ["LimitPolicy", "GeometricRobot", "GeometricRobotComponent", "GeometricJoint", "GeometricLink"]

LimitPolicy = Literal["clip", "warn", "error", "ignore"]
Indices = "int | Sequence[int] | np.ndarray | None"


def _log_warn(message: str) -> None:
    try:
        import carb

        carb.log_warn(f"[myext.robot.geometric] {message}")
    except ImportError:
        print(f"[myext.robot.geometric] 警告: {message}")


def _as_indices(indices, count: int) -> np.ndarray:
    if indices is None:
        return np.arange(count)
    out = np.atleast_1d(np.asarray(indices, dtype=int))
    if out.size and (out.min() < -count or out.max() >= count):
        raise IndexError(f"番号が範囲外: {out.tolist()}(個数 {count})")
    return out


class GeometricJoint:
    """関節 1 つの窓口。robot.joints["名前"] で得る。"""

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

    def get_position(self) -> float:
        return float(self._robot.get_dof_positions(dof_indices=self.dof_index)[0])

    def set_position(self, value: float) -> None:
        """この関節の値を変える。mimic の従動軸に書いた値は、参照軸の値で上書きされる。"""
        self._robot.set_dof_positions([value], dof_indices=self.dof_index)

    def get_limits(self) -> tuple[float, float]:
        lower, upper = self._robot.get_dof_limits(dof_indices=self.dof_index)
        return float(lower[0]), float(upper[0])


class GeometricLink:
    """リンク 1 つの窓口。robot.links["名前"] で得る。"""

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


class GeometricRobot:
    def __init__(
        self,
        stage: Usd.Stage,
        model: GeometricRobotModel,
        link_prim_paths: dict[str, str],
        *,
        info: UsdRobotInfo | None = None,
        layer: GeometricLayer | None = None,
        layer_options: GeometricLayerOptions | None = None,
        write_target: WriteTarget | None = None,
        limit_policy: LimitPolicy = "clip",
        apply_layer: bool = True,
    ) -> None:
        """ふつうは from_articulation / from_model / from_dh / from_mdh / from_screw / from_urdf を使う。

        link_prim_paths : リンク名 -> そのリンクの prim パス(prim の親子関係はリンクの親子関係と違っていてよい)
        layer           : 共有する Sublayer。省略すると、このロボット専用の Sublayer を作る
        write_target    : "fabric" = Fabric に書く(速い。干渉チェックの直前だけ USD にも書く) / "usd" = 毎回 USD に書く /
                          "both" = 毎回両方に書く。省略時は拡張機能の設定 write_target(既定 "fabric")
        limit_policy    : 可動範囲の外の値を渡されたとき。"clip" = 範囲内に切り詰める / "warn" = 警告して切り詰める /
                          "error" = 例外 / "ignore" = そのまま使う
        apply_layer     : True なら作った直後に Sublayer を付ける(幾何モードにする)
        """
        if limit_policy not in get_args(LimitPolicy):
            raise ValueError(f"limit_policy は {get_args(LimitPolicy)} のどれか: {limit_policy!r}")
        target = write_target if write_target is not None else _setting("write_target", "fabric")
        if target not in get_args(WriteTarget):
            raise ValueError(f"write_target は {get_args(WriteTarget)} のどれか: {target!r}")
        self._stage = stage
        self._model = model
        self._info = info
        self._limit_policy: LimitPolicy = limit_policy
        self._write_target: WriteTarget = target
        self._link_path_map = {name: path for name, path in link_prim_paths.items() if path}
        unknown = [n for n in self._link_path_map if n not in model.link_names]
        if unknown:
            raise ValueError(f"link_prim_paths にモデルに無いリンク名がある: {unknown}")
        self._prim_link_indices = np.array([i for i, n in enumerate(model.link_names) if n in self._link_path_map], dtype=int)
        self._prim_paths = [self._link_path_map[model.link_names[i]] for i in self._prim_link_indices]
        self._solver = LocalPoseSolver(stage, self._prim_paths)

        self._lower = model.dof_lower_limits
        self._upper = model.dof_upper_limits
        self._q = model.resolve_mimic(np.clip(np.zeros(model.num_dofs), self._lower, self._upper))
        root = model.root_link or ""
        if root in self._link_path_map:
            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            self._base = split_scale(_np(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._link_path_map[root]))))[0]
        else:
            self._base = np.eye(4)
        self._default_q = self._q.copy()
        self._default_base = self._base.copy()
        self._world = self._base @ model.forward_kinematics(self._q, resolve_mimic=False)

        # 干渉チェック
        self._filter = GeometricCollisionFilter.from_model(model)
        if info is not None:
            for a, b in info.colliding_adjacent_pairs:  # Joint が「つながった 2 物体も衝突させる」設定のとき
                self._filter.remove_pair(a, b)
            for a, b in info.filtered_pairs:
                self._filter.add_pair(a, b, reason="usd_filtered_pairs")
        self._colliders: dict[str, list[str]] | None = None
        self._checker = None
        self._components: dict[str, GeometricRobotComponent] = {}

        # Sublayer と書き込み先
        self._layer = layer if layer is not None else GeometricLayer(stage, layer_options)
        joint_paths = list(info.joint_prim_paths) if info is not None else []
        adjacent = [(self._link_path_map[a], self._link_path_map[b]) for a, b, _ in model.adjacent_link_pairs() if a in self._link_path_map and b in self._link_path_map]
        self._usd_writer = GeometricUsdWriter(self._layer, self._prim_paths)
        self._fabric_writer: GeometricFabricWriter | None = None
        self._fabric_failed = False
        self._usd_dirty = False
        self._repush_frames = 0
        self._update_subscription = None
        self._closed = False
        self._flush_delegate = None  # GeometricRobotView に入っているとき、USD への書き込みを View に任せる
        self._layer.register(
            link_prim_paths=self._prim_paths,
            articulation_root_paths=list(info.articulation_root_paths) if info is not None else [],
            joint_prim_paths=joint_paths,
            adjacent_link_paths=adjacent,
        )
        self._layer.add_listener(self._on_layer_changed)
        self.joints = {spec.name: GeometricJoint(self, spec) for spec in model.joints}
        self.links = {spec.name: GeometricLink(self, spec, i) for i, spec in enumerate(model.links)}
        if self._layer.is_applied():
            self._on_layer_changed(True)
        elif apply_layer:
            self._layer.apply()

    # ------------------------------------------------------------------ 作り方

    @classmethod
    def from_articulation(cls, stage: Usd.Stage, source: UsdArticulationSource | str, **kwargs) -> "GeometricRobot":
        """ステージ上の既存ロボット(ArticulationRoot と Joint を持つ prim)から作る。元の USD は書き換えない。

        source は prim パスか UsdArticulationSource。kwargs は __init__ と同じ(layer, layer_options, write_target, limit_policy, apply_layer)。
        """
        model, info = read_articulation(stage, source)
        for message in info.warnings:
            _log_warn(message)
        return cls(stage, model, info.link_prim_paths, info=info, **kwargs)

    @classmethod
    def from_model(cls, stage: Usd.Stage, model: GeometricRobotModel, prim_path: str, *, style=None, base_transform: np.ndarray | None = None, **kwargs) -> "GeometricRobot":
        """内部モデルから、簡易形状の prim を prim_path の下に新しく組んで作る(Articulation も剛体も付けない)。

        style は GeometricVisualStyle(太さ・色・コライダーを付けるか)。base_transform はワールドで見た根リンクの姿勢(4x4)。
        """
        from .builder import build_robot_prims

        link_paths = build_robot_prims(stage, model, prim_path, style=style, base_transform=base_transform)
        return cls(stage, model, link_paths, **kwargs)

    @classmethod
    def from_dh(cls, stage: Usd.Stage, params: DHParameters, prim_path: str, **kwargs) -> "GeometricRobot":
        """標準 DH パラメータから作る。"""
        return cls.from_model(stage, params.to_model(), prim_path, **kwargs)

    @classmethod
    def from_mdh(cls, stage: Usd.Stage, params: MDHParameters, prim_path: str, **kwargs) -> "GeometricRobot":
        """修正 DH パラメータから作る。"""
        return cls.from_model(stage, params.to_model(), prim_path, **kwargs)

    @classmethod
    def from_screw(cls, stage: Usd.Stage, params: ScrewParameters, prim_path: str, **kwargs) -> "GeometricRobot":
        """スクリュー軸から作る。"""
        return cls.from_model(stage, params.to_model(), prim_path, **kwargs)

    @classmethod
    def from_urdf(cls, stage: Usd.Stage, source: UrdfSource | str, prim_path: str, **kwargs) -> "GeometricRobot":
        """URDF の運動学から作る(形は簡易形状。メッシュは読まない)。"""
        source = UrdfSource(source) if isinstance(source, str) else source
        return cls.from_model(stage, source.to_model(), prim_path, **kwargs)

    # ------------------------------------------------------------------ 名前と個数(Articulation と同じ名前)

    @property
    def name(self) -> str:
        return self._model.name

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def model(self) -> GeometricRobotModel:
        return self._model

    @property
    def layer(self) -> GeometricLayer:
        return self._layer

    @property
    def info(self) -> UsdRobotInfo | None:
        """既存ロボットから作ったときの USD 側の情報(読み取り時の警告など)。"""
        return self._info

    @property
    def num_dofs(self) -> int:
        return self._model.num_dofs

    @property
    def dof_names(self) -> list[str]:
        return self._model.dof_names

    @property
    def dof_paths(self) -> list[str | None]:
        return [j.prim_path for j in self._model.dof_joints]

    @property
    def dof_types(self) -> list[str]:
        return list(self._model.dof_types)

    @property
    def num_joints(self) -> int:
        return self._model.num_joints

    @property
    def joint_names(self) -> list[str]:
        return self._model.joint_names

    @property
    def joint_paths(self) -> list[str | None]:
        return [j.prim_path for j in self._model.joints]

    @property
    def joint_types(self) -> list[str]:
        return list(self._model.joint_types)

    @property
    def num_links(self) -> int:
        return self._model.num_links

    @property
    def link_names(self) -> list[str]:
        return self._model.link_names

    @property
    def link_paths(self) -> list[str | None]:
        return [self._link_path_map.get(n) for n in self._model.link_names]

    @property
    def link_path_map(self) -> dict[str, str]:
        """リンク名 -> prim パス(prim を持つリンクだけ)。"""
        return self._link_path_map

    def get_dof_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        return np.array([self._model.get_dof_index(n) for n in names], dtype=int)

    def get_joint_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        return np.array([self._model.get_joint_index(n) for n in names], dtype=int)

    def get_link_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        return np.array([self._model.get_link_index(n) for n in names], dtype=int)

    # ------------------------------------------------------------------ 関節値

    def set_dof_positions(self, positions, *, dof_indices=None) -> None:
        """関節値を変え、リンクの Transform を書く。mimic の従動軸は参照軸から決まる(渡した値は使われない)。"""
        idx = _as_indices(dof_indices, self.num_dofs)
        values = np.asarray(positions, dtype=float).reshape(-1)
        if values.shape[0] != idx.shape[0]:
            raise ValueError(f"positions の個数 {values.shape[0]} が DOF の個数 {idx.shape[0]} と合わない")
        self._q[idx] = self._apply_limits(values, idx)
        self._q = self._model.resolve_mimic(self._q)
        self._push()

    def get_dof_positions(self, *, dof_indices=None) -> np.ndarray:
        """いまの関節値(mimic を解決した後の値)。"""
        return self._q[_as_indices(dof_indices, self.num_dofs)].copy()

    def set_dof_position_targets(self, positions, *, dof_indices=None) -> None:
        """Drive が無いので set_dof_positions と同じ(Articulation 用のコードを移すための別名)。"""
        self.set_dof_positions(positions, dof_indices=dof_indices)

    def get_dof_position_targets(self, *, dof_indices=None) -> np.ndarray:
        return self.get_dof_positions(dof_indices=dof_indices)

    def get_dof_limits(self, *, dof_indices=None) -> tuple[np.ndarray, np.ndarray]:
        idx = _as_indices(dof_indices, self.num_dofs)
        return self._lower[idx].copy(), self._upper[idx].copy()

    def set_dof_limits(self, lower=None, upper=None, *, dof_indices=None) -> None:
        """このロボットの可動範囲を変える(モデルと USD は変えない)。"""
        idx = _as_indices(dof_indices, self.num_dofs)
        if lower is not None:
            self._lower[idx] = np.asarray(lower, dtype=float).reshape(-1)
        if upper is not None:
            self._upper[idx] = np.asarray(upper, dtype=float).reshape(-1)

    def _apply_limits(self, values: np.ndarray, idx: np.ndarray) -> np.ndarray:
        if self._limit_policy == "ignore":
            return values
        lower, upper = self._lower[idx], self._upper[idx]
        outside = (values < lower - 1e-12) | (values > upper + 1e-12)
        if outside.any() and self._limit_policy in ("warn", "error"):
            names = [self.dof_names[i] for i in idx[outside]]
            message = f"可動範囲の外の値: {dict(zip(names, values[outside].tolist()))}"
            if self._limit_policy == "error":
                raise ValueError(message)
            _log_warn(message + "(範囲内に切り詰めた)")
        return np.clip(values, lower, upper)

    # ------------------------------------------------------------------ 根と各リンクの姿勢

    def get_world_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """根リンクのワールド姿勢 (位置 (3,), 四元数 (4,) = (w, x, y, z))。"""
        return self._base[:3, 3].copy(), tf.matrix_to_quat(self._base[:3, :3])

    def set_world_poses(self, positions=None, orientations=None) -> None:
        """根リンクのワールド姿勢を変える(ロボット全体が動く)。省略した側は今のまま。"""
        if positions is not None:
            self._base[:3, 3] = np.asarray(positions, dtype=float).reshape(3)
        if orientations is not None:
            self._base[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        self._push()

    def get_local_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """根リンクの prim の親 prim から見た姿勢 (位置, 四元数)。親にスケールがあっても、長さはワールドと同じ単位で返す。"""
        local = tf.invert(self._root_parent_world()) @ self._base
        return local[:3, 3].copy(), tf.matrix_to_quat(local[:3, :3])

    def set_local_poses(self, translations=None, orientations=None) -> None:
        parent = self._root_parent_world()
        local = tf.invert(parent) @ self._base
        if translations is not None:
            local[:3, 3] = np.asarray(translations, dtype=float).reshape(3)
        if orientations is not None:
            local[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        self._base = parent @ local
        self._push()

    def _root_parent_world(self) -> np.ndarray:
        path = self._link_path_map.get(self._model.root_link or "")
        if path is None:
            return np.eye(4)
        parent = self._stage.GetPrimAtPath(path).GetParent()
        if parent.IsPseudoRoot():
            return np.eye(4)
        return split_scale(_np(UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(parent)))[0]

    def get_link_world_matrices(self) -> np.ndarray:
        """全リンクのワールド姿勢 (num_links, 4, 4)。列ベクトル形式、スケール抜き。"""
        return self._world.copy()

    def get_link_world_poses(self, *, link_indices=None) -> tuple[np.ndarray, np.ndarray]:
        """リンクのワールド姿勢 (位置 (L, 3), 四元数 (L, 4))。Articulation には無い追加のメソッド。"""
        world = self._world[_as_indices(link_indices, self.num_links)]
        return world[:, :3, 3].copy(), tf.matrix_to_quat(world[:, :3, :3])

    def get_link_local_poses(self, *, link_indices=None) -> tuple[np.ndarray, np.ndarray]:
        """親リンクから見た各リンクの姿勢 (位置 (L, 3), 四元数 (L, 4))。根リンクはワールドから見た姿勢。"""
        idx = _as_indices(link_indices, self.num_links)
        out = np.zeros((len(idx), 4, 4))
        for k, i in enumerate(idx):
            joint = self._model.parent_joint_of(self._model.link_names[i])
            parent = np.eye(4) if joint is None else self._world[self._model.get_link_index(joint.parent_link)]
            out[k] = tf.invert(parent) @ self._world[i]
        return out[:, :3, 3], tf.matrix_to_quat(out[:, :3, :3])

    def get_jacobian_matrices(self) -> np.ndarray:
        """全リンクのヤコビアン (num_links, 6, num_dofs)。ワールド座標系、行は [リンク原点の速度 3 行; 角速度 3 行]。

        mimic の従動軸の列は、その軸だけを動かしたときの値(畳んだものが欲しいときは model.jacobian(q, reduced=True))。
        """
        jac = self._model.jacobian(self._q)
        rot = self._base[:3, :3]
        out = np.empty_like(jac)
        out[:, :3, :] = np.einsum("ij,ljd->lid", rot, jac[:, :3, :])
        out[:, 3:, :] = np.einsum("ij,ljd->lid", rot, jac[:, 3:, :])
        return out

    # ------------------------------------------------------------------ 既定の状態

    def set_default_state(self, positions=None, orientations=None, dof_positions=None) -> None:
        """reset_to_default_state() で戻る状態を決める。省略した項目は今の既定値のまま。"""
        if positions is not None:
            self._default_base[:3, 3] = np.asarray(positions, dtype=float).reshape(3)
        if orientations is not None:
            self._default_base[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        if dof_positions is not None:
            self._default_q = self._model.resolve_mimic(np.asarray(dof_positions, dtype=float).reshape(self.num_dofs))

    def get_default_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(根の位置, 根の四元数, 関節値)。"""
        return self._default_base[:3, 3].copy(), tf.matrix_to_quat(self._default_base[:3, :3]), self._default_q.copy()

    def reset_to_default_state(self) -> None:
        self._base = self._default_base.copy()
        self._q = self._default_q.copy()
        self._push()

    # ------------------------------------------------------------------ IK

    def get_kinematic_chain(self, base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> KinematicChain:
        """base_link から tip_link までの直列部分。chain.dof_indices が、このロボットの DOF 番号。"""
        return self._model.get_kinematic_chain(base_link, tip_link, tip_offset)

    def create_ik_solver(
        self, name: IKSolverName | str, base_link: str, tip_link: str, config: IKSolverConfig | None = None, *, tip_offset: np.ndarray | None = None
    ) -> IKSolver:
        """登録名("jacobian" / "ikpy" / 自分で登録した名前)から IK ソルバーを作る。"""
        return create_ik_solver(name, self.get_kinematic_chain(base_link, tip_link, tip_offset), config)

    def world_to_link_frame(self, link: str, position, orientation=None) -> tuple[np.ndarray, np.ndarray | None]:
        """ワールド座標の目標を、いまの link 座標系で見た値に直す(IK の目標は鎖の base リンク座標系で渡すため)。"""
        inv = tf.invert(self._world[self._model.get_link_index(link)])
        pos = inv[:3, :3] @ np.asarray(position, dtype=float).reshape(3) + inv[:3, 3]
        if orientation is None:
            return pos, None
        return pos, tf.matrix_to_quat(inv[:3, :3] @ tf.quat_to_matrix(np.asarray(orientation, dtype=float).reshape(4)))

    def solve_ik(self, solver: IKSolver, target_position, target_orientation=None, *, apply: bool = True):
        """ワールド座標の目標に手先を合わせる。seed はいまの関節値。apply=True で成功したら関節値に反映する。IKResult を返す。"""
        chain = solver.chain
        pos, quat = self.world_to_link_frame(chain.base_link, target_position, target_orientation)
        result = solver.solve(pos, quat, seed=self._q[chain.dof_indices])
        if apply and result.success:
            self.set_dof_positions(result.dof_positions, dof_indices=chain.dof_indices)
        return result

    # ------------------------------------------------------------------ 干渉チェック

    @property
    def collision_filter(self) -> GeometricCollisionFilter:
        """無視する組。add_pair / add_pattern / add_rule で足し、remove_pair / remove_by_reason で外す。"""
        return self._filter

    @property
    def collider_paths(self) -> dict[str, list[str]]:
        """リンク名 -> コライダー形状の prim パス。初めて使うときに USD から探す(形状を足したら refresh_colliders())。"""
        if self._colliders is None:
            from .collision import find_colliders

            self._colliders = {name: find_colliders(self._stage, path, self._prim_paths) for name, path in self._link_path_map.items()}
        return self._colliders

    def refresh_colliders(self) -> None:
        self._colliders = None

    @property
    def link_set(self):
        """このロボットの全リンクの集合(干渉チェックの範囲)。"""
        from .collision import GeometricLinkSet

        return GeometricLinkSet(self, self._model.link_names, self.name)

    @property
    def collision_checker(self):
        if self._checker is None:
            from .collision import GeometricCollisionChecker

            self._checker = GeometricCollisionChecker(self._stage)
        return self._checker

    def _default_scope(self) -> str:
        """元の Articulation が自己干渉を無効にしていたら、既定では環境との干渉だけを見る。"""
        if self._info is not None and self._info.self_collision_enabled is False:
            return "environment"
        return "all"

    def check_collisions(self, scope=None, *, extra_filter=None, ignore_prim_paths: Sequence[str] = ()) -> list:
        """いまの姿勢で重なっている組(GeometricCollisionPair の列)。

        scope: "within" = このロボットのリンク同士 / "environment" = ロボット以外のコライダーと / "all" = 両方。
               省略時は、元の Articulation が enabledSelfCollisions = false なら "environment"、それ以外は "all"。
        """
        return _check(self, self.link_set, scope, extra_filter, ignore_prim_paths)

    def is_in_collision(self, scope=None, **kwargs) -> bool:
        return bool(self.check_collisions(scope, **kwargs))

    def ignore_touching_pairs(self, reason: str = "touching_at_setup") -> list[tuple[str, str]]:
        """いまの姿勢でロボット内で重なっている組を、すべて「無視する組」に足す。足した組を返す。

        簡易形状で組んだロボットは、短いリンクをはさんだ 2 リンクが関節の所で常に重なることがある。
        干渉していない姿勢で 1 回呼んでおくと、その組を以後のチェックから外せる。外すときは
        collision_filter.remove_by_reason(reason)。
        """
        added = []
        for pair in self.collision_checker.check_within(self.link_set):
            self._filter.add_pair(pair.link_a, pair.link_b, reason=reason)
            added.append((pair.link_a, pair.link_b))
        return added

    # ------------------------------------------------------------------ 構成要素(結合したロボット)

    @property
    def component_names(self) -> list[str]:
        return list(self._model.components)

    def get_component(self, name: str) -> "GeometricRobotComponent":
        """結合したロボットの構成要素(同じ状態を共有する部分ロボット)。"""
        if name not in self._model.components:
            raise KeyError(f"構成要素 {name!r} は無い。ある名前: {self.component_names}")
        if name not in self._components:
            self._components[name] = GeometricRobotComponent(self, name)
        return self._components[name]

    # ------------------------------------------------------------------ 書き込み

    @property
    def write_target(self) -> WriteTarget:
        return self._write_target

    @write_target.setter
    def write_target(self, value: WriteTarget) -> None:
        if value not in get_args(WriteTarget):
            raise ValueError(f"write_target は {get_args(WriteTarget)} のどれか: {value!r}")
        self._write_target = value
        self._push()

    def _fabric(self) -> GeometricFabricWriter | None:
        if self._fabric_writer is None and not self._fabric_failed:
            try:
                self._fabric_writer = GeometricFabricWriter(self._stage, self._prim_paths)
            except Exception as exc:  # Fabric が使えない構成(usdrt が無い、ステージが Kit に開かれていない、など)
                self._fabric_failed = True
                _log_warn(f"Fabric に書けないので USD に書く: {exc}")
        return self._fabric_writer

    def _prim_world(self) -> np.ndarray:
        return self._world[self._prim_link_indices]

    def _push(self) -> None:
        """いまの関節値と根の姿勢から全リンクの姿勢を計算し、幾何モードなら書き込み先へ書く。"""
        self._world = self._base @ self._model.forward_kinematics(self._q, resolve_mimic=False)
        if not self._layer.is_applied() or self._closed or not self._prim_paths:
            return
        world = self._prim_world()
        fabric = self._fabric() if self._write_target in ("fabric", "both") else None
        if self._write_target in ("usd", "both") or fabric is None:
            self._usd_writer.write(self._solver, world)
            self._usd_dirty = False
            if fabric is not None:
                self._repush_frames = 2
        else:
            self._usd_dirty = True
        if fabric is not None:
            fabric.write(self._solver, world)

    def flush_to_usd(self) -> bool:
        """Fabric にだけ書いてある姿勢を USD(Sublayer)にも書く。書いたら True。

        Stop 中の PhysX は USD に書かれた姿勢しか見ないので、干渉チェックの前に自動で呼ばれる。
        USD を読む別の処理に今の姿勢を見せたいときは、自分で呼ぶ。
        """
        if self._flush_delegate is not None:
            return self._flush_delegate()
        if not self._usd_dirty or not self._layer.is_applied() or self._closed:
            return False
        self._usd_writer.write(self._solver, self._prim_world())
        self._usd_dirty = False
        # USD に書くと、次の update で USD → Fabric の同期が走る。その後に Fabric へ書いた新しい姿勢が
        # 古い USD の値で上書きされないよう、数フレームの間は update のたびに Fabric へ書き直す
        self._repush_frames = 2
        return True

    def refresh(self) -> None:
        """ロボットの外側の prim(根より上の Xform など)を動かした後に呼ぶ。親の姿勢を読み直して書き直す。"""
        self._solver.refresh()
        self._push()

    def _on_layer_changed(self, applied: bool) -> None:
        if self._closed:
            return
        if applied:
            self._solver.refresh()
            self._world = self._base @ self._model.forward_kinematics(self._q, resolve_mimic=False)
            if self._prim_paths:
                self._usd_writer.write(self._solver, self._prim_world())  # 付けた時点の姿勢を USD と PhysX に見せる
            self._usd_dirty = False
            self._repush_frames = 3
            self._subscribe_update()
            if self._write_target in ("fabric", "both") and self._prim_paths:
                fabric = self._fabric()
                if fabric is not None:
                    fabric.write(self._solver, self._prim_world())
        else:
            self._usd_writer.close()
            self._update_subscription = None
            self._usd_dirty = False

    def _subscribe_update(self) -> None:
        if self._update_subscription is not None:
            return
        try:
            import omni.kit.app

            self._update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update, name="myext.robot.geometric.repush"
            )
        except Exception:
            self._update_subscription = None

    def _on_update(self, _event) -> None:
        if self._repush_frames <= 0 or self._closed or not self._layer.is_applied():
            return
        self._repush_frames -= 1
        if self._write_target in ("fabric", "both") and self._prim_paths:
            fabric = self._fabric()
            if fabric is not None:
                fabric.write(self._solver, self._prim_world())

    def close(self, *, remove_layer: bool = True) -> None:
        """後片付け。remove_layer=True なら Sublayer を外して元の構成に戻す(Sublayer を共有しているときは False を渡す)。

        以後このオブジェクトは使わない。
        """
        if self._closed:
            return
        if remove_layer:
            self._layer.remove()
        self._layer.remove_listener(self._on_layer_changed)
        self._update_subscription = None
        self._closed = True
        if self._fabric_writer is not None:
            self._fabric_writer.close()


def _check(robot: GeometricRobot, link_set, scope, extra_filter, ignore_prim_paths) -> list:
    from .collision import CollisionScope

    scope = robot._default_scope() if scope is None else scope
    if scope not in get_args(CollisionScope):
        raise ValueError(f"scope は {get_args(CollisionScope)} のどれか: {scope!r}")
    checker = robot.collision_checker
    out = []
    if scope in ("within", "all"):
        out += checker.check_within(link_set, extra_filter=extra_filter)
    if scope in ("environment", "all"):
        out += checker.check_environment(link_set, ignore_prim_paths=ignore_prim_paths)
    return out


class GeometricRobotComponent:
    """結合したロボットの構成要素。関節値などの状態は元の GeometricRobot と共有する。

    DOF は「子リンクがこの構成要素に属する関節」。干渉チェックの "within" は、この構成要素のリンク同士だけを見る
    (結合の前後で意味が変わらない)。
    """

    def __init__(self, robot: GeometricRobot, name: str) -> None:
        self._robot = robot
        self.name = name
        model = robot.model
        self.link_names = list(model.components[name])
        members = set(self.link_names)
        self.dof_indices = np.array([d for d, j in enumerate(model.dof_joints) if j.child_link in members], dtype=int)

    @property
    def robot(self) -> GeometricRobot:
        return self._robot

    @property
    def num_dofs(self) -> int:
        return len(self.dof_indices)

    @property
    def dof_names(self) -> list[str]:
        names = self._robot.dof_names
        return [names[d] for d in self.dof_indices]

    @property
    def link_paths(self) -> list[str | None]:
        return [self._robot.link_path_map.get(n) for n in self.link_names]

    def _map(self, dof_indices) -> np.ndarray:
        return self.dof_indices[_as_indices(dof_indices, self.num_dofs)]

    def set_dof_positions(self, positions, *, dof_indices=None) -> None:
        self._robot.set_dof_positions(positions, dof_indices=self._map(dof_indices))

    def get_dof_positions(self, *, dof_indices=None) -> np.ndarray:
        return self._robot.get_dof_positions(dof_indices=self._map(dof_indices))

    def get_dof_limits(self, *, dof_indices=None) -> tuple[np.ndarray, np.ndarray]:
        return self._robot.get_dof_limits(dof_indices=self._map(dof_indices))

    def get_link_world_poses(self) -> tuple[np.ndarray, np.ndarray]:
        return self._robot.get_link_world_poses(link_indices=self._robot.get_link_indices(self.link_names))

    @property
    def link_set(self):
        from .collision import GeometricLinkSet

        return GeometricLinkSet(self._robot, self.link_names, self.name)

    def check_collisions(self, scope=None, *, extra_filter=None, ignore_prim_paths: Sequence[str] = ()) -> list:
        """"within" はこの構成要素のリンク同士、"environment" はロボット全体以外のコライダーとの間。"""
        return _check(self._robot, self.link_set, scope, extra_filter, ignore_prim_paths)

    def is_in_collision(self, scope=None, **kwargs) -> bool:
        return bool(self.check_collisions(scope, **kwargs))
