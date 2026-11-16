"""GeometricRobotView: 同じ構造のロボット複数台をまとめて扱う。

メソッドの名前・引数(indices=, dof_indices=)・配列の形 (台数, num_dofs)・戻り値(wp.array)を
isaacsim.core.experimental.prims.Articulation に合わせてあるので、Articulation 用に書いたコードを移しやすい。

順運動学は全台をまとめて 1 回で計算し、Transform の書き込みも全台分を 1 回で行う(台数ぶんの Python ループは回さない)。
"""

from __future__ import annotations

from typing import Sequence, get_args

import numpy as np
import warp as wp
from pxr import Usd

from .layer import GeometricLayer, GeometricLayerOptions, _setting
from .model import transforms as tf
from .robot import GeometricRobot, _as_indices
from .usd_reader import UsdArticulationSource
from .writer import GeometricFabricWriter, GeometricUsdWriter, LocalPoseSolver, WriteTarget

__all__ = ["GeometricRobotView"]


def _np(value) -> np.ndarray:
    return value.numpy() if isinstance(value, wp.array) else np.asarray(value, dtype=float)


class GeometricRobotView:
    def __init__(self, robots: Sequence[GeometricRobot], *, write_target: WriteTarget | None = None) -> None:
        """robots は同じ構造(リンク・関節の並びと寸法が同じ)で、同じ Sublayer を共有していること。from_articulations を使うのが簡単。"""
        if not robots:
            raise ValueError("ロボットが 1 台も無い")
        first = robots[0]
        for robot in robots[1:]:
            if not first.model.is_kinematically_equal(robot.model):
                raise ValueError(f"構造が違うロボットは 1 つの View にまとめられない: {first.name} と {robot.name}")
            if robot.layer is not first.layer:
                raise ValueError("View のロボットは同じ Sublayer を共有すること(from_articulations か、layer= を渡して作る)")
        target = write_target if write_target is not None else _setting("write_target", "fabric")
        if target not in get_args(WriteTarget):
            raise ValueError(f"write_target は {get_args(WriteTarget)} のどれか: {target!r}")
        self._robots = list(robots)
        self._model = first.model
        self._layer = first.layer
        self._write_target: WriteTarget = target
        n, d = len(robots), first.num_dofs
        # 各ロボットの関節値を 1 枚の配列の行にする(ロボット側から読み書きしても同じ値になる)
        self._q = np.stack([r._q for r in robots])
        self._base = np.stack([r._base for r in robots])
        self._lower = np.stack([r._lower for r in robots])
        self._upper = np.stack([r._upper for r in robots])
        self._prim_link_indices = first._prim_link_indices
        self._prim_paths = [p for r in robots for p in r._prim_paths]
        self._solver = LocalPoseSolver(first.stage, self._prim_paths)
        self._usd_writer = GeometricUsdWriter(self._layer, self._prim_paths)
        self._fabric_writer: GeometricFabricWriter | None = None
        self._usd_dirty = False
        self._world = self._base[:, None] @ self._model.forward_kinematics(self._q, resolve_mimic=False)
        self._shape = (n, d)
        self._repush_frames = 3  # Sublayer を付けた直後かもしれないので、最初の数フレームは Fabric へ書き直す
        self._update_subscription = None
        self._layer.add_listener(self._on_layer_changed)
        for robot in self._robots:
            robot._flush_delegate = self.flush_to_usd
        try:
            import omni.kit.app

            self._update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update, name="myext.robot.geometric.view")
        except Exception:
            pass
        self._push()

    def _on_layer_changed(self, applied: bool) -> None:
        if applied:
            self._solver.refresh()
            self._repush_frames = 3
            self._push()

    def _on_update(self, _event) -> None:
        # USD に書いた後の数フレームは、USD → Fabric の同期で古い姿勢に戻されないよう Fabric へ書き直す
        if self._repush_frames <= 0 or not self._layer.is_applied() or self._write_target == "usd":
            return
        self._repush_frames -= 1
        fabric = self._fabric()
        if fabric is not None:
            fabric.write(self._solver, self._world[:, self._prim_link_indices].reshape(-1, 4, 4))

    @classmethod
    def from_articulations(
        cls,
        stage: Usd.Stage,
        sources: Sequence[UsdArticulationSource | str],
        *,
        layer_options: GeometricLayerOptions | None = None,
        write_target: WriteTarget | None = None,
        apply_layer: bool = True,
    ) -> "GeometricRobotView":
        """ステージ上の同じ構造のロボット複数台から作る。Sublayer は全台で 1 つ。"""
        layer = GeometricLayer(stage, layer_options)
        robots = [GeometricRobot.from_articulation(stage, s, layer=layer, write_target="usd", apply_layer=False) for s in sources]
        if apply_layer:
            layer.apply()
        return cls(robots, write_target=write_target)

    # ------------------------------------------------------------------ 名前と個数

    def __len__(self) -> int:
        return len(self._robots)

    @property
    def robots(self) -> list[GeometricRobot]:
        return list(self._robots)

    @property
    def layer(self) -> GeometricLayer:
        return self._layer

    @property
    def num_dofs(self) -> int:
        return self._model.num_dofs

    @property
    def dof_names(self) -> list[str]:
        return self._model.dof_names

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
    def num_links(self) -> int:
        return self._model.num_links

    @property
    def link_names(self) -> list[str]:
        return self._model.link_names

    @property
    def dof_paths(self) -> list[list[str | None]]:
        return [r.dof_paths for r in self._robots]

    @property
    def joint_paths(self) -> list[list[str | None]]:
        return [r.joint_paths for r in self._robots]

    @property
    def link_paths(self) -> list[list[str | None]]:
        return [r.link_paths for r in self._robots]

    def get_dof_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_dof_indices(names), dtype=wp.int32, device="cpu")

    def get_joint_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_joint_indices(names), dtype=wp.int32, device="cpu")

    def get_link_indices(self, names: str | Sequence[str]) -> wp.array:
        return wp.array(self._robots[0].get_link_indices(names), dtype=wp.int32, device="cpu")

    # ------------------------------------------------------------------ 関節値

    def _rows_cols(self, indices, dof_indices) -> tuple[np.ndarray, np.ndarray]:
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        cols = _as_indices(None if dof_indices is None else _np(dof_indices).astype(int), self.num_dofs)
        return rows, cols

    def set_dof_positions(self, positions, *, indices=None, dof_indices=None) -> None:
        """positions は (N, D)(1 行だけ渡すと全台に同じ値)。範囲の外の値は切り詰める。mimic の従動軸は参照軸から決まる。"""
        rows, cols = self._rows_cols(indices, dof_indices)
        values = np.broadcast_to(_np(positions).reshape(-1, len(cols)), (len(rows), len(cols)))
        self._q[np.ix_(rows, cols)] = np.clip(values, self._lower[np.ix_(rows, cols)], self._upper[np.ix_(rows, cols)])
        self._q[:] = self._model.resolve_mimic(self._q)
        self._push()

    def get_dof_positions(self, *, indices=None, dof_indices=None) -> wp.array:
        rows, cols = self._rows_cols(indices, dof_indices)
        return wp.array(self._q[np.ix_(rows, cols)].astype(np.float32), dtype=wp.float32, device="cpu")

    def set_dof_position_targets(self, positions, *, indices=None, dof_indices=None) -> None:
        self.set_dof_positions(positions, indices=indices, dof_indices=dof_indices)

    def get_dof_position_targets(self, *, indices=None, dof_indices=None) -> wp.array:
        return self.get_dof_positions(indices=indices, dof_indices=dof_indices)

    def get_dof_limits(self, *, indices=None, dof_indices=None) -> tuple[wp.array, wp.array]:
        rows, cols = self._rows_cols(indices, dof_indices)
        lower = wp.array(self._lower[np.ix_(rows, cols)].astype(np.float32), dtype=wp.float32, device="cpu")
        upper = wp.array(self._upper[np.ix_(rows, cols)].astype(np.float32), dtype=wp.float32, device="cpu")
        return lower, upper

    # ------------------------------------------------------------------ 姿勢

    def get_world_poses(self, *, indices=None) -> tuple[wp.array, wp.array]:
        """各ロボットの根リンクのワールド姿勢 (位置 (N, 3), 四元数 (N, 4) = (w, x, y, z))。"""
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        base = self._base[rows]
        return (
            wp.array(base[:, :3, 3].astype(np.float32), dtype=wp.float32, device="cpu"),
            wp.array(tf.matrix_to_quat(base[:, :3, :3]).astype(np.float32), dtype=wp.float32, device="cpu"),
        )

    def set_world_poses(self, positions=None, orientations=None, *, indices=None) -> None:
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        if positions is not None:
            self._base[rows, :3, 3] = np.broadcast_to(_np(positions).reshape(-1, 3), (len(rows), 3))
        if orientations is not None:
            self._base[rows, :3, :3] = tf.quat_to_matrix(np.broadcast_to(_np(orientations).reshape(-1, 4), (len(rows), 4)))
        self._push()

    def get_link_world_poses(self, *, indices=None, link_indices=None) -> tuple[wp.array, wp.array]:
        """リンクのワールド姿勢 (位置 (N, L, 3), 四元数 (N, L, 4))。Articulation には無い追加のメソッド。"""
        rows = _as_indices(None if indices is None else _np(indices).astype(int), len(self._robots))
        cols = _as_indices(None if link_indices is None else _np(link_indices).astype(int), self.num_links)
        world = self._world[np.ix_(rows, cols)]
        return (
            wp.array(world[..., :3, 3].astype(np.float32), dtype=wp.float32, device="cpu"),
            wp.array(tf.matrix_to_quat(world[..., :3, :3]).astype(np.float32), dtype=wp.float32, device="cpu"),
        )

    def get_link_world_matrices(self) -> np.ndarray:
        """全台・全リンクのワールド姿勢 (N, num_links, 4, 4)。倍精度の numpy。"""
        return self._world.copy()

    # ------------------------------------------------------------------ 書き込み

    def _push(self) -> None:
        self._world = self._base[:, None] @ self._model.forward_kinematics(self._q, resolve_mimic=False)
        for i, robot in enumerate(self._robots):  # 各ロボットが持つ状態をそろえる(代入だけ。計算も書き込みもしない)
            robot._q, robot._base, robot._world = self._q[i], self._base[i], self._world[i]
            robot._usd_dirty = False
        if not self._layer.is_applied() or not self._prim_paths:
            return
        world = self._world[:, self._prim_link_indices].reshape(-1, 4, 4)
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

    def _fabric(self) -> GeometricFabricWriter | None:
        if self._fabric_writer is None:
            try:
                self._fabric_writer = GeometricFabricWriter(self._robots[0].stage, self._prim_paths)
            except Exception:
                self._write_target = "usd"
        return self._fabric_writer

    def flush_to_usd(self) -> bool:
        """Fabric にだけ書いてある姿勢を USD(Sublayer)にも書く。干渉チェックの前に呼ぶ。"""
        if not self._usd_dirty or not self._layer.is_applied():
            return False
        self._usd_writer.write(self._solver, self._world[:, self._prim_link_indices].reshape(-1, 4, 4))
        self._usd_dirty = False
        self._repush_frames = 2
        return True

    def close(self, *, remove_layer: bool = True) -> None:
        """後片付け。remove_layer=True なら共有の Sublayer を外して元の構成に戻す。"""
        self._update_subscription = None
        self._layer.remove_listener(self._on_layer_changed)
        for robot in self._robots:
            robot.close(remove_layer=False)
        if remove_layer:
            self._layer.remove()
