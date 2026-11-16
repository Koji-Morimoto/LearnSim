"""リンクの姿勢をステージに書く部分。

    LocalPoseSolver          ワールド姿勢 → 各 prim のローカル姿勢(prim の親子関係がリンクの親子関係と違っていてもよい)
    GeometricUsdWriter       Sublayer の属性にまとめて書く(Sdf.ChangeBlock)。PhysX のコライダーは Stop 中でも直後に追従する
    GeometricFabricWriter    Fabric にまとめて書く。描画には出るが USD には残らない。Stop 中は PhysX に届かない

書き込み時間の目安(2026-11-10 の実測): USD は 1 リンク約 12 µs、Fabric の一括は個数によらず約 0.13 ms。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from .layer import GeometricLayer
from .model import transforms as tf
from .usd_reader import _np, split_scale

__all__ = ["WriteTarget", "LocalPoseSolver", "GeometricTransformWriter", "GeometricUsdWriter", "GeometricFabricWriter"]

WriteTarget = Literal["fabric", "usd", "both"]
FABRIC_BATCH_MIN_PRIMS = 24  # これ以上の個数は Warp 配列の一括書き込み、未満は 1 個ずつ(少数ではそのほうが速い)


class LocalPoseSolver:
    """K 個の prim について、ワールド姿勢(スケール抜き)から、親 prim に対するローカル姿勢を求める。

    prim の USD 上の祖先に、同じ組の別の prim(= 別のリンク)がある場合は、その prim の新しい姿勢を親として使う。
    それ以外の祖先は動かないものとして、作ったときの姿勢を使う(動かしたら refresh() を呼ぶ)。
    """

    def __init__(self, stage: Usd.Stage, prim_paths: list[str]) -> None:
        self._stage = stage
        self._paths = list(prim_paths)
        self.refresh()

    @property
    def prim_paths(self) -> list[str]:
        return list(self._paths)

    def refresh(self) -> None:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        index = {p: i for i, p in enumerate(self._paths)}
        k = len(self._paths)
        self._ancestor = np.full(k, -1, dtype=int)  # 祖先にあるリンク prim の番号(無ければ -1)
        self._rel = np.tile(np.eye(4), (k, 1, 1))  # 祖先リンク(スケール抜き)から見た親 prim(スケール抜き)。祖先が無ければ親 prim のワールド
        self._parent_scale = np.ones((k, 3))
        self._local_scale = np.ones((k, 3))
        for i, path in enumerate(self._paths):
            prim = self._stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                raise ValueError(f"prim が無い: {path}")
            parent = prim.GetParent()
            parent_world = _np(cache.GetLocalToWorldTransform(parent)) if not parent.IsPseudoRoot() else np.eye(4)
            parent_rigid, parent_scale = split_scale(parent_world)
            world_scale = split_scale(_np(cache.GetLocalToWorldTransform(prim)))[1]
            self._parent_scale[i] = parent_scale
            self._local_scale[i] = world_scale / parent_scale
            anc = parent
            while anc and not anc.IsPseudoRoot() and str(anc.GetPath()) not in index:
                anc = anc.GetParent()
            if anc and not anc.IsPseudoRoot():
                self._ancestor[i] = index[str(anc.GetPath())]
                anc_rigid = split_scale(_np(cache.GetLocalToWorldTransform(anc)))[0]
                self._rel[i] = tf.invert(anc_rigid) @ parent_rigid
            else:
                self._rel[i] = parent_rigid
        self._has_ancestor = self._ancestor >= 0

    def solve(self, world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """world (K, 4, 4) スケール抜きのワールド姿勢 → (ローカルの位置 (K, 3), ローカルの回転行列 (K, 3, 3))。"""
        parent = self._rel.copy()
        if self._has_ancestor.any():
            idx = np.nonzero(self._has_ancestor)[0]
            parent[idx] = world[self._ancestor[idx]] @ self._rel[idx]
        local = tf.invert(parent) @ world
        return local[:, :3, 3] / self._parent_scale, local[:, :3, :3]

    def local_matrices(self, world: np.ndarray) -> np.ndarray:
        """USD に書くローカル行列 (K, 4, 4)。Gf と同じ行ベクトル形式で、スケールを含む。"""
        pos, rot = self.solve(world)
        out = np.zeros((len(self._paths), 4, 4))
        out[:, :3, :3] = np.swapaxes(rot * self._local_scale[:, None, :], -1, -2)
        out[:, 3, :3] = pos
        out[:, 3, 3] = 1.0
        return out


class GeometricTransformWriter(ABC):
    """リンクの姿勢の書き込み先。prim の並びは作ったときの prim_paths の順。"""

    @abstractmethod
    def write(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        """world (K, 4, 4) スケール抜きのワールド姿勢を書く。"""

    def close(self) -> None:
        """持っている資源を手放す。"""


class GeometricUsdWriter(GeometricTransformWriter):
    def __init__(self, layer: GeometricLayer, prim_paths: list[str]) -> None:
        self._layer = layer
        self._paths = list(prim_paths)
        self._specs: list[Sdf.AttributeSpec] | None = None
        self._specs_layer: Sdf.Layer | None = None

    def write(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        layer = self._layer.sdf_layer
        if layer is None:
            raise RuntimeError("Sublayer が付いていないので USD に書けない")
        if self._specs is None or self._specs_layer is not layer:  # 付け直されたら属性を取り直す
            self._specs = self._layer.xform_specs(self._paths)
            self._specs_layer = layer
        flat = solver.local_matrices(world).reshape(len(self._paths), 16)
        with Sdf.ChangeBlock():
            for spec, row in zip(self._specs, flat):
                spec.default = Gf.Matrix4d(*row)

    def close(self) -> None:
        self._specs = None
        self._specs_layer = None


class GeometricFabricWriter(GeometricTransformWriter):
    """Fabric のローカル行列に書く。個数が多いときは Warp 配列で一括、少ないときは 1 個ずつ。"""

    def __init__(self, stage: Usd.Stage, prim_paths: list[str], *, batch: bool | None = None) -> None:
        import usdrt
        from pxr import UsdUtils

        self._paths = list(prim_paths)
        self._batch = (len(prim_paths) >= FABRIC_BATCH_MIN_PRIMS) if batch is None else batch
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = cache.Insert(stage).ToLongInt()
        self._rt_stage = usdrt.Usd.Stage.Attach(stage_id)
        self._hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(self._rt_stage.GetFabricId(), self._rt_stage.GetStageIdAsStageId())
        self._rt_paths = [usdrt.Sdf.Path(p) for p in prim_paths]
        self._usdrt = usdrt
        self._view = None

    def write(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        if self._batch:
            self._write_batch(solver, world)
        else:
            flat = solver.local_matrices(world).reshape(len(self._paths), 16)
            matrix = self._usdrt.Gf.Matrix4d
            for path, row in zip(self._rt_paths, flat):
                self._hierarchy.set_local_xform(path, matrix(*row))

    def _write_batch(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        import isaacsim.core.experimental.utils.backend as backend_utils
        import warp as wp

        if self._view is None:
            from isaacsim.core.experimental.prims import XformPrim

            self._view = XformPrim(self._paths)
        pos, rot = solver.solve(world)
        quat = tf.matrix_to_quat(rot)
        with backend_utils.use_backend("fabric"):
            self._view.set_local_poses(
                translations=wp.array(pos.astype(np.float32), dtype=wp.float32, device="cpu"),
                orientations=wp.array(quat.astype(np.float32), dtype=wp.float32, device="cpu"),
            )

    def read_world(self) -> np.ndarray:
        """Fabric 上のワールド行列 (K, 4, 4)(列ベクトル形式、スケールを含む)。確認用。"""
        self._hierarchy.update_world_xforms()
        return np.array([np.array(self._hierarchy.get_world_xform(p), dtype=float).reshape(4, 4).T for p in self._rt_paths])

    def close(self) -> None:
        self._view = None
