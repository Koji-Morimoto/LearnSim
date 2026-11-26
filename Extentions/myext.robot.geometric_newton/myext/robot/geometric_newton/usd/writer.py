"""リンクの姿勢をステージ (USD / Fabric / GPU Fabric) へ書き込むライター群。

構成クラスと役割:
    - `LocalPoseSolver`:
      スケール抜きのワールド姿勢 `(K, 4, 4)` から、各リンク prim の親 prim に対するローカル姿勢を求める。
      USD 階層上の prim の親子関係が、ロボットの運動学上のリンク親子関係と一致していなくても正しく動作する。
    - `GeometricUsdWriter`:
      `GeometricLayer` の無名 Sublayer 上の `xformOp:transform` 属性へ `Sdf.ChangeBlock` で一括書き込みする。
      PhysX バックエンド (`collision_backend="physx"`) が参照する USD 姿勢を最新化するために必要。
    - `GeometricFabricWriter`:
      Fabric (`IFabricHierarchy` / `XformPrim`) のローカル行列へ書き込む。描画へ高速反映されるが USD には残らない。
      Newton バックエンド (`collision_backend="newton"`) と組めば USD への flush なしで 0 フレーム遅延の干渉チェックが可能。
    - `GeometricGpuFabricWriter`:
      Newton の `state.body_q` (Warp GPU 配列) を CPU へ戻さず、Warp カーネル 1 回で直接 Fabric の
      `omni:fabric:worldMatrix` へ書き込む GPU 高速パス。

座標・行列・四元数の規約 (本モジュール全体で統一):
    - `GeometricRobot` / `LocalPoseSolver` 内部の 4x4 行列:
      **列ベクトル形式** (`v' = M @ v`)。並進は第 3 列 `M[:3, 3]`、回転は左上 `M[:3, :3]`。
    - USD `Gf.Matrix4d` / `usdrt.Gf.Matrix4d` / Fabric `omni:fabric:worldMatrix` / `omni:fabric:localMatrix`:
      **行ベクトル形式** (`v' = v @ M`)。並進は第 3 行 `M[3, :3]`、回転は左上 `M[:3, :3]` の転置。
      そのため USD / Fabric との入出力境界では必ず転置 (`swapaxes(-1, -2)` / `.T` / `wp.transpose`) を行う。
    - 四元数:
      `GeometricRobot` (`model.transforms`) は `(w, x, y, z)`、Warp / Newton (`wp.transform`) は `(x, y, z, w)`。

書き込み時間の目安: USD は 1 リンク約 12 µs、Fabric 一括は約 0.13 ms、GPU Fabric 直接書き込みは約 0.09 ms。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal, Sequence

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from .layer import GeometricLayer
from ..model import transforms as tf
from .reader import _np, split_scale_batch

__all__ = [
    "WriteTarget",
    "LocalPoseSolver",
    "GeometricTransformWriter",
    "GeometricUsdWriter",
    "GeometricFabricWriter",
    "GeometricGpuFabricWriter",
]

# ==============================================================================
# 定数とローカル姿勢ソルバー (LocalPoseSolver)
# ==============================================================================

WriteTarget = Literal["fabric", "usd", "both"]

# `GeometricFabricWriter` が 1 prim ずつの `set_local_xform` を使うか、
# `XformPrim.set_local_poses` (Warp 配列の一括書き込み) を使うかの境界値。
# 24 prim 未満では `XformPrim` 経由のオーバーヘッドのほうが大きく、個別書き込みのほうが速い。
FABRIC_BATCH_MIN_PRIMS = 24


class LocalPoseSolver:
    """K 個のリンク prim について、スケール抜きのワールド姿勢から親 prim に対するローカル姿勢を求める。

    なぜ単純な「親リンクの逆行列 @ 自リンク」では足りないか:
        USD アセットでは、リンク prim の USD 上の親が運動学上の親リンクとは限らない
        (例: 全リンクが `/World/Robot` 直下にフラット配置されている、途中に中間 `Xform` が挟まる、
        あるいは別リンク prim の配下に入れ子になっている)。
        そこで初期化 (`refresh`) 時に USD 階層を調べ、
        - 祖先に別の管理対象リンク prim がある場合は、その祖先リンクの「新しいワールド姿勢」に
          固定相対変換 `self._rel` を合成して親 prim の最新ワールド姿勢を求める。
        - 祖先に管理対象リンク prim がない場合は、親 prim は固定とみなして `refresh()` 時の姿勢を使う
          (ロボット外の親 `Xform` を動かした場合は `robot.refresh()` を呼ぶ)。
    """

    def __init__(self, stage: Usd.Stage, prim_paths: list[str]) -> None:
        self._stage = stage
        self._paths = list(prim_paths)
        self.refresh()

    @property
    def prim_paths(self) -> list[str]:
        return list(self._paths)

    @property
    def world_scales(self) -> np.ndarray:
        """各 prim のワールドスケール `(K, 3)`。`GeometricGpuFabricWriter` がワールド行列再構成に使う。"""
        return self._world_scale.copy()

    def refresh(self) -> None:
        """USD から親 prim と祖先リンクの姿勢・スケールを読み直す。

        USD API 呼び出しは `UsdGeom.XformCache` で prim ごとに 1 回だけ済ませ、
        スケール分離と相対変換計算は NumPy バッチで全 prim まとめて実行する。
        """
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        index = {p: i for i, p in enumerate(self._paths)}
        k = len(self._paths)
        eye = Gf.Matrix4d(1.0)
        parent_m, prim_m, anc_m = [], [], []
        ancestor = np.full(k, -1, dtype=int)  # 祖先にある管理対象リンク prim の番号 (無ければ -1)
        for i, path in enumerate(self._paths):
            prim = self._stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                raise ValueError(f"prim が無い: {path}")
            parent = prim.GetParent()
            parent_m.append(eye if parent.IsPseudoRoot() else cache.GetLocalToWorldTransform(parent))
            prim_m.append(cache.GetLocalToWorldTransform(prim))
            # 祖先にある管理対象リンク prim は USD prim をたどらずパス文字列の親切り詰めで高速探索する
            anc_path = path.rsplit("/", 1)[0]
            while anc_path and anc_path not in index:
                anc_path = anc_path.rsplit("/", 1)[0] if "/" in anc_path[1:] else ""
            if anc_path:
                ancestor[i] = index[anc_path]
                anc_m.append(cache.GetLocalToWorldTransform(self._stage.GetPrimAtPath(anc_path)))
            else:
                anc_m.append(eye)
        self._ancestor = ancestor
        if k == 0:
            self._rel = np.zeros((0, 4, 4))
            self._parent_scale = self._local_scale = self._world_scale = np.ones((0, 3))
        else:
            # `Gf.Matrix4d` は行ベクトル形式 (第 3 行が並進) なので、転置して列ベクトル形式へ揃える
            to_np = lambda ms: np.swapaxes(np.array(ms, dtype=float), -1, -2)  # noqa: E731
            parent_rigid, parent_scale = split_scale_batch(to_np(parent_m))
            world_scale = split_scale_batch(to_np(prim_m))[1]
            anc_rigid = split_scale_batch(to_np(anc_m))[0]
            self._parent_scale = parent_scale
            self._world_scale = world_scale
            # 親 prim のスケールを打ち消して元のワールドスケールを保つためのローカルスケール
            self._local_scale = world_scale / parent_scale
            # 祖先リンク (スケール抜き) から見た親 prim (スケール抜き) の固定相対変換。
            # 祖先リンクが無い場合は `anc_rigid` が単位行列なので、親 prim のワールド姿勢そのものになる。
            self._rel = tf.invert(anc_rigid) @ parent_rigid
        self._has_ancestor = self._ancestor >= 0

    def solve(self, world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """スケール抜きのワールド姿勢 `world (K, 4, 4)` (列ベクトル形式) をローカル姿勢へ変換する。

        戻り値:
            `(ローカル位置 (K, 3), ローカル回転行列 (K, 3, 3))`。
            親 prim に非単位スケール (`self._parent_scale`) がある場合、USD のローカル並進は親スケールで
            拡大されてワールドへ出るため、あらかじめ `self._parent_scale` で割って補正する。
        """
        parent = self._rel.copy()
        if self._has_ancestor.any():
            idx = np.nonzero(self._has_ancestor)[0]
            parent[idx] = world[self._ancestor[idx]] @ self._rel[idx]
        local = tf.invert(parent) @ world
        return local[:, :3, 3] / self._parent_scale, local[:, :3, :3]

    def local_matrices(self, world: np.ndarray) -> np.ndarray:
        """USD / Fabric のローカル行列 `(K, 4, 4)` を組み立てる (`Gf.Matrix4d` と同じ行ベクトル形式・スケール込み)。

        列ベクトル形式では `M_col[:3, :3] = R @ diag(s)` となるが、行ベクトル形式では転置が必要なため
        `out[:, :3, :3] = (rot * local_scale).T`、並進は第 3 行 `out[:, 3, :3] = pos` に配置する。
        """
        pos, rot = self.solve(world)
        out = np.zeros((len(self._paths), 4, 4))
        out[:, :3, :3] = np.swapaxes(rot * self._local_scale[:, None, :], -1, -2)
        out[:, 3, :3] = pos
        out[:, 3, 3] = 1.0
        return out


# ==============================================================================
# ライター抽象基底と USD ライター (GeometricUsdWriter)
# ==============================================================================


class GeometricTransformWriter(ABC):
    """リンク姿勢の書き込み先インターフェース。prim の並びは生成時に渡した `prim_paths` の順。"""

    @abstractmethod
    def write(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        """スケール抜きのワールド姿勢 `world (K, 4, 4)` (列ベクトル形式) を書き込む。"""

    def close(self) -> None:
        """保持している USD / Fabric リソースを解放する。"""


class GeometricUsdWriter(GeometricTransformWriter):
    """`GeometricLayer` の無名 Sublayer 上にある `xformOp:transform` 属性へ直接書き込むライター。

    `Usd.Attribute.Set` ではなく Sdf レベルの `AttributeSpec.default` を `Sdf.ChangeBlock` 内で
    まとめて更新する理由:
        - `Sdf.ChangeBlock` により、全リンク分の変更通知 (`Usd.Notice.ObjectsChanged`) を 1 回に集約できる。
        - Stage の EditTarget 切り替えコストを避け、1 リンクあたり約 12 µs まで高速化できる。
    """

    def __init__(self, layer: GeometricLayer, prim_paths: list[str]) -> None:
        self._layer = layer
        self._paths = list(prim_paths)
        self._specs: list[Sdf.AttributeSpec] | None = None
        self._specs_layer: Sdf.Layer | None = None

    def write(self, solver: LocalPoseSolver, world: np.ndarray) -> None:
        layer = self._layer.sdf_layer
        if layer is None:
            raise RuntimeError("Sublayer が付いていないので USD に書けない")
        # `GeometricLayer` が一度外されて付け直されると `Sdf.Layer` インスタンスが新しくなるため、
        # `self._specs_layer is not layer` を検知して `AttributeSpec` を取り直す。
        if self._specs is None or self._specs_layer is not layer:
            self._specs = self._layer.xform_specs(self._paths)
            self._specs_layer = layer
        flat = solver.local_matrices(world).reshape(len(self._paths), 16)
        with Sdf.ChangeBlock():
            for spec, row in zip(self._specs, flat):
                spec.default = Gf.Matrix4d(*row)

    def close(self) -> None:
        self._specs = None
        self._specs_layer = None


# ==============================================================================
# Fabric ローカル行列ライター (GeometricFabricWriter)
# ==============================================================================


class GeometricFabricWriter(GeometricTransformWriter):
    """Fabric のローカル行列へ姿勢を書き込むライター (描画は高速、USD へは書き出さない)。

    prim 数が `FABRIC_BATCH_MIN_PRIMS` (24) 以上なら `XformPrim.set_local_poses` の Warp 一括パス、
    未満なら `usdrt.hierarchy.IFabricHierarchy.set_local_xform` の個別書き込みを使う。
    """

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
        # `XformPrim.set_local_poses` の四元数規約は `(w, x, y, z)` であり `tf.matrix_to_quat` と一致する
        quat = tf.matrix_to_quat(rot)
        with backend_utils.use_backend("fabric"):
            self._view.set_local_poses(
                translations=wp.array(pos.astype(np.float32), dtype=wp.float32, device="cpu"),
                orientations=wp.array(quat.astype(np.float32), dtype=wp.float32, device="cpu"),
            )

    def read_world(self) -> np.ndarray:
        """Fabric 上のワールド行列 `(K, 4, 4)` (列ベクトル形式・スケール込み) を読み戻す確認用メソッド。

        `get_world_xform` の前に `update_world_xforms()` を呼んでローカル変更をワールドへ伝播させ、
        行ベクトル形式 (`usdrt.Gf.Matrix4d`) を `.T` で列ベクトル形式へ戻す。
        """
        self._hierarchy.update_world_xforms()
        return np.array([np.array(self._hierarchy.get_world_xform(p), dtype=float).reshape(4, 4).T for p in self._rt_paths])

    def close(self) -> None:
        self._view = None


# ==============================================================================
# GPU Fabric 直接ライター (GeometricGpuFabricWriter)
# ==============================================================================

_GPU_FABRIC_KERNEL = None


def _get_gpu_fabric_kernel():
    """`newton_body_q` から `omni:fabric:worldMatrix` を直接生成する Warp カーネルを遅延コンパイルして返す。"""
    global _GPU_FABRIC_KERNEL
    if _GPU_FABRIC_KERNEL is None:
        import warp as wp

        @wp.kernel(enable_backward=False)
        def _write_fabric_world_matrices(
            fabric_transforms: wp.fabricarray(dtype=wp.mat44d),
            newton_indices: wp.fabricarray(dtype=wp.uint32),
            body_scales: wp.array(ndim=1, dtype=wp.vec3f),
            newton_body_q: wp.array(ndim=1, dtype=wp.transformf),
            body_count: int,
        ):
            i = int(wp.tid())
            idx = int(newton_indices[i])
            if idx < body_count:
                xform = newton_body_q[idx]
                s = body_scales[idx]
                # `wp.transform_compose(p, q, s)` は列ベクトル形式の `mat44f` (並進が第 3 列) を返す。
                # 一方 Fabric の `omni:fabric:worldMatrix` は `Gf.Matrix4d` 準拠の行ベクトル形式 (並進が第 3 行)
                # であるため、必ず `wp.transpose(...)` して倍精度 `wp.mat44d` へ変換して書き込む。
                fabric_transforms[i] = wp.mat44d(wp.transpose(wp.transform_compose(xform.p, xform.q, s)))

        _GPU_FABRIC_KERNEL = _write_fabric_world_matrices
    return _GPU_FABRIC_KERNEL


class GeometricGpuFabricWriter:
    """Newton の `state.body_q` (Warp GPU 配列) を CPU を経由せず直接 Fabric に書くライター。

    NVIDIA の `isaacsim.physics.newton` (`FabricManager.update_fabric`) と同じ設計を採用している。

    なぜ `__init__` で `SelectPrims` や `wp.fabricarray` をキャッシュしないのか:
        Kit の `app.update()`・`timeline.play()`・`flush_to_usd()` のタイミングで Fabric の内部バケットが
        再配置されたり、新しい prim 属性が追加されたりすると、過去に取得した `SelectPrims` / `wp.fabricarray`
        が古いバッファを指したまま陳腐化し、書き込んでもビューポートや読み戻しに反映されなくなる。
        そのため毎回の `write_body_q()` で `SelectPrims` と `wp.fabricarray("omni:fabric:worldMatrix")` を
        フレッシュ取得し、選択件数が不足していれば `_tag_prims()` で自動再タグする。

    なぜ `_tag_prims()` で `omni:fabric:localMatrix` を除去するのか:
        `omni:fabric:localMatrix` が prim に残っていると、次の `app.update()` で `IFabricHierarchy` が
        古い `localMatrix` から `omni:fabric:worldMatrix` を再計算して上書きしてしまう。
        `localMatrix` を除去することで、GPU カーネルが書いた `omni:fabric:worldMatrix` をそのまま描画へ通す。
    """

    def __init__(
        self,
        stage: Usd.Stage,
        prim_paths: Sequence[str],
        body_indices: Sequence[int],
        *,
        world_scales: np.ndarray | None = None,
        device: str = "cuda:0",
    ) -> None:
        import usdrt
        import warp as wp
        from pxr import UsdUtils

        self._paths = list(prim_paths)
        self._body_indices = [int(i) for i in body_indices]
        self._device = str(device)
        # インスタンス固有のタグ属性名にし、複数ロボット / 複数 View が同一ステージに共存しても衝突しないようにする
        self._tag_idx = f"geom_newton:idx_{id(self)}"
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = cache.Insert(stage).ToLongInt()
        self._rt_stage = usdrt.Usd.Stage.Attach(stage_id)
        self._usdrt = usdrt

        # `SelectPrims` が返す prim 順は `self._paths` の順とは限らないため、
        # Fabric 側にカスタム scale 属性を足すのではなく Newton の body インデックスで引く GPU 配列として持つ
        scales = (
            np.ones((len(self._paths), 3), dtype=np.float32)
            if world_scales is None
            else np.asarray(world_scales, dtype=np.float32)
        )
        max_idx = max(self._body_indices) if self._body_indices else -1
        scales_by_body = np.ones((max_idx + 1, 3), dtype=np.float32) if max_idx >= 0 else np.ones((1, 3), dtype=np.float32)
        for nb_idx, sc in zip(self._body_indices, scales):
            if nb_idx >= 0:
                scales_by_body[nb_idx] = sc
        self._body_scales = wp.array(scales_by_body, dtype=wp.vec3f, device=self._device)
        self._tag_prims()

    def _tag_prims(self) -> None:
        """対象 prim に Newton body インデックスのタグと `omni:fabric:worldMatrix` を準備し、`localMatrix` を除去する。"""
        usdrt = self._usdrt
        for prim_path, nb_idx in zip(self._paths, self._body_indices):
            rt_prim = self._rt_stage.GetPrimAtPath(usdrt.Sdf.Path(prim_path))
            if not rt_prim.IsValid():
                continue
            rt_prim.CreateAttribute(self._tag_idx, usdrt.Sdf.ValueTypeNames.UInt, True)
            rt_prim.GetAttribute(self._tag_idx).Set(int(nb_idx))
            xf = usdrt.Rt.Xformable(rt_prim)
            if not xf.HasWorldXform():
                xf.SetWorldXformFromUsd()
            # `omni:fabric:localMatrix` が残っていると `IFabricHierarchy` が `worldMatrix` を古い値で上書きするため除去する
            if rt_prim.HasAttribute("omni:fabric:localMatrix"):
                try:
                    rt_prim.RemoveAttribute("omni:fabric:localMatrix")
                except Exception:
                    pass

    def write_body_q(self, body_q_wp, body_count: int) -> None:
        """Newton の `state.body_q` (`wp.array[wp.transform]`) を GPU カーネルで直接 Fabric へ書き込む。

        カーネル起動後に `wp.synchronize_device(self._device)` を呼ぶ理由:
            Warp のカーネル起動は非同期であり、同期せずに戻ると直後の Kit 描画スレッドや
            テスト・ユーザーコードによる Fabric 読み取りが未完了バッファを読んでしまうため。
        """
        if not self._paths:
            return
        import warp as wp

        usdrt = self._usdrt
        require_attrs = [
            (usdrt.Sdf.ValueTypeNames.Matrix4d, "omni:fabric:worldMatrix", usdrt.Usd.Access.ReadWrite),
            (usdrt.Sdf.ValueTypeNames.UInt, self._tag_idx, usdrt.Usd.Access.Read),
        ]
        # 毎回フレッシュに `SelectPrims` を取得し、Play=ON 切替やバケット再配置による陳腐化を防ぐ
        selection = self._rt_stage.SelectPrims(require_attrs=require_attrs, device=self._device)
        if selection.GetCount() < len(self._paths):
            self._tag_prims()
            selection = self._rt_stage.SelectPrims(require_attrs=require_attrs, device=self._device)
        count = int(selection.GetCount())
        if count == 0:
            return
        fabric_transforms = wp.fabricarray(selection, "omni:fabric:worldMatrix")
        newton_indices = wp.fabricarray(selection, self._tag_idx)
        if self._body_scales.shape[0] < body_count:
            pad = np.ones((body_count, 3), dtype=np.float32)
            old = self._body_scales.numpy()
            pad[: old.shape[0]] = old
            self._body_scales = wp.array(pad, dtype=wp.vec3f, device=self._device)
        wp.launch(
            _get_gpu_fabric_kernel(),
            dim=count,
            inputs=[fabric_transforms, newton_indices, self._body_scales, body_q_wp, int(body_count)],
            device=self._device,
        )
        # Kit 描画・後続の Fabric 読み戻しとのデータ競合を防ぐためデバイス同期を明示する
        wp.synchronize_device(self._device)

    def close(self) -> None:
        """Fabric 上に付与したカスタムタグ属性 `self._tag_idx` とスケール配列を解放する。"""
        try:
            usdrt = self._usdrt
            for prim_path in self._paths:
                rt_prim = self._rt_stage.GetPrimAtPath(usdrt.Sdf.Path(prim_path))
                if rt_prim.IsValid() and rt_prim.HasAttribute(self._tag_idx):
                    rt_prim.RemoveAttribute(self._tag_idx)
        except Exception:
            pass
        self._body_scales = None
