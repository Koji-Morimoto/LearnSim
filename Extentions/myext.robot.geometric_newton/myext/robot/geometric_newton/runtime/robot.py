"""GeometricRobot: 物理ソルバーを使わず、関節値からリンクの Transform を計算してステージへ書くロボット。

メソッド名・引数は `isaacsim.core.experimental.prims.Articulation` に合わせてある。
1 台専用のため関節値配列は 1 次元 `(num_dofs,)` の NumPy 配列で、`indices=` 引数は持たない
(同一構造の複数台を一括処理する場合は `GeometricRobotView` を使用する)。
物理シミュレーションを行わない設計のため、速度・トルク・Drive ゲイン・質量などの API は持たない。

    robot = GeometricRobot.from_articulation(stage, "/World/Robot")   # 無名 Sublayer を付けて幾何モードにする
    robot.set_dof_positions([0.1, -0.5, 0.8, 0.0, 0.3, 0.0])
    robot.set_variant("Gripper", "Robotiq_2F_140")                   # Variant を切り替えて自動再構築
    robot.check_collisions()                                          # 既定 Newton バックエンドで 0 フレーム遅延判定
    robot.layer.remove()                                             # 元の Articulation 構成へ非破壊復帰

座標・単位・バックエンドの規約:
    - 長さ: ステージの単位 (通常 m)。回転関節の角度: `angle_unit` (既定 `"rad"`、呼び出しごとに `unit=` で上書き可)。
      直動関節の値は `angle_unit` に関係なく常に長さ。
    - 四元数: `(w, x, y, z)`。4x4 同次変換行列: 列ベクトル形式 (`v' = M @ v`)・スケール抜き。
    - 干渉チェック (`collision_backend`):
      - `"newton"` (既定): 自前コリジョン空間 `GeometricScene` を使い、`write_target="fabric"` のまま
        `flush_to_usd()` を呼ばずに Stop / Play=ON ともに 0 フレーム遅延で干渉・符号付き距離・クリアランスを評価する。
      - `"physx"`: `GeometricCollisionChecker` (`overlap_shape`) を使う。チェック前に必ず `flush_to_usd()` を呼んで
        USD へ姿勢を書き出す必要がある。
    - USD 構造の自動追従: 既存の Articulation から作った場合、ロボット配下の構造変化 (Variant の選択、Joint・剛体の増減)
      を `Usd.Notice.ObjectsChanged` で検知し、次の `update` または次の API 呼び出し時に自動で再構築する。
"""

from __future__ import annotations

from contextlib import contextmanager
import dataclasses
import json
import os
import time
from typing import Any, Callable, Iterator, Literal, Mapping, Sequence, get_args

import numpy as np
from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from ..usd.layer import GeometricLayer, GeometricLayerOptions, LayerTarget, _setting, is_authoring
from ..model import (
    AngleUnit,
    DHParameters,
    GeometricCollisionFilter,
    GeometricCommandConverter,
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
from ..model import transforms as tf
from ..model.command import check_angle_unit
from ..collision.newton_scene import GeometricScene
from ..collision.newton_shapes import _AUTO_FK_MIN_DOFS, CollisionBackend, FkBackend
from ..usd.reader import UsdArticulationSource, UsdRobotInfo, _np, component_patterns, prim_matches, read_articulation, split_scale
from ..usd.writer import GeometricFabricWriter, GeometricGpuFabricWriter, GeometricUsdWriter, LocalPoseSolver, WriteTarget
from .handles import (
    _AUTO_FILTER_REASONS,
    _COLLIDER_ATTRS,
    GeometricDofBinder,
    GeometricJoint,
    GeometricLimitViolation,
    GeometricLink,
    GeometricRobotComponent,
    GeometricStructureChangedError,
    LimitPolicy,
    StructureListener,
    _as_indices,
    _check,
    _log_warn,
)

__all__ = [
    "LimitPolicy",
    "AngleUnit",
    "GeometricStructureChangedError",
    "GeometricLimitViolation",
    "GeometricDofBinder",
    "GeometricRobot",
    "GeometricRobotComponent",
    "GeometricJoint",
    "GeometricLink",
]


# ==============================================================================
# GeometricRobot 本体
# ==============================================================================


class GeometricRobot:
    def __init__(
        self,
        stage: Usd.Stage,
        model: GeometricRobotModel,
        link_prim_paths: dict[str, str],
        *,
        info: UsdRobotInfo | None = None,
        source: UsdArticulationSource | None = None,
        layer: GeometricLayer | None = None,
        layer_options: GeometricLayerOptions | None = None,
        write_target: WriteTarget | None = None,
        limit_policy: LimitPolicy | None = None,
        angle_unit: AngleUnit | None = None,
        apply_layer: bool = True,
        collision_backend: CollisionBackend | None = None,
        fk_backend: FkBackend | None = None,
        scene: GeometricScene | None = None,
        newton_device: str | None = None,
        convex_meshes: bool | None = None,
        debug_draw_colliders: bool | None = None,
        use_gpu_fabric_writer: bool = False,
    ) -> None:
        """通常は `from_articulation` / `from_model` / `from_dh` / `from_mdh` / `from_screw` / `from_urdf` 経由で生成する。

        引数:
            link_prim_paths       : `リンク名 -> prim パス` の対応 (USD の prim 親子関係はリンク親子関係と異なってよい)。
            source                : 既存 USD ロボットの読取元。渡すと Variant 切替や外部構造変更を検知して自動再構築できる。
            layer                 : 共有する `GeometricLayer`。省略時はこのロボット専用の無名 Sublayer を作る。
            write_target          : `"fabric"` (高速描画。`collision_backend="newton"` なら USD flush 不要で 0 フレーム遅延) /
                                    `"usd"` (毎回 Sublayer へ書く) / `"both"` (毎回両方へ書く)。省略時は設定 `write_target` (既定 `"fabric"`)。
            limit_policy          : 可動範囲外の値への対処 (`"clip"` / `"warn"` / `"error"` / `"ignore"`)。
            angle_unit            : 回転関節の既定単位 (`"rad"` / `"deg"`)。直動関節は常に長さ。
            apply_layer           : `True` なら生成直後に `GeometricLayer.apply()` を呼んで幾何モードを有効化する。
            collision_backend     : `"newton"` (既定: `GeometricScene` で 0 フレーム遅延・めり込み量・クリアランス対応) /
                                    `"physx"` (`GeometricCollisionChecker`, チェック前に `flush_to_usd()` が必要)。
            fk_backend            : `"auto"` (既定: 単体は NumPy、View バッチは Newton GPU) / `"numpy"` / `"newton"`。
            scene                 : 複数ロボットで共有する `GeometricScene`。省略時は必要時に専用 `GeometricScene` を遅延生成する。
            newton_device         : Newton バックエンドのデバイス (`"auto"` / `"cuda:0"` / `"cpu"`)。
            convex_meshes         : Newton へ `UsdGeom.Mesh` を登録する際の扱い。
                                    `True` (既定) = 単一凸包 (`GeoType.CONVEX_MESH`) へ変換して高速判定、
                                    `False` = STL 等の元の三角形メッシュそのまま (`GeoType.MESH`) で判定。
            debug_draw_colliders  : `True` にすると Newton 内部コライダー形状 (通常=シアン・干渉中=赤) と
                                    接触点 (オレンジ) を `DebugDraw` で自動可視化する (既定 `False` = 負荷ゼロ)。
                                    生成後も `robot.debug_draw_colliders = True / False` で随時切り替え可能。
            use_gpu_fabric_writer : `True` かつ `fk_backend="newton"` のとき、`state.body_q` を直接 GPU Fabric (`wp.fabricarray`) へ書く。
        """
        lim_pol = limit_policy if limit_policy is not None else _setting("limit_policy", "clip")
        if lim_pol not in get_args(LimitPolicy):
            raise ValueError(f"limit_policy は {get_args(LimitPolicy)} のどれか: {lim_pol!r}")
        target = write_target if write_target is not None else _setting("write_target", "fabric")
        if target not in get_args(WriteTarget):
            raise ValueError(f"write_target は {get_args(WriteTarget)} のどれか: {target!r}")
        col_be = collision_backend if collision_backend is not None else _setting("collision_backend", "newton")
        if col_be not in get_args(CollisionBackend):
            raise ValueError(f"collision_backend は {get_args(CollisionBackend)} のどれか: {col_be!r}")
        fk_be = fk_backend if fk_backend is not None else _setting("fk_backend", "auto")
        if fk_be not in get_args(FkBackend):
            raise ValueError(f"fk_backend は {get_args(FkBackend)} のどれか: {fk_be!r}")
        self._stage = stage
        self._source = source
        self._limit_policy: LimitPolicy = lim_pol
        self._write_target: WriteTarget = target
        self._collision_backend: CollisionBackend = col_be
        self._fk_backend: FkBackend = fk_be
        self._newton_device: str = newton_device if newton_device is not None else _setting("newton_device", "auto")
        self._convex_meshes: bool = (
            bool(convex_meshes)
            if convex_meshes is not None
            else (bool(scene.convex_meshes) if scene is not None else bool(_setting("convex_meshes", True)))
        )
        self._debug_draw_colliders: bool = (
            bool(debug_draw_colliders) if debug_draw_colliders is not None else bool(_setting("debug_draw_colliders", False))
        )
        self._last_collision_pairs: list | None = None
        self._batch_depth: int = 0
        self._batch_dirty: bool = False
        self._use_gpu_fabric_writer: bool = bool(use_gpu_fabric_writer)
        self._scene: GeometricScene | None = scene
        self._owns_scene: bool = False
        self._gpu_fabric_writer: GeometricGpuFabricWriter | None = None
        self._gpu_fabric_failed: bool = False
        self._angle_unit: AngleUnit = check_angle_unit(angle_unit if angle_unit is not None else _setting("angle_unit", "rad"))
        self._closed = False
        self._structure_version = 0
        self._collider_version = 0
        self._collider_layers: list[tuple[str, LayerTarget]] = []
        self._structure_listeners: list[StructureListener] = []
        self._converters: dict[str, tuple[GeometricCommandConverter, dict[tuple[str, str], str] | None]] = {}
        self._active_converters: list[tuple[str, GeometricCommandConverter, np.ndarray, np.ndarray]] | None = None
        self.last_rebuild_timings: dict[str, float] | None = None
        self._filter = GeometricCollisionFilter()
        self._checker = None
        self._maybe_changed = False

        self._setup_structure(model, link_prim_paths, info, keep=None)

        if self._scene is not None:
            self._scene.add_robot(self)

        # Sublayer と書き込み先
        self._layer = layer if layer is not None else GeometricLayer(stage, layer_options)
        self._fabric_writer: GeometricFabricWriter | None = None
        self._fabric_failed = False
        self._usd_dirty = False
        self._usd_sync_pending = False
        self._repush_frames = 0
        self._update_subscription = None
        # `GeometricRobotView` に属している間は、USD への flush を View 側の一括 `Sdf.ChangeBlock` へ委譲する
        self._flush_delegate = None
        self._usd_writer = GeometricUsdWriter(self._layer, self._prim_paths)
        self._layer_token = self._layer.register(**self._entry_kwargs())
        self._layer.add_listener(self._on_layer_changed)

        # 既存ロボットの構造変更 (`source is not None` の Variant / Joint / 剛体) と、
        # コライダー Sublayer の挿抜・Mute/Unmute・形状属性変更を自動検知するため常に通知を購読する。
        self._signature = self._structure_signature() if source is not None else None
        self._notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)
        if source is not None:
            self._subscribe_update()

        if self._layer.is_applied():
            self._on_layer_changed(True)
        elif apply_layer:
            self._layer.apply()

    # ==========================================================================
    # 構造に依存する内部状態の構築
    # ==========================================================================

    def _setup_structure(
        self,
        model: GeometricRobotModel,
        link_prim_paths: dict[str, str],
        info: UsdRobotInfo | None,
        keep: tuple[dict[str, float], dict[str, float]] | None,
    ) -> None:
        """モデル構造に依存する配列・フィルタ・ソルバーを初期化する。

        再構築時 (`keep` 指定時) は `(旧関節値辞書, 旧既定関節値辞書)` を DOF 名で引き継ぎ、
        ユーザー独自の `collision_filter` ルールも消さずに自動ルール (`_AUTO_FILTER_REASONS`) のみ再生成する。
        """
        self._model = model
        self._info = info
        self._link_path_map = {name: path for name, path in link_prim_paths.items() if path}
        unknown = [n for n in self._link_path_map if n not in model.link_names]
        if unknown:
            raise ValueError(f"link_prim_paths にモデルに無いリンク名がある: {unknown}")
        names = model.link_names
        self._prim_link_indices = np.array([i for i, n in enumerate(names) if n in self._link_path_map], dtype=int)
        self._prim_paths = [self._link_path_map[names[i]] for i in self._prim_link_indices]
        self._solver = LocalPoseSolver(self._stage, self._prim_paths)
        # 各リンクの親リンク番号を配列化しておく (根リンクは自分自身)。`get_link_local_poses` を一括計算するため。
        parent_of = {j.child_link: j.parent_link for j in model.joints}
        self._parent_link_index = np.array([model.get_link_index(parent_of.get(n, n)) for n in names], dtype=int)
        self._is_root_link = np.array([n not in parent_of for n in names], dtype=bool)

        self._lower = model.dof_lower_limits
        self._upper = model.dof_upper_limits
        self._revolute = np.array([t == "revolute" for t in model.dof_types], dtype=bool)
        dof_names = model.dof_names
        zero = np.clip(np.zeros(model.num_dofs), self._lower, self._upper)
        if keep is None:
            self._q = model.resolve_mimic(zero)
            self._default_q = self._q.copy()
            root = model.root_link or ""
            if root in self._link_path_map:
                cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                self._base = split_scale(_np(cache.GetLocalToWorldTransform(self._stage.GetPrimAtPath(self._link_path_map[root]))))[0]
            else:
                self._base = np.eye(4)
            self._default_base = self._base.copy()
        else:
            old_q, old_default = keep
            self._q = model.resolve_mimic(np.clip(np.array([old_q.get(n, z) for n, z in zip(dof_names, zero)]), self._lower, self._upper))
            self._default_q = model.resolve_mimic(np.array([old_default.get(n, z) for n, z in zip(dof_names, zero)]))
        self._last_requested_q: np.ndarray = self._q.copy()
        self._world = self._base @ model.forward_kinematics(self._q, resolve_mimic=False)

        # 自動由来の無視ペアだけを差し替え、同一 `self._filter` インスタンスを保持する (利用者の参照を壊さないため)
        for reason in _AUTO_FILTER_REASONS:
            self._filter.remove_by_reason(reason)
        for parent, child, joint_name in model.adjacent_link_pairs():
            self._filter.add_pair(parent, child, reason="mount" if joint_name.endswith("/mount") else "adjacent")
        if info is not None:
            for a, b in info.colliding_adjacent_pairs:  # Joint が「接続 2 物体も衝突させる」設定の組は除外を解除
                self._filter.remove_pair(a, b)
            for a, b in info.filtered_pairs:
                self._filter.add_pair(a, b, reason="usd_filtered_pairs")
        self._colliders: dict[str, list[str]] | None = None
        self._collider_version += 1
        # DOF 名 -> 番号の高速ルックアップ辞書と独立軸一覧を再構築時に 1 回だけ構築する
        self._dof_name_to_idx: dict[str, int] = {name: i for i, name in enumerate(dof_names)}
        self._independent_dof_indices: np.ndarray = model.independent_dof_indices
        self._independent_dof_names: list[str] = [dof_names[i] for i in self._independent_dof_indices]
        # 構成要素 (`GeometricRobotComponent`) はロボット生成・再構築時点で即時生成しておく
        # (`IsaacRobotAPI` 自動検出または `UsdArticulationSource(components=...)` / `attach()` 由来)
        self._components: dict[str, GeometricRobotComponent] = {
            comp_name: GeometricRobotComponent(self, comp_name) for comp_name in model.components
        }
        self._active_converters = None
        self.joints = {spec.name: GeometricJoint(self, spec) for spec in model.joints}
        self.links = {spec.name: GeometricLink(self, spec, i) for i, spec in enumerate(model.links)}

    def _entry_kwargs(self) -> dict:
        info = self._info
        adjacent = [
            (self._link_path_map[a], self._link_path_map[b]) for a, b, _ in self._model.adjacent_link_pairs() if a in self._link_path_map and b in self._link_path_map
        ]
        return {
            "link_prim_paths": list(self._prim_paths),
            "articulation_root_paths": list(info.articulation_root_paths) if info is not None else [],
            "joint_prim_paths": list(info.joint_prim_paths) if info is not None else [],
            "adjacent_link_paths": adjacent,
        }

    # ==========================================================================
    # ファクトリメソッド (USD / モデル / DH / MDH / スクリュー / URDF)
    # ==========================================================================

    @classmethod
    def from_articulation(
        cls,
        stage: Usd.Stage,
        source: UsdArticulationSource | str,
        *,
        auto_components: bool | None = None,
        merge_variant_components: bool | None = None,
        **kwargs,
    ) -> "GeometricRobot":
        """ステージ上の既存ロボット (`ArticulationRoot` と `Joint` を持つ prim) から作る。元の USD は書き換えない。

        引数:
            source                   : prim パス文字列または `UsdArticulationSource`。
            auto_components          : `True` (既定) のとき、ロボット配下の `IsaacRobotAPI` から
                                       `GeometricRobotComponent` を自動生成する (`source.components` 明示時はそちらを優先)。
            merge_variant_components : `True` のとき、VariantSet 経由でぶら下がる子 `IsaacRobotAPI` を
                                       独立コンポーネントにせず親コンポーネントへ統合する (既定 `False`)。
            kwargs                   : `__init__` のキーワード引数 (`convex_meshes`, `debug_draw_colliders`,
                                       `write_target`, `angle_unit`, `limit_policy`, `collision_backend`, `fk_backend` 等)。
        """
        auto_flag = bool(_setting("auto_components", True)) if auto_components is None else bool(auto_components)
        merge_flag = (
            bool(_setting("merge_variant_components", False))
            if merge_variant_components is None
            else bool(merge_variant_components)
        )
        if isinstance(source, str):
            source = UsdArticulationSource(
                source, auto_components=auto_flag, merge_variant_components=merge_flag
            )
        else:
            changes: dict[str, Any] = {}
            if auto_components is not None and source.auto_components != auto_flag:
                changes["auto_components"] = auto_flag
            if merge_variant_components is not None and source.merge_variant_components != merge_flag:
                changes["merge_variant_components"] = merge_flag
            if changes:
                source = dataclasses.replace(source, **changes)
        model, info = read_articulation(stage, source)
        for message in info.warnings:
            _log_warn(message)
        return cls(stage, model, info.link_prim_paths, info=info, source=source, **kwargs)

    @classmethod
    def from_model(cls, stage: Usd.Stage, model: GeometricRobotModel, prim_path: str, *, style=None, base_transform: np.ndarray | None = None, **kwargs) -> "GeometricRobot":
        """内部モデルから簡易形状の prim を `prim_path` 配下に新規生成して作る (`Articulation` も剛体も付けない)。

        `style` は `GeometricVisualStyle` (太さ・色・コライダー付与有無)、`base_transform` は根リンクのワールド姿勢 `(4, 4)`。
        """
        from ..usd.builder import build_robot_prims

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
        """URDF の運動学から作る (形状は簡易カプセル。メッシュファイルは読まない)。"""
        source = UrdfSource(source) if isinstance(source, str) else source
        return cls.from_model(stage, source.to_model(), prim_path, **kwargs)

    # ==========================================================================
    # モデル属性・インデックス照会 (Articulation 互換)
    # ==========================================================================

    @property
    def name(self) -> str:
        return self.model.name

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def model(self) -> GeometricRobotModel:
        self._ensure_current()
        return self._model

    @property
    def layer(self) -> GeometricLayer:
        return self._layer

    @property
    def info(self) -> UsdRobotInfo | None:
        """既存ロボットから作ったときの USD 側の情報(読み取り時の警告など)。"""
        self._ensure_current()
        return self._info

    @property
    def structure_version(self) -> int:
        """作り直すたびに 1 増える番号。"""
        self._ensure_current()
        return self._structure_version

    @property
    def pending_render_sync_frames(self) -> int:
        """Viewport (Fabric/FSD) への再送が完了するまでの残りフレーム数 (`0` なら描画同期完了)。

        FK・`get_link_world_poses()`・`check_collisions()` (Newton バックエンド) は
        この値が `0` より大きくても即座に最新値で動作するため待機不要。
        VariantSet 切替や `flush_to_usd()` の直後に、Viewport の表示・スクリーンショットが
        落ち着くまで待ちたい場合だけ参照する。
        """
        return int(self._repush_frames)

    @property
    def is_render_ready(self) -> bool:
        """構造再構築待ち・Viewport Fabric 再送待ちの両方が完了していれば `True`。

        計算 (FK / Newton 干渉判定) には不要 (常に 0 フレーム遅延)。
        VariantSet 切替直後に Viewport (FSD) の描画が安定した有効フレームかどうかを判定する用途向け。
        """
        return (not self._maybe_changed) and (self._repush_frames == 0)

    @property
    def num_dofs(self) -> int:
        return self.model.num_dofs

    @property
    def dof_names(self) -> list[str]:
        return self.model.dof_names

    @property
    def dof_paths(self) -> list[str | None]:
        return [j.prim_path for j in self.model.dof_joints]

    @property
    def dof_types(self) -> list[str]:
        return list(self.model.dof_types)

    @property
    def independent_dof_indices(self) -> np.ndarray:
        """mimic 従動軸を除いた独立 DOF の番号配列 (このロボットの `dof_names` 上のインデックス)。"""
        self._ensure_current()
        return self._independent_dof_indices.copy()

    @property
    def independent_dof_names(self) -> list[str]:
        """mimic 従動軸を除いた独立 DOF の名前一覧 (`set_dof_positions` で実際に指令できる軸)。"""
        self._ensure_current()
        return list(self._independent_dof_names)

    @property
    def num_joints(self) -> int:
        return self.model.num_joints

    @property
    def joint_names(self) -> list[str]:
        return self.model.joint_names

    @property
    def joint_paths(self) -> list[str | None]:
        return [j.prim_path for j in self.model.joints]

    @property
    def joint_types(self) -> list[str]:
        return list(self.model.joint_types)

    @property
    def num_links(self) -> int:
        return self.model.num_links

    @property
    def link_names(self) -> list[str]:
        return self.model.link_names

    @property
    def link_paths(self) -> list[str | None]:
        return [self._link_path_map.get(n) for n in self.model.link_names]

    @property
    def link_path_map(self) -> dict[str, str]:
        """リンク名 -> prim パス(prim を持つリンクだけ)。"""
        self._ensure_current()
        return self._link_path_map

    def get_dof_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        model = self.model
        return np.array([model.get_dof_index(n) for n in names], dtype=int)

    def get_joint_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        model = self.model
        return np.array([model.get_joint_index(n) for n in names], dtype=int)

    def get_link_indices(self, names: str | Sequence[str]) -> np.ndarray:
        names = [names] if isinstance(names, str) else list(names)
        model = self.model
        return np.array([model.get_link_index(n) for n in names], dtype=int)

    # ==========================================================================
    # 角度の単位変換
    # ==========================================================================

    @property
    def angle_unit(self) -> AngleUnit:
        """回転関節の値の既定単位 (`"rad"` または `"deg"`)。"""
        return self._angle_unit

    @angle_unit.setter
    def angle_unit(self, value: AngleUnit) -> None:
        self._angle_unit = check_angle_unit(value)

    def _unit_scale(self, idx: np.ndarray, unit: AngleUnit | None) -> np.ndarray | None:
        """入力値に掛けるとラジアン (直動関節は 1.0 倍のまま) になる換算係数。`"rad"` なら `None` を返し乗算を省く。"""
        unit = self._angle_unit if unit is None else check_angle_unit(unit)
        if unit == "rad":
            return None
        return np.where(self._revolute[idx], np.pi / 180.0, 1.0)

    # ==========================================================================
    # 関節値の読み書き・可動範囲・バッチ更新・構造確認
    # ==========================================================================

    @contextmanager
    def batch_update(self) -> Iterator[None]:
        """複数の部位・関節更新を 1 回の FK と Fabric 書き込みへまとめるコンテキストマネージャ。

        使用例 (multi-arm で 4 本のアームを順に書いても FK + Fabric 書込はブロック終了時に 1 回だけ):
            with robot.batch_update():
                robot.get_component("arm0").set_dof_positions(q0)
                robot.get_component("arm1").set_dof_positions(q1)
        """
        self._batch_depth += 1
        try:
            yield
        finally:
            self._batch_depth -= 1
            if self._batch_depth == 0 and self._batch_dirty:
                self._batch_dirty = False
                self._q = self._model.resolve_mimic(self._q)
                self._push()

    def _resolve_dof_selector(self, dof_indices, dof_names) -> np.ndarray:
        """`dof_indices` / `dof_names` を DOF 番号配列へ解決する。`dof_indices` 単独時は高速経路を使う。"""
        if dof_names is None:
            return _as_indices(dof_indices, self._model.num_dofs)
        if dof_indices is not None:
            raise ValueError("dof_indices と dof_names は同時に指定できない")
        names = [dof_names] if isinstance(dof_names, str) else list(dof_names)
        name_to_idx = self._dof_name_to_idx
        missing = [n for n in names if n not in name_to_idx]
        if missing:
            raise KeyError(f"存在しない DOF 名: {missing} (利用可能: {self._model.dof_names})")
        return np.fromiter((name_to_idx[n] for n in names), dtype=int, count=len(names))

    def bind_dofs(
        self,
        dof_names: Sequence[str],
        *,
        ignore_unknown: bool = True,
    ) -> GeometricDofBinder:
        """VariantSet 切替で DOF 数や並びが変わっても高速かつ安全に配列書き込みできる `GeometricDofBinder` を作る。

        `dof_names` には全 Variant で登場し得る軸名 (または共通軸名) の並びを渡す。
        毎フレームの `binder.set_positions(arr)` は辞書引きなしで NumPy 配列のまま更新でき、
        VariantSet が切り替わったフレームだけ自動で再バインドして、現 Variant に無い軸をスキップする。
        """
        self._ensure_current()
        return GeometricDofBinder(self, dof_names, ignore_unknown=ignore_unknown)

    def set_dof_positions(
        self,
        positions,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
        ignore_unknown: bool = False,
    ) -> None:
        """関節値を更新し、FK と書き込みを実行する。

        重要仕様:
            - `dof_names` / `dof_indices` ともに省略して配列を渡す場合、配列長は **mimic 従動軸を含む `num_dofs`**。
            - mimic の従動軸へ書いた値は、直後の `resolve_mimic()` により参照軸から再計算された値で必ず上書きされる。
            - **最速ホットパス**: 毎フレーム呼ぶループでは NumPy 配列 + `dof_indices` (または事前取得した
              `component.set_dof_positions(arr)` / `binder = robot.bind_dofs(...)`) を使うこと。
              辞書や `dof_names=` は名前解決のハッシュ引きが入るため初期化・テスト・低頻度操作向け。
            - `positions` に辞書 `{"joint_name": value, ...}` を渡すと指定した軸だけ部分更新できる
              (このとき `dof_indices` / `dof_names` は併用不可)。
            - `ignore_unknown=True` を指定すると、辞書または `dof_names=` に現在の Variant 構成へ存在しない
              DOF 名が含まれていても `KeyError` を出さずスキップして、存在する軸だけを更新する
              (VariantSet 切替で軸数・軸構成が変化するロボット向け)。
            - `unit` 省略時は `angle_unit` を使用。直動関節の値は常にステージの長さ単位。
        """
        self._ensure_current()
        # 最速パス: 非辞書かつ dof_names 未指定
        if dof_names is None and not isinstance(positions, Mapping):
            idx = _as_indices(dof_indices, self._model.num_dofs)
            values = np.asarray(positions, dtype=float).reshape(-1)
            if values.shape[0] != idx.shape[0]:
                raise ValueError(f"positions の個数 {values.shape[0]} が DOF の個数 {idx.shape[0]} と合わない")
            scale = self._unit_scale(idx, unit)
            self._set_positions_rad(values if scale is None else values * scale, idx)
            return

        if isinstance(positions, Mapping):
            if dof_indices is not None or dof_names is not None:
                raise ValueError("positions に辞書を渡すときは dof_indices / dof_names を併用できない")
            if not positions:
                return
            name_to_idx = self._dof_name_to_idx
            if ignore_unknown:
                matched = [(name_to_idx[k], v) for k, v in positions.items() if k in name_to_idx]
                if not matched:
                    return
                idx = np.fromiter((i for i, _ in matched), dtype=int, count=len(matched))
                values = np.fromiter((v for _, v in matched), dtype=float, count=len(matched))
            else:
                keys = list(positions.keys())
                missing = [k for k in keys if k not in name_to_idx]
                if missing:
                    raise KeyError(f"存在しない DOF 名: {missing} (利用可能: {self._model.dof_names})")
                idx = np.fromiter((name_to_idx[k] for k in keys), dtype=int, count=len(keys))
                values = np.fromiter((positions[k] for k in keys), dtype=float, count=len(keys))
        else:
            if dof_indices is not None:
                raise ValueError("dof_indices と dof_names は同時に指定できない")
            names = [dof_names] if isinstance(dof_names, str) else list(dof_names)
            raw_values = np.asarray(positions, dtype=float).reshape(-1)
            if raw_values.shape[0] != len(names):
                raise ValueError(f"positions の個数 {raw_values.shape[0]} が DOF の個数 {len(names)} と合わない")
            name_to_idx = self._dof_name_to_idx
            if ignore_unknown:
                keep_src = [i for i, n in enumerate(names) if n in name_to_idx]
                if not keep_src:
                    return
                idx = np.fromiter((name_to_idx[names[i]] for i in keep_src), dtype=int, count=len(keep_src))
                values = raw_values[keep_src]
            else:
                missing = [n for n in names if n not in name_to_idx]
                if missing:
                    raise KeyError(f"存在しない DOF 名: {missing} (利用可能: {self._model.dof_names})")
                idx = np.fromiter((name_to_idx[n] for n in names), dtype=int, count=len(names))
                values = raw_values
        scale = self._unit_scale(idx, unit)
        self._set_positions_rad(values if scale is None else values * scale, idx)

    def _set_positions_rad(self, values: np.ndarray, idx: np.ndarray) -> None:
        applied = self._apply_limits(values, idx)
        self._q[idx] = applied
        if self._batch_depth > 0:
            # `batch_update()` 内では `_push()` を遅延し、ブロック終了時に 1 回だけ FK + 書き込みを実行する
            self._batch_dirty = True
            return
        # mimic の従動軸をこの時点で解決しておき、後段の `_compute_world` や `GeometricScene.sync_robots` では
        # `resolve_mimic=False` で二重計算を避ける。
        self._q = self._model.resolve_mimic(self._q)
        if len(self._independent_dof_indices) < self._model.num_dofs:
            # ユーザーが直接指定していない mimic 従動軸は、解決後の実姿勢と要求値を揃えておく
            touched = set(int(i) for i in idx.tolist())
            for d, js in enumerate(self._model.dof_joints):
                if js.mimic is not None and d not in touched:
                    self._last_requested_q[d] = self._q[d]
        self._push()

    def get_dof_positions(
        self,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> np.ndarray:
        """現在の関節値 (mimic 解決済み) を返す。"""
        self._ensure_current()
        idx = self._resolve_dof_selector(dof_indices, dof_names)
        scale = self._unit_scale(idx, unit)
        return self._q[idx].copy() if scale is None else self._q[idx] / scale

    def get_dof_positions_dict(
        self,
        *,
        dof_names: str | Sequence[str] | None = None,
        independent_only: bool = False,
        unit: AngleUnit | None = None,
    ) -> dict[str, float]:
        """現在の関節値を `{dof_name: value}` 辞書で返す (デバッグ・ログ・名前指定操作向け)。"""
        self._ensure_current()
        if dof_names is not None:
            idx = self._resolve_dof_selector(None, dof_names)
            names = [self._model.dof_names[i] for i in idx]
        elif independent_only:
            idx = self._independent_dof_indices
            names = self._independent_dof_names
        else:
            idx = np.arange(self._model.num_dofs, dtype=int)
            names = self._model.dof_names
        vals = self.get_dof_positions(dof_indices=idx, unit=unit)
        return dict(zip(names, vals.tolist()))

    def set_dof_position_targets(
        self,
        positions,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> None:
        """Drive を持たないため `set_dof_positions` と同義 (`Articulation` 用コードを移行するための別名)。"""
        self.set_dof_positions(positions, dof_indices=dof_indices, dof_names=dof_names, unit=unit)

    def get_dof_position_targets(
        self,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> np.ndarray:
        return self.get_dof_positions(dof_indices=dof_indices, dof_names=dof_names, unit=unit)

    def get_dof_limits(
        self,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        self._ensure_current()
        idx = self._resolve_dof_selector(dof_indices, dof_names)
        scale = self._unit_scale(idx, unit)
        if scale is None:
            return self._lower[idx].copy(), self._upper[idx].copy()
        return self._lower[idx] / scale, self._upper[idx] / scale

    def set_dof_limits(
        self,
        lower=None,
        upper=None,
        *,
        dof_indices=None,
        dof_names: str | Sequence[str] | None = None,
        unit: AngleUnit | None = None,
    ) -> None:
        """このインスタンス上の可動範囲を変更する (モデルと USD は書き換えない。再構築時はモデルの値へ戻る)。"""
        self._ensure_current()
        idx = self._resolve_dof_selector(dof_indices, dof_names)
        scale = self._unit_scale(idx, unit)
        factor = 1.0 if scale is None else scale
        if lower is not None:
            self._lower[idx] = np.asarray(lower, dtype=float).reshape(-1) * factor
        if upper is not None:
            self._upper[idx] = np.asarray(upper, dtype=float).reshape(-1) * factor

    @property
    def limit_policy(self) -> LimitPolicy:
        """関節可動範囲の適用方針 (`"clip"` / `"reject"` / `"warn"` / `"error"` / `"ignore"`)。

        - `"clip"` (既定): 可動範囲を超えた軸を上下限で切り詰めて適用する (軸リミットを超えない)。
        - `"reject"`: 1 軸でも可動範囲を超えた指令を受け取ったら例外を出さずに指令全体を拒否し、
          直前の有効姿勢を維持する (超過した軸は `get_limit_violations()` で参照可能)。
        - `"warn"`: 警告ログを出してから上下限で切り詰めて適用する。
        - `"error"`: 可動範囲外の値で `ValueError` を送出する。
        - `"ignore"`: 可動範囲チェックを行わずそのまま適用する。
        """
        return self._limit_policy

    @limit_policy.setter
    def limit_policy(self, value: LimitPolicy) -> None:
        if value not in get_args(LimitPolicy):
            raise ValueError(f"limit_policy は {get_args(LimitPolicy)} のどれか: {value!r}")
        self._limit_policy = value

    def get_limit_violations(
        self,
        *,
        atol: float = 1e-6,
        include_at_limit: bool = True,
        unit: AngleUnit | None = None,
    ) -> list[GeometricLimitViolation]:
        """リミットに到達または超過している全 DOF の詳細情報 (`GeometricLimitViolation`) を返す。

        各要素には `child_link` / `child_link_path` / `side` (`"lower"` / `"upper"`) / `exceeded` / `at_limit` が
        含まれるため、リミット到達した軸のリンクに色を付ける処理へそのまま渡せる。

        引数:
            atol             : 内部単位 (回転=rad, 直動=m) での境界判定許容誤差。
            include_at_limit : `True` (既定) ならリミットちょうどの軸も含める。
                               `False` なら直近指令値または実姿勢がリミットを実際に超過した軸のみ返す。
            unit             : 返却する数値フィールドの角度単位 (`None` で `angle_unit`)。
        """
        self._ensure_current()
        n = self._model.num_dofs
        if n == 0:
            return []
        scale = self._unit_scale(np.arange(n, dtype=int), unit)
        out: list[GeometricLimitViolation] = []
        for d, js in enumerate(self._model.dof_joints):
            lo = float(self._lower[d])
            hi = float(self._upper[d])
            req = float(self._last_requested_q[d])
            act = float(self._q[d])
            has_lo = bool(np.isfinite(lo))
            has_hi = bool(np.isfinite(hi))
            exc_lo = has_lo and (req < lo - atol or act < lo - atol)
            exc_hi = has_hi and (req > hi + atol or act > hi + atol)
            at_lo = has_lo and (act <= lo + atol)
            at_hi = has_hi and (act >= hi - atol)
            side: Literal["lower", "upper"] | None = None
            exceeded = False
            at_limit = False
            if exc_lo or (include_at_limit and at_lo):
                side = "lower"
                exceeded = exc_lo
                at_limit = at_lo or exc_lo
            elif exc_hi or (include_at_limit and at_hi):
                side = "upper"
                exceeded = exc_hi
                at_limit = at_hi or exc_hi
            if side is None:
                continue
            inv = 1.0 if scale is None else (1.0 / float(scale[d]))
            out.append(
                GeometricLimitViolation(
                    dof_index=d,
                    dof_name=js.name,
                    joint_name=js.name,
                    child_link=js.child_link,
                    child_link_path=self._link_path_map.get(js.child_link),
                    parent_link=js.parent_link,
                    side=side,
                    requested_value=req * inv,
                    actual_value=act * inv,
                    lower_limit=(lo * inv) if has_lo else lo,
                    upper_limit=(hi * inv) if has_hi else hi,
                    exceeded=exceeded,
                    at_limit=at_limit,
                )
            )
        return out

    def get_limit_violation_links(
        self,
        *,
        atol: float = 1e-6,
        include_at_limit: bool = True,
    ) -> dict[str, Literal["lower", "upper"]]:
        """リミットに到達・超過している関節の子リンク名 -> `"lower"` / `"upper"` の辞書を返す (着色用途向け)。"""
        return {v.child_link: v.side for v in self.get_limit_violations(atol=atol, include_at_limit=include_at_limit)}

    def is_at_limit(self, *, atol: float = 1e-6, include_at_limit: bool = True) -> bool:
        """いずれかの関節がリミット到達または超過状態にあれば `True`。"""
        return bool(self.get_limit_violations(atol=atol, include_at_limit=include_at_limit))

    def describe_dict(self) -> dict[str, Any]:
        """ロボットの構成・設定・構成要素・全 DOF・VariantSet を JSON 互換の辞書で返す。"""
        self._ensure_current()
        model = self._model
        unit_label = self._angle_unit
        lo, hi = self.get_dof_limits()
        pos = self.get_dof_positions()

        def _json_num(val: float) -> float | None:
            f = float(val)
            return f if np.isfinite(f) else None

        return {
            "name": model.name,
            "summary": {
                "num_links": model.num_links,
                "num_joints": model.num_joints,
                "num_dofs": model.num_dofs,
                "num_independent_dofs": len(self._independent_dof_indices),
            },
            "config": {
                "write_target": self._write_target,
                "collision_backend": self._collision_backend,
                "collision_mode": self._layer.options.collision_mode,
                "fk_backend": self._fk_backend,
                "convex_meshes": self._convex_meshes,
                "debug_draw_colliders": self._debug_draw_colliders,
                "angle_unit": unit_label,
                "limit_policy": self._limit_policy,
                "merge_variant_components": (
                    bool(self._source.merge_variant_components) if self._source is not None else False
                ),
            },
            "components": {
                c: {
                    "num_dofs": self._components[c].num_dofs,
                    "num_links": len(lks),
                    "dof_names": self._components[c].dof_names,
                    "independent_dof_names": self._components[c].independent_dof_names,
                }
                for c, lks in model.components.items()
            },
            "dofs": [
                {
                    "index": d,
                    "name": js.name,
                    "type": js.joint_type,
                    "unit": unit_label if js.joint_type == "revolute" else "m",
                    "position": _json_num(pos[d]),
                    "lower_limit": _json_num(lo[d]),
                    "upper_limit": _json_num(hi[d]),
                    "parent_link": js.parent_link,
                    "child_link": js.child_link,
                    "component": model.component_of_link(js.child_link),
                    "prim_path": js.prim_path,
                    "mimic": (
                        {
                            "reference_joint": js.mimic.reference_joint,
                            "multiplier": float(js.mimic.multiplier),
                            "offset": float(js.mimic.offset),
                        }
                        if js.mimic is not None
                        else None
                    ),
                }
                for d, js in enumerate(model.dof_joints)
            ],
            "variant_sets": self.get_variants() if self._source is not None else {},
        }

    def describe(self, *, indent: int = 2) -> str:
        """ロボットの構成・設定・構成要素・全 DOF・VariantSet をエディタで閲覧・折りたたみしやすい JSON 文字列で返す。

        無制限関節の `±inf` は RFC 8259 準拠の `null` として出力する。辞書で受け取りたい場合は `describe_dict()` を使用する。
        """
        return json.dumps(self.describe_dict(), ensure_ascii=False, indent=indent, allow_nan=False)

    def _apply_limits(self, values: np.ndarray, idx: np.ndarray) -> np.ndarray:
        self._last_requested_q[idx] = values
        if self._limit_policy == "ignore":
            return values
        lower, upper = self._lower[idx], self._upper[idx]
        outside = (values < lower - 1e-12) | (values > upper + 1e-12)
        if outside.any():
            if self._limit_policy == "reject":
                # 1 軸でも範囲外なら直前の有効姿勢を維持する (`_last_requested_q` には要求値を残す)
                return self._q[idx].copy()
            if self._limit_policy in ("warn", "error"):
                names = self._model.dof_names
                message = f"可動範囲の外の値(ラジアン): {dict(zip([names[i] for i in idx[outside]], values[outside].tolist()))}"
                if self._limit_policy == "error":
                    raise ValueError(message)
                _log_warn(message + "(範囲内に切り詰めた)")
        return np.clip(values, lower, upper)

    # ==========================================================================
    # 根リンク・各リンクの姿勢とヤコビアン
    # ==========================================================================

    def get_world_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """根リンクのワールド姿勢 `(位置 (3,), 四元数 (4,) = (w, x, y, z))` を返す。"""
        self._ensure_current()
        return self._base[:3, 3].copy(), tf.matrix_to_quat(self._base[:3, :3])

    def set_world_poses(self, positions=None, orientations=None) -> None:
        """根リンクのワールド姿勢を変更してロボット全体を移動させる。省略した側は現在値を維持する。"""
        self._ensure_current()
        if positions is not None:
            self._base[:3, 3] = np.asarray(positions, dtype=float).reshape(3)
        if orientations is not None:
            self._base[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        if self._batch_depth > 0:
            self._batch_dirty = True
            return
        self._push()

    def get_local_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """根リンク prim の親 prim から見た姿勢 `(位置, 四元数 = (w, x, y, z))`。親にスケールがあっても長さはワールド単位。"""
        self._ensure_current()
        local = tf.invert(self._root_parent_world()) @ self._base
        return local[:3, 3].copy(), tf.matrix_to_quat(local[:3, :3])

    def set_local_poses(self, translations=None, orientations=None) -> None:
        self._ensure_current()
        parent = self._root_parent_world()
        local = tf.invert(parent) @ self._base
        if translations is not None:
            local[:3, 3] = np.asarray(translations, dtype=float).reshape(3)
        if orientations is not None:
            local[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        self._base = parent @ local
        if self._batch_depth > 0:
            self._batch_dirty = True
            return
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
        """全リンクのワールド姿勢 `(num_links, 4, 4)` (列ベクトル形式・スケール抜き)。"""
        self._ensure_current()
        return self._world.copy()

    def get_link_world_poses(self, *, link_indices=None) -> tuple[np.ndarray, np.ndarray]:
        """リンクのワールド姿勢 `(位置 (L, 3), 四元数 (L, 4) = (w, x, y, z))`。`Articulation` には無い拡張メソッド。"""
        self._ensure_current()
        world = self._world[_as_indices(link_indices, self._model.num_links)]
        return world[:, :3, 3].copy(), tf.matrix_to_quat(world[:, :3, :3])

    def get_link_local_poses(self, *, link_indices=None) -> tuple[np.ndarray, np.ndarray]:
        """親リンクから見た各リンクの姿勢 `(位置 (L, 3), 四元数 (L, 4) = (w, x, y, z))`。根リンクはワールド基準。"""
        self._ensure_current()
        idx = _as_indices(link_indices, self._model.num_links)
        parent = self._world[self._parent_link_index[idx]]
        parent[self._is_root_link[idx]] = np.eye(4)
        local = tf.invert(parent) @ self._world[idx]
        return local[:, :3, 3], tf.matrix_to_quat(local[:, :3, :3])

    def get_jacobian_matrices(self) -> np.ndarray:
        """全リンクのヤコビアン `(num_links, 6, num_dofs)`。ワールド座標系、行は `[並進速度 3 行; 角速度 3 行]`。

        回転関節の列は常にラジアンあたり (`angle_unit` に依存しない)。
        mimic の従動軸の列はその軸単独を動かした値 (参照軸へ畳み込んだヤコビアンが必要な場合は `model.jacobian(q, reduced=True)`)。
        """
        self._ensure_current()
        jac = self._model.jacobian(self._q)
        rot = self._base[:3, :3]
        out = np.empty_like(jac)
        out[:, :3, :] = np.einsum("ij,ljd->lid", rot, jac[:, :3, :])
        out[:, 3:, :] = np.einsum("ij,ljd->lid", rot, jac[:, 3:, :])
        return out

    # ==========================================================================
    # 既定の状態とリセット
    # ==========================================================================

    def set_default_state(self, positions=None, orientations=None, dof_positions=None, *, unit: AngleUnit | None = None) -> None:
        """`reset_to_default_state()` で復帰する既定状態を設定する。省略した項目は現在の既定値を維持する。"""
        self._ensure_current()
        if positions is not None:
            self._default_base[:3, 3] = np.asarray(positions, dtype=float).reshape(3)
        if orientations is not None:
            self._default_base[:3, :3] = tf.quat_to_matrix(np.asarray(orientations, dtype=float).reshape(4))
        if dof_positions is not None:
            values = np.asarray(dof_positions, dtype=float).reshape(self._model.num_dofs)
            scale = self._unit_scale(np.arange(self._model.num_dofs), unit)
            self._default_q = self._model.resolve_mimic(values if scale is None else values * scale)

    def get_default_state(self, *, unit: AngleUnit | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """`(根の位置, 根の四元数 (w, x, y, z), 関節値)` を返す。"""
        self._ensure_current()
        scale = self._unit_scale(np.arange(self._model.num_dofs), unit)
        q = self._default_q.copy() if scale is None else self._default_q / scale
        return self._default_base[:3, 3].copy(), tf.matrix_to_quat(self._default_base[:3, :3]), q

    def reset_to_default_state(self) -> None:
        self._ensure_current()
        self._base = self._default_base.copy()
        self._q = self._default_q.copy()
        self._last_requested_q = self._q.copy()
        self._push()

    # ==========================================================================
    # 逆運動学 (IK)
    # ==========================================================================

    def get_kinematic_chain(self, base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> KinematicChain:
        """`base_link` から `tip_link` までの直列鎖を取り出す (`chain.dof_indices` がこのロボットの DOF 番号)。

        `source=(id(self), self._structure_version)` を刻印しておくことで、ロボット再構築後に
        古い DOF 並びの鎖を `solve_ik` へ誤用した際に `GeometricStructureChangedError` で即検出できる。
        """
        chain = self.model.get_kinematic_chain(base_link, tip_link, tip_offset)
        return dataclasses.replace(chain, source=(id(self), self._structure_version))

    def create_ik_solver(
        self, name: IKSolverName | str, base_link: str, tip_link: str, config: IKSolverConfig | None = None, *, tip_offset: np.ndarray | None = None
    ) -> IKSolver:
        """登録名 (`"jacobian"` / `"ikpy"` / ユーザー登録名) から IK ソルバーを生成する。"""
        return create_ik_solver(name, self.get_kinematic_chain(base_link, tip_link, tip_offset), config)

    def world_to_link_frame(self, link: str, position, orientation=None) -> tuple[np.ndarray, np.ndarray | None]:
        """ワールド座標の目標姿勢を、現在の `link` 座標系基準へ変換する (IK ソルバーは鎖の base リンク基準で解くため)。"""
        inv = tf.invert(self._world[self.model.get_link_index(link)])
        pos = inv[:3, :3] @ np.asarray(position, dtype=float).reshape(3) + inv[:3, 3]
        if orientation is None:
            return pos, None
        return pos, tf.matrix_to_quat(inv[:3, :3] @ tf.quat_to_matrix(np.asarray(orientation, dtype=float).reshape(4)))

    def solve_ik(self, solver: IKSolver, target_position, target_orientation=None, *, apply: bool = True):
        """ワールド座標の目標へ手先を合わせる。初期値 (`seed`) は現在の関節値、`apply=True` で成功時に反映する。

        `IKResult.dof_positions` は常にラジアン (`angle_unit` に依存しない)。
        """
        self._ensure_current()
        chain = solver.chain
        if chain.source is not None and chain.source != (id(self), self._structure_version):
            raise GeometricStructureChangedError(
                "この IK ソルバーの鎖は、ロボットを作り直す前に作ったもの(DOF の並びが変わっている)。create_ik_solver で作り直す"
            )
        pos, quat = self.world_to_link_frame(chain.base_link, target_position, target_orientation)
        result = solver.solve(pos, quat, seed=self._q[chain.dof_indices])
        if apply and result.success:
            self._set_positions_rad(np.asarray(result.dof_positions, dtype=float), np.asarray(chain.dof_indices, dtype=int))
        return result

    # ==========================================================================
    # VariantSet 切り替え
    # ==========================================================================

    @property
    def root_prim_paths(self) -> list[str]:
        """ロボットの範囲の prim(既存ロボットから作ったとき)。"""
        return list(self._info.root_prim_paths) if self._info is not None else []

    def _require_source(self, what: str) -> UsdArticulationSource:
        if self._source is None:
            raise RuntimeError(f"{what} は、既存のロボットから作った GeometricRobot だけで使える(パラメータから組んだロボットは VariantSet を持たない)")
        return self._source

    def _variant_prims(self) -> list[Usd.Prim]:
        """ロボットの範囲(根の prim を含む)で、VariantSet を持つ prim。"""
        out = []
        for root in self.root_prim_paths:
            for prim in Usd.PrimRange(self._stage.GetPrimAtPath(root)):
                if prim.HasVariantSets():
                    out.append(prim)
        return out

    def get_variants(self) -> dict[str, dict[str, str]]:
        """{prim パス: {VariantSet 名: いまの選択}}。"""
        self._require_source("get_variants")
        out: dict[str, dict[str, str]] = {}
        for prim in self._variant_prims():
            sets = prim.GetVariantSets()
            out[str(prim.GetPath())] = {name: sets.GetVariantSelection(name) for name in sets.GetNames()}
        return out

    def get_variant_choices(self) -> dict[str, dict[str, list[str]]]:
        """{prim パス: {VariantSet 名: [選べる Variant 名, ...]}}。"""
        self._require_source("get_variant_choices")
        out: dict[str, dict[str, list[str]]] = {}
        for prim in self._variant_prims():
            sets = prim.GetVariantSets()
            out[str(prim.GetPath())] = {name: list(sets.GetVariantSet(name).GetVariantNames()) for name in sets.GetNames()}
        return out

    def _abs_prim_path(self, prim: str) -> str:
        if prim.startswith("/"):
            return prim.rstrip("/")
        roots = self.root_prim_paths
        return f"{roots[0]}/{prim}".rstrip("/") if prim else roots[0]

    def _resolve_variant_prim(self, prim: str | None, variant_set: str, patterns: Sequence[str] | None = None) -> Usd.Prim:
        """VariantSet を持つ prim を決める。prim を省略したら、候補(patterns に一致する範囲)の中で一意に決まるときだけ。"""
        if prim is None:
            candidates = [p for p in self._variant_prims() if p.GetVariantSets().HasVariantSet(variant_set)]
            if patterns is not None:
                candidates = [p for p in candidates if any(prim_matches(str(p.GetPath()), pat) for pat in patterns)]
            if len(candidates) == 1:
                return candidates[0]
            where = "" if patterns is None else f"(範囲 {list(patterns)})"
            if not candidates:
                raise ValueError(f"VariantSet {variant_set!r} を持つ prim が無い{where}。ある VariantSet: {self.get_variant_choices()}")
            raise ValueError(f"VariantSet {variant_set!r} を持つ prim が複数ある{where}。prim= で選ぶ: {[str(p.GetPath()) for p in candidates]}")
        path = self._abs_prim_path(prim)
        target = self._stage.GetPrimAtPath(path)
        if not target or not target.IsValid():
            raise ValueError(f"prim が無い: {path}")
        if not any(path == r or path.startswith(r + "/") for r in self.root_prim_paths):
            raise ValueError(f"prim {path} はロボットの範囲 {self.root_prim_paths} の外")
        if not target.GetVariantSets().HasVariantSet(variant_set):
            raise ValueError(f"prim {path} に VariantSet {variant_set!r} は無い。ある VariantSet: {list(target.GetVariantSets().GetNames())}")
        return target

    def set_variant(self, variant_set: str, variant: str, *, prim: str | None = None) -> None:
        """prim の VariantSet を variant に切り替え、ロボットを作り直す。選択は Session Layer に書く(元のファイルは変えない)。

        prim はロボットの prim からの相対パスか絶対パス。省略できるのは、その名前の VariantSet がロボットに 1 つだけのとき。
        """
        self.set_variants({(prim, variant_set): variant})

    def set_variants(self, selections: Mapping[tuple[str | None, str], str], *, _patterns: Sequence[str] | None = None) -> None:
        """複数の VariantSet をまとめて切り替え、作り直しは 1 回だけ行う。selections = {(prim, VariantSet 名): Variant 名}。

        すべて確かめてから書く(1 つでも不正なら何も変えずに ValueError)。
        """
        self._require_source("set_variants")
        self._ensure_current()
        resolved = []
        for (prim, variant_set), variant in selections.items():
            target = self._resolve_variant_prim(prim, variant_set, _patterns)
            names = list(target.GetVariantSets().GetVariantSet(variant_set).GetVariantNames())
            if variant not in names:
                raise ValueError(f"prim {target.GetPath()} の VariantSet {variant_set!r} に Variant {variant!r} は無い。選べるもの: {names}")
            resolved.append((target, variant_set, variant))
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            for target, variant_set, variant in resolved:
                target.GetVariantSets().GetVariantSet(variant_set).SetVariantSelection(variant)
        self._maybe_changed = True
        self._ensure_current(reason="set_variants")

    # ==========================================================================
    # USD 構造変化の検知と再構築
    # ==========================================================================

    def add_structure_listener(self, callback: StructureListener) -> None:
        """再構築の直後に `callback(再構築前の DOF 名一覧, 再構築後の DOF 名一覧)` が呼ばれる。"""
        self._structure_listeners.append(callback)

    def remove_structure_listener(self, callback: StructureListener) -> None:
        if callback in self._structure_listeners:
            self._structure_listeners.remove(callback)

    def _on_objects_changed(self, notice, _sender) -> None:
        """USD 変更通知コールバック。ロボット範囲の resync やコライダー属性変化を検知してフラグを立てる。

        通知コールバック内では USD への書き込み (再構築・Sublayer 更新) が安全でないため、
        ここではキャッシュ破棄と dirty フラグ設定だけを行い、実際の再構築は次の `_on_update`
        または次の API 呼び出し (`_ensure_current` / `GeometricScene._ensure_finalized`) で実行する。
        """
        if self._closed or is_authoring():
            return
        roots = self.root_prim_paths
        watch_paths = list(dict.fromkeys([*roots, *self._prim_paths]))
        collider_dirty = False
        for path in notice.GetResyncedPaths():
            p = str(path)
            p_norm = p.rstrip("/")
            if self._source is not None and not self._maybe_changed:
                if any(p == "/" or p == r or p.startswith(r + "/") or r.startswith(p_norm + "/") for r in roots):
                    self._maybe_changed = True
            if not collider_dirty and watch_paths:
                if any(p == "/" or p == w or p.startswith(w + "/") or w.startswith(p_norm + "/") for w in watch_paths):
                    collider_dirty = True
        if not collider_dirty and watch_paths:
            for path in notice.GetChangedInfoOnlyPaths():
                if path.IsPropertyPath() and path.name in _COLLIDER_ATTRS:
                    prim_p = str(path.GetPrimPath())
                    if any(prim_p == w or prim_p.startswith(w + "/") for w in watch_paths):
                        collider_dirty = True
                        break
        if collider_dirty:
            self._invalidate_colliders()

    def _structure_signature(self) -> tuple:
        """構造の指紋: Variant 選択・`Joint` prim 一覧・`RigidBodyAPI` prim 一覧のタプル。"""
        variants, joints, bodies = [], [], []
        for root in self.root_prim_paths:
            for prim in Usd.PrimRange(self._stage.GetPrimAtPath(root)):
                if prim.HasVariantSets():
                    sets = prim.GetVariantSets()
                    variants.extend((str(prim.GetPath()), n, sets.GetVariantSelection(n)) for n in sets.GetNames())
                if prim.IsA(UsdPhysics.Joint):
                    joints.append(str(prim.GetPath()))
                elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    bodies.append(str(prim.GetPath()))
        return tuple(variants), tuple(joints), tuple(bodies)

    def _ensure_current(self, reason: str = "external") -> None:
        """構造変化フラグが立っていれば指紋を比較し、実際に変化していれば `_rebuild` を実行する。"""
        if not self._maybe_changed or self._source is None or self._closed:
            return
        self._maybe_changed = False
        signature = self._structure_signature()
        if signature != self._signature:
            self._rebuild(reason, signature)

    def rebuild(self) -> None:
        """USD を強制的に読み直して再構築する (Joint の角度オフセット編集など指紋に出ない変更の後に呼ぶ)。"""
        self._require_source("rebuild")
        self._rebuild("manual", self._structure_signature())

    def _rebuild(self, reason: str, signature: tuple) -> None:
        """USD からモデルを再読込し、Sublayer・ライター・Newton シーンを整合した状態へ入れ替える。

        設計上の重要点:
            - `keep` に旧関節値を DOF 名で退避し、Variant 切替後も共通軸の姿勢を引き継ぐ。
            - `self._layer.update_entry(...)` で差分 prim のみを更新し、Sublayer 全体の付け直しを避ける。
            - `self._structure_version` を 1 増やして古い `KinematicChain` / `IKSolver` / `GeometricRobotView` を
              確実に失効させ、`self._scene.mark_dirty()` で Newton モデルも次のクエリ前に自動再構築させる。
        """
        timings: dict[str, float] = {}
        t0 = time.perf_counter()
        old_names = list(self._model.dof_names)
        keep = (dict(zip(old_names, self._q.tolist())), dict(zip(old_names, self._default_q.tolist())))
        model, info = read_articulation(self._stage, self._source)  # type: ignore[arg-type]
        for message in info.warnings:
            _log_warn(message)
        t1 = time.perf_counter()
        self._setup_structure(model, info.link_prim_paths, info, keep=keep)
        t2 = time.perf_counter()
        counts = self._layer.update_entry(self._layer_token, **self._entry_kwargs())
        t3 = time.perf_counter()
        self._usd_writer.close()
        self._usd_writer = GeometricUsdWriter(self._layer, self._prim_paths)
        if self._fabric_writer is not None:
            self._fabric_writer.close()
        self._fabric_writer = None
        if self._gpu_fabric_writer is not None:
            self._gpu_fabric_writer.close()
        self._gpu_fabric_writer = None
        self._signature = signature
        self._structure_version += 1
        if self._scene is not None:
            self._scene.mark_dirty()
        t_usd = t_fabric = t3
        if self._layer.is_applied() and self._prim_paths:
            # 新しく増えた prim も含めて USD / PhysX 側へ現在の姿勢を 1 回確定させる
            self._usd_writer.write(self._solver, self._prim_world())
            self._usd_dirty = False
            self._usd_sync_pending = True
            self._repush_frames = 3
            t_usd = time.perf_counter()
            if self._write_target in ("fabric", "both"):
                fabric = self._fabric()
                t_fabric = time.perf_counter()
                if fabric is not None:
                    fabric.write(self._solver, self._prim_world())
        t4 = time.perf_counter()
        timings.update(
            read_usd_ms=round((t1 - t0) * 1e3, 3),
            setup_ms=round((t2 - t1) * 1e3, 3),
            layer_ms=round((t3 - t2) * 1e3, 3),
            write_ms=round((t4 - t3) * 1e3, 3),
            write_usd_ms=round((max(t_usd, t3) - t3) * 1e3, 3),
            fabric_writer_create_ms=round((max(t_fabric, t_usd) - max(t_usd, t3)) * 1e3, 3) if t_fabric > t3 else 0.0,
            read_usd_breakdown_ms=dict(info.timings_ms),
            total_ms=round((t4 - t0) * 1e3, 3),
            layer_removed_prims=counts["removed"],
            layer_added_prims=counts["added"],
        )
        self.last_rebuild_timings = timings
        new_names = list(model.dof_names)
        for callback in list(self._structure_listeners):
            callback(old_names, new_names)

    # ==========================================================================
    # コマンド変換 (外部指令 <-> DOF 値)
    # ==========================================================================

    def set_command_converter(self, name: str, converter: GeometricCommandConverter, *, when: Mapping[tuple[str, str], str] | None = None) -> None:
        """コマンド変換を名前で登録する (同名があれば置き換える)。

        `when = {(prim, VariantSet 名): Variant 名}` を渡すと、すべての Variant 条件が一致するときだけ有効化される
        (再構築後に再評価される)。`prim` はロボット prim からの相対パスまたは絶対パス。
        """
        condition = None
        if when is not None:
            self._require_source("when=")
            condition = {(self._abs_prim_path(prim), variant_set): variant for (prim, variant_set), variant in when.items()}
        self._converters[name] = (converter, condition)
        self._active_converters = None

    def remove_command_converter(self, name: str) -> None:
        self._converters.pop(name, None)
        self._active_converters = None

    def active_command_converters(self) -> list[str]:
        """現在の Variant 構成で有効なコマンド変換の名前一覧。"""
        return [name for name, *_ in self._active()]

    def _active(self) -> list[tuple[str, GeometricCommandConverter, np.ndarray, np.ndarray]]:
        """`(名前, 変換器, 書込 DOF 番号配列, ラジアン換算係数配列)` のリスト。構造か登録が変わるまでキャッシュする。"""
        self._ensure_current()
        if self._active_converters is None:
            variants = None
            active = []
            for name, (converter, condition) in self._converters.items():
                if condition:
                    if variants is None:
                        variants = self.get_variants()
                    if any(variants.get(prim, {}).get(vset) != v for (prim, vset), v in condition.items()):
                        continue
                missing = [d for d in converter.output_dofs if d not in self._model.dof_names]
                if missing:
                    raise KeyError(f"コマンド変換 {name!r} の出力の DOF がロボットに無い: {missing}")
                idx = np.array([self._model.get_dof_index(d) for d in converter.output_dofs], dtype=int)
                scale = np.where(self._revolute[idx], np.pi / 180.0, 1.0) if converter.output_angle_unit == "deg" else np.ones(len(idx))
                active.append((name, converter, idx, scale))
            self._active_converters = active
        return self._active_converters

    def apply_command(self, command: Mapping[str, float], *, unit: AngleUnit | None = None) -> None:
        """外部コマンド辞書を DOF 値へ変換し、`set_dof_positions` 相当を 1 回だけ適用する。

        有効な変換の入力がすべて `command` に揃っていればその変換を適用し、残りのキーは DOF 名の直接指定
        (`unit` 単位) として扱う。未知のキーは `KeyError`、同一 DOF への重複書き込みは `ValueError`。
        """
        used: set[str] = set()
        idx_parts, value_parts = [], []
        for name, converter, idx, scale in self._active():
            if all(k in command for k in converter.input_names):
                x = np.fromiter((command[k] for k in converter.input_names), dtype=float, count=len(converter.input_names))
                idx_parts.append(idx)
                value_parts.append(converter.to_dof_positions(x) * scale)
                used.update(converter.input_names)
        direct = [k for k in command if k not in used]
        if direct:
            unknown = [k for k in direct if k not in self._model.dof_names]
            if unknown:
                raise KeyError(f"コマンドの名前 {unknown} は、使われている変換の入力でも DOF 名でもない")
            idx = np.array([self._model.get_dof_index(k) for k in direct], dtype=int)
            values = np.fromiter((command[k] for k in direct), dtype=float, count=len(direct))
            scale = self._unit_scale(idx, unit)
            idx_parts.append(idx)
            value_parts.append(values if scale is None else values * scale)
        if not idx_parts:
            return
        idx_all = np.concatenate(idx_parts)
        if len(np.unique(idx_all)) != len(idx_all):
            raise ValueError("同じ DOF に、複数の変換(または変換と直接の値)から書こうとしている")
        self._set_positions_rad(np.concatenate(value_parts), idx_all)

    def read_command(self) -> dict[str, float]:
        """有効な変換のうち逆変換を実装しているものを使い、現在の関節値をコマンド表現へ戻す。"""
        out: dict[str, float] = {}
        for _name, converter, idx, scale in self._active():
            try:
                x = converter.from_dof_positions(self._q[idx] / scale)
            except NotImplementedError:
                continue
            out.update(zip(converter.input_names, np.asarray(x, dtype=float).reshape(-1).tolist()))
        return out

    # ==========================================================================
    # 干渉チェック・クリアランス・Newton バックエンド連携
    # ==========================================================================

    @property
    def collision_backend(self) -> CollisionBackend:
        """干渉チェックのバックエンド (`"newton"` または `"physx"`)。"""
        return self._collision_backend

    @collision_backend.setter
    def collision_backend(self, value: CollisionBackend) -> None:
        if value not in get_args(CollisionBackend):
            raise ValueError(f"collision_backend は {get_args(CollisionBackend)} のどれか: {value!r}")
        self._collision_backend = value

    @property
    def fk_backend(self) -> FkBackend:
        """順運動学のバックエンド (`"auto"` / `"numpy"` / `"newton"`)。"""
        return self._fk_backend

    @fk_backend.setter
    def fk_backend(self, value: FkBackend) -> None:
        if value not in get_args(FkBackend):
            raise ValueError(f"fk_backend は {get_args(FkBackend)} のどれか: {value!r}")
        self._fk_backend = value

    @property
    def convex_meshes(self) -> bool:
        """Newton 干渉判定で `UsdGeom.Mesh` を単一凸包へ変換するか (`True`=凸包 / `False`=元の三角形メッシュ)。"""
        return self._convex_meshes

    @convex_meshes.setter
    def convex_meshes(self, value: bool) -> None:
        flag = bool(value)
        if flag == self._convex_meshes:
            return
        self._convex_meshes = flag
        if self._scene is not None:
            self._scene.convex_meshes = flag

    @property
    def debug_draw_colliders(self) -> bool:
        """デバッグ用 `DebugDraw` コライダー可視化 (通常=シアン / 干渉中=赤 / 接触点=オレンジ) の有効スイッチ。"""
        return self._debug_draw_colliders

    @debug_draw_colliders.setter
    def debug_draw_colliders(self, value: bool) -> None:
        flag = bool(value)
        if flag == self._debug_draw_colliders:
            return
        self._debug_draw_colliders = flag
        if flag:
            self.draw_colliders(self._last_collision_pairs)
        else:
            self.clear_collider_debug_draw()

    def draw_colliders(
        self,
        pairs: Sequence | None = None,
        *,
        include_environment: bool = True,
        clear: bool = True,
        sync_robots: bool = True,
        sync_env: bool = True,
    ) -> int:
        """現在の Newton 内部コライダー形状と接触点を `DebugDraw` で 1 回描画する (デバッグ用途)。

        配色: 通常=シアン / 干渉中=赤 / 接触点とめり込み線分=オレンジ。
        `pairs` 省略時は直近の `check_collisions()` / `compute_clearances()` の結果を再利用する。
        """
        use_pairs = self._last_collision_pairs if pairs is None else pairs
        return self.scene.draw_colliders(
            use_pairs,
            robot=self,
            include_environment=include_environment,
            clear=clear,
            sync_robots=sync_robots,
            sync_env=sync_env,
        )

    def clear_collider_debug_draw(self) -> None:
        """`DebugDraw` で描画したコライダー線分と接触点マーカーを消去する。"""
        if self._scene is not None:
            self._scene.clear_debug_draw()
            return
        dd = GeometricScene._get_debug_draw_interface()
        if dd is not None:
            dd.clear_lines()
            dd.clear_points()

    @property
    def scene(self) -> GeometricScene:
        """このロボットが属する自前コリジョン空間 (`GeometricScene`)。未指定なら必要時に専用インスタンスを自動生成する。"""
        self._ensure_current()
        if self._scene is None:
            self._scene = GeometricScene(self._stage, device=self._newton_device, convex_meshes=self._convex_meshes)
            self._owns_scene = True
            self._scene.add_robot(self)
        return self._scene

    @property
    def collision_filter(self) -> GeometricCollisionFilter:
        """無視するリンクペアのフィルタ (`add_pair` / `add_pattern` / `add_rule` / `remove_pair` / `remove_by_reason`)。"""
        self._ensure_current()
        return self._filter

    @property
    def collider_version(self) -> int:
        """コライダー構成が破棄または更新されるたびに 1 増える番号 (`GeometricScene` / `GeometricRobotView` の同期用)。"""
        return self._collider_version

    @property
    def collider_paths(self) -> dict[str, list[str]]:
        """`リンク名 -> コライダー形状の prim パス一覧`。初回アクセス時に USD から探索する。"""
        self._ensure_current()
        if self._colliders is None:
            from ..collision.types import find_colliders

            self._colliders = {name: find_colliders(self._stage, path, self._prim_paths) for name, path in self._link_path_map.items()}
        return self._colliders

    def _invalidate_colliders(self) -> None:
        """コライダー探索キャッシュと紐づく Newton / PhysX シーンを破棄し、`collider_version` を進める。"""
        self._colliders = None
        self._collider_version += 1
        if self._scene is not None:
            self._scene.mark_dirty()
        if self._checker is not None:
            self._checker._needs_reload = True

    def refresh_colliders(self) -> None:
        """キャッシュ済みのコライダー一覧を破棄し、紐づく `GeometricScene` / `GeometricCollisionChecker` に再構築マークを付ける。"""
        self._invalidate_colliders()

    def reload_colliders(self) -> dict[str, list[str]]:
        """コライダー探索キャッシュと Newton / PhysX シーンを破棄し、最新 USD 構成から読み直した一覧を返す。

        カスタムコライダー用 `.usda` を手動で `subLayerPaths` へ追加・Mute/Unmute した直後や、
        `CollisionAPI` / 形状寸法を直接編集した直後に明示的に再同期したいときに呼ぶ。
        """
        self._invalidate_colliders()
        return self.collider_paths

    @property
    def collider_layers(self) -> list[str]:
        """`attach_collider_layer()` で現在取り付けているカスタムコライダー Sublayer の識別子一覧。"""
        return [ident for ident, _ in self._collider_layers]

    def _host_layer_for_target(self, target: LayerTarget) -> Sdf.Layer:
        if target not in get_args(LayerTarget):
            raise ValueError(f"target は {get_args(LayerTarget)} のどれか: {target!r}")
        return self._stage.GetSessionLayer() if target == "session" else self._stage.GetRootLayer()

    def _resolve_collider_layer_id(self, layer_or_path: str | os.PathLike[str] | Sdf.Layer, *, must_open: bool = False) -> str:
        """`Sdf.Layer` / レイヤー識別子 / ファイルパスを OpenUSD の `Sdf.Layer.identifier` へ正規化する。"""
        if isinstance(layer_or_path, Sdf.Layer):
            return layer_or_path.identifier
        raw = os.fspath(layer_or_path)
        for ident, _ in self._collider_layers:
            if ident == raw:
                return ident
        found = Sdf.Layer.Find(raw)
        if found is not None:
            return found.identifier
        if must_open:
            opened = Sdf.Layer.FindOrOpen(raw)
            if opened is None:
                raise FileNotFoundError(f"コライダー Sublayer を開けない: {raw!r}")
            return opened.identifier
        norm_raw = os.path.normcase(os.path.abspath(raw))
        for ident, _ in self._collider_layers:
            if not ident.startswith("anon:") and os.path.normcase(os.path.abspath(ident)) == norm_raw:
                return ident
        return raw

    @staticmethod
    def _find_sublayer_entry(host: Sdf.Layer, ident: str) -> str | None:
        """`host.subLayerPaths` の中から `ident` に一致する登録文字列を返す (無ければ `None`)。"""
        norm_ident = None if ident.startswith("anon:") else os.path.normcase(os.path.abspath(ident))
        for entry in host.subLayerPaths:
            if entry == ident:
                return entry
            resolved = Sdf.Layer.Find(entry)
            if resolved is not None and resolved.identifier == ident:
                return entry
            if norm_ident is not None and not entry.startswith("anon:"):
                if os.path.normcase(os.path.abspath(entry)) == norm_ident:
                    return entry
        return None

    def attach_collider_layer(
        self,
        layer_or_path: str | os.PathLike[str] | Sdf.Layer,
        *,
        target: LayerTarget = "session",
    ) -> str:
        """カスタムコライダー定義 `.usda` (または `Sdf.Layer`) を Sublayer として取り付け、即座にコライダーを再読込する。

        `GeometricLayer` の直下 (Session Layer で `GeometricLayer` が先頭にある場合は index 1、無ければ index 0) へ
        差し込むことで、`GeometricLayer` が書く `xformOp:transform:geometric` や `rigidBodyEnabled=false` の優先順位を
        維持しつつ、元 USD (Root Layer) の `CollisionAPI` 無効化とカスタムコライダー追加を上書き適用する。
        元のロボット USD ファイルは変更しない。戻り値は取り付けたレイヤーの識別子 (`Sdf.Layer.identifier`)。
        """
        ident = self._resolve_collider_layer_id(layer_or_path, must_open=True)
        host = self._host_layer_for_target(target)
        if self._find_sublayer_entry(host, ident) is None:
            geom_layer = self._layer.sdf_layer
            insert_idx = 0
            if geom_layer is not None:
                sub_paths = list(host.subLayerPaths)
                if geom_layer.identifier in sub_paths:
                    insert_idx = sub_paths.index(geom_layer.identifier) + 1
            host.subLayerPaths.insert(insert_idx, ident)
        if self._stage.IsLayerMuted(ident):
            self._stage.UnmuteLayer(ident)
        if not any(i == ident for i, _ in self._collider_layers):
            self._collider_layers.append((ident, target))
        self.reload_colliders()
        return ident

    def set_collider_layer_enabled(
        self,
        layer_or_path: str | os.PathLike[str] | Sdf.Layer,
        enabled: bool,
    ) -> None:
        """取り付け済みのカスタムコライダー Sublayer を USD の Mute / Unmute で切り替え、即座にコライダーを再読込する。

        `enabled=False` にすると Sublayer を Mute して元のコライダー構成へ戻し、
        `enabled=True` にすると Unmute してカスタムコライダーを再び有効化する (0 フレーム遅延で Newton へ反映)。
        """
        ident = self._resolve_collider_layer_id(layer_or_path, must_open=False)
        if enabled:
            self._stage.UnmuteLayer(ident)
        else:
            self._stage.MuteLayer(ident)
        self.reload_colliders()

    def detach_collider_layer(
        self,
        layer_or_path: str | os.PathLike[str] | Sdf.Layer | None = None,
    ) -> bool:
        """カスタムコライダー Sublayer をステージから外し、元のコライダー構成へ戻す。

        `layer_or_path=None` を渡すと、`attach_collider_layer()` で取り付けたすべてのカスタムコライダー Sublayer を外す。
        1 つ以上外した場合は `True`、該当が無ければ `False` を返す。
        """
        if layer_or_path is None:
            targets = list(self._collider_layers)
        else:
            ident = self._resolve_collider_layer_id(layer_or_path, must_open=False)
            matched = [(i, t) for i, t in self._collider_layers if i == ident]
            targets = matched if matched else [(ident, "session"), (ident, "root")]
        removed_any = False
        removed_ids: set[str] = set()
        for ident, target in targets:
            host = self._host_layer_for_target(target)
            entry = self._find_sublayer_entry(host, ident)
            if entry is not None:
                host.subLayerPaths.remove(entry)
                removed_any = True
                removed_ids.add(ident)
            if self._stage.IsLayerMuted(ident):
                self._stage.UnmuteLayer(ident)
        if removed_ids:
            self._collider_layers = [(i, t) for i, t in self._collider_layers if i not in removed_ids]
        elif layer_or_path is None:
            self._collider_layers.clear()
        if removed_any:
            self.reload_colliders()
        return removed_any

    @property
    def link_set(self):
        """このロボットの全リンク集合 (`GeometricLinkSet`)。"""
        from ..collision.types import GeometricLinkSet

        return GeometricLinkSet(self, self.model.link_names, self.name)

    @property
    def collision_checker(self):
        """`collision_backend="physx"` 用の `GeometricCollisionChecker` (遅延生成)。"""
        if self._checker is None:
            from ..collision.physx import GeometricCollisionChecker

            self._checker = GeometricCollisionChecker(self._stage)
        return self._checker

    def _default_scope(self) -> str:
        """元の USD Articulation で自己干渉が無効 (`enabledSelfCollisions=false`) なら `"environment"`、それ以外は `"all"`。"""
        if self._info is not None and self._info.self_collision_enabled is False:
            return "environment"
        return "all"

    def check_collisions(
        self,
        scope=None,
        *,
        clearance: float = 0.0,
        extra_filter=None,
        ignore_prim_paths: Sequence[str] = (),
        backend: CollisionBackend | None = None,
    ) -> list:
        """現在の姿勢で重なっている (または `clearance` 以内の) `GeometricCollisionPair` 一覧を返す。

        引数:
            scope     : `"within"` (このロボット内のリンク同士) / `"environment"` (環境コライダーとの間) / `"all"` (両方)。
                        省略時は `_default_scope()` の結果を使う。
            clearance : 0 より大きい値を渡すと、その距離 [m] 以内の組も返す (`collision_backend="newton"` のみ)。
            backend   : 省略時は `self.collision_backend` (既定 `"newton"`: `flush_to_usd()` 不要で 0 フレーム遅延)。
                        `"physx"` を選んだ場合は `GeometricCollisionChecker._prepare()` 内で自動的に `flush_to_usd()` が呼ばれる。
        """
        pairs = _check(self, self.link_set, scope, extra_filter, ignore_prim_paths, clearance=clearance, backend=backend)
        self._last_collision_pairs = pairs
        if self._debug_draw_colliders:
            self.scene.draw_colliders(pairs, robot=self, sync_robots=False, sync_env=False)
        return pairs

    def is_in_collision(self, scope=None, **kwargs) -> bool:
        return bool(self.check_collisions(scope, **kwargs))

    def touching_pairs(self, *, clearance: float = 0.0, backend: CollisionBackend | None = None) -> list:
        """`collision_filter` を使わずに現在重なっている組をすべて返す (`collision_backend="newton"` なら `flush_to_usd()` 不要)。"""
        self._ensure_current()
        be = self._collision_backend if backend is None else backend
        if be == "newton":
            return self.scene.touching_pairs(self.link_set, clearance=clearance)
        if clearance > 0.0:
            raise ValueError('clearance > 0 は collision_backend="newton" でのみ使える')
        return self.collision_checker.touching_pairs(self.link_set)

    def compute_clearances(
        self,
        scope=None,
        *,
        max_distance: float = 0.05,
        extra_filter=None,
        ignore_prim_paths: Sequence[str] = (),
    ) -> list:
        """`max_distance` [m] 以内の組と符号付き距離 (`separation`) を近い順 (負=めり込みが先頭) で返す (`GeometricScene` 使用)。"""
        self._ensure_current()
        pairs = self.scene.compute_clearances(
            self,
            link_set=self.link_set,
            scope=scope,
            max_distance=max_distance,
            extra_filter=extra_filter,
            ignore_prim_paths=ignore_prim_paths,
        )
        self._last_collision_pairs = pairs
        if self._debug_draw_colliders:
            self.scene.draw_colliders(pairs, robot=self, sync_robots=False, sync_env=False)
        return pairs

    def ignore_touching_pairs(
        self,
        reason: str | None = None,
        *,
        samples: int = 0,
        seed: int = 0,
        always_ratio: float = 1.0,
    ) -> list[tuple[str, str]]:
        """ロボット内で重なっているリンク組を `collision_filter` へ追加し、追加した `(link_a, link_b)` 一覧を返す。

        2 つの動作モード:
            - `samples == 0` (既定): **現在の 1 姿勢だけ**を検査し、その姿勢で重なっている組をすべて
              `reason` (既定 `"touching_at_setup"`) で除外登録する。現在の 1 姿勢に偶然の自己干渉が含まれると
              その組も除外されるため、干渉のない基準姿勢で呼ぶこと。
            - `samples > 0`: 現在の `self._q` を退避し、可動範囲内の `samples` 個のランダム姿勢
              (無制限の回転関節は `[-pi, pi]`、無制限の直動関節は `[-0.5, 0.5]`、mimic 解決済み) で
              自己干渉を評価し、`always_ratio` (既定 `1.0` = 全サンプル) 以上の割合で重なり続けた組だけを
              `reason` (既定 `"always_touching"`) で除外登録してから元の姿勢へ戻す。
              `collision_backend="newton"` では USD / Fabric へ一切書き出さず、`GeometricScene` の
              メモリ内だけでサンプリングする。

        解除は `collision_filter.remove_by_reason(reason)` で行える。
        """
        self._ensure_current()
        if samples < 0:
            raise ValueError(f"samples は 0 以上: {samples}")
        if not (0.0 < always_ratio <= 1.0):
            raise ValueError(f"always_ratio は (0.0, 1.0] の範囲: {always_ratio}")

        tag = reason if reason is not None else ("touching_at_setup" if samples == 0 else "always_touching")

        if samples == 0:
            pairs = (
                self.scene.check_within(self.link_set)
                if self._collision_backend == "newton"
                else self.collision_checker.check_within(self.link_set)
            )
            added: list[tuple[str, str]] = []
            for pair in pairs:
                self._filter.add_pair(pair.link_a, pair.link_b, reason=tag)
                added.append((pair.link_a, pair.link_b))
            return added

        # 多姿勢サンプリング (`samples > 0`): 元の関節値を退避し、終了時に必ず復元する
        saved_q = self._q.copy()
        num_dofs = self._model.num_dofs
        rng = np.random.default_rng(seed)
        low = self._lower.copy()
        high = self._upper.copy()
        for i in range(num_dofs):
            if not np.isfinite(low[i]):
                low[i] = -np.pi if self._revolute[i] else -0.5
            if not np.isfinite(high[i]):
                high[i] = np.pi if self._revolute[i] else 0.5
            if high[i] < low[i]:
                low[i], high[i] = high[i], low[i]

        counts: dict[tuple[str, str], int] = {}
        try:
            use_newton = self._collision_backend == "newton"
            scene = self.scene if use_newton else None
            for _ in range(samples):
                q_sample = rng.uniform(low, high) if num_dofs > 0 else np.zeros(0, dtype=float)
                q_sample = self._model.resolve_mimic(q_sample)
                if use_newton and scene is not None:
                    # `GeometricScene` のメモリ内のみで FK・衝突判定し、USD / Fabric は書き換えない
                    self._q = q_sample
                    pairs = scene.check_within(self.link_set)
                else:
                    self._set_positions_rad(q_sample, np.arange(num_dofs, dtype=int))
                    pairs = self.collision_checker.check_within(self.link_set)
                seen_in_sample: set[tuple[str, str]] = set()
                for pair in pairs:
                    key = (pair.link_a, pair.link_b) if pair.link_a <= pair.link_b else (pair.link_b, pair.link_a)
                    if key not in seen_in_sample:
                        seen_in_sample.add(key)
                        counts[key] = counts.get(key, 0) + 1
        finally:
            if self._collision_backend == "newton":
                self._q = saved_q
                if self._scene is not None and self._scene._model is not None:
                    self._scene.sync_robots()
            else:
                self._set_positions_rad(saved_q, np.arange(num_dofs, dtype=int))

        min_hits = int(np.ceil(always_ratio * samples - 1e-12))
        added = []
        for a, b in sorted(counts):
            if counts[(a, b)] >= min_hits:
                self._filter.add_pair(a, b, reason=tag)
                added.append((a, b))
        return added

    # ==========================================================================
    # 構成要素 (IsaacRobotAPI 自動検出 / components= / attach() による部分ビュー)
    # ==========================================================================

    @property
    def component_names(self) -> list[str]:
        return list(self.model.components)

    @property
    def components(self) -> dict[str, "GeometricRobotComponent"]:
        """ロボット生成・再構築時に自動生成された全構成要素の辞書 `{名前: GeometricRobotComponent}`。"""
        self._ensure_current()
        return dict(self._components)

    def get_component(self, name: str) -> "GeometricRobotComponent":
        """結合ロボット・`IsaacRobotAPI` サブロボットの構成要素 (`GeometricRobot` と状態を共有する部分ロボットハンドル) を取得する。"""
        if name not in self.model.components:
            raise KeyError(f"構成要素 {name!r} は無い。ある名前: {self.component_names}")
        if name not in self._components:
            self._components[name] = GeometricRobotComponent(self, name)
        return self._components[name]

    # ==========================================================================
    # 姿勢の計算・書き込み・同期・ライフサイクル
    # ==========================================================================

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
            except Exception as exc:  # Fabric が使えない構成 (usdrt 不在、Kit 外の独立ステージなど) は USD へフォールバック
                self._fabric_failed = True
                _log_warn(f"Fabric に書けないので USD に書く: {exc}")
        return self._fabric_writer

    def _prim_world(self) -> np.ndarray:
        return self._world[self._prim_link_indices]

    def _gpu_fabric(self) -> GeometricGpuFabricWriter | None:
        if not self._use_gpu_fabric_writer or self._gpu_fabric_failed or not self._prim_paths:
            return None
        if self._gpu_fabric_writer is None:
            try:
                sc = self.scene
                body_indices = sc.get_robot_body_indices(self)[self._prim_link_indices]
                self._gpu_fabric_writer = GeometricGpuFabricWriter(
                    self._stage,
                    self._prim_paths,
                    body_indices,
                    world_scales=self._solver.world_scales,
                    device=sc.device,
                )
            except Exception as exc:
                self._gpu_fabric_failed = True
                _log_warn(f"GPU Fabric 直接書き込みを使えないので通常 Fabric ライターに切替: {exc}")
        return self._gpu_fabric_writer

    def _use_newton_fk(self) -> bool:
        """現在の `fk_backend`・DOF 数・CUDA 可否から、FK に `GeometricScene` (Newton) を使うか判定する。"""
        if self._fk_backend == "newton":
            return True
        if self._fk_backend == "auto" and self._model.num_dofs >= _AUTO_FK_MIN_DOFS:
            try:
                import warp as wp

                return bool(wp.is_cuda_available())
            except Exception:
                return False
        return False

    def _compute_world(self) -> np.ndarray:
        """現在の `self._q` (mimic 解決済み) と `self._base` から全リンクのワールド行列 `(L, 4, 4)` を計算する。"""
        if self._use_newton_fk():
            return self.scene.eval_fk(self, self._q, base_matrix=self._base, resolve_mimic=False)
        return self._base @ self._model.forward_kinematics(self._q, resolve_mimic=False)

    def _push(self) -> None:
        """FK を評価し、幾何モードが有効であれば `write_target` に従ってステージへ書き込む。

        なぜ `write_target="fabric"` のまま `collision_backend="newton"` が 0 フレーム遅延で動くか:
            - `write_target="fabric"` では `self._usd_dirty = True` のみ立てて USD 書き込みを省略する。
            - Newton バックエンド (`GeometricScene`) は USD ではなく `self._q` / `self._base` を直接
              Warp バッファへ転送して `eval_fk` + `CollisionPipeline` を実行するため、`flush_to_usd()` 不要で
              0 フレーム遅延になる。
        """
        self._world = self._compute_world()
        if not self._layer.is_applied() or self._closed or not self._prim_paths:
            return
        world = self._prim_world()
        gpu_fab = self._gpu_fabric() if (self._use_newton_fk() and self._write_target in ("fabric", "both")) else None
        fabric = None if gpu_fab is not None else (self._fabric() if self._write_target in ("fabric", "both") else None)
        if self._write_target in ("usd", "both") or self._usd_sync_pending or (gpu_fab is None and fabric is None):
            self._usd_writer.write(self._solver, world)
            self._usd_dirty = False
            if (gpu_fab is not None or fabric is not None) and self._write_target in ("usd", "both"):
                self._repush_frames = 2
                self._usd_sync_pending = True
        else:
            self._usd_dirty = True
        if gpu_fab is not None:
            sc = self.scene
            gpu_fab.write_body_q(sc.newton_state.body_q, int(sc.newton_model.body_count))
        elif fabric is not None:
            fabric.write(self._solver, world)
        # Debug モードが OFF (既定) のときはこの 1 回の bool 判定だけで抜け、性能影響はゼロ
        if self._debug_draw_colliders:
            self.scene.draw_colliders(self._last_collision_pairs, robot=self, sync_robots=True, sync_env=False)

    def flush_to_usd(self) -> bool:
        """Fabric にのみ書いてある姿勢を USD (無名 Sublayer) へ書き出す。実際に書いたら `True`。

        `collision_backend="physx"` の `GeometricCollisionChecker._prepare()` から自動で呼ばれる
        (PhysX は USD 側の Transform を参照するため)。
        `collision_backend="newton"` の場合は通常呼ぶ必要がないが、USD を直接読む外部処理に最新姿勢を
        見せたいときに明示的に呼ぶ。
        """
        self._ensure_current()
        if self._flush_delegate is not None:
            return self._flush_delegate()
        if not self._usd_dirty or not self._layer.is_applied() or self._closed:
            return False
        self._usd_writer.write(self._solver, self._prim_world())
        self._usd_dirty = False
        # USD へ書くと次の `app.update()` で USD -> Fabric 同期が走り、直後に Fabric へ書いた新しい姿勢が
        # 古い USD 値で上書きされることがある。同フレーム内の直後更新を USD 側にも追随させつつ
        # 2 フレーム間は `_on_update` で Fabric へ再送して 1 フレーム戻りを完全に防ぐ。
        self._usd_sync_pending = True
        self._repush_frames = 2
        return True

    def refresh(self) -> None:
        """ロボット外の祖先 prim (根より上の `Xform` など) を動かした後に呼ぶ。親姿勢を再取得して書き直す。"""
        self._ensure_current()
        self._solver.refresh()
        self._push()

    def _on_layer_changed(self, applied: bool) -> None:
        if self._closed:
            return
        if applied:
            self._solver.refresh()
            self._world = self._base @ self._model.forward_kinematics(self._q, resolve_mimic=False)
            if self._prim_paths:
                self._usd_writer.write(self._solver, self._prim_world())  # 幾何モード開始時点の姿勢を USD / PhysX へ確定
            self._usd_dirty = False
            self._usd_sync_pending = True
            self._repush_frames = 3
            self._subscribe_update()
            if self._write_target in ("fabric", "both") and self._prim_paths:
                fabric = self._fabric()
                if fabric is not None:
                    fabric.write(self._solver, self._prim_world())
        else:
            self._usd_writer.close()
            self._usd_dirty = False

    def _subscribe_update(self) -> None:
        if self._update_subscription is not None:
            return
        try:
            import omni.kit.app

            self._update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update, name="myext.robot.geometric_newton.update"
            )
        except Exception:
            self._update_subscription = None

    def _on_update(self, _event) -> None:
        """Kit の毎フレーム更新コールバック。外部 Variant 変更の遅延再構築と Fabric 再送を行う。

        設計上の重要点:
            - `self._stage.GetPrimAtPath(...)` を `try/except` で保護する理由:
              ユーザーが `close()` を呼ばずにステージを切り替えて破棄した場合、expired な `Usd.Stage` への
              アクセスが `Boost.Python.ArgumentError` を投げる。ここで検知して購読を自動解除する。
            - `use_gpu_fabric_writer=True` かつ `fk_backend="newton"` のときは `GeometricFabricWriter` へ
              フォールバックせず `GeometricGpuFabricWriter.write_body_q` で再送する
              (`GeometricFabricWriter` を呼ぶと `omni:fabric:localMatrix` が再生成されてしまうため)。
        """
        if self._closed:
            return
        if self._prim_paths:
            try:
                valid = self._stage.GetPrimAtPath(self._prim_paths[0]).IsValid()
            except Exception:
                valid = False
            if not valid:
                # ステージ破棄またはロボット prim 削除済み (`close()` 未呼び出しのまま捨てられた)。購読を解除する
                self._update_subscription = None
                return
        if self._maybe_changed:  # Stage タブ等で Variant が変更されていれば次の update で自動再構築する
            try:
                self._ensure_current()
            except Exception as exc:  # update コールバック内では例外を伝播させず、次の API 呼び出しで再試行する
                self._maybe_changed = True
                _log_warn(f"作り直しに失敗した: {exc}")
        if self._repush_frames <= 0 or not self._layer.is_applied():
            self._usd_sync_pending = False
            return
        self._usd_sync_pending = False
        self._repush_frames -= 1
        if self._write_target in ("fabric", "both") and self._prim_paths:
            if self._use_gpu_fabric_writer and self._use_newton_fk():
                gpu_fab = self._gpu_fabric()
                if gpu_fab is not None:
                    try:
                        sc = self.scene
                        gpu_fab.write_body_q(sc.newton_state.body_q, int(sc.newton_model.body_count))
                        return
                    except Exception as exc:
                        self._repush_frames = 0
                        self._gpu_fabric_writer = None
                        _log_warn(f"GPU Fabric への書き直しに失敗した: {exc}")
            fabric = self._fabric()
            if fabric is not None:
                try:
                    fabric.write(self._solver, self._prim_world())
                except Exception as exc:  # update コールバック内では例外を外に出さない
                    self._repush_frames = 0
                    self._fabric_writer = None
                    _log_warn(f"Fabric への書き直しに失敗した: {exc}")

    def close(self, *, remove_layer: bool = True) -> None:
        """購読・ライター・Newton シーン登録を解放する。

        `remove_layer=True` (既定) なら `GeometricLayer` をステージから外して元の Articulation 構成へ戻す
        (複数ロボットで Sublayer を共有している場合は `remove_layer=False` を渡す)。
        専用に自動生成した `GeometricScene` (`self._owns_scene=True`) は `close()` し、
        共有された `GeometricScene` はこのロボットの登録のみ解除 (`remove_robot`) する。
        """
        if self._closed:
            return
        if remove_layer:
            if self._collider_layers:
                self.detach_collider_layer()
            self._layer.remove()
        self._layer.remove_listener(self._on_layer_changed)
        self._update_subscription = None
        if self._notice is not None:
            self._notice.Revoke()
            self._notice = None
        self._closed = True
        if self._fabric_writer is not None:
            self._fabric_writer.close()
        if self._gpu_fabric_writer is not None:
            self._gpu_fabric_writer.close()
            self._gpu_fabric_writer = None
        if self._debug_draw_colliders:
            self._debug_draw_colliders = False
            if self._scene is not None:
                self._scene.clear_debug_draw()
        if self._scene is not None:
            if self._owns_scene:
                self._scene.close()
            else:
                self._scene.remove_robot(self)
            self._scene = None


