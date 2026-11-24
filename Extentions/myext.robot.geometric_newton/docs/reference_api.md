# API・設定・ファイル構成リファレンス（`myext.robot.geometric_newton`）

この文書は、`myext.robot.geometric_newton` の公開クラス、メソッドシグネチャ、引数・既定値・戻り値型、`config/extension.toml` 設定キー、およびモジュール構成の仕様一覧（**Reference**）です。

- 設計背景・座標規約・編集箇所ガイドは [explanation_architecture.md](explanation_architecture.md) を参照してください。
- 目的別のコード例は [howto_recipes.md](howto_recipes.md) を参照してください。

---

## 1. 型エイリアスと定数

| シンボル | 定義モジュール | 値 / 型定義 |
|---|---|---|
| `CollisionBackend` | `newton_backend.py` | `Literal["newton", "physx"]` |
| `FkBackend` | `newton_backend.py` | `Literal["auto", "numpy", "newton"]` |
| `WriteTarget` | `writer.py` | `Literal["fabric", "usd", "both"]` |
| `CollisionMode` | `layer.py` | `Literal["static", "kinematic"]` |
| `LayerTarget` | `layer.py` | `Literal["session", "root"]` |
| `CollisionScope` | `collision.py` | `Literal["within", "environment", "all"]` |
| `PairKind` | `collision.py` | `Literal["within_component", "between_components", "between_robots", "environment"]` |
| `LimitPolicy` | `robot.py` | `Literal["clip", "warn", "error", "ignore"]` |
| `AngleUnit` | `model/command.py` | `Literal["rad", "deg"]` |
| `JointType` | `model/spec.py` | `Literal["revolute", "prismatic", "fixed"]` |
| `MovableJointType` | `model/sources.py` | `Literal["revolute", "prismatic"]` |
| `OrientationMode` | `model/ik.py` | `Literal["full", "none"]` |
| `IKSolverName` | `model/ik.py` | `Literal["jacobian", "ikpy"]` |
| `UnsupportedJointPolicy` | `usd_reader.py` | `Literal["error", "fixed", "ignore"]` |
| `WorldJointPolicy` | `usd_reader.py` | `Literal["ignore", "dof"]` |
| `SETTINGS_ROOT` | `layer.py` | `"/exts/myext.robot.geometric_newton"` |
| `LAYER_MARKER` | `layer.py` | `"myext.robot.geometric_newton"` |
| `XFORM_OP_NAME` | `layer.py` | `"xformOp:transform:geometric"` |
| `_AUTO_FK_MIN_DOFS` | `newton_backend.py` | `32`（単体 `fk_backend="auto"` の Newton 切替しきい値） |
| `_AUTO_FK_MIN_BATCH` | `newton_backend.py` | `16`（View `fk_backend="auto"` の Newton GPU 切替しきい値） |
| `FABRIC_BATCH_MIN_PRIMS` | `writer.py` | `24`（`GeometricFabricWriter` の `XformPrim` 一括切替しきい値） |

---

## 2. 単体ロボット API（`robot.py`）

### `GeometricRobot`

```python
GeometricRobot(
    stage: Usd.Stage,
    model: GeometricRobotModel,
    link_prim_paths: dict[str, str],
    *,
    info: UsdRobotInfo | None = None,
    source: UsdArticulationSource | None = None,
    layer: GeometricLayer | None = None,
    layer_options: GeometricLayerOptions | None = None,
    write_target: WriteTarget | None = None,
    limit_policy: LimitPolicy = "clip",
    angle_unit: AngleUnit | None = None,
    apply_layer: bool = True,
    collision_backend: CollisionBackend | None = None,
    fk_backend: FkBackend | None = None,
    scene: GeometricScene | None = None,
    newton_device: str | None = None,
    use_gpu_fabric_writer: bool = False,
)
```

#### ファクトリメソッド（`@classmethod`）

| メソッド | シグネチャ |
|---|---|
| `from_articulation` | `(cls, stage: Usd.Stage, source: UsdArticulationSource \| str, **kwargs) -> GeometricRobot` |
| `from_model` | `(cls, stage: Usd.Stage, model: GeometricRobotModel, prim_path: str, *, style=None, base_transform: np.ndarray \| None = None, **kwargs) -> GeometricRobot` |
| `from_dh` | `(cls, stage: Usd.Stage, params: DHParameters, prim_path: str, **kwargs) -> GeometricRobot` |
| `from_mdh` | `(cls, stage: Usd.Stage, params: MDHParameters, prim_path: str, **kwargs) -> GeometricRobot` |
| `from_screw` | `(cls, stage: Usd.Stage, params: ScrewParameters, prim_path: str, **kwargs) -> GeometricRobot` |
| `from_urdf` | `(cls, stage: Usd.Stage, source: UrdfSource \| str, prim_path: str, **kwargs) -> GeometricRobot` |

#### 主なプロパティ

| プロパティ | 型 | 概要 |
|---|---|---|
| `name`, `stage`, `model`, `layer`, `info`, `structure_version` | 各種 | ロボット名、ステージ、内部モデル、Sublayer、USD 読取情報、再構築世代番号 |
| `num_dofs`, `dof_names`, `dof_paths`, `dof_types` | `int` / `list` | DOF 数・名前・prim パス・種別 |
| `num_joints`, `joint_names`, `joint_paths`, `joint_types` | `int` / `list` | 全関節数・名前・prim パス・種別 |
| `num_links`, `link_names`, `link_paths`, `link_path_map` | `int` / `list` / `dict` | リンク数・名前・prim パス・リンク名から prim パスへの写像 |
| `joints`, `links` | `dict[str, GeometricJoint]` / `dict[str, GeometricLink]` | 関節・リンク単位のアクセサ辞書 |
| `angle_unit`（読み書き可） | `AngleUnit` | 回転関節の既定角度単位 |
| `write_target`（読み書き可） | `WriteTarget` | 姿勢書き込み先 |
| `collision_backend`（読み書き可） | `CollisionBackend` | 干渉判定バックエンド（既定 `"newton"`） |
| `fk_backend`（読み書き可） | `FkBackend` | 順運動学バックエンド（既定 `"auto"`） |
| `scene` | `GeometricScene` | 所属する自前コリジョン空間（未指定時は遅延生成） |
| `collision_filter`, `collider_paths`, `collider_layers`, `collider_version`, `link_set`, `collision_checker` | 各種 | フィルタ、コライダー一覧、取付中のカスタムコライダー Sublayer 識別子一覧、コライダー更新世代番号、全リンク集合、PhysX チェッカ |
| `component_names`, `root_prim_paths` | `list[str]` | 構成要素名一覧、ロボットのルート prim パス一覧 |

#### メソッド一覧

| カテゴリ | メソッド・シグネチャ |
|---|---|
| インデックス取得 | `get_dof_indices(names: str \| Sequence[str]) -> np.ndarray`<br>`get_joint_indices(names: str \| Sequence[str]) -> np.ndarray`<br>`get_link_indices(names: str \| Sequence[str]) -> np.ndarray` |
| 関節値・可動範囲 | `set_dof_positions(positions, *, dof_indices=None, unit: AngleUnit \| None = None) -> None`<br>`get_dof_positions(*, dof_indices=None, unit: AngleUnit \| None = None) -> np.ndarray`<br>`set_dof_position_targets(positions, *, dof_indices=None, unit: AngleUnit \| None = None) -> None`<br>`get_dof_position_targets(*, dof_indices=None, unit: AngleUnit \| None = None) -> np.ndarray`<br>`get_dof_limits(*, dof_indices=None, unit: AngleUnit \| None = None) -> tuple[np.ndarray, np.ndarray]`<br>`set_dof_limits(lower=None, upper=None, *, dof_indices=None, unit: AngleUnit \| None = None) -> None` |
| 姿勢・ヤコビアン | `get_world_poses() -> tuple[np.ndarray, np.ndarray]`<br>`set_world_poses(positions=None, orientations=None) -> None`<br>`get_local_poses() -> tuple[np.ndarray, np.ndarray]`<br>`set_local_poses(translations=None, orientations=None) -> None`<br>`get_link_world_matrices() -> np.ndarray`<br>`get_link_world_poses(*, link_indices=None) -> tuple[np.ndarray, np.ndarray]`<br>`get_link_local_poses(*, link_indices=None) -> tuple[np.ndarray, np.ndarray]`<br>`get_jacobian_matrices() -> np.ndarray` |
| 既定状態 | `set_default_state(positions=None, orientations=None, dof_positions=None, *, unit: AngleUnit \| None = None) -> None`<br>`get_default_state(*, unit: AngleUnit \| None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]`<br>`reset_to_default_state() -> None` |
| 逆運動学（IK） | `get_kinematic_chain(base_link: str, tip_link: str, tip_offset: np.ndarray \| None = None) -> KinematicChain`<br>`create_ik_solver(name: IKSolverName \| str, base_link: str, tip_link: str, config: IKSolverConfig \| None = None, *, tip_offset: np.ndarray \| None = None) -> IKSolver`<br>`world_to_link_frame(link: str, position, orientation=None) -> tuple[np.ndarray, np.ndarray \| None]`<br>`solve_ik(solver: IKSolver, target_position, target_orientation=None, *, apply: bool = True) -> IKResult` |
| VariantSet・再構築 | `get_variants() -> dict[str, dict[str, str]]`<br>`get_variant_choices() -> dict[str, dict[str, list[str]]]`<br>`set_variant(variant_set: str, variant: str, *, prim: str \| None = None) -> None`<br>`set_variants(selections: Mapping[tuple[str \| None, str], str], *, _patterns: Sequence[str] \| None = None) -> None`<br>`add_structure_listener(callback: StructureListener) -> None`<br>`remove_structure_listener(callback: StructureListener) -> None`<br>`rebuild() -> None` |
| コマンド変換 | `set_command_converter(name: str, converter: GeometricCommandConverter, *, when: Mapping[tuple[str, str], str] \| None = None) -> None`<br>`remove_command_converter(name: str) -> None`<br>`active_command_converters() -> list[str]`<br>`apply_command(command: Mapping[str, float], *, unit: AngleUnit \| None = None) -> None`<br>`read_command() -> dict[str, float]` |
| 干渉判定・クリアランス | `check_collisions(scope=None, *, clearance: float = 0.0, extra_filter=None, ignore_prim_paths: Sequence[str] = (), backend: CollisionBackend \| None = None) -> list[GeometricCollisionPair]`<br>`is_in_collision(scope=None, **kwargs) -> bool`<br>`touching_pairs(*, clearance: float = 0.0, backend: CollisionBackend \| None = None) -> list[GeometricCollisionPair]`<br>`compute_clearances(scope=None, *, max_distance: float = 0.05, extra_filter=None, ignore_prim_paths: Sequence[str] = ()) -> list[GeometricCollisionPair]`<br>`ignore_touching_pairs(reason: str \| None = None, *, samples: int = 0, seed: int = 0, always_ratio: float = 1.0) -> list[tuple[str, str]]`<br>`attach_collider_layer(layer_or_path: str \| os.PathLike[str] \| Sdf.Layer, *, target: LayerTarget = "session") -> str`<br>`set_collider_layer_enabled(layer_or_path: str \| os.PathLike[str] \| Sdf.Layer, enabled: bool) -> None`<br>`detach_collider_layer(layer_or_path: str \| os.PathLike[str] \| Sdf.Layer \| None = None) -> bool`<br>`reload_colliders() -> dict[str, list[str]]`<br>`refresh_colliders() -> None` |
| 構成要素・同期・終了 | `get_component(name: str) -> GeometricRobotComponent`<br>`flush_to_usd() -> bool`<br>`refresh() -> None`<br>`close(*, remove_layer: bool = True) -> None` |

### `GeometricRobotComponent` / `GeometricJoint` / `GeometricLink` / `GeometricStructureChangedError`

- **`GeometricRobotComponent(robot, name)`**：
  - プロパティ：`robot`, `link_names`, `dof_indices`, `num_dofs`, `dof_names`, `link_paths`, `link_set`
  - メソッド：`set_dof_positions`, `get_dof_positions`, `get_dof_limits`, `get_link_world_poses`, `set_variant`, `get_variants`, `check_collisions`, `compute_clearances`, `is_in_collision`
- **`GeometricJoint(robot, spec)`**：`name`, `is_dof`, `dof_index`, `get_position(*, unit=None)`, `set_position(value, *, unit=None)`, `get_limits(*, unit=None)`
- **`GeometricLink(robot, spec, index)`**：`name`, `prim_path`, `get_world_pose()`, `get_world_matrix()`
- **`GeometricStructureChangedError(RuntimeError)`**：構造再構築前に作成した `KinematicChain` / `IKSolver` / `GeometricRobotView` の使用時に送出。

---

## 3. 複数台バッチ API（`robot_view.py`）

### `GeometricRobotView`

```python
GeometricRobotView(
    robots: Sequence[GeometricRobot],
    *,
    write_target: WriteTarget | None = None,
    angle_unit: AngleUnit | None = None,
    collision_backend: CollisionBackend | None = None,
    fk_backend: FkBackend | None = None,
    newton_device: str | None = None,
    environment_roots: Sequence[str] | None = None,
    use_cuda_graph: bool = True,
    use_gpu_fabric_writer: bool = False,
)
```

| 種別 | シグネチャ |
|---|---|
| ファクトリ | `from_articulations(cls, stage: Usd.Stage, sources: Sequence[UsdArticulationSource \| str], *, layer_options=None, write_target=None, apply_layer=True, collision_backend=None, fk_backend=None, newton_device=None, environment_roots=None, use_cuda_graph=True, use_gpu_fabric_writer=False) -> GeometricRobotView` |
| プロパティ | `angle_unit`, `collision_backend`, `fk_backend`, `newton_scene: BatchedNewtonScene`, `robots`, `layer`, `num_dofs`, `dof_names`, `dof_types`, `num_joints`, `joint_names`, `num_links`, `link_names`, `dof_paths`, `joint_paths`, `link_paths`, `__len__()` |
| インデックス・関節値 | `get_dof_indices(names) -> wp.array`<br>`get_joint_indices(names) -> wp.array`<br>`get_link_indices(names) -> wp.array`<br>`set_dof_positions(positions, *, indices=None, dof_indices=None, unit=None) -> None`<br>`get_dof_positions(*, indices=None, dof_indices=None, unit=None) -> wp.array`<br>`set_dof_position_targets(positions, *, indices=None, dof_indices=None, unit=None) -> None`<br>`get_dof_position_targets(*, indices=None, dof_indices=None, unit=None) -> wp.array`<br>`get_dof_limits(*, indices=None, dof_indices=None, unit=None) -> tuple[wp.array, wp.array]` |
| 姿勢 | `get_world_poses(*, indices=None) -> tuple[wp.array, wp.array]`<br>`set_world_poses(positions=None, orientations=None, *, indices=None) -> None`<br>`get_link_world_poses(*, indices=None, link_indices=None) -> tuple[wp.array, wp.array]`<br>`get_link_world_matrices() -> np.ndarray` |
| Newton バッチ干渉 | `sync_environment(*, use_fabric: bool \| str = "auto") -> None`<br>`reload_colliders() -> None`<br>`check_collisions_batch(scope: CollisionScope = "all", *, clearance: float = 0.0, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True) -> list[list[GeometricCollisionPair]]`<br>`compute_clearances_batch(scope: CollisionScope = "all", *, max_distance: float = 0.05, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True) -> list[list[GeometricCollisionPair]]` |
| 同期・終了 | `flush_to_usd() -> bool`<br>`close(*, remove_layer: bool = True) -> None` |

---

## 4. Newton バックエンド（`newton_backend.py`）

### `resolve_newton_device(device: str = "auto") -> str`

`"auto"` 指定時は CUDA が利用可能なら `"cuda:0"`、そうでなければ `"cpu"` を返します。

### `GeometricScene`

```python
GeometricScene(
    stage: Usd.Stage,
    *,
    device: str = "auto",
    broad_phase: Literal["auto", "explicit", "nxn", "sap"] = "auto",
    convex_meshes: bool = True,
    bake_adjacent_filter: bool = False,
    auto_discover_environment: bool = True,
)
```

| カテゴリ | プロパティ・メソッド |
|---|---|
| プロパティ | `stage: Usd.Stage`, `device: str`, `newton_model`, `newton_state`, `robots: list[Any]`, `environment_collider_paths: list[str]` |
| 登録・管理 | `add_robot(robot: GeometricRobot) -> None`<br>`add_model(model: GeometricRobotModel, link_prim_paths: dict[str, str], *, name=None, collision_filter=None, base_matrix=None) -> _ModelProxy`<br>`remove_robot(robot: Any) -> bool`<br>`add_environment(prim_paths_or_root: str \| Sequence[str], *, exclude_robots: bool = True, exclude_prim_paths: Sequence[str] = ()) -> int`<br>`clear_environment() -> None`<br>`mark_dirty() -> None`<br>`invalidate() -> None`<br>`finalize() -> None`<br>`close() -> None` |
| 順運動学・同期 | `sync_robots() -> None`<br>`eval_fk(robot: Any, q: np.ndarray \| None = None, *, base_matrix: np.ndarray \| None = None, resolve_mimic: bool = True) -> np.ndarray`<br>`get_robot_body_indices(robot: Any) -> np.ndarray`<br>`sync_environment(*, use_fabric: bool \| Literal["auto"] = "auto", reset_manual_overrides: bool = True) -> None`<br>`set_obstacle_pose(prim_path: str, position: Sequence[float], orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0)) -> None` |
| 干渉・クリアランス | `check_within(link_set: GeometricLinkSet, *, clearance: float = 0.0, collision_filter=None, extra_filter=None, sync_env: bool = False) -> list[GeometricCollisionPair]`<br>`check_between(set_a: GeometricLinkSet, set_b: GeometricLinkSet, *, clearance: float = 0.0, extra_filter=None, sync_env: bool = False) -> list[GeometricCollisionPair]`<br>`check_environment(link_set: GeometricLinkSet, *, clearance: float = 0.0, exclude=(), ignore_prim_paths=(), sync_env: bool = True) -> list[GeometricCollisionPair]`<br>`touching_pairs(link_set: GeometricLinkSet, *, clearance: float = 0.0) -> list[GeometricCollisionPair]`<br>`check_collisions(robot=None, *, link_set=None, scope=None, clearance: float = 0.0, collision_filter=None, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True) -> list[GeometricCollisionPair]`<br>`compute_clearances(robot=None, *, link_set=None, scope=None, max_distance: float = 0.05, collision_filter=None, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True) -> list[GeometricCollisionPair]` |

### `BatchedNewtonScene`

```python
BatchedNewtonScene(
    view: GeometricRobotView,
    *,
    device: str = "auto",
    environment_roots: Sequence[str] | None = None,
    broad_phase: Literal["auto", "explicit", "nxn", "sap"] = "auto",
    convex_meshes: bool = True,
    bake_adjacent_filter: bool = True,
    use_cuda_graph: bool = True,
)
```

- プロパティ：`device: str`, `newton_model`, `newton_state`, `prim_body_indices: np.ndarray`
- メソッド：
  - `eval_fk_batch(q_batch: np.ndarray, base_batch: np.ndarray, *, resolve_mimic: bool = True, return_matrices: bool = True) -> np.ndarray | None`
  - `sync_environment(*, use_fabric: bool | Literal["auto"] = "auto") -> None`
  - `check_collisions_batch(q_batch: np.ndarray, base_batch: np.ndarray, *, scope: CollisionScope = "all", clearance: float = 0.0, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True, resolve_mimic: bool = True) -> list[list[GeometricCollisionPair]]`
  - `compute_clearances_batch(q_batch: np.ndarray, base_batch: np.ndarray, *, scope: CollisionScope = "all", max_distance: float = 0.05, extra_filter=None, ignore_prim_paths=(), sync_env: bool = True, resolve_mimic: bool = True) -> list[list[GeometricCollisionPair]]`
  - `close() -> None`

---

## 5. 干渉データ・PhysX チェッカ・Sublayer・Writer

### `collision.py`

#### `GeometricCollisionPair`（`@dataclass(frozen=True)`）

| フィールド | 型 | 既定値 | 意味 |
|---|---|---|---|
| `link_a` | `str` | 必須 | リンク A 名 |
| `link_b` | `str \| None` | 必須 | リンク B 名（環境コライダーとの組は `None`） |
| `path_a` | `str` | 必須 | リンク A の prim パス |
| `path_b` | `str` | 必須 | リンク B または環境コライダーの prim パス |
| `kind` | `PairKind` | 必須 | `"within_component"` / `"between_components"` / `"between_robots"` / `"environment"` |
| `component_a`, `component_b` | `str \| None` | `None` | 構成要素名 |
| `robot_a`, `robot_b` | `str` / `str \| None` | `""` / `None` | ロボット名 |
| `separation` | `float \| None` | `None` | 符号付き距離 [m]（負＝めり込み、正＝クリアランス。Newton バックエンドで設定） |
| `point_a`, `point_b` | `tuple[float, float, float] \| None` | `None` | ワールド座標系の接触表面点（A 側 / B 側） |
| `normal` | `tuple[float, float, float] \| None` | `None` | 単位法線ベクトル（A → B 向き） |

- プロパティ：`names -> tuple[str, str]`（`(link_a, link_b or path_b)`）
- 補助関数・クラス：
  - `find_colliders(stage: Usd.Stage, link_prim_path: str, other_link_paths: Sequence[str] = ()) -> list[str]`
  - `GeometricLinkSet(robot, link_names, name)`：`collider_paths`, `link_paths`, `union(other, name=None)`
  - `GeometricCollisionChecker(stage)`：`check_within`, `check_between`, `check_environment`, `touching_pairs`, `reload`（PhysX `overlap_shape`）
  - `GeometricContactMonitor(robots, max_events=10000)`：`touching`, `penetrating`, `separations`, `clear`, `close`（`collision_mode="kinematic"` + `Play=ON`）

### `layer.py`

- **`GeometricLayerOptions`**（`@dataclass(frozen=True)`）：
  - `collision_mode: CollisionMode = "static"`
  - `layer_target: LayerTarget = "session"`
  - `report_contacts: bool = True`
  - `contact_threshold: float = 0.0`
  - `root_layer_file: str | None = None`
  - クラスメソッド：`from_settings() -> GeometricLayerOptions`
- **`GeometricLayerReport`**（`@dataclass`）：`layer_identifier: str = ""`, `changes: list[tuple[str, str, Any, Any]]`, `warnings: list[str]`
- **`GeometricLayer(stage, options=None)`**：
  - プロパティ：`stage`, `options`, `report`, `sdf_layer`
  - メソッド：`is_applied()`, `add_listener(cb)`, `remove_listener(cb)`, `register(*, link_prim_paths, articulation_root_paths=None, joint_prim_paths=None, adjacent_link_paths=None) -> int`, `update_entry(token, ...) -> dict[str, int]`, `unregister(token)`, `apply() -> GeometricLayerReport`, `remove() -> bool`, `edit_context()`, `export(path)`, `save()`, `xform_specs(link_prim_paths)`
- コンテキスト：`authoring() -> Iterator[None]`, `is_authoring() -> bool`

### `writer.py`

| クラス | コンストラクタ・主なメソッド |
|---|---|
| `LocalPoseSolver` | `(stage: Usd.Stage, prim_paths: list[str])` — `prim_paths`, `world_scales`, `refresh()`, `solve(world)`, `local_matrices(world)` |
| `GeometricTransformWriter` (`ABC`) | `write(solver: LocalPoseSolver, world: np.ndarray) -> None`, `close() -> None` |
| `GeometricUsdWriter` | `(layer: GeometricLayer, prim_paths: list[str])` — Sublayer 属性へ `Sdf.ChangeBlock` で一括書き込み |
| `GeometricFabricWriter` | `(stage: Usd.Stage, prim_paths: list[str], *, batch: bool \| None = None)` — `write()`, `read_world() -> np.ndarray`, `close()` |
| `GeometricGpuFabricWriter` | `(stage: Usd.Stage, prim_paths: Sequence[str], body_indices: Sequence[int], *, world_scales: np.ndarray \| None = None, device: str = "cuda:0")` — `write_body_q(body_q_wp, body_count: int) -> None`, `close() -> None` |

---

## 6. モデル・生成元・IK・フィルタ・コマンド変換（`model/`・`usd_reader.py`・`builder.py`）

### モデル仕様（`model/spec.py`）

- **`GeometricMimicSpec`**：`reference_joint: str`, `multiplier: float = 1.0`, `offset: float = 0.0`
- **`GeometricJointSpec`**：`name`, `joint_type`, `parent_link`, `child_link`, `parent_frame=tf.identity()`, `child_frame=tf.identity()`, `axis=(0.0, 0.0, 1.0)`, `lower_limit=-np.inf`, `upper_limit=np.inf`, `mimic=None`, `prim_path=None`（`is_dof`, `replaced(**changes)`）
- **`GeometricLinkSpec`**：`name: str`, `prim_path: str | None = None`
- **`GeometricRobotModel`**：`name`, `links`, `joints`, `root_link=None`, `components={}`
  - プロパティ：`num_links`, `num_joints`, `num_dofs`, `link_names`, `joint_names`, `joint_types`, `dof_joints`, `dof_names`, `dof_types`, `dof_lower_limits`, `dof_upper_limits`, `independent_dof_indices`, `mimic_matrix`, `mimic_offset`
  - メソッド：`get_link_index`, `get_joint_index`, `get_dof_index`, `parent_joint_of`, `child_joints_of`, `component_of_link`, `adjacent_link_pairs`, `resolve_mimic`, `joint_local_transforms`, `forward_kinematics(dof_positions=None, *, resolve_mimic=True)`, `forward_kinematics_reference`, `jacobian(dof_positions=None, *, reduced=False)`, `path_between`, `get_kinematic_chain`, `with_prefix`, `attach(other, parent_link, mount_transform=None, *, name, base_name=None)`, `is_kinematically_equal(other, tol=1e-9)`, `to_urdf(path=None, *, robot_name=None)`
- **`serial_model(name, joint_specs, link_names) -> GeometricRobotModel`**

### 生成元とプリム構築（`usd_reader.py`・`model/sources.py`・`model/urdf.py`・`builder.py`）

| クラス / 関数 | フィールド / シグネチャ |
|---|---|
| `UsdArticulationSource` | `root_prim_path`, `components=None`, `extra_joint_paths=()`, `ignore_joint_paths=()`, `dof_order=None`, `unsupported_joints="error"`, `world_joints="ignore"`, `name=None` |
| `UsdRobotInfo` | `root_prim_paths`, `articulation_root_paths`, `joint_prim_paths`, `link_prim_paths`, `self_collision_enabled`, `colliding_adjacent_pairs`, `filtered_pairs`, `reversed_joints`, `warnings`, `timings_ms` |
| `read_articulation` | `(stage: Usd.Stage, source: UsdArticulationSource \| str) -> tuple[GeometricRobotModel, UsdRobotInfo]` |
| `DHRow` / `DHParameters` | `DHRow(theta=0.0, d=0.0, a=0.0, alpha=0.0, joint_type="revolute", lower_limit=-np.inf, upper_limit=np.inf, name=None)` / `DHParameters(rows, base=None, tool=None, name="dh_robot").to_model()` |
| `MDHRow` / `MDHParameters` | `MDHRow(alpha=0.0, a=0.0, theta=0.0, d=0.0, joint_type="revolute", lower_limit=-np.inf, upper_limit=np.inf, name=None)` / `MDHParameters(rows, base=None, tool=None, name="mdh_robot").to_model()` |
| `ScrewAxisRow` / `ScrewParameters` | `ScrewAxisRow(direction, moment=(0,0,0), joint_type="revolute", ...)` (`through(point, direction, ...)`) / `ScrewParameters(rows, tool_home=None, name="screw_robot").to_model()` |
| `UrdfSource` / `model/urdf.py` | `UrdfSource(urdf_path, name=None).to_model()` / `read_urdf(path, name=None)`, `read_urdf_string(text, name=None)`, `model_to_urdf(model, robot_name=None)` |
| `GeometricVisualStyle` / `build_robot_prims` | `GeometricVisualStyle(link_radius=0.025, joint_radius=0.035, joint_length=0.09, link_color=(0.75,0.78,0.82), revolute_color=(0.95,0.55,0.1), prismatic_color=(0.15,0.55,0.95), collisions=True, collision_radius=None)` / `build_robot_prims(stage, model, prim_path, *, style=None, base_transform=None) -> dict[str, str]` |

### IK・鎖・フィルタ・コマンド変換

- **`KinematicChain`（`model/chain.py`）**：`from_model(model, base_link, tip_link, tip_offset=None)`, `num_variables`, `num_joints`, `has_mimic`, `joint_values(variables)`, `forward_kinematics(variables)`, `jacobian(variables)`, `pose_and_jacobian(variables, need_jacobian=True)`, `clip(variables)`, `to_model(name="chain")`, `to_urdf(path=None, *, robot_name="chain")`
- **`IKSolverConfig`（`model/ik.py`）**：`active_dof_mask=None`, `orientation_mode="full"`, `position_tolerance=1e-5`, `orientation_tolerance=1e-4`, `max_iterations=200`, `num_restarts=20`, `damping=0.01`, `orientation_weight=1.0`, `respect_limits=True`, `random_seed=0`
- **`IKResult`**：`dof_positions`, `success`, `position_error`, `orientation_error`, `iterations=0`, `message=""`
- **`IKSolver` / `JacobianIKSolver` / `IKPySolver`**：`set_config(**changes)`, `set_active_dofs(mask_or_names)`, `get_active_dofs()`, `solve(target_position, target_orientation=None, *, seed=None) -> IKResult`；レジストリ関数 `register_ik_solver(name, cls)`, `available_ik_solvers()`, `create_ik_solver(name, chain, config=None)`, `pose_error(chain, variables, target)`
- **`GeometricCollisionFilter`（`model/collision_filter.py`）**：`from_model(model)`, `copy()`, `merged_with(other, *, prefix="", other_prefix="")`, `add_pair(a, b, reason="user")`, `remove_pair(a, b)`, `add_pattern(pa, pb, reason="user")`, `add_rule(rule, reason="user")`, `remove_by_reason(reason)`, `clear()`, `reason_for(a, b)`, `is_ignored(a, b)`, `pairs`, `patterns`, `num_rules`, `to_json(path=None)`, `from_json(text_or_path)`
- **`GeometricCommandConverter`（`model/command.py`）**：
  - `LinearCommandConverter(input_names, output_dofs, matrix, offset=None, output_angle_unit="rad")`
  - `InterpCommandConverter(input_name, output_dofs, table_inputs, table_outputs, output_angle_unit="rad", output_for_inverse=None)`
  - `FunctionCommandConverter(input_names, output_dofs, forward, inverse=None, output_angle_unit="rad")`
  - 共通メソッド：`to_dof_positions(command)`, `from_dof_positions(dof_positions)`

---

## 7. `config/extension.toml` 設定キー一覧

すべての設定キーは `exts."myext.robot.geometric_newton".<key>`（起動引数では `--/exts/myext.robot.geometric_newton/<key>=<value>`）に属します。

| キー名 | 既定値 | 取り得る値 | 説明 |
|---|---|---|---|
| `layer_target` | `"session"` | `"session"`, `"root"` | `GeometricLayer` を差し込むレイヤー |
| `root_layer_file` | `""` | ファイルパス文字列 | `layer_target="root"` 時の Sublayer 保存先 `.usda` パス |
| `collision_mode` | `"static"` | `"static"`, `"kinematic"` | PhysX リンク設定（静的コライダー / kinematic 剛体） |
| `write_target` | `"fabric"` | `"fabric"`, `"usd"`, `"both"` | リンク姿勢の書き込み先 |
| `angle_unit` | `"rad"` | `"rad"`, `"deg"` | 回転関節の既定角度単位 |
| `collision_backend` | `"newton"` | `"newton"`, `"physx"` | 干渉判定の既定バックエンド |
| `fk_backend` | `"auto"` | `"auto"`, `"numpy"`, `"newton"` | 順運動学の既定バックエンド |
| `newton_device` | `"auto"` | `"auto"`, `"cuda:0"`, `"cpu"` | Newton バックエンドの実行デバイス |

---

## 8. 20 モジュール構成マップ（`myext/robot/geometric_newton/`）

| # | ファイルパス | 役割 |
|---:|---|---|
| 1 | `__init__.py` | 公開シンボル定義（`model/` は即時公開、USD/Kit/Newton 系は `__getattr__` 遅延 import） |
| 2 | `extension.py` | Kit 拡張エントリポイント `GeometricRobotNewtonExtension` |
| 3 | `newton_backend.py` | `GeometricScene`、`BatchedNewtonScene`、`resolve_newton_device`、USD コライダー変換・動的同期 |
| 4 | `robot.py` | `GeometricRobot`、`GeometricRobotComponent`、`GeometricJoint`、`GeometricLink`、`GeometricStructureChangedError` |
| 5 | `robot_view.py` | `GeometricRobotView`（複数台バッチ FK・干渉判定・GPU Fabric 連携） |
| 6 | `collision.py` | `GeometricCollisionPair`、`GeometricLinkSet`、`GeometricCollisionChecker`、`GeometricContactMonitor`、`find_colliders` |
| 7 | `writer.py` | `LocalPoseSolver`、`GeometricUsdWriter`、`GeometricFabricWriter`、`GeometricGpuFabricWriter` |
| 8 | `layer.py` | `GeometricLayer`、`GeometricLayerOptions`、`GeometricLayerReport`、`authoring` |
| 9 | `usd_reader.py` | `UsdArticulationSource`、`UsdRobotInfo`、`read_articulation`、`split_scale` |
| 10 | `builder.py` | `GeometricVisualStyle`、`build_robot_prims`、`safe_prim_name` |
| 11 | `model/__init__.py` | Pure NumPy 層の公開 re-export |
| 12 | `model/spec.py` | `GeometricRobotModel`、`GeometricJointSpec`、`GeometricLinkSpec`、`GeometricMimicSpec`、`serial_model` |
| 13 | `model/transforms.py` | 同次変換・回転行列・四元数 `(w, x, y, z)`・RPY・回転ベクトル演算 |
| 14 | `model/sources.py` | `DHParameters` / `DHRow`、`MDHParameters` / `MDHRow`、`ScrewParameters` / `ScrewAxisRow`、`UrdfSource` |
| 15 | `model/urdf.py` | `read_urdf`、`read_urdf_string`、`model_to_urdf` |
| 16 | `model/chain.py` | `KinematicChain`、`SerialJoint`（mimic 畳み込み・ヤコビアン計算） |
| 17 | `model/ik.py` | `IKSolver`、`JacobianIKSolver`、`IKSolverConfig`、`IKResult`、`register_ik_solver` |
| 18 | `model/ik_ikpy.py` | `IKPySolver`、`import_ikpy` |
| 19 | `model/collision_filter.py` | `GeometricCollisionFilter`（ペア・パターン・関数ルール・JSON 入出力） |
| 20 | `model/command.py` | `GeometricCommandConverter`、`LinearCommandConverter`、`InterpCommandConverter`、`FunctionCommandConverter` |
