"""`GeometricLayer`: 幾何モード用の上書きを隔離する Sublayer。

`apply()` で Sublayer を差し込むと幾何モードになり、`remove()` で外すと元の Articulation 構成へ完全に戻る。
すべての上書きはこの Sublayer だけに書き込むため、元のロボット USD (Root Layer や参照元アセット) は
一度も変更されない。

レイヤー構成:
    Session Layer
      └─ GeometricLayer (匿名 Sublayer、既定) ← ステージを保存しても Root Layer に混入しない
    Root Layer

設計上の意図 (なぜ匿名 Session Sublayer を使うか):
    1. 非破壊性: `layer_target="session"` (既定) では匿名 `Sdf.Layer` を Session Layer の先頭 (最強位置) に
       差し込むため、ユーザーがステージを保存しても幾何モード用の `rigidBodyEnabled=false` や
       `xformOp:transform:geometric` が元ファイルに汚染しない。
    2. `Play=ON` 共存: 既存の PhysX Articulation を `Play=ON` のまま幾何駆動すると、PhysX ソルバーが
       毎フレーム Fabric / USD のリンク姿勢を上書きしてしまう。Sublayer 側で `articulationEnabled=false`、
       `jointEnabled=false`、`rigidBodyEnabled=false` を掛けることで、同じステージ上の他オブジェクトの
       PhysX 物理シミュレーションを生かしたままロボットだけ幾何駆動できる。
    3. `static` と `kinematic` プロファイルのトレードオフ:
       - `collision_mode="static"` (既定): 各リンクを `physics:rigidBodyEnabled = false` の静的コライダーにする。
         PhysX ソルバーが一切触らないため `Stop` / `Play=ON` 両方で姿勢が保持され、Newton バックエンド
         (`GeometricScene`) と組み合わせると Fabric のみの書き込みで 0 フレーム遅延の干渉判定ができる。
         PhysX バックエンド (`overlap_shape`) も `Stop` 中から動作する。
       - `collision_mode="kinematic"`: 各リンクを `physics:kinematicEnabled = true` の kinematic 剛体にする。
         `Play=ON` 中に PhysX の `ContactReportAPI` (`GeometricContactMonitor`) で接触イベントを受け取れる
         反面、PhysX 側の姿勢反映に 1〜2 フレームの遅延があり、かつ Joint を無効化したことで失われる
         「隣接リンク同士の暗黙衝突除外」を `UsdPhysics.FilteredPairsAPI` で明示再構築する必要がある。
    4. `CollisionAPI` とコライダー形状そのものには一切触らないため、元の衝突形状定義をそのまま再利用できる。
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Literal, get_args

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

__all__ = [
    "CollisionMode",
    "LayerTarget",
    "GeometricLayerOptions",
    "GeometricLayerReport",
    "GeometricLayer",
    "XFORM_OP_NAME",
    "LAYER_MARKER",
    "SETTINGS_ROOT",
    "is_collision_enabled_ignoring_geometric_layer",
]

# ==============================================================================
# 定数と Sublayer 書き込み中ガード
# ==============================================================================

CollisionMode = Literal["none", "static", "kinematic"]
LayerTarget = Literal["session", "root"]

# リンク姿勢の上書きに使う専用 xformOp 名 (元の translate/orient 等を破壊せず単一行列で上書きする)
XFORM_OP_NAME = "xformOp:transform:geometric"
# `customLayerData` に付ける目印。`usd_reader._marker_layers` が幾何モード Sublayer を識別して無視するために使う
LAYER_MARKER = "myext.robot.geometric_newton"
SETTINGS_ROOT = "/exts/myext.robot.geometric_newton"


# ネストした `with authoring():` に対応するための再入カウンタ
_AUTHORING_DEPTH = [0]


@contextmanager
def authoring() -> Iterator[None]:
    """この拡張自身が Sublayer を編集している区間を表すコンテキストマネージャ。

    なぜ必要か:
        `GeometricRobot` や `GeometricScene` は `Usd.Notice.ObjectsChanged` を監視して
        VariantSet 切替などの構造変更を自動検知する。自分自身の姿勢書き込みや Sublayer 編集による
        通知を「外部からの構造変更」と誤認して無限リビルドを起こさないよう、このフラグで抑止する。
    """
    _AUTHORING_DEPTH[0] += 1
    try:
        yield
    finally:
        _AUTHORING_DEPTH[0] -= 1


def is_authoring() -> bool:
    """現在 `authoring()` コンテキストの内側であれば `True` を返す。"""
    return _AUTHORING_DEPTH[0] > 0


def is_collision_enabled_ignoring_geometric_layer(prim: Usd.Prim) -> bool:
    """`GeometricLayer` 自身の `collision_mode="none"` 上書きを無視して `physics:collisionEnabled` を判定する。

    設計上の意図:
        `collision_mode="none"` では PhysX 側のコライダーを完全に無効化するため、
        Session Sublayer (`LAYER_MARKER` 付き) に `physics:collisionEnabled = False` を書く。
        一方、Newton バックエンド (`GeometricScene`) はロボット本来のコライダー形状を使って干渉判定を行う
        必要があるため、属性の PropertyStack から `LAYER_MARKER` 付きレイヤーの上書きだけをスキップし、
        元の USD やユーザーが追加したカスタムコライダー Sublayer の `collisionEnabled` 値を読む。
    """
    attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
    if not attr or not attr.IsValid():
        return True
    for spec in attr.GetPropertyStack():
        if spec.layer.customLayerData.get(LAYER_MARKER):
            continue
        if spec.HasDefaultValue():
            val = spec.default
            return True if val is None else bool(val)
    # どのレイヤーにも明示値が無ければ USD スキーマ既定値 (True)
    return True


def _setting(key: str, default: Any) -> Any:
    """Kit の `carb.settings` から拡張設定を読む。Kit 外 (`carb` 未導入) や未設定時は `default` を返す。"""
    try:
        import carb.settings

        value = carb.settings.get_settings().get(f"{SETTINGS_ROOT}/{key}")
        return default if value is None else value
    except Exception:
        return default


# ==============================================================================
# 設定・レポート・内部レコード
# ==============================================================================


@dataclass(frozen=True)
class GeometricLayerOptions:
    """`GeometricLayer` の動作プロファイル。

    collision_mode    : `"static"`    = リンクを静的コライダーにする (`rigidBodyEnabled=false`)。
                                        Newton バックエンド推奨設定であり、PhysX `overlap_shape` も `Stop` 中から動作する。
                        `"kinematic"` = リンクを kinematic 剛体にする (`kinematicEnabled=true`)。
                                        `Play=ON` 中に PhysX の Contact Report (`GeometricContactMonitor`) を使うときに選ぶ。
    layer_target      : `"session"`   = Session Layer の下に匿名 Sublayer を差す (保存しても元ファイルに混ざらない)。
                        `"root"`      = Root Layer の下にファイル Sublayer を差す (保存時に参照が残る)。`root_layer_file` 必須。
    report_contacts   : `collision_mode="kinematic"` のとき、各リンクへ `PhysxContactReportAPI` と
                        `PhysicsScene` の kinematic 接触報告フラグを書き込むか。
    contact_threshold : `PhysxContactReportAPI` の `threshold` 値。
    root_layer_file   : `layer_target="root"` のときに作成する Sublayer の `.usda` ファイルパス。
    """

    collision_mode: CollisionMode = "none"
    layer_target: LayerTarget = "session"
    report_contacts: bool = True
    contact_threshold: float = 0.0
    root_layer_file: str | None = None

    def __post_init__(self) -> None:
        if self.collision_mode not in get_args(CollisionMode):
            raise ValueError(f"collision_mode は {get_args(CollisionMode)} のどれか: {self.collision_mode!r}")
        if self.layer_target not in get_args(LayerTarget):
            raise ValueError(f"layer_target は {get_args(LayerTarget)} のどれか: {self.layer_target!r}")
        if self.layer_target == "root" and not self.root_layer_file:
            raise ValueError('layer_target="root" のときは root_layer_file(Sublayer を保存する .usda のパス)が必要')

    @classmethod
    def from_settings(cls) -> "GeometricLayerOptions":
        """Kit 設定 (`/exts/myext.robot.geometric_newton/{collision_mode,layer_target,root_layer_file}`) から生成する。"""
        return cls(
            collision_mode=_setting("collision_mode", "none"),
            layer_target=_setting("layer_target", "session"),
            root_layer_file=_setting("root_layer_file", None) or None,
        )


@dataclass
class GeometricLayerReport:
    """`apply()` が Sublayer へ書き込んだ上書き一覧と警告の記録。

    `changes` の各要素は `(prim_path, attr_name, before_value, written_value)`。
    """

    layer_identifier: str = ""
    changes: list[tuple[str, str, Any, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class _Entry:
    """1 つの `GeometricLayer` が管理するロボット 1 台分の対象 prim 情報。"""

    articulation_root_paths: list[str]
    joint_prim_paths: list[str]
    link_prim_paths: list[str]
    # 関節で直接つながったリンクの prim パスの組 (`collision_mode="kinematic"` の `FilteredPairsAPI` 用)
    adjacent_link_paths: list[tuple[str, str]]
    # `collision_mode="none"` で `physics:collisionEnabled=False` を書いたコライダー所有者 prim パス一覧
    collider_owner_paths: list[str] = field(default_factory=list)


# ==============================================================================
# GeometricLayer 本体
# ==============================================================================


class GeometricLayer:
    """複数ロボットで共有可能な幾何モード Sublayer 管理者。"""

    def __init__(self, stage: Usd.Stage, options: GeometricLayerOptions | None = None) -> None:
        self._stage = stage
        self._options = options or GeometricLayerOptions.from_settings()
        self._entries: dict[int, _Entry] = {}
        self._next_token = 0
        self._layer: Sdf.Layer | None = None
        self._report = GeometricLayerReport()
        self._listeners: list[Callable[[bool], None]] = []
        # 各リンクの `xformOp:transform:geometric` の `Sdf.AttributeSpec` キャッシュ (`GeometricUsdWriter` が高速更新する)
        self._xform_specs: dict[str, Sdf.AttributeSpec] = {}

    # ------------------------------------------------------------------ 状態

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def options(self) -> GeometricLayerOptions:
        return self._options

    @property
    def report(self) -> GeometricLayerReport:
        return self._report

    @property
    def sdf_layer(self) -> Sdf.Layer | None:
        """適用中のみ有効な `Sdf.Layer`。未適用時は `None`。"""
        return self._layer

    def is_applied(self) -> bool:
        return self._layer is not None

    def add_listener(self, callback: Callable[[bool], None]) -> None:
        """`apply()` 直後に `callback(True)`、`remove()` 直後に `callback(False)` を通知するリスナーを登録する。"""
        self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[bool], None]) -> None:
        if callback in self._listeners:
            self._listeners.remove(callback)

    # ------------------------------------------------------------------ ロボットの登録・更新・解除

    def register(
        self,
        *,
        link_prim_paths: list[str],
        articulation_root_paths: list[str] | None = None,
        joint_prim_paths: list[str] | None = None,
        adjacent_link_paths: list[tuple[str, str]] | None = None,
    ) -> int:
        """この Sublayer の管理対象にロボット 1 台を追加し、識別トークンを返す。

        すでに `apply()` 済みの場合は、その場で追加分の上書きを書き足して PhysX キャッシュを解放する。
        """
        entry = _Entry(list(articulation_root_paths or []), list(joint_prim_paths or []), list(link_prim_paths), list(adjacent_link_paths or []))
        token = self._next_token
        self._next_token += 1
        self._entries[token] = entry
        if self._layer is not None:
            with authoring(), Usd.EditContext(self._stage, self._layer):
                self._author(entry)
            self._release_physics()
        return token

    def update_entry(
        self,
        token: int,
        *,
        link_prim_paths: list[str],
        articulation_root_paths: list[str] | None = None,
        joint_prim_paths: list[str] | None = None,
        adjacent_link_paths: list[tuple[str, str]] | None = None,
    ) -> dict[str, int]:
        """トークン `token` のロボット対象 prim を差分差し替えする (VariantSet 切替等の再構築用)。

        適用中の場合は不要になった prim への上書きだけを Sublayer から削除し、増えた prim のみ追記する。
        両方に共通する prim の上書きは維持することで既存の `Sdf.AttributeSpec` 参照を壊さない。
        戻り値は `{"removed": 削除 prim 数, "added": 追記 prim 数}`。
        """
        old = self._entries[token]
        new = _Entry(list(articulation_root_paths or []), list(joint_prim_paths or []), list(link_prim_paths), list(adjacent_link_paths or []))
        self._entries[token] = new
        if self._layer is None:
            return {"removed": 0, "added": 0}
        old_paths = set(old.articulation_root_paths) | set(old.joint_prim_paths) | set(old.link_prim_paths) | set(old.collider_owner_paths)
        new_paths = set(new.articulation_root_paths) | set(new.joint_prim_paths) | set(new.link_prim_paths)
        still_used: set[str] = set()
        # 同じ Sublayer を共有している他ロボットが参照中の prim は誤って消さないよう保護する
        for other_token, other in self._entries.items():
            if other_token != token:
                still_used |= (
                    set(other.articulation_root_paths)
                    | set(other.joint_prim_paths)
                    | set(other.link_prim_paths)
                    | set(other.collider_owner_paths)
                )
        # 子 prim を先に消してから空の親 over を畳めるようパスの深い順に並べる
        gone = sorted(old_paths - new_paths - still_used, key=lambda p: -p.count("/"))
        old_pairs, new_pairs = set(old.adjacent_link_paths), set(new.adjacent_link_paths)
        added = _Entry(
            [p for p in new.articulation_root_paths if p not in old_paths],
            [p for p in new.joint_prim_paths if p not in old_paths],
            [p for p in new.link_prim_paths if p not in old_paths],
            [pair for pair in new.adjacent_link_paths if pair not in old_pairs],
        )
        dropped_pairs = [pair for pair in old.adjacent_link_paths if pair not in new_pairs]
        with authoring():
            with Sdf.ChangeBlock():
                for path in gone:
                    self._remove_prim_spec(path)
            with Usd.EditContext(self._stage, self._layer):
                if self._options.collision_mode == "kinematic":
                    for a, b in dropped_pairs:  # 消えたリンクを指す `FilteredPairsAPI` の参照を外す
                        prim = self._stage.GetPrimAtPath(a)
                        if prim and prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                            UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().RemoveTarget(b)
                self._author(added, scene_flags=False)
                # `collision_mode="none"` では残存リンクのコライダー上書きも `new.collider_owner_paths` へ引き継ぐ
                if self._options.collision_mode == "none":
                    new.collider_owner_paths = list(
                        dict.fromkeys([p for p in old.collider_owner_paths if p not in gone] + added.collider_owner_paths)
                    )
        self._release_physics()
        return {"removed": len(gone), "added": len(added.link_prim_paths) + len(added.joint_prim_paths) + len(added.articulation_root_paths)}

    def unregister(self, token: int) -> None:
        """トークン `token` のロボットを管理対象から外し、適用中ならその上書きを Sublayer から削除する。"""
        if token not in self._entries:
            return
        self.update_entry(token, link_prim_paths=[])
        del self._entries[token]

    def _remove_prim_spec(self, path: str) -> None:
        """Sublayer から `path` の prim spec を消し、空になった親 over もさかのぼって掃除する (`Sdf.ChangeBlock` 内で呼ぶ)。"""
        assert self._layer is not None
        self._xform_specs.pop(path, None)
        sdf_path = Sdf.Path(path)
        spec = self._layer.GetPrimAtPath(sdf_path)
        while spec:
            parent_path = sdf_path.GetParentPath()
            parent = self._layer.pseudoRoot if parent_path == Sdf.Path.absoluteRootPath else self._layer.GetPrimAtPath(parent_path)
            if parent is None:
                break
            del parent.nameChildren[sdf_path.name]
            if parent_path == Sdf.Path.absoluteRootPath or len(parent.nameChildren) or not parent.IsInert():
                break
            sdf_path, spec = parent_path, parent

    # ------------------------------------------------------------------ Sublayer の適用・解除

    def apply(self) -> GeometricLayerReport:
        """Sublayer を差し込んで幾何モードを有効化する (適用済みなら何もしない)。

        PhysX は Articulation / RigidBody の有効無効を Play 開始時に読み込むため、
        タイムライン再生中に呼ばれた場合は安全のため一度 `stop()` してから適用する。
        """
        if self._layer is not None:
            return self._report
        self._stop_timeline("apply")
        opt = self._options
        if opt.layer_target == "root":
            layer = Sdf.Layer.FindOrOpen(opt.root_layer_file) or Sdf.Layer.CreateNew(opt.root_layer_file)
            layer.Clear()
            host = self._stage.GetRootLayer()
        else:
            layer = Sdf.Layer.CreateAnonymous("geometric_robot.usda")
            host = self._stage.GetSessionLayer()
        # `usd_reader.read_articulation` が幾何モード Sublayer を検出して無視できるよう目印を付ける
        layer.customLayerData = {LAYER_MARKER: True}
        self._report = GeometricLayerReport(layer.identifier)
        self._xform_specs = {}
        # 空の Sublayer を先に挿入しても合成結果は変わらないため、`_put` が記録する `before` は元の USD 値になる
        with authoring():
            host.subLayerPaths.insert(0, layer.identifier)
            self._layer = layer
            with Usd.EditContext(self._stage, layer):
                for entry in self._entries.values():
                    self._author(entry)
        self._release_physics()
        for callback in list(self._listeners):
            callback(True)
        return self._report

    def remove(self) -> bool:
        """Sublayer を外して元の Articulation 構成へ戻す。外したら `True`。Play 中なら停止してから外す。"""
        if self._layer is None:
            return False
        self._stop_timeline("remove")
        layer, self._layer = self._layer, None
        self._xform_specs = {}
        host = self._stage.GetRootLayer() if self._options.layer_target == "root" else self._stage.GetSessionLayer()
        paths = host.subLayerPaths
        with authoring():
            for ident in list(paths):
                if ident == layer.identifier or Sdf.Layer.Find(ident) == layer:
                    paths.remove(ident)
        self._release_physics()
        for callback in list(self._listeners):
            callback(False)
        return True

    @contextmanager
    def edit_context(self) -> Iterator[Sdf.Layer]:
        """編集ターゲットをこの Sublayer に切り替えるコンテキスト。中で書いた値は `remove()` でまとめて消える。"""
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない(先に apply() を呼ぶ)")
        with authoring(), Usd.EditContext(self._stage, self._layer):
            yield self._layer

    def export(self, path: str) -> None:
        """現在の Sublayer の内容を `.usda` 等へ書き出す (デバッグ確認用)。"""
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない")
        self._layer.Export(path)

    def save(self) -> None:
        """`layer_target="root"` で作成した非匿名 Sublayer をディスクへ保存する。"""
        if self._layer is not None and not self._layer.anonymous:
            self._layer.Save()

    # ------------------------------------------------------------------ リンク姿勢の書き込み先 (Sdf.AttributeSpec)

    def xform_specs(self, link_prim_paths: list[str]) -> list[Sdf.AttributeSpec]:
        """各リンクの `xformOp:transform:geometric` の `Sdf.AttributeSpec` を返す。

        `GeometricUsdWriter` は `Usd.Attribute.Set` の代わりに `spec.default = Gf.Matrix4d(...)` を
        `Sdf.ChangeBlock` 内で直接代入することで、Python-USD 間のオーバーヘッドを最小化している。
        """
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない")
        missing = [p for p in link_prim_paths if p not in self._xform_specs]
        if missing:
            with authoring(), Usd.EditContext(self._stage, self._layer):
                self._author_xforms(missing)
        return [self._xform_specs[p] for p in link_prim_paths]

    def _author_xforms(self, link_prim_paths: list[str]) -> None:
        """各リンクの `xformOpOrder` を `[xformOp:transform:geometric]` へ差し替える。

        なぜ先に `GetLocalTransformation` を読んで初期値にするか:
            元の `translate` / `orient` / `scale` 等の複合 op 列を単一の `Matrix4d` op へ置き換えるとき、
            置換前のローカル行列をそのまま初期値に入れておけば、`apply()` した瞬間にロボットが原点へ飛ばない。
        """
        assert self._layer is not None
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        locals_: dict[str, Gf.Matrix4d] = {}
        for path in link_prim_paths:
            prim = self._stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                raise ValueError(f"リンクの prim が無い: {path}")
            locals_[path] = cache.GetLocalTransformation(prim)[0]
        with Sdf.ChangeBlock():
            for path in link_prim_paths:
                prim_spec = Sdf.CreatePrimInLayer(self._layer, path)
                attr = prim_spec.attributes.get(XFORM_OP_NAME) or Sdf.AttributeSpec(prim_spec, XFORM_OP_NAME, Sdf.ValueTypeNames.Matrix4d)
                attr.default = locals_[path]
                order = prim_spec.attributes.get("xformOpOrder") or Sdf.AttributeSpec(prim_spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray, Sdf.VariabilityUniform)
                order.default = Vt.TokenArray([XFORM_OP_NAME])
                self._xform_specs[path] = attr

    # ------------------------------------------------------------------ 物理無効化・kinematic 化の上書き書き込み

    def _put(self, prim: Usd.Prim, attr: Usd.Attribute, value: Any) -> None:
        before = attr.Get() if attr and attr.IsValid() and attr.HasValue() else None
        attr.Set(value)
        self._report.changes.append((str(prim.GetPath()), attr.GetName(), before, value))

    def _author(self, entry: _Entry, scene_flags: bool = True) -> None:
        """`Usd.EditContext(stage, self._layer)` の内側で、物理無効化・姿勢 op 上書きを書き込む。"""
        # Pure OpenUSD 環境 (Kit / PhysxSchema なし) でも `GeometricLayer.apply()` が動くよう遅延・任意 import にする
        try:
            from pxr import PhysxSchema
        except ImportError:
            PhysxSchema = None

        stage, opt = self._stage, self._options
        kinematic = opt.collision_mode == "kinematic"
        disable_physx_colliders = opt.collision_mode == "none"
        all_links_set = set(entry.link_prim_paths)
        for path in entry.articulation_root_paths:
            prim = stage.GetPrimAtPath(path)
            if prim:
                if PhysxSchema is not None:
                    self._put(prim, PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateArticulationEnabledAttr(), False)
                else:
                    self._put(prim, prim.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool), False)
        for path in entry.joint_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsA(UsdPhysics.Joint):
                self._put(prim, UsdPhysics.Joint(prim).CreateJointEnabledAttr(), False)
        disabled_owners: list[str] = []
        for path in entry.link_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                self._report.warnings.append(f"リンクの prim が無い: {path}")
                continue
            has_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            if kinematic:
                body = UsdPhysics.RigidBodyAPI(prim) if has_body else UsdPhysics.RigidBodyAPI.Apply(prim)
                self._put(prim, body.CreateRigidBodyEnabledAttr(), True)
                self._put(prim, body.CreateKinematicEnabledAttr(), True)
                if opt.report_contacts and PhysxSchema is not None:
                    self._put(prim, PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(), opt.contact_threshold)
            elif has_body:
                self._put(prim, UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(), False)
            if disable_physx_colliders:
                # `collision_mode="none"`: PhysX 側の静的/kinematic コライダーを完全に無くし、
                # PhysX 広域・狭域計算および Show by Type のピンク表示負荷をゼロにする。
                # Newton バックエンド側は `is_collision_enabled_ignoring_geometric_layer()` で
                # この Sublayer 上書きを無視して元のコライダー定義をそのまま読み取る。
                # UR10e 等では `collisions` が instanceable になっているため、CollisionAPI を内包する
                # instance ルートのみ Session Sublayer 上で一時的に `instanceable=False` に展開してから
                # `physics:collisionEnabled=False` を上書きする (`remove()` で元の instanceable 状態に完全復帰する)。
                while True:
                    inst_to_expand: list[str] = []
                    it_inst = iter(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
                    for sub in it_inst:
                        sub_path = str(sub.GetPath())
                        if sub_path != path and sub_path in all_links_set:
                            it_inst.PruneChildren()
                            continue
                        if sub.IsInstance():
                            has_col = False
                            sub_it = iter(Usd.PrimRange(sub, Usd.TraverseInstanceProxies()))
                            for desc in sub_it:
                                dp = str(desc.GetPath())
                                if dp != path and dp in all_links_set:
                                    sub_it.PruneChildren()
                                    continue
                                if desc.HasAPI(UsdPhysics.CollisionAPI):
                                    has_col = True
                                    break
                            if has_col:
                                inst_to_expand.append(sub_path)
                            it_inst.PruneChildren()
                    if not inst_to_expand:
                        break
                    for inst_path in inst_to_expand:
                        inst_prim = stage.GetPrimAtPath(inst_path)
                        if inst_prim and inst_prim.IsValid() and inst_prim.IsInstance():
                            inst_prim.SetInstanceable(False)
                            self._report.changes.append((inst_path, "instanceable", True, False))
                            disabled_owners.append(inst_path)
                it = iter(Usd.PrimRange(prim))
                for sub in it:
                    sub_path = str(sub.GetPath())
                    if sub_path != path and sub_path in all_links_set:
                        it.PruneChildren()
                        continue
                    if sub.IsInstanceProxy():
                        continue
                    if sub.HasAPI(UsdPhysics.CollisionAPI):
                        self._put(sub, UsdPhysics.CollisionAPI(sub).CreateCollisionEnabledAttr(), False)
                        disabled_owners.append(sub_path)
        if disabled_owners:
            entry.collider_owner_paths = list(dict.fromkeys(entry.collider_owner_paths + disabled_owners))
        if kinematic:
            # なぜ `FilteredPairsAPI` を書き足すか:
            # PhysX では有効な `UsdPhysics.Joint` が両端の隣接リンク間の衝突を暗黙に除外している。
            # `jointEnabled=false` にするとこの暗黙除外が消えてしまい、`kinematic` モードで隣接リンク同士が
            # 常に接触報告を出し続けるため、同じ隣接ペアを `FilteredPairsAPI` として Sublayer に明示記述する。
            for a, b in entry.adjacent_link_paths:
                prim = stage.GetPrimAtPath(a)
                if prim and stage.GetPrimAtPath(b):
                    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().AddTarget(b)
                    self._report.changes.append((a, "physics:filteredPairs", None, f"+{b}"))
            if opt.report_contacts and scene_flags and PhysxSchema is not None:
                scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
                if not scenes:
                    self._report.warnings.append("PhysicsScene が無いので、kinematic 同士の接触の報告フラグを書けなかった")
                for prim in scenes:
                    api = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    self._put(prim, api.CreateReportKinematicKinematicPairsAttr(), True)
                    self._put(prim, api.CreateReportKinematicStaticPairsAttr(), True)
        self._author_xforms([p for p in entry.link_prim_paths if stage.GetPrimAtPath(p)])

    # ------------------------------------------------------------------ Kit 依存処理 (Kit 外では無動作)

    def _stop_timeline(self, what: str) -> None:
        """再生中のタイムラインを停止する (Kit 外では `ImportError` を握りつぶしてスキップ)。"""
        try:
            import carb
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            if not timeline.is_stopped():
                carb.log_warn(f"[myext.robot.geometric_newton] GeometricLayer.{what}(): タイムラインを止めた(物理の設定は Play 開始時に読まれるため)")
                timeline.stop()
        except ImportError:
            pass

    def _release_physics(self) -> None:
        """`Stop` 中に PhysX がキャッシュ済みの物理オブジェクトを破棄し、次回チェックで最新 Sublayer を再読込させる。"""
        try:
            import omni.timeline
            from omni.physx import get_physx_interface

            if omni.timeline.get_timeline_interface().is_stopped():
                get_physx_interface().release_physics_objects()
        except ImportError:
            pass
