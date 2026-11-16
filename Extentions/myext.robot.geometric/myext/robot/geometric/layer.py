"""GeometricLayer: 幾何モード用の Sublayer。付けると幾何モード、外すと元の構成に戻る。

上書きはすべてこの Sublayer にだけ書く。元のロボットの USD(Root Layer や参照先のファイル)は一度も書き換えない。

    Session Layer
      └─ GeometricLayer(匿名)  ← 既定。保存しても元ファイルに混ざらない
    Root Layer

書く内容(既存の Articulation から作ったロボットの場合):
    ArticulationRoot の prim   physxArticulation:articulationEnabled = false
    各 Joint                   physics:jointEnabled = false
    各リンク(static)           physics:rigidBodyEnabled = false          → 静的コライダーとして残る
    各リンク(kinematic)        physics:kinematicEnabled = true、ContactReportAPI(threshold 0)、隣のリンクへの FilteredPairsAPI
    PhysicsScene(kinematic)    reportKinematicKinematicPairs / reportKinematicStaticPairs = true
    各リンク                   xformOpOrder = [xformOp:transform:geometric] と、その行列(リンクの姿勢の書き込み先)

CollisionAPI とコリジョン形状には触らない。
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Literal, get_args

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, Vt

__all__ = ["CollisionMode", "LayerTarget", "GeometricLayerOptions", "GeometricLayerReport", "GeometricLayer", "XFORM_OP_NAME", "LAYER_MARKER", "SETTINGS_ROOT"]

CollisionMode = Literal["static", "kinematic"]
LayerTarget = Literal["session", "root"]

XFORM_OP_NAME = "xformOp:transform:geometric"
LAYER_MARKER = "myext.robot.geometric"
SETTINGS_ROOT = "/exts/myext.robot.geometric"


def _setting(key: str, default: Any) -> Any:
    """拡張機能の設定を読む。Kit の外(carb が無い)や未設定のときは default。"""
    try:
        import carb.settings

        value = carb.settings.get_settings().get(f"{SETTINGS_ROOT}/{key}")
        return default if value is None else value
    except Exception:
        return default


@dataclass(frozen=True)
class GeometricLayerOptions:
    """Sublayer の作り方。

    collision_mode    : "static"    = リンクを静的コライダーにする。Play しなくても overlap で干渉を調べられる
                        "kinematic" = リンクを kinematic 剛体にする。Play 中に Contact Report が使える
    layer_target      : "session" = Session Layer の下に差す(保存しても元ファイルに混ざらない)
                        "root"    = Root Layer の下に差す(ステージを保存すると Sublayer への参照が残る)。root_layer_file が必要
    report_contacts   : kinematic のとき、リンクに ContactReportAPI を付け、シーンの報告フラグを立てるか
    contact_threshold : ContactReportAPI の threshold
    root_layer_file   : layer_target="root" のときの Sublayer のファイルパス(.usda)
    """

    collision_mode: CollisionMode = "static"
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
        """拡張機能の設定(/exts/myext.robot.geometric/collision_mode, layer_target, root_layer_file)から作る。"""
        return cls(
            collision_mode=_setting("collision_mode", "static"),
            layer_target=_setting("layer_target", "session"),
            root_layer_file=_setting("root_layer_file", None) or None,
        )


@dataclass
class GeometricLayerReport:
    """apply() が Sublayer に書いた上書きの一覧。changes は (prim パス, 名前, 前に見えていた値, 書いた値)。"""

    layer_identifier: str = ""
    changes: list[tuple[str, str, Any, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class _Entry:
    """Sublayer が受け持つロボット 1 台分。"""

    articulation_root_paths: list[str]
    joint_prim_paths: list[str]
    link_prim_paths: list[str]
    adjacent_link_paths: list[tuple[str, str]]  # 関節でつながったリンクの prim パスの組(kinematic の FilteredPairs 用)


class GeometricLayer:
    def __init__(self, stage: Usd.Stage, options: GeometricLayerOptions | None = None) -> None:
        self._stage = stage
        self._options = options or GeometricLayerOptions.from_settings()
        self._entries: list[_Entry] = []
        self._layer: Sdf.Layer | None = None
        self._report = GeometricLayerReport()
        self._listeners: list[Callable[[bool], None]] = []
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
        """付いている間だけ有効な Sdf.Layer。外れているときは None。"""
        return self._layer

    def is_applied(self) -> bool:
        return self._layer is not None

    def add_listener(self, callback: Callable[[bool], None]) -> None:
        """付けた直後に callback(True)、外した直後に callback(False) が呼ばれる。"""
        self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[bool], None]) -> None:
        if callback in self._listeners:
            self._listeners.remove(callback)

    def register(
        self,
        *,
        link_prim_paths: list[str],
        articulation_root_paths: list[str] | None = None,
        joint_prim_paths: list[str] | None = None,
        adjacent_link_paths: list[tuple[str, str]] | None = None,
    ) -> None:
        """この Sublayer が受け持つロボットを足す。すでに付いているときは、その場で上書きを書き足す。"""
        entry = _Entry(list(articulation_root_paths or []), list(joint_prim_paths or []), list(link_prim_paths), list(adjacent_link_paths or []))
        self._entries.append(entry)
        if self._layer is not None:
            with Usd.EditContext(self._stage, self._layer):
                self._author(entry)
            self._release_physics()

    # ------------------------------------------------------------------ 付ける・外す

    def apply(self) -> GeometricLayerReport:
        """幾何モードにする。すでに付いていれば何もしない。Play 中なら止めてから付ける(物理の設定は Play 開始時に読まれるため)。"""
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
        layer.customLayerData = {LAYER_MARKER: True}
        self._report = GeometricLayerReport(layer.identifier)
        self._xform_specs = {}
        # 空のレイヤーを差しても見え方は変わらないので、_put の「前の値」は元の値になる
        host.subLayerPaths.insert(0, layer.identifier)
        self._layer = layer
        with Usd.EditContext(self._stage, layer):
            for entry in self._entries:
                self._author(entry)
        self._release_physics()
        for callback in list(self._listeners):
            callback(True)
        return self._report

    def remove(self) -> bool:
        """Sublayer を外して元の構成に戻す。外したら True。Play 中なら止めてから外す。"""
        if self._layer is None:
            return False
        self._stop_timeline("remove")
        layer, self._layer = self._layer, None
        self._xform_specs = {}
        host = self._stage.GetRootLayer() if self._options.layer_target == "root" else self._stage.GetSessionLayer()
        paths = host.subLayerPaths
        for ident in list(paths):
            if ident == layer.identifier or Sdf.Layer.Find(ident) == layer:
                paths.remove(ident)
        self._release_physics()
        for callback in list(self._listeners):
            callback(False)
        return True

    @contextmanager
    def edit_context(self) -> Iterator[Sdf.Layer]:
        """この Sublayer を編集先にする。中で書いた値は remove() で一緒に消える。"""
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない(先に apply() を呼ぶ)")
        with Usd.EditContext(self._stage, self._layer):
            yield self._layer

    def export(self, path: str) -> None:
        """いまの Sublayer の内容をファイルに書き出す(確認用)。"""
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない")
        self._layer.Export(path)

    def save(self) -> None:
        """layer_target="root" のとき、Sublayer のファイルを保存する。"""
        if self._layer is not None and not self._layer.anonymous:
            self._layer.Save()

    # ------------------------------------------------------------------ リンクの姿勢の書き込み先

    def xform_specs(self, link_prim_paths: list[str]) -> list[Sdf.AttributeSpec]:
        """各リンクの「xformOp:transform:geometric」の属性(Sdf)。書き込みは spec.default = Gf.Matrix4d(...)。"""
        if self._layer is None:
            raise RuntimeError("Sublayer が付いていない")
        missing = [p for p in link_prim_paths if p not in self._xform_specs]
        if missing:
            with Usd.EditContext(self._stage, self._layer):
                self._author_xforms(missing)
        return [self._xform_specs[p] for p in link_prim_paths]

    def _author_xforms(self, link_prim_paths: list[str]) -> None:
        assert self._layer is not None
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        locals_: dict[str, Gf.Matrix4d] = {}
        for path in link_prim_paths:  # 上書きする前に、いまのローカル行列を読んでおく(付けた瞬間に姿勢が変わらないように)
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

    # ------------------------------------------------------------------ 上書きを書く

    def _put(self, prim: Usd.Prim, attr: Usd.Attribute, value: Any) -> None:
        before = attr.Get() if attr and attr.IsValid() and attr.HasValue() else None
        attr.Set(value)
        self._report.changes.append((str(prim.GetPath()), attr.GetName(), before, value))

    def _author(self, entry: _Entry) -> None:
        """編集先がこの Sublayer になっている状態で呼ぶ。"""
        stage, opt = self._stage, self._options
        kinematic = opt.collision_mode == "kinematic"
        for path in entry.articulation_root_paths:
            prim = stage.GetPrimAtPath(path)
            if prim:
                self._put(prim, PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateArticulationEnabledAttr(), False)
        for path in entry.joint_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsA(UsdPhysics.Joint):
                self._put(prim, UsdPhysics.Joint(prim).CreateJointEnabledAttr(), False)
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
                if opt.report_contacts:
                    self._put(prim, PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(), opt.contact_threshold)
            elif has_body:
                self._put(prim, UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(), False)
        if kinematic:
            # Joint を無効にすると「つながった 2 リンクは衝突しない」除外も消えるので、同じ除外を書き直す
            for a, b in entry.adjacent_link_paths:
                prim = stage.GetPrimAtPath(a)
                if prim and stage.GetPrimAtPath(b):
                    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().AddTarget(b)
                    self._report.changes.append((a, "physics:filteredPairs", None, f"+{b}"))
            if opt.report_contacts:
                scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
                if not scenes:
                    self._report.warnings.append("PhysicsScene が無いので、kinematic 同士の接触の報告フラグを書けなかった")
                for prim in scenes:
                    api = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    self._put(prim, api.CreateReportKinematicKinematicPairsAttr(), True)
                    self._put(prim, api.CreateReportKinematicStaticPairsAttr(), True)
        self._author_xforms([p for p in entry.link_prim_paths if stage.GetPrimAtPath(p)])

    # ------------------------------------------------------------------ Kit があるときだけの処理

    def _stop_timeline(self, what: str) -> None:
        try:
            import carb
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            if not timeline.is_stopped():
                carb.log_warn(f"[myext.robot.geometric] GeometricLayer.{what}(): タイムラインを止めた(物理の設定は Play 開始時に読まれるため)")
                timeline.stop()
        except ImportError:
            pass

    def _release_physics(self) -> None:
        """Stop 中に PhysX へ読み込み済みのシーンがあれば捨てる(次の干渉チェックで、いまの設定のまま読み直させる)。"""
        try:
            import omni.timeline
            from omni.physx import get_physx_interface

            if omni.timeline.get_timeline_interface().is_stopped():
                get_physx_interface().release_physics_objects()
        except ImportError:
            pass
