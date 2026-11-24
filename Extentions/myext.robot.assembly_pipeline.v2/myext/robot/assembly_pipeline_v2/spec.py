"""パイプラインの入力仕様(データクラス群)と TOML パーサ。

責務:
  - ロボット組み立て仕様を表す不変データクラス(`PipelineSpec` とその構成要素)を定義する。
  - TOML ファイルから `PipelineSpec` を構築し、Isaac Sim / pxr を使わずに検証できる
    スキーマチェック（ファイル名規則、VariantSet 整合、Windows 大文字小文字衝突、結合 DAG の循環検出）をすべて実行する。
  - 依存順に並べた結合ファイル列(`file_order`)や、スコープ付きリンク名の所有先解決(`resolve_link`)を提供する。

主な入出力・公開 API:
  - 入力: TOML ファイルパス → `load_spec(path) -> PipelineSpec`
  - データクラス: `UrdfSource`, `UsdSource`, `JoinSpec`, `JoinFileSpec`, `RuleSetting`, `ProbeSettings`, `PipelineSpec`
  - 定数・補助: `DEFAULT_RULES`, `ALLOWED_TOP`, `join_id(f, j)`

呼ばれ元:
  - `__init__.py`（公開 API として再エクスポート）
  - `spec_check.py`, `part_import.py`, `compose.py`, `official.py`, `probes.py`,
    `kinematics_check.py`, `rules.py`, `pipeline.py`, `run_pipeline.py`

用語(この拡張内では同一の意味に統一して使う):
  - usda        : defaultPrim を持つ USD ファイル。URDF / USD から作った正規化 usda、または結合ファイル。
  - 結合        : 子の usda を親の usda のリンクに固定する 1 回の操作(`compose.py` の書き込み処理)。
  - 結合ファイル: 親の usda に 1 回以上の結合を書いた新しい usda。さらに別の結合の親にも子にもできる。
  - 公式のまま  : URDF Importer の出力を Robot Assembler でそのまま結合した場合(比較検証用)。
  - 修正後      : このパイプラインが出力した正規化 usda / 結合ファイル。
  - 修正        : 既知の不具合を回避するためにパイプラインが適用する補正。

usda はすべて `output_dir` に出力するファイル名(拡張子 `.usda` まで)で一意に識別する:
  - 作る側 : `[[urdf_to_usda]]` の `usda`、`[[usd_to_usda]]` の `usda`、`[[join_file]]` の `usda`
  - 使う側 : `[[join_file]]` の `parent_usda`、`[[join_file.join]]` の `child_usda`
  TOML の全キー仕様は `docs/README.md` を参照。

拡張・変更ポイント:
  - TOML に新しいオプションを追加する場合は、対象 dataclass へのフィールド追加と `load_spec` の
    キー検証(`_keys`)・変換処理を対で更新する。
"""

from __future__ import annotations

import os
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import TypeVar

from .kinds import RULE_IDS, FixMode, OnViolation, PhysicsVariant, RuleId, check_literal

T = TypeVar("T")


class _CallableDict(dict[str, T]):
    """辞書添字(`obj[key]`)と関数呼び出し(`obj(key) -> value | None`)の両方を受け付けるラッパー。

    なぜ必要か:
      `spec.source_by_usda` / `spec.join_file_by_usda` を辞書として走査する呼び出し側と、
      `spec.source_by_usda("arm.usda")` のように関数スタイルで引く呼び出し側の両方を壊さずに支えるため。
    """

    def __call__(self, key: str) -> T | None:
        return self.get(key)


class _CallableList(list[T]):
    """リスト操作と引数なし関数呼び出し(`obj() -> list`)の両方を受け付けるラッパー。

    なぜ必要か:
      `spec.all_usda_names` をリスト属性として読む経路と、メソッド呼び出しスタイルで読む経路の両立のため。
    """

    def __call__(self) -> list[T]:
        return list(self)


def _check_usda_name(value: object, where: str) -> str:
    """usda ファイル名が `output_dir` 直下の `.usda` かつ USD prim 名に使える識別子かを検証する。

    なぜ識別子に制限するか:
      `.usda` を除いた幹名(`stem`)をそのまま USD の `defaultPrim` 名や作業ディレクトリ名に使うため、
      空白・ハイフン・先頭数字などを事前に弾いて USD パス構文エラーを防ぐ。
    """
    if not isinstance(value, str) or not value.endswith(".usda") or os.path.basename(value) != value:
        raise ValueError(f"{where}: usda のファイル名は、フォルダを含まず拡張子 .usda まで書く(例 \"arm.usda\"): {value!r}")
    stem = value[: -len(".usda")]
    if not stem.isidentifier():
        raise ValueError(f"{where}: usda のファイル名の .usda より前は、prim 名に使うので英数字と _ だけにする(先頭は数字以外): {value!r}")
    return value


@dataclass(frozen=True)
class UrdfSource:
    """[[urdf_to_usda]] 1 つ = URDF 1 本から usda を 1 つ作る。

    urdf                  : 入力。URDF ファイルのパス
    usda                  : 出力。output_dir に作る usda のファイル名(例 "arm.usda")。ほかの場所ではこの名前で指す
    collision_from_visuals: URDF Importer の同名の設定。None なら「URDF に collision が 1 つでもあれば False、無ければ True」
    collision_type        : collision_from_visuals のときの近似
    physics_variant       : 使う Physics variant
    ros_package_paths     : package:// の解決先。((パッケージ名, フォルダ), ...)
    joint_drive_stiffness : URDF Importer の override_joint_stiffness(全関節。mimic の follower にも入るので、修正で follower は 0 に戻す = R03)
    joint_drive_damping   : URDF Importer の override_joint_damping(同上)
    fix_base              : URDF Importer の fix_base。None なら Importer 既定、False で floating-base
    """

    urdf: str
    usda: str
    collision_from_visuals: bool | None = None
    collision_type: str = "Convex Hull"
    physics_variant: PhysicsVariant = "auto"
    ros_package_paths: tuple[tuple[str, str], ...] = ()
    joint_drive_stiffness: float | None = None
    joint_drive_damping: float | None = None
    fix_base: bool | None = None

    def __post_init__(self) -> None:
        _check_usda_name(self.usda, "[[urdf_to_usda]] の usda")
        check_literal(self.physics_variant, PhysicsVariant, f"[[urdf_to_usda]] usda = {self.usda!r} の physics_variant")
        if self.fix_base is not None and not isinstance(self.fix_base, bool):
            raise ValueError(f"[[urdf_to_usda]] usda = {self.usda!r} の fix_base は bool または None にする: {self.fix_base!r}")

    # 中のコードで使う読み替え
    @property
    def name(self) -> str:
        return self.usda

    @property
    def path(self) -> str:
        return self.urdf

    @property
    def stem(self) -> str:
        return self.usda[: -len(".usda")]


@dataclass(frozen=True)
class UsdSource:
    """[[usd_to_usda]] 1 つ = 既存の USD アセット 1 つから正規化 usda を 1 つ作る。

    usd                : 入力。既存 USD のローカルパスまたは URI("://" を含む)
    usda               : 出力。output_dir に作る usda のファイル名(例 "ur10e_base.usda")
    prim_path          : 参照する prim パス(省略時は defaultPrim。"/" で始める)
    variant_selections : ソース USD の VariantSet 選択 ((VariantSet 名, 選択肢), ...)
    physics_variant    : 使う Physics variant(自由文字列。既定 "auto")
    """

    usd: str
    usda: str
    prim_path: str | None = None
    variant_selections: tuple[tuple[str, str], ...] = ()
    physics_variant: str = "auto"

    def __post_init__(self) -> None:
        _check_usda_name(self.usda, "[[usd_to_usda]] の usda")
        if not isinstance(self.usd, str) or not self.usd.strip():
            raise ValueError(f"[[usd_to_usda]] usda = {self.usda!r} の usd は空でない文字列にする: {self.usd!r}")
        if self.prim_path is not None:
            if not isinstance(self.prim_path, str) or not self.prim_path.startswith("/"):
                raise ValueError(
                    f"[[usd_to_usda]] usda = {self.usda!r} の prim_path は '/' で始める: {self.prim_path!r}"
                )
        if isinstance(self.variant_selections, Mapping):
            object.__setattr__(
                self,
                "variant_selections",
                tuple((str(k), str(v)) for k, v in self.variant_selections.items()),
            )
        else:
            object.__setattr__(
                self,
                "variant_selections",
                tuple((str(k), str(v)) for k, v in self.variant_selections),
            )
        if not isinstance(self.physics_variant, str) or not self.physics_variant.strip():
            raise ValueError(
                f"[[usd_to_usda]] usda = {self.usda!r} の physics_variant は空でない文字列にする: {self.physics_variant!r}"
            )

    @property
    def name(self) -> str:
        return self.usda

    @property
    def path(self) -> str:
        return self.usd

    @property
    def stem(self) -> str:
        return self.usda[: -len(".usda")]


@dataclass(frozen=True)
class JoinSpec:
    """[[join_file.join]] 1 つ = 結合 1 回。

    name                   : この結合の名前(この結合ファイルの中で一意)。使われる場所: 報告の表示(「結合ファイル/結合の名前」)、
                             child_prim_name の既定値、この結合ファイルを親にした後の結合でのリンクの前置き("arm1/flange")
    child_usda             : 入力。子の usda のファイル名([[urdf_to_usda]] か [[join_file]] の usda のどれか)
    parent_link            : 親の usda のリンク名。子のリンク child_link の座標系をここに一致させる
    child_link             : 子の usda のリンク名
    place_child_under      : 子を Reference で置く親側の prim。リンク名、"."(この結合ファイルの一番上の prim)、
                             または "/" で始まる prim のパス。省略時は parent_link の prim(stage の親子が実際のリンクの親子に揃う)
    child_prim_name        : 出力。子を置く prim の名前(省略時は name)。VariantSet の選択肢どうしで同じにすると、どれを選んでもパスが同じ
    offset_xyz             : parent_link の座標系で見た child_link の位置(m)
    offset_rpy             : 同じく向き(rad、URDF と同じ固定軸 XYZ)
    variant_set            : 出力。この結合を選択肢にする VariantSet の名前(この結合ファイルの一番上の prim に付く)
    variant                : 出力。その選択肢の名前。同じ variant_set の結合は排他で、最初のものが既定の選択。"None" は自動で足す
    child_variant_selection: 子の usda の VariantSet の選択 ((VariantSet の名前, 選択肢の名前), ...)。
                             VariantSet の名前・選択肢の名前は、子の usda を作った [[join_file.join]] の variant_set・variant
    """

    name: str
    child_usda: str
    parent_link: str
    child_link: str
    place_child_under: str | None = None
    child_prim_name: str | None = None
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    variant_set: str | None = None
    variant: str | None = None
    child_variant_selection: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        where = f"[[join_file.join]] name = {self.name!r}"
        if not self.name.isidentifier():
            raise ValueError(f"{where}: name は英数字と _ だけにする(先頭は数字以外。リンクの前置きと prim 名に使う)")
        _check_usda_name(self.child_usda, f"{where} の child_usda")
        if (self.variant_set is None) != (self.variant is None):
            raise ValueError(f"{where}: variant_set と variant は両方書くか、両方書かない")
        if self.variant == "None":
            raise ValueError(f"{where}: variant の名前 None は予約済み(何も結合しない選択肢として自動で足す)")
        if not self.prim.isidentifier():
            raise ValueError(f"{where}: child_prim_name は英数字と _ だけにする: {self.prim!r}")
        if isinstance(self.child_variant_selection, Mapping):
            object.__setattr__(
                self,
                "child_variant_selection",
                tuple((str(k), str(v)) for k, v in self.child_variant_selection.items()),
            )

    # 中のコードで使う読み替え
    @property
    def child(self) -> str:
        return self.child_usda

    @property
    def parent_prim(self) -> str | None:
        return self.place_child_under

    @property
    def prim(self) -> str:
        return self.child_prim_name or self.name

    @property
    def select(self) -> tuple[tuple[str, str], ...]:
        return self.child_variant_selection


@dataclass(frozen=True)
class JoinFileSpec:
    """`[[join_file]]` 1 件分 = 結合ファイルを 1 つ作る仕様。

    属性:
      usda            : 出力。`output_dir` に作る結合ファイルのファイル名(例: `"arm_unit.usda"`)。
      parent_usda     : 入力。親となる usda のファイル名(`[[urdf_to_usda]]` / `[[usd_to_usda]]` / 他の `[[join_file]]`)。
      joins           : この結合ファイルで行う `[[join_file.join]]` のタプル。
      variant_defaults: VariantSet ごとの既定選択肢 `((VariantSet名, 選択肢名), ...)`。
                        未指定の VariantSet は最初に定義された結合の `variant` が既定になる。
      parent_prim_name: 出力(任意)。親 `parent_usda` を結合ファイル最上位 prim の 1 段下
                        (`/<usda stem>/<parent_prim_name>`)に子 prim として配置するときの prim 名。
                        省略(`None`)時は従来どおり最上位 prim へ直接 Reference 合成する。
    """

    usda: str
    parent_usda: str
    joins: tuple[JoinSpec, ...] = ()
    variant_defaults: tuple[tuple[str, str], ...] = ()
    parent_prim_name: str | None = None

    def __post_init__(self) -> None:
        _check_usda_name(self.usda, "[[join_file]] の usda")
        _check_usda_name(self.parent_usda, f"[[join_file]] usda = {self.usda!r} の parent_usda")
        if self.parent_prim_name is not None:
            if not isinstance(self.parent_prim_name, str) or not self.parent_prim_name.isidentifier():
                raise ValueError(
                    f"[[join_file]] usda = {self.usda!r} の parent_prim_name は英数字と _ だけにする(先頭は数字以外): "
                    f"{self.parent_prim_name!r}"
                )
            for j in self.joins:
                if j.place_child_under == "." and j.prim == self.parent_prim_name:
                    raise ValueError(
                        f"[[join_file]] usda = {self.usda!r}: parent_prim_name = {self.parent_prim_name!r} が "
                        f"place_child_under = '.' の結合 {j.name!r} (child_prim_name = {j.prim!r}) と衝突している"
                    )
        names = [j.name for j in self.joins]
        dup = sorted({n for n in names if names.count(n) > 1})
        if dup:
            raise ValueError(f"[[join_file]] usda = {self.usda!r}: [[join_file.join]] の name が重複している: {dup}")
        # frozen dataclass のため、辞書またはイテラブルで渡された variant_defaults をタプルへ正規化する。
        if isinstance(self.variant_defaults, Mapping):
            object.__setattr__(
                self,
                "variant_defaults",
                tuple((str(k), str(v)) for k, v in self.variant_defaults.items()),
            )
        else:
            object.__setattr__(
                self,
                "variant_defaults",
                tuple((str(k), str(v)) for k, v in self.variant_defaults),
            )
        # 指定された既定選択が、実際にこの [[join_file]] で作る VariantSet と選択肢(または自動生成される "None")に存在するか検証する。
        if self.variant_defaults:
            vsets: dict[str, list[str]] = {}
            for j in self.joins:
                if j.variant_set and j.variant:
                    vsets.setdefault(j.variant_set, []).append(j.variant)
            for vs_name, choice in self.variant_defaults:
                if vs_name not in vsets:
                    raise ValueError(
                        f"[[join_file]] usda = {self.usda!r} の variant_defaults: "
                        f"VariantSet {vs_name!r} が無い。ある VariantSet: {sorted(vsets)}"
                    )
                allowed = vsets[vs_name] + ["None"]
                if choice not in allowed:
                    raise ValueError(
                        f"[[join_file]] usda = {self.usda!r} の variant_defaults: "
                        f"VariantSet {vs_name!r} に選択肢 {choice!r} が無い。ある選択肢: {allowed}"
                    )

    @property
    def name(self) -> str:
        return self.usda

    @property
    def parent(self) -> str:
        return self.parent_usda

    @property
    def stem(self) -> str:
        return self.usda[: -len(".usda")]

    def join(self, name: str) -> JoinSpec:
        """結合名 `name` に一致する `JoinSpec` を返す。見つからなければ `StopIteration`。"""
        return next(j for j in self.joins if j.name == name)

    def default_variant(self, variant_set: str) -> str | None:
        """`variant_set` の既定選択肢を返す(`variant_defaults` 優先、未指定時は先頭の結合の `variant`)。"""
        for k, v in self.variant_defaults:
            if k == variant_set:
                return v
        return next((j.variant for j in self.joins if j.variant_set == variant_set and j.variant), None)


@dataclass(frozen=True)
class RuleSetting:
    """個別規則(`[checks.rules.<規則>]`)の上書き設定。`None` の項目は `[checks]` の全体設定を継承する。"""

    on_violation: OnViolation | None = None
    fix: FixMode | None = None


DEFAULT_RULES: dict[str, RuleSetting] = {
    rid: RuleSetting(on_violation="continue", fix="apply") for rid in RULE_IDS
}


@dataclass(frozen=True)
class ProbeSettings:
    """`[probe]` セクションで指定する事後検証プローブの条件。

    属性:
      steps            : PhysX を Play するステップ数。
      drive_stiffness  : 関節を動かす drive の剛性(回転は 1 deg あたり、直動は 1 m あたり)。
      drive_damping    : 同じく減衰。
      revolute_target  : 回転関節の目標角(rad。可動範囲の 8 割にクランプされる)。
      prismatic_ratio  : 直動関節の目標位置(可動範囲の何割か)。
      mimic_leader_*   : mimic の leader を動かす柔らかい drive(follower の不要 drive による引きずりを顕在化させるため弱くする)。
      mimic_steps      : mimic プローブで進めるステップ数。
      angle_tol_deg    : 角度誤差の許容値(deg)。
      length_tol_m     : 位置誤差の許容値(m)。
    """

    steps: int = 240
    drive_stiffness: float = 1.0e4
    drive_damping: float = 1.0e3
    revolute_target: float = 0.5
    prismatic_ratio: float = 0.5
    mimic_leader_stiffness_linear: float = 20.0
    mimic_leader_damping_linear: float = 2.0
    mimic_leader_stiffness_angular: float = 0.05
    mimic_leader_damping_angular: float = 0.005
    mimic_steps: int = 300
    angle_tol_deg: float = 1.0
    length_tol_m: float = 1.0e-3


@dataclass(frozen=True)
class PipelineSpec:
    """パイプライン 1 回分の全仕様を表すルートデータクラス。

    生成時(`__post_init__`)に、規則設定の妥当性・出力ファイル名重複・Windows の大文字小文字衝突・
    未定義の `parent_usda`/`child_usda` 参照・結合 DAG の循環をすべて検証する。
    """

    output_dir: str
    urdfs: tuple[UrdfSource, ...] = ()
    join_files: tuple[JoinFileSpec, ...] = ()
    on_violation: OnViolation = "continue"
    fix: FixMode = "apply"
    rules: dict[str, RuleSetting] = field(default_factory=dict)
    probe: ProbeSettings = ProbeSettings()
    compare_official: bool = True  # 公式のまま(Robot Assembler)でも結合して症状を測る
    check_kinematics: bool = True  # 結合ファイルごとに順運動学と PhysX を突き合わせる
    usds: tuple[UsdSource, ...] = ()

    def __post_init__(self) -> None:
        check_literal(self.on_violation, OnViolation, "[checks] の on_violation")
        check_literal(self.fix, FixMode, "[checks] の fix")
        for rid, s in self.rules.items():
            check_literal(rid, RuleId, "[checks.rules.<規則>] の <規則>")
            if s.on_violation is not None:
                check_literal(s.on_violation, OnViolation, f"[checks.rules.{rid}] の on_violation")
            if s.fix is not None:
                check_literal(s.fix, FixMode, f"[checks.rules.{rid}] の fix")
        # 出力 usda 名の重複を全セクション横断で検出する。
        made: dict[str, str] = {}
        for i, u in enumerate(self.urdfs, 1):
            self._made(made, u.usda, f"[[urdf_to_usda]] {i} 番目")
        for i, u in enumerate(self.usds, 1):
            self._made(made, u.usda, f"[[usd_to_usda]] {i} 番目")
        for i, f in enumerate(self.join_files, 1):
            self._made(made, f.usda, f"[[join_file]] {i} 番目")
        # Windows ファイルシステムは大文字小文字を区別しないため、小文字化した幹名の衝突も事前に禁止する。
        stems: dict[str, str] = {}
        for name in made:
            stem = name[: -len(".usda")].lower()
            if stem in stems:
                raise ValueError(f"usda のファイル名 {stems[stem]!r} と {name!r} は大文字と小文字しか違わない(Windows では同じファイルになる)")
            stems[stem] = name
        # すべての parent_usda / child_usda が同一 TOML 内で生成されることを確認する。
        known = sorted(made)
        for f in self.join_files:
            if f.parent_usda not in made:
                raise ValueError(f"[[join_file]] usda = {f.usda!r} の parent_usda = {f.parent_usda!r} は、この TOML のどこでも作っていない。"
                                 f"作っている usda: {known}")
            for j in f.joins:
                if j.child_usda not in made:
                    raise ValueError(f"[[join_file]] usda = {f.usda!r} の [[join_file.join]] name = {j.name!r} の child_usda = {j.child_usda!r} は、"
                                     f"この TOML のどこでも作っていない。作っている usda: {known}")
        # トポロジカルソートを試行して循環参照(A -> B -> A)を早期に弾く。
        self.file_order()

    @staticmethod
    def _made(made: dict[str, str], usda: str, where: str) -> None:
        if usda in made:
            raise ValueError(f"usda = {usda!r} を 2 か所で作ろうとしている: {made[usda]} と {where}。出力のファイル名は 1 つずつ別にする")
        made[usda] = where

    @property
    def check_geometric(self) -> bool:
        """互換用プロパティ(`check_kinematics` の別名)。"""
        return self.check_kinematics

    # ---------------------------------------------------------------- 参照・検索ヘルパー
    def urdf(self, usda: str) -> UrdfSource | None:
        """`usda` 名に一致する `UrdfSource` を返す(無ければ `None`)。"""
        return next((u for u in self.urdfs if u.usda == usda), None)

    def usd(self, usda: str) -> UsdSource | None:
        """`usda` 名に一致する `UsdSource` を返す(無ければ `None`)。"""
        return next((u for u in self.usds if u.usda == usda), None)

    def leaf_source(self, usda: str) -> UrdfSource | UsdSource | None:
        """`usda` 名に対応するリーフ入力(`UrdfSource` または `UsdSource`)を返す。結合ファイルなら `None`。"""
        return self.urdf(usda) or self.usd(usda)

    def join_file(self, usda: str) -> JoinFileSpec | None:
        """`usda` 名に一致する `JoinFileSpec` を返す(無ければ `None`)。"""
        return next((f for f in self.join_files if f.usda == usda), None)

    @property
    def source_by_usda(self) -> _CallableDict[UrdfSource | UsdSource]:
        """全リーフ入力(URDF / USD)の `{usda名: Source}` マップ。"""
        out: dict[str, UrdfSource | UsdSource] = {u.usda: u for u in self.urdfs}
        out.update({u.usda: u for u in self.usds})
        return _CallableDict(out)

    @property
    def join_file_by_usda(self) -> _CallableDict[JoinFileSpec]:
        """全結合ファイルの `{usda名: JoinFileSpec}` マップ。"""
        return _CallableDict({f.usda: f for f in self.join_files})

    @property
    def all_usda_names(self) -> _CallableList[str]:
        """ビルド順(URDF 部品 → USD 部品 → 依存順の結合ファイル)に並べた全 usda 名リスト。"""
        return _CallableList(
            [u.usda for u in self.urdfs]
            + [u.usda for u in self.usds]
            + [f.usda for f in self.file_order()]
        )

    def file_order(self) -> list[JoinFileSpec]:
        """依存先(`parent_usda` / `child_usda`)の結合ファイルが先に来るトポロジカル順で返す。

        循環がある場合は辿った経路を含めて `ValueError` を送出する。
        """
        done: list[str] = []
        visiting: list[str] = []

        def visit(usda: str) -> None:
            f = self.join_file(usda)
            if f is None or usda in done:
                return
            if usda in visiting:
                cycle = visiting[visiting.index(usda):] + [usda]
                raise ValueError(f"結合ファイルが循環している: {' → '.join(cycle)}(parent_usda か child_usda でたどった順)")
            visiting.append(usda)
            for dep in [f.parent_usda] + [j.child_usda for j in f.joins]:
                visit(dep)
            visiting.pop()
            done.append(usda)

        for f in self.join_files:
            visit(f.usda)
        return [self.join_file(n) for n in done]  # type: ignore[misc]

    def root_source(self, usda: str) -> UrdfSource | UsdSource:
        """`usda` のルートとなるリーフ入力(`UrdfSource` または `UsdSource`)を返す。

        結合ファイルが渡された場合は `parent_usda` をリーフに到達するまで再帰的にたどる。
        """
        cur = usda
        while self.leaf_source(cur) is None:
            f = self.join_file(cur)
            if f is None:
                raise KeyError(f"usda {cur!r} が仕様に見つからない")
            cur = f.parent_usda
        return self.leaf_source(cur)  # type: ignore[return-value]

    def root_urdf(self, usda: str) -> UrdfSource | None:
        """`usda` のルートとなる `UrdfSource` を返す(ルートが `UsdSource` なら `None`)。"""
        src = self.root_source(usda)
        return src if isinstance(src, UrdfSource) else None

    def resolve_urdf_path(self, usda_or_source: str | UrdfSource) -> str:
        """`UrdfSource` または usda 名(あるいは直接のファイルパス)から URDF ファイルパスを解決する。

        例外:
          ValueError: 指定した結合ファイルのルートが `UsdSource` で URDF を持たない場合。
        """
        if isinstance(usda_or_source, UrdfSource):
            return usda_or_source.urdf
        u = self.urdf(usda_or_source)
        if u is not None:
            return u.urdf
        if self.join_file(usda_or_source) is not None:
            ru = self.root_urdf(usda_or_source)
            if ru is not None:
                return ru.urdf
            raise ValueError(f"{usda_or_source!r} の一番上は USD 入力のため URDF パスが無い")
        return os.path.abspath(usda_or_source)

    def resolve_link(self, usda: str, scoped: str) -> tuple[str, str]:
        """スコープ付きリンク名を `(所有リーフ usda 名, リンク名)` へ静的に解決する。

        引数:
          usda: 起点となる usda 名(結合ファイルまたはリーフ usda)。
          scoped: `"flange"` のような単純名、または `"arm1/flange"` のような結合名プレフィックス付きリンク名。

        戻り値:
          `(owner_usda, link_name)` のタプル。

        注意:
          先頭セグメントが結合名に一致しない場合は `parent_usda` 側へフォールバックする
          (USD パーツ内部の相対パス表記と衝突しないようにするための仕様)。
        """
        f = self.join_file(usda)
        if f is None:
            return usda, scoped
        head, _, rest = scoped.partition("/")
        if rest:
            j = next((j for j in f.joins if j.name == head), None)
            if j is not None:
                return self.resolve_link(j.child_usda, rest)
        return self.resolve_link(f.parent_usda, scoped)

    def all_joins(self) -> list[tuple[JoinFileSpec, JoinSpec]]:
        """依存順(`file_order`)に展開した全 `(JoinFileSpec, JoinSpec)` の一覧を返す。"""
        return [(f, j) for f in self.file_order() for j in f.joins]

    def rule(self, rid: RuleId) -> RuleSetting:
        """規則 `rid` の実効設定(個別指定が無ければ全体の既定値)を返す。"""
        return RuleSetting(on_violation=self.on_violation_of(rid), fix=self.fix_of(rid))

    def on_violation_of(self, rid: RuleId) -> OnViolation:
        """規則 `rid` の実効 `on_violation` (`"stop"` または `"continue"`)を返す。"""
        s = self.rules.get(rid)
        return s.on_violation if s and s.on_violation else self.on_violation

    def fix_of(self, rid: RuleId) -> FixMode:
        """規則 `rid` の実効 `fix` (`"apply"` または `"skip"`)を返す。"""
        s = self.rules.get(rid)
        return s.fix if s and s.fix else self.fix

    def fix_applied(self, rid: RuleId) -> bool:
        """規則 `rid` の修正を適用すべきなら `True` を返す。"""
        return self.fix_of(rid) == "apply"


def join_id(f: JoinFileSpec, j: JoinSpec) -> str:
    """レポート・ルール判定で 1 つの結合を一意に指す識別子(`"<結合ファイル名>/<結合名>"`)を返す。"""
    return f"{f.usda}/{j.name}"


# ==================================================================== TOML パーサ
# TOML トップレベルで許可するキー集合。未知のキー(タイプミス等)を即時エラーにするために使用する。
ALLOWED_TOP = {
    "output_dir",
    "urdf_to_usda",
    "usd_to_usda",
    "join_file",
    "checks",
    "probe",
    "compare_official",
    "check_kinematics",
    "check_geometric",
}
_TOP_KEYS = ALLOWED_TOP


def _keys(d: dict, allowed: set[str], required: set[str], where: str) -> None:
    """TOML テーブル `d` に未知キーや必須キー欠落がないか検証する。"""
    unknown = sorted(set(d) - allowed)
    if unknown:
        raise ValueError(f"{where}: 知らないキー {unknown}。書けるキー: {sorted(allowed)}")
    missing = sorted(required - set(d))
    if missing:
        raise ValueError(f"{where}: 必須のキー {missing} が無い")


def load_spec(path: str | Path) -> PipelineSpec:
    """TOML ファイルを読み込み、検証済みの `PipelineSpec` を構築して返す。

    引数:
      path: TOML 仕様ファイルのパス。

    戻り値:
      すべてのローカルパスを絶対パスへ正規化した `PipelineSpec`。

    注意:
      - 相対パス(`output_dir`, `urdf`, `usd`, `ros_package_paths`)はカレントディレクトリではなく
        **TOML ファイルのあるディレクトリ基準**で解決される(どこから実行しても同じ結果になるため)。
      - `usd` に `"://"` を含む場合(S3 / Omniverse Nucleus URI 等)はローカル存在確認を行わずそのまま保持する。
      - `check_geometric` は `check_kinematics` の互換キーとして受け付ける(`check_kinematics` が優先)。

    例外:
      ValueError: 未知キー・必須キー欠落・存在しない入力ファイル・不整合な仕様がある場合。
    """
    path_str = os.fspath(path)
    with open(path_str, "rb") as fh:
        data = tomllib.load(fh)
    root = os.path.dirname(os.path.abspath(path_str))

    def absolute(p: str) -> str:
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(root, p))

    _keys(data, ALLOWED_TOP, {"output_dir"}, f"{os.path.basename(path_str)} の一番上")
    raw_urdfs = data.get("urdf_to_usda", [])
    raw_usds = data.get("usd_to_usda", [])
    if not isinstance(raw_urdfs, list):
        raise ValueError("urdf_to_usda は [[urdf_to_usda]](表の配列)で書く")
    if not isinstance(raw_usds, list):
        raise ValueError("usd_to_usda は [[usd_to_usda]](表の配列)で書く")
    if not raw_urdfs and not raw_usds:
        raise ValueError(f"{os.path.basename(path_str)}: [[urdf_to_usda]] または [[usd_to_usda]] を 1 つ以上書く")

    urdfs = []
    u_keys = {f.name for f in fields(UrdfSource)}
    for i, d in enumerate(raw_urdfs, 1):
        where = f"[[urdf_to_usda]] {i} 番目(usda = {d.get('usda')!r})"
        _keys(d, u_keys, {"urdf", "usda"}, where)
        d = dict(d)
        d["urdf"] = absolute(d["urdf"])
        if not os.path.exists(d["urdf"]):
            raise ValueError(f"{where}: urdf = {d['urdf']!r} のファイルが無い")
        try:
            d["ros_package_paths"] = tuple((r["name"], absolute(r["path"])) for r in d.get("ros_package_paths", []))
            urdfs.append(UrdfSource(**d))
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"{where}: {exc}") from None

    usds = []
    usd_keys = {f.name for f in fields(UsdSource)}
    for i, d in enumerate(raw_usds, 1):
        where = f"[[usd_to_usda]] {i} 番目(usda = {d.get('usda')!r})"
        _keys(d, usd_keys, {"usd", "usda"}, where)
        d = dict(d)
        if not isinstance(d["usd"], str):
            raise ValueError(f"{where}: usd は文字列で書く: {d['usd']!r}")
        if "://" not in d["usd"]:
            d["usd"] = absolute(d["usd"])
            if not os.path.exists(d["usd"]):
                raise ValueError(f"{where}: usd = {d['usd']!r} のファイルが無い")
        if "variant_selections" in d:
            if not isinstance(d["variant_selections"], dict):
                raise ValueError(f"{where}: variant_selections はテーブル(辞書)で書く")
            d["variant_selections"] = tuple((str(k), str(v)) for k, v in d["variant_selections"].items())
        try:
            usds.append(UsdSource(**d))
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"{where}: {exc}") from None

    files = []
    f_keys = {"usda", "parent_usda", "join", "variant_defaults", "parent_prim_name"}
    j_keys = {f.name for f in fields(JoinSpec)}
    for i, d in enumerate(data.get("join_file", []), 1):
        where = f"[[join_file]] {i} 番目(usda = {d.get('usda')!r})"
        _keys(d, f_keys, {"usda", "parent_usda"}, where)
        joins = []
        for k, jd in enumerate(d.get("join", []), 1):
            jwhere = f"{where} の [[join_file.join]] {k} 番目(name = {jd.get('name')!r})"
            _keys(jd, j_keys, {"name", "child_usda", "parent_link", "child_link"}, jwhere)
            jd = dict(jd)
            for key in ("offset_xyz", "offset_rpy"):
                if key in jd:
                    jd[key] = tuple(float(v) for v in jd[key])
            if "child_variant_selection" in jd:
                jd["child_variant_selection"] = tuple(jd["child_variant_selection"].items())
            try:
                joins.append(JoinSpec(**jd))
            except (ValueError, TypeError) as exc:
                raise ValueError(f"{jwhere}: {exc}") from None
        vdefs: tuple[tuple[str, str], ...] = ()
        if "variant_defaults" in d:
            if not isinstance(d["variant_defaults"], dict):
                raise ValueError(f"{where}: variant_defaults はテーブル(辞書)で書く")
            vdefs = tuple((str(k), str(v)) for k, v in d["variant_defaults"].items())
        try:
            files.append(
                JoinFileSpec(
                    usda=d["usda"],
                    parent_usda=d["parent_usda"],
                    joins=tuple(joins),
                    variant_defaults=vdefs,
                    parent_prim_name=d.get("parent_prim_name"),
                )
            )
        except ValueError as exc:
            raise ValueError(f"{where}: {exc}") from None
    checks = data.get("checks", {})
    _keys(checks, {"on_violation", "fix", "rules"}, set(), "[checks]")
    rules = {}
    for rid, s in checks.get("rules", {}).items():
        _keys(s, {"on_violation", "fix"}, set(), f"[checks.rules.{rid}]")
        rules[rid] = RuleSetting(**s)
    probe = data.get("probe", {})
    _keys(probe, {f.name for f in fields(ProbeSettings)}, set(), "[probe]")
    if "check_kinematics" in data:
        check_kinematics = bool(data["check_kinematics"])
    elif "check_geometric" in data:
        check_kinematics = bool(data["check_geometric"])
    else:
        check_kinematics = True
    return PipelineSpec(
        output_dir=absolute(data["output_dir"]),
        urdfs=tuple(urdfs),
        join_files=tuple(files),
        on_violation=checks.get("on_violation", "continue"),
        fix=checks.get("fix", "apply"),
        rules=rules,
        probe=ProbeSettings(**probe),
        compare_official=bool(data.get("compare_official", True)),
        check_kinematics=check_kinematics,
        usds=tuple(usds),
    )


__all__ = [
    "UrdfSource",
    "UsdSource",
    "JoinSpec",
    "JoinFileSpec",
    "RuleSetting",
    "ProbeSettings",
    "PipelineSpec",
    "DEFAULT_RULES",
    "ALLOWED_TOP",
    "load_spec",
    "join_id",
]
