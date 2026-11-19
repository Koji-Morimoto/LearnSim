"""パイプラインの入力(仕様)と、TOML の読み込み。Isaac Sim なしで import できる(pxr を使わない)。

言葉(この拡張の中では、この意味でだけ使う):
  usda        : default prim を持つ USD ファイル。URDF から作ったもの、または結合ファイル
  結合        : 子の usda を、親の usda のリンクに固定する 1 回の操作(compose.py の 7 つの書き込み)
  結合ファイル: 親の usda に 1 回以上の結合を書いた、新しい usda。次の結合の親にも子にもできる
  公式のまま  : URDF Importer の出力を Robot Assembler でそのまま結合した場合(比較用に実際に結合して測る)
  修正後      : このパイプラインの出力
  修正        : このパイプラインが入れる直し

usda は、どこでも output_dir に作るファイル名(拡張子 .usda まで)で指す。
  作る側 : [[urdf_to_usda]] の usda、[[join_file]] の usda
  使う側 : [[join_file]] の parent_usda、[[join_file.join]] の child_usda
TOML の書き方の全部は docs/README.md の「TOML の書き方」。
"""

from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, field, fields

from .kinds import FixMode, OnViolation, PhysicsVariant, RuleId, check_literal


def _check_usda_name(value: object, where: str) -> str:
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
    """

    urdf: str
    usda: str
    collision_from_visuals: bool | None = None
    collision_type: str = "Convex Hull"
    physics_variant: PhysicsVariant = "auto"
    ros_package_paths: tuple[tuple[str, str], ...] = ()
    joint_drive_stiffness: float | None = None
    joint_drive_damping: float | None = None

    def __post_init__(self) -> None:
        _check_usda_name(self.usda, "[[urdf_to_usda]] の usda")
        check_literal(self.physics_variant, PhysicsVariant, f"[[urdf_to_usda]] usda = {self.usda!r} の physics_variant")

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
    """[[join_file]] 1 つ = 結合ファイルを 1 つ作る。

    usda        : 出力。output_dir に作る結合ファイルのファイル名(例 "arm_unit.usda")。ほかの場所ではこの名前で指す
    parent_usda : 入力。親の usda のファイル名([[urdf_to_usda]] か、ほかの [[join_file]] の usda)
    joins       : [[join_file.join]] の並び
    """

    usda: str
    parent_usda: str
    joins: tuple[JoinSpec, ...] = ()

    def __post_init__(self) -> None:
        _check_usda_name(self.usda, "[[join_file]] の usda")
        _check_usda_name(self.parent_usda, f"[[join_file]] usda = {self.usda!r} の parent_usda")
        names = [j.name for j in self.joins]
        dup = sorted({n for n in names if names.count(n) > 1})
        if dup:
            raise ValueError(f"[[join_file]] usda = {self.usda!r}: [[join_file.join]] の name が重複している: {dup}")

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
        return next(j for j in self.joins if j.name == name)


@dataclass(frozen=True)
class RuleSetting:
    on_violation: OnViolation | None = None
    fix: FixMode | None = None


@dataclass(frozen=True)
class ProbeSettings:
    """[probe] 症状を測るときの条件。

    steps            : Play するステップ数
    drive_stiffness  : 関節を動かす drive の強さ(回転は 1 度あたり、直動は 1 m あたり)
    drive_damping    : 同じく damping
    revolute_target  : 回転関節の目標(rad。可動範囲の 8 割に切り詰める)
    prismatic_ratio  : 直動関節の目標(可動範囲の何割か)
    mimic_leader_*   : mimic の leader を動かす柔らかい drive(follower の drive の影響が見えるように柔らかくする)
    angle_tol_deg    : 角度の許容(度)
    length_tol_m     : 位置の許容(m)
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
    output_dir: str
    urdfs: tuple[UrdfSource, ...]
    join_files: tuple[JoinFileSpec, ...] = ()
    on_violation: OnViolation = "continue"
    fix: FixMode = "apply"
    rules: dict[str, RuleSetting] = field(default_factory=dict)
    probe: ProbeSettings = ProbeSettings()
    compare_official: bool = True  # 公式のまま(Robot Assembler)でも結合して症状を測る
    check_geometric: bool = True  # 結合ファイルごとに GeometricRobot を作って PhysX と順運動学を突き合わせる

    def __post_init__(self) -> None:
        check_literal(self.on_violation, OnViolation, "[checks] の on_violation")
        check_literal(self.fix, FixMode, "[checks] の fix")
        for rid, s in self.rules.items():
            check_literal(rid, RuleId, "[checks.rules.<規則>] の <規則>")
            if s.on_violation is not None:
                check_literal(s.on_violation, OnViolation, f"[checks.rules.{rid}] の on_violation")
            if s.fix is not None:
                check_literal(s.fix, FixMode, f"[checks.rules.{rid}] の fix")
        made: dict[str, str] = {}
        for i, u in enumerate(self.urdfs, 1):
            self._made(made, u.usda, f"[[urdf_to_usda]] {i} 番目")
        for i, f in enumerate(self.join_files, 1):
            self._made(made, f.usda, f"[[join_file]] {i} 番目")
        stems = {}
        for name in made:
            stem = name[: -len(".usda")].lower()
            if stem in stems:
                raise ValueError(f"usda のファイル名 {stems[stem]!r} と {name!r} は大文字と小文字しか違わない(Windows では同じファイルになる)")
            stems[stem] = name
        known = sorted(made)
        for f in self.join_files:
            if f.parent_usda not in made:
                raise ValueError(f"[[join_file]] usda = {f.usda!r} の parent_usda = {f.parent_usda!r} は、この TOML のどこでも作っていない。"
                                 f"作っている usda: {known}")
            for j in f.joins:
                if j.child_usda not in made:
                    raise ValueError(f"[[join_file]] usda = {f.usda!r} の [[join_file.join]] name = {j.name!r} の child_usda = {j.child_usda!r} は、"
                                     f"この TOML のどこでも作っていない。作っている usda: {known}")
        self.file_order()  # 循環の確認

    @staticmethod
    def _made(made: dict[str, str], usda: str, where: str) -> None:
        if usda in made:
            raise ValueError(f"usda = {usda!r} を 2 か所で作ろうとしている: {made[usda]} と {where}。出力のファイル名は 1 つずつ別にする")
        made[usda] = where

    # ---------------------------------------------------------------- 参照
    def urdf(self, usda: str) -> UrdfSource | None:
        return next((u for u in self.urdfs if u.usda == usda), None)

    def join_file(self, usda: str) -> JoinFileSpec | None:
        return next((f for f in self.join_files if f.usda == usda), None)

    def file_order(self) -> list[JoinFileSpec]:
        """依存する結合ファイルが先に来る順。"""
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

    def root_urdf(self, usda: str) -> UrdfSource:
        """usda の一番上の URDF(結合ファイルなら parent_usda をたどった先)。"""
        while self.urdf(usda) is None:
            usda = self.join_file(usda).parent_usda  # type: ignore[union-attr]
        return self.urdf(usda)  # type: ignore[return-value]

    def resolve_link(self, usda: str, scoped: str) -> tuple[str, str]:
        """リンクの書き方を (そのリンクを持つ URDF から作った usda のファイル名, リンク名) にする。

        "flange" = usda の一番上の URDF のリンク。"arm1/flange" = usda(結合ファイル)の中の結合 arm1 の子の usda のリンク。
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
        return [(f, j) for f in self.file_order() for j in f.joins]

    def on_violation_of(self, rid: RuleId) -> OnViolation:
        s = self.rules.get(rid)
        return s.on_violation if s and s.on_violation else self.on_violation

    def fix_of(self, rid: RuleId) -> FixMode:
        s = self.rules.get(rid)
        return s.fix if s and s.fix else self.fix

    def fix_applied(self, rid: RuleId) -> bool:
        return self.fix_of(rid) == "apply"


def join_id(f: JoinFileSpec, j: JoinSpec) -> str:
    """報告で結合を指す名前:「結合ファイルのファイル名/結合の名前」。"""
    return f"{f.usda}/{j.name}"


# ==================================================================== TOML
_TOP_KEYS = {"output_dir", "urdf_to_usda", "join_file", "checks", "probe", "compare_official", "check_geometric"}


def _keys(d: dict, allowed: set[str], required: set[str], where: str) -> None:
    unknown = sorted(set(d) - allowed)
    if unknown:
        raise ValueError(f"{where}: 知らないキー {unknown}。書けるキー: {sorted(allowed)}")
    missing = sorted(required - set(d))
    if missing:
        raise ValueError(f"{where}: 必須のキー {missing} が無い")


def load_spec(path: str) -> PipelineSpec:
    """TOML から読む。パスは TOML ファイルのフォルダから見た相対パスでもよい。"""
    with open(path, "rb") as fh:
        data = tomllib.load(fh)
    root = os.path.dirname(os.path.abspath(path))

    def absolute(p: str) -> str:
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(root, p))

    _keys(data, _TOP_KEYS, {"output_dir", "urdf_to_usda"}, f"{os.path.basename(path)} の一番上")
    if not isinstance(data["urdf_to_usda"], list):
        raise ValueError("urdf_to_usda は [[urdf_to_usda]](表の配列)で書く")
    urdfs = []
    u_keys = {f.name for f in fields(UrdfSource)}
    for i, d in enumerate(data["urdf_to_usda"], 1):
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
    files = []
    f_keys = {"usda", "parent_usda", "join"}
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
        try:
            files.append(JoinFileSpec(usda=d["usda"], parent_usda=d["parent_usda"], joins=tuple(joins)))
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
    return PipelineSpec(
        output_dir=absolute(data["output_dir"]), urdfs=tuple(urdfs), join_files=tuple(files),
        on_violation=checks.get("on_violation", "continue"), fix=checks.get("fix", "apply"), rules=rules,
        probe=ProbeSettings(**probe), compare_official=data.get("compare_official", True),
        check_geometric=data.get("check_geometric", True))


__all__ = ["UrdfSource", "JoinSpec", "JoinFileSpec", "RuleSetting", "ProbeSettings", "PipelineSpec", "load_spec", "join_id"]
