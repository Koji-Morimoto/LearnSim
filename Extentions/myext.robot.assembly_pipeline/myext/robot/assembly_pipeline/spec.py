"""パイプラインの入力(仕様)。

言葉(この拡張の中では、この意味でだけ使う):
  usda        : default prim を持つ USD ファイル。URDF から作ったもの、または結合ファイル
  結合        : 子の usda を、親の usda のリンクに固定する 1 回の操作(compose.py の 7 つの書き込み)
  結合ファイル: 親の usda に 1 回以上の結合を書いた、新しい usda。次の結合の親にも子にもできる
  公式のまま  : URDF Importer の出力を Robot Assembler でそのまま結合した場合(比較用に実際に結合して測る)
  修正後      : このパイプラインの出力
  修正        : このパイプラインが入れる直し

TOML の例(相対パスは TOML ファイルから見る):

    output_dir = "out"

    [urdf.arm]
    path = "urdf/arm.urdf"

    [urdf.gripper]
    path = "urdf/gripper.urdf"

    [join_file.arm_unit]                 # 出力 out/arm_unit.usda
    parent = "arm"

    [[join_file.arm_unit.join]]
    name = "gripper"
    child = "gripper"
    parent_link = "flange"
    child_link = "ee_base"
    prim_name = "end_effector"
    variant_set = "EndEffector"          # 結合ファイルの一番上の prim に付く VariantSet
    variant = "gripper"                  # その選択肢の名前

結合のリンクの書き方: 親(または子)の usda の一番上の URDF のリンクは名前だけ("flange")。
結合ファイルの中で結合した子のリンクは「結合の名前/リンク名」("arm1/flange"、入れ子なら "arm1/gripper/finger_l")。
"""

from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, field

from .kinds import FixMode, OnViolation, PhysicsVariant, RuleId, check_literal


@dataclass(frozen=True)
class UrdfSource:
    """URDF 1 本から作る usda。

    name                  : 名前(識別子)。usda の名前の空間は URDF と結合ファイルで共通
    path                  : URDF のパス
    collision_from_visuals: URDF Importer の同名の設定。None なら「URDF に collision が 1 つでもあれば False、無ければ True」
    collision_type        : collision_from_visuals のときの近似
    physics_variant       : 使う Physics variant
    ros_package_paths     : package:// の解決先。((name, path), ...)
    joint_drive_stiffness : URDF Importer の override_joint_stiffness(全関節。mimic の follower にも入るので、修正で follower は 0 に戻す = R03)
    joint_drive_damping   : URDF Importer の override_joint_damping(同上)
    """

    name: str
    path: str
    collision_from_visuals: bool | None = None
    collision_type: str = "Convex Hull"
    physics_variant: PhysicsVariant = "auto"
    ros_package_paths: tuple[tuple[str, str], ...] = ()
    joint_drive_stiffness: float | None = None
    joint_drive_damping: float | None = None

    def __post_init__(self) -> None:
        check_literal(self.physics_variant, PhysicsVariant, f"urdf.{self.name}.physics_variant")
        if not self.name.isidentifier():
            raise ValueError(f"usda の名前は prim 名に使うので識別子にする: {self.name!r}")


@dataclass(frozen=True)
class JoinSpec:
    """結合 1 回。

    name        : 結合の名前(結合ファイルの中で一意)。報告では「結合ファイル名/結合の名前」で出る
    child       : 子の usda の名前(urdf か join_file)
    parent_link : 親の usda のリンク(上の書き方)。子のリンク child_link の座標系をここに一致させる
    child_link  : 子の usda のリンク
    parent_prim : 子を Reference で置く親側の prim。リンクの書き方、"." (結合ファイルの一番上の prim)、
                  または "/" で始まる結合ファイルの中のパス。省略時は parent_link の prim(stage の親子が実際のリンクの親子に揃う)
    prim_name   : 子の prim の名前(省略時は name)。VariantSet の選択肢どうしで同じにするとパスが変わらない
    offset_xyz  : parent_link の座標系で見た child_link の位置(m)
    offset_rpy  : 同じく向き(rad、URDF と同じ固定軸 XYZ)
    variant_set : 結合を VariantSet の選択肢にするとき、その VariantSet の名前(結合ファイルの一番上の prim に付く)
    variant     : その選択肢の名前。同じ variant_set の結合は排他で、最初のものが既定の選択。"None" の選択肢は自動で足す
    select      : 子の usda の VariantSet の選択 ((VariantSet 名, 選択肢), ...)。例: 4 本のアームでエンドエフェクタを選び分ける
    """

    name: str
    child: str
    parent_link: str
    child_link: str
    parent_prim: str | None = None
    prim_name: str | None = None
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    variant_set: str | None = None
    variant: str | None = None
    select: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"結合の名前は識別子にする: {self.name!r}")
        if (self.variant_set is None) != (self.variant is None):
            raise ValueError(f"結合 {self.name}: variant_set と variant は両方書くか、両方書かない")
        if self.variant == "None":
            raise ValueError(f"結合 {self.name}: 選択肢の名前 None は予約済み(何も結合しない選択肢として自動で足す)")
        if not self.prim.isidentifier():
            raise ValueError(f"結合 {self.name}: prim_name は識別子にする: {self.prim!r}")

    @property
    def prim(self) -> str:
        return self.prim_name or self.name


@dataclass(frozen=True)
class JoinFileSpec:
    """結合ファイル。出力は <output_dir>/<name>.usda。"""

    name: str
    parent: str
    joins: tuple[JoinSpec, ...] = ()

    def __post_init__(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"usda の名前は prim 名に使うので識別子にする: {self.name!r}")
        names = [j.name for j in self.joins]
        if len(set(names)) != len(names):
            raise ValueError(f"結合ファイル {self.name}: 結合の名前が重複している: {names}")

    def join(self, name: str) -> JoinSpec:
        return next(j for j in self.joins if j.name == name)


@dataclass(frozen=True)
class RuleSetting:
    on_violation: OnViolation | None = None
    fix: FixMode | None = None


@dataclass(frozen=True)
class ProbeSettings:
    """症状を測るときの条件。

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
        check_literal(self.on_violation, OnViolation, "checks.on_violation")
        check_literal(self.fix, FixMode, "checks.fix")
        for rid, s in self.rules.items():
            check_literal(rid, RuleId, "checks.rules のキー")
            if s.on_violation is not None:
                check_literal(s.on_violation, OnViolation, f"checks.rules.{rid}.on_violation")
            if s.fix is not None:
                check_literal(s.fix, FixMode, f"checks.rules.{rid}.fix")
        names = [u.name for u in self.urdfs] + [f.name for f in self.join_files]
        if len(set(names)) != len(names):
            raise ValueError(f"usda の名前が重複している(URDF と結合ファイルで共通の名前の空間): {names}")
        for f in self.join_files:
            if f.parent not in names:
                raise ValueError(f"結合ファイル {f.name} の parent {f.parent!r} が無い")
            for j in f.joins:
                if j.child not in names:
                    raise ValueError(f"結合 {f.name}/{j.name} の child {j.child!r} が無い")
        self.file_order()  # 循環の確認

    # ---------------------------------------------------------------- 参照
    def urdf(self, name: str) -> UrdfSource | None:
        return next((u for u in self.urdfs if u.name == name), None)

    def join_file(self, name: str) -> JoinFileSpec | None:
        return next((f for f in self.join_files if f.name == name), None)

    def file_order(self) -> list[JoinFileSpec]:
        """依存する結合ファイルが先に来る順。"""
        done: list[str] = []
        visiting: set[str] = set()

        def visit(name: str) -> None:
            f = self.join_file(name)
            if f is None or name in done:
                return
            if name in visiting:
                raise ValueError(f"結合ファイルが循環している: {name}")
            visiting.add(name)
            for dep in [f.parent] + [j.child for j in f.joins]:
                visit(dep)
            visiting.discard(name)
            done.append(name)

        for f in self.join_files:
            visit(f.name)
        return [self.join_file(n) for n in done]  # type: ignore[misc]

    def root_urdf(self, name: str) -> UrdfSource:
        """usda の一番上の URDF(結合ファイルなら parent をたどった先)。"""
        while self.urdf(name) is None:
            name = self.join_file(name).parent  # type: ignore[union-attr]
        return self.urdf(name)  # type: ignore[return-value]

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
    return f"{f.name}/{j.name}"


def load_spec(path: str) -> PipelineSpec:
    """TOML から読む。相対パスは TOML ファイルのフォルダから見る。"""
    with open(path, "rb") as fh:
        data = tomllib.load(fh)
    root = os.path.dirname(os.path.abspath(path))

    def absolute(p: str) -> str:
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(root, p))

    known = {"output_dir", "urdf", "join_file", "checks", "probe", "compare_official", "check_geometric"}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"知らない設定: {sorted(unknown)}")
    urdfs = []
    for name, d in data.get("urdf", {}).items():
        d = dict(d)
        d["path"] = absolute(d["path"])
        d["ros_package_paths"] = tuple((r["name"], absolute(r["path"])) for r in d.get("ros_package_paths", []))
        urdfs.append(UrdfSource(name=name, **d))
    files = []
    for name, d in data.get("join_file", {}).items():
        joins = []
        for jd in d.get("join", []):
            jd = dict(jd)
            for k in ("offset_xyz", "offset_rpy"):
                if k in jd:
                    jd[k] = tuple(float(v) for v in jd[k])
            if "select" in jd:
                jd["select"] = tuple(jd["select"].items())
            joins.append(JoinSpec(**jd))
        files.append(JoinFileSpec(name=name, parent=d["parent"], joins=tuple(joins)))
    checks = data.get("checks", {})
    rules = {rid: RuleSetting(**s) for rid, s in checks.get("rules", {}).items()}
    return PipelineSpec(
        output_dir=absolute(data["output_dir"]), urdfs=tuple(urdfs), join_files=tuple(files),
        on_violation=checks.get("on_violation", "continue"), fix=checks.get("fix", "apply"), rules=rules,
        probe=ProbeSettings(**data.get("probe", {})), compare_official=data.get("compare_official", True),
        check_geometric=data.get("check_geometric", True))


__all__ = ["UrdfSource", "JoinSpec", "JoinFileSpec", "RuleSetting", "ProbeSettings", "PipelineSpec", "load_spec", "join_id"]
