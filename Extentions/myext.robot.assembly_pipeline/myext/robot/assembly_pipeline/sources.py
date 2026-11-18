"""usda の中身の地図:usda の中のどこに、どの URDF から作った部分があるか。

usda は URDF から作ったもの 1 つか、結合ファイル(親の usda + 結合した子の usda)なので、中身は
「URDF から作った部分」の集まりになる。この 1 つずつを Owner と呼ぶ(リンクと関節の持ち主)。

  prefix   : リンクの書き方の前置き。一番上の URDF は ""、結合した子は "結合の名前"、入れ子は "arm1/gripper"
  rel_top  : usda の一番上の prim から見た、その部分の一番上の prim の相対パス
  model    : その部分の URDF
  link_rel : リンク名 -> その部分の一番上の prim から見た相対パス
  joint_rel: 関節名 -> 同上
  conditions: その部分が現れる VariantSet の選択 ((VariantSet を持つ prim の相対パス, VariantSet 名, 選択肢), ...)
"""

from __future__ import annotations

from dataclasses import dataclass, field

from pxr import Usd

from .urdf_model import UrdfModel


@dataclass(frozen=True)
class Owner:
    prefix: str
    rel_top: str
    model: UrdfModel
    link_rel: dict[str, str] = field(compare=False, hash=False)
    joint_rel: dict[str, str] = field(compare=False, hash=False)
    urdf_name: str = ""
    conditions: tuple[tuple[str, str, str], ...] = ()

    def under(self, prefix: str, rel: str, conditions: tuple[tuple[str, str, str], ...] = ()) -> "Owner":
        """結合ファイルに子として置いたときの Owner(前置きと相対パスを足す)。"""
        moved = tuple((_join(rel, c[0]), c[1], c[2]) for c in self.conditions)
        return Owner(prefix=_join_prefix(prefix, self.prefix), rel_top=_join(rel, self.rel_top), model=self.model,
                     link_rel=self.link_rel, joint_rel=self.joint_rel, urdf_name=self.urdf_name,
                     conditions=conditions + moved)

    # ---- 絶対パス(top = usda の一番上の prim の、そのステージでのパス)
    def top(self, top: str) -> str:
        return _join(top, self.rel_top)

    def link_path(self, top: str, link: str) -> str:
        return _join(self.top(top), self.link_rel[link])

    def joint_path(self, top: str, joint: str) -> str:
        return _join(self.top(top), self.joint_rel[joint])

    def present(self, stage: Usd.Stage, top: str) -> bool:
        """そのステージで、この部分が現れているか(VariantSet の選択と prim の有無)。"""
        for rel, vs_name, choice in self.conditions:
            prim = stage.GetPrimAtPath(_join(top, rel))
            if not prim:
                return False
            vs = prim.GetVariantSets()
            if not vs.HasVariantSet(vs_name) or vs.GetVariantSet(vs_name).GetVariantSelection() != choice:
                return False
        return bool(stage.GetPrimAtPath(self.top(top)))


@dataclass
class UsdaInfo:
    name: str
    path: str  # 修正後の usda(URDF なら正規化層、結合ファイルならそのファイル)
    official_path: str | None  # 公式のままの usda(URDF の生成物)。結合ファイルは None
    owners: list[Owner]

    def resolve(self, scoped: str) -> tuple[Owner, str]:
        return resolve(self.owners, scoped)


def resolve(owners: list[Owner], scoped: str) -> tuple[Owner, str]:
    """リンクの書き方("flange"、"arm1/flange"、"arm1/gripper/finger_l")を (Owner, リンク名) にする。"""
    best: tuple[Owner, str] | None = None
    for o in owners:
        if o.prefix == "":
            rest = scoped
        elif scoped.startswith(o.prefix + "/"):
            rest = scoped[len(o.prefix) + 1:]
        else:
            continue
        if rest in o.model.links and (best is None or len(o.prefix) > len(best[0].prefix)):
            best = (o, rest)
    if best is None:
        known = sorted({(o.prefix + "/" if o.prefix else "") + "<リンク名>" for o in owners})
        raise KeyError(f"リンク {scoped!r} が見つからない。書き方: {known}")
    return best


def static_resolve(spec, usda_name: str, scoped: str) -> tuple[str, str]:
    """ステージを開かずに、仕様だけからリンクの書き方を (URDF の名前, リンク名) にする(事前チェック用)。"""
    f = spec.join_file(usda_name)
    if f is None:
        return usda_name, scoped
    head, _, rest = scoped.partition("/")
    if rest:
        j = next((j for j in f.joins if j.name == head), None)
        if j is not None:
            return static_resolve(spec, j.child, rest)
    return static_resolve(spec, f.parent, scoped)


def _join(a: str, b: str) -> str:
    if not b:
        return a
    if not a:
        return b
    return a.rstrip("/") + "/" + b.lstrip("/")


def _join_prefix(a: str, b: str) -> str:
    return a if not b else (b if not a else f"{a}/{b}")
