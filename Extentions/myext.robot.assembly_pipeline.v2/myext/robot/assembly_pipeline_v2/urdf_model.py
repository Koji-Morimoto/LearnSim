"""URDF XML の軽量パーサと純 NumPy 順運動学(FK)・ゴーストリンク判定。

責務:
  - Isaac Sim / pxr を使わず、標準ライブラリ(`xml.etree.ElementTree`)と NumPy だけで URDF を解析する。
  - リンク・関節・mimic 関係・メッシュ参照を抽出し、事前チェック規則(`rules.py`)・仕様検査(`spec_check.py`)・
    結合時の初期位置合わせ(`compose.py`)・順運動学検証(`sources.py`, `probes.py`)の基準モデルを提供する。
  - ゴーストリンク判定(`UrdfLink.ghost` / `_zero_inertial`)は Isaac Sim の URDF Importer と同一条件に合わせている。

主な入出力・公開 API:
  - 入力: URDF ファイルパス → `load_urdf(path) -> UrdfModel`
  - データクラス: `UrdfGeometry`, `UrdfLink`, `UrdfMimic`, `UrdfJoint`, `UrdfModel`
  - 変換行列ユーティリティ: `rpy_matrix`(`rpy_to_matrix`), `pose_matrix`, `axis_angle_matrix`

呼ばれ元:
  - `__init__.py`（公開 API として再エクスポート）
  - `spec_check.py`, `sources.py`, `part_import.py`, `compose.py`, `official.py`, `probes.py`, `pipeline.py`

座標・行列規約:
  - すべての 4x4 同次変換行列は NumPy の列ベクトル形式(`p_parent = T @ p_child`)で扱う。
  - `rpy` は URDF 仕様どおり固定軸 X→Y→Z の順で適用する(`Rz(yaw) @ Ry(pitch) @ Rx(roll)`)。
"""

from __future__ import annotations

import hashlib
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _floats(text: str | None, n: int, default: float = 0.0) -> tuple[float, ...]:
    """空白区切りの数値文字列を長さ `n` の float タプルへ変換する(欠損分は `default` で補完)。"""
    if not text:
        return (default,) * n
    vals = [float(v) for v in text.split()]
    return tuple(vals + [default] * (n - len(vals)))[:n]


def rpy_matrix(r: float | tuple[float, float, float], p: float | None = None, y: float | None = None) -> np.ndarray:
    """URDF の `rpy`(固定軸 X→Y→Z)から 3x3 回転行列 `Rz(y) @ Ry(p) @ Rx(r)` を返す。

    引数:
      r: roll(rad)、または `(r, p, y)` の 3 要素タプル/配列(`p`, `y` 省略時)。
      p: pitch(rad)。
      y: yaw(rad)。

    戻り値:
      shape `(3, 3)` の回転行列。
    """
    if p is None and y is None and isinstance(r, (tuple, list, np.ndarray)):
        rr, pp, yy = float(r[0]), float(r[1]), float(r[2])
    else:
        rr, pp, yy = float(r), float(p or 0.0), float(y or 0.0)  # type: ignore[arg-type]
    cr, sr, cp, sp, cy, sy = math.cos(rr), math.sin(rr), math.cos(pp), math.sin(pp), math.cos(yy), math.sin(yy)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


rpy_to_matrix = rpy_matrix


def pose_matrix(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    """並進 `xyz`(m)と姿勢 `rpy`(rad)から 4x4 同次変換行列(列ベクトル形式)を作る。"""
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def axis_angle_matrix(axis: tuple[float, float, float] | np.ndarray, angle: float) -> np.ndarray:
    """回転軸 `axis` と回転角 `angle`(rad)から Rodrigues の公式で 3x3 回転行列を返す。

    `axis` がゼロベクトルまたは `angle == 0` の場合は単位行列を返す。
    """
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm == 0.0 or angle == 0.0:
        return np.eye(3)
    a = a / norm
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _origin(el: ET.Element | None) -> np.ndarray:
    """URDF の `<origin xyz="..." rpy="..."/>` 要素を 4x4 同次変換行列に変換する。"""
    if el is None:
        return np.eye(4)
    return pose_matrix(_floats(el.get("xyz"), 3), _floats(el.get("rpy"), 3))  # type: ignore[arg-type]


@dataclass(frozen=True)
class UrdfGeometry:
    """`<visual>` または `<collision>` 内の形状 1 件。"""

    kind: str  # "mesh" / "box" / "cylinder" / "sphere"
    mesh: str | None  # mesh の filename 属性(URDF に書かれたままの相対/package:// パス)
    origin: np.ndarray = field(repr=False, compare=False)


@dataclass(frozen=True)
class UrdfLink:
    """URDF の `<link>` 1 件の要約情報。"""

    name: str
    visuals: tuple[UrdfGeometry, ...]
    collisions: tuple[UrdfGeometry, ...]
    zero_inertial: bool  # inertial が無い、または質量 0(かつ慣性対角が 0 か未指定)

    @property
    def ghost(self) -> bool:
        """URDF Importer が剛体を作らない空リンク(ゴーストリンク)なら `True`。

        なぜ重要か:
          ゴーストリンクを結合点やルートにすると、USD 上で `RigidBodyAPI` が存在せず
          `FixedJoint` の接続先やルート固定が破綻するため、事前チェック(R01 等)や結合処理で特別扱いする。
        """
        return self.zero_inertial and not self.visuals and not self.collisions


@dataclass(frozen=True)
class UrdfMimic:
    """URDF の `<mimic joint="..." multiplier="..." offset="..."/>` 定義。"""

    joint: str
    multiplier: float
    offset: float


@dataclass(frozen=True)
class UrdfJoint:
    """URDF の `<joint>` 1 件の定義。"""

    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray = field(repr=False, compare=False)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 0.0
    has_limit: bool = False
    damping: float | None = None
    mimic: UrdfMimic | None = None

    @property
    def movable(self) -> bool:
        """1 自由度の可動関節(`revolute` / `continuous` / `prismatic`)なら `True`。"""
        return self.type in ("revolute", "continuous", "prismatic")


@dataclass
class UrdfModel:
    """単一 URDF ファイルの解析結果。木構造探索・順運動学(FK)・メッシュ一覧を提供する。"""

    path: str
    name: str
    links: dict[str, UrdfLink]
    joints: dict[str, UrdfJoint]
    root_link: str
    text_sha256: str  # part_import.py のハッシュキャッシュ判定に使う URDF 原文の SHA-256

    # ------------------------------------------------------------ 木構造の探索
    def parent_joint(self, link: str) -> UrdfJoint | None:
        """`link` を子(`child`)に持つ親関節を返す。ルートリンクなら `None`。"""
        return next((j for j in self.joints.values() if j.child == link), None)

    def child_joints(self, link: str) -> list[UrdfJoint]:
        """`link` を親(`parent`)に持つ子関節一覧を返す。"""
        return [j for j in self.joints.values() if j.parent == link]

    def is_leaf(self, link: str) -> bool:
        """`link` に子関節が無ければ `True`。"""
        return not self.child_joints(link)

    def _fk_single_zero(self, link: str) -> np.ndarray:
        """関節値 0 固定で、ルートリンクから `link` までの `<origin>` を累積した 4x4 行列を返す。"""
        chain = []
        cur = link
        while cur != self.root_link:
            j = self.parent_joint(cur)
            if j is None:
                raise KeyError(f"{self.name}: リンク {link!r} がルートにつながっていない")
            chain.append(j.origin)
            cur = j.parent
        m = np.eye(4)
        for o in reversed(chain):
            m = m @ o
        return m

    def fk_zero(self, link: str | None = None) -> np.ndarray | dict[str, np.ndarray]:
        """関節値 0 のときの、ルートリンク座標系で見た `link` の姿勢(4x4、列ベクトル形式)。

        引数:
          link: 対象リンク名。省略(`None`)時は全リンクの `{link_name: 4x4}` 辞書を返す。
        """
        if link is None:
            return {ln: self._fk_single_zero(ln) for ln in self.links}
        return self._fk_single_zero(link)

    def _fk_single(self, link: str, q: dict[str, float]) -> np.ndarray:
        """関節値 `q` でのルートリンク→`link` の 4x4 変換行列を計算する。

        各関節で `T_parent_to_child = j.origin @ motion(v)` をルート側から順に掛ける。
        follower 関節の値が `q` に直接与えられておらず leader が `q` にある場合は、
        URDF の mimic 式 `v = multiplier * q[leader] + offset` を自動適用する。
        """
        chain = []
        cur = link
        while cur != self.root_link:
            j = self.parent_joint(cur)
            if j is None:
                raise KeyError(f"{self.name}: リンク {link!r} がルートにつながっていない")
            chain.append(j)
            cur = j.parent
        m = np.eye(4)
        for j in reversed(chain):
            v = q.get(j.name, 0.0)
            if j.mimic and j.mimic.joint in q and j.name not in q:
                v = j.mimic.multiplier * q[j.mimic.joint] + j.mimic.offset
            motion = np.eye(4)
            if j.type in ("revolute", "continuous"):
                motion[:3, :3] = axis_angle_matrix(j.axis, v)
            elif j.type == "prismatic":
                axis = np.asarray(j.axis, dtype=float)
                axis = axis / (np.linalg.norm(axis) or 1.0)
                motion[:3, 3] = axis * v
            m = m @ j.origin @ motion
        return m

    def fk(
        self,
        link_or_q: str | dict[str, float] | None = None,
        q: dict[str, float] | None = None,
    ) -> np.ndarray | dict[str, np.ndarray]:
        """関節値 `q`(関節名 → rad または m、未指定関節は 0)での順運動学を計算する。

        呼び出し形式:
          - `fk(link, q)` → 単一リンクの 4x4 行列
          - `fk(q)` または `fk()` → 全リンクの `{link_name: 4x4}` 辞書
        """
        if isinstance(link_or_q, str):
            return self._fk_single(link_or_q, q or {})
        q_dict = link_or_q if isinstance(link_or_q, dict) else (q or {})
        return {ln: self._fk_single(ln, q_dict) for ln in self.links}

    def fk_all(self, q: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        """全リンクの順運動学 `{link_name: 4x4}` を返す。"""
        q_dict = q or {}
        return {ln: self._fk_single(ln, q_dict) for ln in self.links}

    def sample_random_q(self, rng: np.random.Generator | None = None, scale: float = 0.6) -> dict[str, float]:
        """可動関節の安全なランダム関節値を生成する(mimic follower は leader から従属計算)。

        引数:
          rng: NumPy 乱数生成器(省略時は既定 RNG)。
          scale: 可動域中央から何割の幅でサンプルするか(0.0〜1.0。既定 0.6 はリミット端での衝突を避けるため)。

        戻り値:
          `{joint_name: value}` 辞書(回転は rad、直動は m)。
        """
        rng = rng or np.random.default_rng()
        followers = {f.name for f, _ in self.mimic_pairs()}
        q: dict[str, float] = {}
        half = 0.5 * max(0.0, min(1.0, float(scale)))
        lo_r, hi_r = 0.5 - half, 0.5 + half
        for j in self.joints.values():
            if not j.movable or j.name in followers:
                continue
            if j.has_limit and j.upper > j.lower:
                lo, hi = j.lower, j.upper
            elif j.type == "prismatic":
                lo, hi = -0.05, 0.05
            else:
                lo, hi = -0.5, 0.5
            u = float(rng.uniform(lo_r, hi_r))
            q[j.name] = lo + u * (hi - lo)
        for follower, leader in self.mimic_pairs():
            if leader.name in q and follower.mimic is not None:
                q[follower.name] = follower.mimic.multiplier * q[leader.name] + follower.mimic.offset
        return q

    def fixed_descendants(self, link: str) -> list[str]:
        """`link` から `fixed` 関節のみを BFS でたどって到達できる子孫リンク名を近い順に返す。

        用途:
          ゴーストリンクを結合先に選んだ際、物理的に一体化した最寄りの実剛体リンクを探索するために使う。
        """
        out, queue = [], [link]
        while queue:
            cur = queue.pop(0)
            for j in self.child_joints(cur):
                if j.type == "fixed":
                    out.append(j.child)
                    queue.append(j.child)
        return out

    def mimic_pairs(self) -> list[tuple[UrdfJoint, UrdfJoint]]:
        """URDF 内に存在する `(follower_joint, leader_joint)` の組一覧を返す。"""
        out = []
        for j in self.joints.values():
            if j.mimic and j.mimic.joint in self.joints:
                out.append((j, self.joints[j.mimic.joint]))
        return out

    def has_collisions(self) -> bool:
        """いずれかのリンクに `<collision>` 形状が定義されていれば `True`。"""
        return any(lnk.collisions for lnk in self.links.values())

    # ------------------------------------------------------------ メッシュ参照解決
    def mesh_files(self, ros_package_paths: tuple[tuple[str, str], ...] = ()) -> list[str]:
        """URDF が参照する全メッシュの正規化ファイルパス一覧を返す(重複排除・ソート済み)。

        用途:
          `part_import.py` がメッシュ更新を検知して再インポート要否を判定するマニフェスト入力に使う。
        """
        base = os.path.dirname(os.path.abspath(self.path))
        packages = dict(ros_package_paths)
        out = []
        for lnk in self.links.values():
            for g in lnk.visuals + lnk.collisions:
                if not g.mesh:
                    continue
                f = g.mesh
                if f.startswith("package://"):
                    pkg, _, rel = f[len("package://"):].partition("/")
                    f = os.path.join(packages.get(pkg, os.path.join(base, pkg)), rel)
                elif f.startswith("file://"):
                    f = f[len("file://"):]
                elif not os.path.isabs(f):
                    f = os.path.join(base, f)
                out.append(os.path.normpath(f))
        return sorted(set(out))


def _geometry(el: ET.Element) -> UrdfGeometry | None:
    """`<visual>` / `<collision>` 要素から最初の `<geometry>` 形状を抽出する。"""
    g = el.find("geometry")
    if g is None or len(g) == 0:
        return None
    shape = g[0]
    return UrdfGeometry(shape.tag, shape.get("filename"), _origin(el.find("origin")))


def _zero_inertial(el: ET.Element | None) -> bool:
    """`<inertial>` が実質ゼロ慣性かを Isaac Sim URDF Importer の `_is_zero_inertial` と同一条件で判定する。

    判定基準:
      - `<inertial>` 要素または `<mass value="..."/>` が欠落している → `True`
      - 質量が正(`mass > 0`) → `False`
      - 質量が 0 以下のとき、`<inertia>` が無い、または対角成分(`ixx`, `iyy`, `izz`)がすべて未指定か 0 → `True`
    """
    if el is None:
        return True
    m = el.find("mass")
    mass = float(m.get("value")) if m is not None and m.get("value") is not None else None
    if mass is None:
        return True
    if mass > 0.0:
        return False
    inertia = el.find("inertia")
    if inertia is None:
        return True
    diag = [inertia.get(k) for k in ("ixx", "iyy", "izz")]
    return all(v is None or float(v) == 0.0 for v in diag)


def load_urdf(path: str | Path) -> UrdfModel:
    """URDF ファイルを読み込んで `UrdfModel` を構築する。

    引数:
      path: URDF ファイルパス。

    戻り値:
      リンク・関節・単一ルートリンク・原文 SHA-256 を持つ `UrdfModel`。

    例外:
      ValueError: ルートリンク(どの関節の `child` にもなっていないリンク)が 1 つに定まらない場合
                  (閉ループ構造または孤立リンクが存在する場合)。
    """
    path_str = os.fspath(path)
    with open(path_str, "rb") as f:
        raw = f.read()
    root = ET.fromstring(raw)
    links: dict[str, UrdfLink] = {}
    for el in root.findall("link"):
        vis = tuple(g for g in (_geometry(v) for v in el.findall("visual")) if g is not None)
        col = tuple(g for g in (_geometry(c) for c in el.findall("collision")) if g is not None)
        links[el.get("name")] = UrdfLink(el.get("name"), vis, col, _zero_inertial(el.find("inertial")))  # type: ignore[index,arg-type]
    joints: dict[str, UrdfJoint] = {}
    for el in root.findall("joint"):
        limit = el.find("limit")
        dyn = el.find("dynamics")
        mim = el.find("mimic")
        joints[el.get("name")] = UrdfJoint(  # type: ignore[index]
            name=el.get("name"),  # type: ignore[arg-type]
            type=el.get("type"),  # type: ignore[arg-type]
            parent=el.find("parent").get("link"),  # type: ignore[union-attr,arg-type]
            child=el.find("child").get("link"),  # type: ignore[union-attr,arg-type]
            origin=_origin(el.find("origin")),
            axis=_floats(el.find("axis").get("xyz") if el.find("axis") is not None else "1 0 0", 3),  # type: ignore[union-attr,arg-type]
            lower=float(limit.get("lower", 0.0)) if limit is not None else 0.0,
            upper=float(limit.get("upper", 0.0)) if limit is not None else 0.0,
            has_limit=limit is not None and (limit.get("lower") is not None or limit.get("upper") is not None),
            damping=float(dyn.get("damping")) if dyn is not None and dyn.get("damping") is not None else None,  # type: ignore[arg-type]
            mimic=UrdfMimic(mim.get("joint"), float(mim.get("multiplier", 1.0)), float(mim.get("offset", 0.0))) if mim is not None else None,  # type: ignore[arg-type]
        )
    # どの関節の子にもなっていないリンクをルートとして特定する(URDF は単一木を前提とする)。
    children = {j.child for j in joints.values()}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"{path_str}: ルートリンクが 1 つに決まらない(閉ループか、つながっていないリンクがある): {roots}")
    return UrdfModel(
        path=os.path.abspath(path_str),
        name=root.get("name"),  # type: ignore[arg-type]
        links=links,
        joints=joints,
        root_link=roots[0],
        text_sha256=hashlib.sha256(raw).hexdigest(),
    )


__all__ = [
    "UrdfGeometry",
    "UrdfLink",
    "UrdfMimic",
    "UrdfJoint",
    "UrdfModel",
    "load_urdf",
    "rpy_matrix",
    "rpy_to_matrix",
    "pose_matrix",
    "axis_angle_matrix",
]
