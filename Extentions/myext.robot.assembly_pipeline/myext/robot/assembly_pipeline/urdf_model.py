"""URDF を読む(Isaac Sim 不要。標準ライブラリと numpy だけ)。

事前チェックの判定と、結合の位置合わせ(URDF 上のリンク座標系)に使う。
空のリンク(ゴーストリンク)の判定は URDF の変換器と同じ条件にしてある:
visual が無い・collision が無い・inertial が無いか質量 0(質量 0 なら慣性の対角も 0 か未指定)。
"""

from __future__ import annotations

import hashlib
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


def _floats(text: str | None, n: int, default: float = 0.0) -> tuple[float, ...]:
    if not text:
        return (default,) * n
    vals = [float(v) for v in text.split()]
    return tuple(vals + [default] * (n - len(vals)))[:n]


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF の rpy(固定軸 X→Y→Z)の回転行列 = Rz(y) Ry(p) Rx(r)。"""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_matrix(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def _origin(el: ET.Element | None) -> np.ndarray:
    if el is None:
        return np.eye(4)
    return pose_matrix(_floats(el.get("xyz"), 3), _floats(el.get("rpy"), 3))  # type: ignore[arg-type]


@dataclass(frozen=True)
class UrdfGeometry:
    kind: str  # "mesh" / "box" / "cylinder" / "sphere"
    mesh: str | None  # mesh の filename(書かれたまま)
    origin: np.ndarray = field(repr=False, compare=False)


@dataclass(frozen=True)
class UrdfLink:
    name: str
    visuals: tuple[UrdfGeometry, ...]
    collisions: tuple[UrdfGeometry, ...]
    zero_inertial: bool  # inertial が無い、または質量 0(かつ慣性の対角が 0 か未指定)

    @property
    def ghost(self) -> bool:
        return self.zero_inertial and not self.visuals and not self.collisions


@dataclass(frozen=True)
class UrdfMimic:
    joint: str
    multiplier: float
    offset: float


@dataclass(frozen=True)
class UrdfJoint:
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
        return self.type in ("revolute", "continuous", "prismatic")


@dataclass
class UrdfModel:
    path: str
    name: str
    links: dict[str, UrdfLink]
    joints: dict[str, UrdfJoint]
    root_link: str
    text_sha256: str

    # ------------------------------------------------------------ 木
    def parent_joint(self, link: str) -> UrdfJoint | None:
        return next((j for j in self.joints.values() if j.child == link), None)

    def child_joints(self, link: str) -> list[UrdfJoint]:
        return [j for j in self.joints.values() if j.parent == link]

    def is_leaf(self, link: str) -> bool:
        return not self.child_joints(link)

    def fk_zero(self, link: str) -> np.ndarray:
        """関節値 0 のときの、ルートリンクの座標系で見た link の座標系(4x4、列ベクトル形式)。"""
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

    def fk(self, link: str, q: dict[str, float]) -> np.ndarray:
        """関節値 q(関節名 -> rad か m。無い関節は 0)のときの、ルートリンクの座標系で見た link の座標系。"""
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
            axis = np.asarray(j.axis, dtype=float)
            axis = axis / (np.linalg.norm(axis) or 1.0)
            if j.type in ("revolute", "continuous"):
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                motion[:3, :3] = np.eye(3) + math.sin(v) * k + (1 - math.cos(v)) * (k @ k)
            elif j.type == "prismatic":
                motion[:3, 3] = axis * v
            m = m @ j.origin @ motion
        return m

    def fixed_descendants(self, link: str) -> list[str]:
        """fixed joint だけでたどれる子孫のリンク(近い順)。"""
        out, queue = [], [link]
        while queue:
            cur = queue.pop(0)
            for j in self.child_joints(cur):
                if j.type == "fixed":
                    out.append(j.child)
                    queue.append(j.child)
        return out

    def mimic_pairs(self) -> list[tuple[UrdfJoint, UrdfJoint]]:
        """(follower, leader) の組。"""
        out = []
        for j in self.joints.values():
            if j.mimic and j.mimic.joint in self.joints:
                out.append((j, self.joints[j.mimic.joint]))
        return out

    def has_collisions(self) -> bool:
        return any(lnk.collisions for lnk in self.links.values())

    # ------------------------------------------------------------ メッシュ
    def mesh_files(self, ros_package_paths: tuple[tuple[str, str], ...] = ()) -> list[str]:
        """参照しているメッシュのファイルパス(解決できたもの)。import をやり直すべきかの判定に使う。"""
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
    g = el.find("geometry")
    if g is None or len(g) == 0:
        return None
    shape = g[0]
    return UrdfGeometry(shape.tag, shape.get("filename"), _origin(el.find("origin")))


def _zero_inertial(el: ET.Element | None) -> bool:
    """URDF の変換器の _is_zero_inertial と同じ判定。"""
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


def load_urdf(path: str) -> UrdfModel:
    with open(path, "rb") as f:
        raw = f.read()
    root = ET.fromstring(raw)
    links: dict[str, UrdfLink] = {}
    for el in root.findall("link"):
        vis = tuple(g for g in (_geometry(v) for v in el.findall("visual")) if g is not None)
        col = tuple(g for g in (_geometry(c) for c in el.findall("collision")) if g is not None)
        links[el.get("name")] = UrdfLink(el.get("name"), vis, col, _zero_inertial(el.find("inertial")))
    joints: dict[str, UrdfJoint] = {}
    for el in root.findall("joint"):
        limit = el.find("limit")
        dyn = el.find("dynamics")
        mim = el.find("mimic")
        joints[el.get("name")] = UrdfJoint(
            name=el.get("name"), type=el.get("type"), parent=el.find("parent").get("link"), child=el.find("child").get("link"),
            origin=_origin(el.find("origin")),
            axis=_floats(el.find("axis").get("xyz") if el.find("axis") is not None else "1 0 0", 3),  # type: ignore[arg-type]
            lower=float(limit.get("lower", 0.0)) if limit is not None else 0.0,
            upper=float(limit.get("upper", 0.0)) if limit is not None else 0.0,
            has_limit=limit is not None and (limit.get("lower") is not None or limit.get("upper") is not None),
            damping=float(dyn.get("damping")) if dyn is not None and dyn.get("damping") is not None else None,
            mimic=UrdfMimic(mim.get("joint"), float(mim.get("multiplier", 1.0)), float(mim.get("offset", 0.0))) if mim is not None else None,
        )
    children = {j.child for j in joints.values()}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"{path}: ルートリンクが 1 つに決まらない(閉ループか、つながっていないリンクがある): {roots}")
    return UrdfModel(path=os.path.abspath(path), name=root.get("name"), links=links, joints=joints, root_link=roots[0],
                     text_sha256=hashlib.sha256(raw).hexdigest())
