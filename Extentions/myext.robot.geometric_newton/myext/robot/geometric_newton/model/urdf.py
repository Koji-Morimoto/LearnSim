"""URDF XML の読み込みと書き出し (運動学のみ・NumPy + 標準 `xml.etree.ElementTree`)。

読み込み (`read_urdf` / `read_urdf_string`):
    各 `<joint>` の `origin` (`xyz`, `rpy`)・`axis`・`type` (`revolute` / `continuous` / `prismatic` / `fixed`)・
    `limit`・`mimic` を読み取り `GeometricRobotModel` を返す。`continuous` は可動範囲 `(-inf, +inf)` の `revolute` として扱う。

書き出し (`model_to_urdf`) の設計意図 (なぜ `FRAME_SUFFIX = "_frame"` 補助リンクが出ることがあるか):
    URDF 仕様では「子リンクの座標系 = 親関節の座標系 (`child_frame = I`)」と固定されている。
    一方、`GeometricRobotModel` (特に USD Articulation や DH / 修正 DH 由来) では `child_frame != I` が
    許容される。そこで書き出し時は:
      1. URDF 上のリンク `<name>` を親関節の座標系 (`モデル上のリンク座標系 @ child_frame`) に置き、
         子関節の `origin` は `inv(offset[parent]) @ parent_frame` へ補正する。
      2. `child_frame != I` のリンクに限り、モデル本来のリンク座標系を再現するための固定子リンク
         `<name>_frame` (`origin = inv(child_frame)`) を追加する。
    `child_frame == I` (URDF 由来モデル等) では余分な `_frame` リンクは一切生成されない。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from . import transforms as tf
from .spec import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, GeometricRobotModel

__all__ = ["read_urdf", "read_urdf_string", "model_to_urdf", "FRAME_SUFFIX"]

FRAME_SUFFIX = "_frame"


# ==============================================================================
# URDF 読み込み
# ==============================================================================


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return default if text is None else tuple(float(v) for v in text.split())


def read_urdf(path: str, name: str | None = None) -> GeometricRobotModel:
    """URDF ファイルを UTF-8 で読み込み、`GeometricRobotModel` を返す。"""
    with open(path, encoding="utf-8") as f:
        return read_urdf_string(f.read(), name=name)


def read_urdf_string(text: str, name: str | None = None) -> GeometricRobotModel:
    """URDF XML 文字列を解析して `GeometricRobotModel` を返す。"""
    root = ET.fromstring(text)
    if root.tag != "robot":
        raise ValueError("URDF の最上位の要素が <robot> ではない")
    links = [GeometricLinkSpec(el.attrib["name"]) for el in root.findall("link")]
    joints = []
    for el in root.findall("joint"):
        jname, jtype = el.attrib["name"], el.attrib.get("type", "fixed")
        if jtype not in ("revolute", "continuous", "prismatic", "fixed"):
            raise NotImplementedError(f"URDF の関節 {jname} の type={jtype!r} には対応していない(revolute / continuous / prismatic / fixed のみ)")
        origin_el = el.find("origin")
        xyz = _floats(None if origin_el is None else origin_el.attrib.get("xyz"), (0.0, 0.0, 0.0))
        rpy = _floats(None if origin_el is None else origin_el.attrib.get("rpy"), (0.0, 0.0, 0.0))
        axis_el = el.find("axis")
        # URDF 仕様の既定 `<axis>` は `(1, 0, 0)`
        axis = _floats(None if axis_el is None else axis_el.attrib.get("xyz"), (1.0, 0.0, 0.0))
        limit_el = el.find("limit")
        lower, upper = -np.inf, np.inf
        if jtype in ("revolute", "prismatic") and limit_el is not None:
            lower = float(limit_el.attrib.get("lower", 0.0))
            upper = float(limit_el.attrib.get("upper", 0.0))
        mimic_el = el.find("mimic")
        mimic = None
        if mimic_el is not None and jtype != "fixed":
            mimic = GeometricMimicSpec(mimic_el.attrib["joint"], float(mimic_el.attrib.get("multiplier", 1.0)), float(mimic_el.attrib.get("offset", 0.0)))
        joints.append(
            GeometricJointSpec(
                name=jname,
                joint_type="revolute" if jtype == "continuous" else jtype,  # type: ignore[arg-type]
                parent_link=el.find("parent").attrib["link"],  # type: ignore[union-attr]
                child_link=el.find("child").attrib["link"],  # type: ignore[union-attr]
                parent_frame=tf.make_transform(tf.rpy_to_matrix(*rpy), np.array(xyz)),
                axis=axis if jtype != "fixed" else (0.0, 0.0, 1.0),  # type: ignore[arg-type]
                lower_limit=lower,
                upper_limit=upper,
                mimic=mimic,
            )
        )
    return GeometricRobotModel(name=name or root.attrib.get("name", "urdf_robot"), links=tuple(links), joints=tuple(joints))


# ==============================================================================
# URDF 書き出し
# ==============================================================================


def _fmt(values) -> str:
    """浮動小数点数列を倍精度往復可能な文字列へ整形する (`-0.0` ノイズを `0.0` に丸める)。"""
    return " ".join(f"{0.0 if abs(v) < 5e-17 else v:.17g}" for v in values)


def _safe(name: str) -> str:
    """`attach()` で付く `/` 区切り文字を URDF XML 互換の `__` へ置換する。"""
    return name.replace("/", "__")


def model_to_urdf(model: GeometricRobotModel, robot_name: str | None = None) -> str:
    """`GeometricRobotModel` を運動学専用 URDF 文字列へ変換する (`/` は `__` に置換)。"""
    root = ET.Element("robot", name=_safe(robot_name or model.name))
    # URDF 上の各リンク座標系 = モデル上のリンク座標系 @ `offset[link]`
    # (根リンクは単位行列、それ以外は親関節の `child_frame` に一致させることで URDF の `child_frame = I` 制約を満たす)
    offset = {link.name: np.eye(4) for link in model.links}
    for joint in model.joints:
        offset[joint.child_link] = joint.child_frame
    for link in model.links:
        ET.SubElement(root, "link", name=_safe(link.name))
    for joint in model.joints:
        el = ET.SubElement(root, "joint", name=_safe(joint.name), type=joint.joint_type)
        origin = tf.invert(offset[joint.parent_link]) @ joint.parent_frame
        ET.SubElement(el, "origin", xyz=_fmt(origin[:3, 3]), rpy=_fmt(tf.matrix_to_rpy(origin[:3, :3])))
        ET.SubElement(el, "parent", link=_safe(joint.parent_link))
        ET.SubElement(el, "child", link=_safe(joint.child_link))
        if joint.is_dof:
            ET.SubElement(el, "axis", xyz=_fmt(joint.axis))
            # URDF の `revolute` / `prismatic` 要素は `<limit>` が必須のため、無制限 (`±inf`) 時は十分大きな有限値で代用する
            lower = joint.lower_limit if np.isfinite(joint.lower_limit) else -1.0e6
            upper = joint.upper_limit if np.isfinite(joint.upper_limit) else 1.0e6
            ET.SubElement(el, "limit", lower=_fmt([lower]), upper=_fmt([upper]), effort="0", velocity="0")
            if joint.mimic is not None:
                ET.SubElement(el, "mimic", joint=_safe(joint.mimic.reference_joint), multiplier=_fmt([joint.mimic.multiplier]), offset=_fmt([joint.mimic.offset]))
    for link in model.links:
        off = offset[link.name]
        if tf.is_identity(off):
            continue
        # `child_frame != I` のリンクだけ、モデル本来のリンク座標系 (`inv(child_frame)`) を固定子リンクで復元する
        frame = _safe(link.name) + FRAME_SUFFIX
        ET.SubElement(root, "link", name=frame)
        el = ET.SubElement(root, "joint", name=frame + "_fixed", type="fixed")
        inv = tf.invert(off)
        ET.SubElement(el, "origin", xyz=_fmt(inv[:3, 3]), rpy=_fmt(tf.matrix_to_rpy(inv[:3, :3])))
        ET.SubElement(el, "parent", link=_safe(link.name))
        ET.SubElement(el, "child", link=frame)
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"
