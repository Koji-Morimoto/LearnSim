"""URDF の読み込みと書き出し(運動学だけ)。

読み込み: joint の origin・axis・type・limit・mimic を読む。type は revolute / continuous / prismatic / fixed に対応。
書き出し: URDF は「子リンクの座標系 = 関節の座標系」と決まっているので、モデルのリンク座標系が関節の位置に無い場合は、
          URDF のリンク <名前> を関節の位置に置き、モデル本来のリンク座標系を固定の子リンク <名前>_frame として足す。
          モデルのリンク座標系がもともと関節の位置にあれば(DH・URDF 由来など)、_frame リンクは出ない。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from . import transforms as tf
from .spec import GeometricJointSpec, GeometricLinkSpec, GeometricMimicSpec, GeometricRobotModel

__all__ = ["read_urdf", "read_urdf_string", "model_to_urdf", "FRAME_SUFFIX"]

FRAME_SUFFIX = "_frame"


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return default if text is None else tuple(float(v) for v in text.split())


def read_urdf(path: str, name: str | None = None) -> GeometricRobotModel:
    with open(path, encoding="utf-8") as f:
        return read_urdf_string(f.read(), name=name)


def read_urdf_string(text: str, name: str | None = None) -> GeometricRobotModel:
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


def _fmt(values) -> str:
    return " ".join(f"{0.0 if abs(v) < 5e-17 else v:.17g}" for v in values)


def _safe(name: str) -> str:
    return name.replace("/", "__")


def model_to_urdf(model: GeometricRobotModel, robot_name: str | None = None) -> str:
    """モデルを URDF 文字列にする。リンク名・関節名の "/" は "__" に置き換える。"""
    root = ET.Element("robot", name=_safe(robot_name or model.name))
    # URDF のリンク座標系 = モデルのリンク座標系 × offset[link](根は単位行列、ほかは親関節の child_frame)
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
            # URDF の revolute / prismatic は limit が必須。範囲が無限のときは十分広い値を書く
            lower = joint.lower_limit if np.isfinite(joint.lower_limit) else -1.0e6
            upper = joint.upper_limit if np.isfinite(joint.upper_limit) else 1.0e6
            ET.SubElement(el, "limit", lower=_fmt([lower]), upper=_fmt([upper]), effort="0", velocity="0")
            if joint.mimic is not None:
                ET.SubElement(el, "mimic", joint=_safe(joint.mimic.reference_joint), multiplier=_fmt([joint.mimic.multiplier]), offset=_fmt([joint.mimic.offset]))
    for link in model.links:
        off = offset[link.name]
        if tf.is_identity(off):
            continue
        frame = _safe(link.name) + FRAME_SUFFIX
        ET.SubElement(root, "link", name=frame)
        el = ET.SubElement(root, "joint", name=frame + "_fixed", type="fixed")
        inv = tf.invert(off)
        ET.SubElement(el, "origin", xyz=_fmt(inv[:3, 3]), rpy=_fmt(tf.matrix_to_rpy(inv[:3, :3])))
        ET.SubElement(el, "parent", link=_safe(link.name))
        ET.SubElement(el, "child", link=frame)
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"
