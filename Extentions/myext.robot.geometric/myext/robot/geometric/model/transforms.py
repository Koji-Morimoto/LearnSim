"""同次変換の小さな道具(numpy だけ)。

約束:
    - 4x4 行列は「列ベクトルに左から掛ける」形。T_ab は「b 座標系の点を a 座標系で表す」変換。
    - 四元数は (w, x, y, z)。
    - 角度はラジアン。長さの単位は決めない(呼ぶ側の単位をそのまま使う)。
    - どの関数も、先頭に任意の個数の次元(まとめて計算する件数)を付けられる。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "identity",
    "make_transform",
    "invert",
    "translation",
    "rotation_about_axis",
    "rot_x",
    "rot_z",
    "rpy_to_matrix",
    "matrix_to_rpy",
    "quat_to_matrix",
    "matrix_to_quat",
    "rotation_vector",
    "is_identity",
]


def identity() -> np.ndarray:
    return np.eye(4)


def make_transform(rotation: np.ndarray | None = None, position: np.ndarray | None = None) -> np.ndarray:
    """回転行列 (..., 3, 3) と位置 (..., 3) から 4x4 (..., 4, 4) を作る。"""
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    position = np.zeros(3) if position is None else np.asarray(position, dtype=float)
    shape = np.broadcast_shapes(rotation.shape[:-2], position.shape[:-1])
    out = np.zeros(shape + (4, 4))
    out[..., :3, :3] = rotation
    out[..., :3, 3] = position
    out[..., 3, 3] = 1.0
    return out


def invert(transform: np.ndarray) -> np.ndarray:
    """剛体変換の逆(回転は転置で求める。スケールやせん断を含む行列には使えない)。"""
    transform = np.asarray(transform, dtype=float)
    rot_t = np.swapaxes(transform[..., :3, :3], -1, -2)
    out = np.zeros_like(transform)
    out[..., :3, :3] = rot_t
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", rot_t, transform[..., :3, 3])
    out[..., 3, 3] = 1.0
    return out


def translation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    out = np.eye(4)
    out[:3, 3] = (x, y, z)
    return out


def rotation_about_axis(axis: np.ndarray, angle: np.ndarray | float) -> np.ndarray:
    """単位ベクトル axis (3,) のまわりに angle (...) だけ回す回転行列 (..., 3, 3)。右ねじの向きが正。"""
    axis = np.asarray(axis, dtype=float)
    angle = np.asarray(angle, dtype=float)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    sin = np.sin(angle)[..., None, None]
    cos = np.cos(angle)[..., None, None]
    return np.eye(3) + sin * skew + (1.0 - cos) * (skew @ skew)


def rot_x(angle: float) -> np.ndarray:
    return make_transform(rotation_about_axis(np.array([1.0, 0.0, 0.0]), angle))


def rot_z(angle: float) -> np.ndarray:
    return make_transform(rotation_about_axis(np.array([0.0, 0.0, 1.0]), angle))


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF の rpy(固定軸 X→Y→Z の順)から回転行列 (3, 3)。R = Rz(yaw) Ry(pitch) Rx(roll)。"""
    rx = rotation_about_axis(np.array([1.0, 0.0, 0.0]), roll)
    ry = rotation_about_axis(np.array([0.0, 1.0, 0.0]), pitch)
    rz = rotation_about_axis(np.array([0.0, 0.0, 1.0]), yaw)
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    """rpy_to_matrix の逆。pitch が ±90° のときは roll = 0 に決める。"""
    r = np.asarray(rotation, dtype=float)
    sy = -r[2, 0]
    if abs(sy) < 1.0 - 1e-10:
        pitch = float(np.arcsin(sy))
        roll = float(np.arctan2(r[2, 1], r[2, 2]))
        yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    else:
        pitch = float(np.pi / 2 * np.sign(sy))
        roll = 0.0
        yaw = float(np.arctan2(-r[0, 1], r[1, 1]))
    return roll, pitch, yaw


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """四元数 (..., 4) (w, x, y, z) から回転行列 (..., 3, 3)。正規化してから使う。"""
    q = np.asarray(quat, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    out = np.empty(q.shape[:-1] + (3, 3))
    out[..., 0, 0] = 1 - 2 * (y * y + z * z)
    out[..., 0, 1] = 2 * (x * y - w * z)
    out[..., 0, 2] = 2 * (x * z + w * y)
    out[..., 1, 0] = 2 * (x * y + w * z)
    out[..., 1, 1] = 1 - 2 * (x * x + z * z)
    out[..., 1, 2] = 2 * (y * z - w * x)
    out[..., 2, 0] = 2 * (x * z - w * y)
    out[..., 2, 1] = 2 * (y * z + w * x)
    out[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return out


def matrix_to_quat(rotation: np.ndarray) -> np.ndarray:
    """回転行列 (..., 3, 3) から四元数 (..., 4) (w, x, y, z)。w >= 0 に揃える。"""
    r = np.asarray(rotation, dtype=float)
    m00, m11, m22 = r[..., 0, 0], r[..., 1, 1], r[..., 2, 2]
    # 4 通りの式のうち、分母がいちばん大きくなるものを要素ごとに選ぶ(数値的に安定)
    cand = np.stack([1 + m00 + m11 + m22, 1 + m00 - m11 - m22, 1 - m00 + m11 - m22, 1 - m00 - m11 + m22], axis=-1)
    best = np.argmax(cand, axis=-1)
    s = 2.0 * np.sqrt(np.maximum(np.take_along_axis(cand, best[..., None], axis=-1)[..., 0], 1e-30))
    q = np.empty(r.shape[:-2] + (4,))
    opts = [
        (s / 4, (r[..., 2, 1] - r[..., 1, 2]) / s, (r[..., 0, 2] - r[..., 2, 0]) / s, (r[..., 1, 0] - r[..., 0, 1]) / s),
        ((r[..., 2, 1] - r[..., 1, 2]) / s, s / 4, (r[..., 0, 1] + r[..., 1, 0]) / s, (r[..., 0, 2] + r[..., 2, 0]) / s),
        ((r[..., 0, 2] - r[..., 2, 0]) / s, (r[..., 0, 1] + r[..., 1, 0]) / s, s / 4, (r[..., 1, 2] + r[..., 2, 1]) / s),
        ((r[..., 1, 0] - r[..., 0, 1]) / s, (r[..., 0, 2] + r[..., 2, 0]) / s, (r[..., 1, 2] + r[..., 2, 1]) / s, s / 4),
    ]
    for k, comps in enumerate(opts):
        mask = best == k
        for c in range(4):
            q[..., c] = np.where(mask, comps[c], q[..., c] if k else comps[c])
    q = np.where(q[..., :1] < 0, -q, q)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """回転行列 (..., 3, 3) を「軸 × 角度」のベクトル (..., 3) にする(角度は 0〜π)。"""
    q = matrix_to_quat(rotation)
    vec = q[..., 1:]
    norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(norm, q[..., :1])
    scale = np.where(norm > 1e-12, angle / np.maximum(norm, 1e-12), 2.0)
    return vec * scale


def is_identity(transform: np.ndarray, tol: float = 1e-9) -> bool:
    return bool(np.allclose(transform, np.eye(4), atol=tol))
