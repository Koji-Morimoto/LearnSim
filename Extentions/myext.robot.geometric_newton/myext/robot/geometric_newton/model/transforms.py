"""SE(3) 同次変換と SO(3) 回転表現のユーティリティ (NumPy のみ)。

本パッケージ全体で統一する数学規約:
    - **4x4 同次変換行列**: 列ベクトルに左から掛ける形式 (`p_a = T_ab @ p_b`、並進ベクトルは第 3 列 `T[..., :3, 3]`)。
      ※ USD の `Gf.Matrix4d` および Fabric の `omni:fabric:worldMatrix` は行ベクトル形式 (第 3 行が並進) であるため、
      境界 (`usd_reader.py` / `writer.py` / `newton_backend.py`) で転置変換を行う。
    - **四元数**: スカラー先頭の `(w, x, y, z)` 形式 (単位四元数、`w >= 0` の正半球へ正規化)。
      ※ Warp / Newton の `wp.quat` / `body_q[..., 3:7]` は `(x, y, z, w)` 形式であるため、`newton_backend.py` 境界で変換する。
    - **角度・長さの単位**: 角度は常にラジアン (`rad`)。長さの単位はモデル作成元の単位 (通常メートル) をそのまま保持する。
    - **バッチ対応**: 全関数が先頭の任意バッチ次元 `(...)` に対するブロードキャスト計算に対応する。
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


# ==============================================================================
# 同次変換行列 (SE(3)) の生成・逆変換
# ==============================================================================


def identity() -> np.ndarray:
    """4x4 の単位同次変換行列を返す。"""
    return np.eye(4)


def make_transform(rotation: np.ndarray | None = None, position: np.ndarray | None = None) -> np.ndarray:
    """回転行列 `(..., 3, 3)` と位置 `(..., 3)` から列ベクトル形式の 4x4 同次変換行列 `(..., 4, 4)` を組む。"""
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    position = np.zeros(3) if position is None else np.asarray(position, dtype=float)
    shape = np.broadcast_shapes(rotation.shape[:-2], position.shape[:-1])
    out = np.zeros(shape + (4, 4))
    out[..., :3, :3] = rotation
    out[..., :3, 3] = position
    out[..., 3, 3] = 1.0
    return out


def invert(transform: np.ndarray) -> np.ndarray:
    """剛体変換 `T = [R, p; 0, 1]` の解析的逆変換 `[R^T, -R^T p; 0, 1]` を返す。

    `np.linalg.inv` より高速かつ高精度だが、回転部分が直交行列 (`R^T R = I`) であることを前提とする
    (スケールやせん断を含む行列は先に `usd_reader.split_scale` でスケールを分離してから渡す)。
    """
    transform = np.asarray(transform, dtype=float)
    rot_t = np.swapaxes(transform[..., :3, :3], -1, -2)
    out = np.zeros_like(transform)
    out[..., :3, :3] = rot_t
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", rot_t, transform[..., :3, 3])
    out[..., 3, 3] = 1.0
    return out


def translation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    """並進 `(x, y, z)` のみの 4x4 同次変換行列を返す。"""
    out = np.eye(4)
    out[:3, 3] = (x, y, z)
    return out


# ==============================================================================
# 軸角・基本回転・RPY (SO(3))
# ==============================================================================


def rotation_about_axis(axis: np.ndarray, angle: np.ndarray | float) -> np.ndarray:
    """単位ベクトル `axis (3,)` のまわりに `angle (...)` [rad] 回す回転行列 `(..., 3, 3)` を Rodrigues の公式で計算する。

    `R = I + sin(q) * [axis]_x + (1 - cos(q)) * [axis]_x^2` (右ねじの向きが正)。
    """
    axis = np.asarray(axis, dtype=float)
    angle = np.asarray(angle, dtype=float)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    sin = np.sin(angle)[..., None, None]
    cos = np.cos(angle)[..., None, None]
    return np.eye(3) + sin * skew + (1.0 - cos) * (skew @ skew)


def rot_x(angle: float) -> np.ndarray:
    """X 軸まわり `angle` [rad] 回転の 4x4 同次変換行列 (主に DH / 修正 DH 用)。"""
    return make_transform(rotation_about_axis(np.array([1.0, 0.0, 0.0]), angle))


def rot_z(angle: float) -> np.ndarray:
    """Z 軸まわり `angle` [rad] 回転の 4x4 同次変換行列 (主に DH / 修正 DH 用)。"""
    return make_transform(rotation_about_axis(np.array([0.0, 0.0, 1.0]), angle))


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 規約の固定軸 RPY (`X -> Y -> Z` の順) から回転行列 `(3, 3)` を計算する (`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`)。"""
    rx = rotation_about_axis(np.array([1.0, 0.0, 0.0]), roll)
    ry = rotation_about_axis(np.array([0.0, 1.0, 0.0]), pitch)
    rz = rotation_about_axis(np.array([0.0, 0.0, 1.0]), yaw)
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    """`rpy_to_matrix` の逆変換 `(roll, pitch, yaw)`。ジンバルロック (`pitch = ±pi/2`) では `roll = 0.0` に一意化する。"""
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


# ==============================================================================
# 四元数 (w, x, y, z) と回転ベクトル
# ==============================================================================


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """四元数 `(..., 4)` (`(w, x, y, z)`) を正規化してから回転行列 `(..., 3, 3)` へ変換する。"""
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
    """回転行列 `(..., 3, 3)` を四元数 `(..., 4)` (`(w, x, y, z)`, `w >= 0`) へ変換する。

    なぜ Shepperd 法の 4 分岐を使うか:
        回転角が `pi` に近いとき `1 + trace(R) -> 0` となり単純式ではゼロ割・桁落ちを起こす。
        4 成分のうち最大値を要素ごとに選んで分母 `s` とすることで、全 SO(3) 領域で数値安定に計算できる。
        また二重被覆 (`q` と `-q` が同一回転) の符号揺れを防ぐため `w >= 0` の半球へ揃えている。
    """
    r = np.asarray(rotation, dtype=float)
    m00, m11, m22 = r[..., 0, 0], r[..., 1, 1], r[..., 2, 2]
    # 4 通りの分岐から分母が最大になる枝を要素ごとに選択する (バッチ対応 Shepperd 法)
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
    """回転行列 `(..., 3, 3)` を回転ベクトル `axis * angle` (`(..., 3)`, `angle in [0, pi]`) へ変換する。

    なぜ `acos(trace)` ではなく四元数経由の `2 * atan2(||v||, w)` を使うか:
        微小角 (`angle -> 0`) で `acos(1 - eps)` は桁落ちして精度が 1/2 桁落ちるが、
        `atan2(norm, w)` と `norm -> 0` 極限 `scale -> 2.0` を使うことで、IK の姿勢誤差計算でも高精度を保てる。
    """
    q = matrix_to_quat(rotation)
    vec = q[..., 1:]
    norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(norm, q[..., :1])
    scale = np.where(norm > 1e-12, angle / np.maximum(norm, 1e-12), 2.0)
    return vec * scale


def is_identity(transform: np.ndarray, tol: float = 1e-9) -> bool:
    """4x4 行列が許容誤差 `tol` の範囲で単位行列と等しいか判定する。"""
    return bool(np.allclose(transform, np.eye(4), atol=tol))
