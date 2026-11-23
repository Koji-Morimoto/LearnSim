"""外部指令変換 (`GeometricCommandConverter`): 外部ログ・操作指令からロボット DOF 値への写像層 (NumPy のみ)。

`GeometricMimicSpec` との役割分担 (なぜ別モジュールに分けているか):
    - `GeometricMimicSpec` (mimic):
        ロボット機構そのものの物理的連動拘束 (平行リンク・左右対称グリッパ等)。FK・IK・衝突判定で常に適用される。
    - `GeometricCommandConverter` (本モジュール):
        外部データ形式 (実機ログの単位・符号・オフセット・グリッパ開き幅 1 スカラー -> 左右 2 軸など) と
        ロボットの DOF 名を仲介するアダプタ。`GeometricRobot` に差し込んだときだけ機能し、
        出力先を DOF **インデックスではなく関節名 (`output_dofs`)** で持つことで、VariantSet 切替等で
        DOF 数や並びが変わった再構築後でも壊れずに名前解決できる。

使用例 (ログは「グリッパ開き幅」1 変数、ロボットは左右 2 軸):
    conv = LinearCommandConverter(
        input_names=["gripper_width"],
        output_dofs=["left_jaw", "right_jaw"],
        matrix=[[0.5], [-0.5]],
    )
    conv.to_dof_positions([0.04])  # -> array([0.02, -0.02])

全コンバータが先頭の任意バッチ次元 `(...)` に対応 (`(..., num_inputs) -> (..., num_outputs)`)。
回転関節の単位は `output_angle_unit` (`"rad"` / `"deg"`) で宣言し、`GeometricRobot` 側が受け取り時にラジアンへ統一する。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Literal, Sequence, get_args

import numpy as np

__all__ = ["AngleUnit", "GeometricCommandConverter", "LinearCommandConverter", "InterpCommandConverter", "FunctionCommandConverter"]

# ==============================================================================
# 角度単位と抽象基底クラス
# ==============================================================================

AngleUnit = Literal["rad", "deg"]


def check_angle_unit(unit: str) -> AngleUnit:
    """角度単位が `"rad"` または `"deg"` であることを検証して返す。"""
    if unit not in get_args(AngleUnit):
        raise ValueError(f"角度の単位は {get_args(AngleUnit)} のどれか: {unit!r}")
    return unit  # type: ignore[return-value]


class GeometricCommandConverter(ABC):
    """コマンド変換の共通抽象クラス。

    input_names       : 外部指令側のチャンネル名 (`to_dof_positions` 入力の並び)
    output_dofs       : 書き込み先のロボット DOF 名 (`to_dof_positions` 出力の並び)
    output_angle_unit : 出力のうち回転関節成分の単位 (`"rad"` または `"deg"`)
    """

    def __init__(self, input_names: Sequence[str], output_dofs: Sequence[str], output_angle_unit: AngleUnit = "rad") -> None:
        self.input_names = tuple(input_names)
        self.output_dofs = tuple(output_dofs)
        self.output_angle_unit = check_angle_unit(output_angle_unit)
        if len(set(self.input_names)) != len(self.input_names) or len(set(self.output_dofs)) != len(self.output_dofs):
            raise ValueError("input_names / output_dofs に重複がある")

    @abstractmethod
    def to_dof_positions(self, command: np.ndarray) -> np.ndarray:
        """外部指令 `(..., num_inputs)` を DOF 値 `(..., num_outputs)` へ変換する。"""

    def from_dof_positions(self, dof_positions: np.ndarray) -> np.ndarray:
        """DOF 値 `(..., num_outputs)` から外部指令 `(..., num_inputs)` へ逆変換する (一意に定まらない変換では `NotImplementedError`)。"""
        raise NotImplementedError(f"{type(self).__name__} は逆変換を持たない")

    def _check_input(self, command: np.ndarray, n: int, what: str) -> np.ndarray:
        x = np.asarray(command, dtype=float)
        if x.shape[-1:] != (n,):
            raise ValueError(f"{what} の最後の次元は {n}: {x.shape}")
        return x


# ==============================================================================
# 線形変換・表補間・任意関数コンバータ
# ==============================================================================


class LinearCommandConverter(GeometricCommandConverter):
    """アフィン変換 `q = matrix @ command + offset` (`matrix` は形状 `(num_outputs, num_inputs)`)。

    単位換算・符号反転・オフセット補正・1 対多の線形分配を一括で表現できる。
    逆変換 (`from_dof_positions`) は `matrix` が列フルランク (出力から入力が一意に復元可能) のときのみ
    Moore-Penrose 擬似逆行列 `pinv(matrix)` で提供する。
    """

    def __init__(
        self,
        input_names: Sequence[str],
        output_dofs: Sequence[str],
        matrix: Sequence[Sequence[float]] | np.ndarray,
        offset: Sequence[float] | np.ndarray | None = None,
        output_angle_unit: AngleUnit = "rad",
    ) -> None:
        super().__init__(input_names, output_dofs, output_angle_unit)
        self.matrix = np.asarray(matrix, dtype=float).reshape(len(self.output_dofs), len(self.input_names))
        self.offset = np.zeros(len(self.output_dofs)) if offset is None else np.asarray(offset, dtype=float).reshape(len(self.output_dofs))
        # 列ランク不足 (例: 2 入力 -> 1 出力) では入力を一意に戻せないため `_pinv = None` にして誤復元を防ぐ
        rank = np.linalg.matrix_rank(self.matrix) if self.matrix.size else 0
        self._pinv = np.linalg.pinv(self.matrix) if rank == len(self.input_names) else None

    def to_dof_positions(self, command: np.ndarray) -> np.ndarray:
        return self._check_input(command, len(self.input_names), "command") @ self.matrix.T + self.offset

    def from_dof_positions(self, dof_positions: np.ndarray) -> np.ndarray:
        if self._pinv is None:
            raise NotImplementedError("matrix が列フルランクでないので、出力から入力が一意に決まらない")
        y = self._check_input(dof_positions, len(self.output_dofs), "dof_positions")
        return (y - self.offset) @ self._pinv.T


class InterpCommandConverter(GeometricCommandConverter):
    """1 スカラー入力から実測テーブルの区分線形補間で各 DOF 値を求めるコンバータ (非線形リンク機構・キャリブレーション用)。

    table_inputs  : 単調増加の入力サンプル列 `(K,)`。範囲外は両端値へクランプされる。
    table_outputs : 各入力に対応する DOF 値テーブル `(K, num_outputs)`。
    output_for_inverse : 逆変換に用いる単調な出力 DOF 名 (指定時のみ `from_dof_positions` が有効)。
    """

    def __init__(
        self,
        input_name: str,
        output_dofs: Sequence[str],
        table_inputs: Sequence[float],
        table_outputs: Sequence[Sequence[float]] | np.ndarray,
        output_angle_unit: AngleUnit = "rad",
        output_for_inverse: str | None = None,
    ) -> None:
        super().__init__([input_name], output_dofs, output_angle_unit)
        self.table_inputs = np.asarray(table_inputs, dtype=float)
        self.table_outputs = np.asarray(table_outputs, dtype=float).reshape(len(self.table_inputs), len(self.output_dofs))
        if len(self.table_inputs) < 2 or np.any(np.diff(self.table_inputs) <= 0):
            raise ValueError("table_inputs は 2 個以上の単調増加の列")
        self._inverse_column = None
        if output_for_inverse is not None:
            col = self.output_dofs.index(output_for_inverse)
            diffs = np.diff(self.table_outputs[:, col])
            if not (np.all(diffs > 0) or np.all(diffs < 0)):
                raise ValueError(f"{output_for_inverse} の値が単調でないので、逆変換に使えない")
            self._inverse_column = col

    def to_dof_positions(self, command: np.ndarray) -> np.ndarray:
        x = self._check_input(command, 1, "command")[..., 0]
        # `np.searchsorted` で全バッチ要素の所属区間 `k` を一括特定し、出力列方向のループなしで補間する
        k = np.clip(np.searchsorted(self.table_inputs, x, side="right") - 1, 0, len(self.table_inputs) - 2)
        x0, x1 = self.table_inputs[k], self.table_inputs[k + 1]
        t = np.clip((x - x0) / (x1 - x0), 0.0, 1.0)[..., None]
        return self.table_outputs[k] * (1.0 - t) + self.table_outputs[k + 1] * t

    def from_dof_positions(self, dof_positions: np.ndarray) -> np.ndarray:
        if self._inverse_column is None:
            raise NotImplementedError("output_for_inverse を指定していないので、逆変換できない")
        y = self._check_input(dof_positions, len(self.output_dofs), "dof_positions")[..., self._inverse_column]
        col = self.table_outputs[:, self._inverse_column]
        # 単調減少テーブルでも `np.interp` (単調増加前提) を使えるよう昇順インデックスで並び替える
        order = np.argsort(col)
        return np.interp(y, col[order], self.table_inputs[order])[..., None]


class FunctionCommandConverter(GeometricCommandConverter):
    """ユーザー定義関数 `forward(command) -> dof_positions` (および任意の `inverse`) による変換コンバータ。"""

    def __init__(
        self,
        input_names: Sequence[str],
        output_dofs: Sequence[str],
        forward: Callable[[np.ndarray], np.ndarray],
        inverse: Callable[[np.ndarray], np.ndarray] | None = None,
        output_angle_unit: AngleUnit = "rad",
    ) -> None:
        super().__init__(input_names, output_dofs, output_angle_unit)
        self._forward = forward
        self._inverse = inverse

    def to_dof_positions(self, command: np.ndarray) -> np.ndarray:
        out = np.asarray(self._forward(self._check_input(command, len(self.input_names), "command")), dtype=float)
        if out.shape[-1:] != (len(self.output_dofs),):
            raise ValueError(f"forward の出力の最後の次元が {len(self.output_dofs)} でない: {out.shape}")
        return out

    def from_dof_positions(self, dof_positions: np.ndarray) -> np.ndarray:
        if self._inverse is None:
            raise NotImplementedError("inverse を渡していないので、逆変換できない")
        return np.asarray(self._inverse(self._check_input(dof_positions, len(self.output_dofs), "dof_positions")), dtype=float)
