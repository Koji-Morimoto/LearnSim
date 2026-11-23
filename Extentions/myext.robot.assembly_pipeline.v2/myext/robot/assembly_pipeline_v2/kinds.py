"""パイプライン全体で共有する列挙型(`Literal`)と実行時検証ヘルパー。

責務:
  - 設定値・規則 ID・最終判定など「決まった文字列しか取らない値」の定義を一箇所に集約する。
  - 実行時の許容値タプルは `typing.get_args` で `Literal` 型から自動導出し、型定義と実行時チェックの乖離を防ぐ。
  - Isaac Sim / pxr に一切依存せず、純 Python 環境で import 可能にする。

主な入出力・公開 API:
  - 型定義と許容値一覧:
    `PhysicsVariant` / `PHYSICS_VARIANTS`, `RuleId` / `RULE_IDS`,
    `OnViolation` / `ON_VIOLATION_VALUES`, `FixMode` / `FIX_MODES`, `Verdict` / `VERDICTS`
  - バリデータ: `check_literal(value, kind, what)`

呼ばれ元:
  - `__init__.py`（公開型の再エクスポート）
  - `spec.py`（TOML 読み込みおよび dataclass バリデーション）
  - `rules.py`（規則 ID `R01`〜`R14` と最終判定 `Verdict` の型付け）

拡張・変更ポイント:
  - 新しい検査規則（例: `R15`）を追加する場合は、このファイルの `RuleId` に追記したうえで
    `rules.py` の `RULES` に判定ロジックを追加する。
"""

from __future__ import annotations

from typing import Literal, get_args

# Physics VariantSet の選び方。
# "auto" は physx があれば physx、無ければ physics を優先選択する(詳細は usd_util.physics_variant_choice)。
PhysicsVariant = Literal["auto", "physx", "physics", "mujoco"]
PHYSICS_VARIANTS: tuple[str, ...] = get_args(PhysicsVariant)

# 事前チェックおよび修正ルールの規則 ID。各規則の定義本体は rules.py の RULES。
RuleId = Literal["R01", "R02", "R03", "R04", "R05", "R06", "R07", "R08", "R09", "R10", "R11", "R12", "R13", "R14"]
RULE_IDS: tuple[str, ...] = get_args(RuleId)

# 事前チェックで違反を検知したときの挙動。
# "stop" は結合ファイルを作る前に停止、"continue" は結合を進めて事後プローブで答え合わせを行う。
OnViolation = Literal["stop", "continue"]
ON_VIOLATION_VALUES: tuple[str, ...] = get_args(OnViolation)

# 既知不具合への修正を適用するかどうか。
# "skip" にすると修正を入れず、比較・再現検証用の出力を生成する。
FixMode = Literal["apply", "skip"]
FIX_MODES: tuple[str, ...] = get_args(FixMode)

# 規則ごとの最終判定（rules.py の evaluate_all が決定し、report.py が出力する）。
#   ok             : 事前違反なし・症状なし
#   fixed          : 事前違反あり。公式のままでは症状が出たが、修正後は解消した
#   upstream_fixed : 事前違反あり。公式のままでも症状が出なかった(Isaac Sim 側で修正済みの可能性)
#   still_broken   : 修正後(または fix="skip" の出力)でも症状が残っている
#   unexpected     : 事前違反なしなのに公式のままで症状が出た(未検出の新原因の可能性)
#   not_measured   : 対象が無い、または測定処理に失敗して症状を測れなかった
Verdict = Literal["ok", "fixed", "upstream_fixed", "still_broken", "unexpected", "not_measured"]
VERDICTS: tuple[str, ...] = get_args(Verdict)


def check_literal(value: object, kind: object, what: str) -> None:
    """`value` が `Literal` 型または許容値コレクション `kind` に含まれるかを検証する。

    引数:
      value: 検査対象の値。
      kind: `typing.Literal[...]` 型、または許容値の tuple / list / set。
      what: エラーメッセージに表示する設定項目名（利用者が TOML 箇所を特定できる文言）。

    例外:
      ValueError: `value` が許容値に含まれない場合。
    """
    allowed = tuple(kind) if isinstance(kind, (tuple, list, set)) else get_args(kind)
    if value not in allowed:
        raise ValueError(f"{what} は {allowed} のどれか: {value!r}")


__all__ = [
    "PhysicsVariant",
    "PHYSICS_VARIANTS",
    "RuleId",
    "RULE_IDS",
    "OnViolation",
    "ON_VIOLATION_VALUES",
    "FixMode",
    "FIX_MODES",
    "Verdict",
    "VERDICTS",
    "check_literal",
]
