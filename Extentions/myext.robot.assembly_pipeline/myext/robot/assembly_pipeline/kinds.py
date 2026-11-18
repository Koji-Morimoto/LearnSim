"""決まった値しか取らないものの型。取りうる値の一覧はここにだけ書き、実行時の確認は typing.get_args で取り出す。"""

from __future__ import annotations

from typing import Literal, get_args

# Physics variant の選び方。auto = physx があれば physx、無ければ physics
PhysicsVariant = Literal["auto", "physx", "physics", "mujoco"]

# 事前チェックの規則 ID。中身は rules.py の RULES
RuleId = Literal["R01", "R02", "R03", "R04", "R05", "R06", "R07", "R08", "R09", "R10", "R11", "R12", "R13", "R14"]

# 判定が出たときの動き。stop = 結合ファイルを作る前に止める、continue = 進めて最後に答え合わせする
OnViolation = Literal["stop", "continue"]

# 修正を入れるか。skip にすると、修正なしの結果がそのまま最終の出力になる
FixMode = Literal["apply", "skip"]

# 規則ごとの最終判定
#   ok             : 判定なし・症状なし
#   fixed          : 判定あり。公式のままでは症状が出た。修正後は出なかった
#   upstream_fixed : 判定あり。公式のままでも症状が出なかった(公式側の修正の可能性)
#   still_broken   : 修正後(または修正を skip した出力)でも症状が出た
#   unexpected     : 判定なしなのに、公式のままで症状が出た(新しい原因の可能性)
#   not_measured   : 症状を測れなかった(対象が無い、測定が失敗した)
Verdict = Literal["ok", "fixed", "upstream_fixed", "still_broken", "unexpected", "not_measured"]


def check_literal(value: object, kind: object, what: str) -> None:
    """value が Literal の型 kind の値でなければ ValueError。"""
    allowed = get_args(kind)
    if value not in allowed:
        raise ValueError(f"{what} は {allowed} のどれか: {value!r}")
