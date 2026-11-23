"""干渉チェックの除外ルール管理者 (`GeometricCollisionFilter`, 標準ライブラリのみで動作)。

Newton バックエンド (`GeometricScene` / `BatchedNewtonScene`) と PhysX バックエンド (`GeometricCollisionChecker`) の
両方が共通で参照する「リンク名ベース」の無視ペアフィルタ。

ルールは 3 層で構成され、`reason_for(link_a, link_b)` は次の優先順で評価し、最初に一致した理由 (`reason`) を返す:
    1. **名指しペア (`add_pair`)**:    `frozenset((link_a, link_b))` による O(1) 辞書引き (順不同)。
    2. **glob パターン (`add_pattern`)**: `fnmatch.fnmatchcase` によるワイルドカード照合 (順不同)。
    3. **関数ルール (`add_rule`)**:    任意述語 `rule(a, b) or rule(b, a)` (順不同)。

設計上の意図 (`reason` タグによる管理):
    自動登録される隣接ペア (`reason="adjacent"`)、取り付け部 (`reason="mount"`)、USD 既存の `FilteredPairsAPI`
    (`reason="usd_filtered_pairs"`)、初期接触ペア (`reason="touching"`)、ユーザー追加 (`reason="user"`) を
    すべて同じ仕組みへ収め、`reason` タグで区別している。これにより、たとえばグリッパ換装 (VariantSet 切替) 時に
    ユーザー追加ルールを保ったまま古い自動ルールだけを `remove_by_reason(...)` で安全に入れ替えられる。
"""

from __future__ import annotations

import fnmatch
import json
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from .spec import GeometricRobotModel

__all__ = ["GeometricCollisionFilter"]

PairRule = Callable[[str, str], bool]


# ==============================================================================
# GeometricCollisionFilter 本体
# ==============================================================================


class GeometricCollisionFilter:
    """リンク名ペア単位の衝突除外フィルタ (順不同・`reason` タグ付き・JSON 永続化対応)。"""

    def __init__(self) -> None:
        # 順序に依存しないよう `frozenset((a, b))` をキーにする
        self._pairs: dict[frozenset[str], str] = {}
        self._patterns: list[tuple[str, str, str]] = []
        self._rules: list[tuple[PairRule, str]] = []

    # ==========================================================================
    # 生成・複製・結合 (attach 用)
    # ==========================================================================

    @classmethod
    def from_model(cls, model: "GeometricRobotModel") -> "GeometricCollisionFilter":
        """関節で直接結ばれた隣接リンクペアを自動登録したフィルタを作る (`<name>/mount` は `reason="mount"`, 他は `"adjacent"`)。"""
        out = cls()
        for parent, child, joint in model.adjacent_link_pairs():
            out.add_pair(parent, child, reason="mount" if joint.endswith("/mount") else "adjacent")
        return out

    def copy(self) -> "GeometricCollisionFilter":
        out = GeometricCollisionFilter()
        out._pairs = dict(self._pairs)
        out._patterns = list(self._patterns)
        out._rules = list(self._rules)
        return out

    def merged_with(self, other: "GeometricCollisionFilter", *, prefix: str = "", other_prefix: str = "") -> "GeometricCollisionFilter":
        """2 つのフィルタを結合した新フィルタを返す (`GeometricRobotModel.attach` と同じプレフィックス規約)。

        名指しペアとパターンには `prefix` / `other_prefix` を前置して名前空間を整合させる。
        関数ルール (`_rules`) はクロージャ内のリンク名を自動書き換えできないためそのまま引き継ぐ。
        """
        out = GeometricCollisionFilter()
        for src, pre in ((self, prefix), (other, other_prefix)):
            for pair, reason in src._pairs.items():
                out._pairs[frozenset(pre + n for n in pair)] = reason
            out._patterns += [(pre + a, pre + b, reason) for a, b, reason in src._patterns]
            out._rules += src._rules
        return out

    # ==========================================================================
    # ルールの追加・削除
    # ==========================================================================

    def add_pair(self, link_a: str, link_b: str, reason: str = "user") -> None:
        """名指しペア `(link_a, link_b)` を無視対象に追加する (同一ペアが既にあれば `reason` を上書き)。"""
        if link_a == link_b:
            raise ValueError("同じリンク同士の組は足せない")
        self._pairs[frozenset((link_a, link_b))] = reason

    def remove_pair(self, link_a: str, link_b: str) -> bool:
        """名指しペアを削除する。存在して削除できたら `True`。"""
        return self._pairs.pop(frozenset((link_a, link_b)), None) is not None

    def add_pattern(self, pattern_a: str, pattern_b: str, reason: str = "user") -> None:
        """`fnmatch` パターン組 `(pattern_a, pattern_b)` を無視対象に追加する (評価時は順不同)。"""
        self._patterns.append((pattern_a, pattern_b, reason))

    def add_rule(self, rule: PairRule, reason: str = "user") -> None:
        """述語関数 `rule(link_a, link_b) -> bool` を追加する (評価時は `(a, b)` と `(b, a)` の両順で判定)。"""
        self._rules.append((rule, reason))

    def remove_by_reason(self, reason: str) -> int:
        """`reason` タグが一致するルール (ペア・パターン・関数のすべて) を一括削除し、削除件数を返す。"""
        before = len(self._pairs) + len(self._patterns) + len(self._rules)
        self._pairs = {p: r for p, r in self._pairs.items() if r != reason}
        self._patterns = [p for p in self._patterns if p[2] != reason]
        self._rules = [r for r in self._rules if r[1] != reason]
        return before - (len(self._pairs) + len(self._patterns) + len(self._rules))

    def clear(self) -> None:
        self._pairs.clear()
        self._patterns.clear()
        self._rules.clear()

    # ==========================================================================
    # 評価・照会
    # ==========================================================================

    def reason_for(self, link_a: str, link_b: str) -> str | None:
        """ペア `(link_a, link_b)` を無視する理由を返す (`pairs` -> `patterns` -> `rules` の優先順)。無視しないなら `None`。"""
        reason = self._pairs.get(frozenset((link_a, link_b)))
        if reason is not None:
            return reason
        for pa, pb, why in self._patterns:
            if (fnmatch.fnmatchcase(link_a, pa) and fnmatch.fnmatchcase(link_b, pb)) or (fnmatch.fnmatchcase(link_b, pa) and fnmatch.fnmatchcase(link_a, pb)):
                return why
        for rule, why in self._rules:
            if rule(link_a, link_b) or rule(link_b, link_a):
                return why
        return None

    def is_ignored(self, link_a: str, link_b: str) -> bool:
        """ペア `(link_a, link_b)` がいずれかのルールで無視対象なら `True`。"""
        return self.reason_for(link_a, link_b) is not None

    @property
    def pairs(self) -> dict[tuple[str, str], str]:
        """登録済み名指しペアを辞書 `{(link_a, link_b): reason}` (リンク名昇順タプル) で返す。"""
        return {tuple(sorted(pair)): reason for pair, reason in self._pairs.items()}  # type: ignore[misc]

    @property
    def patterns(self) -> list[tuple[str, str, str]]:
        return list(self._patterns)

    @property
    def num_rules(self) -> int:
        return len(self._rules)

    # ==========================================================================
    # JSON シリアライズ / 復元
    # ==========================================================================

    def to_json(self, path: str | None = None) -> str:
        """名指しペアとパターンを JSON 文字列へ書き出す (関数ルールは直列化不可のため件数のみ記録)。"""
        data = {
            "pairs": [[*sorted(pair), reason] for pair, reason in sorted(self._pairs.items(), key=lambda kv: sorted(kv[0]))],
            "patterns": [list(p) for p in self._patterns],
            "unsaved_function_rules": len(self._rules),
        }
        text = json.dumps(data, ensure_ascii=False, indent=1)
        if path is not None:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        return text

    @classmethod
    def from_json(cls, text_or_path: str) -> "GeometricCollisionFilter":
        """JSON 文字列または JSON ファイルパスから `GeometricCollisionFilter` を復元する。"""
        text = text_or_path
        if not text_or_path.lstrip().startswith("{"):
            with open(text_or_path, encoding="utf-8") as f:
                text = f.read()
        data = json.loads(text)
        out = cls()
        for a, b, reason in data.get("pairs", []):
            out.add_pair(a, b, reason)
        for a, b, reason in data.get("patterns", []):
            out.add_pattern(a, b, reason)
        return out
