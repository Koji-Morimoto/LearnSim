"""干渉チェックで「無視する組」を決めるフィルタ(numpy も USD も使わない)。

ルールは 3 種類で、後から足したり外したりできる。どれか 1 つでも当てはまった組は無視する。

    filter.add_pair("link3", "link5", reason="常に近接")           # 組を名指しする
    filter.add_pattern("*/finger_*", "*/finger_*", reason="指同士")  # 名前のパターン(fnmatch)。順不同で当てはめる
    filter.add_rule(lambda a, b: a.startswith("left/") and b.startswith("left/"), reason="左腕の内側")

自動で入る組(関節でつながった隣同士 = "adjacent"、取り付け部 = "mount")も同じ仕組みに入るので、
remove_pair や remove_by_reason で個別に外せる。名前はリンク名(結合したモデルなら "left/link3" の形)。
"""

from __future__ import annotations

import fnmatch
import json
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from .spec import GeometricRobotModel

__all__ = ["GeometricCollisionFilter"]

PairRule = Callable[[str, str], bool]


class GeometricCollisionFilter:
    def __init__(self) -> None:
        self._pairs: dict[frozenset[str], str] = {}
        self._patterns: list[tuple[str, str, str]] = []
        self._rules: list[tuple[PairRule, str]] = []

    # ------------------------------------------------------------------ 作る

    @classmethod
    def from_model(cls, model: "GeometricRobotModel") -> "GeometricCollisionFilter":
        """関節で直接つながったリンクの組を入れたフィルタ。取り付けの関節("<名前>/mount")は reason="mount"。"""
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
        """2 つのフィルタを合わせた新しいフィルタ。prefix / other_prefix は、名指しの組とパターンの名前の前に付ける。

        関数のルールは名前を付け替えられないので、そのまま両方入れる。
        """
        out = GeometricCollisionFilter()
        for src, pre in ((self, prefix), (other, other_prefix)):
            for pair, reason in src._pairs.items():
                out._pairs[frozenset(pre + n for n in pair)] = reason
            out._patterns += [(pre + a, pre + b, reason) for a, b, reason in src._patterns]
            out._rules += src._rules
        return out

    # ------------------------------------------------------------------ ルールを足す・外す

    def add_pair(self, link_a: str, link_b: str, reason: str = "user") -> None:
        if link_a == link_b:
            raise ValueError("同じリンク同士の組は足せない")
        self._pairs[frozenset((link_a, link_b))] = reason

    def remove_pair(self, link_a: str, link_b: str) -> bool:
        return self._pairs.pop(frozenset((link_a, link_b)), None) is not None

    def add_pattern(self, pattern_a: str, pattern_b: str, reason: str = "user") -> None:
        self._patterns.append((pattern_a, pattern_b, reason))

    def add_rule(self, rule: PairRule, reason: str = "user") -> None:
        """rule(link_a, link_b) が True を返す組を無視する。順不同で 2 回(a, b)と(b, a)呼ばれる。"""
        self._rules.append((rule, reason))

    def remove_by_reason(self, reason: str) -> int:
        """reason が一致するルールをすべて外し、外した数を返す。"""
        before = len(self._pairs) + len(self._patterns) + len(self._rules)
        self._pairs = {p: r for p, r in self._pairs.items() if r != reason}
        self._patterns = [p for p in self._patterns if p[2] != reason]
        self._rules = [r for r in self._rules if r[1] != reason]
        return before - (len(self._pairs) + len(self._patterns) + len(self._rules))

    def clear(self) -> None:
        self._pairs.clear()
        self._patterns.clear()
        self._rules.clear()

    # ------------------------------------------------------------------ 調べる

    def reason_for(self, link_a: str, link_b: str) -> str | None:
        """この組を無視する理由。無視しないなら None。"""
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
        return self.reason_for(link_a, link_b) is not None

    @property
    def pairs(self) -> dict[tuple[str, str], str]:
        """名指しの組 {(リンク, リンク): 理由}。"""
        return {tuple(sorted(pair)): reason for pair, reason in self._pairs.items()}  # type: ignore[misc]

    @property
    def patterns(self) -> list[tuple[str, str, str]]:
        return list(self._patterns)

    @property
    def num_rules(self) -> int:
        return len(self._rules)

    # ------------------------------------------------------------------ 保存

    def to_json(self, path: str | None = None) -> str:
        """名指しの組とパターンを JSON にする(関数のルールは保存できないので入らない)。"""
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
