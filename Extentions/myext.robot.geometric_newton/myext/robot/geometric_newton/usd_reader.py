"""後方互換 re-export シム (実体は `usd/reader.py`)。"""

from .usd.reader import *  # noqa: F401,F403
from .usd.reader import _np, component_patterns, prim_matches, split_scale, split_scale_batch
