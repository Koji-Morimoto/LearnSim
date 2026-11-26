"""後方互換 re-export シム (実体は `usd/layer.py`)。"""

from .usd.layer import *  # noqa: F401,F403
from .usd.layer import _Entry, _setting, authoring, is_authoring, is_collision_enabled_ignoring_geometric_layer
