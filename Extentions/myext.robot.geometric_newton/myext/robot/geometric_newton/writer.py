"""後方互換 re-export シム (実体は `usd/writer.py`)。"""

from .usd.writer import *  # noqa: F401,F403
from .usd.writer import _get_gpu_fabric_kernel
