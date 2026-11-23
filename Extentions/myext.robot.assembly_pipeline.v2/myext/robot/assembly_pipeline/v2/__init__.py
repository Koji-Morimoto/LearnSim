"""Thin re-export module so `import myext.robot.assembly_pipeline.v2` works seamlessly."""

from __future__ import annotations

import myext.robot.assembly_pipeline_v2 as _v2
from myext.robot.assembly_pipeline_v2 import *  # noqa: F401,F403
from myext.robot.assembly_pipeline_v2 import __all__ as __all__


def __getattr__(name: str):
    return getattr(_v2, name)
