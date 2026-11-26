"""後方互換 re-export シム (実体は `collision/newton_*.py`)。"""

from .collision.newton_batched import *  # noqa: F401,F403
from .collision.newton_scene import *  # noqa: F401,F403
from .collision.newton_scene import _EnvShapeEntry, _ModelProxy, _RobotRecord
from .collision.newton_shapes import *  # noqa: F401,F403
from .collision.newton_shapes import (
    _AUTO_FK_MIN_BATCH,
    _AUTO_FK_MIN_DOFS,
    _COLLIDER_ATTRS,
    _add_usd_collider_shape,
    _axis_rot_to_z,
    _discover_stage_colliders,
    _has_active_rigid_body,
    _import_newton,
    _is_timeline_playing,
    _mat4_to_wp_transform,
    _stage_up_axis,
    _try_fabric_hierarchy,
    _wp_body_q_to_mat4,
)
