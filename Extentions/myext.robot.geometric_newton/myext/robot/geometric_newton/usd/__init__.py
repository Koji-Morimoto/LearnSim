"""OpenUSD / Fabric 入出力層 (Layer 1: `model/` のみに依存)。"""

from .builder import GeometricVisualStyle, build_robot_prims, safe_prim_name
from .layer import (
    LAYER_MARKER,
    XFORM_OP_NAME,
    CollisionMode,
    GeometricLayer,
    GeometricLayerOptions,
    GeometricLayerReport,
    LayerTarget,
    authoring,
    is_authoring,
    is_collision_enabled_ignoring_geometric_layer,
)
from .reader import (
    UsdArticulationSource,
    UsdRobotInfo,
    component_patterns,
    prim_matches,
    read_articulation,
    split_scale,
    split_scale_batch,
)
from .writer import (
    GeometricFabricWriter,
    GeometricGpuFabricWriter,
    GeometricTransformWriter,
    GeometricUsdWriter,
    LocalPoseSolver,
    WriteTarget,
)

__all__ = [
    "LAYER_MARKER",
    "XFORM_OP_NAME",
    "CollisionMode",
    "LayerTarget",
    "GeometricLayerOptions",
    "GeometricLayerReport",
    "GeometricLayer",
    "authoring",
    "is_authoring",
    "is_collision_enabled_ignoring_geometric_layer",
    "WriteTarget",
    "LocalPoseSolver",
    "GeometricTransformWriter",
    "GeometricUsdWriter",
    "GeometricFabricWriter",
    "GeometricGpuFabricWriter",
    "GeometricVisualStyle",
    "safe_prim_name",
    "build_robot_prims",
    "UsdArticulationSource",
    "UsdRobotInfo",
    "read_articulation",
    "split_scale",
    "split_scale_batch",
    "component_patterns",
    "prim_matches",
]
