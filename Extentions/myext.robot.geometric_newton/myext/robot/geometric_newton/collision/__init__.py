"""干渉チェックおよび Newton 1.6.1 エンジン層 (Layer 2: `model/` と `usd/` に依存)。"""

from .newton_batched import BatchedNewtonScene
from .newton_scene import GeometricScene
from .newton_shapes import CollisionBackend, FkBackend, resolve_newton_device
from .physx import GeometricCollisionChecker, GeometricContactMonitor
from .types import (
    CollisionScope,
    GeometricCollisionPair,
    GeometricLinkSet,
    PairFilter,
    PairKind,
    _Owners,
    find_colliders,
)

__all__ = [
    "CollisionScope",
    "PairKind",
    "PairFilter",
    "GeometricCollisionPair",
    "GeometricLinkSet",
    "find_colliders",
    "GeometricCollisionChecker",
    "GeometricContactMonitor",
    "CollisionBackend",
    "FkBackend",
    "resolve_newton_device",
    "GeometricScene",
    "BatchedNewtonScene",
]
