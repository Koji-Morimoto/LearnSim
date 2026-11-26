"""幾何モードのロボット API (`myext.robot.geometric_newton` / Newton 1.6.1 統合版)。

物理ソルバー (`solver.step()`) や Joint Drive を使わず、関節値からリンクの変換行列を順運動学で計算して
USD / Fabric へ直接書き込む。`GeometricLayer` を付けると幾何モード、外すと元の Articulation 構成に戻るため、
元の USD ファイル (Root Layer) は一切書き換えない。

Newton バックエンド (`collision_backend="newton"`, 既定) では、PhysX の `PhysicsScene` に依存しない
自前コリジョン空間 (`GeometricScene`) を持ち、Fabric のみに姿勢を書き込む構成でも `Play=ON` / `Stop` の
両方で 0 フレーム遅延の干渉チェック・符号付き距離 (めり込み量 / クリアランス) 計算ができる。

    from myext.robot.geometric_newton import GeometricRobot, GeometricScene, MDHParameters, MDHRow

設計上の意図 (なぜ遅延 import にしているか):
    `model` サブパッケージは NumPy のみで完結しており、Isaac Sim / OpenUSD / Warp / Newton が
    入っていない通常の Python 環境でも単体テストやオフライン計画に利用できる。
    USD / Kit / Newton に依存するシンボル (`GeometricRobot`、`GeometricScene` 等) を `_LAZY` に閉じ込め、
    `__getattr__` で初回参照時のみ `importlib.import_module` することで、`import myext.robot.geometric_newton`
    の時点で `pxr` / `omni` / `warp` の読み込みを強制しない設計にしている。
"""

# ==============================================================================
# 公開シンボル (NumPy 完結層は即時公開、USD / Kit / Newton 層は遅延公開)
# ==============================================================================

from .model import *  # noqa: F401,F403
from .model import __all__ as _model_all

# シンボル名 -> サブモジュール名 (`__getattr__` で初回参照時に遅延 import する対象)
_LAZY = {
    "UsdArticulationSource": "usd.reader",
    "UsdRobotInfo": "usd.reader",
    "read_articulation": "usd.reader",
    "GeometricLayer": "usd.layer",
    "GeometricLayerOptions": "usd.layer",
    "GeometricLayerReport": "usd.layer",
    "CollisionMode": "usd.layer",
    "LayerTarget": "usd.layer",
    "GeometricTransformWriter": "usd.writer",
    "GeometricUsdWriter": "usd.writer",
    "GeometricFabricWriter": "usd.writer",
    "GeometricGpuFabricWriter": "usd.writer",
    "WriteTarget": "usd.writer",
    "GeometricRobot": "runtime.robot",
    "GeometricRobotComponent": "runtime.handles",
    "GeometricJoint": "runtime.handles",
    "GeometricLink": "runtime.handles",
    "GeometricLimitViolation": "runtime.handles",
    "GeometricDofBinder": "runtime.handles",
    "LimitPolicy": "runtime.handles",
    "GeometricStructureChangedError": "runtime.handles",
    "GeometricRobotView": "runtime.robot_view",
    "GeometricCollisionChecker": "collision.physx",
    "GeometricCollisionPair": "collision.types",
    "GeometricLinkSet": "collision.types",
    "GeometricContactMonitor": "collision.physx",
    "CollisionScope": "collision.types",
    "GeometricScene": "collision.newton_scene",
    "BatchedNewtonScene": "collision.newton_batched",
    "CollisionBackend": "collision.newton_shapes",
    "FkBackend": "collision.newton_shapes",
    "GeometricVisualStyle": "usd.builder",
    "build_robot_prims": "usd.builder",
}

__all__ = list(_model_all) + list(_LAZY)


def __getattr__(name: str):
    """`_LAZY` に登録された USD / Kit / Newton 依存シンボルを初回アクセス時に解決してキャッシュする。"""
    if name in _LAZY:
        import importlib

        module = importlib.import_module(f"{__name__}.{_LAZY[name]}")
        value = getattr(module, name)
        # 2 回目以降の属性アクセスで `__getattr__` を再呼び出ししないようモジュール辞書へキャッシュする
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
