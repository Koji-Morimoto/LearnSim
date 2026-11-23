"""Isaac Sim 拡張機能としてのエントリポイント (`omni.ext.IExt`)。

UI やバックグラウンド常駐処理は持たない。`config/extension.toml` でこの拡張を有効化すると、
Kit の Python パスに `myext.robot.geometric_newton` が公開され、他スクリプトや Script Editor から
Python API を import できるようになる。
"""

from __future__ import annotations

import omni.ext


class GeometricRobotNewtonExtension(omni.ext.IExt):
    """拡張機能のライフサイクルを受け取る最小クラス。"""

    def on_startup(self, ext_id: str) -> None:
        """拡張機能の起動時に Kit から呼ばれる。"""
        self._ext_id = ext_id

    def on_shutdown(self) -> None:
        """拡張機能の終了時に Kit から呼ばれる。"""
        pass
