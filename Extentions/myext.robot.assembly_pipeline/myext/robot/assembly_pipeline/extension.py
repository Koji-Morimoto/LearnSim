"""Isaac Sim / Omniverse Kit 拡張機能としてのライフサイクル入口。

責務:
  - Kit の拡張機能マネージャが `myext.robot.assembly_pipeline` を有効化/無効化するための
    最小限の `omni.ext.IExt` 実装を提供する。
  - UI ウィンドウや常駐イベント購読は持たず、Python API(`myext.robot.assembly_pipeline`)を
    Kit プロセス内から import できる状態にする役割に専念する。

主な入出力・公開 API:
  - `Extension`: `omni.ext.IExt` を実装した拡張クラス。
  - `AssemblyPipelineExtension`: 互換用の別名。

呼ばれ元:
  - Omniverse Kit の拡張機能ローダ（`config/extension.toml` の `[[python.module]]` 経由）。
"""

from __future__ import annotations

import omni.ext


class Extension(omni.ext.IExt):
    """Kit に登録される拡張本体。起動時に拡張 ID のみを保持し、終了時は状態を持たないため何もしない。"""

    def on_startup(self, ext_id: str) -> None:
        """拡張機能の起動フック。

        引数:
          ext_id: Kit が割り当てた拡張機能識別子（例: `myext.robot.assembly_pipeline-0.2.0`）。
        """
        self._ext_id = ext_id

    def on_shutdown(self) -> None:
        """拡張機能の終了フック。保持リソースが無いため処理なし。"""
        pass


AssemblyPipelineExtension = Extension

__all__ = ["Extension", "AssemblyPipelineExtension"]
