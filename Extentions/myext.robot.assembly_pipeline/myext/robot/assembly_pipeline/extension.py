"""拡張機能としての入口。UI は持たない。Python API(myext.robot.assembly_pipeline)を使えるようにするだけ。"""

from __future__ import annotations

import omni.ext


class AssemblyPipelineExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._ext_id = ext_id

    def on_shutdown(self) -> None:
        pass
