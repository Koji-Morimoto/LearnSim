"""今開いているステージの幾何モードを切り替える。Isaac Sim の Script Editor に貼って実行するか、
python send.py gui_toggle.py で送る。

1 回目: 幾何モードにする(重力 0・全剛体 kinematic・接触レポート・kinematic 組の報告)
2 回目: 元の物理設定に戻す(幾何レイヤーを外すだけ。Root Layer は一度も書き換えていない)
"""

import importlib
import sys

CODE_DIR = r"C:\isaacsim-6.2.0\CodeByClaude\geometric_mode_verification\code"
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)
import geometric_mode as gm  # noqa: E402

importlib.reload(gm)
import omni.usd  # noqa: E402

stage = omni.usd.get_context().get_stage()
if gm.is_enabled(stage):
    gm.disable(stage)
    print("[geometric_mode] 物理モードに戻した")
else:
    report = gm.enable(stage)
    print(f"[geometric_mode] 幾何モードにした: 上書き {len(report.changes)} 件 (レイヤー {report.layer_identifier})")
    for c in report.changes:
        print(f"  {c.path}.{c.name}: {c.before} -> {c.after}")
    for k, v in report.unused.items():
        print(f"  効かなくなる設定 {k}: {len(v)} prim")
    for w in report.warnings:
        print("  注意:", w)
