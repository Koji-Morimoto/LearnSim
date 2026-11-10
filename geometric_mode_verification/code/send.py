"""起動中の Isaac Sim GUI (isaacsim.code_editor.python_server, port 8226) にコードを送って実行する。

使い方: python send.py <file.py> [--timeout 秒]   /   python send.py -c "print(1)"
結果 JSON の output / traceback を表示する。--save <path> で JSON をファイルにも保存する。
"""

import argparse
import json
import socket
import sys

p = argparse.ArgumentParser()
p.add_argument("file", nargs="?")
p.add_argument("-c", "--code")
p.add_argument("--timeout", type=float, default=600)
p.add_argument("--save")
a = p.parse_args()
code = a.code if a.code is not None else open(a.file, encoding="utf-8").read()
env = {"code": code, "timeout": a.timeout}
s = socket.create_connection(("127.0.0.1", 8226), timeout=a.timeout + 30)
s.sendall(json.dumps(env).encode("utf-8"))
s.shutdown(socket.SHUT_WR)
buf = b""
while True:
    chunk = s.recv(65536)
    if not chunk:
        break
    buf += chunk
r = json.loads(buf.decode("utf-8"))
if a.save:
    with open(a.save, "w", encoding="utf-8") as f:
        json.dump(r, f, ensure_ascii=False, indent=1)
sys.stdout.reconfigure(encoding="utf-8")
print("status:", r.get("status"))
print(r.get("output", ""))
if r.get("result") is not None:
    print("result:", r["result"])
if r.get("traceback"):
    print("".join(r["traceback"]))
