"""results/*.json の snapshot を 1 行 1 段階の短い表にして出す(Isaac Sim 不要。普通の Python で動く)。

使い方:
    python summarize.py m01_play_matrix              # 既定構成
    python summarize.py m01_play_matrix __gpu        # 接尾辞付きの構成
    python summarize.py m01_play_matrix "" __physx_fabric __gpu   # 複数構成を並べる
    python summarize.py q01_pause __gpu --filter set_velocities

1 行の書式:
    <段階名>  U=<USD 位置>/<USD 速度y>/<USD 質量>  F=<Fabric 位置>/<速度y>/<質量>  P=<PhysX 位置>  T=<tensor 位置>/<速度y>/<質量>
    位置は (x,y,z)、- は値なし(属性なし・Stop 中で PhysX/tensor が無いなど)
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any

RESULTS: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")


def _v(x: Any) -> str:
    if x is None:
        return "-"
    if isinstance(x, list):
        return "(" + ",".join(f"{c:g}" for c in x) + ")"
    return f"{x:g}" if isinstance(x, (int, float)) else str(x)


def _vy(x: Any) -> str:
    """速度ベクトルは y 成分だけ見れば足りる(書く値が (0, 0.6, 0) なので)。"""
    return f"{x[1]:g}" if isinstance(x, list) else "-"


def fmt_snapshot(s: dict[str, Any]) -> str:
    u, f, p, t = s["usd"], s["fabric"], s["physx"], s["tensor"]
    us = f"U={_v(u.get('translate'))}/{_vy(u.get('velocity'))}/{_v(u.get('mass'))}"
    fs = f"F={_v(f.get('local'))}/{_vy(f.get('velocity'))}/{_v(f.get('mass'))}" if "error" not in f else "F=err"
    ps = f"P={_v(p.get('position'))}"
    ts = f"T={_v(t.get('position'))}/{_vy(t.get('velocity'))}/{_v(t.get('mass'))}" if t.get("valid") else "T=-"
    return f"{us:28s} {fs:28s} {ps:18s} {ts}"


def is_snapshot(v: Any) -> bool:
    return isinstance(v, dict) and "physx" in v and "usd" in v


def show(group: str, suffix: str, flt: str | None) -> None:
    path = os.path.join(RESULTS, f"{group}{suffix}.json")
    with open(path, encoding="utf-8") as fp:
        data = json.load(fp)
    print(f"########## {group}{suffix}")
    for item in data["items"]:
        if flt and flt not in item["id"]:
            continue
        print(f"== {item['id']}  {item.get('description', '')}")
        for k, v in item.items():
            if k in ("id", "description"):
                continue
            if is_snapshot(v):
                print(f"   {k:26s} {fmt_snapshot(v)}")
            elif isinstance(v, dict) and "touched" in v:
                warn = "; ".join(w[:90] for w in v.get("warnings", []))
                print(f"   {k:26s} touched={v['touched']} error={v.get('error')} warn={warn or '-'}")
            elif "readback" in k:
                print(f"   {k:26s} {json.dumps(v, ensure_ascii=False)}")


def main(argv: list[str]) -> None:
    flt = None
    if "--filter" in argv:
        i = argv.index("--filter")
        flt = argv[i + 1]
        argv = argv[:i] + argv[i + 2 :]
    group, suffixes = argv[0], argv[1:] or [""]
    for sfx in suffixes:
        show(group, sfx, flt)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main(sys.argv[1:])
