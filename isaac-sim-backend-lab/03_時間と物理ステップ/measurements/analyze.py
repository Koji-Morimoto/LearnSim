import json
import sys
from collections import defaultdict

rows = [json.loads(l) for l in open(sys.argv[1], encoding="utf-8")]
only = sys.argv[2] if len(sys.argv) > 2 else None
by = defaultdict(list)
for r in rows:
    if r["kind"] == "upd":
        by[r["tag"]].append(r)
    elif only is None or only in json.dumps(r):
        if r["kind"] != "phase":
            print(json.dumps(r, ensure_ascii=False))


def fmt(r):
    ext = r["ext"]
    ext = f"{ext:.5f}" if isinstance(ext, float) else ext
    s = (f"{r['i']:3d} wall={r['wall_ms']:6.2f} dtau={r['d_tau']:.5f} N={r['N']} pre={r['Npre']} fr={r['frames']} "
         f"n={r['n']} tau={r['tau']:.5f} t={r['t']:.5f} lag={r['lag']:+.5f} ext={ext} mono={r['mono']:.4f} "
         f"play={int(r['playing'])} sim={int(r['sm_sim'])} pau={int(r['sm_paused'])}")
    for k in ("z_physx", "z_usd", "z_fabric", "manual_step", "manual_mode"):
        if k in r:
            v = r[k]
            s += f" {k}={v:.5f}" if isinstance(v, float) else f" {k}={v}"
    return s


for tag, rs in by.items():
    if only and only not in tag:
        continue
    h = rs[0]["h"]
    lags = [r["lag"] for r in rs if r["playing"]]
    Ns = "".join(str(r["N"]) for r in rs)
    print(f"\n=== {tag}  h={h:.5f}  N-seq={Ns}")
    if lags:
        print(f"    lag min={min(lags):+.6f} max={max(lags):+.6f}  in[0,h)={all(-1e-6 <= x < h - 1e-6 for x in lags)}")
    walls = [r["wall_ms"] for r in rs[5:]]
    if walls:
        print(f"    wall mean={sum(walls)/len(walls):.2f}ms  min={min(walls):.2f} max={max(walls):.2f}")
    lim = 12 if len(rs) > 24 else len(rs)
    for r in rs[:lim]:
        print("   ", fmt(r))
    if len(rs) > lim:
        print("    ...")
        for r in rs[-4:]:
            print("   ", fmt(r))
