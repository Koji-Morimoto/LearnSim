import os
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "time_lab.py")
code = open(SRC, encoding="utf-8").read().split("# ================================================================ E1")[0]
exec(compile(code, SRC, "exec"))
new_scene(30, 60)
emit("fig0", **sample())
tl.play(); updates(6, "J_play1")
tl.pause(); updates(2, "J_pause")
b = sample(); SM.step(steps=3); emit("J_step", before=b, after=sample())
tl.play(); updates(8, "J_play2")
phase("done"); fout.close(); simapp.close()
