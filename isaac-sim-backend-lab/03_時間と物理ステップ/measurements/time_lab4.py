import os
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "time_lab.py")
code = open(SRC, encoding="utf-8").read().split("# ================================================================ E1")[0]
exec(compile(code, SRC, "exec"))
new_scene(60, 60)
updates(60, "I_stop")
tl.play(); updates(120, "I_play")
tl.pause(); updates(60, "I_pause")
tl.stop(); updates(60, "I_stop2")
phase("done"); fout.close(); simapp.close()
