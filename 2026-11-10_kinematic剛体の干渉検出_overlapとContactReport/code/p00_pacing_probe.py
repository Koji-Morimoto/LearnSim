"""P00: headless で Play 中の update が 16.7 ms に揃う原因の設定を探す(P01 の測定条件を決めるための下調べ)。"""

from __future__ import annotations

import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, r"C:\isaacsim-6.2.0\CodeByClaude\backend_pipeline_verification\code")
import bplib

bplib.RESULT_DIR = os.path.join(os.path.dirname(HERE), "results")
app = bplib.start(render=False)

import carb.settings

rec = bplib.Recorder("p00_pacing_probe")
s = carb.settings.get_settings()


def play_idle_ms() -> float:
    bplib.new_stage()
    bplib.update(2)
    bplib.play(5)
    d = []
    for _ in range(40):
        t0 = time.perf_counter()
        bplib.update(1)
        d.append(time.perf_counter() - t0)
    bplib.stop(2)
    return round(statistics.median(d) * 1e3, 3)


KEYS = [
    "/app/runLoops/main/rateLimitEnabled",
    "/app/runLoops/main/rateLimitFrequency",
    "/app/runLoops/main/manualModeEnabled",
    "/app/player/useFixedTimeStepping",
    "/app/player/playSimulations",
    "/persistent/simulation/minFrameRate",
]


@bplib.guarded(rec, "P-00")
def run() -> None:
    out = {"initial_settings": {k: s.get(k) for k in KEYS}, "0_default": play_idle_ms()}
    s.set("/app/runLoops/main/rateLimitEnabled", False)
    out["1_rateLimit_off"] = play_idle_ms()
    s.set("/app/runLoops/main/manualModeEnabled", True)
    out["2_plus_manualMode_on"] = play_idle_ms()
    s.set("/app/runLoops/main/manualModeEnabled", False)
    bplib.timeline().set_play_every_frame(True)
    out["3_rateLimit_off_plus_play_every_frame"] = play_idle_ms()
    bplib.timeline().set_play_every_frame(False)
    s.set("/app/player/useFixedTimeStepping", False)
    out["4_rateLimit_off_plus_fixedTimeStepping_off"] = play_idle_ms()
    rec.add("P-00", "Play 中、空のシーンの update 時間の中央値 [ms]", **out)


run()
bplib.finish(rec)
