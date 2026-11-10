# bench_probe.py — APIの呼び出し時間を計る共通関数と、計測方法そのものの基準値
# 解説: 05_観測Snippet/04_bench_probe.md
"""API の呼び出し時間を計る共通関数 measure と、計測方法そのものの基準値を提供する。

Isaac Sim の Script Editor で実行する。実行後は、別のタブから measure(...) を import なしで呼べる。

Note:
    呼べる状態: Stop／Play／Pause のどれでも実行できる。計る対象の API が呼べる状態は、対象ごとに異なる。
    backend: measure 自体は backend を選ばない。backend を指定して計る場合は、target の中で use_backend を使う。
"""

from __future__ import annotations

import builtins  # 別のScript Editorタブからも measure を呼べるように、組み込み名前空間を使う
import statistics  # 中央値を計算する標準ライブラリを読み込む
import time  # 高分解能タイマー perf_counter を使うために読み込む
import typing

import warp as wp  # GPU処理の完了待ち（synchronize）に使うWarpを読み込む


def measure(
    label: str,
    target: typing.Callable[[], object],
    repeat_count: int = 100,
    warmup_count: int = 5,
    sync: bool = False,
) -> list[float]:
    """target を warmup_count 回呼んで捨ててから、repeat_count 回呼んで1回ごとの所要時間を計る。

    結果の中央値と最小値をマイクロ秒で1行に表示し、生データを返す。

    Args:
        label: 表示する計測の名前。
        target: 計る対象の処理。引数を取らない関数。戻り値は使わないので object とする。
        repeat_count: 計測する回数。1 以上。
        warmup_count: 計測の前に空打ちする回数。0 以上。初回の初期化コストを計測から外すために使う。
        sync: True なら、各回の終了時刻を取る前に warp.synchronize で GPU 処理の完了を待つ。

    Returns:
        1回ごとの所要時間 [s] のリスト。要素数は repeat_count。
    """
    for _ in range(warmup_count):  # 初回の初期化コストを計測から外すため、先に空打ちする
        target()  # 空打ちの結果は使わずに捨てる
    if sync:  # GPUを使う計測では
        wp.synchronize()  # 空打ちで発行したGPU処理が終わるまで待ってから計測を始める
    samples: list[float] = []  # 1回ごとの所要時間（秒）を貯めるリストを用意する
    for _ in range(repeat_count):  # repeat_count回くり返して計る
        start = time.perf_counter()  # 開始時刻を取る
        target()  # 計測対象を1回呼ぶ
        if sync:  # GPU処理の完了まで含めて計る場合は
            wp.synchronize()  # GPU処理の完了を待ってから終了時刻を取る
        samples.append(time.perf_counter() - start)  # 経過時間をリストに記録する
    median_us = statistics.median(samples) * 1e6  # 中央値を秒からマイクロ秒に直す
    min_us = min(samples) * 1e6  # 最小値を秒からマイクロ秒に直す
    print(f"{label:<28} median={median_us:9.1f}us min={min_us:9.1f}us repeat={repeat_count} sync={sync}")  # 結果を1行で表示する
    return samples  # 分布を後で見られるように生データを返す


builtins.measure = measure  # 別のタブから measure(...) と呼べるように登録する
measure("baseline: 空の関数", lambda: None)  # 計測方法そのもののオーバーヘッドを基準値として出す
measure("baseline: 空の関数+sync", lambda: None, sync=True)  # synchronize自体のコストを基準値として出す
