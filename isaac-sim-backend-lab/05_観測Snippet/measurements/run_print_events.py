# run_print_events.py - Script Editor と同じ実行経路で、print 1回あたりに出る標準出力のイベント数と費用を数える
import builtins
import os
import sys

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lab_common.py"), encoding="utf-8").read())

import carb.eventdispatcher

event_texts = []
subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=omni.kit.app.GLOBAL_EVENT_SCRIPT_STDOUT_IMMEDIATE,
    on_event=lambda event: event_texts.append(event.get("text")),
    observer_name="lab print event counter",
)
cases = {
    "print(f'...') 引数1つ": "print(f'timeline.is_playing: {True}')",
    "print('...', 値) 引数2つ": "print('timeline.is_playing:', True)",
    "print(100行をjoinした文字列)": "print(chr(10).join(f'line {i}' for i in range(100)))",
}
for label, statement in cases.items():
    event_texts.clear()
    kit_app.get_python_scripting().execute_string(statement, "lab-print-events", False)
    run_updates(1)
    emit("print_events", label=label, event_count=len(event_texts), texts=[text[:40] for text in event_texts][:6])
timing_code = (
    "import builtins, time, statistics\n"
    "def time_us(action, repeat_count=200):\n"
    "    samples = []\n"
    "    for _ in range(repeat_count):\n"
    "        start = time.perf_counter(); action(); samples.append(time.perf_counter() - start)\n"
    "    return statistics.median(samples) * 1e6\n"
    "builtins._lab_print_cost = {\n"
    "    'one_arg': time_us(lambda: print(f'timeline.is_playing: {True}')),\n"
    "    'two_args': time_us(lambda: print('timeline.is_playing:', True)),\n"
    "    'join_100_lines': time_us(lambda: print(chr(10).join(f'line {i}' for i in range(100))), 50),\n"
    "}\n"
)
kit_app.get_python_scripting().execute_string(timing_code, "lab-print-cost", False)
run_updates(1)
emit("print_cost_us", **builtins._lab_print_cost)
close()
