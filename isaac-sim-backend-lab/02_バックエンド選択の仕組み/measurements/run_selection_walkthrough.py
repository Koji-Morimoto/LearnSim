# run_selection_walkthrough.py - use_backend の指定・候補リスト・状態の組み合わせごとに、get_current_backend が返した値と最終的に使われたバックエンドを記録する
# 使い方: python.bat run_selection_walkthrough.py <experience.kit> <out.jsonl> <lab_root>
import os
import sys
sys.argv = sys.argv[:4]
exec(open(os.path.join(sys.argv[3], "05_観測Snippet", "measurements", "lab_common.py"), encoding="utf-8").read())
import isaacsim.core.experimental.utils.backend as backend_utils
from isaacsim.core.experimental.prims import RigidPrim, XformPrim

calls = []
original = backend_utils.get_current_backend
def recording(supported_backends, **kwargs):
    result = original(supported_backends, **kwargs)
    calls.append({"候補リスト": list(supported_backends), "返した値": result})
    return result
backend_utils.get_current_backend = recording

stage = new_stage(); add_cube(stage, "/World/Cube")
rigid = RigidPrim("/World/Cube"); xform = XformPrim("/World/Cube")

def case(label, wrapper, backend=None):
    calls.clear()
    carb.log_warn(f"===MARK {label}===")
    if backend is None:
        wrapper.get_world_poses()
    else:
        with backend_utils.use_backend(backend):
            wrapper.get_world_poses()
    emit("case", label=label, calls=list(calls))
    print("CASE", label, calls, flush=True)

case("A Stop・指定なし・RigidPrim", rigid)
case("C Stop・usdrt を指定・RigidPrim", rigid, "usdrt")
case("D Stop・tensor を指定・RigidPrim", rigid, "tensor")
set_state("Play", settle_updates=5)
rigid = RigidPrim("/World/Cube"); xform = XformPrim("/World/Cube")
case("B Play・指定なし・RigidPrim", rigid)
case("E Play・指定なし・XformPrim", xform)
case("F Play・tensor を指定・XformPrim", xform, "tensor")
carb.log_warn("===MARK end===")
set_state("Stop")
close()
