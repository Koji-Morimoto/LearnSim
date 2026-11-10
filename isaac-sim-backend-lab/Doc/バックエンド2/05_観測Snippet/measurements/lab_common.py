# lab_common.py - 05_観測Snippet の実測で共通に使う部分（ヘッドレス起動・シーン作成・出力の記録）
# 使い方: python.bat <run_*.py> <experience.kit> <out.jsonl> <lab_root>
# run_*.py の先頭で exec される。SimulationApp をここで起動する。
import contextlib
import io
import json
import os
import sys
import time

EXPERIENCE = sys.argv[1]
OUT_PATH = sys.argv[2]
LAB_ROOT = sys.argv[3]

from isaacsim import SimulationApp

simapp = SimulationApp({"headless": True}, experience=EXPERIENCE)

import carb
import carb.settings
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from omni.kit.loop import _loop as kit_loop
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

settings = carb.settings.get_settings()
kit_app = omni.kit.app.get_app()
timeline = omni.timeline.get_timeline_interface()
loop_runner = kit_loop.acquire_loop_interface()
_out_file = open(OUT_PATH, "w", encoding="utf-8")


def emit(kind, **fields):
    """1件の記録を JSON の1行として書く。"""
    fields["kind"] = kind
    _out_file.write(json.dumps(fields, ensure_ascii=False, default=str) + "\n")
    _out_file.flush()


def phase(name):
    """実験の区切りを記録し、端末にも表示する。"""
    emit("phase", name=name)
    print("PHASE", name, flush=True)


def run_updates(update_count):
    """メインループの更新を update_count 回行い、1回ごとの実時間 [s] のリストを返す。"""
    durations = []
    for _ in range(update_count):
        start = time.perf_counter()
        simapp.update()
        durations.append(time.perf_counter() - start)
    return durations


@contextlib.contextmanager
def captured_stdout(label):
    """with の中で Python の print が出した文字列を集め、終わったら記録する。"""
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        yield buffer
    emit("stdout", label=label, text=buffer.getvalue())


def state_name():
    """現在の timeline の状態を Stop / Play / Pause の文字列で返す。"""
    if timeline.is_playing():
        return "Play"
    if timeline.is_stopped():
        return "Stop"
    return "Pause"


def set_state(target_state, settle_updates=3):
    """timeline を target_state（Stop / Play / Pause）にし、settle_updates 回更新して反映させる。"""
    if target_state == "Play":
        timeline.play()
    elif target_state == "Pause":
        timeline.pause()
    else:
        timeline.stop()
    run_updates(settle_updates)


def new_stage(steps_per_second=60, time_codes_per_second=60, gpu=False):
    """新しいステージを作り、Physics Scene を置いて、そのステージを返す。"""
    omni.usd.get_context().new_stage()
    run_updates(3)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(time_codes_per_second)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(1e6)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr(int(steps_per_second))
    physx_scene.CreateEnableGPUDynamicsAttr(bool(gpu))
    physx_scene.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    timeline.set_time_codes_per_second(time_codes_per_second)
    timeline.set_start_time(0)
    timeline.set_end_time(1e6)
    run_updates(2)
    return stage


def add_cube(stage, path="/World/Cube", position=(0.0, 0.0, 1.0), size=0.1, rigid=True):
    """立方体を置く。rigid が True なら剛体とコライダーを付ける。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*position))
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
    return cube


def add_articulation(stage, root_path="/World/robot", base_position=(2.0, 0.0, 1.0)):
    """土台を世界に固定し、1つの回転関節で棒をつないだ Articulation を置く。"""
    UsdGeom.Xform.Define(stage, root_path)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root_path))
    base = add_cube(stage, root_path + "/base", position=base_position, size=0.1)
    link_position = (base_position[0], base_position[1], base_position[2] + 0.3)
    link = add_cube(stage, root_path + "/link", position=link_position, size=0.1)
    fixed = UsdPhysics.FixedJoint.Define(stage, root_path + "/fixed_joint")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*base_position))
    revolute = UsdPhysics.RevoluteJoint.Define(stage, root_path + "/revolute_joint")
    revolute.CreateBody0Rel().SetTargets([base.GetPath()])
    revolute.CreateBody1Rel().SetTargets([link.GetPath()])
    revolute.CreateAxisAttr("Y")
    revolute.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0.15))
    revolute.CreateLocalPos1Attr(Gf.Vec3f(0, 0, -0.15))
    drive = UsdPhysics.DriveAPI.Apply(revolute.GetPrim(), "angular")
    drive.CreateStiffnessAttr(1000.0)
    drive.CreateDampingAttr(100.0)
    return root_path


def snippet_source(snippet_name):
    """snippets/<snippet_name>.py の中身を返す。"""
    path = os.path.join(LAB_ROOT, "snippets", snippet_name + ".py")
    with open(path, encoding="utf-8") as source_file:
        return path, source_file.read()


def run_snippet(snippet_name, replacements=None, namespace=None):
    """Snippet を Script Editor と同じように実行する（モジュールの最上位として exec する）。"""
    path, source = snippet_source(snippet_name)
    for old_text, new_text in (replacements or {}).items():
        if old_text not in source:
            raise RuntimeError(f"{snippet_name}: replacement target not found: {old_text[:60]}")
        source = source.replace(old_text, new_text)
    namespace = namespace if namespace is not None else {"__name__": "__main__"}
    exec(compile(source, path, "exec"), namespace)
    return namespace


def close():
    """記録を閉じ、アプリを終了する。"""
    phase("done")
    _out_file.close()
    simapp.close()
