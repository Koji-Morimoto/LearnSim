"""パイプラインのテスト。headless の Isaac Sim を 1 つ起動し、場面を順に流して判定を確かめる。

    C:\\isaacsim-6.2.0\\python.bat tests_kit\\run_tests.py [--only=名前の一部]

結果: tests_kit/last_run.json、終了コード(失敗があれば 1、テストまでたどり着けなければ 2)
"""

from __future__ import annotations

import dataclasses
import glob
import hashlib
import json
import os
import shutil
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EX = os.path.join(ROOT, "examples")
OUT = os.path.join(HERE, "out")

timer = threading.Timer(3600, lambda: (print("[WATCHDOG]", flush=True), os._exit(3)))
timer.daemon = True
timer.start()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
try:
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    mgr.add_path(os.path.dirname(ROOT))
    mgr.set_extension_enabled_immediate("myext.robot.assembly_pipeline", True)
    from myext.robot.assembly_pipeline import (
        JoinFileSpec,
        JoinSpec,
        PipelineSpec,
        PipelineStopped,
        RuleSetting,
        UrdfSource,
        load_spec,
        load_urdf,
        run_pipeline,
    )
except Exception:  # noqa: BLE001
    traceback.print_exc()
    print("[SUMMARY] passed 0 / failed 1(読み込みで失敗)", flush=True)
    os._exit(2)

results: list[dict] = []
only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--only=")]


def test(fn):
    if only and not any(o in fn.__name__ for o in only):
        return fn
    t0 = time.perf_counter()
    try:
        fn()
        results.append({"name": fn.__name__, "ok": True, "s": round(time.perf_counter() - t0, 1)})
        print(f"[PASS] {fn.__name__} ({time.perf_counter() - t0:.0f}s)", flush=True)
    except Exception:  # noqa: BLE001
        results.append({"name": fn.__name__, "ok": False, "traceback": traceback.format_exc()})
        print(f"[FAIL] {fn.__name__}\n{traceback.format_exc()}", flush=True)
    return fn


def urdf(name: str) -> str:
    return os.path.join(EX, "urdf", name)


def one_join(out: str, child_urdf: str = "gripper.urdf", child_link: str = "gripper_base", parent_prim: str | None = None,
             child_kw: dict | None = None, **kw) -> PipelineSpec:
    """腕(arm)の flange に子(gripper)を 1 回結合する結合ファイル arm_gripper。"""
    return PipelineSpec(
        output_dir=os.path.join(OUT, out),
        urdfs=(UrdfSource("arm", urdf("arm.urdf")), UrdfSource("gripper", urdf(child_urdf), **(child_kw or {}))),
        join_files=(JoinFileSpec("arm_gripper", "arm", (JoinSpec("gripper", "gripper", "flange", child_link, parent_prim=parent_prim),)),),
        **kw)


def verdicts(rep: dict) -> dict[tuple[str, str], str]:
    return {(o["rule"], o["target"]): o["verdict"] for o in rep["outcomes"]}


def expect(v: dict, key: tuple[str, str], want: str | set[str], rep: dict) -> None:
    got = v.get(key)
    wants = {want} if isinstance(want, str) else want
    if got not in wants:
        msg = next((o["message"] for o in rep["outcomes"] if (o["rule"], o["target"]) == key), "(無い)")
        raise AssertionError(f"{key}: 期待 {wants}、実際 {got}\n  {msg}")


def problems(rep: dict) -> list[str]:
    return [o["message"] for o in rep["outcomes"] if o["verdict"] in ("still_broken", "unexpected")]


def sha(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


J = "arm_gripper/gripper"


# ====================================================================== 純粋な部分
@test
def test_urdf_model_ghost_and_fk():
    import numpy as np

    m = load_urdf(urdf("arm.urdf"))
    assert not m.links["base_link"].ghost and m.links["flange"].ghost
    assert abs(m.fk_zero("flange")[2, 3] - 0.9) < 1e-9
    f = m.fk("link2", {"j1": np.pi / 2})
    assert np.allclose(f[:3, 3], [0, 0, 0.5]) and np.allclose(f[:3, 0], [0, 1, 0], atol=1e-9)
    g = load_urdf(urdf("gripper_ghost.urdf"))
    assert g.links[g.root_link].ghost and g.root_link == "gripper_mount"
    assert g.fixed_descendants("gripper_mount") == ["gripper_base"]


@test
def test_spec_validation():
    try:
        PipelineSpec(output_dir=OUT, urdfs=(UrdfSource("a", urdf("arm.urdf")),),
                     join_files=(JoinFileSpec("f", "a", (JoinSpec("x", "missing", "flange", "b"),)),))
    except ValueError as exc:
        assert "missing" in str(exc)
    else:
        raise AssertionError("存在しない子を通した")
    try:
        PipelineSpec(output_dir=OUT, urdfs=(UrdfSource("a", urdf("arm.urdf")),),
                     join_files=(JoinFileSpec("f", "g", (JoinSpec("x", "a", "flange", "base_link"),)), JoinFileSpec("g", "f", ())))
    except ValueError as exc:
        assert "循環" in str(exc)
    else:
        raise AssertionError("循環を通した")


# ====================================================================== 1 回の結合:リンクの子に置く(既定)
@test
def test_join_under_link():
    shutil.rmtree(os.path.join(OUT, "under_link"), ignore_errors=True)
    rep = run_pipeline(one_join("under_link")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    for key, want in {("R06", "arm"): "fixed", ("R06", "gripper"): "fixed", ("R07", J): "fixed", ("R09", J): "fixed",
                      ("R14", J): "fixed", ("R10", "arm"): "upstream_fixed", ("R02", J): "ok", ("R12", J): "ok", ("R08", J): "ok"}.items():
        expect(v, key, want, rep)
    g = rep["geometric"]["arm_gripper"]
    assert g["dof_names_match"] and g["max_link_position_error_m"] < 1e-4, g
    assert set(g["components_in_model"]) == {"arm_gripper", "gripper"}, g


@test
def test_idempotent_rerun():
    out = os.path.join(OUT, "under_link")
    if not os.path.exists(os.path.join(out, "arm_gripper.usda")):
        run_pipeline(one_join("under_link"))
    files = {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}
    logs: list[str] = []
    rep = run_pipeline(one_join("under_link"), log=logs.append).report
    assert sum(s.startswith("[import]") and "入力が同じなので飛ばした" in s for s in logs) == 2, logs
    assert any("1 回目 変化なし、2 回目 変化なし" in s for s in logs), logs
    assert files == {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}, "USD ファイルが変わった"
    assert not glob.glob(os.path.join(out, "urdf", "*_1")), "別名のフォルダができた"
    assert rep["status"] == "ok" and os.path.exists(os.path.join(out, "report_previous.json"))


# ====================================================================== 1 回の結合:一番上の prim の子に置く
@test
def test_join_under_top():
    rep = run_pipeline(one_join("under_top", parent_prim=".")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R02", J), "fixed", rep)  # Robot Assembler では親の関節が動かない
    expect(v, ("R12", J), "fixed", rep)  # Robot Assembler では teleport で置いて行かれる
    expect(v, ("R09", J), "ok", rep)


@test
def test_skip_shows_still_broken():
    rules = {r: RuleSetting(fix="skip") for r in ("R02", "R07", "R12")}
    rep = run_pipeline(one_join("under_top_skip", parent_prim=".", rules=rules)).report
    v = verdicts(rep)
    assert rep["status"] == "problems"
    for r in ("R02", "R07", "R12"):
        expect(v, (r, J), "still_broken", rep)
    msg = next(o["message"] for o in rep["outcomes"] if o["rule"] == "R02")
    assert "修正を skip した出力でも" in msg, msg


@test
def test_stop_before_join():
    s = one_join("stop", parent_prim=".", rules={"R02": RuleSetting(on_violation="stop")})
    try:
        run_pipeline(s)
    except PipelineStopped as exc:
        assert exc.report["status"] == "stopped_before_join"
        assert any(x.startswith(f"[R02] {J}") for x in exc.report["stopped_by"]), exc.report["stopped_by"]
        assert not os.path.exists(os.path.join(OUT, "stop", "arm_gripper.usda")), "止めたのに結合ファイルができている"
        return
    raise AssertionError("止まらなかった")


# ====================================================================== 空のルートリンクの子(子のリンクも空のリンク)
@test
def test_ghost_root_child():
    rep = run_pipeline(one_join("ghost", child_urdf="gripper_ghost.urdf", child_link="gripper_mount", parent_prim=".")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R08", J), "fixed", rep)  # Robot Assembler は ValueError
    expect(v, ("R01", "gripper"), "fixed", rep)  # ワールド固定 joint の名前が mount_joint
    assert rep["geometric"]["arm_gripper"]["max_link_position_error_m"] < 1e-4


# ====================================================================== mimic follower の drive
@test
def test_mimic_damping():
    # URDF の <dynamics damping> は newton:damping に入るだけで PhysX の drive には入らない → 判定も症状も無し
    rep = run_pipeline(one_join("damp_urdf", child_urdf="gripper_damp.urdf")).report
    expect(verdicts(rep), ("R03", "gripper:finger_r_joint"), "ok", rep)
    assert rep["measurements"]["statics"]["gripper"]["raw_follower_drive"]["finger_r_joint"] == {"stiffness": 0.0, "damping": 0.0}
    # importer の一括 damping 指定は follower にも入る → 判定あり、公式のままで遅れる、修正後は無し
    rep = run_pipeline(one_join("damp_override", child_kw={"joint_drive_damping": 5.0})).report
    expect(verdicts(rep), ("R03", "gripper:finger_r_joint"), "fixed", rep)


# ====================================================================== 4 本アーム(結合ファイルを 2 段、VariantSet、選択)
@test
def test_multi_arm():
    s = dataclasses.replace(load_spec(os.path.join(EX, "multi_arm.toml")), output_dir=os.path.join(OUT, "multi_arm"))
    shutil.rmtree(s.output_dir, ignore_errors=True)
    rep = run_pipeline(s).report
    assert rep["status"] == "ok", problems(rep)
    v = verdicts(rep)
    for jid in ("arm_unit/gripper", "arm_unit/rod", "robot/arm1", "robot/arm2", "robot/arm3", "robot/arm4"):
        expect(v, ("R09", jid), "fixed", rep)
        expect(v, ("R07", jid), "fixed", rep)
    fx = rep["measurements"]["fixed"]["robot/arm1"]
    assert fx["articulation_roots"] == ["/robot/Geometry/base_body"], fx["articulation_roots"]
    assert [a["path"] for a in fx["robot_anchors"]] == ["/robot/Physics/root_joint"]
    assert len(fx["dof_names"]) == 13, fx["dof_names"]  # 台座 1 + アーム 4×2 + 二本指 2 個×2(arm2 は rod、arm4 は None)
    assert all(r["error"] is not None and r["error"] < 1e-3 for r in fx["reach"].values()), fx["reach"]
    g = rep["geometric"]["robot"]
    assert set(g["components_in_model"]) == {"robot", "arm1", "arm2", "arm3", "arm4", "arm1/end_effector", "arm2/end_effector",
                                             "arm3/end_effector"}, g
    assert g["max_link_position_error_m"] < 1e-4
    from pxr import Usd

    unit = Usd.Stage.Open(os.path.join(s.output_dir, "arm_unit.usda"))  # 中間の結合ファイルは単体で開ける
    vs = unit.GetDefaultPrim().GetVariantSet("EndEffector")
    assert vs.GetVariantNames() == ["None", "gripper", "rod"] and vs.GetVariantSelection() == "gripper", vs.GetVariantNames()
    robot = Usd.Stage.Open(os.path.join(s.output_dir, "robot.usda"))
    sel = {p.GetName(): p.GetVariantSet("EndEffector").GetVariantSelection() for p in robot.Traverse()
           if p.GetName() in ("arm1", "arm2", "arm3", "arm4")}
    assert sel == {"arm1": "gripper", "arm2": "rod", "arm3": "gripper", "arm4": "None"}, sel


@test
def test_multi_arm_rerun():
    out = os.path.join(OUT, "multi_arm")
    s = dataclasses.replace(load_spec(os.path.join(EX, "multi_arm.toml")), output_dir=out)
    if not os.path.exists(os.path.join(out, "robot.usda")):
        run_pipeline(s)
    files = {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}
    rep = run_pipeline(s, log=lambda *_: None).report
    assert files == {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}, "USD ファイルが変わった"
    assert rep["status"] == "ok"


# ====================================================================== 実物のロボット(Isaac Sim 同梱の Franka の腕と手)
@test
def test_franka_example():
    s = dataclasses.replace(load_spec(os.path.join(EX, "franka", "franka.toml")), output_dir=os.path.join(OUT, "franka"))
    rep = run_pipeline(s).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R09", "franka/hand"), "fixed", rep)
    expect(v, ("R04", "hand"), "ok", rep)  # 公式のまま側は offset を除いて比べるので、Assembler で 45° ずれても数えない
    al = rep["measurements"]["fixed"]["franka/hand"]["joins"]["franka/hand"]["urdf_mount_alignment"]
    assert al[0] < 1e-6 and al[1] < 1e-3, al  # offset(z 軸まわり -45°)込みで結合面が一致
    g = rep["geometric"]["franka"]
    assert g["max_link_position_error_m"] < 1e-4 and set(g["components_in_model"]) == {"franka", "hand"}, g


# ====================================================================== TOML の例
@test
def test_example_toml():
    s = load_spec(os.path.join(EX, "arm_gripper.toml"))
    assert s.urdfs[0].path.endswith(os.path.join("examples", "urdf", "arm.urdf"))
    assert s.join_files[0].joins[0].parent_link == "flange"
    m = load_spec(os.path.join(EX, "multi_arm.toml"))
    assert [f.name for f in m.file_order()] == ["arm_unit", "robot"]
    assert m.join_file("robot").join("arm2").select == (("EndEffector", "rod"),)


summary = {"passed": sum(r["ok"] for r in results), "failed": sum(not r["ok"] for r in results), "results": results}
with open(os.path.join(HERE, "last_run.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=1)
print(f"[SUMMARY] passed {summary['passed']} / failed {summary['failed']}", flush=True)
sys.stdout.flush()
os._exit(1 if summary["failed"] else 0)
