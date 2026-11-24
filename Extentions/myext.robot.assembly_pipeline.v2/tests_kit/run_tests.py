r"""assembly_pipeline.v2 の統合テストスイート（全 18 テスト）。

テスト方針:
  - Headless の Isaac Sim を 1 プロセスだけ起動し、v2 パイプラインの全機能と
    14 個の検証規則（R01〜R14）の動作・冪等性・非破壊性・依存分離を順番に検証する。
  - 各テストは独立した出力ディレクトリ（`tests_kit/out/<テスト名>`）を使用し、
    既存アセットや他のテストの出力を汚染しない。
  - テスト対象のカテゴリ構成:
      1〜2   : 純 Python 層（URDF モデル解析・FK、PipelineSpec 静的検証・check_links）
      3〜9   : 単体結合と規則 R01〜R14 の挙動（配置先・冪等性・skip・stop・空ルート・mimic）
      10〜13 : 多段結合・VariantSet（4 本アーム）・実機 Franka・サンプル TOML 読み込み
      14〜16 : 非破壊ビルド（build_assembly）・Descriptor/discover_components・混在 Variant 順序・外部依存ゼロ確認
      17〜18 : 公式 USD+USD（UR10e + Robotiq）・混在 URDF+USD パイプラインと冪等性

実行方法:
  # 全テスト実行（推奨: PowerShell ラッパー経由）
  powershell -ExecutionPolicy Bypass -File run_tests.ps1

  # 直接実行（AGENTS.md 必須の UTF-8 コンソールラッパー経由）
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\run_tests.py

  # 特定テストだけ実行（関数名の部分一致、複数指定可）
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\run_tests.py --only=franka
  C:\isaacsim-6.2.0\tools\myext\run_sim.bat tests_kit\run_tests.py --only=ur10e --only=mixed

出力と終了コード:
  - 実行結果 JSON: `tests_kit/last_run.json`（passed / failed / 各テストの所要時間・エラー詳細）
  - 終了コード: 0 = 全件成功、1 = テスト失敗あり、2 = Isaac Sim 起動・拡張ロード失敗、3 = タイムアウト
  - 注意: `run_pipeline()` の公式比較フェーズ（`compare_official=True`）では、公式 RobotAssembler の
    不具合を意図的に再現するため PhysX の `closed articulation` 等のエラーログが出力される。
    合否は端末末尾の `[SUMMARY] passed N / failed 0` と終了コードで判断すること。

テストの追加・変更方法:
  1. `@test` デコレータを付けた引数なしの関数を定義する。
  2. 単体結合のバリエーションを試す場合は `one_join()` ヘルパで `PipelineSpec` を組み立てる。
  3. 規則の判定検証には `verdicts(rep)` と `expect(v, (規則ID, 対象), 期待判定, rep)` を使う。
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
import tomllib
import traceback
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EX = os.path.join(ROOT, "examples")
OUT = os.path.join(HERE, "out")

# ハング時の無限待機を防ぐウォッチドッグ（60 分で強制終了コード 3）
timer = threading.Timer(3600, lambda: (print("[WATCHDOG]", flush=True), os._exit(3)))
timer.daemon = True
timer.start()

# AGENTS.md 必須ルール: SimulationApp は sim_app_launcher.create_simulation_app のみ経由で起動する
sys.path.insert(0, str(Path("C:/isaacsim-6.2.0/tools/myext")))
from sim_app_launcher import create_simulation_app  # noqa: E402

app = create_simulation_app({"headless": True})
try:
    import omni.kit.app

    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    mgr = omni.kit.app.get_app().get_extension_manager()
    mgr.add_path(os.path.dirname(ROOT))
    mgr.set_extension_enabled_immediate("myext.robot.assembly_pipeline.v2", True)
    from myext.robot.assembly_pipeline_v2 import (
        AssembledRobotDescriptor,
        JoinFileSpec,
        JoinSpec,
        PipelineSpec,
        PipelineStopped,
        RuleSetting,
        UrdfSource,
        UsdSource,
        build_assembly,
        check_links,
        discover_components,
        load_spec,
        load_urdf,
        run_pipeline,
        spawn_assembled_robot,
    )
except Exception:  # noqa: BLE001
    traceback.print_exc()
    print("[SUMMARY] passed 0 / failed 1(読み込みで失敗)", flush=True)
    os._exit(2)

results: list[dict] = []
only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--only=")]


def test(fn):
    """テスト関数を即時実行し、合否・所要時間・トレースバックを `results` へ記録するデコレータ。

    `--only=<文字列>` が指定されている場合は、関数名に一致するテストだけを実行する。
    """
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
    """`examples/urdf/<name>` の絶対パスを返すヘルパ。"""
    return os.path.join(EX, "urdf", name)


def one_join(
    out: str,
    child_urdf: str = "gripper.urdf",
    child_link: str = "gripper_base",
    place: str | None = None,
    child_kw: dict | None = None,
    **kw,
) -> PipelineSpec:
    """単体結合テスト用の共通 `PipelineSpec` を組み立てるヘルパ。

    セットアップ:
      - 親: `examples/urdf/arm.urdf` -> `arm.usda`
      - 子: `examples/urdf/<child_urdf>` -> `two_finger.usda`（`child_kw` でインポート設定を上書き可能）
      - 結合: `arm.usda` の `flange` に `two_finger.usda` の `child_link` を名前 `"ee"` で結合し
        `arm_with_ee.usda` を出力する（`place` で `place_child_under` を切替可能）。
    """
    return PipelineSpec(
        output_dir=os.path.join(OUT, out),
        urdfs=(
            UrdfSource(urdf=urdf("arm.urdf"), usda="arm.usda"),
            UrdfSource(urdf=urdf(child_urdf), usda="two_finger.usda", **(child_kw or {})),
        ),
        join_files=(
            JoinFileSpec(
                usda="arm_with_ee.usda",
                parent_usda="arm.usda",
                joins=(
                    JoinSpec(
                        name="ee",
                        child_usda="two_finger.usda",
                        parent_link="flange",
                        child_link=child_link,
                        place_child_under=place,
                    ),
                ),
            ),
        ),
        **kw,
    )


def verdicts(rep: dict) -> dict[tuple[str, str], str]:
    """レポートの `outcomes` を `{(規則ID, 対象名): 判定文字列}` の辞書へ変換する。"""
    return {(o["rule"], o["target"]): o["verdict"] for o in rep["outcomes"]}


def expect(v: dict, key: tuple[str, str], want: str | set[str], rep: dict) -> None:
    """指定した `(規則ID, 対象名)` の判定が期待値（単一文字列または集合）と一致することを検証する。

    不一致時はレポート内の詳細メッセージ付きで `AssertionError` を送出する。
    """
    got = v.get(key)
    wants = {want} if isinstance(want, str) else want
    if got not in wants:
        msg = next((o["message"] for o in rep["outcomes"] if (o["rule"], o["target"]) == key), "(無い)")
        raise AssertionError(f"{key}: 期待 {wants}、実際 {got}\n  {msg}")


def problems(rep: dict) -> list[str]:
    """レポートから異常判定（`still_broken` / `unexpected`）のメッセージ一覧を抽出する。"""
    return [o["message"] for o in rep["outcomes"] if o["verdict"] in ("still_broken", "unexpected")]


def sha(path: str) -> str:
    """ファイルの SHA-256 ハッシュ文字列を返す（冪等性チェックのバイト完全一致確認用）。"""
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


# one_join() で生成される既定のターゲット名定数
J = "arm_with_ee.usda/ee"
A, C = "arm.usda", "two_finger.usda"


# ====================================================================== 1〜2: 純粋な部分（URDF モデル・仕様検証）
@test
def test_urdf_model_ghost_and_fk():
    """URDF モデルの空リンク（ghost）判定・純 Python 順運動学（FK）・固定子孫探索を検証する。

    検証内容:
      - `arm.urdf`: `base_link` は実体あり、`flange` は空リンク（ghost）と判定されること。
      - ゼロ姿勢での `flange` 高さ（z = 0.9 m）と `j1 = pi/2` 回転後の `link2` 位置・向きが
        解析解と `1e-9` 以内で一致すること（浮動小数点丸め誤差のみ許容）。
      - `gripper_ghost.urdf`: ルート `gripper_mount` が ghost で、固定関節配下の実体子孫が
        `["gripper_base"]` として解決されること。
    """
    import numpy as np

    m = load_urdf(urdf("arm.urdf"))
    assert not m.links["base_link"].ghost and m.links["flange"].ghost
    assert abs(m.fk_zero("flange")[2, 3] - 0.9) < 1e-9
    f = m.fk("link2", {"j1": np.pi / 2})
    assert np.allclose(f[:3, 3], [0, 0, 0.5]) and np.allclose(f[:3, 0], [0, 1, 0], atol=1e-9)
    g = load_urdf(urdf("gripper_ghost.urdf"))
    assert g.links[g.root_link].ghost and g.root_link == "gripper_mount"
    assert g.fixed_descendants("gripper_mount") == ["gripper_base"]


def raises(fn, *words: str) -> None:
    """`fn()` が `ValueError` を送出し、エラーメッセージに `words` がすべて含まれることを検証する。"""
    try:
        fn()
    except ValueError as exc:
        for w in words:
            assert w in str(exc), (w, str(exc))
        return
    raise AssertionError(f"誤りを通した: {words}")


@test
def test_spec_validation():
    """`PipelineSpec` と `check_links()` が設定ミスをビルド前に検出できることを検証する。

    検証内容:
      - 同一出力 `.usda` 名の重複、未定義 `parent_usda` 参照、`.usda` 拡張子漏れ、循環参照で
        具体的な日本語ヒント付きの `ValueError` が送出されること。
      - `check_links()` が親リンク名のタイポ、未定義 Variant 選択肢、非アクティブ Variant 配下の
        リンク参照（`child_variant_selection` 指定あり/なし両方）を漏れなく検出すること。
    """
    arm = UrdfSource(urdf=urdf("arm.urdf"), usda="arm.usda")
    two = UrdfSource(urdf=urdf("gripper.urdf"), usda="two_finger.usda")
    raises(
        lambda: PipelineSpec(output_dir=OUT, urdfs=(arm, UrdfSource(urdf=urdf("tool.urdf"), usda="arm.usda"))),
        "'arm.usda' を 2 か所で作ろうとしている",
        "[[urdf_to_usda]] 1 番目",
        "[[urdf_to_usda]] 2 番目",
    )
    raises(
        lambda: PipelineSpec(output_dir=OUT, urdfs=(arm,), join_files=(JoinFileSpec("u.usda", "arms.usda"),)),
        "parent_usda = 'arms.usda' は、この TOML のどこでも作っていない",
        "'arm.usda'",
    )
    raises(lambda: JoinFileSpec("unit", "arm.usda"), "拡張子 .usda まで書く")
    raises(
        lambda: PipelineSpec(
            output_dir=OUT,
            urdfs=(arm,),
            join_files=(
                JoinFileSpec("f.usda", "g.usda", (JoinSpec("x", "arm.usda", "flange", "base_link"),)),
                JoinFileSpec("g.usda", "f.usda"),
            ),
        ),
        "循環",
    )
    bad_link = PipelineSpec(
        output_dir=OUT,
        urdfs=(arm, two),
        join_files=(JoinFileSpec("u.usda", "arm.usda", (JoinSpec("ee", "two_finger.usda", "flang", "gripper_base"),)),),
    )
    msgs = check_links(bad_link)
    assert len(msgs) == 1 and "parent_link = 'flang'" in msgs[0] and "'flange'" in msgs[0], msgs
    bad_sel = PipelineSpec(
        output_dir=OUT,
        urdfs=(arm, two),
        join_files=(
            JoinFileSpec(
                "u.usda",
                "arm.usda",
                (JoinSpec("ee", "two_finger.usda", "flange", "gripper_base", variant_set="EE", variant="A"),),
            ),
            JoinFileSpec(
                "r.usda",
                "arm.usda",
                (JoinSpec("u1", "u.usda", "flange", "base_link", child_variant_selection=(("EE", "B"),)),),
            ),
        ),
    )
    msgs = check_links(bad_sel)
    assert len(msgs) == 1 and "選択肢 'B' が無い" in msgs[0], msgs
    two_ab = (
        JoinSpec("ee_a", "two_finger.usda", "flange", "gripper_base", variant_set="EE", variant="A"),
        JoinSpec("ee_b", "two_finger.usda", "flange", "gripper_base", variant_set="EE", variant="B"),
    )

    def in_variant(child_link: str, sel: tuple = ()) -> list[str]:
        return check_links(
            PipelineSpec(
                output_dir=OUT,
                urdfs=(arm, two),
                join_files=(
                    JoinFileSpec("u.usda", "arm.usda", two_ab),
                    JoinFileSpec(
                        "r.usda",
                        "arm.usda",
                        (JoinSpec("u1", "u.usda", "flange", child_link, child_variant_selection=sel),),
                    ),
                ),
            )
        )

    assert in_variant("ee_a/finger_l") == [], in_variant("ee_a/finger_l")
    msgs = in_variant("ee_b/finger_l")
    assert len(msgs) == 1 and "選ばれているのは 'A'" in msgs[0], msgs
    assert in_variant("ee_b/finger_l", (("EE", "B"),)) == []
    msgs = in_variant("ee_a/finger_l", (("EE", "None"),))
    assert len(msgs) == 1 and "選ばれているのは 'None'" in msgs[0], msgs
    msgs = check_links(
        PipelineSpec(
            output_dir=OUT,
            urdfs=(arm, two),
            join_files=(
                JoinFileSpec("u.usda", "arm.usda", two_ab),
                JoinFileSpec("r.usda", "u.usda", (JoinSpec("x", "arm.usda", "ee_b/finger_l", "base_link"),)),
            ),
        )
    )
    assert len(msgs) == 1 and "parent_link = 'ee_b/finger_l'" in msgs[0], msgs


# ====================================================================== 3〜4: 1 回の結合（既定のリンク配下配置）と冪等性
@test
def test_join_under_link():
    """既定配置（子を親リンク prim の配下に配置）での単体結合と規則判定・FK 一致を検証する。

    セットアップ:
      - `tests_kit/out/under_link` を初期化したうえで `one_join("under_link")` を `run_pipeline` で実行する。
    検証内容:
      - 全体ステータスが `"ok"` であること。
      - R06（Physics variant）・R07（ArticulationRoot 除去）・R09（閉ループ回避）・R14（固定関節増殖防止）が
        `"fixed"`、R10（衝突メッシュ可視化）が `"upstream_fixed"`、R02/R08/R12 が `"ok"` になること。
      - PhysX 順運動学検証で DOF 名完全一致、最大リンク位置誤差 `< 1e-4 m`（0.1 mm 未満、PhysX の
        単精度浮動小数と関節収束誤差に対して十分厳しい基準）であり、コンポーネント集合が
        `{"arm_with_ee", "ee"}` となること。
    """
    shutil.rmtree(os.path.join(OUT, "under_link"), ignore_errors=True)
    rep = run_pipeline(one_join("under_link")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    for key, want in {
        ("R06", A): "fixed",
        ("R06", C): "fixed",
        ("R07", J): "fixed",
        ("R09", J): "fixed",
        ("R14", J): "fixed",
        ("R10", A): "upstream_fixed",
        ("R02", J): "ok",
        ("R12", J): "ok",
        ("R08", J): "ok",
    }.items():
        expect(v, key, want, rep)
    g = rep["kinematics"]["arm_with_ee.usda"]
    # 後方互換キー `geometric` と新キー `kinematics` が同内容であることを確認
    assert g == rep["geometric"]["arm_with_ee.usda"]
    assert g["dof_names_match"] and g["max_link_position_error_m"] < 1e-4, g
    assert set(g["components_in_model"]) == {"arm_with_ee", "ee"}, g


@test
def test_idempotent_rerun():
    """同一入力で 2 回目の `run_pipeline()` を実行した際の完全な冪等性を検証する。

    検証内容:
      - 2 個の URDF とも入力ハッシュ一致により再インポートがスキップされること。
      - 結合フェーズの 1 回目・2 回目ともにファイル変化なしとログ出力されること。
      - 出力フォルダ配下の全 `.usd*` ファイルの SHA-256 が 1 バイトも変わらないこと。
      - R13 対策により `_import/*_1` のような重複フォルダが生成されず、`report_previous.json` が残ること。
    """
    out = os.path.join(OUT, "under_link")
    if not os.path.exists(os.path.join(out, "arm_with_ee.usda")):
        run_pipeline(one_join("under_link"))
    files = {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}
    logs: list[str] = []
    rep = run_pipeline(one_join("under_link"), log=logs.append).report
    assert sum(s.startswith("[import]") and "入力が同じなので飛ばした" in s for s in logs) == 2, logs
    assert any("1 回目 変化なし、2 回目 変化なし" in s for s in logs), logs
    assert files == {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}, "USD ファイルが変わった"
    assert not glob.glob(os.path.join(out, "_import", "*_1")), "別名のフォルダができた"
    assert rep["status"] == "ok" and os.path.exists(os.path.join(out, "report_previous.json"))


# ====================================================================== 5〜7: ルート prim 直下配置・skip・stop
@test
def test_join_under_top():
    """子を最上位 prim 配下（`place_child_under="."`）へ置く場合の規則発火と修正を検証する。

    検証内容:
      - 親剛体の子に置かないため R09（閉ループ）は発火せず `"ok"` になる一方、
        R02（子のワールド固定 joint 無効化）と R12（RobotPoser 追従修正）が `"fixed"` になること。
    """
    rep = run_pipeline(one_join("under_top", place=".")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R02", J), "fixed", rep)
    expect(v, ("R12", J), "fixed", rep)
    expect(v, ("R09", J), "ok", rep)


@test
def test_skip_shows_still_broken():
    """規則の自動修正を `fix="skip"` にしたとき、未修正の不具合が `still_broken` として検出されることを検証する。

    セットアップ:
      - `place="."` の結合に対して R02 / R07 / R12 の修正を意図的に無効化する。
    検証内容:
      - レポート全体が `"problems"` になり、3 規則すべてが `"still_broken"` と判定され、
        レポート文言に `"修正を skip した出力でも"` が含まれること。
    """
    rules = {r: RuleSetting(fix="skip") for r in ("R02", "R07", "R12")}
    rep = run_pipeline(one_join("under_top_skip", place=".", rules=rules)).report
    v = verdicts(rep)
    assert rep["status"] == "problems"
    for r in ("R02", "R07", "R12"):
        expect(v, (r, J), "still_broken", rep)
    msg = next(o["message"] for o in rep["outcomes"] if o["rule"] == "R02")
    assert "修正を skip した出力でも" in msg, msg


@test
def test_stop_before_join():
    """`on_violation="stop"` 指定時に、結合ファイルを生成せず事前停止することを検証する。

    検証内容:
      - R02 違反検出時点で `PipelineStopped` が送出され、`status == "stopped_before_join"`、
        `stopped_by` に `[R02] arm_with_ee.usda/ee` が記録され、`arm_with_ee.usda` が未作成であること。
    """
    s = one_join("stop", place=".", rules={"R02": RuleSetting(on_violation="stop")})
    try:
        run_pipeline(s)
    except PipelineStopped as exc:
        assert exc.report["status"] == "stopped_before_join"
        assert any(x.startswith(f"[R02] {J}") for x in exc.report["stopped_by"]), exc.report["stopped_by"]
        assert not os.path.exists(os.path.join(OUT, "stop", "arm_with_ee.usda")), "止めたのに結合ファイルができている"
        return
    raise AssertionError("止まらなかった")


# ====================================================================== 8: 空のルートリンクを持つ子
@test
def test_ghost_root_child():
    """空のルートリンク（`gripper_mount`）を持つ子 URDF 結合での R01 / R08 修正を検証する。

    検証内容:
      - R01（空ルートリンクによる非 `root_joint` アンカー名）と R08（剛体なしリンク結合の
        実体子孫フォールバック）が両方 `"fixed"` となり、FK 最大位置誤差が `< 1e-4 m` であること。
    """
    rep = run_pipeline(one_join("ghost", child_urdf="gripper_ghost.urdf", child_link="gripper_mount", place=".")).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R08", J), "fixed", rep)
    expect(v, ("R01", C), "fixed", rep)
    assert rep["kinematics"]["arm_with_ee.usda"]["max_link_position_error_m"] < 1e-4


# ====================================================================== 9: mimic follower の drive 減衰
@test
def test_mimic_damping():
    """R03（mimic follower の不要な drive 除去）の発火条件と修正効果を検証する。

    検証内容:
      - `gripper_damp.urdf` の `<dynamics damping>` は PhysX drive damping に入らないため
        `raw_follower_drive` は `0.0 / 0.0` で R03 は `"ok"`（不要な誤検知をしないこと）。
      - `joint_drive_damping=5.0` を明示指定した場合は follower drive が残るため
        R03 が検出・ゼロクリアして `"fixed"` になること。
    """
    rep = run_pipeline(one_join("damp_urdf", child_urdf="gripper_damp.urdf")).report
    expect(verdicts(rep), ("R03", f"{C}:finger_r_joint"), "ok", rep)
    assert rep["measurements"]["statics"][C]["raw_follower_drive"]["finger_r_joint"] == {"stiffness": 0.0, "damping": 0.0}
    rep = run_pipeline(one_join("damp_override", child_kw={"joint_drive_damping": 5.0})).report
    expect(verdicts(rep), ("R03", f"{C}:finger_r_joint"), "fixed", rep)


# ====================================================================== 10〜11: 4 本アーム（2 段結合・VariantSet・選択反映）
@test
def test_multi_arm():
    """多段結合 + VariantSet 選択（`examples/multi_arm.toml`）の完全動作を検証する。

    セットアップ:
      - 段 1 (`arm_unit.usda`): アーム先端に `EndEffector` VariantSet（`None` / `Rod` / `TwoFinger`）を定義。
      - 段 2 (`robot.usda`): 4 ポート台座へ `arm1=TwoFinger`, `arm2=Rod`, `arm3=TwoFinger`, `arm4=None` で結合。
    検証内容:
      - 全 6 結合で R07 / R09 が `"fixed"` であること。
      - `ArticulationRoot` が `/robot/Geometry/base_body` の 1 箇所、アンカーが `/robot/Physics/root_joint` の 1 本、
        総 DOF 数が 13（4 アーム×3 + 2 グリッパの代表軸等）で全関節の到達誤差 `< 1e-3` であること。
      - FK 最大リンク位置誤差 `< 1e-4 m`、モデル内コンポーネント一覧および USD 上の VariantSet 選択が
        TOML の指定通りであること。
    """
    s = dataclasses.replace(load_spec(os.path.join(EX, "multi_arm.toml")), output_dir=os.path.join(OUT, "multi_arm"))
    shutil.rmtree(s.output_dir, ignore_errors=True)
    rep = run_pipeline(s).report
    assert rep["status"] == "ok", problems(rep)
    v = verdicts(rep)
    for jid in (
        "arm_unit.usda/ee_two_finger",
        "arm_unit.usda/ee_rod",
        "robot.usda/arm1",
        "robot.usda/arm2",
        "robot.usda/arm3",
        "robot.usda/arm4",
    ):
        expect(v, ("R09", jid), "fixed", rep)
        expect(v, ("R07", jid), "fixed", rep)
    fx = rep["measurements"]["fixed"]["robot.usda/arm1"]
    assert fx["articulation_roots"] == ["/robot/Geometry/base_body"], fx["articulation_roots"]
    assert [a["path"] for a in fx["robot_anchors"]] == ["/robot/Physics/root_joint"]
    assert len(fx["dof_names"]) == 13, fx["dof_names"]
    assert all(r["error"] is not None and r["error"] < 1e-3 for r in fx["reach"].values()), fx["reach"]
    g = rep["kinematics"]["robot.usda"]
    assert set(g["components_in_model"]) == {
        "robot",
        "arm1",
        "arm2",
        "arm3",
        "arm4",
        "arm1/end_effector",
        "arm2/end_effector",
        "arm3/end_effector",
    }, g
    assert g["max_link_position_error_m"] < 1e-4
    from pxr import Usd

    unit = Usd.Stage.Open(os.path.join(str(s.output_dir), "arm_unit.usda"))
    vs = unit.GetDefaultPrim().GetVariantSet("EndEffector")
    assert vs.GetVariantNames() == ["None", "Rod", "TwoFinger"] and vs.GetVariantSelection() == "TwoFinger", vs.GetVariantNames()
    robot = Usd.Stage.Open(os.path.join(str(s.output_dir), "robot.usda"))
    sel = {
        p.GetName(): p.GetVariantSet("EndEffector").GetVariantSelection()
        for p in robot.Traverse()
        if p.GetName() in ("arm1", "arm2", "arm3", "arm4")
    }
    assert sel == {"arm1": "TwoFinger", "arm2": "Rod", "arm3": "TwoFinger", "arm4": "None"}, sel


@test
def test_multi_arm_rerun():
    """多段結合 + VariantSet（`multi_arm.toml`）の 2 回目再実行における冪等性を検証する。

    検証内容:
      - 出力フォルダ配下の全 `.usd*` ファイルの SHA-256 が再実行前後で完全一致すること。
    """
    out = os.path.join(OUT, "multi_arm")
    s = dataclasses.replace(load_spec(os.path.join(EX, "multi_arm.toml")), output_dir=out)
    if not os.path.exists(os.path.join(out, "robot.usda")):
        run_pipeline(s)
    files = {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}
    rep = run_pipeline(s, log=lambda *_: None).report
    assert files == {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}, "USD ファイルが変わった"
    assert rep["status"] == "ok"


# ====================================================================== 12: 実機ロボット（Franka Panda + Hand）
@test
def test_franka_example():
    """実機モデル `examples/franka/franka.toml`（Panda アーム + Hand）の結合精度を検証する。

    検証内容:
      - R09（閉ループ回避）が `"fixed"`、R04（末端リンク座標ずれ補正）が `"ok"` であること。
      - URDF 結合面アライメント誤差が位置 `< 1e-6 m`（1 μm 未満）・角度 `< 1e-3 deg` 未満であること
        （URDF 理論座標から厳密に配置されるためサブミクロン精度を要求）。
      - PhysX FK 最大リンク位置誤差が `< 1e-4 m`、コンポーネントが `{"panda_with_hand", "hand_mount"}` であること。
    """
    s = dataclasses.replace(load_spec(os.path.join(EX, "franka", "franka.toml")), output_dir=os.path.join(OUT, "franka"))
    rep = run_pipeline(s).report
    v = verdicts(rep)
    assert rep["status"] == "ok", problems(rep)
    expect(v, ("R09", "panda_with_hand.usda/hand_mount"), "fixed", rep)
    expect(v, ("R04", "panda_hand.usda"), "ok", rep)
    jid = "panda_with_hand.usda/hand_mount"
    al = rep["measurements"]["fixed"][jid]["joins"][jid]["urdf_mount_alignment"]
    assert al[0] < 1e-6 and al[1] < 1e-3, al
    g = rep["kinematics"]["panda_with_hand.usda"]
    assert g["max_link_position_error_m"] < 1e-4 and set(g["components_in_model"]) == {"panda_with_hand", "hand_mount"}, g


# ====================================================================== 13: サンプル TOML の読み込み
@test
def test_example_toml():
    """同梱のサンプル TOML（`arm_gripper.toml` / `multi_arm.toml`）が正しく解釈されることを検証する。

    検証内容:
      - 相対パスの解決、結合順トポロジカルソート `file_order()`、および
        `child_variant_selection` のタプル正規化が期待通りであること。
    """
    s = load_spec(os.path.join(EX, "arm_gripper.toml"))
    assert str(s.urdfs[0].urdf).endswith(os.path.join("examples", "urdf", "arm.urdf")) and s.urdfs[0].usda == "arm.usda"
    assert s.join_files[0].joins[0].parent_link == "flange"
    m = load_spec(os.path.join(EX, "multi_arm.toml"))
    assert [f.usda for f in m.file_order()] == ["arm_unit.usda", "robot.usda"]
    assert m.join_file("robot.usda").join("arm2").child_variant_selection == (("EndEffector", "Rod"),)


# ====================================================================== 14: build_assembly 非破壊ビルドと Descriptor API
@test
def test_build_assembly_and_descriptor():
    """`build_assembly()` のアクティブステージ非破壊性と Descriptor / Spawn API を検証する。

    検証内容:
      - アクティブステージに配置したセンチネル prim（`/SentinelNonDestructiveCheck`）が
        `build_assembly()` 実行後も破壊されずに残ること（GUI 編集中でも安全に呼べる保証）。
      - `AssembledRobotDescriptor` と `discover_components(..., include_inactive_variants=True)` が
        `arm4` の非アクティブ Variant（`EndEffector="None"`）を含む全コンポーネント経路を返すこと。
      - `spawn_assembled_robot()` で任意パス（`/World/robot`）へ配置した際にも、
        `to_articulation_source_dict()` が `/World/robot` プレフィックス付き辞書を正しく返すこと。
    """
    import omni.usd
    from pxr import Usd

    out = os.path.join(OUT, "multi_arm")
    s = dataclasses.replace(load_spec(os.path.join(EX, "multi_arm.toml")), output_dir=out)
    if not os.path.exists(os.path.join(out, "robot.usda")):
        build_assembly(s, log=lambda *_: None)

    # アクティブステージにセンチネル prim を置き、キャッシュ済み build_assembly がステージを壊さないことを確認
    ctx = omni.usd.get_context()
    if ctx.get_stage() is None:
        ctx.new_stage()
    active_stage = ctx.get_stage()
    active_stage.DefinePrim("/SentinelNonDestructiveCheck", "Xform")

    built = build_assembly(s, log=lambda *_: None)
    assert ctx.get_stage() is not None and ctx.get_stage().GetPrimAtPath("/SentinelNonDestructiveCheck").IsValid(), (
        "build_assembly がアクティブな USD ステージを破壊した"
    )

    desc: AssembledRobotDescriptor = built["descriptors"]["robot.usda"]
    assert isinstance(desc, AssembledRobotDescriptor)
    assert desc.prim_path == "/robot"
    expected_components = {
        "robot",
        "arm1",
        "arm2",
        "arm3",
        "arm4",
        "arm1/end_effector",
        "arm2/end_effector",
        "arm3/end_effector",
        "arm4/end_effector",
    }
    assert expected_components.issubset(set(desc.components.keys())), desc.components

    robot_stage = Usd.Stage.Open(os.path.join(out, "robot.usda"))
    comps_all = discover_components(robot_stage, "/robot", include_inactive_variants=True)
    assert "arm4/end_effector" in comps_all, comps_all

    src_dict = desc.to_articulation_source_dict()
    assert set(src_dict.keys()) == {"root_prim_path", "components"}, src_dict
    assert src_dict["root_prim_path"] == "/robot"
    assert "arm4/end_effector" in src_dict["components"], src_dict["components"]

    # spawn_assembled_robot でも同様に /World/robot プレフィックス付きの構成要素辞書が得られることを確認
    spawn_stage = Usd.Stage.CreateInMemory()
    spawn_desc = spawn_assembled_robot(spawn_stage, os.path.join(out, "robot.usda"), "/World/robot")
    spawn_src = spawn_desc.to_articulation_source_dict()
    assert spawn_src["root_prim_path"] == "/World/robot"
    assert spawn_src["components"]["robot"] == "/World/robot"
    assert "arm4/end_effector" in spawn_src["components"]
    assert spawn_src["components"]["arm4/end_effector"].startswith("/World/robot/")


# ====================================================================== 15: 複数 VariantSet と非 Variant 結合の混在順序
@test
def test_multi_variant_and_non_variant_ordering():
    """同一 `JoinFileSpec` 内に複数 VariantSet と非 Variant 結合が混在する場合の合成順序を検証する。

    セットアップ:
      - `Slot1` (ArmA / ArmB) -> 非 Variant (`fixed_arm_on_port3`) -> `Slot2` (Grip / Tool) の順で定義。
    検証内容:
      - `build_assembly(..., check_idempotency=True)` がエラーなく完了し、
        `Slot1` の既定選択が `"ArmA"`、`Slot2` の既定選択が `"Grip"` であること。
      - Descriptor に全コンポーネント（`mixed`, `slot1`, `slot2`, `fixed_arm_on_port3`）が含まれること。
    """
    from pxr import Usd

    out = os.path.join(OUT, "multi_variant_order")
    shutil.rmtree(out, ignore_errors=True)
    spec = PipelineSpec(
        output_dir=out,
        urdfs=(
            UrdfSource(urdf=urdf("multi_base.urdf"), usda="base.usda"),
            UrdfSource(urdf=urdf("arm.urdf"), usda="arm.usda"),
            UrdfSource(urdf=urdf("gripper.urdf"), usda="two_finger.usda"),
            UrdfSource(urdf=urdf("tool.urdf"), usda="rod.usda"),
        ),
        join_files=(
            JoinFileSpec(
                usda="mixed.usda",
                parent_usda="base.usda",
                joins=(
                    # VariantSet 1: Slot1 (既定 = "ArmA", 第 2 候補 = "ArmB")
                    JoinSpec(
                        name="slot1_a",
                        child_usda="arm.usda",
                        parent_link="port1",
                        child_link="base_link",
                        child_prim_name="slot1",
                        variant_set="Slot1",
                        variant="ArmA",
                    ),
                    JoinSpec(
                        name="slot1_b",
                        child_usda="rod.usda",
                        parent_link="port1",
                        child_link="tool_link",
                        child_prim_name="slot1",
                        variant_set="Slot1",
                        variant="ArmB",
                    ),
                    # Slot1 と Slot2 の間に挟まれた非 Variant 結合
                    JoinSpec(
                        name="fixed_arm_on_port3",
                        child_usda="arm.usda",
                        parent_link="port3",
                        child_link="base_link",
                    ),
                    # VariantSet 2: Slot2 (既定 = "Grip", 第 2 候補 = "Tool")
                    JoinSpec(
                        name="slot2_grip",
                        child_usda="two_finger.usda",
                        parent_link="port2",
                        child_link="gripper_base",
                        child_prim_name="slot2",
                        variant_set="Slot2",
                        variant="Grip",
                    ),
                    JoinSpec(
                        name="slot2_tool",
                        child_usda="rod.usda",
                        parent_link="port2",
                        child_link="tool_link",
                        child_prim_name="slot2",
                        variant_set="Slot2",
                        variant="Tool",
                    ),
                ),
            ),
        ),
    )
    built = build_assembly(spec, check_idempotency=True, log=lambda *_: None)
    comp = built["compositions"]["mixed.usda"]
    errors = {r.jid: r.error for r in comp.joins if r.error}
    assert not errors, f"混在 VariantSet の結合でエラーが発生した: {errors}"

    st = Usd.Stage.Open(os.path.join(out, "mixed.usda"))
    dp = st.GetDefaultPrim()
    assert dp.GetVariantSet("Slot1").GetVariantSelection() == "ArmA"
    assert dp.GetVariantSet("Slot2").GetVariantSelection() == "Grip"
    desc = built["descriptors"]["mixed.usda"]
    assert {"mixed", "slot1", "slot2", "fixed_arm_on_port3"}.issubset(set(desc.components.keys())), desc.components


# ====================================================================== 16: 外部幾何モデル拡張への依存ゼロ確認
@test
def test_zero_external_geometric_dependency():
    """v2 拡張が外部幾何モデル拡張（`myext.robot.geometric*`）に一切依存していないことを検証する。

    検証内容:
      - `myext.robot.assembly_pipeline.v2` と `myext.robot.assembly_pipeline_v2` が同一モジュールであること。
      - `sys.modules` にも `config/extension.toml` の `dependencies` にも禁止モジュールが存在しないこと。
    """
    import myext.robot.assembly_pipeline.v2 as pkg_dot
    import myext.robot.assembly_pipeline_v2 as pkg_under

    assert pkg_dot is pkg_under
    ext_mod_1 = ".".join(("myext", "robot", "geometric"))
    ext_mod_2 = ".".join(("myext", "robot", "geometric_newton"))
    loaded = [m for m in sys.modules if m == ext_mod_1 or m.startswith(ext_mod_1 + ".") or m == ext_mod_2 or m.startswith(ext_mod_2 + ".")]
    assert not loaded, f"禁止されている外部幾何モジュールが読み込まれている: {loaded}"

    with open(os.path.join(ROOT, "config", "extension.toml"), "rb") as f:
        ext_toml = tomllib.load(f)
    deps = set(ext_toml.get("dependencies", {}).keys())
    assert ext_mod_1 not in deps and ext_mod_2 not in deps, deps


# ====================================================================== 17: USD+USD（UR10e + Robotiq）パイプラインと冪等性
@test
def test_usd_to_usd_ur10e_pipeline():
    """公式 USD アセット同士の結合（`examples/official_assets/ur10e/ur10e.toml`）と冪等性を検証する。

    検証内容:
      - `run_pipeline()` が `"ok"` で完走し、FK 検証で DOF 名一致・最大リンク位置誤差 `< 1e-4 m` であること。
      - `discover_components(..., include_inactive_variants=True)` で本体とグリッパを検出できること。
      - 2 回目の `run_pipeline()` で全 `.usd*` の SHA-256 が完全に一致すること（冪等性）。
    """
    from pxr import Usd

    toml_path = os.path.join(EX, "official_assets", "ur10e", "ur10e.toml")
    out = os.path.join(OUT, "ur10e_usd_pipeline")
    shutil.rmtree(out, ignore_errors=True)
    s = dataclasses.replace(load_spec(toml_path), output_dir=out)
    rep = run_pipeline(s).report
    assert rep["status"] == "ok", problems(rep)

    final_usda = s.join_files[-1].usda
    g = rep["kinematics"][final_usda]
    assert g.get("dof_names_match"), g
    assert g["max_link_position_error_m"] < 1e-4, g

    st = Usd.Stage.Open(os.path.join(out, final_usda))
    dp_path = str(st.GetDefaultPrim().GetPath())
    comps = discover_components(st, dp_path, include_inactive_variants=True)
    assert len(comps) >= 2, comps

    # 冪等性チェック: 2 回目の run_pipeline で USD ファイルの SHA-256 が変わらない
    files = {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}
    rep2 = run_pipeline(s, log=lambda *_: None).report
    assert rep2["status"] == "ok", problems(rep2)
    assert files == {p: sha(p) for p in glob.glob(os.path.join(out, "**", "*.usd*"), recursive=True)}, "USD ファイルが変わった"


# ====================================================================== 18: 混在パイプライン（URDF アーム + USD グリッパ）
@test
def test_mixed_urdf_and_usd_pipeline():
    """URDF ソース（`arm.urdf`）と公式 USD ソース（`Robotiq_2F_85.usda`）の混在結合を検証する。

    セットアップ:
      - 親: `UrdfSource("arm.urdf")`、子: `UsdSource("Robotiq_2F_85.usda", Physics="Physx_Mimic")`
      - 親リンク `flange` に子リンク `base_link` を `"gripper"` として結合。
    検証内容:
      - `run_pipeline()` が `"ok"`、FK 検証で DOF 名一致・最大リンク位置誤差 `< 1e-4 m` であり、
        モデル内コンポーネントに `arm_with_robotiq` と `gripper` が含まれること。
    """
    out = os.path.join(OUT, "mixed_urdf_usd")
    shutil.rmtree(out, ignore_errors=True)
    robotiq_usd = (
        "C:/isaacsim-6.2.0/exts/isaacsim.asset.transformer.rules/data/tests/ur10e/Robotiq/2F-85/Robotiq_2F_85.usda"
    )
    spec = PipelineSpec(
        output_dir=out,
        urdfs=(UrdfSource(urdf=urdf("arm.urdf"), usda="arm.usda"),),
        usds=(
            UsdSource(
                usd=robotiq_usd,
                usda="robotiq_2f_85.usda",
                variant_selections=(("Physics", "Physx_Mimic"),),
            ),
        ),
        join_files=(
            JoinFileSpec(
                usda="arm_with_robotiq.usda",
                parent_usda="arm.usda",
                joins=(
                    JoinSpec(
                        name="gripper",
                        child_usda="robotiq_2f_85.usda",
                        parent_link="flange",
                        child_link="base_link",
                    ),
                ),
            ),
        ),
    )
    rep = run_pipeline(spec).report
    assert rep["status"] == "ok", problems(rep)
    g = rep["kinematics"]["arm_with_robotiq.usda"]
    assert g.get("dof_names_match"), g
    assert g["max_link_position_error_m"] < 1e-4, g
    assert {"arm_with_robotiq", "gripper"}.issubset(set(g["components_in_model"])), g


# ====================================================================== 19: parent_prim_name（親パーツも Prim 名として残す 2 段結合）
@test
def test_parent_prim_name_hierarchy():
    """`[[join_file]]` の `parent_prim_name` で親パーツも 1 段下に配置し全パーツ名が Stage に現れることを検証する。

    セットアップ:
      - 1 段目: `arm.usda` + `gripper.usda` → `child_unit.usda`（結合名 `grandchild`）
      - 2 段目: `base.usda` + `child_unit.usda` → `robot.usda`（`parent_prim_name = "base"`, 結合名 `child`）
    検証内容:
      - Stage 上に `/robot`・`/robot/base`・`/robot/base/.../child`・`/robot/base/.../child/.../grandchild` がすべて存在すること。
      - `discover_components(stage, "/robot")` に `"robot"`, `"base"`, `"base/child"`, `"base/child/grandchild"` が含まれること。
      - `run_pipeline` が `"ok"`、Play / RobotPoser / 順運動学検証（`< 1e-4 m`）すべて通ること。
      - TOML からの `parent_prim_name` 読み込みと、`place_child_under = "."` 衝突時のバリデーションが機能すること。
    """
    from pxr import Usd
    from myext.robot.assembly_pipeline_v2 import discover_components

    out = os.path.join(OUT, "parent_prim_name")
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out, exist_ok=True)
    toml_path = os.path.join(out, "robot_with_parent_prim.toml")
    with open(toml_path, "w", encoding="utf-8") as fh:
        fh.write(
            f"""output_dir = "build"

[[urdf_to_usda]]
urdf = {urdf("multi_base.urdf")!r}
usda = "base.usda"

[[urdf_to_usda]]
urdf = {urdf("arm.urdf")!r}
usda = "arm.usda"

[[urdf_to_usda]]
urdf = {urdf("gripper.urdf")!r}
usda = "gripper.usda"

[[join_file]]
usda = "child_unit.usda"
parent_usda = "arm.usda"

  [[join_file.join]]
  name = "grandchild"
  child_usda = "gripper.usda"
  parent_link = "flange"
  child_link = "gripper_base"

[[join_file]]
usda = "robot.usda"
parent_usda = "base.usda"
parent_prim_name = "base"

  [[join_file.join]]
  name = "child"
  child_usda = "child_unit.usda"
  parent_link = "port1"
  child_link = "base_link"
"""
        )

    spec = load_spec(toml_path)
    assert spec.join_file("robot.usda").parent_prim_name == "base"
    rep = run_pipeline(spec).report
    assert rep["status"] == "ok", problems(rep)

    g = rep["kinematics"]["robot.usda"]
    assert g.get("dof_names_match"), g
    assert g["max_link_position_error_m"] < 1e-4, g

    st = Usd.Stage.Open(os.path.join(spec.output_dir, "robot.usda"))
    assert st.GetPrimAtPath("/robot").IsValid()
    assert st.GetPrimAtPath("/robot/base").IsValid()
    assert st.GetPrimAtPath("/robot/base/Geometry/base_body/turret/port1/child").IsValid()
    assert st.GetPrimAtPath(
        "/robot/base/Geometry/base_body/turret/port1/child/Geometry/base_link/link1/link2/flange/grandchild"
    ).IsValid()

    comps = discover_components(st, "/robot", include_inactive_variants=True)
    assert {"robot", "base", "base/child", "base/child/grandchild"}.issubset(set(comps.keys())), comps
    assert comps["base"] == "/robot/base", comps

    # バリデーション: place_child_under="." で parent_prim_name と同名はエラー
    try:
        JoinFileSpec(
            usda="bad.usda",
            parent_usda="base.usda",
            parent_prim_name="base",
            joins=(
                JoinSpec(
                    name="base",
                    child_usda="arm.usda",
                    parent_link="port1",
                    child_link="base_link",
                    place_child_under=".",
                ),
            ),
        )
        raise AssertionError("parent_prim_name と place_child_under='.' の衝突がエラーにならなかった")
    except ValueError:
        pass


# ====================================================================== 20: 0 を跨がない URDF 関節リミット内の目標角クランプ検証
@test
def test_urdf_non_zero_joint_limit_clamping():
    """0 を含まない URDF 関節リミットでも `_drive_targets` がリミット内側の目標角を選ぶことを検証する。

    セットアップ:
      - `_clamp_revolute_target` を負区間（Franka `panda_joint4`: `[-3.0718, -0.0698]`）・正区間（`[0.5, 2.0]`）・
        0 端区間（`[0.0, 0.8]`）・対称区間（`[-3.0, 3.0]`）で単体検証。
      - `test_franka_example` と同じ Franka パイプラインの `reach["panda_joint4"]` で目標値が上限 `-0.0698 rad` より
        内側に設定され、リミット衝突なく到達することを確認。
    """
    from myext.robot.assembly_pipeline_v2.probes import _clamp_revolute_target

    # 負のみの可動域（Franka panda_joint4）: 以前のように 0.8 * (-0.0698) = -0.05584 (上限外) にならない
    t_neg = _clamp_revolute_target(0.3, -3.0718, -0.0698)
    assert -3.0718 < t_neg < -0.0698, f"負区間のリミット外: {t_neg}"

    # 正のみの可動域: 以前のように 0.8 * 0.5 = 0.4 (下限外) にならない
    t_pos = _clamp_revolute_target(0.3, 0.5, 2.0)
    assert 0.5 < t_pos < 2.0, f"正区間のリミット外: {t_pos}"

    # 0 境界区間・対称区間
    t_zero = _clamp_revolute_target(0.0, 0.0, 0.8)
    assert 0.0 < t_zero < 0.8, f"0 境界区間の内側でない: {t_zero}"
    assert abs(_clamp_revolute_target(0.3, -3.0, 3.0) - 0.3) < 1e-12

    # Franka 実測確認: panda_joint4 の目標角が URDF 上限 (-0.0698 rad) の内側で、誤差 < 0.01 rad で到達する
    franka_rep_path = os.path.join(OUT, "franka", "report.json")
    if os.path.exists(franka_rep_path):
        with open(franka_rep_path, encoding="utf-8") as fh:
            frep = json.load(fh)
        fixed_map = frep["measurements"]["fixed"]
        first_probe = next(iter(fixed_map.values()))
        j4 = first_probe["reach"]["panda_joint4"]
        assert -3.0718 < j4["target"] < -0.0698, f"Franka panda_joint4 target がリミット外: {j4}"
        assert j4["error"] < 0.01, f"Franka panda_joint4 到達誤差が大きい: {j4}"


# ====================================================================== 集計・出力
summary = {"passed": sum(r["ok"] for r in results), "failed": sum(not r["ok"] for r in results), "results": results}
with open(os.path.join(HERE, "last_run.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=1)
print(f"[SUMMARY] passed {summary['passed']} / failed {summary['failed']}", flush=True)
sys.stdout.flush()
os._exit(1 if summary["failed"] else 0)
