# lab_parts.py — 観測 Snippet と実測スクリプトから取り出した、再利用できる部品をまとめて builtins.lab に登録する
# 解説: 05_観測Snippet/10_lab_parts.md
"""観測 Snippet と実測スクリプトで何度も使った処理を、部品として1か所にまとめる。

Isaac Sim の Script Editor で1回実行すると、別のタブからも lab.add_rigid_cube(...) のように呼べる。
部品の一覧は、実行したときに表示される。

Note:
    呼べる状態: 登録は Stop／Play／Pause のどれでもできる。部品ごとに呼べる状態はそれぞれの docstring に書いた。
    backend: read_with_backend 以外の部品は backend を選ばない。
"""

from __future__ import annotations

import asyncio  # async の関数を Kit のイベントループで走らせるために使う
import builtins  # 別のタブからも lab.〇〇 と呼べるようにするために使う
import os  # ファイルを探すために使う
import re  # 設定のパスを入れた定数を探すために使う
import statistics  # 中央値を計算するために使う
import time  # 高分解能タイマー perf_counter を使うために使う
import types  # 部品をまとめる入れ物（SimpleNamespace）を作るために使う
import typing

import carb.settings  # Kit の設定を読み書きするために使う
import isaacsim.core.experimental.utils.backend as backend_utils  # バックエンドを指定する use_backend を使う
import omni.kit.app  # メインループの更新を待つために使う
import omni.timeline  # Play / Pause / Stop を切り替えるために使う
import omni.usd  # 開いているステージを取得するために使う
from isaacsim.core.simulation_manager import SimulationManager  # 物理の出力先の切り替えと、Stop のままの物理の準備に使う
from pxr import Gf, UsdGeom, UsdPhysics  # USD のプリムと物理の API

LAB_ROOT: typing.Final[str] = r"C:\isaacsim-6.2.0\CodeByClaude\isaac-sim-backend-lab"  # このノートブックの場所（移動したら書き換える）
ISAAC_SIM_ROOT: typing.Final[str] = r"C:\isaacsim-6.2.0"  # Isaac Sim のインストール先（find_setting_usages が探す場所）
StateName: typing.TypeAlias = typing.Literal["Play", "Pause", "Stop"]  # timeline の状態の名前
OutputMode: typing.TypeAlias = typing.Literal["usd", "fabric", "none"]  # 物理の計算結果の書き出し先
USD_WRITEBACK_KEYS: typing.Final[tuple[str, ...]] = (  # 物理の結果を USD へ書き戻す4つの設定
    "/physics/updateToUsd",  # 姿勢
    "/physics/updateVelocitiesToUsd",  # 速度
    "/physics/updateParticlesToUsd",  # パーティクル
    "/physics/updateForceSensorsToUsd",  # 力覚センサー
)


# ================================================================ 時間を測る
def time_per_call_us(action: typing.Callable[[], object], repeat_count: int = 100) -> float:
    """action を repeat_count 回呼び、1回あたりの所要時間の中央値を µs で返す。

    呼べる状態: どれでもよい（action が呼べる状態であること）。
    """
    samples = []  # 1回ごとの所要時間（秒）
    for _ in range(repeat_count):  # repeat_count 回くり返す
        start = time.perf_counter()  # 開始時刻
        action()  # 計る処理を1回呼ぶ
        samples.append(time.perf_counter() - start)  # 経過時間を記録する
    return statistics.median(samples) * 1e6  # 中央値を µs にして返す


# ================================================================ 状態の切り替えと、更新を待つ
def state_name() -> StateName:
    """今の timeline の状態を "Play" / "Pause" / "Stop" で返す。"""
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    if timeline.is_playing():  # Play 中なら
        return "Play"
    return "Stop" if timeline.is_stopped() else "Pause"  # Stop か Pause かを返す


async def wait_updates(update_count: int = 1) -> None:
    """メインループの更新を update_count 回待つ。async の関数の中で await して使う。"""
    app = omni.kit.app.get_app()  # 更新を待つためのアプリのインスタンス
    for _ in range(update_count):  # update_count 回
        await app.next_update_async()  # 更新を1回待つ


async def switch_state(target_state: StateName, settle_updates: int = 1) -> None:
    """timeline を target_state に切り替え、settle_updates 回の更新を待って反映させる。async の関数の中で await して使う。

    play() などは呼んだ時点では状態を変えず、次の更新で反映される（実測）。
    """
    timeline = omni.timeline.get_timeline_interface()  # timeline のインスタンス
    {"Play": timeline.play, "Pause": timeline.pause, "Stop": timeline.stop}[target_state]()  # 状態を切り替える
    await wait_updates(settle_updates)  # 反映されるまで待つ


def run_async(coroutine: typing.Coroutine[object, object, object]) -> asyncio.Future:
    """async の関数の呼び出し結果を、Kit のイベントループで走らせる。この関数はすぐに戻る。

    使い方: lab.run_async(my_steps())  ※ my_steps は async def で書いた関数
    """
    return asyncio.ensure_future(coroutine)  # Kit のイベントループに登録する


# ================================================================ シーンを作る
def add_rigid_cube(path: str, position: tuple[float, float, float] = (0.0, 0.0, 1.0), size: float = 0.1, kinematic: bool = False) -> str:
    """剛体とコライダーを付けた Cube を path に置き、そのパスを返す。kinematic=True なら、力で動かず、指定した姿勢に置くだけの剛体にする。

    呼べる状態: Stop 中に呼ぶ（Play 中に足した剛体は、次の Play まで物理に入らない）。
    """
    stage = omni.usd.get_context().get_stage()  # 開いているステージ
    cube = UsdGeom.Cube.Define(stage, path)  # Cube を作る
    cube.CreateSizeAttr(size)  # 一辺の長さ
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*position))  # 位置を決める
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())  # 剛体にする
    rigid_body.CreateKinematicEnabledAttr(kinematic)  # キネマティックにするか
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())  # コライダーを付ける
    return path  # パスを返す


def add_cube_grid(cube_count: int, root_path: str = "/World/Grid", spacing: float = 0.3, height: float = 1.0, kinematic: bool = False) -> list[str]:
    """cube_count 個の剛体の Cube を、xy 平面に spacing 間隔の格子状に並べ、パスのリストを返す。"""
    columns = max(1, int(cube_count ** 0.5))  # 1行に並べる数
    return [  # 作った Cube のパスのリスト
        add_rigid_cube(f"{root_path}/c_{index}", (spacing * (index % columns), spacing * (index // columns), height), kinematic=kinematic)  # 1個作る
        for index in range(cube_count)  # cube_count 個
    ]


def add_minimal_articulation(root_path: str = "/World/robot", base_position: tuple[float, float, float] = (2.0, 0.0, 1.0)) -> str:
    """土台を世界に固定し、1つの回転関節（角度のドライブ付き）で棒をつないだ最小の Articulation を置き、ルートのパスを返す。

    ドライブ（drive:angular）を付けるのは、Stop 中に Articulation が DOF の種類を USD のドライブの属性から決めるためである。
    ドライブがないと、Stop 中に DOF の目標値などを読んだときに「Invalid DOF type」の警告が出る（ソースと実測）。
    """
    stage = omni.usd.get_context().get_stage()  # 開いているステージ
    UsdGeom.Xform.Define(stage, root_path)  # Articulation のルート
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root_path))  # ルートに Articulation の印を付ける
    base = add_rigid_cube(f"{root_path}/base", base_position)  # 土台
    link = add_rigid_cube(f"{root_path}/link", (base_position[0], base_position[1], base_position[2] + 0.3))  # 棒
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/fixed_joint")  # 土台を世界に固定する関節
    fixed.CreateBody1Rel().SetTargets([base])  # body0 を空にすると世界につながる
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*base_position))  # 世界側の固定位置
    revolute = UsdPhysics.RevoluteJoint.Define(stage, f"{root_path}/revolute_joint")  # 回転関節
    revolute.CreateBody0Rel().SetTargets([base])  # 土台側
    revolute.CreateBody1Rel().SetTargets([link])  # 棒側
    revolute.CreateAxisAttr("Y")  # Y 軸まわりに回る
    revolute.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0.15))  # 土台から見た関節の位置
    revolute.CreateLocalPos1Attr(Gf.Vec3f(0, 0, -0.15))  # 棒から見た関節の位置
    drive = UsdPhysics.DriveAPI.Apply(revolute.GetPrim(), "angular")  # 角度のドライブを付ける
    drive.CreateStiffnessAttr(1000.0)  # 目標角へ引き戻す強さ
    drive.CreateDampingAttr(100.0)  # 速度を抑える強さ
    return root_path  # ルートのパスを返す


# ================================================================ 物理の出力先
def set_physics_output(mode: OutputMode) -> None:
    """物理の計算結果の書き出し先を切り替える。"usd": USD へ書く（既定）、"fabric": Fabric へ書き USD へは書かない、"none": どちらにも書かない。

    呼べる状態: Stop 中に呼ぶ。Play 中に /physics/updateToUsd を変えた場合も、その場で効く（実測）。
    """
    SimulationManager.enable_fabric(mode == "fabric")  # omni.physx.fabric と、USD へ書き戻す4つの設定を切り替える
    if mode == "none":  # どちらにも書かない場合は
        for key in USD_WRITEBACK_KEYS:  # USD へ書き戻す4つの設定を
            carb.settings.get_settings().set_bool(key, False)  # false にする


def physics_output_summary() -> str:
    """今の物理の出力先の設定を1行の文字列で返す。"""
    settings = carb.settings.get_settings()  # 設定ストア
    writeback = {key.rsplit("/", 1)[1]: settings.get(key) for key in USD_WRITEBACK_KEYS}  # USD へ書き戻す4つの設定
    return f"omni.physx.fabric={SimulationManager.is_fabric_enabled()} {writeback}"  # 1行にまとめる


# ================================================================ バックエンドを指定して読む
def read_with_backend(wrapper: object, backend: str | None, method_name: str = "get_world_poses", **kwargs: object) -> object:
    """wrapper.method_name(**kwargs) を、指定した backend で呼ぶ。指定どおりに処理されなければ例外にする。backend が None なら既定の選択に任せる。

    例: lab.read_with_backend(xform, "fabric") は、with use_backend("fabric", raise_on_unsupported=True, raise_on_fallback=True) の中で xform.get_world_poses() を呼ぶ。
    """
    method = getattr(wrapper, method_name)  # 呼ぶメソッド
    if backend is None:  # 指定しない場合は
        return method(**kwargs)  # そのまま呼ぶ
    with backend_utils.use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):  # 指定どおりでなければ例外にする
        return method(**kwargs)  # 指定したバックエンドで呼ぶ


# ================================================================ Stop のまま物理を使う
def start_physics_without_play() -> None:
    """timeline を Stop のまま、Play を押したときと同じ物理の準備（物理の読み込み・ウォームアップ・view の作成）をする。

    この後は、timeline が Stop のままでも tensor の view が使え、SimulationManager.step(steps=n) を呼んだときだけ物理が進む。
    メインループの更新では物理は進まない。PhysX のシーンクエリ（指定した箱や球と重なっている剛体を問い合わせる
    omni.physx.get_physx_scene_query_interface().overlap_box など）も使えるようになる（実測）。
    Play を押した後に Stop すると view は無効になり、もう一度使うにはこの関数を呼び直す。
    """
    SimulationManager.initialize_physics()  # Play 時に SimulationManager が呼ぶ処理を、ここで呼ぶ


def set_kinematic_targets(rigid_prim: object, positions: object = None, orientations: object = None) -> None:
    """キネマティックな剛体の「目標の姿勢」を tensor で設定する。次の SimulationManager.step から、剛体はこの姿勢に置かれ続ける。

    RigidPrim.set_world_poses（tensor）は剛体の今の姿勢を書き換えるだけで、目標の姿勢は変えない。
    そのため、キネマティックな剛体は次の step で目標の姿勢（USD に書いてあった最初の姿勢）へ戻る（実測）。
    重なりを step の後にも調べたいときは、この部品で目標の姿勢を設定してから step する。

    Args:
        rigid_prim: キネマティックな剛体だけを受け持つ isaacsim.core.experimental.prims.RigidPrim。
        positions: 位置（形 (N, 3)）。None なら変えない。
        orientations: 向きの四元数 w, x, y, z（形 (N, 4)）。None なら変えない。

    呼べる状態: tensor の view が有効なとき（Play 中・Pause 中、または start_physics_without_play の後）。
    RigidPrim に公開のメソッドがないため、RigidPrim の内部の view（_physics_rigid_body_view）の set_kinematic_targets を呼ぶ。
    """
    import isaacsim.core.experimental.utils.ops as ops_utils  # 配列の形をそろえる関数（RigidPrim と同じもの）
    import warp as wp  # GPU / CPU の配列

    view = rigid_prim._physics_rigid_body_view  # omni.physics.tensors の RigidBodyView
    assert view is not None, "tensor の view が有効でない。Play するか start_physics_without_play() を呼ぶ"
    data = view.get_transforms()  # 今の姿勢（形 (N, 7)、並びは x, y, z, qx, qy, qz, qw）
    indices = ops_utils.resolve_indices(None, count=len(rigid_prim), device=data.device)  # すべての剛体の番号
    if positions is not None:  # 位置を変えるなら
        positions = ops_utils.broadcast_to(positions, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device)  # 形と置き場所をそろえる
        wp.copy(data[indices, :3], positions)  # 位置の列に書き込む
    if orientations is not None:  # 向きを変えるなら
        orientations = ops_utils.broadcast_to(orientations, shape=(indices.shape[0], 4), dtype=wp.float32, device=data.device)  # 形と置き場所をそろえる
        wp.copy(data[indices, wp.array([6, 3, 4, 5], dtype=wp.int32, device=data.device)], orientations)  # w, x, y, z を qw, qx, qy, qz の列に書き込む
    view.set_kinematic_targets(data, indices)  # 目標の姿勢として設定する（反映されるのは次の step）


# ================================================================ Snippet を実行する
def run_snippet(snippet_name: str, replacements: dict[str, str] | None = None) -> dict:
    """snippets/<snippet_name>.py を、Script Editor で実行したのと同じように実行する。replacements で、実行前にソースの一部を書き換えられる。

    例: lab.run_snippet("pose_freshness_probe", {"OUTPUT_MODE: OutputMode = None": "OUTPUT_MODE: OutputMode = 'fabric'"})
    """
    path = os.path.join(LAB_ROOT, "snippets", f"{snippet_name}.py")  # Snippet のファイル
    with open(path, encoding="utf-8") as source_file:  # ソースを読む
        source = source_file.read()
    for old_text, new_text in (replacements or {}).items():  # 書き換えを順に当てる
        if old_text not in source:  # 書き換える場所がなければ
            raise ValueError(f"{snippet_name}.py に {old_text!r} がない")  # 間違いに気づけるように例外にする
        source = source.replace(old_text, new_text)  # 書き換える
    namespace = {"__name__": "__main__"}  # Script Editor で実行したときと同じ名前空間の名前
    exec(compile(source, path, "exec"), namespace)  # 実行する
    return namespace  # 実行後の変数を返す


# ================================================================ 設定がどこで読まれているかを探す
def _enclosing_place(lines: list[str], line_index: int) -> str:
    """lines[line_index] の行が、モジュールの最上位にあるか、どの関数の中にあるかを返す。"""
    line = lines[line_index]  # 調べる行
    indent = len(line) - len(line.lstrip())  # 行の字下げ
    for previous in reversed(lines[:line_index]):  # 上の行をさかのぼって、囲んでいる関数を探す
        stripped = previous.lstrip()  # 先頭の空白を除いた行
        if stripped.startswith(("def ", "async def ")) and len(previous) - len(stripped) < indent:  # 字下げの浅い def があれば
            return f"関数 {stripped.split('(')[0].split()[-1]} の中"  # その関数の中
    return "モジュールの最上位"  # 見つからなければモジュールの最上位


def find_setting_usages(setting_key: str, max_hits: int = 40) -> None:
    """Isaac Sim のインストール先の Python ソースと kit / toml ファイルから、setting_key を読んでいる場所を探して表示する。

    「起動後に変えて効くか」を判断する材料にする。表示する「位置」は次の意味である。
      モジュールの最上位 … import されたときに1回だけ読まれる（起動後に変えても、読み込み済みなら効かない）
      関数 <名前> の中 … その関数が呼ばれるたびに読まれる（その関数がいつ呼ばれるかで決まる。__init__ や on_startup なら作成時・起動時に1回）
      subscribe を含む行 … 値の変更を受け取る仕組みに登録している（起動後に変えても効く）
    設定のパスを定数（NAME = "/a/b"）に入れている場合は、その定数を使っている行も同じファイルから探して表示する。
    tests フォルダの中と、テストの起動オプション（"--/a/b=..."）は表示しない。
    C++ のプラグインの中で読まれている設定（omni.physx など）は、インストール先には Python のソースがないので見つからない。
    """
    python_hits: list[str] = []  # Python のソースで見つけた行
    config_hits: list[str] = []  # kit / toml で見つけた行
    leaf = setting_key.rsplit("/", 1)[-1]  # kit / toml では「a.b.leaf = 値」の形で書かれるので、最後の名前で探す
    constant_pattern = re.compile(r"^\s*([A-Z_][A-Z0-9_]*)\s*=\s*[\"']" + re.escape(setting_key) + r"[\"']")  # 「定数名 = "設定のパス"」の行
    for folder in ("exts", "extscache", "apps", "kit"):  # 探すフォルダ
        for directory, _, file_names in os.walk(os.path.join(ISAAC_SIM_ROOT, folder)):  # 中のファイルをたどる
            if f"{os.sep}tests" in directory:  # テストのフォルダは飛ばす
                continue
            for file_name in file_names:  # ファイルごとに
                if not file_name.endswith((".py", ".kit", ".toml")):  # 対象の種類だけ見る
                    continue
                path = os.path.join(directory, file_name)  # ファイルのパス
                try:  # 読めないファイルは飛ばす
                    with open(path, encoding="utf-8", errors="ignore") as source_file:
                        lines = source_file.readlines()
                except OSError:
                    continue
                if file_name.endswith(".py"):  # Python のソースなら
                    for index, line in enumerate(lines):  # 行ごとに
                        if setting_key not in line:  # 設定のパスを含まない行は飛ばす
                            continue
                        constant = constant_pattern.match(line)  # 定数に入れている行か
                        if constant:  # 定数なら、その定数を使っている行を探す
                            python_hits.append(f"{path}:{index + 1}  [定数 {constant.group(1)} の定義]  {line.strip()[:110]}")
                            for use_index, use_line in enumerate(lines):  # 同じファイルの中で
                                if use_index != index and re.search(rf"\b{constant.group(1)}\b", use_line):  # 定数を使っている行
                                    place = _enclosing_place(lines, use_index) + ("（subscribe を含む行）" if "subscribe" in use_line else "")
                                    python_hits.append(f"{path}:{use_index + 1}  [{place}]  {use_line.strip()[:110]}")
                        else:  # 設定のパスを直接書いている行なら
                            place = _enclosing_place(lines, index) + ("（subscribe を含む行）" if "subscribe" in line else "")
                            python_hits.append(f"{path}:{index + 1}  [{place}]  {line.strip()[:110]}")
                else:  # kit / toml なら
                    for index, line in enumerate(lines):  # 行ごとに
                        if "=" in line and leaf in line.split("=")[0] and "--/" not in line:  # 「…最後の名前 = 値」の行（テストの起動オプションは除く）
                            config_hits.append(f"{path}:{index + 1}  [既定値を書いている場所]  {line.strip()[:110]}")
    hits = python_hits + config_hits  # Python のソースを先に表示する
    for hit in hits[:max_hits]:  # 多すぎる場合は max_hits 件まで
        print(hit)  # 見つけた場所を表示する
    if len(hits) > max_hits:  # 打ち切った場合は
        print(f"... 全 {len(hits)} 件のうち {max_hits} 件を表示した")
    if not hits:  # 1件もなければ
        print(f"{setting_key} を読んでいる Python ソースと kit / toml は見つからなかった（C++ のプラグインで読まれている可能性がある）")


# ================================================================ 登録
builtins.lab = types.SimpleNamespace(  # 部品をまとめて lab という名前で登録する
    time_per_call_us=time_per_call_us,  # 時間を測る
    state_name=state_name,  # 今の状態
    wait_updates=wait_updates,  # 更新を待つ（async）
    switch_state=switch_state,  # 状態を切り替えて待つ（async）
    run_async=run_async,  # async の関数を走らせる
    add_rigid_cube=add_rigid_cube,  # 剛体の Cube を置く
    add_cube_grid=add_cube_grid,  # 剛体の Cube を格子状に並べる
    add_minimal_articulation=add_minimal_articulation,  # 最小の Articulation を置く
    set_physics_output=set_physics_output,  # 物理の出力先を切り替える
    physics_output_summary=physics_output_summary,  # 物理の出力先の設定を表示用にまとめる
    read_with_backend=read_with_backend,  # バックエンドを指定して読む
    start_physics_without_play=start_physics_without_play,  # Stop のまま物理を準備する
    set_kinematic_targets=set_kinematic_targets,  # キネマティックな剛体の目標の姿勢を設定する
    run_snippet=run_snippet,  # Snippet を実行する
    find_setting_usages=find_setting_usages,  # 設定を読んでいる場所を探す
)
print("lab に登録した部品:", ", ".join(name for name in vars(builtins.lab)))  # 登録した部品の名前を表示する
