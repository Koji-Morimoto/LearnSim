"""OpenUSD(pxr)を扱う共通ユーティリティ層。

責務:
  - 座標変換行列の規約統一: この拡張内部ではすべて NumPy 4x4 列ベクトル形式(`p_parent = T @ p_child`)で扱い、
    USD `Gf.Matrix4d`(行ベクトル形式)との相互変換を本モジュールの `gf_to_np` / `np_to_gf` に集約する。
  - API スキーマの安全な存在判定(`has_api`)、剛体・関節・ワールド固定アンカーの構造解析(`resolve_body`,
    `rigid_descendants_via_fixed`, `is_structural_anchor`, `world_anchors`)を提供する。
  - URDF 由来・既存 USD 由来のリンク / 関節 / mimic 関係抽出(`link_prims`, `usd_link_prims`, `usd_mimic_joints`)を提供する。
  - Physics VariantSet 選択(`physics_variant_choice`, `select_physics_variant`)と、
    冪等なレイヤー再生成・差分保存・強制リロード(`open_for_rewrite`, `write_if_changed`, `reload_layers_under`)を提供する。

主な入出力・公開 API:
  - 定数: `ROBOT_LINKS`, `ROBOT_JOINTS`, `ARTICULATION_ROOT_APIS`, `ALL_PRIMS`, `DEFINED_PRIMS`, `ACTIVE_PRIMS`
  - パス・スキーマ: `asset_ref_path`, `has_api`, `display_name`
  - 座標変換: `gf_to_np`, `np_to_gf`, `rigid`, `scale_of`, `world`, `local_xform`, `pose_error`,
    `set_local_pose`, `preserve_scale`, `joint_local`
  - 物理・構造解析: `has_rigid_body`, `resolve_body`, `rigid_body_at_or_above`, `rigid_descendants_via_fixed`,
    `joint_targets`, `joints_under`, `joint_active`, `is_structural_anchor`, `world_anchors`,
    `articulation_roots`, `rigid_bodies`
  - 抽出: `link_prims`, `joint_prims`, `usd_link_prims`, `usd_mimic_joints`
  - Variant・レイヤー: `physics_variant_choice`, `select_physics_variant`, `open_for_rewrite`,
    `write_if_changed`, `reload_layers_under`

呼ばれ元:
  - `__init__.py`（一部公開 API の再エクスポート）
  - `descriptor.py`, `sources.py`, `part_import.py`, `compose.py`, `official.py`,
    `probes.py`, `kinematics_check.py`, `pipeline.py`

拡張・変更ポイント:
  - 新しい USD Physics / Isaac / Newton スキーマへ対応する場合は、`ARTICULATION_ROOT_APIS` や
    `usd_mimic_joints`・`usd_link_prims` のみを更新すればパイプライン全体へ波及する。
"""

from __future__ import annotations

import os
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# Isaac Sim の IsaacRobotAPI がロボット prim に書き込むリレーションシップ名。
ROBOT_LINKS = "isaac:physics:robotLinks"
ROBOT_JOINTS = "isaac:physics:robotJoints"
# Articulation ルートになり得る全スキーマ名(USD Physics 標準 / Newton / PhysX 拡張)。
ARTICULATION_ROOT_APIS = ("PhysicsArticulationRootAPI", "NewtonArticulationRootAPI", "PhysxArticulationAPI")
# インスタンスプロキシ配下も含めて走査するための PrimRange 述語群。
# 既存 USD アセットはメッシュやリンクを instanceable にしていることが多いため TraverseInstanceProxies を常に有効化する。
ALL_PRIMS = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
DEFINED_PRIMS = Usd.TraverseInstanceProxies(Usd.PrimIsDefined & ~Usd.PrimIsAbstract)
ACTIVE_PRIMS = Usd.TraverseInstanceProxies(Usd.PrimIsActive & Usd.PrimIsDefined & ~Usd.PrimIsAbstract)
_PHYSX_MIMIC_PREFIX = "PhysxMimicJointAPI:"
# `usd_link_prims` の第 3 優先探索で、リンクではなく描画・衝突・マテリアル整理用とみなして除外するコンテナ名。
_SKIP_XFORM_CONTAINERS = frozenset(
    {"visuals", "collisions", "looks", "materials", "joints", "meshes", "colliders", "render", "sensors", "camera"}
)


def asset_ref_path(target: str, start_dir: str) -> str:
    """USD の Reference に書き込むアセットパスを返す。

    引数:
      target: 参照先のファイルパスまたは URI(`"://"` を含む S3 / Omniverse パスなど)。
      start_dir: 参照元 `.usda` ファイルが置かれるディレクトリ。

    戻り値:
      URI ならそのまま、ローカルパスなら `start_dir` からのスラッシュ区切り相対パス(`"./..."` または `"../..."`)。
      Windows でドライブ文字が異なるなど相対化不能な場合はスラッシュ正規化済みの絶対パスにフォールバックする。
    """
    if "://" in target:
        return target
    try:
        rel = os.path.relpath(target, start_dir).replace("\\", "/")
    except ValueError:
        return target.replace("\\", "/")
    # USD のアセット解決で探索パスと誤認されないよう、同一階層配下は明示的に "./" を付ける。
    return rel if rel.startswith(".") else "./" + rel


# ---------------------------------------------------------------- スキーマ判定
def has_api(prim: Usd.Prim | None, schema_name: str | Any) -> bool:
    """`prim` に指定 API スキーマが適用されているかを例外を出さずに判定する。

    引数:
      prim: 検査対象の `Usd.Prim`。無効または `None` の場合は `False`。
      schema_name: API スキーマ名文字列(例: `"IsaacRobotAPI"`, `"PhysxMimicJointAPI"`)、
                   または `UsdPhysics.RigidBodyAPI` などのスキーマクラス。

    戻り値:
      適用されていれば `True`。

    なぜ 2 段フォールバックしているか:
      - Isaac / Newton / PhysX 系の一部スキーマは、プラグイン未ロード環境では `prim.HasAPI("...")` が
        型解決できずに例外または `False` を返すことがある。
      - また複数適用(multiple-apply)スキーマは `GetAppliedSchemas()` に `"SchemaName:instanceName"` 形式で
        格納されるため、完全一致と `schema_name + ":"` 前方一致の両方を確認したうえで `HasAPI` にフォールバックする。
    """
    if not prim or not prim.IsValid():
        return False
    if not isinstance(schema_name, str):
        try:
            return bool(prim.HasAPI(schema_name))
        except Exception:  # noqa: BLE001
            schema_name = getattr(schema_name, "__name__", str(schema_name))
    try:
        applied = prim.GetAppliedSchemas()
        if schema_name in applied or any(s.startswith(schema_name + ":") for s in applied):
            return True
    except Exception:  # noqa: BLE001
        pass
    try:
        return bool(prim.HasAPI(schema_name))
    except Exception:  # noqa: BLE001
        return False


def _prim_path_str(stage: Usd.Stage, path_or_prim: str | Sdf.Path | Usd.Prim | None) -> str | None:
    """`str` / `Sdf.Path` / `Usd.Prim` のいずれかを prim パス文字列に正規化する内部補助。"""
    if path_or_prim is None:
        return None
    if isinstance(path_or_prim, Usd.Prim):
        return str(path_or_prim.GetPath()) if path_or_prim.IsValid() else None
    return str(path_or_prim)


def _get_prim(stage: Usd.Stage, path_or_prim: str | Sdf.Path | Usd.Prim | None) -> Usd.Prim | None:
    """`str` / `Sdf.Path` / `Usd.Prim` から有効な `Usd.Prim` を取得する(無効なら `None`)。"""
    if path_or_prim is None:
        return None
    if isinstance(path_or_prim, Usd.Prim):
        return path_or_prim if path_or_prim.IsValid() else None
    p = stage.GetPrimAtPath(str(path_or_prim))
    return p if p and p.IsValid() else None


# ---------------------------------------------------------------- 行列変換
def gf_to_np(m: Gf.Matrix4d) -> np.ndarray:
    """USD の `Gf.Matrix4d`(行ベクトル形式)を NumPy 4x4 列ベクトル形式へ転置変換する。"""
    return np.array(m, dtype=float).T


def np_to_gf(m: np.ndarray) -> Gf.Matrix4d:
    """NumPy 4x4 列ベクトル形式を USD の `Gf.Matrix4d`(行ベクトル形式)へ転置変換する。"""
    return Gf.Matrix4d(*np.asarray(m, dtype=float).T.flatten().tolist())


def rigid(m: np.ndarray) -> np.ndarray:
    """4x4 同次変換行列 `m` からスケール・せん断を除いた最寄りの剛体変換(SO(3) + 並進)を返す。

    なぜ SVD + 行列式チェックを使うか:
      親 prim に非一様スケールや数値誤差が含まれるワールド行列から関節ローカルフレームや姿勢誤差を求める際、
      単純な列正規化では直交性が崩れる。SVD で最寄り直交行列へ射影し、`det < 0` の鏡映を最終列反転で補正する。
    """
    out = np.eye(4)
    r = m[:3, :3]
    u, _, vt = np.linalg.svd(r)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u_fix = u.copy()
        u_fix[:, -1] *= -1.0
        rot = u_fix @ vt
    out[:3, :3] = rot
    out[:3, 3] = m[:3, 3]
    return out


rigid_part = rigid


def scale_of(m: np.ndarray) -> np.ndarray:
    """4x4 変換行列の左上 3x3 の各列ノルムから `(sx, sy, sz)` スケールベクトルを取り出す。"""
    return np.linalg.norm(m[:3, :3], axis=0)


def world(
    stage: Usd.Stage,
    path: str | Sdf.Path | Usd.Prim,
    cache: UsdGeom.XformCache | None = None,
) -> np.ndarray:
    """`path` のローカル→ワールド変換行列(NumPy 4x4 列ベクトル形式)を返す。prim 無効時は単位行列。"""
    prim = _get_prim(stage, path)
    if prim is None:
        return np.eye(4)
    cache = cache or UsdGeom.XformCache()
    return gf_to_np(cache.GetLocalToWorldTransform(prim))


world_xform = world


def local_xform(prim: Usd.Prim) -> np.ndarray:
    """`prim` の親に対するローカル変換行列(NumPy 4x4 列ベクトル形式)を返す。prim 無効時は単位行列。"""
    if not prim or not prim.IsValid():
        return np.eye(4)
    return gf_to_np(UsdGeom.Xformable(prim).GetLocalTransformation())


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """2 つの 4x4 姿勢 `a`, `b` の差を `(並進誤差[m], 回転角度誤差[deg])` で返す。"""
    d = np.linalg.inv(rigid(a)) @ rigid(b)
    # 浮動小数点誤差で [-1, 1] をわずかに超えると arccos が NaN になるためクランプする。
    c = max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1.0) / 2.0))
    return float(np.linalg.norm(d[:3, 3])), float(np.degrees(np.arccos(c)))


def set_local_pose(prim: Usd.Prim, m: np.ndarray) -> None:
    """`prim` の既存 `xformOpOrder` をクリアし、`translate` + `orient`(倍精度)で剛体姿勢 `m` を書き込む。"""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    r = rigid(m)
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*r[:3, 3]))
    q = Gf.Matrix4d(np_to_gf(r)).ExtractRotationQuat()
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))


set_translate_orient = set_local_pose


def preserve_scale(prim: Usd.Prim, m: np.ndarray | None = None) -> None:
    """`prim` が元々持つ非 1 スケールを保ったまま `translate` + `orient`(+ 必要なら `scale`)を書き込む。

    なぜ必要か:
      `set_local_pose` は `ClearXformOpOrder()` するため、単位変換用スケール(例: cm→m の 0.01)が付いた
      USD アセットへそのまま適用するとスケールが消える。先に現在スケールを退避し、1 でなければ末尾に復元する。
    """
    cur = local_xform(prim)
    s = scale_of(cur)
    target = cur if m is None else m
    set_local_pose(prim, target)
    if not np.allclose(s, np.ones(3), atol=1e-6):
        UsdGeom.Xformable(prim).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*s.tolist()))


def joint_local(m: np.ndarray) -> tuple[Gf.Vec3f, Gf.Quatf]:
    """NumPy 4x4 剛体変換行列を `UsdPhysics.Joint` の `localPos` / `localRot` 用の `(Gf.Vec3f, Gf.Quatf)` へ変換する。"""
    r = rigid(m)
    q = np_to_gf(r).ExtractRotationQuat()
    return Gf.Vec3f(*r[:3, 3]), Gf.Quatf(float(q.GetReal()), Gf.Vec3f(q.GetImaginary()))


# ---------------------------------------------------------------- 剛体と関節の構造解析
def has_rigid_body(prim: Usd.Prim | None) -> bool:
    """`prim` 自身に `UsdPhysics.RigidBodyAPI` が付いていれば `True`。"""
    return bool(prim and prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI))


def resolve_body(stage: Usd.Stage, path: str | Sdf.Path | Usd.Prim | None) -> Usd.Prim | None:
    """USD Physics / PhysX の `body0` / `body1` 解決規則を再現して対応する剛体(または静的コライダー)を返す。

    解決手順:
      1. `path` から親方向へ祖先をたどり、最初に見つかった `RigidBodyAPI` 付き prim を返す。
      2. 祖先すべてに `RigidBodyAPI` が無ければ、最初に見つかった `CollisionAPI` 付き prim(静的コライダー)を返す。
      3. どちらも無ければ `None`(ワールド固定扱い)を返す。
    """
    if not path:
        return None
    prim = _get_prim(stage, path)
    collider = None
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        if collider is None and prim.HasAPI(UsdPhysics.CollisionAPI):
            collider = prim
        prim = prim.GetParent()
    return collider


def rigid_body_at_or_above(stage: Usd.Stage, path: str | Sdf.Path | Usd.Prim | None) -> Usd.Prim | None:
    """`path` 自身またはその祖先で最初に見つかる `RigidBodyAPI` 付き prim を返す(無ければ `None`)。"""
    if not path:
        return None
    prim = _get_prim(stage, path)
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def rigid_descendants_via_fixed(
    stage: Usd.Stage,
    root: str | Sdf.Path | Usd.Prim,
    start_link_path: str | Sdf.Path | Usd.Prim | None = None,
) -> list[Usd.Prim]:
    """`root` 配下で有効な `FixedJoint` をたどって到達できる剛体 prim を近い順(BFS)に返す。

    引数:
      stage: 対象の `Usd.Stage`。
      root: 探索範囲を限定する部品ルート prim(またはパス)。この配下の剛体のみ返す。
      start_link_path: BFS の起点リンク(省略時は `root`)。

    戻り値:
      到達可能な `RigidBodyAPI` 付き `Usd.Prim` の重複排除済みリスト。

    なぜ無向グラフ BFS を使うか:
      USD の `FixedJoint` は `body0`/`body1` の向きが必ずしも親子順と一致しない。
      無向辺として構築したうえで BFS し、各到達先を `rigid_body_at_or_above` で剛体へ射影することで、
      結合点のリンクがゴーストリンク(剛体なし)でも物理的に剛結された最寄りの実剛体を確実に拾える。
    """
    root_path = _prim_path_str(stage, root)
    if not root_path:
        return []
    start_path = _prim_path_str(stage, start_link_path) or root_path
    fixed_edges: dict[str, list[str]] = {}
    for jp in joints_under(stage, root_path):
        if not jp.IsA(UsdPhysics.FixedJoint) or not joint_active(jp):
            continue
        t0, t1 = joint_targets(jp)
        if t0 and t1:
            fixed_edges.setdefault(t0, []).append(t1)
            fixed_edges.setdefault(t1, []).append(t0)
    out: list[Usd.Prim] = []
    seen_bodies: set[str] = set()
    visited: set[str] = {start_path}
    queue: list[str] = [start_path]
    while queue:
        cur = queue.pop(0)
        for nxt in fixed_edges.get(cur, []):
            if nxt in visited:
                continue
            visited.add(nxt)
            queue.append(nxt)
            rb = rigid_body_at_or_above(stage, nxt)
            if rb is not None:
                rbp = str(rb.GetPath())
                # 部品スコープ外の親ロボット側剛体へ漏れ出さないよう `root_path` 配下のみ収集する。
                if (rbp == root_path or rbp.startswith(root_path + "/")) and rbp not in seen_bodies:
                    seen_bodies.add(rbp)
                    out.append(rb)
    return out


rigid_Descendants_via_fixed = rigid_descendants_via_fixed


def joint_targets(prim: Usd.Prim) -> tuple[str | None, str | None]:
    """`UsdPhysics.Joint` prim の `(body0 パス, body1 パス)` を返す(未設定側は `None`)。"""
    j = UsdPhysics.Joint(prim)
    t0, t1 = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
    return (str(t0[0]) if t0 else None, str(t1[0]) if t1 else None)


def joints_under(stage: Usd.Stage, root: str | Sdf.Path | Usd.Prim) -> list[Usd.Prim]:
    """`root` 配下の全 `UsdPhysics.Joint` prim を返す(非アクティブやインスタンスプロキシ配下も含む)。"""
    prim = _get_prim(stage, root)
    if not prim:
        return []
    return [p for p in Usd.PrimRange(prim, ALL_PRIMS) if p.IsA(UsdPhysics.Joint)]


def joint_active(prim: Usd.Prim) -> bool:
    """`prim` が有効・アクティブかつ `physics:jointEnabled` が `False` でなければ `True`。"""
    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    en = UsdPhysics.Joint(prim).GetJointEnabledAttr().Get()
    return en is None or bool(en)


def is_structural_anchor(stage: Usd.Stage, joint_prim: Usd.Prim, *, scope: str | None = None) -> bool:
    """`joint_prim` が `scope` の内側の剛体を外側(ワールドまたは別部品)へ固定するアンカー関節かを構造で判定する。

    引数:
      stage: 対象の `Usd.Stage`。
      joint_prim: 判定対象の関節 prim。
      scope: 部品のルート prim パス。`None` のときは「任意の剛体 vs ワールド(`None`)」を判定する。

    戻り値:
      有効かつ Articulation 除外されていない関節で、`body0` と `body1` の片方だけが `scope` 内に解決されれば `True`。

    なぜ名前(`"root_joint"` 等)ではなく構造で判定するか:
      URDF Importer や外部 USD アセットによってルート固定関節の命名はバラバラ(`root_joint`, `world_joint`,
      `FixedJoint` 等)だが、「片側が部品内剛体、もう片側が部品外(ワールドまたは親剛体)」というトポロジ条件は普遍的であり、
      R05/R09 の子部品ルート固定解除で見落としや誤削除を防げる。
    """
    if not joint_active(joint_prim) or UsdPhysics.Joint(joint_prim).GetExcludeFromArticulationAttr().Get():
        return False
    t0, t1 = joint_targets(joint_prim)
    b0, b1 = resolve_body(stage, t0), resolve_body(stage, t1)

    def inside(b: Usd.Prim | None) -> bool:
        if b is None:
            return False
        if scope is None:
            return True
        bp = str(b.GetPath())
        return bp == scope or bp.startswith(scope + "/")

    return inside(b0) != inside(b1)


def world_anchors(
    stage: Usd.Stage,
    root: str | Sdf.Path | Usd.Prim,
    *,
    scope: str | None = None,
    skip: tuple[str, ...] = (),
) -> list[dict[str, Any]]:
    """`root` 配下にある外部固定アンカー関節の一覧を辞書リストで返す(関節名には依存しない)。

    引数:
      stage: 対象の `Usd.Stage`。
      root: 走査起点の prim またはパス。
      scope: `is_structural_anchor` に渡す境界パス。省略時は「剛体 vs ワールド」のみ検出。
             子部品を親剛体の子階層に Reference 配置した場合、子部品内の `root_joint` の `body0` が
             親剛体に解決されるため、子部品のルートパスを `scope` に渡すことでその外部固定も確実に検出できる。
      skip: 検査から除外する関節 prim パスのタプル(結合で新規作成した自身の FixedJoint を除外する用途など)。

    戻り値:
      `{"path", "name", "type", "body0", "body1", "both_targets_authored"}` を要素とするリスト。
    """
    root_path = _prim_path_str(stage, root)
    if not root_path:
        return []
    out: list[dict[str, Any]] = []
    for p in joints_under(stage, root_path):
        if str(p.GetPath()) in skip:
            continue
        if not is_structural_anchor(stage, p, scope=scope):
            continue
        t0, t1 = joint_targets(p)
        out.append({
            "path": str(p.GetPath()),
            "name": p.GetName(),
            "type": p.GetTypeName(),
            "body0": t0,
            "body1": t1,
            "both_targets_authored": t0 is not None and t1 is not None,
        })
    return out


def articulation_roots(stage: Usd.Stage, root: str | Sdf.Path | Usd.Prim) -> list[str]:
    """`root` 配下で `UsdPhysics.ArticulationRootAPI` を持つ全 prim パスを返す。"""
    prim = _get_prim(stage, root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, ALL_PRIMS) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def rigid_bodies(stage: Usd.Stage, root: str | Sdf.Path | Usd.Prim) -> list[str]:
    """`root` 配下で `UsdPhysics.RigidBodyAPI` を持つ全 prim パスを返す。"""
    prim = _get_prim(stage, root)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, ALL_PRIMS) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


# ---------------------------------------------------------------- リンク・関節・Mimic の抽出
def display_name(prim: Usd.Prim) -> str:
    """`prim` の `displayName` メタデータを返す(未設定なら USD prim 名)。

    なぜ `displayName` を優先するか:
      URDF のリンク/関節名にハイフン等が含まれる場合、URDF Importer は prim 名をサニタイズし、
      元の URDF 名を `displayName` に保存するため。
    """
    if not prim or not prim.IsValid():
        return ""
    v = prim.GetMetadata("displayName")
    return v if v else prim.GetName()


def link_prims(
    stage: Usd.Stage,
    robot_root: str | Sdf.Path | Usd.Prim,
    names: list[str] | None = None,
) -> dict[str, str]:
    """URDF 由来ロボットの `{リンク名: prim パス}` マップを構築する。

    探索順序:
      1. `robot_root` の `isaac:physics:robotLinks` リレーションシップ(子部品ロボット `IsaacRobotAPI` は除外)。
      2. 手順 1 が空なら `robot_root` 配下の全 `RigidBodyAPI` prim。
      3. `names` が渡された場合、まだ見つかっていないリンク(剛体を持たない空のルートリンク等)を
         既知リンクの祖先パスから同名・非 Gprim で補完。
      4. さらに残る未発見リンク(空の末端フレーム `flange` / `panda_link8` 等)を
         `IsaacSiteAPI` / `IsaacLinkAPI` または既知リンク直下の子 prim から補完。
    """
    robot_root_str = _prim_path_str(stage, robot_root) or ""
    robot = _get_prim(stage, robot_root)
    out: dict[str, str] = {}
    rel = robot.GetRelationship(ROBOT_LINKS) if robot else None
    targets = [str(t) for t in rel.GetTargets()] if rel else []
    for t in targets:
        p = stage.GetPrimAtPath(t)
        # 結合後に子部品のルート prim(IsaacRobotAPI 付き)が robotLinks に混在しても、自部品リンクと混同しないよう除外する。
        if p and t.startswith(robot_root_str + "/") and not has_api(p, "IsaacRobotAPI"):
            out.setdefault(display_name(p), t)
    if not out and robot:
        for p in Usd.PrimRange(robot, ALL_PRIMS):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                out.setdefault(display_name(p), str(p.GetPath()))
    for name in names or []:
        if name in out:
            continue
        for known in list(out.values()):
            path = Sdf.Path(known)
            while path != Sdf.Path(robot_root_str) and path != Sdf.Path.absoluteRootPath:
                path = path.GetParentPath()
                prim = stage.GetPrimAtPath(path)
                if prim and display_name(prim) == name and not prim.IsA(UsdGeom.Gprim):
                    out[name] = str(path)
                    break
            if name in out:
                break
    # 空の末端リンクは site(IsaacSiteAPI 付きの Xform)として残るが、URDF Importer のバージョンや構造により
    # robotLinks に入る場合(例: 自作 arm の flange)と入らない場合(例: Franka の panda_link8)があるため、ここで補う。
    missing = [n for n in names or [] if n not in out]
    if missing and robot:
        known = set(out.values())
        for p in Usd.PrimRange(robot, ALL_PRIMS):
            n = display_name(p)
            if (
                n in missing
                and n not in out
                and not p.IsA(UsdGeom.Gprim)
                and (
                    has_api(p, "IsaacSiteAPI")
                    or has_api(p, "IsaacLinkAPI")
                    or str(p.GetParent().GetPath()) in known
                )
            ):
                out[n] = str(p.GetPath())
    return out


def joint_prims(stage: Usd.Stage, robot_root: str | Sdf.Path | Usd.Prim) -> dict[str, str]:
    """`robot_root` 配下の全関節を `{関節名(displayName 優先): prim パス}` で返す。"""
    return {display_name(p): str(p.GetPath()) for p in joints_under(stage, robot_root)}


def usd_link_prims(stage: Usd.Stage, robot_root: str | Sdf.Path | Usd.Prim) -> dict[str, str]:
    """既存 USD アセット部品の `{リンク/取付フレーム名: prim パス}` マップを優先順位付きで抽出する。

    なぜ 3 段階の優先順位で `setdefault` するか:
      公式 USD アセット(UR10e / G1 / Robotiq / OpenArm 等)は URDF が手元に無く、
      取付フレーム(`tool0`, `flange`, `ee_link`, `backpack_link` 等)が `RigidBodyAPI` を持たない単なる `Xform` として
      置かれていることが多い。同名 prim が衝突したときに物理的信頼度の高い候補を必ず勝たせるため、以下の順で登録する:
        1. `link_prims`(`robotLinks` または `RigidBodyAPI`)のうち現在アクティブな prim
        2. アクティブな `RigidBodyAPI` / `IsaacLinkAPI` / `IsaacSiteAPI`(`robot_root` 自身も含む)
        3. アクティブな非 Gprim / 非 Joint の `Xformable` 子孫(`_SKIP_XFORM_CONTAINERS` 配下を除外) + `robot_root` フォールバック
    """
    robot_root_str = _prim_path_str(stage, robot_root) or ""
    robot = _get_prim(stage, robot_root)
    if not robot or not robot_root_str:
        return {}

    # 優先度 1: IsaacRobotAPI の robotLinks(または RigidBody)由来のアクティブ prim
    out: dict[str, str] = {}
    for k, v in link_prims(stage, robot_root_str, names=None).items():
        p = stage.GetPrimAtPath(v)
        if p and p.IsValid() and p.IsActive():
            out.setdefault(k, v)

    # 優先度 2: 明示的な物理/Isaac タグ付き prim(剛体を持たない子ロボットルートは除外)
    for p in Usd.PrimRange(robot, ACTIVE_PRIMS):
        p_path = str(p.GetPath())
        is_rb = p.HasAPI(UsdPhysics.RigidBodyAPI)
        is_isaac = has_api(p, "IsaacLinkAPI") or has_api(p, "IsaacSiteAPI")
        if not (is_rb or is_isaac):
            continue
        if p_path != robot_root_str and has_api(p, "IsaacRobotAPI") and not is_rb:
            continue
        out.setdefault(display_name(p), p_path)

    # 優先度 3: 剛体や Isaac タグが付かない取付用 Xform(visuals / collisions / looks 等の中身は除外して誤爆を防ぐ)
    prefix = robot_root_str.rstrip("/") + "/"
    for p in Usd.PrimRange(robot, ACTIVE_PRIMS):
        p_path = str(p.GetPath())
        if p_path == robot_root_str:
            continue
        if p.IsA(UsdGeom.Gprim) or p.IsA(UsdPhysics.Joint) or not p.IsA(UsdGeom.Xformable):
            continue
        if has_api(p, "IsaacRobotAPI"):
            continue
        rel = p_path[len(prefix) :] if p_path.startswith(prefix) else p_path
        parts = [seg.lower() for seg in rel.split("/") if seg]
        if any(seg in _SKIP_XFORM_CONTAINERS for seg in parts):
            continue
        out.setdefault(display_name(p), p_path)

    # 優先度 4(フォールバック): ルート prim 自身の名前でも参照できるようにする
    out.setdefault(display_name(robot), robot_root_str)
    return out


def usd_mimic_joints(stage: Usd.Stage, root: str | Sdf.Path | Usd.Prim) -> list[dict[str, Any]]:
    """`root` 配下のアクティブな mimic follower 関節情報を USD スキーマから抽出する。

    戻り値:
      `{"follower", "follower_path", "leader", "leader_path", "multiplier", "offset", "joint_type", "schema"}`
      を要素とする辞書リスト。`multiplier` と `offset` は URDF 規約 `q_follower = multiplier * q_leader + offset` に統一される。

    符号規約の注意点(なぜ PhysxMimicJointAPI で符号反転するのか):
      - `NewtonMimicAPI`: `q_follower = coef1 * q_leader + coef0` をそのまま表すため `multiplier = coef1, offset = coef0`。
      - `PhysxMimicJointAPI:<axis>`: PhysX の拘束式は `gearing * q_leader + q_follower + offset = 0` すなわち
        `q_follower = -gearing * q_leader - offset` で定義されているため、`multiplier = -gearing, offset = -offset` へ反転する。
    """
    out: list[dict[str, Any]] = []
    for p in joints_under(stage, root):
        if not joint_active(p):
            continue
        schemas = list(p.GetAppliedSchemas())
        instances = [s[len(_PHYSX_MIMIC_PREFIX) :] for s in schemas if s.startswith(_PHYSX_MIMIC_PREFIX)]
        if "NewtonMimicAPI" in schemas:
            en_attr = p.GetAttribute("newton:mimicEnabled")
            en = en_attr.Get() if en_attr else True
            if en is not None and not en:
                continue
            rel = p.GetRelationship("newton:mimicJoint")
            targets = rel.GetTargets() if rel else []
            if not targets:
                continue
            c0_attr = p.GetAttribute("newton:mimicCoef0")
            c1_attr = p.GetAttribute("newton:mimicCoef1")
            c0 = c0_attr.Get() if c0_attr else None
            c1 = c1_attr.Get() if c1_attr else None
            multiplier = float(1.0 if c1 is None else c1)
            offset = float(0.0 if c0 is None else c0)
            schema = "NewtonMimicAPI"
        elif instances:
            inst = instances[0]
            rel = p.GetRelationship(f"physxMimicJoint:{inst}:referenceJoint")
            targets = rel.GetTargets() if rel else []
            if not targets:
                continue
            g_attr = p.GetAttribute(f"physxMimicJoint:{inst}:gearing")
            o_attr = p.GetAttribute(f"physxMimicJoint:{inst}:offset")
            gearing = g_attr.Get() if g_attr else None
            off = o_attr.Get() if o_attr else None
            multiplier = -float(1.0 if gearing is None else gearing)
            offset = -float(0.0 if off is None else off)
            schema = f"{_PHYSX_MIMIC_PREFIX}{inst}"
        else:
            continue
        leader_path = str(targets[0])
        leader_prim = stage.GetPrimAtPath(leader_path)
        jtype = (
            "revolute"
            if p.IsA(UsdPhysics.RevoluteJoint)
            else ("prismatic" if p.IsA(UsdPhysics.PrismaticJoint) else p.GetTypeName())
        )
        out.append(
            {
                "follower": display_name(p),
                "follower_path": str(p.GetPath()),
                "leader": (
                    display_name(leader_prim)
                    if (leader_prim and leader_prim.IsValid())
                    else leader_path.rsplit("/", 1)[-1]
                ),
                "leader_path": leader_path,
                "multiplier": multiplier,
                "offset": offset,
                "joint_type": jtype,
                "schema": schema,
            }
        )
    return out


# ---------------------------------------------------------------- Variant とレイヤー管理
def physics_variant_choice(prim: Usd.Prim, wanted: str = "auto", *, keep_existing: bool = False) -> str | None:
    """`prim` の `Physics` VariantSet から選ぶべき選択肢名を決定する(ステージは書き換えない)。

    引数:
      prim: 検査対象の prim。`Physics` VariantSet を持たなければ `None`。
      wanted: 希望する選択肢名、または `"auto"`。
      keep_existing: `True` かつ `wanted == "auto"` のとき、既に `"None"` 以外の有効な選択が付いていれば
                     それを尊重して維持する(既存 USD アセット固有の物理バリアントを勝手に上書きしないため)。

    戻り値:
      選択すべきバリアント名。該当なしなら `None`。
    """
    if not prim or not prim.IsValid():
        return None
    vs = prim.GetVariantSets()
    if not vs.HasVariantSet("Physics"):
        return None
    vset = vs.GetVariantSet("Physics")
    names = list(vset.GetVariantNames())
    if wanted == "auto":
        if keep_existing:
            cur = vset.GetVariantSelection()
            if cur and cur.lower() != "none" and cur in names:
                return cur
        for n in ("physx", "physics"):
            if n in names:
                return n
        # 公式アセットによって大文字小文字や名前が異なる(PhysX / Physx_Mimic / SimplifiedPhysX 等)ため小文字照合する。
        by_lower = {n.lower(): n for n in names}
        for key in ("physx", "physics", "physx_mimic", "simplifiedphysx"):
            if key in by_lower:
                return by_lower[key]
        return None
    if wanted in names:
        return wanted
    by_lower = {n.lower(): n for n in names}
    return by_lower.get(wanted.lower())


def select_physics_variant(prim: Usd.Prim, wanted: str = "auto", *, keep_existing: bool = False) -> str | None:
    """`physics_variant_choice` の結果があれば `prim` の `Physics` VariantSet に設定し、選択名を返す。"""
    choice = physics_variant_choice(prim, wanted, keep_existing=keep_existing)
    if choice:
        prim.GetVariantSet("Physics").SetVariantSelection(choice)
    return choice


def open_for_rewrite(path: str | os.PathLike) -> Sdf.Layer:
    """出力 `.usda` 用の `Sdf.Layer` を空の状態で開いて返す(毎回クリーンに作り直す冪等生成の入口)。

    既存ファイルがある場合は既存レイヤーを `Clear()` して再利用する。
    ディスク書き込みは `write_if_changed` に委ねるため、内容が変わらなければタイムスタンプを更新しない。
    """
    p = os.fspath(path)
    parent_dir = os.path.dirname(os.path.abspath(p))
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)
    layer = Sdf.Layer.FindOrOpen(p) if os.path.exists(p) else None
    if layer is None:
        layer = Sdf.Layer.CreateNew(p)
    else:
        layer.Clear()
    return layer


def write_if_changed(layer: Sdf.Layer, path: str | os.PathLike | None = None) -> bool:
    """`layer` のシリアライズ結果が既存ファイルと異なる場合のみ `layer.Save()` を実行する。

    戻り値:
      実際に保存した場合は `True`、内容が完全一致して保存を省略した場合は `False`。
    """
    target = os.fspath(path) if path is not None else (layer.realPath or layer.identifier)
    text = layer.ExportToString()
    try:
        with open(target, encoding="utf-8") as f:
            if f.read() == text:
                return False
    except OSError:
        pass
    layer.Save()
    return True


_write_if_changed = write_if_changed


def reload_layers_under(folder: str | os.PathLike) -> None:
    """`folder` 配下に実体を持つ読み込み済み `Sdf.Layer` をすべて強制リロードする。

    なぜ必要か:
      同一 Kit プロセス内で複数回パイプラインを実行したり結合ファイルを上書きした直後に
      `Usd.Stage.Open` を呼ぶと、OpenUSD のレイヤーキャッシュが古い内容を返すことがあるため。
    """
    folder_norm = os.path.normcase(os.path.abspath(os.fspath(folder)))
    for layer in Sdf.Layer.GetLoadedLayers():
        real = layer.realPath
        if real and os.path.normcase(os.path.abspath(real)).startswith(folder_norm):
            layer.Reload(force=True)


__all__ = [
    "ROBOT_LINKS",
    "ROBOT_JOINTS",
    "ARTICULATION_ROOT_APIS",
    "ALL_PRIMS",
    "DEFINED_PRIMS",
    "ACTIVE_PRIMS",
    "asset_ref_path",
    "has_api",
    "gf_to_np",
    "np_to_gf",
    "rigid",
    "rigid_part",
    "scale_of",
    "world",
    "world_xform",
    "local_xform",
    "pose_error",
    "set_local_pose",
    "set_translate_orient",
    "preserve_scale",
    "joint_local",
    "has_rigid_body",
    "resolve_body",
    "rigid_body_at_or_above",
    "rigid_descendants_via_fixed",
    "rigid_Descendants_via_fixed",
    "joint_targets",
    "joints_under",
    "joint_active",
    "is_structural_anchor",
    "world_anchors",
    "articulation_roots",
    "rigid_bodies",
    "display_name",
    "link_prims",
    "joint_prims",
    "usd_link_prims",
    "usd_mimic_joints",
    "physics_variant_choice",
    "select_physics_variant",
    "open_for_rewrite",
    "write_if_changed",
    "_write_if_changed",
    "reload_layers_under",
]
