"""純粋な USD 関節木による順運動学（FK）と PhysX Articulation のリンク姿勢を突き合わせる自己完結検証モジュール。

責務:
  1. 結合済み USD ステージの ``UsdPhysics.Joint`` 群と剛体ツリーだけから、外部運動学ライブラリに依存せず
     順運動学木（``_KinematicTree``）を構築する（``_build_usd_kinematic_tree``）。
  2. ランダムな関節姿勢サンプルを PhysX の ``ArticulationView`` へ適用し、PhysX が計算した全リンク姿勢と
     純粋 USD 順運動学の結果を比較して最大位置誤差・DOF 一致を検証する（``check_against_physx``）。

主な設計上のポイント:
  - 浮遊ベース（ワールド固定 joint なし）では無重力下での角運動量保存によりベース自体が反動回転するため、
    ワールド絶対位置ではなくルート剛体からの相対姿勢で比較する。
  - 閉ループ機構（``excludeFromArticulation`` やサイクル辺）では PhysX が制約を解いた後の実測関節角
    ``av.get_dof_positions()`` を用いて FK を再計算し比較する。
  - ``NewtonMimicAPI`` と ``PhysxMimicJointAPI`` の符号規約の違いを吸収して mimic 関係を解決する。

主な公開 API:
  - ``check_against_physx(...)`` / ``check_kinematics_against_physx``: 結合ファイル 1 本に対する FK vs PhysX 検証

パイプライン全体での位置づけ:
  ``pipeline.run_pipeline`` の Step 8（結合ファイルごとの最終検証）で呼ばれ、
  ``report.py`` の「順運動学の検証」セクションおよび ``status`` 判定（``"ok"`` / ``"problems"``）に直結する。

手を入れやすい箇所:
  - 対応関節種別（現在は revolute / prismatic / fixed）を増やしたい場合は
    ``_build_usd_kinematic_tree`` と ``_KinematicTree.forward_kinematics_world`` を拡張する。
  - サンプル数や関節可動域のサンプリング範囲（現在は可動域の 20%〜80%）を変えたい場合は
    ``check_against_physx`` の ``n_samples`` / ``rng.uniform`` 箇所を調整する。
"""

from __future__ import annotations

import fnmatch
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from . import runtime
from . import usd_util as _u
from .descriptor import discover_components
from .sources import UsdaInfo
from .spec import JoinFileSpec, PipelineSpec

_MIMIC_PREFIX = "PhysxMimicJointAPI:"


def _all_prims():
    return getattr(_u, "ALL_PRIMS", Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))


def _gf_to_np(m: Gf.Matrix4d) -> np.ndarray:
    fn = getattr(_u, "gf_to_np", None)
    if fn is not None:
        return fn(m)
    return np.array(m, dtype=float).T


def _split_scale(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """列ベクトル形式の 4x4 を (スケール抜きの剛体変換 4x4, スケール (3,)) に分ける。"""
    scale = np.linalg.norm(matrix[:3, :3], axis=0)
    scale = np.where(scale < 1e-12, 1.0, scale)
    rigid_m = np.eye(4)
    rigid_m[:3, :3] = matrix[:3, :3] / scale
    if np.linalg.det(rigid_m[:3, :3]) < 0:
        rigid_m[:3, 0] *= -1.0
        scale = scale * np.array([-1.0, 1.0, 1.0])
    rigid_m[:3, 3] = matrix[:3, 3]
    return rigid_m, scale


def _quat_to_rot(real: float, imag: Gf.Vec3f | Sequence[float]) -> np.ndarray:
    """クォータニオン ``(w, (x, y, z))`` を列ベクトル形式の 3x3 回転行列に変換する。"""
    q = Gf.Quatd(float(real), Gf.Vec3d(float(imag[0]), float(imag[1]), float(imag[2])))
    return np.array(Gf.Matrix3d(Gf.Rotation(q)), dtype=float).T


def _pose_from_physx_row(row: Sequence[float]) -> np.ndarray:
    """PhysX ``RigidBodyView`` の 1 行 ``[x, y, z, qx, qy, qz, qw]`` を 4x4 同次変換行列に変換する。"""
    m = np.eye(4)
    m[:3, :3] = _quat_to_rot(float(row[6]), (float(row[3]), float(row[4]), float(row[5])))
    m[:3, 3] = [float(row[0]), float(row[1]), float(row[2])]
    return m


def _axis_angle_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """ロドリゲスの回転公式で軸 ``axis`` 周りの角度 ``angle`` [rad] の 3x3 回転行列を計算する。"""
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm < 1e-12:
        return np.eye(3)
    a = a / norm
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _body_of(prim: Usd.Prim | None) -> Usd.Prim | None:
    """``prim`` 自身または祖先を辿って最初に見つかる ``UsdPhysics.RigidBodyAPI`` 付き prim を返す。"""
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def _unique(name: str, used: dict[str, int]) -> str:
    """同名リンク・同名関節が衝突しないように重複時のみ連番サフィックスを付与する。"""
    if name not in used:
        used[name] = 0
        return name
    out = f"{name}_{used[name]}"
    used[name] += 1
    return out


def _limit(value: float | None, to_rad: bool, sign: float) -> float:
    """USD の関節可動域属性を内部単位（revolute は rad、prismatic は m）へ正規化する。未設定時は ±inf。"""
    if value is None or not np.isfinite(value) or abs(value) > 1e30:
        return sign * np.inf
    return float(np.deg2rad(value)) if to_rad else float(value)


def _prim_matches(path: str, pattern: str) -> bool:
    """prim パス ``path`` が構成要素パターン（前方一致または glob）に一致するか判定する。"""
    pattern = pattern.rstrip("/")
    return path == pattern or path.startswith(pattern + "/") or fnmatch.fnmatchcase(path, pattern)


def _component_of_prim(path: str, components: dict[str, str | Sequence[str]]) -> str | None:
    """最も長いパターン一致を優先して、prim ``path`` が属する構成要素名を返す。"""
    best, best_len = None, -1
    for name, spec in components.items():
        patterns = [spec] if isinstance(spec, str) else list(spec)
        for pattern in patterns:
            if _prim_matches(path, pattern) and len(pattern) > best_len:
                best, best_len = name, len(pattern)
    return best


@dataclass
class _MimicSpec:
    """従動関節（mimic follower）の参照先 leader 関節と線形関係 ``q_follower = multiplier * q_leader + offset``。"""

    reference_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class _JointNode:
    """USD 関節木における 1 本の有向辺（親剛体 -> 子剛体）。"""

    name: str
    joint_type: str  # "revolute" | "prismatic" | "fixed"
    parent_link: str
    child_link: str
    parent_frame: np.ndarray
    child_frame: np.ndarray
    axis: np.ndarray
    lower_limit: float
    upper_limit: float
    prim_path: str
    mimic: _MimicSpec | None = None

    @property
    def is_dof(self) -> bool:
        return self.joint_type != "fixed"


@dataclass
class _KinematicTree:
    """USD ステージから構築した純粋な順運動学木。``joints`` は必ず親→子（BFS 順）に並ぶ。"""

    root_link: str
    root_world_pose: np.ndarray
    link_names: list[str]
    link_paths: dict[str, str]
    joints: list[_JointNode]
    components_in_model: list[str]
    is_floating_base: bool = False
    has_closed_loop: bool = False

    @property
    def dof_joints(self) -> list[_JointNode]:
        return [j for j in self.joints if j.is_dof]

    @property
    def dof_names(self) -> list[str]:
        return [j.name for j in self.dof_joints]

    def resolve_mimic(self, q_by_name: dict[str, float]) -> dict[str, float]:
        """全 mimic 従動関節の値を参照先 leader から計算して上書きした関節値辞書を返す。

        多段 mimic チェーン（A -> B -> C）にも対応するため倍率・オフセットを合成しながら辿り、
        ``seen`` セットで循環参照による無限ループを防ぐ。
        """
        out = dict(q_by_name)
        by_name = {j.name: j for j in self.dof_joints}
        for j in self.dof_joints:
            if j.mimic is None:
                continue
            cur = j
            m_acc, o_acc = 1.0, 0.0
            seen = {cur.name}
            while cur.mimic is not None and cur.mimic.reference_joint in by_name:
                o_acc = o_acc + m_acc * cur.mimic.offset
                m_acc = m_acc * cur.mimic.multiplier
                ref_name = cur.mimic.reference_joint
                if ref_name in seen:
                    break
                seen.add(ref_name)
                cur = by_name[ref_name]
            out[j.name] = m_acc * out.get(cur.name, 0.0) + o_acc
        return out

    def forward_kinematics_world(self, q_by_name: dict[str, float]) -> dict[str, np.ndarray]:
        """関節値辞書 ``q_by_name`` から全リンクのワールド 4x4 姿勢を計算する。"""
        resolved = self.resolve_mimic(q_by_name)
        poses: dict[str, np.ndarray] = {self.root_link: self.root_world_pose.copy()}
        for j in self.joints:
            parent_pose = poses[j.parent_link]
            motion = np.eye(4)
            if j.joint_type == "revolute":
                motion[:3, :3] = _axis_angle_rot(j.axis, resolved.get(j.name, 0.0))
            elif j.joint_type == "prismatic":
                motion[:3, 3] = j.axis * resolved.get(j.name, 0.0)
            poses[j.child_link] = parent_pose @ j.parent_frame @ motion @ np.linalg.inv(j.child_frame)
        return poses


def _build_usd_kinematic_tree(stage: Usd.Stage, top: str, components: dict[str, str]) -> _KinematicTree:
    """USD ステージの ``top`` 配下から純粋 USD 順運動学木（``_KinematicTree``）を構築する。

    処理の流れ:
      1. 有効な ``UsdPhysics.Joint``（revolute / prismatic / fixed）を収集し、``excludeFromArticulation`` は
         閉ループ制約としてマークして木から外す。
      2. 各関節の ``body0`` / ``body1`` ターゲットから剛体 prim と、剛体基準のローカルジョイントフレーム
         （ターゲット側のスケール適用済み）を算出する。片側が空の関節はワールド固定剛体（``world_fixed``）として記録する。
      3. ルート剛体を「ワールド固定 > ``ArticulationRootAPI`` 持ち > 子側に出ない剛体 > 先頭」の優先度で選ぶ。
      4. ルート剛体から BFS で全域木を構築し、body1 -> body0 向きに辿った辺は ``parent_frame`` / ``child_frame`` を
         入れ替えて回転・並進軸 ``axis`` の符号を反転する。既訪問ノードに当たる辺は閉ループとして記録する。
      5. ``NewtonMimicAPI`` / ``PhysxMimicJointAPI`` の符号規約を統一して各関節へ ``_MimicSpec`` を紐付ける。
    """
    root_prim = stage.GetPrimAtPath(top)
    if not root_prim or not root_prim.IsValid():
        raise ValueError(f"prim が無い: {top}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    joint_prims = [p for p in Usd.PrimRange(root_prim, _all_prims()) if p.IsA(UsdPhysics.Joint)]

    world_fixed: list[str] = []
    has_closed_loop = False
    raw: list[tuple[Usd.Prim, str, str | None, str | None, np.ndarray, np.ndarray, np.ndarray, float, float]] = []

    for prim in joint_prims:
        if not prim.IsActive():
            continue
        joint = UsdPhysics.Joint(prim)
        en = joint.GetJointEnabledAttr().Get()
        if en is not None and not bool(en):
            continue
        if joint.GetExcludeFromArticulationAttr().Get():
            # PhysX は閉ループを切るために excludeFromArticulation=True の補助関節を使うため、木には入れず記録のみする
            has_closed_loop = True
            continue

        if prim.IsA(UsdPhysics.RevoluteJoint):
            jtype = "revolute"
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            jtype = "prismatic"
        elif prim.IsA(UsdPhysics.FixedJoint):
            jtype = "fixed"
        else:
            continue

        bodies: list[str | None] = []
        frames: list[np.ndarray] = []
        for rel, pos_attr, rot_attr in (
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
        ):
            targets = rel.GetTargets()
            target = stage.GetPrimAtPath(targets[0]) if targets else None
            body = _body_of(target) if (target and target.IsValid()) else None
            body_path = str(body.GetPath()) if body is not None else None
            bodies.append(body_path)

            # USD の localPos はターゲット prim のローカル座標（スケール含む）で定義されているため、
            # ターゲット側スケールを掛けたうえで剛体フレーム基準（スケール抜き剛体変換）へ引き直す
            t_mat = _gf_to_np(cache.GetLocalToWorldTransform(target)) if (target and target.IsValid()) else np.eye(4)
            b_mat = _gf_to_np(cache.GetLocalToWorldTransform(body)) if body is not None else np.eye(4)
            t_rigid, t_scale = _split_scale(t_mat)
            b_rigid, _ = _split_scale(b_mat)

            pos = pos_attr.Get()
            quat = rot_attr.Get()
            p_vec = np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float) if pos is not None else np.zeros(3)
            r_mat = _quat_to_rot(float(quat.GetReal()), quat.GetImaginary()) if quat is not None else np.eye(3)

            local = np.eye(4)
            local[:3, :3] = r_mat
            local[:3, 3] = t_scale * p_vec
            frames.append(np.linalg.inv(b_rigid) @ t_rigid @ local)

        if bodies[0] is None and bodies[1] is None:
            continue
        if None in bodies:
            world_fixed.append(bodies[0] or bodies[1])  # type: ignore[arg-type]
            continue

        axis_token = (
            prim.GetAttribute("physics:axis").Get()
            if jtype != "fixed" and prim.GetAttribute("physics:axis")
            else "X"
        ) or "X"
        axis = {
            "X": np.array([1.0, 0.0, 0.0]),
            "Y": np.array([0.0, 1.0, 0.0]),
            "Z": np.array([0.0, 0.0, 1.0]),
        }[axis_token]
        lower_raw = (
            prim.GetAttribute("physics:lowerLimit").Get()
            if jtype != "fixed" and prim.GetAttribute("physics:lowerLimit")
            else None
        )
        upper_raw = (
            prim.GetAttribute("physics:upperLimit").Get()
            if jtype != "fixed" and prim.GetAttribute("physics:upperLimit")
            else None
        )
        raw.append(
            (
                prim,
                jtype,
                bodies[0],
                bodies[1],
                frames[0],
                frames[1],
                axis,
                _limit(lower_raw, jtype == "revolute", -1.0),
                _limit(upper_raw, jtype == "revolute", 1.0),
            )
        )

    body_paths: list[str] = []
    for _, _, b0, b1, *_ in raw:
        for b in (b0, b1):
            if b is not None and b not in body_paths:
                body_paths.append(b)
    for b in world_fixed:
        if b not in body_paths:
            body_paths.append(b)
    if not body_paths:
        raise ValueError(f"{top} の下に剛体が見つからない")

    child_side = {b1 for _, _, b0, b1, *_ in raw if b0 is not None and b1 is not None}
    if world_fixed:
        root_body = world_fixed[0]
    else:
        with_api = [b for b in body_paths if stage.GetPrimAtPath(b).HasAPI(UsdPhysics.ArticulationRootAPI)]
        never_child = [b for b in body_paths if b not in child_side]
        root_body = with_api[0] if with_api else (never_child[0] if never_child else body_paths[0])

    root_world_pose, _ = _split_scale(_gf_to_np(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root_body))))

    used_links: dict[str, int] = {}
    used_joints: dict[str, int] = {}
    link_name_of: dict[str, str] = {}
    link_names: list[str] = []
    link_paths: dict[str, str] = {}

    def add_link(body: str) -> None:
        name = _unique(body.rsplit("/", 1)[-1], used_links)
        link_name_of[body] = name
        link_names.append(name)
        link_paths[name] = body

    add_link(root_body)
    queue: list[str] = [root_body]
    used_raw: set[int] = set()
    joints: list[_JointNode] = []
    joint_name_of_path: dict[str, str] = {}
    pending_mimic: dict[str, Usd.Prim] = {}
    incident: dict[str, list[int]] = {}
    for k, item in enumerate(raw):
        for b in dict.fromkeys((item[2], item[3])):
            if b is not None:
                incident.setdefault(b, []).append(k)

    # BFS でルート剛体から全域木を構築する
    head = 0
    while head < len(queue):
        cur = queue[head]
        head += 1
        for k in incident.get(cur, ()):
            if k in used_raw:
                continue
            prim, jtype, b0, b1, f0, f1, axis, lower, upper = raw[k]
            assert b0 is not None and b1 is not None
            other = b1 if cur == b0 else b0
            if other in link_name_of:
                used_raw.add(k)
                has_closed_loop = True
                continue
            used_raw.add(k)
            add_link(other)
            queue.append(other)
            # 関節の定義向き (body0 -> body1) と逆向きに辿る場合は、相対回転・並進の符号を合わせるため軸を反転する
            forward = cur == b0
            jname = _unique(prim.GetName(), used_joints)
            jpath = str(prim.GetPath())
            joint_name_of_path[jpath] = jname
            joints.append(
                _JointNode(
                    name=jname,
                    joint_type=jtype,
                    parent_link=link_name_of[cur],
                    child_link=link_name_of[other],
                    parent_frame=f0 if forward else f1,
                    child_frame=f1 if forward else f0,
                    axis=axis if forward else -axis,
                    lower_limit=lower,
                    upper_limit=upper,
                    prim_path=jpath,
                )
            )
            if jtype != "fixed":
                pending_mimic[jname] = prim

    # Mimic 制約の解決: スキーマごとの符号規約を q_follower = multiplier * q_leader + offset へ統一する
    type_of = {j.name: j.joint_type for j in joints}
    for j in joints:
        prim = pending_mimic.get(j.name)
        if prim is None:
            continue
        schemas = prim.GetAppliedSchemas()
        instances = [s[len(_MIMIC_PREFIX) :] for s in schemas if s.startswith(_MIMIC_PREFIX)]
        if "NewtonMimicAPI" in schemas:
            # NewtonMimicAPI の規約: q_follower = coef1 * q_leader + coef0
            enabled = prim.GetAttribute("newton:mimicEnabled").Get() if prim.GetAttribute("newton:mimicEnabled") else True
            if enabled is not None and not enabled:
                continue
            targets = prim.GetRelationship("newton:mimicJoint").GetTargets()
            coef0 = prim.GetAttribute("newton:mimicCoef0").Get() if prim.GetAttribute("newton:mimicCoef0") else None
            coef1 = prim.GetAttribute("newton:mimicCoef1").Get() if prim.GetAttribute("newton:mimicCoef1") else None
            multiplier, offset = float(1.0 if coef1 is None else coef1), float(coef0 or 0.0)
        elif instances:
            # PhysxMimicJointAPI の規約: q_follower + gearing * q_leader + offset = 0
            # よって q_follower = (-gearing) * q_leader + (-offset) と符号反転する
            inst = instances[0]
            rel = prim.GetRelationship(f"physxMimicJoint:{inst}:referenceJoint")
            targets = rel.GetTargets() if rel else []
            g_attr = prim.GetAttribute(f"physxMimicJoint:{inst}:gearing")
            o_attr = prim.GetAttribute(f"physxMimicJoint:{inst}:offset")
            gearing = g_attr.Get() if g_attr else None
            multiplier = -float(1.0 if gearing is None else gearing)
            offset = -float((o_attr.Get() if o_attr else None) or 0.0)
        else:
            continue
        if not targets or str(targets[0]) not in joint_name_of_path:
            continue
        reference = joint_name_of_path[str(targets[0])]
        if type_of.get(reference) != j.joint_type:
            continue
        if j.joint_type == "revolute":
            # USD の回転オフセットは度単位で格納されているため rad に変換する
            offset = float(np.deg2rad(offset))
        j.mimic = _MimicSpec(reference_joint=reference, multiplier=multiplier, offset=offset)

    components_in_model: list[str] = []
    for lname in link_names:
        bpath = link_paths[lname]
        best = _component_of_prim(bpath, components)
        if best is not None and best not in components_in_model:
            components_in_model.append(best)

    return _KinematicTree(
        root_link=link_names[0],
        root_world_pose=root_world_pose,
        link_names=link_names,
        link_paths=link_paths,
        joints=joints,
        components_in_model=components_in_model,
        is_floating_base=not bool(world_fixed),
        has_closed_loop=has_closed_loop,
    )


def check_against_physx(
    arg1: PipelineSpec | str | Path,
    arg2: JoinFileSpec | str | None = None,
    info: UsdaInfo | int | None = None,
    n_samples: int = 5,
    seed: int = 0,
    *,
    n_poses: int | None = None,
) -> dict[str, Any]:
    """結合済み USD を新規ステージへ読み込み、同一関節値で純粋 USD 順運動学と PhysX のリンク姿勢を比較する。

    2 通りの呼び出し形式を受け付ける:
      - ``check_against_physx(spec, f, info=None, n_samples=5, seed=0)``
      - ``check_against_physx(composed_usd, robot_name, n_poses=3, seed=0)``

    Returns:
        ``components``・``components_in_model``・``dof_names``・``dof_names_match``・
        ``is_floating_base``・``has_closed_loop``・``max_link_position_error_m``・``links_compared``
        を格納した辞書。例外発生時は ``{"error": "..."}`` を返す。

    副作用:
        ``runtime.new_probe_stage(gravity=0.0)`` でグローバルステージを作り直し、Timeline を Play / Stop する。
    """
    try:
        import omni.physics.tensors as tensors
        import omni.usd

        if isinstance(arg1, (str, Path)):
            composed_usd = str(arg1)
            if isinstance(arg2, str) and arg2:
                robot_name = arg2.lstrip("/")
            else:
                robot_name = Path(composed_usd).stem
            if isinstance(info, int):
                n_samples = info
        else:
            spec = arg1
            if isinstance(arg2, str):
                f_spec = spec.join_file(arg2)
                if f_spec is None:
                    raise ValueError(f"join_file {arg2!r} が仕様に無い")
            elif isinstance(arg2, JoinFileSpec):
                f_spec = arg2
            else:
                raise ValueError("JoinFileSpec または結合ファイル名が必要")
            composed_usd = os.path.join(spec.output_dir, f_spec.usda)
            robot_name = f_spec.stem

        if n_poses is not None:
            n_samples = n_poses

        stage = runtime.new_probe_stage(gravity=0.0)
        robot_path = f"/{robot_name}"
        stage.DefinePrim(robot_path, "Xform").GetReferences().AddReference(composed_usd)
        runtime.step(2)

        components = discover_components(stage, robot_path)
        tree = _build_usd_kinematic_tree(stage, robot_path, components)

        rng = np.random.default_rng(seed)
        dof_joints = tree.dof_joints
        # 片側のみリミットが設定されている場合でも lo < hi を保つようにサンプリング区間を決める
        lo_list: list[float] = []
        hi_list: list[float] = []
        for j in dof_joints:
            lo_ok = np.isfinite(j.lower_limit)
            hi_ok = np.isfinite(j.upper_limit)
            if lo_ok and hi_ok and j.upper_limit > j.lower_limit:
                lo_list.append(float(j.lower_limit))
                hi_list.append(float(j.upper_limit))
            elif lo_ok and not hi_ok:
                lo_list.append(float(j.lower_limit))
                hi_list.append(float(j.lower_limit) + 1.0)
            elif hi_ok and not lo_ok:
                lo_list.append(float(j.upper_limit) - 1.0)
                hi_list.append(float(j.upper_limit))
            else:
                lo_list.append(-0.5)
                hi_list.append(0.5)
        lo = np.array(lo_list, dtype=float)
        hi = np.array(hi_list, dtype=float)

        # 可動域の極端な端（リミット衝突）を避けるため、各関節の 20%〜80% の範囲で目標姿勢をサンプルする
        expected: list[tuple[dict[str, float], dict[str, np.ndarray]]] = []
        for _ in range(n_samples):
            q_vec = lo + (hi - lo) * rng.uniform(0.2, 0.8, len(lo)) if len(lo) else np.zeros(0)
            q_raw = {j.name: float(q_vec[i]) for i, j in enumerate(dof_joints)}
            q_full = tree.resolve_mimic(q_raw)
            mats = tree.forward_kinematics_world(q_full)
            expected.append((q_full, mats))

        out: dict[str, Any] = {
            "components": components,
            "components_in_model": list(tree.components_in_model),
            "dof_names": list(tree.dof_names),
        }

        link_names = sorted(tree.link_paths)
        runtime.timeline().play()
        runtime.step(3)
        sv = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
        roots = [
            str(p.GetPath())
            for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path), _all_prims())
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        av = sv.create_articulation_view(roots[0]) if roots else None
        if av is None:
            runtime.stop()
            out["error"] = "PhysX の Articulation を作れなかった"
            return out

        px_names = list(av.shared_metatype.dof_names)
        px_paths: list[str] = []
        if hasattr(av, "dof_paths"):
            try:
                raw_paths = av.dof_paths
                if raw_paths and len(raw_paths) > 0:
                    px_paths = [str(p) for p in raw_paths[0]]
            except Exception:  # noqa: BLE001
                px_paths = []
        # 同名関節が複数部品（例: 左右のアームや 3 本の車輪）に存在しても取り違えないよう、
        # prim パス優先で PhysX DOF と USD 関節木の DOF を対応付ける
        path_to_tree_name = {j.prim_path: j.name for j in dof_joints}
        tree_name_set = set(tree.dof_names)
        px_to_tree: list[str | None] = []
        for i, n in enumerate(px_names):
            tname: str | None = None
            if i < len(px_paths):
                tname = path_to_tree_name.get(px_paths[i])
            if tname is None and n in tree_name_set:
                tname = n
            px_to_tree.append(tname)

        rb = sv.create_rigid_body_view([tree.link_paths[n] for n in link_names])
        root_idx = link_names.index(tree.root_link)
        indices = np.array([0], dtype=np.uint32)
        errors: list[float] = []

        for q_full, mats in expected:
            q = np.zeros((1, len(px_names)), dtype=np.float32)
            for i, tname in enumerate(px_to_tree):
                if tname is not None and tname in q_full:
                    q[0, i] = float(q_full[tname])
            av.set_dof_positions(q, indices)
            try:
                av.set_dof_position_targets(q, indices)
            except Exception:  # noqa: BLE001
                pass
            av.set_dof_velocities(np.zeros_like(q), indices)
            runtime.step(1)
            t = rb.get_transforms()
            if tree.has_closed_loop:
                # 閉ループ機構では独立なランダム関節角 q_full がループ制約を満たさないため、
                # PhysX が 1 ステップで投影・解決した後の実測関節角で FK を計算し直す
                q_actual = [float(v) for v in av.get_dof_positions()[0]]
                q_act_by_name = dict(q_full)
                for i, val in enumerate(q_actual):
                    tname = px_to_tree[i]
                    if tname is not None:
                        q_act_by_name[tname] = val
                mats_cmp = tree.forward_kinematics_world(q_act_by_name)
            else:
                mats_cmp = mats
            if tree.is_floating_base:
                # 浮遊ベースでは無重力でも関節姿勢変更の反動でベース全体が並進・回転するため、
                # ルート剛体基準の相対変換 (inv(T_root) @ T_link) で比較する
                t_px_root_inv = np.linalg.inv(_pose_from_physx_row(t[root_idx]))
                t_fk_root_inv = np.linalg.inv(mats_cmp[tree.root_link])
                for k, n in enumerate(link_names):
                    t_px_rel = t_px_root_inv @ _pose_from_physx_row(t[k])
                    t_fk_rel = t_fk_root_inv @ mats_cmp[n]
                    errors.append(float(np.linalg.norm(t_px_rel[:3, 3] - t_fk_rel[:3, 3])))
            else:
                for k, n in enumerate(link_names):
                    errors.append(float(np.linalg.norm(t[k, :3] - mats_cmp[n][:3, 3])))

        runtime.stop()
        out["dof_names"] = list(px_names)
        tree_dof_paths = [j.prim_path for j in dof_joints]
        if px_paths:
            out["dof_paths"] = list(px_paths)
            out["dof_names_match"] = sorted(px_paths) == sorted(tree_dof_paths)
        else:
            out["dof_names_match"] = sorted(px_names) == sorted(tree.dof_names)
        out["is_floating_base"] = bool(tree.is_floating_base)
        out["has_closed_loop"] = bool(tree.has_closed_loop)
        out["max_link_position_error_m"] = float(max(errors)) if errors else 0.0
        out["links_compared"] = int(len(link_names))
        return out
    except Exception as exc:  # noqa: BLE001
        try:
            runtime.stop()
        except Exception:  # noqa: BLE001
            pass
        return {"error": f"{type(exc).__name__}: {exc}"}


check_kinematics_against_physx = check_against_physx

__all__ = [
    "check_against_physx",
    "check_kinematics_against_physx",
]
