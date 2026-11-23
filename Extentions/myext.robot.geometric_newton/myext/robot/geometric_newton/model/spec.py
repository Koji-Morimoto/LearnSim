"""ロボットの純粋運動学モデル (`GeometricRobotModel`) とリンク・関節・mimic 仕様 (NumPy のみ)。

USD / Kit / Newton に依存しない不変 (`frozen=True`) の木構造モデルであり、USD Articulation・DH・
修正 DH・スクリュー軸・URDF のすべてがこの単一表現へ変換される。

統一された関節変換式 (列ベクトル形式 4x4 同次変換行列):
    `X_wc = X_wp @ parent_frame @ motion(q) @ inv(child_frame)`
    - `parent_frame` : 親リンク座標系で見た関節座標系 (4x4)
    - `child_frame`  : 子リンク座標系で見た関節座標系 (4x4)
    - `motion(q)`    : `revolute` は `axis` まわりに `q` [rad] 回転、`prismatic` は `axis` 方向に `q` 並進、`fixed` は単位行列
    - `axis`         : 関節座標系で表した単位ベクトル `(3,)`
    ※ この式は `newton.eval_fk` の `X_wc = (X_wp * X_pj * X_j) * inv(X_cj)` と数学的に完全一致するため、
      `newton_backend.py` が `GeometricRobotModel` から直接 `newton.ModelBuilder` を構築できる。

DOF・mimic・単位の規約:
    - DOF は `fixed` 以外の関節 (`joints` の出現順)。mimic の従動軸も DOF 配列の要素として保持する。
    - 連鎖する mimic (例: `J3 -> J2 -> J1`) は構築時に `mimic_matrix` / `mimic_offset` へ展開され、
      `resolve_mimic(q)` が 1 回の行列積 `q_indep @ mimic_matrix.T + mimic_offset` で厳密に解決する。
    - 角度は常にラジアン (`rad`)。長さの単位は生成元の単位をそのまま保持する。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, Sequence, get_args

import numpy as np

from . import transforms as tf

if TYPE_CHECKING:
    from .chain import KinematicChain

__all__ = ["JointType", "GeometricMimicSpec", "GeometricJointSpec", "GeometricLinkSpec", "GeometricRobotModel"]

# ==============================================================================
# 仕様データクラス (Mimic / Joint / Link)
# ==============================================================================

JointType = Literal["revolute", "prismatic", "fixed"]


@dataclass(frozen=True)
class GeometricMimicSpec:
    """従動軸の線形拘束定義: `q_follower = multiplier * q_reference + offset`。"""

    reference_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True, eq=False)
class GeometricJointSpec:
    """1 本の関節の運動学仕様 (不変)。構築時に 4x4 形状チェック・読み取り専用化・`axis` 単位ベクトル化を行う。"""

    name: str
    joint_type: JointType
    parent_link: str
    child_link: str
    parent_frame: np.ndarray = field(default_factory=tf.identity)
    child_frame: np.ndarray = field(default_factory=tf.identity)
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    lower_limit: float = -np.inf
    upper_limit: float = np.inf
    mimic: GeometricMimicSpec | None = None
    prim_path: str | None = None

    def __post_init__(self) -> None:
        if self.joint_type not in get_args(JointType):
            raise ValueError(f"joint_type は {get_args(JointType)} のどれか: {self.joint_type!r} (関節 {self.name})")
        for key in ("parent_frame", "child_frame"):
            value = np.array(getattr(self, key), dtype=float)
            if value.shape != (4, 4):
                raise ValueError(f"{key} は 4x4 行列: 関節 {self.name}")
            # モデルの不変性を守るため NumPy 配列を書き込み禁止にする
            value.setflags(write=False)
            object.__setattr__(self, key, value)
        axis = np.asarray(self.axis, dtype=float)
        norm = np.linalg.norm(axis)
        if self.joint_type != "fixed" and norm < 1e-12:
            raise ValueError(f"axis の長さが 0: 関節 {self.name}")
        if norm >= 1e-12:
            axis = axis / norm
        object.__setattr__(self, "axis", tuple(float(v) for v in axis))
        if self.joint_type == "fixed" and self.mimic is not None:
            raise ValueError(f"fixed 関節に mimic は付けられない: 関節 {self.name}")

    @property
    def is_dof(self) -> bool:
        """`fixed` 以外の可動関節なら `True`。"""
        return self.joint_type != "fixed"

    def replaced(self, **changes: object) -> "GeometricJointSpec":
        """指定フィールドだけ差し替えた新しい `GeometricJointSpec` を返す。"""
        values = {f: getattr(self, f) for f in self.__dataclass_fields__}
        values.update(changes)
        return GeometricJointSpec(**values)  # type: ignore[arg-type]


@dataclass(frozen=True)
class GeometricLinkSpec:
    """1 つのリンクの識別仕様 (`name` と任意の USD `prim_path`)。"""

    name: str
    prim_path: str | None = None


# ==============================================================================
# GeometricRobotModel 本体 (木構造検証・前計算キャッシュ・ベクトル化 FK / ヤコビアン・attach)
# ==============================================================================


@dataclass(frozen=True, eq=False)
class GeometricRobotModel:
    """リンクと関節から成る不変の木構造モデル (変更時は `attach()` 等で新インスタンスを作る)。

    構築時の `__post_init__` で次の項目を事前検証・前計算して `_cache` (`_c`) に保持する:
      - 木構造検証 (リンク/関節名の重複禁止、閉ループ禁止、全リンクの根からの到達性)
      - 幅優先順 (`order`) と木の深さごとのグループ (`fk_levels`) -> バッチ FK の Python ループを深さ回数へ圧縮
      - 連鎖 mimic の線形写像展開 (`independent`, `mimic_matrix`, `mimic_offset`)
      - 各回転関節の歪対称行列 `K`, `K^2` と `inv(child_frame)` の事前配列化
      - ヤコビアン用の子孫マスク `descends (L, D)`
    """

    name: str
    links: tuple[GeometricLinkSpec, ...]
    joints: tuple[GeometricJointSpec, ...]
    root_link: str | None = None
    # 構成要素名 -> その構成要素に属するリンク名タプル (`attach()` 結合や部位別干渉判定で使用)
    components: dict[str, tuple[str, ...]] = field(default_factory=dict)

    # ------------------------------------------------------------------ 構築時の検査と前計算

    def __post_init__(self) -> None:
        object.__setattr__(self, "links", tuple(self.links))
        object.__setattr__(self, "joints", tuple(self.joints))
        link_index = {link.name: i for i, link in enumerate(self.links)}
        joint_index = {joint.name: i for i, joint in enumerate(self.joints)}
        if len(link_index) != len(self.links):
            raise ValueError("リンク名が重複している")
        if len(joint_index) != len(self.joints):
            raise ValueError("関節名が重複している")

        parent_joint = [-1] * len(self.links)
        children: list[list[int]] = [[] for _ in self.links]
        for j, joint in enumerate(self.joints):
            for key in (joint.parent_link, joint.child_link):
                if key not in link_index:
                    raise ValueError(f"関節 {joint.name} が存在しないリンク {key!r} を指している")
            child = link_index[joint.child_link]
            if parent_joint[child] != -1:
                raise ValueError(f"リンク {joint.child_link} に親の関節が 2 つある(閉じた輪は扱えない)")
            parent_joint[child] = j
            children[link_index[joint.parent_link]].append(j)

        roots = [link.name for i, link in enumerate(self.links) if parent_joint[i] == -1]
        root = self.root_link if self.root_link is not None else (roots[0] if len(roots) == 1 else None)
        if root is None or roots != [root]:
            raise ValueError(f"根のリンクが 1 つに決まらない: 親の関節を持たないリンク = {roots}, root_link = {self.root_link!r}")
        object.__setattr__(self, "root_link", root)

        # 根から幅優先 (BFS) でリンク順を決定する (親が必ず子より前に現れるためトポロジカル順序になる)
        order = [link_index[root]]
        for li in order:
            for j in children[li]:
                order.append(link_index[self.joints[j].child_link])
        if len(order) != len(self.links):
            raise ValueError("根からたどれないリンクがある")

        dof_joints = [j for j, joint in enumerate(self.joints) if joint.is_dof]
        dof_of_joint = {j: d for d, j in enumerate(dof_joints)}
        num_dofs = len(dof_joints)

        # 連鎖 mimic の展開:
        #   `q_cur = m * q_ref + o` を独立 DOF までたどって合成し、
        #   全 DOF ベクトル `q = mimic_matrix @ q_indep + mimic_offset` の 1 回の線形変換へ畳み込む。
        #   循環参照 (`A -> B -> A`) は `seen` 集合で検出して構築時に例外を送出する。
        root_dof = list(range(num_dofs))
        mult = np.ones(num_dofs)
        offs = np.zeros(num_dofs)
        for d, j in enumerate(dof_joints):
            cur, m_acc, o_acc, seen = d, 1.0, 0.0, {d}
            while self.joints[dof_joints[cur]].mimic is not None:
                mimic = self.joints[dof_joints[cur]].mimic
                assert mimic is not None
                if mimic.reference_joint not in joint_index or joint_index[mimic.reference_joint] not in dof_of_joint:
                    raise ValueError(f"関節 {self.joints[dof_joints[cur]].name} の mimic の参照先 {mimic.reference_joint!r} が DOF ではない")
                # 既存の累積変換 `m_acc * q_cur + o_acc` に `q_cur = multiplier * q_ref + offset` を代入する
                o_acc = o_acc + m_acc * mimic.offset
                m_acc = m_acc * mimic.multiplier
                cur = dof_of_joint[joint_index[mimic.reference_joint]]
                if cur in seen:
                    raise ValueError(f"mimic の参照が輪になっている: 関節 {self.joints[j].name}")
                seen.add(cur)
            root_dof[d], mult[d], offs[d] = cur, m_acc, o_acc
        independent = [d for d in range(num_dofs) if root_dof[d] == d]
        col = {d: i for i, d in enumerate(independent)}
        mimic_matrix = np.zeros((num_dofs, len(independent)))
        for d in range(num_dofs):
            mimic_matrix[d, col[root_dof[d]]] = mult[d]

        # ヤコビアン用子孫マスク: `descends[li, d] == True` なら DOF `d` はリンク `li` の姿勢に影響する
        descends = np.zeros((len(self.links), num_dofs), dtype=bool)
        for li in order[1:]:
            pj = parent_joint[li]
            descends[li] = descends[link_index[self.joints[pj].parent_link]]
            if pj in dof_of_joint:
                descends[li, dof_of_joint[pj]] = True

        cache = {
            "link_index": link_index,
            "joint_index": joint_index,
            "parent_joint": parent_joint,
            "order": order,
            "dof_joints": dof_joints,
            "dof_of_joint": dof_of_joint,
            "independent": np.array(independent, dtype=int),
            "mimic_matrix": mimic_matrix,
            "mimic_offset": offs,
            "descends": descends,
            "parent_frames": np.array([j.parent_frame for j in self.joints]).reshape(-1, 4, 4),
            "child_frames_inv": np.array([tf.invert(j.child_frame) for j in self.joints]).reshape(-1, 4, 4),
            "axes": np.array([j.axis for j in self.joints]).reshape(-1, 3),
        }
        cache.update(self._precompute_fk(cache, link_index, parent_joint, order, dof_joints))
        object.__setattr__(self, "_cache", cache)
        known = set(link_index)
        for comp, names in self.components.items():
            missing = [n for n in names if n not in known]
            if missing:
                raise ValueError(f"構成要素 {comp!r} が存在しないリンクを含む: {missing}")

    # ------------------------------------------------------------------ 名前・個数・トポロジ参照

    @property
    def _c(self) -> dict:
        """前計算キャッシュ辞書への内部ショートカット (`newton_backend.py` / `chain.py` 等も参照)。"""
        return self._cache  # type: ignore[attr-defined]

    @property
    def num_links(self) -> int:
        return len(self.links)

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    @property
    def num_dofs(self) -> int:
        return len(self._c["dof_joints"])

    @property
    def link_names(self) -> list[str]:
        return [link.name for link in self.links]

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]

    @property
    def joint_types(self) -> list[JointType]:
        return [joint.joint_type for joint in self.joints]

    @property
    def dof_joints(self) -> list[GeometricJointSpec]:
        return [self.joints[j] for j in self._c["dof_joints"]]

    @property
    def dof_names(self) -> list[str]:
        return [joint.name for joint in self.dof_joints]

    @property
    def dof_types(self) -> list[JointType]:
        return [joint.joint_type for joint in self.dof_joints]

    @property
    def dof_lower_limits(self) -> np.ndarray:
        return np.array([joint.lower_limit for joint in self.dof_joints], dtype=float)

    @property
    def dof_upper_limits(self) -> np.ndarray:
        return np.array([joint.upper_limit for joint in self.dof_joints], dtype=float)

    @property
    def independent_dof_indices(self) -> np.ndarray:
        """mimic の従動軸ではない独立 DOF のインデックス配列。"""
        return self._c["independent"].copy()

    @property
    def mimic_matrix(self) -> np.ndarray:
        """`q_all = mimic_matrix @ q_indep + mimic_offset` を表す行列 `(num_dofs, num_independent)`。"""
        return self._c["mimic_matrix"].copy()

    @property
    def mimic_offset(self) -> np.ndarray:
        """`q_all = mimic_matrix @ q_indep + mimic_offset` の定数オフセット `(num_dofs,)`。"""
        return self._c["mimic_offset"].copy()

    def get_link_index(self, name: str) -> int:
        try:
            return self._c["link_index"][name]
        except KeyError:
            raise KeyError(f"リンク {name!r} は無い。ある名前: {self.link_names}") from None

    def get_joint_index(self, name: str) -> int:
        try:
            return self._c["joint_index"][name]
        except KeyError:
            raise KeyError(f"関節 {name!r} は無い。ある名前: {self.joint_names}") from None

    def get_dof_index(self, name: str) -> int:
        j = self.get_joint_index(name)
        if j not in self._c["dof_of_joint"]:
            raise KeyError(f"関節 {name!r} は fixed なので DOF ではない")
        return self._c["dof_of_joint"][j]

    def parent_joint_of(self, link: str) -> GeometricJointSpec | None:
        j = self._c["parent_joint"][self.get_link_index(link)]
        return None if j == -1 else self.joints[j]

    def child_joints_of(self, link: str) -> list[GeometricJointSpec]:
        return [joint for joint in self.joints if joint.parent_link == link]

    def component_of_link(self, link: str) -> str | None:
        for comp, names in self.components.items():
            if link in names:
                return comp
        return None

    def adjacent_link_pairs(self) -> list[tuple[str, str, str]]:
        """関節で直接つながったリンクの組 `[(親リンク名, 子リンク名, 関節名), ...]` を返す。"""
        return [(joint.parent_link, joint.child_link, joint.name) for joint in self.joints]

    # ------------------------------------------------------------------ mimic 解決・ベクトル化 FK・ヤコビアン

    def resolve_mimic(self, dof_positions: np.ndarray) -> np.ndarray:
        """従動軸の値を `multiplier * 参照軸 + offset` で上書きした DOF 配列 `(..., num_dofs)` を返す。

        ※ `newton.eval_fk` は `model.constraint_mimic` を評価しないため、Newton バックエンドへ渡す前にも
          このメソッドで従動軸を解決しておく必要がある。
        """
        q = np.asarray(dof_positions, dtype=float)
        if q.shape[-1] != self.num_dofs:
            raise ValueError(f"dof_positions の最後の次元は {self.num_dofs}: {q.shape}")
        return q[..., self._c["independent"]] @ self._c["mimic_matrix"].T + self._c["mimic_offset"]

    def _precompute_fk(self, cache: dict, link_index: dict, parent_joint: list, order: list, dof_joints: list) -> dict:
        """深さレベル別ベクトル化 FK の前計算を行う。

        設計上の意図:
            1. 全回転関節の歪対称行列 `K` と `K^2` を事前構築し、Rodrigues の式
               `R(q) = I + sin(q) K + (1 - cos(q)) K^2` を 1 回の NumPy 演算で全回転関節・全バッチ分まとめて求める。
            2. `fixed` 関節や `q = 0` 関節の既定局所変換 `parent_frame @ inv(child_frame)` を `fk_fixed_local` にキャッシュする。
            3. 同じ木の深さにあるリンク同士は互いに依存しないため `fk_levels` へ束ね、
               Python ループ回数を「リンク数」ではなく「木の深さ」へ削減する (双腕・多指ハンドで特に効く)。
        """
        axes = cache["axes"]
        rev = [(d, j) for d, j in enumerate(dof_joints) if self.joints[j].joint_type == "revolute"]
        pri = [(d, j) for d, j in enumerate(dof_joints) if self.joints[j].joint_type == "prismatic"]
        rev_axes = axes[[j for _, j in rev]] if rev else np.zeros((0, 3))
        skew = np.zeros((len(rev), 3, 3))
        skew[:, 0, 1], skew[:, 0, 2], skew[:, 1, 2] = -rev_axes[:, 2], rev_axes[:, 1], -rev_axes[:, 0]
        skew[:, 1, 0], skew[:, 2, 0], skew[:, 2, 1] = rev_axes[:, 2], -rev_axes[:, 1], rev_axes[:, 0]
        depth = {order[0]: 0}
        groups: dict[int, list[tuple[int, int, int]]] = {}
        for li in order[1:]:
            j = parent_joint[li]
            pi = link_index[self.joints[j].parent_link]
            depth[li] = depth[pi] + 1
            groups.setdefault(depth[li], []).append((li, pi, j))
        levels = [tuple(np.array(col, dtype=int) for col in zip(*groups[k])) for k in sorted(groups)]
        parent_frames, child_inv = cache["parent_frames"], cache["child_frames_inv"]
        return {
            "fk_fixed_local": parent_frames @ child_inv,  # `motion(q) = I` の局所変換 (`fixed` 関節のベース配列)
            "fk_rev_dofs": np.array([d for d, _ in rev], dtype=int),
            "fk_rev_joints": np.array([j for _, j in rev], dtype=int),
            "fk_rev_skew": skew,
            "fk_rev_skew2": skew @ skew,
            "fk_pri_dofs": np.array([d for d, _ in pri], dtype=int),
            "fk_pri_joints": np.array([j for _, j in pri], dtype=int),
            "fk_pri_axes": axes[[j for _, j in pri]] if pri else np.zeros((0, 3)),
            "fk_levels": levels,  # 深さごとの `(子リンク配列, 親リンク配列, 関節配列)` タプル列
        }

    def joint_local_transforms(self, q: np.ndarray) -> np.ndarray:
        """全関節の局所変換 `parent_frame @ motion(q) @ inv(child_frame)` (`(..., num_joints, 4, 4)`) を一括計算する (`q` は mimic 解決済み)。"""
        c = self._c
        batch = q.shape[:-1]
        local = np.broadcast_to(c["fk_fixed_local"], batch + c["fk_fixed_local"].shape).copy()
        rj = c["fk_rev_joints"]
        if len(rj):
            angle = q[..., c["fk_rev_dofs"]]
            motion = np.zeros(angle.shape + (4, 4))
            motion[..., :3, :3] = np.eye(3) + np.sin(angle)[..., None, None] * c["fk_rev_skew"] + (1.0 - np.cos(angle))[..., None, None] * c["fk_rev_skew2"]
            motion[..., 3, 3] = 1.0
            local[..., rj, :, :] = c["parent_frames"][rj] @ motion @ c["child_frames_inv"][rj]
        pj = c["fk_pri_joints"]
        if len(pj):
            motion = np.broadcast_to(np.eye(4), batch + (len(pj), 4, 4)).copy()
            motion[..., :3, 3] = q[..., c["fk_pri_dofs"], None] * c["fk_pri_axes"]
            local[..., pj, :, :] = c["parent_frames"][pj] @ motion @ c["child_frames_inv"][pj]
        return local

    def forward_kinematics(self, dof_positions: np.ndarray | None = None, *, resolve_mimic: bool = True) -> np.ndarray:
        """根リンク座標系で見た全リンクの 4x4 姿勢 `(..., num_links, 4, 4)` を深さレベル単位のバッチ行列積で計算する。

        `dof_positions` は `(..., num_dofs)` (`None` なら全 0)。先頭の任意バッチ次元をそのまま一括処理する。
        """
        q = np.zeros(self.num_dofs) if dof_positions is None else np.asarray(dof_positions, dtype=float)
        if resolve_mimic:
            q = self.resolve_mimic(q)
        c = self._c
        local = self.joint_local_transforms(q)
        out = np.empty(q.shape[:-1] + (self.num_links, 4, 4))
        out[..., c["order"][0], :, :] = np.eye(4)
        for child, parent, joint in c["fk_levels"]:
            out[..., child, :, :] = out[..., parent, :, :] @ local[..., joint, :, :]
        return out

    def forward_kinematics_reference(self, dof_positions: np.ndarray | None = None) -> np.ndarray:
        """テスト・検算用の素朴な逐次 FK 実装 (リンク 1 個ずつ `X_wp @ parent_frame @ motion(q) @ inv(child_frame)` を計算)。"""
        q = self.resolve_mimic(np.zeros(self.num_dofs) if dof_positions is None else np.asarray(dof_positions, dtype=float))
        c = self._c
        out = np.zeros(q.shape[:-1] + (self.num_links, 4, 4))
        out[..., c["order"][0], :, :] = np.eye(4)
        for li in c["order"][1:]:
            j = c["parent_joint"][li]
            joint = self.joints[j]
            motion = np.broadcast_to(np.eye(4), q.shape[:-1] + (4, 4)).copy()
            if joint.is_dof:
                d = c["dof_of_joint"][j]
                if joint.joint_type == "revolute":
                    motion[..., :3, :3] = tf.rotation_about_axis(c["axes"][j], q[..., d])
                else:
                    motion[..., :3, 3] = q[..., d, None] * c["axes"][j]
            pi = c["link_index"][joint.parent_link]
            out[..., li, :, :] = out[..., pi, :, :] @ c["parent_frames"][j] @ motion @ c["child_frames_inv"][j]
        return out

    def jacobian(self, dof_positions: np.ndarray | None = None, *, reduced: bool = False) -> np.ndarray:
        """根リンク座標系における全リンクの幾何ヤコビアン `(..., num_links, 6, num_dofs)` を解析的に計算する。

        行順は `[線速度 3 行; 角速度 3 行]`。
        `reduced=True` のときは右から `mimic_matrix` を掛けることで、連鎖則により
        `J_ref += multiplier * J_follower` として従動軸を独立 DOF 列へ畳み込む。
        """
        q = np.zeros(self.num_dofs) if dof_positions is None else np.asarray(dof_positions, dtype=float)
        q = self.resolve_mimic(q)
        c = self._c
        poses = self.forward_kinematics(q, resolve_mimic=False)
        dof_joints = c["dof_joints"]
        parents = [c["link_index"][self.joints[j].parent_link] for j in dof_joints]
        joint_frames = poses[..., parents, :, :] @ c["parent_frames"][dof_joints]  # (..., D, 4, 4)
        axes = np.einsum("...dij,dj->...di", joint_frames[..., :3, :3], c["axes"][dof_joints])  # (..., D, 3)
        origins = joint_frames[..., :3, 3]
        is_rev = np.array([self.joints[j].joint_type == "revolute" for j in dof_joints], dtype=float)
        diff = poses[..., :, None, :3, 3] - origins[..., None, :, :]  # (..., L, D, 3)
        # 回転関節: `v = axis x (p_link - p_joint)`, `w = axis` / 直動関節: `v = axis`, `w = 0`
        lin = np.cross(axes[..., None, :, :], diff) * is_rev[:, None] + axes[..., None, :, :] * (1.0 - is_rev)[:, None]
        ang = np.broadcast_to(axes[..., None, :, :] * is_rev[:, None], lin.shape)
        jac = np.concatenate([lin, ang], axis=-1) * c["descends"][..., None]  # (..., L, D, 6)
        jac = np.swapaxes(jac, -1, -2)
        return jac @ c["mimic_matrix"] if reduced else jac

    # ------------------------------------------------------------------ 経路探索と直列鎖の抽出

    def path_between(self, base_link: str, tip_link: str) -> list[tuple[int, bool]]:
        """`base_link` から `tip_link` までの最短経路を `[(joint_index, forward), ...]` で返す。

        最小共通祖先 (LCA) より上の共通経路を打ち消すことで、根を通らない 2 リンク間 (例: 左手先 -> 右手先)
        についても、`base_link` 側は逆向き (`forward=False`)、`tip_link` 側は順向き (`forward=True`) の鎖を得られる。
        """
        c = self._c

        def to_root(link: str) -> list[int]:
            out = []
            li = self.get_link_index(link)
            while c["parent_joint"][li] != -1:
                j = c["parent_joint"][li]
                out.append(j)
                li = c["link_index"][self.joints[j].parent_link]
            return out

        up, down = to_root(base_link), to_root(tip_link)
        while up and down and up[-1] == down[-1]:  # 共通祖先より根側の経路を相殺する
            up.pop()
            down.pop()
        return [(j, False) for j in up] + [(j, True) for j in reversed(down)]

    def get_kinematic_chain(self, base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> "KinematicChain":
        """`base_link` -> `tip_link` の直列部分 (`KinematicChain`) を切り出す (`tip_offset` は `tip_link` ローカルの 4x4)。"""
        from .chain import KinematicChain

        return KinematicChain.from_model(self, base_link, tip_link, tip_offset)

    # ------------------------------------------------------------------ 名前空間付与とモデル結合 (attach)

    def with_prefix(self, prefix: str) -> "GeometricRobotModel":
        """全リンク名・関節名・mimic 参照先・構成要素リンク名へ `prefix` (例: `"left/"`) を前置した新モデルを返す。"""

        def rename_joint(joint: GeometricJointSpec) -> GeometricJointSpec:
            mimic = joint.mimic
            if mimic is not None:
                mimic = GeometricMimicSpec(prefix + mimic.reference_joint, mimic.multiplier, mimic.offset)
            return joint.replaced(name=prefix + joint.name, parent_link=prefix + joint.parent_link, child_link=prefix + joint.child_link, mimic=mimic)

        return GeometricRobotModel(
            name=self.name,
            links=tuple(GeometricLinkSpec(prefix + link.name, link.prim_path) for link in self.links),
            joints=tuple(rename_joint(joint) for joint in self.joints),
            root_link=prefix + (self.root_link or ""),
            components={comp: tuple(prefix + n for n in names) for comp, names in self.components.items()},
        )

    def attach(
        self,
        other: "GeometricRobotModel",
        parent_link: str,
        mount_transform: np.ndarray | None = None,
        *,
        name: str,
        base_name: str | None = None,
    ) -> "GeometricRobotModel":
        """`other` を `parent_link` へ固定関節 `f"{name}/mount"` で結合した新しいモデルを返す (`self` / `other` 非破壊)。

        設計上の意図:
            - `other` 側の全リンク・関節へ `f"{name}/"` プレフィックスを付けることで、同じアームやグリッパを
              2 台以上結合しても名前衝突を起こさない。
            - `components` 辞書へ親側 (`base_name` または `self.name`) と子側 (`name`) のリンク集合を記録するため、
              結合後も `within_component` (部品内自己干渉) と `between_components` (部品間干渉) を自動判別できる。
            - 干渉フィルタ (`GeometricCollisionFilter`) 側の結合は `GeometricCollisionFilter.merged_with(...)` と
              対になるプレフィックス規約 (`other_prefix=f"{name}/"`) で整合させている。
        """
        if name in self.components or name == (base_name or self.name):
            raise ValueError(f"構成要素名 {name!r} はすでに使われている")
        self.get_link_index(parent_link)
        added = other.with_prefix(name + "/")
        mount = GeometricJointSpec(
            name=f"{name}/mount",
            joint_type="fixed",
            parent_link=parent_link,
            child_link=added.root_link or "",
            parent_frame=tf.identity() if mount_transform is None else mount_transform,
        )
        components = dict(self.components) if self.components else {base_name or self.name: tuple(self.link_names)}
        if added.components:
            components.update(added.components)
        else:
            components[name] = tuple(added.link_names)
        return GeometricRobotModel(
            name=self.name,
            links=self.links + added.links,
            joints=self.joints + (mount,) + added.joints,
            root_link=self.root_link,
            components=components,
        )

    # ------------------------------------------------------------------ 比較と URDF 出力

    def is_kinematically_equal(self, other: "GeometricRobotModel", tol: float = 1e-9) -> bool:
        """トポロジ・関節種類・フレーム・軸・mimic が許容誤差 `tol` 以内で一致するか判定する (名前と `prim_path` は不問)。"""
        if self.num_links != other.num_links or self.num_joints != other.num_joints:
            return False
        a, b = self._c, other._c
        if a["parent_joint"] != b["parent_joint"] or a["order"] != b["order"] or self.joint_types != other.joint_types:
            return False
        for ja, jb in zip(self.joints, other.joints):
            if self.get_link_index(ja.parent_link) != other.get_link_index(jb.parent_link):
                return False
        return bool(
            np.allclose(a["parent_frames"], b["parent_frames"], atol=tol)
            and np.allclose(a["child_frames_inv"], b["child_frames_inv"], atol=tol)
            and np.allclose(a["axes"], b["axes"], atol=tol)
            and np.allclose(a["mimic_matrix"], b["mimic_matrix"], atol=tol)
            and np.allclose(a["mimic_offset"], b["mimic_offset"], atol=tol)
        )

    def to_urdf(self, path: str | None = None, *, robot_name: str | None = None) -> str:
        """運動学専用 URDF 文字列を返す (`path` 指定時はファイルにも保存)。変換規約は `urdf.model_to_urdf` を参照。"""
        from .urdf import model_to_urdf

        text = model_to_urdf(self, robot_name=robot_name)
        if path is not None:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        return text


def serial_model(
    name: str,
    joint_specs: Sequence[GeometricJointSpec],
    link_names: Sequence[str],
) -> GeometricRobotModel:
    """直列ロボット用の簡易コンストラクタ (`link_names[0]` を根リンクにする)。"""
    return GeometricRobotModel(name=name, links=tuple(GeometricLinkSpec(n) for n in link_names), joints=tuple(joint_specs), root_link=link_names[0])
