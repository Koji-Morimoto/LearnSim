"""ロボットの内部モデル(リンクと関節の木)。numpy だけで動き、USD にも Kit にも依存しない。

関節の定義(どの生成元から作っても同じ式):

    子リンク = 親リンク × parent_frame × 動き(q) × inverse(child_frame)

    parent_frame : 親リンク座標系で見た関節座標系
    child_frame  : 子リンク座標系で見た関節座標系
    動き(q)      : revolute は axis まわりに q [rad] 回す / prismatic は axis 方向に q だけ進める / fixed は何もしない
    axis         : 関節座標系で表した単位ベクトル

DOF(自由度)は fixed 以外の関節で、並びは joints の並び順。mimic の従動軸も DOF に数える。
長さの単位はモデルを作った側の単位のまま(USD から作ればステージの単位、URDF から作ればメートル)。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, Sequence, get_args

import numpy as np

from . import transforms as tf

if TYPE_CHECKING:
    from .chain import KinematicChain

__all__ = ["JointType", "GeometricMimicSpec", "GeometricJointSpec", "GeometricLinkSpec", "GeometricRobotModel"]

JointType = Literal["revolute", "prismatic", "fixed"]


@dataclass(frozen=True)
class GeometricMimicSpec:
    """従動軸の値 = multiplier × 参照軸の値 + offset。"""

    reference_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True, eq=False)
class GeometricJointSpec:
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
        return self.joint_type != "fixed"

    def replaced(self, **changes: object) -> "GeometricJointSpec":
        values = {f: getattr(self, f) for f in self.__dataclass_fields__}
        values.update(changes)
        return GeometricJointSpec(**values)  # type: ignore[arg-type]


@dataclass(frozen=True)
class GeometricLinkSpec:
    name: str
    prim_path: str | None = None


@dataclass(frozen=True, eq=False)
class GeometricRobotModel:
    """リンクと関節の木。作った後は変更しない(変更したいときは新しいモデルを作る)。"""

    name: str
    links: tuple[GeometricLinkSpec, ...]
    joints: tuple[GeometricJointSpec, ...]
    root_link: str | None = None
    # 構成要素名 -> その構成要素に属するリンク名。結合(attach)したモデルで使う
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

        order = [link_index[root]]  # 根から幅優先
        for li in order:
            for j in children[li]:
                order.append(link_index[self.joints[j].child_link])
        if len(order) != len(self.links):
            raise ValueError("根からたどれないリンクがある")

        dof_joints = [j for j, joint in enumerate(self.joints) if joint.is_dof]
        dof_of_joint = {j: d for d, j in enumerate(dof_joints)}
        num_dofs = len(dof_joints)

        # mimic: 各 DOF を「独立な DOF の値 × 倍率 + offset」に直す
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
                # 値(cur) = multiplier × 値(ref) + offset を、これまでの倍率・offset に合成する
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

        # 各 DOF が動かすリンク(その関節の子リンク以下)
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
        object.__setattr__(self, "_cache", cache)
        known = set(link_index)
        for comp, names in self.components.items():
            missing = [n for n in names if n not in known]
            if missing:
                raise ValueError(f"構成要素 {comp!r} が存在しないリンクを含む: {missing}")

    # ------------------------------------------------------------------ 名前と個数

    @property
    def _c(self) -> dict:
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
        """mimic の従動軸ではない DOF の番号。"""
        return self._c["independent"].copy()

    @property
    def mimic_matrix(self) -> np.ndarray:
        """全 DOF の値 = mimic_matrix @ 独立な DOF の値 + mimic_offset。形は (num_dofs, 独立な DOF の数)。"""
        return self._c["mimic_matrix"].copy()

    @property
    def mimic_offset(self) -> np.ndarray:
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
        """関節で直接つながったリンクの組 [(親リンク, 子リンク, 関節名), ...]。"""
        return [(joint.parent_link, joint.child_link, joint.name) for joint in self.joints]

    # ------------------------------------------------------------------ mimic と順運動学

    def resolve_mimic(self, dof_positions: np.ndarray) -> np.ndarray:
        """従動軸の値を「multiplier × 参照軸 + offset」で置き換えた DOF の値を返す。形は入力と同じ (..., num_dofs)。"""
        q = np.asarray(dof_positions, dtype=float)
        if q.shape[-1] != self.num_dofs:
            raise ValueError(f"dof_positions の最後の次元は {self.num_dofs}: {q.shape}")
        return q[..., self._c["independent"]] @ self._c["mimic_matrix"].T + self._c["mimic_offset"]

    def _joint_motions(self, q: np.ndarray) -> np.ndarray:
        """全関節の「動き(q)」(..., num_joints, 4, 4)。fixed は単位行列。"""
        out = np.zeros(q.shape[:-1] + (self.num_joints, 4, 4))
        out[..., :, :, :] = np.eye(4)
        for d, j in enumerate(self._c["dof_joints"]):
            joint = self.joints[j]
            axis = self._c["axes"][j]
            if joint.joint_type == "revolute":
                out[..., j, :3, :3] = tf.rotation_about_axis(axis, q[..., d])
            else:
                out[..., j, :3, 3] = q[..., d, None] * axis
        return out

    def forward_kinematics(self, dof_positions: np.ndarray | None = None, *, resolve_mimic: bool = True) -> np.ndarray:
        """根リンク座標系で見た全リンクの姿勢 (..., num_links, 4, 4)。

        dof_positions は (..., num_dofs)。先頭の次元の分だけまとめて計算する。None なら全部 0。
        """
        q = np.zeros(self.num_dofs) if dof_positions is None else np.asarray(dof_positions, dtype=float)
        if resolve_mimic:
            q = self.resolve_mimic(q)
        motions = self._joint_motions(q)
        c = self._c
        out = np.zeros(q.shape[:-1] + (self.num_links, 4, 4))
        order = c["order"]
        out[..., order[0], :, :] = np.eye(4)
        for li in order[1:]:
            j = c["parent_joint"][li]
            pi = c["link_index"][self.joints[j].parent_link]
            out[..., li, :, :] = out[..., pi, :, :] @ c["parent_frames"][j] @ motions[..., j, :, :] @ c["child_frames_inv"][j]
        return out

    def jacobian(self, dof_positions: np.ndarray | None = None, *, reduced: bool = False) -> np.ndarray:
        """全リンクのヤコビアン (..., num_links, 6, num_dofs)。根リンク座標系。

        行は [リンク原点の速度 3 行; 角速度 3 行]。reduced=True なら mimic を畳んで、列を独立な DOF だけにする。
        入力の従動軸の値は mimic で置き換えてから計算する。
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
        lin = np.cross(axes[..., None, :, :], diff) * is_rev[:, None] + axes[..., None, :, :] * (1.0 - is_rev)[:, None]
        ang = np.broadcast_to(axes[..., None, :, :] * is_rev[:, None], lin.shape)
        jac = np.concatenate([lin, ang], axis=-1) * c["descends"][..., None]  # (..., L, D, 6)
        jac = np.swapaxes(jac, -1, -2)
        return jac @ c["mimic_matrix"] if reduced else jac

    # ------------------------------------------------------------------ 経路と直列の鎖

    def path_between(self, base_link: str, tip_link: str) -> list[tuple[int, bool]]:
        """base から tip までにたどる関節 [(関節番号, 順方向か), ...]。順方向 = 親リンクから子リンクへたどる。"""
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
        while up and down and up[-1] == down[-1]:  # 共通の祖先より上は打ち消す
            up.pop()
            down.pop()
        return [(j, False) for j in up] + [(j, True) for j in reversed(down)]

    def get_kinematic_chain(self, base_link: str, tip_link: str, tip_offset: np.ndarray | None = None) -> "KinematicChain":
        """base_link から tip_link までの直列部分を取り出す。tip_offset は tip リンク座標系で見た手先の姿勢(4x4)。"""
        from .chain import KinematicChain

        return KinematicChain.from_model(self, base_link, tip_link, tip_offset)

    # ------------------------------------------------------------------ 結合

    def with_prefix(self, prefix: str) -> "GeometricRobotModel":
        """リンク名・関節名の前に prefix を付けたモデル(例: prefix="left/" なら joint1 -> left/joint1)。"""

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
        """other を parent_link に固定で取り付けた新しいモデルを返す(self も other も変えない)。

        other のリンク名・関節名には "<name>/" を前に付ける。取り付けの固定関節は "<name>/mount"。
        mount_transform は parent_link 座標系で見た other の根リンクの姿勢(4x4)。
        self がまだ構成要素を持たないとき、self のリンクは構成要素 base_name(省略時は self.name)になる。
        同じロボットを何台も付けるときは、name を変えて attach を繰り返す。
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

    # ------------------------------------------------------------------ 比較と書き出し

    def is_kinematically_equal(self, other: "GeometricRobotModel", tol: float = 1e-9) -> bool:
        """リンク・関節の並びと、関節の位置・軸・種類・mimic が同じか(名前と prim パスは比べない)。"""
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
        """運動学だけの URDF 文字列を返す。path を渡すとファイルにも書く。詳細は urdf.model_to_urdf。"""
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
    """直列ロボット用の補助。link_names[0] が根。"""
    return GeometricRobotModel(name=name, links=tuple(GeometricLinkSpec(n) for n in link_names), joints=tuple(joint_specs), root_link=link_names[0])
