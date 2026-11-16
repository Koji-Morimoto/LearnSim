"""干渉チェック。

「自分自身との干渉」に固定の意味を持たせず、必ず「どのリンク集合の内側か / どの集合とどの集合の間か」を指定する。

    checker.check_within(robot.link_set)                         ロボットの全リンク同士
    checker.check_within(robot.get_component("left").link_set)   左アームのリンク同士(結合後も意味は変わらない)
    checker.check_between(a.link_set, b.link_set)                2 つの集合の間
    checker.check_environment(robot.link_set)                    ロボット以外のコライダーとの間

仕組み: 調べる側のコライダー 1 個ずつを PhysX のシーンクエリ overlap_shape に渡し、重なった相手を受け取る。
組の振り分けと除外(GeometricCollisionFilter)は Python 側で行う。CollisionGroup など PhysX 側の設定には頼らない。
メッシュのコライダーは、調べる側だけ凸包で近似される(overlap_shape の仕様)。

static(既定): Stop でも Play でも、関節値を変えた直後から結果に反映される(チェックの前に姿勢を USD に書く)。
kinematic   : Play 中だけ使える。書いた姿勢が PhysX に入るのは次の update なので、結果は 1 update 遅れる。
              Play 中に接触の始まり・終わりを受け取るなら GeometricContactMonitor を使う。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Literal, Sequence

import carb
import omni.timeline
from omni.physx import get_physx_interface, get_physx_scene_query_interface, get_physx_simulation_interface
from pxr import PhysicsSchemaTools, Sdf, Tf, Usd, UsdGeom, UsdPhysics

from .model import GeometricCollisionFilter

if TYPE_CHECKING:
    from .robot import GeometricRobot

__all__ = ["CollisionScope", "PairKind", "GeometricCollisionPair", "GeometricLinkSet", "GeometricCollisionChecker", "GeometricContactMonitor", "find_colliders"]

CollisionScope = Literal["within", "environment", "all"]
PairKind = Literal["within_component", "between_components", "between_robots", "environment"]
PairFilter = Callable[[str, str], bool]


@dataclass(frozen=True)
class GeometricCollisionPair:
    """重なっている 1 組。相手が環境(ロボット以外)のとき link_b は None で、path_b はコライダーの prim パス。"""

    link_a: str
    link_b: str | None
    path_a: str
    path_b: str
    kind: PairKind
    component_a: str | None = None
    component_b: str | None = None
    robot_a: str = ""
    robot_b: str | None = None

    @property
    def names(self) -> tuple[str, str]:
        return self.link_a, (self.link_b if self.link_b is not None else self.path_b)


def find_colliders(stage: Usd.Stage, link_prim_path: str, other_link_paths: Sequence[str] = ()) -> list[str]:
    """リンクの prim 以下にあるコライダー形状(CollisionAPI が効いている Gprim)のパス。

    リンクの prim の下に別のリンクの prim が入れ子になっている場合、その下は数えない。
    physics:collisionEnabled = false のものは除く。
    """
    root = stage.GetPrimAtPath(link_prim_path)
    if not root or not root.IsValid():
        return []
    others = {p for p in other_link_paths if p != link_prim_path}
    out: list[str] = []
    it = iter(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    for prim in it:
        if str(prim.GetPath()) in others:
            it.PruneChildren()
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue
        owner = prim
        while owner and owner.IsValid() and not owner.HasAPI(UsdPhysics.CollisionAPI):
            if owner == root:
                owner = None
                break
            owner = owner.GetParent()
        if not owner:
            continue
        enabled = UsdPhysics.CollisionAPI(owner).GetCollisionEnabledAttr().Get()
        if enabled is None or enabled:
            out.append(str(prim.GetPath()))
    return out


class GeometricLinkSet:
    """干渉チェックの範囲になるリンクの集合。robot.link_set や component.link_set から得る。"""

    def __init__(self, robot: "GeometricRobot", link_names: Sequence[str], name: str) -> None:
        self.robot = robot
        self.name = name
        self.link_names = [n for n in link_names]

    @property
    def collider_paths(self) -> dict[str, list[str]]:
        """リンク名 -> コライダー形状の prim パス。"""
        all_colliders = self.robot.collider_paths
        return {n: all_colliders.get(n, []) for n in self.link_names}

    @property
    def link_paths(self) -> list[str]:
        paths = self.robot.link_path_map
        return [paths[n] for n in self.link_names if n in paths]

    def union(self, other: "GeometricLinkSet", name: str | None = None) -> "GeometricLinkSet":
        if other.robot is not self.robot:
            raise ValueError("別のロボットのリンク集合は合わせられない(check_between を使う)")
        return GeometricLinkSet(self.robot, list(dict.fromkeys(self.link_names + other.link_names)), name or f"{self.name}+{other.name}")


class _Owners:
    """重なった相手のコライダーのパスから、それがどのロボットのどのリンクかを求める。

    PhysX が返すパスは、CollisionAPI が付いたいちばん上の prim(Xform のこともある)で、形状の Gprim とは限らない。
    そこで、パスを親へたどって最初に見つかるリンクの prim で判定する。
    """

    def __init__(self, robots: Sequence["GeometricRobot"]) -> None:
        self._links: dict[str, tuple["GeometricRobot", str]] = {}
        for robot in robots:
            for link, path in robot.link_path_map.items():
                self._links[path] = (robot, link)
        self._cache: dict[str, tuple["GeometricRobot", str] | None] = {}

    def find(self, collider_path: str) -> tuple["GeometricRobot", str] | None:
        if collider_path in self._cache:
            return self._cache[collider_path]
        path = collider_path
        out = None
        while path and path != "/":
            if path in self._links:
                out = self._links[path]
                break
            path = path.rsplit("/", 1)[0]
        self._cache[collider_path] = out
        return out

class GeometricCollisionChecker:
    def __init__(self, stage: Usd.Stage) -> None:
        self._stage = stage
        self._query = get_physx_scene_query_interface()
        # Stop 中に PhysX へ読み込んだシーンは、後から足した prim を拾わない(2026-11-16 の実測)。
        # prim の追加・削除・スキーマの変更があったら、次のチェックの前に読み直す
        self._needs_reload = True
        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)

    def _on_objects_changed(self, notice, _sender) -> None:
        if notice.GetResyncedPaths():
            self._needs_reload = True

    # ------------------------------------------------------------------ 公開の問い合わせ

    def check_within(
        self, link_set: GeometricLinkSet, *, collision_filter: GeometricCollisionFilter | None = None, extra_filter: PairFilter | None = None
    ) -> list[GeometricCollisionPair]:
        """link_set の内側で重なっているリンクの組。

        collision_filter を省略すると、そのロボットのフィルタ(robot.collision_filter)を使う。
        extra_filter(link_a, link_b) が True を返す組は、この 1 回だけ追加で無視する。
        """
        robot = link_set.robot
        flt = robot.collision_filter if collision_filter is None else collision_filter
        owners = _Owners([robot])
        members = set(link_set.link_names)
        self._prepare([robot], link_set, owners)
        found: dict[frozenset[str], GeometricCollisionPair] = {}
        for link, colliders in link_set.collider_paths.items():
            for collider in colliders:
                for hit in self._hits(collider):
                    other = owners.find(hit)
                    if other is None or other[1] == link or other[1] not in members:
                        continue
                    key = frozenset((link, other[1]))
                    if key in found or flt.is_ignored(link, other[1]) or (extra_filter and (extra_filter(link, other[1]) or extra_filter(other[1], link))):
                        continue
                    found[key] = self._pair(robot, link, robot, other[1])
        return self._sorted(found.values())

    def check_between(self, set_a: GeometricLinkSet, set_b: GeometricLinkSet, *, extra_filter: PairFilter | None = None) -> list[GeometricCollisionPair]:
        """set_a のリンクと set_b のリンクの間で重なっている組。同じロボットの中の 2 集合なら、そのロボットのフィルタも使う。"""
        owners = _Owners([set_a.robot, set_b.robot])
        members_b = set(set_b.link_names)
        self._prepare([set_a.robot, set_b.robot], set_a, owners)
        same_robot = set_a.robot is set_b.robot
        found: dict[tuple[str, str], GeometricCollisionPair] = {}
        for link, colliders in set_a.collider_paths.items():
            for collider in colliders:
                for hit in self._hits(collider):
                    other = owners.find(hit)
                    if other is None or other[0] is not set_b.robot or other[1] not in members_b or (same_robot and other[1] == link):
                        continue
                    key = (link, other[1])
                    if key in found:
                        continue
                    if same_robot and set_a.robot.collision_filter.is_ignored(link, other[1]):
                        continue
                    if extra_filter and (extra_filter(link, other[1]) or extra_filter(other[1], link)):
                        continue
                    found[key] = self._pair(set_a.robot, link, set_b.robot, other[1])
        return self._sorted(found.values())

    def check_environment(
        self, link_set: GeometricLinkSet, *, exclude: Sequence[GeometricLinkSet] = (), ignore_prim_paths: Sequence[str] = ()
    ) -> list[GeometricCollisionPair]:
        """link_set のリンクと「環境」の間で重なっている組。

        環境 = link_set のロボットのどのリンクの prim の下にも無いコライダー(exclude に渡した集合のロボットも除く)。
        ignore_prim_paths に渡したパス以下のコライダーも除く。
        """
        robot = link_set.robot
        robots = list(dict.fromkeys([robot] + [s.robot for s in exclude]))
        owners = _Owners(robots)
        self._prepare(robots, link_set, owners)
        ignore = tuple(p.rstrip("/") for p in ignore_prim_paths)
        found: dict[tuple[str, str], GeometricCollisionPair] = {}
        for link, colliders in link_set.collider_paths.items():
            for collider in colliders:
                for hit in self._hits(collider):
                    if owners.find(hit) is not None or any(hit == p or hit.startswith(p + "/") for p in ignore):
                        continue
                    if (link, hit) not in found:
                        found[(link, hit)] = GeometricCollisionPair(
                            link, None, robot.link_path_map[link], hit, "environment", robot.model.component_of_link(link), None, robot.name, None
                        )
        return self._sorted(found.values())

    def touching_pairs(self, link_set: GeometricLinkSet) -> list[GeometricCollisionPair]:
        """フィルタを使わずに、いま重なっている組をすべて返す(無視する組を決める材料)。"""
        return self.check_within(link_set, collision_filter=GeometricCollisionFilter())

    def reload(self) -> None:
        """Stop 中に、PhysX のシーンを USD から読み直す。prim の追加・削除の後は自動で呼ばれる。"""
        if omni.timeline.get_timeline_interface().is_stopped():
            physx = get_physx_interface()
            physx.release_physics_objects()
            physx.force_load_physics_from_usd()
        self._needs_reload = False

    # ------------------------------------------------------------------ 内部

    def _hits(self, collider_path: str) -> list[str]:
        hits: list[str] = []

        def report(hit) -> bool:
            hits.append(str(hit.collision))
            return True

        id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(collider_path))
        self._query.overlap_shape(id0, id1, report, False)
        return hits

    def _prepare(self, robots: Sequence["GeometricRobot"], link_set: GeometricLinkSet, owners: "_Owners") -> None:
        """チェックの前に、姿勢を PhysX が見る場所(USD)へ書き、PhysX にいまのシーンが読み込まれていることを確かめる。"""
        stopped = omni.timeline.get_timeline_interface().is_stopped()
        for robot in robots:
            if robot.layer.is_applied() and robot.layer.options.collision_mode == "kinematic" and stopped:
                raise RuntimeError(
                    'collision_mode="kinematic" のロボットは、Stop 中は書いた姿勢が PhysX に入らないので干渉チェックできない。'
                    'Play 中に使うか、collision_mode="static" にする'
                )
            robot.flush_to_usd()
        if stopped and self._needs_reload:
            self.reload()
        probe = next((c for link, paths in link_set.collider_paths.items() for c in paths), None)
        if probe is None:
            return
        link = owners.find(probe)

        def loaded() -> bool:  # 形状は必ず自分自身に当たるので、それで読み込み済みかを判定する
            return any(owners.find(h) == link for h in self._hits(probe))

        if loaded():
            return
        if stopped:
            get_physx_interface().force_load_physics_from_usd()
        if not loaded():
            raise RuntimeError(f"PhysX のシーンにコライダー {probe} が無い(PhysicsScene はある？)")

    @staticmethod
    def _pair(robot_a: "GeometricRobot", link_a: str, robot_b: "GeometricRobot", link_b: str) -> GeometricCollisionPair:
        comp_a, comp_b = robot_a.model.component_of_link(link_a), robot_b.model.component_of_link(link_b)
        if robot_a is not robot_b:
            kind: PairKind = "between_robots"
        elif comp_a == comp_b:
            kind = "within_component"
        else:
            kind = "between_components"
        return GeometricCollisionPair(link_a, link_b, robot_a.link_path_map[link_a], robot_b.link_path_map[link_b], kind, comp_a, comp_b, robot_a.name, robot_b.name)

    @staticmethod
    def _sorted(pairs) -> list[GeometricCollisionPair]:
        return sorted(pairs, key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))


class GeometricContactMonitor:
    """kinematic のロボットの接触を、Play 中に Contact Report で受け取る。

        monitor = GeometricContactMonitor(robot)     # robot は collision_mode="kinematic" で作る
        ...(Play 中に robot.set_dof_positions(...) と update を繰り返す)...
        monitor.penetrating()      # いまめり込んでいる組(separation <= 0)
        monitor.touching()         # 報告が続いている組(触れる少し手前から含む)
        monitor.close()

    報告は、姿勢を書いてから 2 回目の update で届く(1 update 遅れる)。一度も動いていない組は報告されない。
    ロボットのフィルタ(robot.collision_filter)に入っている組は捨てる。
    """

    def __init__(self, robots: "GeometricRobot | Sequence[GeometricRobot]", max_events: int = 10000) -> None:
        self._robots = [robots] if not isinstance(robots, (list, tuple)) else list(robots)
        for robot in self._robots:
            if robot.layer.options.collision_mode != "kinematic":
                raise ValueError('GeometricContactMonitor は collision_mode="kinematic" のロボット用')
        self._owner: dict[str, tuple["GeometricRobot", str]] = {}
        for robot in self._robots:
            for link, path in robot.link_path_map.items():
                self._owner[path] = (robot, link)
        self._touch: dict[tuple[str, str], tuple[GeometricCollisionPair, float | None]] = {}
        self.events: list[tuple[str, GeometricCollisionPair, float | None]] = []
        self._max_events = max_events
        self._subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact)
        if omni.timeline.get_timeline_interface().is_stopped():
            carb.log_warn("[myext.robot.geometric] GeometricContactMonitor: Contact Report は Play 中だけ届く")

    def _make_pair(self, path0: str, path1: str) -> GeometricCollisionPair | None:
        a, b = self._owner.get(path0), self._owner.get(path1)
        if a is None and b is None:
            return None
        if a is None:
            a, b, path0, path1 = b, a, path1, path0
        assert a is not None
        if b is None:
            return GeometricCollisionPair(a[1], None, path0, path1, "environment", a[0].model.component_of_link(a[1]), None, a[0].name, None)
        if a[0] is b[0] and a[0].collision_filter.is_ignored(a[1], b[1]):
            return None
        return GeometricCollisionChecker._pair(a[0], a[1], b[0], b[1])

    def _on_contact(self, headers, contact_data) -> None:
        for header in headers:
            path0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            path1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            pair = self._make_pair(path0, path1)
            if pair is None:
                continue
            key = (pair.path_a, pair.path_b)
            kind = str(header.type).split(".")[-1]
            if kind == "CONTACT_LOST":
                self._touch.pop(key, None)
                separation = None
            else:
                seps = [contact_data[header.contact_data_offset + i].separation for i in range(header.num_contact_data)]
                separation = min(seps) if seps else None
                self._touch[key] = (pair, separation)
            if len(self.events) < self._max_events:
                self.events.append((kind.replace("CONTACT_", ""), pair, separation))

    def touching(self) -> list[GeometricCollisionPair]:
        return GeometricCollisionChecker._sorted([p for p, _ in self._touch.values()])

    def penetrating(self) -> list[GeometricCollisionPair]:
        return GeometricCollisionChecker._sorted([p for p, sep in self._touch.values() if sep is not None and sep <= 0.0])

    def separations(self) -> dict[tuple[str, str], float | None]:
        """{(リンク名, 相手のリンク名かコライダーのパス): いちばん小さい separation}。負ならめり込み量。"""
        return {p.names: sep for p, sep in self._touch.values()}

    def clear(self) -> None:
        self._touch.clear()
        self.events.clear()

    def close(self) -> None:
        self._subscription = None
