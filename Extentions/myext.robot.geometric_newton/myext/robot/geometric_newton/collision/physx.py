"""PhysX バックエンド (`GeometricCollisionChecker` / `GeometricContactMonitor`)。"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from ..model import GeometricCollisionFilter
from .types import GeometricCollisionPair, GeometricLinkSet, PairFilter, _Owners

if TYPE_CHECKING:
    from ..runtime.robot import GeometricRobot

__all__ = [
    "GeometricCollisionChecker",
    "GeometricContactMonitor",
]


# ==============================================================================
# PhysX 干渉チェッカー (collision_backend="physx" 用)
# ==============================================================================


class GeometricCollisionChecker:
    """PhysX の `overlap_shape` を使う干渉チェッカー (`collision_backend="physx"` 専用)。

    `collision_backend="newton"` (`GeometricScene`) とは異なり、PhysX の空間索引を参照するため
    クエリ前に `robot.flush_to_usd()` で姿勢を USD Sublayer へ書き出す必要がある。
    また `omni.physx` への依存は `__init__` 内で遅延 import しており、Kit なしの純 Python 環境でも
    `GeometricCollisionPair` や `find_colliders` を問題なく import できる。
    """

    def __init__(self, stage: Usd.Stage) -> None:
        from omni.physx import get_physx_scene_query_interface

        self._stage = stage
        self._query = get_physx_scene_query_interface()
        # Stop 中に PhysX へ読み込んだシーンは、後から USD に追加された prim を自動では拾わない。
        # そのため `Usd.Notice.ObjectsChanged` の resync 通知を監視し、次のチェック前に `reload()` を走らせる。
        self._needs_reload = True
        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)

    def _on_objects_changed(self, notice, _sender) -> None:
        if notice.GetResyncedPaths():
            self._needs_reload = True

    # ------------------------------------------------------------------ 公開の問い合わせ

    def check_within(
        self, link_set: GeometricLinkSet, *, collision_filter: GeometricCollisionFilter | None = None, extra_filter: PairFilter | None = None
    ) -> list[GeometricCollisionPair]:
        """`link_set` の内側で重なっているリンクの組を返す。

        `collision_filter` を省略すると `link_set.robot.collision_filter` を使用する。
        `extra_filter(link_a, link_b)` が `True` を返す組は、この呼び出しに限り追加で除外する。
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
        """`set_a` のリンクと `set_b` のリンクの間で重なっている組を返す。

        同一ロボット内の 2 集合であれば、そのロボットの `collision_filter` も適用する。
        """
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
        """`link_set` のリンクと環境コライダーの間で重なっている組を返す。

        ここでの「環境」は、`link_set.robot` および `exclude` に含まれるロボットのどのリンク prim の配下にも
        属さないコライダーを指す。`ignore_prim_paths` に一致するパスまたはその配下のコライダーも除外する。
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
        """`collision_filter` を空にして現在重なっている組をすべて返す (診断用; `ignore_touching_pairs` は既存フィルタ適用済みの `check_within` を使う)。"""
        return self.check_within(link_set, collision_filter=GeometricCollisionFilter())

    def reload(self) -> None:
        """Stop 中に PhysX のシーンを USD から再読込する。prim の追加・削除の後は `_prepare` から自動で呼ばれる。"""
        import omni.timeline
        from omni.physx import get_physx_interface

        if omni.timeline.get_timeline_interface().is_stopped():
            physx = get_physx_interface()
            physx.release_physics_objects()
            physx.force_load_physics_from_usd()
        self._needs_reload = False

    # ------------------------------------------------------------------ 内部処理

    def _hits(self, collider_path: str) -> list[str]:
        """単一コライダー prim に対する PhysX `overlap_shape` を実行し、接触したコライダー prim パス一覧を返す。"""
        from pxr import PhysicsSchemaTools

        hits: list[str] = []

        def report(hit) -> bool:
            hits.append(str(hit.collision))
            return True

        id0, id1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(collider_path))
        self._query.overlap_shape(id0, id1, report, False)
        return hits

    def _prepare(self, robots: Sequence["GeometricRobot"], link_set: GeometricLinkSet, owners: "_Owners") -> None:
        """PhysX クエリの前準備として姿勢を USD へ書き出し、PhysX へのシーン読込済み状態を保証する。

        なぜ `robot.flush_to_usd()` が必須か:
            - `write_target="fabric"` の既定動作では、ロボットの姿勢は Fabric にのみ書かれている。
            - PhysX の `overlap_shape` は USD (Sublayer) に書かれた Transform を参照するため、
              `flush_to_usd()` を呼ばないままクエリすると古い姿勢で判定されてしまう。
            - `collision_mode="kinematic"` の剛体は Stop 中に PhysX へ位置同期されないため、明示エラーにする。
        """
        import omni.timeline
        from omni.physx import get_physx_interface

        stopped = omni.timeline.get_timeline_interface().is_stopped()
        for robot in robots:
            if robot.layer.is_applied() and robot.layer.options.collision_mode == "none":
                raise RuntimeError(
                    'collision_mode="none" では PhysX コライダーが無効化されているため collision_backend="physx" は使えない。'
                    '既定の collision_backend="newton" を使うか、GeometricLayerOptions(collision_mode="static") を指定する'
                )
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

        def loaded() -> bool:
            # PhysX に形状が読み込まれていれば `overlap_shape` は必ず自分自身にヒットする。
            # この性質を「PhysX シーン読込済みか」の確実な判定プローブとして使う。
            return any(owners.find(h) == link for h in self._hits(probe))

        if loaded():
            return
        if stopped:
            get_physx_interface().force_load_physics_from_usd()
        if not loaded():
            raise RuntimeError(f"PhysX のシーンにコライダー {probe} が無い(PhysicsScene はある？)")

    @staticmethod
    def _pair(
        robot_a: "GeometricRobot",
        link_a: str,
        robot_b: "GeometricRobot",
        link_b: str,
        *,
        separation: float | None = None,
        point_a: tuple[float, float, float] | None = None,
        point_b: tuple[float, float, float] | None = None,
        normal: tuple[float, float, float] | None = None,
    ) -> GeometricCollisionPair:
        """2 つのロボットリンクの組から `kind` を自動分類して `GeometricCollisionPair` を組み立てる。"""
        comp_a, comp_b = robot_a.model.component_of_link(link_a), robot_b.model.component_of_link(link_b)
        if robot_a is not robot_b:
            kind: PairKind = "between_robots"
        elif comp_a == comp_b:
            kind = "within_component"
        else:
            kind = "between_components"
        return GeometricCollisionPair(
            link_a,
            link_b,
            robot_a.link_path_map.get(link_a, link_a),
            robot_b.link_path_map.get(link_b, link_b),
            kind,
            comp_a,
            comp_b,
            robot_a.name,
            robot_b.name,
            separation=separation,
            point_a=point_a,
            point_b=point_b,
            normal=normal,
        )

    @staticmethod
    def _sorted(pairs) -> list[GeometricCollisionPair]:
        return sorted(pairs, key=lambda p: (p.kind, p.link_a, p.link_b or p.path_b))


# ==============================================================================
# PhysX 接触モニタ (Play=ON + collision_mode="kinematic" 用)
# ==============================================================================


class GeometricContactMonitor:
    """`collision_mode="kinematic"` のロボットの接触を、Play=ON 中に PhysX の Contact Report で購読する。

        monitor = GeometricContactMonitor(robot)     # robot は collision_mode="kinematic" で作る
        # ... Play=ON 中に robot.set_dof_positions(...) と app.update() を繰り返す ...
        monitor.penetrating()      # 現在めり込んでいる組 (separation <= 0)
        monitor.touching()         # Contact Report が継続している組 (接触手前のマージン内も含む)
        monitor.close()

    制約と挙動:
        - 姿勢を書いてから PhysX ステップに反映されるまで 1 `update` の遅延があるため、
          Contact Report が届くのは 2 回目の `update` になる。
        - 一度も動いていない組は PhysX がスリープ扱いにするため報告されない。
        - 0 フレーム遅延でめり込み量・クリアランスを取得したい場合は、本クラスではなく
          `collision_backend="newton"` (`GeometricScene.check_collisions` / `compute_clearances`) を使う。
        - ロボットの `collision_filter` に登録済みの組は受信時に破棄する。
    """

    def __init__(self, robots: "GeometricRobot | Sequence[GeometricRobot]", max_events: int = 10000) -> None:
        import carb
        import omni.timeline
        from omni.physx import get_physx_simulation_interface

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
            carb.log_warn("[myext.robot.geometric_newton] GeometricContactMonitor: Contact Report は Play 中だけ届く")

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
        from pxr import PhysicsSchemaTools

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
        """現在接触が継続している組 (接触手前のマージン内も含む) を返す。"""
        return GeometricCollisionChecker._sorted([p for p, _ in self._touch.values()])

    def penetrating(self) -> list[GeometricCollisionPair]:
        """現在めり込んでいる組 (`separation <= 0.0`) のみを返す。"""
        return GeometricCollisionChecker._sorted([p for p, sep in self._touch.values() if sep is not None and sep <= 0.0])

    def separations(self) -> dict[tuple[str, str], float | None]:
        """`{(リンク名, 相手リンク名または環境コライダーパス): 最小 separation}`。負ならめり込み量。"""
        return {p.names: sep for p, sep in self._touch.values()}

    def clear(self) -> None:
        """蓄積済みの接触状態とイベント履歴を空にする。"""
        self._touch.clear()
        self.events.clear()

    def close(self) -> None:
        """Contact Report の購読を解除する。"""
        self._subscription = None
