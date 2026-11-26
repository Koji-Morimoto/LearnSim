"""干渉チェックの共通データ型および PhysX バックエンド実装。

「自分自身との干渉」に固定の意味を持たせず、呼び出し側が必ず
「どのリンク集合の内側か (`check_within`) / どの集合とどの集合の間か (`check_between`) / 環境との間か (`check_environment`)」
を明示する設計になっている。結合後にコンポーネント単位で同じ意味のまま干渉判定を継続するためである。

    checker.check_within(robot.link_set)                         # ロボットの全リンク同士
    checker.check_within(robot.get_component("left").link_set)   # 左アームのリンク同士 (結合後も意味は同じ)
    checker.check_between(a.link_set, b.link_set)                # 2 つのリンク集合の間
    checker.check_environment(robot.link_set)                    # ロボット以外の環境コライダーとの間

バックエンド間の設計差と使い分け:
    - Newton バックエンド (`collision_backend="newton"`, 実装は `newton_backend.GeometricScene`):
      `GeometricRobot` の関節値から自前の Warp メモリ上で `newton.eval_fk` + `newton.CollisionPipeline` を直接実行する。
      そのため `write_target="fabric"` のまま `flush_to_usd()` を呼ばずに Stop / Play=ON ともに 0 フレーム遅延で判定でき、
      `GeometricCollisionPair` の `separation` (符号付き距離)・`point_a`・`point_b`・`normal` も埋まる。
    - PhysX バックエンド (`collision_backend="physx"`, 実装は `GeometricCollisionChecker`):
      PhysX の `overlap_shape` を使うため、チェック前に必ず `robot.flush_to_usd()` で姿勢を USD に書き出す必要がある。
      - `collision_mode="static"` (既定): Stop でも Play=ON でも `flush_to_usd()` 直後から結果に反映される。
      - `collision_mode="kinematic"`: Play=ON 中のみ使用可能。書き込んだ姿勢が PhysX に反映されるのは次の `update` 以降のため、
        結果は 1 `update` 遅れる。Play=ON 中に接触開始・終了イベントを受け取る場合は `GeometricContactMonitor` を使う。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Literal, Sequence

from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from ..usd.layer import is_collision_enabled_ignoring_geometric_layer
from ..model import GeometricCollisionFilter

if TYPE_CHECKING:
    from ..runtime.robot import GeometricRobot

__all__ = [
    "CollisionScope",
    "PairKind",
    "PairFilter",
    "GeometricCollisionPair",
    "GeometricLinkSet",
    "find_colliders",
]

# ==============================================================================
# 公開型定義 (Newton / PhysX バックエンド共通)
# ==============================================================================

CollisionScope = Literal["within", "environment", "all"]
PairKind = Literal["within_component", "between_components", "between_robots", "environment"]
PairFilter = Callable[[str, str], bool]


@dataclass(frozen=True)
class GeometricCollisionPair:
    """重なっている、またはクリアランス範囲内にある 1 組の干渉結果。

    `collision_backend="newton"` と `collision_backend="physx"` の両方で共通の戻り値型として使う。
    相手が環境コライダー (ロボット以外) のときは `link_b` が `None`、`path_b` が環境コライダーの prim パスになる。

    Newton バックエンド (`GeometricScene` / `BatchedNewtonScene`) で埋まる幾何詳細フィールド:
        separation : 符号付き距離 [m] (負ならめり込み量、0 なら接触、正ならクリアランス距離)。PhysX では `None`。
        point_a    : `link_a` 側の表面接触点 (ワールド座標, ステージ単位)。PhysX では `None`。
        point_b    : `link_b` または環境コライダー側の表面接触点 (ワールド座標, ステージ単位)。PhysX では `None`。
        normal     : `link_a` 側から `link_b` / 環境側へ向かう単位法線ベクトル (ワールド座標)。PhysX では `None`。
    """

    link_a: str
    link_b: str | None
    path_a: str
    path_b: str
    kind: PairKind
    component_a: str | None = None
    component_b: str | None = None
    robot_a: str = ""
    robot_b: str | None = None
    separation: float | None = None
    point_a: tuple[float, float, float] | None = None
    point_b: tuple[float, float, float] | None = None
    normal: tuple[float, float, float] | None = None

    @property
    def names(self) -> tuple[str, str]:
        """`(link_a, 相手名)` の 2 要素タプル。環境相手のときは `path_b` を返す。"""
        return self.link_a, (self.link_b if self.link_b is not None else self.path_b)


# ==============================================================================
# コライダー探索とリンク集合・所有者解決
# ==============================================================================


def find_colliders(stage: Usd.Stage, link_prim_path: str, other_link_paths: Sequence[str] = ()) -> list[str]:
    """リンクの prim 配下にある有効なコライダー形状 (`UsdGeom.Gprim`) のパス一覧を返す。

    設計上の理由:
        - `UsdPhysics.CollisionAPI` は `UsdGeom.Gprim` 自身ではなく、親の `Xform` (例: `collisions`) に
          まとめて付与されているアセットが多いため、各 `Gprim` から `link_prim_path` まで親をたどって
          `CollisionAPI` の有無と `physics:collisionEnabled` を判定する。
        - USD 階層上でリンク prim の配下に別リンクの prim が入れ子になっている場合がある
          (ロボットの関節親子と USD 階層が一致している構成)。別リンクのコライダーを親リンク側へ重複計上
          しないよう、`other_link_paths` に一致した時点で `PruneChildren()` して枝刈りする。
        - インスタンス化されたアセット配下のコライダーも拾えるよう `Usd.TraverseInstanceProxies()` を使う。
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
        if is_collision_enabled_ignoring_geometric_layer(owner):
            out.append(str(prim.GetPath()))
    return out


class GeometricLinkSet:
    """干渉チェックの対象範囲を表すリンク集合。`robot.link_set` や `component.link_set` から取得する。"""

    def __init__(self, robot: "GeometricRobot", link_names: Sequence[str], name: str) -> None:
        self.robot = robot
        self.name = name
        self.link_names = [n for n in link_names]

    @property
    def collider_paths(self) -> dict[str, list[str]]:
        """この集合に含まれる `リンク名 -> コライダー形状の prim パス一覧` の辞書。"""
        all_colliders = self.robot.collider_paths
        return {n: all_colliders.get(n, []) for n in self.link_names}

    @property
    def link_paths(self) -> list[str]:
        """この集合に含まれるリンクの USD prim パス一覧 (prim を持つリンクのみ)。"""
        paths = self.robot.link_path_map
        return [paths[n] for n in self.link_names if n in paths]

    def union(self, other: "GeometricLinkSet", name: str | None = None) -> "GeometricLinkSet":
        """同一ロボット内の 2 つのリンク集合を重複なく結合した新しい `GeometricLinkSet` を返す。"""
        if other.robot is not self.robot:
            raise ValueError("別のロボットのリンク集合は合わせられない(check_between を使う)")
        return GeometricLinkSet(self.robot, list(dict.fromkeys(self.link_names + other.link_names)), name or f"{self.name}+{other.name}")


class _Owners:
    """コライダーの prim パスから、所有する `(GeometricRobot, リンク名)` を逆引きするヘルパー。

    PhysX の `overlap_shape` が返す `hit.collision` は `UsdPhysics.CollisionAPI` が付いた最上位 prim
    (`Xform` の場合もあれば末端の `Gprim` の場合もある) であり、リンク prim そのもののパスとは限らない。
    そのためパス文字列を親方向へ 1 段ずつ切り詰め、最初に一致したリンク prim で所有者を確定する。
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


