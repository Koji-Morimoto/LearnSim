"""組み立て済みロボットの中立記述子(`AssembledRobotDescriptor`)と構成要素(`components`)の自動発見。

責務:
  - 特定の幾何・運動学ライブラリへ一切依存せず、OpenUSD(`pxr`)だけで組み立て済みロボットの
    構成要素・Articulation ルート・リンク・関節・VariantSet 選択状態を抽出する。
  - VariantSet が `"None"` 等に切り替わってステージ上に現在展開されていない構成要素も、
    `compose.py` が `customLayerData` / `customData` に記録したメタデータから補完発見する。
  - 組み立て済み `.usda` を任意ステージへ Reference 配置して即座に記述子を返す(`spawn_assembled_robot`)。

主な入出力・公開 API:
  - 定数: `CUSTOM_LAYER_KEY`
  - データクラス: `AssembledRobotDescriptor`（別名: `ArticulationDescriptor`）
  - 発見・検査・配置:
    `discover_components`（別名: `components_from_robot_api`）,
    `inspect_assembled_robot`（別名: `extract_articulation_descriptor`）,
    `spawn_assembled_robot`
  - 内部共有補助: `_extract_authored_components`（`compose.py` が子結合ファイルの既存メタデータ継承に使用）

呼ばれ元:
  - `__init__.py`（公開 API として再エクスポート）
  - `compose.py`, `part_import.py`, `kinematics_check.py`, `pipeline.py`, `run_pipeline.py`

設計方針:
  - 下流の呼び出し側は `AssembledRobotDescriptor.to_articulation_source_dict()` または
    `discover_components()` の返り値をそのまま外部の Articulation リーダーへ渡せる。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pxr import Sdf, Usd, UsdPhysics

from .usd_util import (
    ALL_PRIMS,
    DEFINED_PRIMS,
    ROBOT_LINKS,
    articulation_roots,
    display_name,
    has_api,
    joint_active,
    joints_under,
)

# `compose.py` / `part_import.py` が構成要素マップを書き込む USD customLayerData / customData キー。
# `_LEGACY_LAYER_KEY` は旧名(.v2)時代に生成された既存アセット互換の読み取り専用フォールバック。
CUSTOM_LAYER_KEY = "myext.robot.assembly_pipeline"
_LEGACY_LAYER_KEY = "myext.robot.assembly_pipeline.v2"


def _extract_authored_components(prim: Usd.Prim) -> dict[str, str]:
    """`prim` の `customData` および `primStack` 各レイヤーの `customLayerData` から `components` 辞書を取り出す。

    なぜ `customData` → `primStack` の順で `setdefault` するか:
      - Reference で埋め込まれた子結合ファイルのメタデータは `prim.GetCustomData()` や
        `prim.GetPrimStack()` のレイヤー群に分散する。
      - `primStack` は USD の強い意見(上位レイヤー)から弱い意見の順に並ぶため、`setdefault` で
        最初に見つかった定義を優先しつつ、下位レイヤーにしかない構成要素も漏れなく統合できる。
    """
    merged: dict[str, str] = {}
    if not prim or not prim.IsValid():
        return merged

    try:
        cd = prim.GetCustomData()
        for key in (CUSTOM_LAYER_KEY, _LEGACY_LAYER_KEY):
            block = cd.get(key)
            if isinstance(block, dict) and isinstance(block.get("components"), dict):
                for k, v in block["components"].items():
                    if isinstance(k, str) and isinstance(v, str):
                        merged.setdefault(k, v)
    except Exception:  # noqa: BLE001
        pass

    try:
        for spec in prim.GetPrimStack():
            layer = spec.layer
            if not layer:
                continue
            cld = layer.customLayerData or {}
            for key in (CUSTOM_LAYER_KEY, _LEGACY_LAYER_KEY):
                block = cld.get(key)
                if isinstance(block, dict) and isinstance(block.get("components"), dict):
                    for k, v in block["components"].items():
                        if isinstance(k, str) and isinstance(v, str):
                            merged.setdefault(k, v)
    except Exception:  # noqa: BLE001
        pass

    return merged


def discover_components(
    stage: Usd.Stage,
    robot_prim_path: str,
    *,
    include_inactive_variants: bool = True,
) -> dict[str, str]:
    """組み立て済みロボットの構成要素マップ `{構成要素名: prim パス}` を発見して返す。

    引数:
      stage: 検査対象の `Usd.Stage`。
      robot_prim_path: ロボットルート prim のパス。
      include_inactive_variants: `True`(既定)のとき、現在 VariantSet が `"None"` や別選択肢に切り替わっていて
                                 ステージ上に実体がない構成要素も、`customLayerData` / `customData` から補完する。

    戻り値:
      `{構成要素名: 絶対 prim パス}` 辞書。ルート自身は `display_name(root_prim)`、
      その配下は `IsaacRobotAPI` を持つ prim 名を階層順に `"/"` で連結した名前になる。

    非アクティブ Variant メタデータマージの意図:
      1. まず現在のステージ上で展開されている `IsaacRobotAPI` prim を実測ベースで収集する(実パスが最優先)。
      2. 次に `names` に集まった各構成要素 prim の `customData` / `primStack` から相対 `components` を展開し、
         未登録の非アクティブ構成要素を `setdefault` で補う(Reference 経由の多段結合にも対応)。
      3. 最後に結合ファイルを直接 `Usd.Stage.Open` した場合に備え、`stage.GetRootLayer().customLayerData` も補完確認する。
    """
    root_prim = stage.GetPrimAtPath(str(robot_prim_path))
    if not root_prim or not root_prim.IsValid():
        return {}
    root_path = str(root_prim.GetPath())
    root_name = display_name(root_prim)
    out: dict[str, str] = {root_name: root_path}
    names: dict[str, str] = {root_path: ""}

    # ステージ上で現在展開されている構成要素(IsaacRobotAPI 付き prim)を階層順に登録する。
    for p in Usd.PrimRange(root_prim, DEFINED_PRIMS):
        path = str(p.GetPath())
        if path == root_path or not has_api(p, "IsaacRobotAPI"):
            continue
        parent = max((q for q in names if path.startswith(q + "/")), key=len)
        name = f"{names[parent]}/{p.GetName()}" if names[parent] else p.GetName()
        while name in out and out[name] != path:
            name += "_"
        names[path] = name
        out[name] = path

    if include_inactive_variants:
        # 1. ルート prim および発見済み各構成要素 prim に記録された相対 components をマージする。
        for comp_path, comp_prefix in list(names.items()):
            comp_prim = stage.GetPrimAtPath(comp_path)
            authored = _extract_authored_components(comp_prim)
            for sub_name, rel_path in authored.items():
                if not sub_name or not rel_path or rel_path == ".":
                    continue
                rel_clean = rel_path.lstrip("/")
                full_name = f"{comp_prefix}/{sub_name}" if comp_prefix else sub_name
                full_path = f"{comp_path}/{rel_clean}"
                out.setdefault(full_name, full_path)

        # 2. 結合ファイルを直接 Stage.Open した場合、rootLayer の customLayerData からも補完する。
        try:
            root_cld = stage.GetRootLayer().customLayerData or {}
            for key in (CUSTOM_LAYER_KEY, _LEGACY_LAYER_KEY):
                block = root_cld.get(key)
                if isinstance(block, dict) and isinstance(block.get("components"), dict):
                    for sub_name, rel_path in block["components"].items():
                        if not sub_name or not rel_path or rel_path == ".":
                            continue
                        if isinstance(sub_name, str) and isinstance(rel_path, str):
                            out.setdefault(sub_name, f"{root_path}/{rel_path.lstrip('/')}")
        except Exception:  # noqa: BLE001
            pass

    return out


components_from_robot_api = discover_components


@dataclass(frozen=True)
class AssembledRobotDescriptor:
    """組み立て済みロボットの構造を外部へ引き渡すためのドメイン中立な不変記述子。

    属性:
      prim_path             : ステージ上のロボットルート prim パス。
      usda_path             : 参照元の usda ファイルパス(不明な場合は `None`)。
      components            : `{構成要素名: prim パス}`。
      articulation_root_path: 検出された先頭の `ArticulationRootAPI` prim パス(無ければ `None`)。
      link_prim_paths       : `{リンク名(スコープ付きおよび短縮名): prim パス}`。
      joint_prim_paths      : `{アクティブ関節名(スコープ付きおよび短縮名): prim パス}`。
      variant_sets          : `{prim パス: {VariantSet 名: 現在の選択肢}}`。
    """

    prim_path: str
    usda_path: str | None
    components: dict[str, str] = field(default_factory=dict)
    articulation_root_path: str | None = None
    link_prim_paths: dict[str, str] = field(default_factory=dict)
    joint_prim_paths: dict[str, str] = field(default_factory=dict)
    variant_sets: dict[str, dict[str, str]] = field(default_factory=dict)

    @property
    def robot_path(self) -> str:
        """`prim_path` の互換別名。"""
        return self.prim_path

    @property
    def articulation_root(self) -> str | None:
        """`articulation_root_path` の互換別名。"""
        return self.articulation_root_path

    @property
    def link_paths(self) -> dict[str, str]:
        """`link_prim_paths` の互換別名。"""
        return self.link_prim_paths

    @property
    def joint_paths(self) -> dict[str, str]:
        """`joint_prim_paths` の互換別名。"""
        return self.joint_prim_paths

    def to_articulation_source_dict(self) -> dict[str, Any]:
        """外部 Articulation ソース(例: `UsdArticulationSource(**desc.to_articulation_source_dict())`)用の辞書を返す。"""
        return {
            "root_prim_path": self.prim_path,
            "components": dict(self.components),
        }

    def to_dict(self) -> dict[str, Any]:
        """JSON シリアライズやデバッグ出力向けに全フィールドを浅いコピー辞書へ変換する。"""
        return {
            "prim_path": self.prim_path,
            "usda_path": self.usda_path,
            "components": dict(self.components),
            "articulation_root_path": self.articulation_root_path,
            "link_prim_paths": dict(self.link_prim_paths),
            "joint_prim_paths": dict(self.joint_prim_paths),
            "variant_sets": {k: dict(v) for k, v in self.variant_sets.items()},
        }

    def __getitem__(self, key: str) -> Any:
        return self.to_dict()[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.to_dict().get(key, default)


ArticulationDescriptor = AssembledRobotDescriptor


def _component_prefix_for_path(prim_path: str, root_path: str, components: dict[str, str]) -> str:
    """`prim_path` が属する最も深い構成要素名を返す(ルート直下なら `""`)。"""
    best_name = ""
    best_len = len(root_path)
    for cname, cpath in components.items():
        if cpath == root_path:
            continue
        if (prim_path == cpath or prim_path.startswith(cpath + "/")) and len(cpath) > best_len:
            best_name = cname
            best_len = len(cpath)
    return best_name


def inspect_assembled_robot(
    stage: Usd.Stage,
    prim_path: str | None = None,
    *,
    usda_path: str | Path | None = None,
) -> AssembledRobotDescriptor:
    """ステージ上の組み立て済みロボット prim を検査して `AssembledRobotDescriptor` を返す。

    引数:
      stage: 検査対象の `Usd.Stage`。
      prim_path: 対象ロボット prim のパス。省略時は `stage.GetDefaultPrim()`(未設定なら `"/robot"`)を使う。
      usda_path: 記録用の元 usda ファイルパス(省略可)。

    戻り値:
      `AssembledRobotDescriptor`。指定 prim が無効な場合は各コレクションが空の記述子を返す。

    注意:
      - `link_prim_paths` と `joint_prim_paths` は、構成要素付きの `"comp/name"` と
        短縮名 `"name"` の両方を `setdefault` で登録する(同名リンクが複数腕にある場合は
        先に見つかった短縮名を残しつつ、`"left_arm/link1"` のようにスコープ付きで一意に引ける)。
    """
    if prim_path is None:
        dp = stage.GetDefaultPrim()
        prim_path = str(dp.GetPath()) if dp and dp.IsValid() else "/robot"
    root_prim = stage.GetPrimAtPath(str(prim_path))
    root_path = str(prim_path)
    usda_str = os.fspath(usda_path) if usda_path is not None else None

    if not root_prim or not root_prim.IsValid():
        return AssembledRobotDescriptor(
            prim_path=root_path,
            usda_path=usda_str,
            components={},
            articulation_root_path=None,
            link_prim_paths={},
            joint_prim_paths={},
            variant_sets={},
        )

    components = discover_components(stage, root_path, include_inactive_variants=True)
    roots = articulation_roots(stage, root_path)
    art_root = roots[0] if roots else None

    # リンク prim の収集(アクティブな RigidBodyAPI / IsaacSiteAPI / IsaacLinkAPI / robotLinks 参照先)
    link_paths: dict[str, str] = {}
    rel = root_prim.GetRelationship(ROBOT_LINKS)
    rel_targets = {str(t) for t in rel.GetTargets()} if rel else set()
    for p in Usd.PrimRange(root_prim, ALL_PRIMS):
        if not p.IsActive():
            continue
        p_path = str(p.GetPath())
        is_rb = p.HasAPI(UsdPhysics.RigidBodyAPI)
        is_site = has_api(p, "IsaacSiteAPI") or has_api(p, "IsaacLinkAPI")
        is_rel_link = p_path in rel_targets and not has_api(p, "IsaacRobotAPI")
        if not (is_rb or is_site or is_rel_link):
            continue
        leaf = display_name(p)
        comp = _component_prefix_for_path(p_path, root_path, components)
        scoped = f"{comp}/{leaf}" if comp else leaf
        link_paths.setdefault(scoped, p_path)
        link_paths.setdefault(leaf, p_path)

    # アクティブな関節 prim の収集
    joint_paths: dict[str, str] = {}
    for jp in joints_under(stage, root_path):
        if not joint_active(jp):
            continue
        jp_path = str(jp.GetPath())
        leaf = display_name(jp)
        comp = _component_prefix_for_path(jp_path, root_path, components)
        scoped = f"{comp}/{leaf}" if comp else leaf
        joint_paths.setdefault(scoped, jp_path)
        joint_paths.setdefault(leaf, jp_path)

    # 配下の全 prim に定義されている VariantSet の現在選択状態を収集
    vsets: dict[str, dict[str, str]] = {}
    for p in Usd.PrimRange(root_prim, DEFINED_PRIMS):
        vs = p.GetVariantSets()
        names = vs.GetNames()
        if not names:
            continue
        p_map: dict[str, str] = {}
        for vs_name in names:
            p_map[vs_name] = vs.GetVariantSet(vs_name).GetVariantSelection()
        if p_map:
            vsets[str(p.GetPath())] = p_map

    return AssembledRobotDescriptor(
        prim_path=root_path,
        usda_path=usda_str,
        components=components,
        articulation_root_path=art_root,
        link_prim_paths=link_paths,
        joint_prim_paths=joint_paths,
        variant_sets=vsets,
    )


extract_articulation_descriptor = inspect_assembled_robot


def spawn_assembled_robot(
    stage: Usd.Stage,
    usda_path: str | Path,
    prim_path: str | None = None,
) -> AssembledRobotDescriptor:
    """組み立て済み usda を `stage` の `prim_path` に Reference 配置し、検査済みの記述子を返す。

    引数:
      stage: 配置先の `Usd.Stage`(副作用として `prim_path` に `Xform` prim と Reference を追加する)。
      usda_path: 配置する組み立て済み `.usda` のパス。
      prim_path: 配置先 prim パス。省略時は usda の `defaultPrim`(無ければファイル幹名)から `/<name>` を決定する。

    戻り値:
      配置後の prim を `inspect_assembled_robot` で検査した `AssembledRobotDescriptor`。
    """
    usda_str = os.fspath(usda_path)
    if prim_path is None:
        layer = Sdf.Layer.FindOrOpen(usda_str)
        default_prim = layer.defaultPrim if (layer and layer.defaultPrim) else Path(usda_str).stem
        prim_path = f"/{default_prim}"
    prim = stage.DefinePrim(str(prim_path), "Xform")
    prim.GetReferences().AddReference(usda_str)
    return inspect_assembled_robot(stage, str(prim_path), usda_path=usda_str)


__all__ = [
    "CUSTOM_LAYER_KEY",
    "AssembledRobotDescriptor",
    "ArticulationDescriptor",
    "discover_components",
    "components_from_robot_api",
    "inspect_assembled_robot",
    "extract_articulation_descriptor",
    "spawn_assembled_robot",
]
