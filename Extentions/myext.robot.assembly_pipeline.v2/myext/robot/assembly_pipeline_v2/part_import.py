"""部品の import と正規化層の生成を担うモジュール。

責務:
  1. URDF 部品を Isaac Sim の URDF Importer で USD へ変換する（入力ハッシュ一致時はキャッシュ再利用）。
  2. 規則 R13 が有効な場合、一時フォルダ経由で既定の出力先 ``_import/<usda名>/`` を置換し、
     importer 既定の連番ディレクトリ（``<名前>_1`` など）発生を防ぐ。
  3. 生成済み USD（または既存の USD アセット）を直接編集せず、Reference と over のみを持つ
     正規化層 ``<output_dir>/<usda>`` を書き出す（R06: Physics variant 選択、R03: mimic follower の drive 無効化）。

主な公開 API:
  - ``PartBuild``: 1 部品ぶんの import・正規化結果レコード
  - ``input_hash(part, model, cfv)``: 再 import 要否を判定する入力ハッシュ計算
  - ``build_part(spec, part, model=None)`` / ``import_urdf``: URDF 部品の import と正規化層生成
  - ``write_normalized(spec, part, generated)``: URDF 生成物に対する正規化層（over）の書き出し
  - ``build_usd_part(spec, src)``: 既存 USD アセットに対する正規化層の書き出し

パイプライン全体での位置づけ:
  ``pipeline.build_assembly`` の Step 2（URDF 事前チェックの後、静的部品検査および ``compose.compose_file`` の前）
  で呼ばれ、後段が参照する正規化済み ``.usda`` ファイルを生成する。

手を入れやすい箇所:
  - 正規化層に追加の over 処理を入れたい場合は ``write_normalized``（URDF 由来）または
    ``build_usd_part``（USD 由来）を修正し、キャッシュを確実に更新するため ``NORMALIZE_VERSION`` を上げる。
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
from dataclasses import dataclass
from typing import Any

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .descriptor import CUSTOM_LAYER_KEY
from .spec import PipelineSpec, UrdfSource, UsdSource
from .urdf_model import UrdfModel, load_urdf
from .usd_util import (
    ALL_PRIMS,
    _write_if_changed,
    asset_ref_path,
    has_api,
    open_for_rewrite,
    physics_variant_choice,
    reload_layers_under,
    select_physics_variant,
    write_if_changed,
)

NORMALIZE_VERSION = "1"  # 正規化層の中身を変えたら上げる(ハッシュに入る)


@dataclass
class PartBuild:
    """1 部品ぶんの import・正規化結果を保持するデータクラス。

    後段の ``probes.static_part`` や ``rules`` の事前チェック・事後確認（特に R13）で利用される。
    互換性のため ``__getitem__`` / ``get`` による辞書風アクセスも提供する。
    """

    spec: UrdfSource | UsdSource
    model: UrdfModel | None
    generated_usd: str  # importer の出力または既存 USD の入力パス（直接編集しない）
    fixed_usd: str  # 正規化層（<output_dir>/<usda>）のパス
    imported: bool  # 今回 import を実行したか（False = ハッシュ一致で再利用、または USD 入力）
    expected_dir: str  # R13 適用時に期待する出力ディレクトリ（_import/<stem>）
    collision_from_visuals: bool  # 実際に適用した collision_from_visuals の値
    physics_variant: str | None  # 正規化層で選択した Physics variant 名
    importer_would_suffix: bool  # 公式 importer 既定ロジックなら別名フォルダになる状態だったか（R13 比較用）
    target_was_nonempty: bool  # import 開始時点で expected_dir が空でなかったか（R13 事前判定用）
    source_kind: str = "urdf"  # "urdf" | "usd"

    def to_dict(self) -> dict[str, Any]:
        """全フィールドを辞書として返す。"""
        return {
            "spec": self.spec,
            "model": self.model,
            "generated_usd": self.generated_usd,
            "fixed_usd": self.fixed_usd,
            "imported": self.imported,
            "expected_dir": self.expected_dir,
            "collision_from_visuals": self.collision_from_visuals,
            "physics_variant": self.physics_variant,
            "importer_would_suffix": self.importer_would_suffix,
            "target_was_nonempty": self.target_was_nonempty,
            "source_kind": self.source_kind,
        }

    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)


def _sha_file(path: str) -> str:
    """ファイルの SHA-256 を返す。存在しない場合は ``<missing>`` のハッシュで安定させる。"""
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        h.update(b"<missing>")
    return h.hexdigest()


def _importer_version() -> str:
    """現在有効な ``isaacsim.asset.importer.urdf`` の拡張 ID（バージョン込み）を返す。"""
    try:
        import omni.kit.app

        return (
            omni.kit.app.get_app()
            .get_extension_manager()
            .get_enabled_extension_id("isaacsim.asset.importer.urdf")
            or "?"
        )
    except Exception:  # noqa: BLE001
        return "?"


def input_hash(part: UrdfSource, model: UrdfModel, cfv: bool) -> str:
    """URDF 部品の再 import 要否を判定する SHA-256 ダイジェストを計算する。

    URDF 本文・参照メッシュ・importer 設定・``NORMALIZE_VERSION``・importer のバージョンを
    すべて含めることで、いずれかが変化したときだけ再 import が走るようにする。

    Args:
        part: TOML の ``[[urdf_to_usda]]`` 設定。
        model: パース済み ``UrdfModel``。
        cfv: 実際に使用する ``collision_from_visuals`` フラグ。

    Returns:
        16 進 SHA-256 ハッシュ文字列。副作用なし。
    """
    h = hashlib.sha256()
    payload = {
        "urdf": model.text_sha256,
        "meshes": {os.path.basename(m): _sha_file(m) for m in model.mesh_files(part.ros_package_paths)},
        "cfv": cfv,
        "collision_type": part.collision_type,
        "ros": list(part.ros_package_paths),
        "stiffness": part.joint_drive_stiffness,
        "damping": part.joint_drive_damping,
        "normalize": NORMALIZE_VERSION,
        "importer": _importer_version(),
    }
    if part.fix_base is not None:
        payload["fix_base"] = part.fix_base
    h.update(json.dumps(payload, sort_keys=True).encode())
    return h.hexdigest()


def build_part(spec: PipelineSpec, part: UrdfSource, model: UrdfModel | None = None) -> PartBuild:
    """URDF 部品を USD へ import し、正規化層 ``<output_dir>/<part.usda>`` を生成する。

    前回 import 時の ``_import/<stem>.manifest.json`` と ``input_hash`` を比較し、
    ハッシュ一致かつ生成 USD が現存する場合は重い URDF Importer の実行を省略する。
    正規化層は毎回軽量に生成し、内容が変わった場合のみディスクへ書き戻す。

    Args:
        spec: パイプライン全体仕様（出力先ディレクトリと規則の有効/無効を参照）。
        part: 対象 URDF 部品の仕様。
        model: 事前ロード済み ``UrdfModel``（省略時は内部で ``load_urdf`` する）。

    Returns:
        import 結果と正規化層パスをまとめた ``PartBuild``。

    副作用:
        ``<output_dir>/_import/`` 配下へ USD・マニフェストを書き出し、
        ``<output_dir>/<part.usda>`` を更新し、関連 SdfLayer キャッシュをリロードする。
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.asset.importer.utils import importer_utils

    if model is None:
        model = load_urdf(part.path)

    parts_dir = os.path.join(spec.output_dir, "_import")  # URDF Importer の生成物を置く場所
    os.makedirs(parts_dir, exist_ok=True)
    # TOML で明示指定が無ければ、URDF に <collision> が 1 つも無いときだけ visual を collision に流用する
    cfv = part.collision_from_visuals if part.collision_from_visuals is not None else not model.has_collisions()
    stem = os.path.splitext(os.path.basename(part.path))[0]
    expected_dir = os.path.join(parts_dir, part.stem)
    manifest_path = os.path.join(parts_dir, f"{part.stem}.manifest.json")
    digest = input_hash(part, model, cfv)
    target_nonempty = os.path.isdir(expected_dir) and any(os.scandir(expected_dir))
    # R13 の事後比較用に、公式 importer のパス解決関数を直接呼んで「別名フォルダになるか」を事前記録する
    would = importer_utils.resolve_unique_path(os.path.join(parts_dir, part.stem), is_file=False)
    importer_would_suffix = os.path.normcase(os.path.abspath(would)) != os.path.normcase(os.path.abspath(expected_dir))

    manifest: dict[str, Any] = {}
    if os.path.exists(manifest_path):
        with open(manifest_path, encoding="utf-8") as f:
            manifest = json.load(f)
    generated = manifest.get("generated_usd")
    reuse = bool(manifest.get("hash") == digest and generated and os.path.exists(generated))

    cfg = dict(
        urdf_path=part.path,
        collision_from_visuals=cfv,
        collision_type=part.collision_type,
        ros_package_paths=[{"name": n, "path": p} for n, p in part.ros_package_paths],
        override_joint_stiffness=part.joint_drive_stiffness,
        override_joint_damping=part.joint_drive_damping,
    )
    if part.fix_base is not None:
        cfg["fix_base"] = part.fix_base
    imported = False
    if not reuse:
        if spec.fix_applied("R13"):
            # R13: URDF Importer は既存ディレクトリがあると _1 等の連番フォルダを勝手に作るため、
            # 空の一時ステージング領域(.staging/)へ import してから expected_dir を丸ごと置換する
            staging = os.path.join(spec.output_dir, ".staging", part.stem)
            shutil.rmtree(staging, ignore_errors=True)
            os.makedirs(staging)
            out = URDFImporter(URDFImporterConfig(usd_path=staging, **cfg)).import_urdf()
            shutil.rmtree(expected_dir, ignore_errors=True)
            shutil.move(os.path.dirname(out), expected_dir)
            shutil.rmtree(staging, ignore_errors=True)
            generated = os.path.join(expected_dir, os.path.basename(out))
        else:
            # 修正なし: importer の既定どおり _import/ の下に出させる（出力先が空でなければ別名フォルダになる）
            out = URDFImporter(URDFImporterConfig(usd_path=parts_dir, **cfg)).import_urdf()
            generated = out
        imported = True
        # 同一 Kit プロセス内で Sdf.Layer キャッシュに古い生成物が残ると後段が旧データを参照するため破棄する
        reload_layers_under(os.path.dirname(generated))
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump({"hash": digest, "generated_usd": generated, "stem": stem}, f, indent=1)

    fixed_usd, variant = write_normalized(spec, part, generated)
    return PartBuild(
        spec=part,
        model=model,
        generated_usd=generated,
        fixed_usd=fixed_usd,
        imported=imported,
        expected_dir=expected_dir,
        collision_from_visuals=cfv,
        physics_variant=variant,
        importer_would_suffix=importer_would_suffix,
        target_was_nonempty=target_nonempty,
    )


import_urdf = build_part


def _zero_mimic_follower_drives(prim: Usd.Prim) -> None:
    """R03: mimic follower 関節の ``UsdPhysics.DriveAPI`` の stiffness / damping を 0 に戻す。

    URDF Importer が全可動関節に既定の drive ゲインを一律付与するため、mimic follower 側にも
    drive が付くと leader 側の動きを妨害する。follower 側の駆動をゼロ化して制約追従のみにする。
    """
    for p in Usd.PrimRange(prim, ALL_PRIMS):
        if not p.IsA(UsdPhysics.Joint):
            continue
        if not (has_api(p, "NewtonMimicAPI") or has_api(p, "PhysxMimicJointAPI")):
            continue
        for kind in ("angular", "linear"):
            if p.HasAPI(UsdPhysics.DriveAPI, kind):
                d = UsdPhysics.DriveAPI(p, kind)
                d.CreateStiffnessAttr().Set(0.0)
                d.CreateDampingAttr().Set(0.0)


def write_normalized(spec: PipelineSpec, part: UrdfSource, generated: str) -> tuple[str, str | None]:
    """URDF 生成物を Reference する正規化層 ``<output_dir>/<part.usda>`` を書き出す。

    生成物 USD 自体は一切変更せず、正規化層に over のみを書くことで再 import や公式比較と
    両立させる。適用する修正は R06（Physics variant の明示選択）と R03（mimic follower の drive 無効化）。
    一番上の prim の名前は ``part.stem``（usda ファイル名の拡張子前）になる。

    Args:
        spec: パイプライン全体仕様。
        part: 対象 URDF 部品仕様。
        generated: URDF Importer が出力した生成物 USD パス。

    Returns:
        ``(正規化層ファイルパス, 選択した Physics variant 名または None)``。

    副作用:
        内容に差分がある場合のみ ``<output_dir>/<part.usda>`` をディスクに書き出す。
    """
    path = os.path.join(spec.output_dir, part.usda)
    src_layer = Sdf.Layer.FindOrOpen(generated)
    layer = open_for_rewrite(path)
    layer.customLayerData = {
        CUSTOM_LAYER_KEY: {"normalize_version": NORMALIZE_VERSION, "urdf": os.path.basename(part.urdf)}
    }
    stage = Usd.Stage.Open(layer)
    src_stage = Usd.Stage.Open(src_layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src_stage))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src_stage))
    prim = stage.DefinePrim(f"/{part.stem}", "Xform")
    # 出力ディレクトリを別場所へ移しても参照が壊れないよう、正規化層からの相対パスで参照する
    rel = os.path.relpath(generated, spec.output_dir).replace("\\", "/")
    prim.GetReferences().AddReference(rel if rel.startswith(".") else "./" + rel)
    stage.SetDefaultPrim(prim)
    variant = None
    if spec.fix_applied("R06"):
        # R06: Importer 出力の Physics VariantSet は未選択のままになることがあるため明示選択する
        variant = physics_variant_choice(prim, part.physics_variant)
        if variant:
            prim.GetVariantSet("Physics").SetVariantSelection(variant)
    if spec.fix_applied("R03"):
        _zero_mimic_follower_drives(prim)
    write_if_changed(layer, path)
    return path, variant


def build_usd_part(spec: PipelineSpec, src: UsdSource) -> PartBuild:
    """既存の USD アセットを正規化層 ``<output_dir>/<src.usda>`` にラップする。

    URDF Importer を介さず、既存 USD（ローカルまたは Nucleus / HTTP 等の URI）を Reference し、
    TOML 指定の ``variant_selections``・R06（Physics variant 補完）・R03（mimic follower drive ゼロ化）
    を over として書き出す。

    Args:
        spec: パイプライン全体仕様。
        src: TOML の ``[[usd_to_usda]]`` 設定。

    Returns:
        正規化結果の ``PartBuild``（``model=None``, ``source_kind="usd"``）。

    Raises:
        FileNotFoundError: 入力 USD アセットを開けない場合。
        ValueError: 指定した ``prim_path``・``defaultPrim``・VariantSet・選択肢が存在しない場合。

    副作用:
        内容に差分がある場合のみ ``<output_dir>/<src.usda>`` をディスクに書き出す。
    """
    os.makedirs(spec.output_dir, exist_ok=True)
    src_stage = Usd.Stage.Open(src.usd)
    if src_stage is None:
        raise FileNotFoundError(f"USD アセットを開けない: {src.usd!r}")
    if src.prim_path is not None:
        target_prim = src_stage.GetPrimAtPath(src.prim_path)
        if not target_prim or not target_prim.IsValid():
            raise ValueError(f"[[usd_to_usda]] usda = {src.usda!r}: {src.usd} に prim_path {src.prim_path!r} が無い")
    else:
        target_prim = src_stage.GetDefaultPrim()
        if not target_prim or not target_prim.IsValid():
            raise ValueError(
                f"[[usd_to_usda]] usda = {src.usda!r}: {src.usd} に defaultPrim が無い(prim_path を指定してください)"
            )

    path = os.path.join(spec.output_dir, src.usda)
    layer = open_for_rewrite(path)
    layer.customLayerData = {
        CUSTOM_LAYER_KEY: {
            "normalize_version": NORMALIZE_VERSION,
            "source_kind": "usd",
            "usd": src.usd if "://" in src.usd else os.path.basename(src.usd),
        }
    }
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src_stage))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src_stage))

    prim = stage.DefinePrim(f"/{src.stem}", "Xform")
    ref_path = asset_ref_path(src.usd, str(spec.output_dir))
    if src.prim_path is not None:
        prim.GetReferences().AddReference(ref_path, Sdf.Path(src.prim_path))
    else:
        prim.GetReferences().AddReference(ref_path)
    stage.SetDefaultPrim(prim)

    # TOML で明示指定された VariantSet を検証しながら適用する（誤記を早期に ValueError で検出）
    selected_vs: dict[str, str] = {}
    for vs_name, choice in src.variant_selections:
        vsets = prim.GetVariantSets()
        if not vsets.HasVariantSet(vs_name):
            raise ValueError(
                f"[[usd_to_usda]] usda = {src.usda!r}: {src.usd} に VariantSet {vs_name!r} が無い。"
                f"ある VariantSet: {sorted(vsets.GetNames())}"
            )
        vset = vsets.GetVariantSet(vs_name)
        vnames = list(vset.GetVariantNames())
        if choice not in vnames:
            raise ValueError(
                f"[[usd_to_usda]] usda = {src.usda!r}: VariantSet {vs_name!r} に選択肢 {choice!r} が無い。"
                f"ある選択肢: {vnames}"
            )
        vset.SetVariantSelection(choice)
        selected_vs[vs_name] = choice

    # 既存 USD アセットがすでに有効な Physics variant を選択済みの場合は keep_existing=True で尊重する
    variant: str | None = selected_vs.get("Physics")
    if "Physics" not in selected_vs and spec.fix_applied("R06"):
        variant = select_physics_variant(prim, src.physics_variant, keep_existing=True)

    if spec.fix_applied("R03"):
        _zero_mimic_follower_drives(prim)

    write_if_changed(layer, path)
    return PartBuild(
        spec=src,
        model=None,
        generated_usd=src.usd,
        fixed_usd=path,
        imported=False,
        expected_dir=os.path.dirname(path),
        collision_from_visuals=False,
        physics_variant=variant,
        importer_would_suffix=False,
        target_was_nonempty=False,
        source_kind="usd",
    )


__all__ = [
    "NORMALIZE_VERSION",
    "PartBuild",
    "input_hash",
    "open_for_rewrite",
    "write_if_changed",
    "_write_if_changed",
    "build_part",
    "build_usd_part",
    "import_urdf",
    "write_normalized",
]
