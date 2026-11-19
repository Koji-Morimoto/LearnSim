"""部品の import と正規化層。

- import は入力(URDF・メッシュ・設定・importer の版)のハッシュが前回と同じなら飛ばす。
- 修正 R13 を入れると、一時フォルダに import してから決まった出力先 _import/<usda の名前>/ を置き換える
  (importer に任せると、出力先が空でなければ <名前>_1 のような別フォルダになる)。
- 正規化層 <output_dir>/<usda>(TOML の [[urdf_to_usda]] の usda)は生成物を参照し、over だけを書く。生成物は触らない。
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
from dataclasses import dataclass

from pxr import Sdf, Usd, UsdPhysics

from .spec import PipelineSpec, UrdfSource
from .urdf_model import UrdfModel
from .usd_util import physics_variant_choice

NORMALIZE_VERSION = "1"  # 正規化層の中身を変えたら上げる(ハッシュに入る)


@dataclass
class PartBuild:
    spec: UrdfSource
    model: UrdfModel
    generated_usd: str  # importer の出力(手を入れない)
    fixed_usd: str  # 正規化層
    imported: bool  # 今回 import したか(False = ハッシュが同じなので飛ばした)
    expected_dir: str  # 決めた出力先
    collision_from_visuals: bool
    physics_variant: str | None  # 正規化層で選んだ variant
    importer_would_suffix: bool  # 公式のまま(importer に出力先を任せる)なら、別名のフォルダに出力したか
    target_was_nonempty: bool  # import の前に、出力先が空でなかったか


def _sha_file(path: str) -> str:
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        h.update(b"<missing>")
    return h.hexdigest()


def _importer_version() -> str:
    try:
        import omni.kit.app

        return omni.kit.app.get_app().get_extension_manager().get_enabled_extension_id("isaacsim.asset.importer.urdf") or "?"
    except Exception:  # noqa: BLE001
        return "?"


def input_hash(part: UrdfSource, model: UrdfModel, cfv: bool) -> str:
    h = hashlib.sha256()
    payload = {
        "urdf": model.text_sha256,
        "meshes": {os.path.basename(m): _sha_file(m) for m in model.mesh_files(part.ros_package_paths)},
        "cfv": cfv, "collision_type": part.collision_type, "ros": list(part.ros_package_paths),
        "stiffness": part.joint_drive_stiffness, "damping": part.joint_drive_damping, "normalize": NORMALIZE_VERSION,
        "importer": _importer_version(),
    }
    h.update(json.dumps(payload, sort_keys=True).encode())
    return h.hexdigest()


def _write_if_changed(layer: Sdf.Layer, path: str) -> bool:
    text = layer.ExportToString()
    try:
        with open(path, encoding="utf-8") as f:
            if f.read() == text:
                return False
    except OSError:
        pass
    layer.Save()
    return True


def open_for_rewrite(path: str) -> Sdf.Layer:
    """path のレイヤーを空にして返す(毎回まるごと作り直すため)。中身が同じならファイルは書き換えない。"""
    layer = Sdf.Layer.FindOrOpen(path) if os.path.exists(path) else None
    if layer is None:
        layer = Sdf.Layer.CreateNew(path)
    else:
        layer.Clear()
    return layer


def build_part(spec: PipelineSpec, part: UrdfSource, model: UrdfModel) -> PartBuild:
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.asset.importer.utils import importer_utils

    parts_dir = os.path.join(spec.output_dir, "_import")  # URDF Importer の生成物を置く場所
    os.makedirs(parts_dir, exist_ok=True)
    cfv = part.collision_from_visuals if part.collision_from_visuals is not None else not model.has_collisions()
    stem = os.path.splitext(os.path.basename(part.path))[0]
    expected_dir = os.path.join(parts_dir, part.stem)
    manifest_path = os.path.join(parts_dir, f"{part.stem}.manifest.json")
    digest = input_hash(part, model, cfv)
    target_nonempty = os.path.isdir(expected_dir) and any(os.scandir(expected_dir))
    # importer にそのまま出力させた場合の出力先(公式のままの比較用。importer 自身の関数で求める)
    would = importer_utils.resolve_unique_path(os.path.join(parts_dir, part.stem), is_file=False)
    importer_would_suffix = os.path.normcase(os.path.abspath(would)) != os.path.normcase(os.path.abspath(expected_dir))

    manifest = {}
    if os.path.exists(manifest_path):
        with open(manifest_path, encoding="utf-8") as f:
            manifest = json.load(f)
    generated = manifest.get("generated_usd")
    reuse = manifest.get("hash") == digest and generated and os.path.exists(generated)

    cfg = dict(urdf_path=part.path, collision_from_visuals=cfv, collision_type=part.collision_type,
               ros_package_paths=[{"name": n, "path": p} for n, p in part.ros_package_paths],
               override_joint_stiffness=part.joint_drive_stiffness, override_joint_damping=part.joint_drive_damping)
    imported = False
    if not reuse:
        if spec.fix_applied("R13"):
            staging = os.path.join(spec.output_dir, ".staging", part.stem)
            shutil.rmtree(staging, ignore_errors=True)
            os.makedirs(staging)
            out = URDFImporter(URDFImporterConfig(usd_path=staging, **cfg)).import_urdf()
            shutil.rmtree(expected_dir, ignore_errors=True)
            shutil.move(os.path.dirname(out), expected_dir)
            shutil.rmtree(staging, ignore_errors=True)
            generated = os.path.join(expected_dir, os.path.basename(out))
        else:
            # 修正なし:importer の既定どおり _import/ の下に出させる(出力先が空でなければ別名になる)
            out = URDFImporter(URDFImporterConfig(usd_path=parts_dir, **cfg)).import_urdf()
            generated = out
        imported = True
        from .usd_util import reload_layers_under

        reload_layers_under(os.path.dirname(generated))  # 同じプロセスで前回読んだ古い中身を捨てる
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump({"hash": digest, "generated_usd": generated, "stem": stem}, f, indent=1)

    fixed_usd, variant = write_normalized(spec, part, generated)
    return PartBuild(spec=part, model=model, generated_usd=generated, fixed_usd=fixed_usd, imported=imported,
                     expected_dir=expected_dir, collision_from_visuals=cfv, physics_variant=variant,
                     importer_would_suffix=importer_would_suffix, target_was_nonempty=target_nonempty)


def write_normalized(spec: PipelineSpec, part: UrdfSource, generated: str) -> tuple[str, str | None]:
    """<output_dir>/<usda> に、生成物を参照し、Physics variant の選択(R06)と mimic follower の drive を 0 にする(R03)over を書く。

    一番上の prim の名前は usda のファイル名の .usda より前(URDF の名前とは限らない)。
    """
    path = os.path.join(spec.output_dir, part.usda)
    src_layer = Sdf.Layer.FindOrOpen(generated)
    layer = open_for_rewrite(path)
    layer.customLayerData = {"myext.robot.assembly_pipeline": {"normalize_version": NORMALIZE_VERSION, "urdf": os.path.basename(part.urdf)}}
    stage = Usd.Stage.Open(layer)
    from pxr import UsdGeom

    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(Usd.Stage.Open(src_layer)))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(Usd.Stage.Open(src_layer)))
    prim = stage.DefinePrim(f"/{part.stem}", "Xform")
    rel = os.path.relpath(generated, spec.output_dir).replace("\\", "/")
    prim.GetReferences().AddReference(rel if rel.startswith(".") else "./" + rel)
    stage.SetDefaultPrim(prim)
    variant = None
    if spec.fix_applied("R06"):
        variant = physics_variant_choice(prim, part.physics_variant)
        if variant:
            prim.GetVariantSet("Physics").SetVariantSelection(variant)
    if spec.fix_applied("R03"):
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if not p.IsA(UsdPhysics.Joint):
                continue
            schemas = p.GetAppliedSchemas()
            if not ("NewtonMimicAPI" in schemas or any(s.startswith("PhysxMimicJointAPI") for s in schemas)):
                continue
            for kind in ("angular", "linear"):
                if p.HasAPI(UsdPhysics.DriveAPI, kind):
                    d = UsdPhysics.DriveAPI(p, kind)
                    d.CreateStiffnessAttr().Set(0.0)
                    d.CreateDampingAttr().Set(0.0)
    _write_if_changed(layer, path)
    return path, variant
