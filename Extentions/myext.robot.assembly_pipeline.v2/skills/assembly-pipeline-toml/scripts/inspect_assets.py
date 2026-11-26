"""assembly_pipeline.v2 の TOML 生成前に URDF / USD アセットを実地検査する補助スクリプト。

使い方（すべて SimulationApp 起動不要・`kit\\python\\python.exe` 単体で動作）:
  # 1. URDF のリンク・関節・リミット・mimic・collision・package:// を検査
  C:\\isaacsim-6.2.0\\kit\\python\\python.exe inspect_assets.py path/to/robot.urdf [path/to/gripper.urdf ...]

  # 2. USD / USDA の defaultPrim・VariantSet・剛体・リンク prim を検査（同梱 omni.usd.libs を自動ロード）
  C:\\isaacsim-6.2.0\\kit\\python\\python.exe inspect_assets.py path/to/robot.usd

  # 3. 生成した TOML の JSON Schema + #:schema 検証を実行
  C:\\isaacsim-6.2.0\\kit\\python\\python.exe inspect_assets.py --validate path/to/spec.toml
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ISAAC_ROOT = Path(r"C:\isaacsim-6.2.0")
EXT_ROOT = ISAAC_ROOT / r"extsUser\Extentions\myext.robot.assembly_pipeline.v2"
PACKAGE_RE = re.compile(r"^package://([^/]+)/")
_DLL_HANDLES: list[object] = []


def _bootstrap_isaac_pxr() -> None:
    """SimulationApp を起動せずに Isaac Sim 同梱の pxr (omni.usd.libs) を読み込めるようにする。"""
    matches = sorted(glob.glob(str(ISAAC_ROOT / "extscache" / "omni.usd.libs-*")))
    if not matches:
        return
    base = matches[-1]
    dll_dirs = [os.path.join(base, "bin"), str(ISAAC_ROOT / "kit"), str(ISAAC_ROOT / "kit" / "python")]
    existing = [d for d in dll_dirs if os.path.isdir(d)]
    for d in existing:
        if hasattr(os, "add_dll_directory"):
            _DLL_HANDLES.append(os.add_dll_directory(d))
    os.environ["PATH"] = os.pathsep.join(existing) + os.pathsep + os.environ.get("PATH", "")
    # Host PYTHONPATH (BuiltOpenUSD 等) より Isaac Sim 同梱 pxr を優先させる
    sys.path = [p for p in sys.path if "builtopenusd" not in p.lower()]
    if base not in sys.path:
        sys.path.insert(0, base)


def _bootstrap_isaac_pip() -> None:
    """kit\\python\\python.exe 単体実行時でも numpy 等を解決できるよう pip_prebundle を追加する。"""
    for pat in ("omni.kit.pip_archive-*", "omni.pip.compute-*"):
        for m in sorted(glob.glob(str(ISAAC_ROOT / "extscache" / pat / "pip_prebundle"))):
            if m not in sys.path:
                sys.path.append(m)


def _parse_float(val: str | None, default: float = 0.0) -> float:
    if val is None:
        return default
    try:
        return float(val)
    except ValueError:
        return default


def inspect_urdf(path: Path) -> None:
    """URDF XML を純 Python で解析し、TOML 作成に必要な情報を一覧表示する。"""
    tree = ET.parse(path)
    root = tree.getroot()
    robot_name = root.attrib.get("name", "(unnamed)")

    links: list[str] = []
    ghost_links: list[str] = []
    has_any_collision = False
    packages: set[str] = set()

    for link in root.findall("link"):
        lname = link.attrib["name"]
        links.append(lname)
        visuals = link.findall("visual")
        collisions = link.findall("collision")
        if collisions:
            has_any_collision = True
        mass_el = link.find("./inertial/mass")
        mass = _parse_float(mass_el.attrib.get("value") if mass_el is not None else None, 0.0)
        if not visuals and not collisions and mass <= 0.0:
            ghost_links.append(lname)
        for mesh in link.findall(".//mesh"):
            fn = mesh.attrib.get("filename", "")
            m = PACKAGE_RE.match(fn)
            if m:
                packages.add(m.group(1))

    parents: set[str] = set()
    children: set[str] = set()
    joints_info: list[str] = []
    mimic_info: list[str] = []

    for j in root.findall("joint"):
        jname = j.attrib["name"]
        jtype = j.attrib.get("type", "fixed")
        p_el = j.find("parent")
        c_el = j.find("child")
        plink = p_el.attrib["link"] if p_el is not None else "?"
        clink = c_el.attrib["link"] if c_el is not None else "?"
        parents.add(plink)
        children.add(clink)

        lim_el = j.find("limit")
        lim_str = ""
        if lim_el is not None and jtype in ("revolute", "prismatic"):
            lo = lim_el.attrib.get("lower")
            hi = lim_el.attrib.get("upper")
            if lo is not None and hi is not None:
                lim_str = f" limit=[{lo}, {hi}]"

        mimic_el = j.find("mimic")
        mimic_str = ""
        if mimic_el is not None:
            leader = mimic_el.attrib.get("joint", "?")
            mult = mimic_el.attrib.get("multiplier", "1")
            off = mimic_el.attrib.get("offset", "0")
            mimic_str = f" mimic(leader={leader}, mult={mult}, offset={off})"
            mimic_info.append(f"{jname} -> {leader} (x{mult} + {off})")

        if jtype != "fixed":
            joints_info.append(f"  - {jname} ({jtype}): {plink} -> {clink}{lim_str}{mimic_str}")

    root_links = [l for l in links if l not in children]
    leaf_links = [l for l in links if l not in parents]

    print(f"=== URDF: {path} (robot={robot_name}) ===")
    print(f"  リンク総数          : {len(links)}")
    print(f"  ルートリンク候補    : {root_links}")
    print(f"  末端リンク候補      : {leaf_links}")
    print(f"  ゴーストリンク      : {ghost_links}")
    print(f"  <collision> の有無  : {has_any_collision} (collision_from_visuals 省略時の実効値: {not has_any_collision})")
    print(f"  package:// 参照     : {sorted(packages) if packages else 'なし'}")
    print(f"  全リンク一覧        : {links}")
    print(f"  可動関節 ({len(joints_info)} 件):")
    if joints_info:
        print("\n".join(joints_info))
    else:
        print("  - なし")
    if mimic_info:
        print(f"  mimic 関係          : {mimic_info}")
    print()


def inspect_usd(path_str: str) -> None:
    """OpenUSD (pxr) で USD / USDA アセットの defaultPrim・VariantSet・剛体・リンク候補を一覧表示する。"""
    _bootstrap_isaac_pxr()
    try:
        from pxr import Usd, UsdPhysics
    except ImportError as exc:
        print(
            f"[ERROR] USD 検査に必要な pxr の読み込みに失敗しました: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(2) from None

    stage = Usd.Stage.Open(path_str)
    if stage is None:
        print(f"[ERROR] USD を開けません: {path_str}", file=sys.stderr)
        raise SystemExit(2)

    dp = stage.GetDefaultPrim()
    top_prim = dp if (dp and dp.IsValid()) else stage.GetPseudoRoot()
    top_path = str(top_prim.GetPath())

    vsets: dict[str, tuple[str, list[str]]] = {}
    if dp and dp.IsValid():
        vs = dp.GetVariantSets()
        for name in vs.GetNames():
            vset = vs.GetVariantSet(name)
            vsets[name] = (vset.GetVariantSelection(), list(vset.GetVariantNames()))

    rigid_bodies: list[str] = []
    art_roots: list[str] = []
    joints: list[str] = []

    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for p in Usd.PrimRange(top_prim, pred):
        p_path = str(p.GetPath())
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(p_path)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_roots.append(p_path)
        if p.IsA(UsdPhysics.Joint):
            joints.append(f"{p_path} ({p.GetTypeName()})")

    print(f"=== USD: {path_str} ===")
    print(f"  defaultPrim         : {str(dp.GetPath()) if (dp and dp.IsValid()) else '(未設定 — prim_path 指定が必要)'}")
    print(f"  defaultPrim VariantSets:")
    if vsets:
        for k, (sel, names) in vsets.items():
            print(f"    - {k}: 現在='{sel}', 選択肢={names}")
    else:
        print("    - なし")
    print(f"  ArticulationRoot    : {art_roots}")
    print(f"  剛体 (リンク候補 {len(rigid_bodies)} 件):")
    for rb in rigid_bodies:
        rel = rb[len(top_path) + 1 :] if rb.startswith(top_path + "/") else rb
        print(f"    - {os.path.basename(rb)}  (相対パス: {rel})")
    print(f"  関節 ({len(joints)} 件):")
    for j in joints[:30]:
        print(f"    - {j}")
    if len(joints) > 30:
        print(f"    ... 他 {len(joints) - 30} 件")
    print()


def validate_toml(toml_path: Path) -> int:
    """`myext.robot.assembly_pipeline_v2.schema.validate_toml_file` で TOML をスキーマ検証する。"""
    _bootstrap_isaac_pip()
    if str(EXT_ROOT) not in sys.path:
        sys.path.insert(0, str(EXT_ROOT))
    from myext.robot.assembly_pipeline_v2.schema import DEFAULT_SCHEMA_PATH, validate_toml_file

    errs = validate_toml_file(toml_path)
    first_line = toml_path.read_text(encoding="utf-8").splitlines()[0].strip() if toml_path.exists() else ""
    if not first_line.startswith("#:schema "):
        errs.append("1 行目に #:schema <相対パス>/schema/assembly_pipeline.schema.json がありません")
    else:
        rel = first_line[len("#:schema ") :].strip()
        resolved = (toml_path.parent / rel).resolve()
        if resolved != DEFAULT_SCHEMA_PATH.resolve():
            errs.append(f"#:schema の参照先がスキーマ本体と一致しません: {resolved} != {DEFAULT_SCHEMA_PATH.resolve()}")

    if errs:
        print(f"[SCHEMA NG] {toml_path}:")
        for e in errs:
            print(f"  - {e}")
        return 1
    print(f"[SCHEMA OK] {toml_path} (#:schema と JSON Schema 検証を通過)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="assembly_pipeline.v2 TOML 作成用のアセット検査・スキーマ検証ツール")
    parser.add_argument("targets", nargs="*", help="検査する URDF または USD ファイルのパス / URI")
    parser.add_argument("--validate", type=Path, help="指定した TOML ファイルを JSON Schema で検証する")
    args = parser.parse_args()

    if args.validate is not None:
        return validate_toml(args.validate)

    if not args.targets:
        parser.print_help()
        return 1

    for t in args.targets:
        lower = t.lower()
        if lower.endswith(".urdf") or lower.endswith(".xml"):
            inspect_urdf(Path(t))
        elif lower.endswith((".usd", ".usda", ".usdc", ".usdz")) or "://" in t:
            inspect_usd(t)
        else:
            print(f"[SKIP] 未対応拡張子: {t}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
