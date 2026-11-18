"""結合ロボットから GeometricRobot(myext.robot.geometric)を作る部品と、PhysX との順運動学の突き合わせ。

構成要素(components)は IsaacRobotAPI から自動で作る:結合した子の usda の一番上の prim には IsaacRobotAPI が残るので、
「結合ファイルの一番上の prim」と「その下の IsaacRobotAPI の prim」をそれぞれ 1 つの構成要素にする
(どのリンクも、いちばん深い IsaacRobotAPI の prim の構成要素になる)。
構成要素の名前は、上にある IsaacRobotAPI の prim の名前を "/" でつないだもの(例: arm1、arm1/end_effector)。
"""

from __future__ import annotations

import numpy as np
from pxr import Usd, UsdPhysics

from . import runtime
from .usd_util import ALL_PRIMS, display_name


def components_from_robot_api(stage: Usd.Stage, robot_path: str) -> dict[str, str]:
    """構成要素名 -> prim パス。一番上はその prim の名前、その下は IsaacRobotAPI の prim の名前を "/" でつないだもの。"""
    out = {display_name(stage.GetPrimAtPath(robot_path)): robot_path}
    names: dict[str, str] = {robot_path: ""}
    for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path), ALL_PRIMS):
        path = str(p.GetPath())
        if path == robot_path or not p.HasAPI("IsaacRobotAPI"):
            continue
        parent = max((q for q in names if path.startswith(q + "/")), key=len)
        name = f"{names[parent]}/{p.GetName()}" if names[parent] else p.GetName()
        while name in out:
            name += "_"
        names[path] = name
        out[name] = path
    return out


def make_geometric_robot(stage: Usd.Stage, robot_path: str, **kwargs):
    """IsaacRobotAPI から構成要素を作って GeometricRobot.from_articulation する。"""
    from myext.robot.geometric import GeometricRobot, UsdArticulationSource

    source = UsdArticulationSource(robot_path, components=components_from_robot_api(stage, robot_path))
    return GeometricRobot.from_articulation(stage, source, **kwargs)


def check_against_physx(composed_usd: str, robot_name: str, n_poses: int = 3, seed: int = 0) -> dict:
    """新しいステージに結合ロボットを置き、同じ関節値で GeometricRobot と PhysX のリンク姿勢を比べる。"""
    try:
        import omni.kit.app

        mgr = omni.kit.app.get_app().get_extension_manager()
        if not mgr.is_extension_enabled("myext.robot.geometric"):
            mgr.set_extension_enabled_immediate("myext.robot.geometric", True)
        from myext.robot.geometric import GeometricRobot  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        return {"error": f"myext.robot.geometric を読めない: {exc}"}
    import omni.physics.tensors as tensors
    import omni.usd

    stage = runtime.new_probe_stage(gravity=0.0)
    robot_path = f"/{robot_name}"
    stage.DefinePrim(robot_path, "Xform").GetReferences().AddReference(composed_usd)
    runtime.update(2)
    out: dict = {"components": components_from_robot_api(stage, robot_path)}
    geo = make_geometric_robot(stage, robot_path)
    rng = np.random.default_rng(seed)
    lo, hi = geo.get_dof_limits()
    lo = np.where(np.isfinite(lo), lo, -0.5)
    hi = np.where(np.isfinite(hi), hi, 0.5)
    qs = [lo + (hi - lo) * rng.uniform(0.2, 0.8, len(lo)) for _ in range(n_poses)]
    names, paths = geo.dof_names, dict(geo.link_path_map)
    expected = []
    for q in qs:
        geo.set_dof_positions(q)
        full = geo.get_dof_positions()  # mimic で決まる値も含む
        mats = geo.get_link_world_matrices()
        expected.append((full, {n: mats[i] for i, n in enumerate(geo.link_names) if n in paths}))
    cn = geo.component_names
    out["components_in_model"] = list(cn() if callable(cn) else cn)
    out["dof_names"] = list(names)
    geo.close()
    runtime.update(2)
    link_names = sorted(paths)
    runtime.timeline().play()
    runtime.update(2)
    sv = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
    roots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path), ALL_PRIMS)
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    av = sv.create_articulation_view(roots[0]) if roots else None
    if av is None:
        runtime.stop()
        out["error"] = "PhysX の Articulation を作れなかった"
        return out
    px_names = list(av.shared_metatype.dof_names)
    rb = sv.create_rigid_body_view([paths[n] for n in link_names])
    errors = []
    for full, mats in expected:
        q = np.zeros((1, len(px_names)), dtype=np.float32)
        for i, n in enumerate(px_names):
            if n in names:
                q[0, i] = full[list(names).index(n)]
        av.set_dof_positions(q, np.array([0], dtype=np.uint32))
        av.set_dof_velocities(np.zeros_like(q), np.array([0], dtype=np.uint32))
        runtime.update(1)
        t = rb.get_transforms()
        for k, n in enumerate(link_names):
            errors.append(float(np.linalg.norm(t[k, :3] - mats[n][:3, 3])))
    runtime.stop()
    out["dof_names_match"] = sorted(px_names) == sorted(names)
    out["max_link_position_error_m"] = max(errors) if errors else None
    out["links_compared"] = len(link_names)
    return out
