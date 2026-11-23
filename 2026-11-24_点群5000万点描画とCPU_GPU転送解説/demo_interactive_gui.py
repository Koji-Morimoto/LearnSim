"""Interactive GUI for UnifiedPointCloud in Isaac Sim 6.2.0.

Follows AGENTS.md mandatory SimulationApp launch rule via tools/myext/sim_app_launcher.py
while loading the full Isaac Sim experience (apps/isaacsim.exp.full.kit) so all standard
extensions + myext.viewport.axissnap + myext.coi_marker + Viewport FPS HUD are active.

Run via:
    tools\\myext\\run_sim.bat tools\\pointcloud_renderer\\demo_interactive_gui.py
"""

from __future__ import annotations

import os
import runpy
import sys
import time

os.environ["PYTHONPATH"] = os.pathsep.join(
    p for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p and "BuiltOpenUSD" not in p
)
sys.path[:] = [p for p in sys.path if "BuiltOpenUSD" not in p]
_SITECUSTOMIZE = r"C:\isaacsim-6.2.0\site\sitecustomize.py"
if os.path.exists(_SITECUSTOMIZE):
    runpy.run_path(_SITECUSTOMIZE)
sys.path[:] = [p for p in sys.path if "BuiltOpenUSD" not in p]

sys.path.insert(0, r"C:/isaacsim-6.2.0/tools/myext")
sys.path.insert(0, r"C:/isaacsim-6.2.0/tools/pointcloud_renderer")

from sim_app_launcher import create_simulation_app  # noqa: E402

simulation_app = create_simulation_app(
    {
        "headless": False,
        "width": 1280,
        "height": 720,
        "window_width": 1680,
        "window_height": 980,
        "renderer": "RayTracedLighting",
        "experience": r"C:/isaacsim-6.2.0/apps/isaacsim.exp.full.kit",
    }
)

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.viewport.utility as viewport_utils  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, Vt  # noqa: E402

from pointcloud_api import UnifiedPointCloud, create_pointcloud  # noqa: E402


def setup_studio(stage: Usd.Stage) -> None:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    s = carb.settings.get_settings()
    s.set("/rtx/rendermode", "RayTracedLighting")
    s.set("/rtx/ecoMode/enabled", False)
    s.set("/app/runLoops/main/rateLimitEnabled", False)

    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    ground.GetPointsAttr().Set(
        Vt.Vec3fArray([(-25, -25, -2.6), (25, -25, -2.6), (25, 25, -2.6), (-25, 25, -2.6)])
    )
    ground.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    ground.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    ground.GetDisplayColorPrimvar().Set(Vt.Vec3fArray([(0.16, 0.19, 0.24)]))

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(450.0)
    dome.GetColorAttr().Set(Gf.Vec3f(0.90, 0.95, 1.0))

    sun = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    sun.GetIntensityAttr().Set(2800.0)
    sun.GetAngleAttr().Set(1.2)
    UsdLux.ShadowAPI.Apply(sun.GetPrim()).CreateShadowEnableAttr(True)
    xf = UsdGeom.Xformable(sun.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddRotateXYZOp().Set(Gf.Vec3f(-45, 35, 0))


def main() -> None:
    stage = omni.usd.get_context().get_stage()
    setup_studio(stage)

    rng = np.random.default_rng(42)
    n_default = 200_000
    pos = np.empty((n_default, 3), dtype=np.float32)
    pos[:, 0] = rng.uniform(-4.5, 4.5, size=n_default)
    pos[:, 1] = rng.uniform(-4.5, 4.5, size=n_default)
    pos[:, 2] = rng.uniform(-2.0, 2.0, size=n_default)

    cols_rainbow = np.empty_like(pos)
    cols_rainbow[:, 0] = (pos[:, 0] + 4.5) / 9.0
    cols_rainbow[:, 1] = (pos[:, 1] + 4.5) / 9.0
    cols_rainbow[:, 2] = (pos[:, 2] + 2.0) / 4.0

    state: dict = {
        "method_idx": 0,
        "methods": ["octree_lod", "usd_points", "point_instancer", "hybrid_compute_rtx"],
        "radius": 0.045,
        "glossy": True,
        "cast_shadows": True,
        "color_mode": 0,
        "offset_z": 0.0,
    }

    pc: UnifiedPointCloud = create_pointcloud(
        stage,
        positions=pos,
        colors=cols_rainbow,
        method="octree_lod",
        point_radius=state["radius"],
        glossy=state["glossy"],
        cast_shadows=state["cast_shadows"],
    )

    def rebuild_method() -> None:
        nonlocal pc
        state["method_idx"] = (state["method_idx"] + 1) % len(state["methods"])
        m = state["methods"][state["method_idx"]]
        pc = create_pointcloud(
            stage,
            positions=pos,
            colors=cols_rainbow if state["color_mode"] == 0 else (0.18, 0.85, 0.55),
            method=m,
            point_radius=state["radius"],
            world_offset=(0.0, 0.0, state["offset_z"]),
            glossy=state["glossy"],
            cast_shadows=state["cast_shadows"],
        )
        btn_method.text = f"描画方式 (Method): {m}"

    def toggle_glossy() -> None:
        state["glossy"] = not state["glossy"]
        pc.set_appearance(glossy=state["glossy"])
        btn_gloss.text = f"テカリ (Glossy Specular): {'ON (ツヤあり roughness=0.08)' if state['glossy'] else 'OFF (マット roughness=0.92)'}"

    def toggle_shadows() -> None:
        state["cast_shadows"] = not state["cast_shadows"]
        pc.set_cast_shadows(state["cast_shadows"])
        btn_shadow.text = f"影の描画 (Cast Shadows): {'ON (影あり)' if state['cast_shadows'] else 'OFF (影なし・高速)'}"

    def cycle_radius() -> None:
        radii = [0.025, 0.045, 0.075, 0.11]
        cur = radii.index(state["radius"]) if state["radius"] in radii else 0
        state["radius"] = radii[(cur + 1) % len(radii)]
        pc.set_point_radius(state["radius"])
        btn_radius.text = f"点の大きさ (Radius): {state['radius']*1000:.0f} mm"

    def toggle_color() -> None:
        state["color_mode"] = 1 - state["color_mode"]
        pc.set_colors(cols_rainbow if state["color_mode"] == 0 else (0.18, 0.85, 0.55))
        btn_color.text = f"点群の色 (Color): {'3D Rainbow (点ごとRGB)' if state['color_mode'] == 0 else 'Emerald Green (単色)'}"

    def cycle_offset() -> None:
        offsets = [0.0, 0.8, -0.6]
        cur = offsets.index(state["offset_z"]) if state["offset_z"] in offsets else 0
        state["offset_z"] = offsets[(cur + 1) % len(offsets)]
        pc.set_world_offset((0.0, 0.0, state["offset_z"]))
        btn_offset.text = f"点群の位置 (World Offset Z): {state['offset_z']:+.1f} m"

    win = ui.Window("Unified Point Cloud API Controller", width=440, height=310)
    with win.frame:
        with ui.VStack(spacing=6, style={"margin": 10}):
            ui.Label("UnifiedPointCloud (create_pointcloud 統一IF)", style={"font_size": 15, "color": 0xFF66CCFF})
            lbl_info = ui.Label("Active Spheres: -- | FPS: --", style={"font_size": 14, "color": 0xFF55FF55})
            btn_method = ui.Button("描画方式 (Method): octree_lod", height=28, clicked_fn=rebuild_method)
            btn_radius = ui.Button("点の大きさ (Radius): 45 mm", height=28, clicked_fn=cycle_radius)
            btn_color = ui.Button("点群の色 (Color): 3D Rainbow (点ごとRGB)", height=28, clicked_fn=toggle_color)
            btn_offset = ui.Button("点群の位置 (World Offset Z): +0.0 m", height=28, clicked_fn=cycle_offset)
            btn_gloss = ui.Button("テカリ (Glossy Specular): ON (ツヤあり roughness=0.08)", height=28, clicked_fn=toggle_glossy)
            btn_shadow = ui.Button("影の描画 (Cast Shadows): ON (影あり)", height=28, clicked_fn=toggle_shadows)

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    vp = viewport_utils.get_active_viewport()
    last_t = time.perf_counter()
    fps_smooth = 60.0
    frame_i = 0

    while simulation_app.is_running():
        simulation_app.update()
        frame_i += 1
        now = time.perf_counter()
        dt = max(1e-5, now - last_t)
        last_t = now
        fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        cam_path = vp.camera_path if vp is not None else Sdf.Path("/OmniverseKit_Persp")
        cam_prim = stage.GetPrimAtPath(cam_path)
        if cam_prim and cam_prim.IsValid():
            xform_cache.Clear()
            wm = xform_cache.GetLocalToWorldTransform(cam_prim)
            eye = wm.ExtractTranslation()
            fwd = -Gf.Vec3d(wm[2][0], wm[2][1], wm[2][2]).GetNormalized()
            stats = pc.update_camera(
                cam_eye=(float(eye[0]), float(eye[1]), float(eye[2])),
                cam_fwd=(float(fwd[0]), float(fwd[1]), float(fwd[2])),
            )
            if frame_i % 10 == 0:
                lbl_info.text = f"Active: {stats.get('active_spheres', 0):,} pts | FPS: {fps_smooth:.1f}"

    simulation_app.close()


if __name__ == "__main__":
    main()
