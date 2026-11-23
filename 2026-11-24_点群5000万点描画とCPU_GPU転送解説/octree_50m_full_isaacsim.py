import asyncio
import builtins
import math
import time

import carb
import numpy as np
import omni.kit.app
import omni.kit.viewport.utility as viewport_utils
import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, Vt


class Octree50MController:
    def __init__(self):
        self.stage = None
        self.block_centers = []
        self.block_prims = []
        self.num_blocks = 0
        self.win = None
        self.update_sub = None
        self.xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        self.frame_idx = 0
        self.last_time = time.perf_counter()
        self.smoothed_fps = 0.0

        self.state = {
            "mode": "octree",  # "octree" or "all_50m_visible"
            "debug_color": False,
            "frustum_cull": True,
            "freeze_lod": False,
            "near_dist": 4.2,
            "mid_dist": 7.8,
            "prev_lods": [],
        }

    def enable_viewport_fps_hud(self):
        s = carb.settings.get_settings()
        s.set("/app/viewport/forceHideFps", False)
        s.set("/rtx/ecoMode/enabled", False)
        s.set("/app/runLoops/main/rateLimitEnabled", False)

        vp = viewport_utils.get_active_viewport()
        if vp is not None:
            vp_id = vp.id
            s.set(f"/persistent/app/viewport/{vp_id}/hud/visible", True)
            s.set(f"/persistent/app/viewport/{vp_id}/hud/renderFPS/visible", True)
            s.set(f"/persistent/app/viewport/{vp_id}/hud/deviceMemory/visible", True)
            s.set(f"/persistent/app/viewport/{vp_id}/hud/hostMemory/visible", True)
            s.set(f"/persistent/app/viewport/{vp_id}/hud/renderResolution/visible", True)
            try:
                from omni.kit.viewport.actions.actions import toggle_hud_visibility, toggle_viewport_visibility

                toggle_hud_visibility(vp, visible=True)
                toggle_viewport_visibility(
                    ["hud/renderFPS", "hud/deviceMemory", "hud/hostMemory", "hud/renderResolution"],
                    vp,
                    visible=True,
                )
            except Exception as e:
                print(f"[Octree 50M] Viewport actions HUD note: {e}", flush=True)

    def setup_scene(self):
        self.stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)

        s = carb.settings.get_settings()
        s.set("/rtx/rendermode", "RayTracedLighting")
        s.set("/rtx/post/tonemap/op", 4)
        s.set("/rtx/post/tonemap/filmIso", 200.0)
        s.set("/rtx/post/tonemap/whitepoint", 6500.0)
        s.set("/rtx/post/tonemap/enabled", True)
        s.set("/rtx/post/aa/op", 3)

        ground = UsdGeom.Mesh.Define(self.stage, "/World/Ground")
        ground.GetPointsAttr().Set(
            Vt.Vec3fArray([(-25, -25, -2.8), (25, -25, -2.8), (25, 25, -2.8), (-25, 25, -2.8)])
        )
        ground.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
        ground.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
        ground.GetDisplayColorPrimvar().Set(Vt.Vec3fArray([(0.16, 0.18, 0.21)]))

        dome = UsdLux.DomeLight.Define(self.stage, "/World/DomeLight")
        dome.GetIntensityAttr().Set(450.0)
        dome.GetColorAttr().Set(Gf.Vec3f(0.92, 0.96, 1.0))

        sun = UsdLux.DistantLight.Define(self.stage, "/World/Sun")
        sun.GetIntensityAttr().Set(2400.0)
        sun.GetAngleAttr().Set(1.5)
        sun_xf = UsdGeom.Xformable(sun.GetPrim())
        sun_xf.ClearXformOpOrder()
        sun_xf.AddRotateXYZOp().Set(Gf.Vec3f(-45, 35, 0))

        persp_prim = self.stage.GetPrimAtPath("/OmniverseKit_Persp")
        if persp_prim and persp_prim.IsValid():
            eye = Gf.Vec3d(6.8, -6.8, 3.0)
            target = Gf.Vec3d(0.0, 0.0, 0.0)
            up = Gf.Vec3d(0.0, 0.0, 1.0)
            fwd = (target - eye).GetNormalized()
            right = (fwd ^ up).GetNormalized()
            cam_up = (right ^ fwd).GetNormalized()
            m = Gf.Matrix4d()
            m[0] = [right[0], right[1], right[2], 0]
            m[1] = [cam_up[0], cam_up[1], cam_up[2], 0]
            m[2] = [-fwd[0], -fwd[1], -fwd[2], 0]
            m[3] = [eye[0], eye[1], eye[2], 1]
            xf = UsdGeom.Xformable(persp_prim)
            xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(m)

    def build_octree_pointcloud(self):
        t0 = time.perf_counter()
        UsdGeom.Xform.Define(self.stage, "/World/OctreePointCloud")

        nx, ny, nz = 4, 4, 2
        self.num_blocks = nx * ny * nz
        self.block_centers = []
        for ix in range(nx):
            cx = -5.0 + (ix + 0.5) * (10.0 / nx)
            for iy in range(ny):
                cy = -5.0 + (iy + 0.5) * (10.0 / ny)
                for iz in range(nz):
                    cz = -2.5 + (iz + 0.5) * (5.0 / nz)
                    self.block_centers.append((cx, cy, cz))

        # True 50,000,000 points across the 32 LOD0 blocks (1,562,500 * 32 = 50,000,000)
        lod_counts = [1_562_500, 195_312, 24_414]
        lod_tags = ["Full", "Mid", "Far"]
        rng = np.random.default_rng(42)

        lod_debug_rgb = [
            np.array([[0.15, 0.95, 0.35]], dtype=np.float32),  # LOD0: Green
            np.array([[1.00, 0.75, 0.10]], dtype=np.float32),  # LOD1: Yellow
            np.array([[0.20, 0.55, 1.00]], dtype=np.float32),  # LOD2: Blue
        ]

        templates = {}
        vt_debug_shared = {}
        vt_widths_shared = {}
        for lod_i, k_pts in enumerate(lod_counts):
            t_list = []
            for _ in range(4):
                t_list.append(rng.uniform(-1.25, 1.25, size=(k_pts, 3)).astype(np.float32))
            templates[lod_i] = t_list
            vt_debug_shared[lod_i] = Vt.Vec3fArray.FromNumpy(np.repeat(lod_debug_rgb[lod_i], k_pts, axis=0))
            r_eff = 0.045 * math.sqrt(10000.0 / float(self.num_blocks * k_pts))
            vt_widths_shared[lod_i] = Vt.FloatArray.FromNumpy(np.full(k_pts, r_eff * 2.0, dtype=np.float32))

        self.block_prims = []
        for b_idx, (bcx, bcy, bcz) in enumerate(self.block_centers):
            blk_path = f"/World/OctreePointCloud/Block_{b_idx:02d}"
            blk_xf = UsdGeom.Xform.Define(self.stage, blk_path)
            b_lods = []
            for lod_i, k_pts in enumerate(lod_counts):
                pts_path = f"{blk_path}/LOD{lod_i}_{lod_tags[lod_i]}_{k_pts}pts"
                pts_prim = UsdGeom.Points.Define(self.stage, pts_path)

                local_pos = templates[lod_i][b_idx % 4]
                pos = np.empty_like(local_pos)
                pos[:, 0] = local_pos[:, 0] + bcx
                pos[:, 1] = local_pos[:, 1] + bcy
                pos[:, 2] = local_pos[:, 2] + bcz

                cols_normal = np.empty_like(pos)
                cols_normal[:, 0] = (pos[:, 0] + 5.0) * 0.085 + 0.1
                cols_normal[:, 1] = (pos[:, 1] + 5.0) * 0.085 + 0.1
                cols_normal[:, 2] = (pos[:, 2] + 2.5) * 0.17 + 0.1
                vt_cols_normal = Vt.Vec3fArray.FromNumpy(cols_normal)

                pts_prim.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pos))
                pts_prim.GetWidthsAttr().Set(vt_widths_shared[lod_i])
                cpv = UsdGeom.PrimvarsAPI(pts_prim).CreatePrimvar(
                    "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
                )
                cpv.Set(vt_cols_normal)

                b_lods.append(
                    {
                        "vis_attr": pts_prim.GetVisibilityAttr(),
                        "color_pv": cpv,
                        "vt_normal": vt_cols_normal,
                        "vt_debug": vt_debug_shared[lod_i],
                        "k_pts": k_pts,
                    }
                )
            self.block_prims.append(
                {
                    "block_vis_attr": blk_xf.GetVisibilityAttr(),
                    "lods": b_lods,
                }
            )

        self.state["prev_lods"] = [-99] * self.num_blocks
        print(
            f"[Octree 50M] Built 50,000,000-point Octree (32 blocks x 3 LODs) in {time.perf_counter() - t0:.2f}s",
            flush=True,
        )

    def force_all_50m_visible(self):
        for b_idx in range(self.num_blocks):
            blk = self.block_prims[b_idx]
            blk["block_vis_attr"].Set(UsdGeom.Tokens.inherited)
            for lod_i, lod_entry in enumerate(blk["lods"]):
                lod_entry["vis_attr"].Set(UsdGeom.Tokens.inherited if lod_i == 0 else UsdGeom.Tokens.invisible)
            self.state["prev_lods"][b_idx] = 0

    def build_ui(self):
        def apply_color_mode(use_debug):
            self.state["debug_color"] = use_debug
            for b_idx in range(self.num_blocks):
                for lod_entry in self.block_prims[b_idx]["lods"]:
                    lod_entry["color_pv"].Set(lod_entry["vt_debug"] if use_debug else lod_entry["vt_normal"])

        def toggle_mode():
            if self.state["mode"] == "octree":
                self.state["mode"] = "all_50m_visible"
                self.force_all_50m_visible()
                self.mode_btn.text = "表示モード: 全5,000万点を全てVisible (LOD0 x 32 = 50M点)"
            else:
                self.state["mode"] = "octree"
                self.state["prev_lods"] = [-99] * self.num_blocks
                self.mode_btn.text = "表示モード: Octree LOD (距離に応じてLOD0/1/2自動切替)"

        def toggle_color_mode():
            apply_color_mode(not self.state["debug_color"])
            self.color_btn.text = (
                "Color Mode: LOD Debug (緑=LOD0 156万/黄=LOD1 19万/青=LOD2 2.4万)"
                if self.state["debug_color"]
                else "Color Mode: 3D Rainbow"
            )

        def toggle_frustum_cull():
            self.state["frustum_cull"] = not self.state["frustum_cull"]
            self.state["prev_lods"] = [-99] * self.num_blocks
            self.cull_btn.text = (
                f"Frustum Culling (視野外ブロック非表示): {'ON' if self.state['frustum_cull'] else 'OFF (全32ブロック常時Visible)'}"
            )

        def toggle_freeze_lod():
            self.state["freeze_lod"] = not self.state["freeze_lod"]
            self.freeze_btn.text = (
                f"Freeze Current LOD: {'ON (カメラ移動で裏側確認可)' if self.state['freeze_lod'] else 'OFF (リアルタイム追従)'}"
            )

        self.win = ui.Window("50M Point Cloud - Octree LOD Controller", width=440, height=295)
        with self.win.frame:
            with ui.VStack(spacing=6, style={"margin": 10}):
                ui.Label(
                    "50,000,000 Points Octree LOD (1,562,500 pts x 32 Blocks)",
                    style={"font_size": 15, "color": 0xFF66CCFF},
                )
                self.fps_label = ui.Label("Live FPS: --", style={"font_size": 15, "color": 0xFF55FF55})
                self.stats_label = ui.Label("Active Spheres: --", style={"font_size": 13})
                self.blocks_label = ui.Label(
                    "Blocks: LOD0(Full)=-- | LOD1(Mid)=-- | LOD2(Far)=-- | Culled=--", style={"font_size": 12}
                )
                self.cam_label = ui.Label("Camera Pos: --", style={"font_size": 12, "color": 0xFFAAAAAA})
                ui.Spacer(height=4)
                self.mode_btn = ui.Button(
                    "表示モード: Octree LOD (距離に応じてLOD0/1/2自動切替)", height=30, clicked_fn=toggle_mode
                )
                self.cull_btn = ui.Button(
                    "Frustum Culling (視野外ブロック非表示): ON", height=28, clicked_fn=toggle_frustum_cull
                )
                self.color_btn = ui.Button("Color Mode: 3D Rainbow", height=28, clicked_fn=toggle_color_mode)
                self.freeze_btn = ui.Button(
                    "Freeze Current LOD: OFF (リアルタイム追従)", height=28, clicked_fn=toggle_freeze_lod
                )

    def on_update(self, _e):
        now = time.perf_counter()
        dt = max(1e-5, now - self.last_time)
        self.last_time = now
        inst_fps = 1.0 / dt
        self.smoothed_fps = inst_fps if self.smoothed_fps == 0.0 else (0.9 * self.smoothed_fps + 0.1 * inst_fps)

        self.frame_idx += 1
        if not self.stage:
            return

        vp = viewport_utils.get_active_viewport()
        cam_path = vp.camera_path if vp is not None else Sdf.Path("/OmniverseKit_Persp")
        cam_prim = self.stage.GetPrimAtPath(cam_path)
        if not cam_prim or not cam_prim.IsValid():
            return

        self.xform_cache.Clear()
        world_mat = self.xform_cache.GetLocalToWorldTransform(cam_prim)
        eye = world_mat.ExtractTranslation()
        fwd = -Gf.Vec3d(world_mat[2][0], world_mat[2][1], world_mat[2][2]).GetNormalized()
        ex, ey, ez = float(eye[0]), float(eye[1]), float(eye[2])
        fx, fy, fz = float(fwd[0]), float(fwd[1]), float(fwd[2])

        if self.state["mode"] == "all_50m_visible":
            if self.frame_idx % 10 == 0:
                self.fps_label.text = f"Live FPS: {self.smoothed_fps:.1f} FPS ({1000.0 / max(0.1, self.smoothed_fps):.1f} ms)"
                self.stats_label.text = "Active Spheres: 50,000,000 / 50,000,000 (100.00% 全点Visible)"
                self.blocks_label.text = "LOD0(156万点): 32 blocks | LOD1: 0 | LOD2: 0 | 視野外カリング: 0"
                self.cam_label.text = f"Camera: ({ex:.1f}, {ey:.1f}, {ez:.1f}) Dir: ({fx:.2f}, {fy:.2f}, {fz:.2f})"
            return

        if not self.state["freeze_lod"]:
            active_total = 0
            counts = {0: 0, 1: 0, 2: 0, -1: 0}
            lod_changed = False
            for b_idx, (bcx, bcy, bcz) in enumerate(self.block_centers):
                dx, dy, dz = bcx - ex, bcy - ey, bcz - ez
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                fwd_dist = dx * fx + dy * fy + dz * fz

                if self.state["frustum_cull"] and (
                    fwd_dist < -2.2 or (dist > 3.2 and fwd_dist / max(0.001, dist) < 0.30)
                ):
                    sel = -1
                elif dist < self.state["near_dist"]:
                    sel = 0
                elif dist < self.state["mid_dist"]:
                    sel = 1
                else:
                    sel = 2

                counts[sel] += 1
                blk = self.block_prims[b_idx]
                if sel >= 0:
                    active_total += blk["lods"][sel]["k_pts"]

                if sel != self.state["prev_lods"][b_idx]:
                    self.state["prev_lods"][b_idx] = sel
                    lod_changed = True
                    if sel == -1:
                        blk["block_vis_attr"].Set(UsdGeom.Tokens.invisible)
                    else:
                        blk["block_vis_attr"].Set(UsdGeom.Tokens.inherited)
                        for lod_i, lod_entry in enumerate(blk["lods"]):
                            lod_entry["vis_attr"].Set(
                                UsdGeom.Tokens.inherited if lod_i == sel else UsdGeom.Tokens.invisible
                            )

            if lod_changed or self.frame_idx % 10 == 0:
                self.stats_label.text = (
                    f"Active Spheres: {active_total:,} / 50,000,000 ({active_total / 500000.0:.2f}%)"
                )
                self.blocks_label.text = (
                    f"LOD0(156万): {counts[0]} | LOD1(19万): {counts[1]} | LOD2(2.4万): {counts[2]} | 視野外非表示: {counts[-1]}"
                )

        if self.frame_idx % 10 == 0:
            self.fps_label.text = f"Live FPS: {self.smoothed_fps:.1f} FPS ({1000.0 / max(0.1, self.smoothed_fps):.1f} ms)"
            self.cam_label.text = f"Camera: ({ex:.1f}, {ey:.1f}, {ez:.1f}) Dir: ({fx:.2f}, {fy:.2f}, {fz:.2f})"


async def _async_startup():
    app = omni.kit.app.get_app()
    for _ in range(15):
        await app.next_update_async()

    ctrl = Octree50MController()
    builtins._octree_50m_controller = ctrl
    ctrl.setup_scene()
    ctrl.build_octree_pointcloud()
    ctrl.build_ui()

    for _ in range(5):
        await app.next_update_async()

    ctrl.enable_viewport_fps_hud()
    ctrl.update_sub = app.get_update_event_stream().create_subscription_to_pop(
        ctrl.on_update, name="octree_50m_controller_update"
    )
    print("[Octree 50M] Full Isaac Sim setup complete! HUD FPS enabled.", flush=True)


asyncio.ensure_future(_async_startup())
