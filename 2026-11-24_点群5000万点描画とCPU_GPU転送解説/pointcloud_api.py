"""Unified Point Cloud Renderer API for Isaac Sim 6.2.0.

Provides a single unified function/class interface (`create_pointcloud` / `UnifiedPointCloud`)
across all 6 point-cloud rendering architectures:
  1. "naive_spheres"         : Individual UsdGeom.Sphere prim per point (1 Prim = 1 DrawCall/Node)
  2. "usd_points"            : Chunked UsdGeom.Points (RTX analytic raytraced sphere primitives)
  3. "point_instancer"       : UsdGeom.PointInstancer (GPU RT-Core hardware mesh instancing)
  4. "octree_lod"            : 32-block 3-tier USD Octree LOD + Frustum Culling
  5. "hybrid_compute_rtx"    : Warp CUDA Compute Shader Frustum/LOD culling -> RTX UsdGeom.Points
  6. "warp_compute_atomic64" : Warp CUDA 64-bit atomicMin((depth<<32)|color) software sphere rasterizer

All methods share identical parameters and runtime setters for:
  - positions         : (N, 3) float32 xyz coordinates (or world_offset translation)
  - colors            : (N, 3) float32 RGB in [0, 1] or single (R, G, B) tuple
  - point_radius      : Sphere radius in meters
  - unlit             : bool (Default True [F2]: zero light reflection, emissive flat RGB)
  - outline_thickness : float (Default 0.08 [F2]: inverted-hull black edge ratio, 0.0 = off)
  - outline_color     : (R, G, B) tuple for edge outline (Default black (0, 0, 0))
  - anti_aliasing     : "DLAA" (4, Default [F2]) or "DLSS" (3)
  - glossy            : bool (when unlit=False: True=shiny roughness 0.08, False=matte 1.0)
  - roughness         : float in [0.02, 1.0] (1.0 = pure matte, 0.08 = glossy)
  - metallic          : float in [0.0, 1.0]
  - specular_level    : float in [0.0, 1.0] (0.0 = zero specular highlight)
  - cast_shadows      : bool (Default False [F2]: True = raytraced shadows, False = no shadows)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal

import carb.settings
import numpy as np
import warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

MethodType = Literal[
    "naive_spheres",
    "usd_points",
    "point_instancer",
    "octree_lod",
    "hybrid_compute_rtx",
    "warp_compute_atomic64",
]
AntiAliasingType = Literal["DLAA", "DLSS"]


@dataclass
class PointCloudConfig:
    """Unified configuration for all point cloud rendering methods.

    Default appearance preset is F2 (Zero-Reflection Unlit + 8% Thin Black Outline + DLAA + No Shadows).
    """

    method: MethodType = "octree_lod"
    root_path: str = "/World/PointCloud"
    point_radius: float = 0.035
    world_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Appearance / Material settings (Default: Preset F2)
    unlit: bool = True
    outline_thickness: float = 0.08
    outline_color: tuple[float, float, float] = (0.0, 0.0, 0.0)
    anti_aliasing: AntiAliasingType | int = "DLAA"
    glossy: bool = False
    roughness: float | None = None  # If None, derived from glossy (0.08 if glossy else 1.0)
    metallic: float = 0.0
    specular_level: float = 0.0
    cast_shadows: bool = False
    max_outline_points: int = 1_500_000
    # Method-specific tuning
    chunk_size: int = 1_000_000  # Max points per UsdGeom.Points prim (< 3.93M RTX 45MiB limit)
    octree_grid: tuple[int, int, int] = (4, 4, 2)
    octree_bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] = (
        (-5.0, 5.0),
        (-5.0, 5.0),
        (-2.5, 2.5),
    )
    octree_near_dist: float = 4.2
    octree_mid_dist: float = 7.8
    frustum_cull: bool = True
    octree_use_instancer: bool = False
    hybrid_max_budget: int = 500_000
    compute_resolution: tuple[int, int] = (1280, 720)
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def effective_roughness(self) -> float:
        if self.roughness is not None:
            return float(np.clip(self.roughness, 0.02, 1.0))
        return 0.08 if self.glossy else 1.0

    @property
    def effective_specular(self) -> float:
        if self.glossy:
            return float(self.specular_level) if self.specular_level > 0.0 else 0.85
        return float(np.clip(self.specular_level, 0.0, 1.0))

    @property
    def effective_aa_op(self) -> int:
        if isinstance(self.anti_aliasing, int):
            return int(self.anti_aliasing)
        return 4 if str(self.anti_aliasing).upper() == "DLAA" else 3


# =============================================================================
# Warp CUDA Kernels for Method 5 (Hybrid Compute+RTX) & Method 6 (64-bit Atomic)
# =============================================================================
@wp.kernel
def _kernel_cull_and_lod(
    pos: wp.array(dtype=wp.vec3),
    col: wp.array(dtype=wp.vec3),
    n_total: int,
    cam_eye: wp.vec3,
    cam_fwd: wp.vec3,
    world_offset: wp.vec3,
    base_diameter: float,
    near_dist: float,
    mid_dist: float,
    frustum_cull: int,
    max_out: int,
    out_count: wp.array(dtype=int),
    out_pos: wp.array(dtype=wp.vec3),
    out_col: wp.array(dtype=wp.vec3),
    out_widths: wp.array(dtype=float),
):
    tid = wp.tid()
    if tid >= n_total:
        return

    p = pos[tid] + world_offset
    d = p - cam_eye
    dist = wp.length(d)
    fwd_z = wp.dot(d, cam_fwd)

    if frustum_cull != 0:
        if fwd_z < 0.15:
            return
        if dist > 2.5 and (fwd_z / wp.max(0.001, dist)) < 0.42:
            return

    # Deterministic spatial stride decimation + area-preserving diameter
    stride = int(1)
    scale = float(1.0)
    if dist >= mid_dist:
        stride = 32
        scale = 5.65
    elif dist >= near_dist:
        stride = 6
        scale = 2.45

    if (tid % stride) != 0:
        return

    slot = wp.atomic_add(out_count, 0, 1)
    if slot < max_out:
        out_pos[slot] = p
        out_col[slot] = col[tid]
        out_widths[slot] = base_diameter * scale


@wp.kernel
def _kernel_clear_fb64(
    fb64: wp.array(dtype=wp.uint64),
    n_pixels: int,
    bg_pack: wp.uint64,
):
    tid = wp.tid()
    if tid < n_pixels:
        fb64[tid] = bg_pack


@wp.kernel
def _kernel_rasterize_spheres_atomic64(
    pos: wp.array(dtype=wp.vec3),
    col: wp.array(dtype=wp.vec3),
    n_total: int,
    world_offset: wp.vec3,
    cam_eye: wp.vec3,
    cam_right: wp.vec3,
    cam_up: wp.vec3,
    cam_fwd: wp.vec3,
    focal_px: float,
    width: int,
    height: int,
    sphere_radius: float,
    fb64: wp.array(dtype=wp.uint64),
):
    tid = wp.tid()
    if tid >= n_total:
        return

    p = pos[tid] + world_offset
    rel = p - cam_eye
    cz = wp.dot(rel, cam_fwd)
    if cz <= 0.15 or cz > 120.0:
        return

    cx = wp.dot(rel, cam_right)
    cy = wp.dot(rel, cam_up)

    inv_z = 1.0 / cz
    u_f = float(width) * 0.5 + cx * focal_px * inv_z
    v_f = float(height) * 0.5 - cy * focal_px * inv_z

    rad_px = wp.clamp(int(sphere_radius * focal_px * inv_z + 0.5), 1, 10)
    u0 = int(u_f)
    v0 = int(v_f)
    if u0 + rad_px < 0 or u0 - rad_px >= width or v0 + rad_px < 0 or v0 - rad_px >= height:
        return

    c = col[tid]
    # Pack base RGB + encoded local point ID bits for post-resolve analytical normal & shading
    r_base = wp.clamp(int(c[0] * 255.0), 0, 255)
    g_base = wp.clamp(int(c[1] * 255.0), 0, 255)
    b_base = wp.clamp(int(c[2] * 255.0), 0, 255)

    for dy in range(-rad_px, rad_px + 1):
        py = v0 + dy
        if py < 0 or py >= height:
            continue
        for dx in range(-rad_px, rad_px + 1):
            px = u0 + dx
            if px < 0 or px >= width:
                continue
            r2 = float(dx * dx + dy * dy)
            max_r2 = float(rad_px * rad_px)
            if r2 > max_r2:
                continue

            # True 3D sphere surface normal & depth offset
            nx = float(dx) / wp.max(1.0, float(rad_px))
            ny = float(-dy) / wp.max(1.0, float(rad_px))
            nz = wp.sqrt(wp.max(0.0, 1.0 - nx * nx - ny * ny))
            z_surf = cz - nz * sphere_radius

            depth_u32 = wp.uint64(wp.clamp(int(z_surf * 65535.0), 1, 0x7FFFFFFF))
            # Pack normal (nx, ny) into high bits of color payload or compute shading in resolve
            # Here we pack (r, g, b, normal_code) into lower 32 bits
            nx_u4 = wp.clamp(int((nx * 0.5 + 0.5) * 15.0), 0, 15)
            ny_u4 = wp.clamp(int((ny * 0.5 + 0.5) * 15.0), 0, 15)
            n_byte = (nx_u4 << 4) | ny_u4
            rgba_u32 = wp.uint64((n_byte << 24) | (r_base << 16) | (g_base << 8) | b_base)
            packed = (depth_u32 << wp.uint64(32)) | rgba_u32
            wp.atomic_min(fb64, py * width + px, packed)


@wp.kernel
def _kernel_resolve_fb64_shaded(
    fb64: wp.array(dtype=wp.uint64),
    out_rgba: wp.array(dtype=wp.uint8, ndim=3),
    width: int,
    height: int,
    roughness: float,
    specular_strength: float,
    cast_shadows: int,
):
    tid = wp.tid()
    n_pixels = width * height
    if tid >= n_pixels:
        return

    py = tid // width
    px = tid % width
    val = fb64[tid]
    depth_u32 = int(val >> wp.uint64(32))
    low = int(val & wp.uint64(0xFFFFFFFF))

    r_f = float((low >> 16) & 255) / 255.0
    g_f = float((low >> 8) & 255) / 255.0
    b_f = float(low & 255) / 255.0

    # Background pixel
    if depth_u32 >= 0x7F000000:
        out_rgba[py, px, 0] = wp.uint8(int(r_f * 255.0))
        out_rgba[py, px, 1] = wp.uint8(int(g_f * 255.0))
        out_rgba[py, px, 2] = wp.uint8(int(b_f * 255.0))
        out_rgba[py, px, 3] = wp.uint8(255)
        return

    n_byte = (low >> 24) & 255
    nx = (float((n_byte >> 4) & 15) / 15.0) * 2.0 - 1.0
    ny = (float(n_byte & 15) / 15.0) * 2.0 - 1.0
    nz = wp.sqrt(wp.max(0.05, 1.0 - nx * nx - ny * ny))

    # Key light direction in view space: top-right-front (0.45, 0.55, 0.70)
    lx = 0.45
    ly = 0.55
    lz = 0.703
    ndotl = wp.max(0.0, nx * lx + ny * ly + nz * lz)

    # Screen-space shadow ray check along light direction when cast_shadows == 1
    shadow_mult = float(1.0)
    if cast_shadows != 0:
        sx = wp.clamp(px + 4, 0, width - 1)
        sy = wp.clamp(py - 5, 0, height - 1)
        s_depth = int(fb64[sy * width + sx] >> wp.uint64(32))
        if s_depth < depth_u32 - 6500:
            shadow_mult = 0.42

    diffuse = 0.26 + 0.74 * ndotl * shadow_mult

    # Blinn-Phong specular highlight controlled by roughness & specular_strength
    # Halfway vector between view (0,0,1) and light (lx, ly, lz)
    hx = 0.258
    hy = 0.315
    hz = 0.913
    ndoth = wp.max(0.0, nx * hx + ny * hy + nz * hz)
    shininess = wp.clamp(2.0 / wp.max(0.02, roughness * roughness) - 1.5, 2.0, 256.0)
    spec = wp.pow(ndoth, shininess) * specular_strength * shadow_mult

    r_out = wp.clamp(int((r_f * diffuse + spec) * 255.0), 0, 255)
    g_out = wp.clamp(int((g_f * diffuse + spec) * 255.0), 0, 255)
    b_out = wp.clamp(int((b_f * diffuse + spec) * 255.0), 0, 255)

    out_rgba[py, px, 0] = wp.uint8(r_out)
    out_rgba[py, px, 1] = wp.uint8(g_out)
    out_rgba[py, px, 2] = wp.uint8(b_out)
    out_rgba[py, px, 3] = wp.uint8(255)


# =============================================================================
# Unified Point Cloud Class
# =============================================================================
class UnifiedPointCloud:
    """Unified controller for all 6 point cloud rendering methods in Isaac Sim 6.2.0."""

    def __init__(
        self,
        stage: Usd.Stage,
        positions: np.ndarray,
        colors: np.ndarray | tuple[float, float, float] = (0.2, 0.75, 1.0),
        config: PointCloudConfig | None = None,
    ) -> None:
        self.stage = stage
        self.cfg = config or PointCloudConfig()
        self.positions = np.ascontiguousarray(positions, dtype=np.float32)
        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError(f"positions must have shape (N, 3), got {self.positions.shape}")
        self.num_points = int(self.positions.shape[0])
        self.colors = self._normalize_colors(colors, self.num_points)

        self.root_prim: Usd.Prim | None = None
        self.root_xform: UsdGeom.Xform | None = None
        self.material: UsdShade.Material | None = None
        self.shader: UsdShade.Shader | None = None
        self.color_reader: UsdShade.Shader | None = None
        self.geometry_prims: list[Usd.Prim] = []
        self.width_attrs: list[tuple[Usd.Attribute, int, float]] = []
        self.proto_scale_ops: list[tuple[UsdGeom.XformOp, float]] = []
        self.color_primvars: list[UsdGeom.Primvar] = []
        self._outline_scale_op: UsdGeom.XformOp | None = None
        self._outline_shader: UsdShade.Shader | None = None

        # Method-specific runtime state
        self._octree_blocks: list[dict[str, Any]] = []
        self._octree_centers: list[tuple[float, float, float]] = []
        self._octree_prev_lods: list[int] = []
        self._octree_active_spheres: int = self.num_points

        self._wp_pos: wp.array | None = None
        self._wp_col: wp.array | None = None
        self._wp_out_count: wp.array | None = None
        self._wp_out_pos: wp.array | None = None
        self._wp_out_col: wp.array | None = None
        self._wp_out_widths: wp.array | None = None
        self._hybrid_points_prim: UsdGeom.Points | None = None
        self._hybrid_last_cam: tuple[float, float, float, float, float, float] | None = None
        self._hybrid_last_active: int = 0
        self._hybrid_dirty: bool = True

        self._wp_fb64: wp.array | None = None
        self._wp_rgba: wp.array | None = None
        self.last_compute_image: np.ndarray | None = None

        self._build()

    @staticmethod
    def _normalize_colors(
        colors: np.ndarray | tuple[float, float, float] | list[float],
        n_pts: int,
    ) -> np.ndarray:
        arr = np.asarray(colors, dtype=np.float32)
        if arr.ndim == 1 and arr.size == 3:
            out = np.empty((n_pts, 3), dtype=np.float32)
            out[:, 0] = arr[0]
            out[:, 1] = arr[1]
            out[:, 2] = arr[2]
            return out
        if arr.ndim == 2 and arr.shape == (n_pts, 3):
            return np.ascontiguousarray(arr, dtype=np.float32)
        raise ValueError(f"colors must be an RGB 3-tuple or shape ({n_pts}, 3), got {arr.shape}")

    def _create_primvar_pbr_material(self) -> UsdShade.Material:
        """Create a UsdPreviewSurface material that reads per-point displayColor via UsdPrimvarReader_float3
        and supports both zero-reflection Unlit (emissiveColor) and PBR Matte/Glossy modes."""
        mat_path = f"{self.cfg.root_path}/PointCloudMaterial"
        mat = UsdShade.Material.Define(self.stage, mat_path)

        pbr = UsdShade.Shader.Define(self.stage, f"{mat_path}/PBRShader")
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)

        reader = UsdShade.Shader.Define(self.stage, f"{mat_path}/DisplayColorReader")
        reader.CreateIdAttr("UsdPrimvarReader_float3")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
        reader.CreateInput("fallback", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.25, 0.75, 1.0))
        reader.CreateOutput("result", Sdf.ValueTypeNames.Color3f)

        mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")

        self.material = mat
        self.shader = pbr
        self.color_reader = reader
        self._apply_material_inputs()
        return mat

    def _apply_material_inputs(self) -> None:
        if self.shader is None or self.color_reader is None:
            return
        pbr = self.shader
        diff_in = pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        emis_in = pbr.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
        diff_in.DisconnectSource()
        emis_in.DisconnectSource()

        if self.cfg.unlit:
            # Pure zero-reflection flat color: diffuse=0, specular=0, emissive=displayColor
            diff_in.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            emis_in.ConnectToSource(self.color_reader.ConnectableAPI(), "result")
            pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
            pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            pbr.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        else:
            diff_in.ConnectToSource(self.color_reader.ConnectableAPI(), "result")
            emis_in.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(self.cfg.effective_roughness)
            pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(self.cfg.metallic))
            sv = self.cfg.effective_specular
            pbr.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(sv, sv, sv))

    @staticmethod
    def _define_inward_unit_sphere(stage: Usd.Stage, path: str, n_lon: int = 24, n_lat: int = 16) -> UsdGeom.Mesh:
        """Create a unit-radius UV sphere with inward-facing winding and normals for RTX inverted-hull outline."""
        lat = np.linspace(0, np.pi, n_lat + 1)
        lon = np.linspace(0, 2 * np.pi, n_lon, endpoint=False)
        v = [(0.0, 0.0, 1.0)]
        for th in lat[1:-1]:
            for ph in lon:
                v.append((math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)))
        v.append((0.0, 0.0, -1.0))
        idx, counts = [], []

        def ring(i: int) -> int:
            return 1 + i * n_lon

        for j in range(n_lon):
            idx += [0, ring(0) + (j + 1) % n_lon, ring(0) + j]
            counts.append(3)
        for i in range(n_lat - 2):
            for j in range(n_lon):
                a, b = ring(i) + j, ring(i) + (j + 1) % n_lon
                c, d = ring(i + 1) + (j + 1) % n_lon, ring(i + 1) + j
                idx += [a, b, c, d]
                counts.append(4)
        south = len(v) - 1
        for j in range(n_lon):
            idx += [south, ring(n_lat - 2) + j, ring(n_lat - 2) + (j + 1) % n_lon]
            counts.append(3)

        mesh = UsdGeom.Mesh.Define(stage, path)
        pts = np.asarray(v, np.float32)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(-pts))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(False)
        return mesh

    def _apply_shadow_flag_to_prim(self, prim: Usd.Prim, cast_shadows: bool) -> None:
        pv_api = UsdGeom.PrimvarsAPI(prim)
        pv = pv_api.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool, UsdGeom.Tokens.constant)
        pv.Set(bool(not cast_shadows))

    def _build(self) -> None:
        if self.stage.GetPrimAtPath(self.cfg.root_path).IsValid():
            self.stage.RemovePrim(self.cfg.root_path)

        self.root_xform = UsdGeom.Xform.Define(self.stage, self.cfg.root_path)
        self.root_prim = self.root_xform.GetPrim()
        self.root_xform.ClearXformOpOrder()
        self._translate_op = self.root_xform.AddTranslateOp()
        self._translate_op.Set(Gf.Vec3d(*self.cfg.world_offset))

        self._create_primvar_pbr_material()
        UsdShade.MaterialBindingAPI.Apply(self.root_prim).Bind(self.material)

        method = self.cfg.method
        if method == "naive_spheres":
            self._build_naive_spheres()
        elif method == "usd_points":
            self._build_usd_points()
        elif method == "point_instancer":
            self._build_point_instancer()
        elif method == "octree_lod":
            self._build_octree_lod()
        elif method == "hybrid_compute_rtx":
            self._build_hybrid_compute_rtx()
        elif method == "warp_compute_atomic64":
            self._build_warp_compute_atomic64()
        else:
            raise ValueError(f"Unknown pointcloud method: {method}")

        self.set_anti_aliasing(self.cfg.anti_aliasing)
        self.set_cast_shadows(self.cfg.cast_shadows)
        self.set_outline(self.cfg.outline_thickness, self.cfg.outline_color)

    def _build_naive_spheres(self) -> None:
        r = float(self.cfg.point_radius)
        for i in range(self.num_points):
            sp_path = f"{self.cfg.root_path}/Sphere_{i:06d}"
            sp = UsdGeom.Sphere.Define(self.stage, sp_path)
            sp.GetRadiusAttr().Set(r)
            xf = UsdGeom.Xformable(sp.GetPrim())
            xf.AddTranslateOp().Set(
                Gf.Vec3d(float(self.positions[i, 0]), float(self.positions[i, 1]), float(self.positions[i, 2]))
            )
            cpv = sp.GetDisplayColorPrimvar()
            c = self.colors[i]
            cpv.Set(Vt.Vec3fArray([Gf.Vec3f(float(c[0]), float(c[1]), float(c[2]))]))
            self.geometry_prims.append(sp.GetPrim())

    def _build_usd_points(self) -> None:
        chunk_sz = max(1, int(self.cfg.chunk_size))
        n_chunks = (self.num_points + chunk_sz - 1) // chunk_sz
        diam = float(self.cfg.point_radius * 2.0)

        for ci in range(n_chunks):
            s = ci * chunk_sz
            e = min(self.num_points, s + chunk_sz)
            cnt = e - s
            pts_prim = UsdGeom.Points.Define(self.stage, f"{self.cfg.root_path}/PointsChunk_{ci:03d}")
            pts_prim.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(self.positions[s:e]))
            w_attr = pts_prim.GetWidthsAttr()
            w_attr.Set(Vt.FloatArray.FromNumpy(np.full(cnt, diam, dtype=np.float32)))
            cpv = UsdGeom.PrimvarsAPI(pts_prim).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
            )
            cpv.Set(Vt.Vec3fArray.FromNumpy(self.colors[s:e]))
            self.geometry_prims.append(pts_prim.GetPrim())
            self.width_attrs.append((w_attr, cnt, 1.0))
            self.color_primvars.append(cpv)

    def _build_point_instancer(self) -> None:
        inst_path = f"{self.cfg.root_path}/Instancer"
        inst = UsdGeom.PointInstancer.Define(self.stage, inst_path)
        proto_path = f"{inst_path}/SphereProto"
        sphere = UsdGeom.Sphere.Define(self.stage, proto_path)
        sphere.GetRadiusAttr().Set(1.0)
        sphere.GetPrim().CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool).Set(True)
        sphere.GetPrim().CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(0)
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(self.material)

        r = float(self.cfg.point_radius)
        scale_op = UsdGeom.Xformable(sphere.GetPrim()).AddScaleOp()
        scale_op.Set(Gf.Vec3d(r, r, r))
        self.proto_scale_ops.append((scale_op, 1.0))

        inst.CreatePrototypesRel().SetTargets([sphere.GetPath()])
        inst.GetProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(np.zeros(self.num_points, dtype=np.int32)))
        inst.GetPositionsAttr().Set(Vt.Vec3fArray.FromNumpy(self.positions))

        cpv = UsdGeom.PrimvarsAPI(inst).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
        )
        cpv.Set(Vt.Vec3fArray.FromNumpy(self.colors))
        cpv.SetIndices(Vt.IntArray.FromNumpy(np.arange(self.num_points, dtype=np.int32)))

        self.geometry_prims.append(inst.GetPrim())
        self.geometry_prims.append(sphere.GetPrim())
        self.color_primvars.append(cpv)

    def _build_octree_lod(self) -> None:
        nx, ny, nz = self.cfg.octree_grid
        num_blocks = nx * ny * nz
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = self.cfg.octree_bounds
        dx = (xmax - xmin) / nx
        dy = (ymax - ymin) / ny
        dz = (zmax - zmin) / nz

        self._octree_centers = []
        for ix in range(nx):
            cx = xmin + (ix + 0.5) * dx
            for iy in range(ny):
                cy = ymin + (iy + 0.5) * dy
                for iz in range(nz):
                    cz = zmin + (iz + 0.5) * dz
                    self._octree_centers.append((cx, cy, cz))

        pts_per_block_l0 = max(16, self.num_points // num_blocks)
        pts_per_block_l1 = max(8, pts_per_block_l0 // 8)
        pts_per_block_l2 = max(4, pts_per_block_l0 // 64)
        lod_counts = [pts_per_block_l0, pts_per_block_l1, pts_per_block_l2]
        lod_tags = ["Full", "Mid", "Far"]

        # Partition or slice points across the 32 spatial blocks
        self._octree_blocks = []
        for b_idx, (bcx, bcy, bcz) in enumerate(self._octree_centers):
            blk_path = f"{self.cfg.root_path}/Block_{b_idx:02d}"
            blk_xf = UsdGeom.Xform.Define(self.stage, blk_path)
            s0 = (b_idx * pts_per_block_l0) % max(1, self.num_points)
            e0 = min(self.num_points, s0 + pts_per_block_l0)
            blk_pos_full = self.positions[s0:e0]
            blk_col_full = self.colors[s0:e0]
            if len(blk_pos_full) == 0:
                blk_pos_full = self.positions[:pts_per_block_l0]
                blk_col_full = self.colors[:pts_per_block_l0]

            b_lods = []
            for lod_i, k_req in enumerate(lod_counts):
                stride = max(1, len(blk_pos_full) // max(1, k_req))
                sub_pos = np.ascontiguousarray(blk_pos_full[::stride][:k_req])
                sub_col = np.ascontiguousarray(blk_col_full[::stride][:k_req])
                k_pts = len(sub_pos)
                # Area-preserving radius scale factor
                scale_factor = math.sqrt(float(len(blk_pos_full)) / float(max(1, k_pts)))
                r_eff = float(self.cfg.point_radius) * scale_factor

                prim_path = f"{blk_path}/LOD{lod_i}_{lod_tags[lod_i]}_{k_pts}pts"
                if self.cfg.octree_use_instancer:
                    inst = UsdGeom.PointInstancer.Define(self.stage, prim_path)
                    sphere = UsdGeom.Sphere.Define(self.stage, f"{prim_path}/SphereProto")
                    sphere.GetRadiusAttr().Set(1.0)
                    sphere.GetPrim().CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool).Set(True)
                    sphere.GetPrim().CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(0)
                    UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(self.material)
                    sc_op = UsdGeom.Xformable(sphere.GetPrim()).AddScaleOp()
                    sc_op.Set(Gf.Vec3d(r_eff, r_eff, r_eff))
                    self.proto_scale_ops.append((sc_op, scale_factor))

                    inst.CreatePrototypesRel().SetTargets([sphere.GetPath()])
                    inst.GetProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(np.zeros(k_pts, dtype=np.int32)))
                    inst.GetPositionsAttr().Set(Vt.Vec3fArray.FromNumpy(sub_pos))
                    cpv = UsdGeom.PrimvarsAPI(inst).CreatePrimvar(
                        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
                    )
                    cpv.Set(Vt.Vec3fArray.FromNumpy(sub_col))
                    cpv.SetIndices(Vt.IntArray.FromNumpy(np.arange(k_pts, dtype=np.int32)))
                    g_prim = inst.GetPrim()
                    self.geometry_prims.append(sphere.GetPrim())
                else:
                    pts_prim = UsdGeom.Points.Define(self.stage, prim_path)
                    pts_prim.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(sub_pos))
                    w_attr = pts_prim.GetWidthsAttr()
                    w_attr.Set(Vt.FloatArray.FromNumpy(np.full(k_pts, r_eff * 2.0, dtype=np.float32)))
                    cpv = UsdGeom.PrimvarsAPI(pts_prim).CreatePrimvar(
                        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
                    )
                    cpv.Set(Vt.Vec3fArray.FromNumpy(sub_col))
                    g_prim = pts_prim.GetPrim()
                    self.width_attrs.append((w_attr, k_pts, scale_factor))

                if lod_i > 0:
                    UsdGeom.Imageable(g_prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

                self.geometry_prims.append(g_prim)
                self.color_primvars.append(cpv)
                b_lods.append(
                    {
                        "vis_attr": UsdGeom.Imageable(g_prim).GetVisibilityAttr(),
                        "k_pts": k_pts,
                    }
                )

            self._octree_blocks.append(
                {
                    "block_vis_attr": blk_xf.GetVisibilityAttr(),
                    "lods": b_lods,
                }
            )
        self._octree_prev_lods = [0] * num_blocks

    def _build_hybrid_compute_rtx(self) -> None:
        self._wp_pos = wp.array(self.positions, dtype=wp.vec3, device="cuda:0")
        self._wp_col = wp.array(self.colors, dtype=wp.vec3, device="cuda:0")
        budget = min(self.num_points, int(self.cfg.hybrid_max_budget))
        self._wp_out_count = wp.zeros(1, dtype=int, device="cuda:0")
        self._wp_out_pos = wp.zeros(budget, dtype=wp.vec3, device="cuda:0")
        self._wp_out_col = wp.zeros(budget, dtype=wp.vec3, device="cuda:0")
        self._wp_out_widths = wp.zeros(budget, dtype=float, device="cuda:0")

        pts_prim = UsdGeom.Points.Define(self.stage, f"{self.cfg.root_path}/HybridActivePoints")
        self._hybrid_points_prim = pts_prim
        self.geometry_prims.append(pts_prim.GetPrim())
        # Initialize with initial camera view
        self.update_camera(cam_eye=(6.5, -6.5, 3.0), cam_target=(0.0, 0.0, 0.0))

    def _build_warp_compute_atomic64(self) -> None:
        w, h = self.cfg.compute_resolution
        self._wp_pos = wp.array(self.positions, dtype=wp.vec3, device="cuda:0")
        self._wp_col = wp.array(self.colors, dtype=wp.vec3, device="cuda:0")
        self._wp_fb64 = wp.zeros(w * h, dtype=wp.uint64, device="cuda:0")
        self._wp_rgba = wp.zeros((h, w, 4), dtype=wp.uint8, device="cuda:0")
        self.update_camera(cam_eye=(6.5, -6.5, 3.0), cam_target=(0.0, 0.0, 0.0))

    # =========================================================================
    # Runtime Appearance & Transform Setters (Unified across all methods!)
    # =========================================================================
    def set_world_offset(self, offset: tuple[float, float, float]) -> None:
        """Move the entire point cloud in 3D space without re-uploading point buffers."""
        self.cfg.world_offset = (float(offset[0]), float(offset[1]), float(offset[2]))
        if hasattr(self, "_translate_op") and self._translate_op:
            self._translate_op.Set(Gf.Vec3d(*self.cfg.world_offset))

    def set_point_radius(self, radius: float) -> None:
        """Adjust sphere radius dynamically (also scales inverted-hull outline shell if active)."""
        self.cfg.point_radius = float(max(0.0005, radius))
        self._hybrid_dirty = True
        diam = self.cfg.point_radius * 2.0
        for w_attr, cnt, scale_f in self.width_attrs:
            w_attr.Set(Vt.FloatArray.FromNumpy(np.full(cnt, diam * scale_f, dtype=np.float32)))
        for sc_op, scale_f in self.proto_scale_ops:
            r_eff = self.cfg.point_radius * scale_f
            sc_op.Set(Gf.Vec3d(r_eff, r_eff, r_eff))
        if self._outline_scale_op is not None and self.cfg.outline_thickness > 0.0:
            r_out = self.cfg.point_radius * (1.0 + self.cfg.outline_thickness)
            self._outline_scale_op.Set(Gf.Vec3d(r_out, r_out, r_out))
        if self.cfg.method == "naive_spheres":
            for prim in self.geometry_prims:
                UsdGeom.Sphere(prim).GetRadiusAttr().Set(self.cfg.point_radius)

    def set_appearance(
        self,
        *,
        unlit: bool | None = None,
        glossy: bool | None = None,
        roughness: float | None = None,
        metallic: float | None = None,
        specular_level: float | None = None,
    ) -> None:
        """Toggle or fine-tune zero-reflection Unlit / Matte / Glossy appearance."""
        if unlit is not None:
            self.cfg.unlit = bool(unlit)
        if glossy is not None:
            self.cfg.glossy = bool(glossy)
            if roughness is None:
                self.cfg.roughness = None
            if specular_level is None:
                self.cfg.specular_level = 0.85 if self.cfg.glossy else 0.0
            if metallic is None and not self.cfg.glossy:
                self.cfg.metallic = 0.0
        if roughness is not None:
            self.cfg.roughness = float(np.clip(roughness, 0.02, 1.0))
        if metallic is not None:
            self.cfg.metallic = float(np.clip(metallic, 0.0, 1.0))
        if specular_level is not None:
            self.cfg.specular_level = float(np.clip(specular_level, 0.0, 1.0))

        self._apply_material_inputs()

    def set_outline(
        self,
        thickness: float,
        color: tuple[float, float, float] | None = None,
    ) -> None:
        """Add, resize, or remove inverted-hull black edge outline (thickness = ratio of point_radius; 0.0 = off)."""
        self.cfg.outline_thickness = float(max(0.0, thickness))
        if color is not None:
            self.cfg.outline_color = (float(color[0]), float(color[1]), float(color[2]))

        out_path = f"{self.cfg.root_path}/OutlineShell"
        if self.cfg.outline_thickness <= 0.0 or self.cfg.method == "warp_compute_atomic64":
            if self.stage.GetPrimAtPath(out_path).IsValid():
                self.stage.RemovePrim(out_path)
            self._outline_scale_op = None
            self._outline_shader = None
            return

        if self.num_points > self.cfg.max_outline_points:
            # Avoid VRAM overflow on ultra-large point clouds (>1.5M) unless explicitly raised
            if self.stage.GetPrimAtPath(out_path).IsValid():
                self.stage.RemovePrim(out_path)
            self._outline_scale_op = None
            self._outline_shader = None
            return

        r_out = self.cfg.point_radius * (1.0 + self.cfg.outline_thickness)
        oc = self.cfg.outline_color
        if self._outline_scale_op is not None and self.stage.GetPrimAtPath(out_path).IsValid():
            self._outline_scale_op.Set(Gf.Vec3d(r_out, r_out, r_out))
            if self._outline_shader is not None:
                self._outline_shader.GetInput("emissiveColor").Set(Gf.Vec3f(oc[0], oc[1], oc[2]))
            carb.settings.get_settings().set_bool("/rtx/hydra/faceCulling/enabled", True)
            return

        inst = UsdGeom.PointInstancer.Define(self.stage, out_path)
        sp = self._define_inward_unit_sphere(self.stage, f"{out_path}/ShellProto")
        prim = sp.GetPrim()
        prim.CreateAttribute("singleSided", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        self._apply_shadow_flag_to_prim(inst.GetPrim(), False)

        sc_op = UsdGeom.Xformable(prim).AddScaleOp()
        sc_op.Set(Gf.Vec3d(r_out, r_out, r_out))
        self._outline_scale_op = sc_op

        mat = UsdShade.Material.Define(self.stage, f"{out_path}/ShellMat")
        sh = UsdShade.Shader.Define(self.stage, f"{out_path}/ShellMat/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(oc[0], oc[1], oc[2]))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        self._outline_shader = sh

        inst.CreatePrototypesRel().SetTargets([sp.GetPath()])
        inst.GetPositionsAttr().Set(Vt.Vec3fArray.FromNumpy(self.positions))
        inst.GetProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(np.zeros(self.num_points, dtype=np.int32)))
        carb.settings.get_settings().set_bool("/rtx/hydra/faceCulling/enabled", True)

    def set_anti_aliasing(self, mode: AntiAliasingType | int = "DLAA") -> None:
        """Set RTX post anti-aliasing: 'DLAA' (4, crisp edges) or 'DLSS' (3, upscaled)."""
        self.cfg.anti_aliasing = mode
        carb.settings.get_settings().set_int("/rtx/post/aa/op", self.cfg.effective_aa_op)

    def set_cast_shadows(self, cast_shadows: bool, light_paths: list[str] | None = None) -> None:
        """Toggle shadow casting (影有り無し) on the point cloud prims and optional lights."""
        self.cfg.cast_shadows = bool(cast_shadows)
        if self.root_prim and self.root_prim.IsValid():
            self._apply_shadow_flag_to_prim(self.root_prim, self.cfg.cast_shadows)
        for prim in self.geometry_prims:
            if prim and prim.IsValid():
                self._apply_shadow_flag_to_prim(prim, self.cfg.cast_shadows)
        for lp in light_paths or ["/World/KeyLight", "/World/Sun"]:
            lprim = self.stage.GetPrimAtPath(lp)
            if lprim and lprim.IsValid():
                sh_api = UsdLux.ShadowAPI.Apply(lprim)
                sh_api.CreateShadowEnableAttr().Set(bool(cast_shadows))

    def set_colors(self, colors: np.ndarray | tuple[float, float, float]) -> None:
        """Update point cloud RGB colors."""
        self.colors = self._normalize_colors(colors, self.num_points)
        self._hybrid_dirty = True
        if self.cfg.method in ("hybrid_compute_rtx", "warp_compute_atomic64") and self._wp_col is not None:
            self._wp_col = wp.array(self.colors, dtype=wp.vec3, device="cuda:0")
        elif self.cfg.method == "point_instancer" and self.color_primvars:
            self.color_primvars[0].Set(Vt.Vec3fArray.FromNumpy(self.colors))
        elif self.cfg.method == "usd_points":
            chunk_sz = max(1, int(self.cfg.chunk_size))
            for ci, cpv in enumerate(self.color_primvars):
                s = ci * chunk_sz
                e = min(self.num_points, s + chunk_sz)
                cpv.Set(Vt.Vec3fArray.FromNumpy(self.colors[s:e]))

    # =========================================================================
    # Per-Frame Camera Update (for Octree LOD, Hybrid Compute, and Atomic64)
    # =========================================================================
    def update_camera(
        self,
        cam_eye: tuple[float, float, float],
        cam_target: tuple[float, float, float] | None = None,
        cam_fwd: tuple[float, float, float] | None = None,
        fov_deg: float = 60.0,
    ) -> dict[str, Any]:
        """Update camera-dependent LOD / Frustum Culling / Compute Rasterizer."""
        ex, ey, ez = float(cam_eye[0]), float(cam_eye[1]), float(cam_eye[2])
        if cam_fwd is not None:
            fx, fy, fz = float(cam_fwd[0]), float(cam_fwd[1]), float(cam_fwd[2])
        elif cam_target is not None:
            dx = float(cam_target[0]) - ex
            dy = float(cam_target[1]) - ey
            dz = float(cam_target[2]) - ez
            ln = max(1e-6, math.sqrt(dx * dx + dy * dy + dz * dz))
            fx, fy, fz = dx / ln, dy / ln, dz / ln
        else:
            fx, fy, fz = 0.0, 1.0, 0.0

        method = self.cfg.method
        if method == "octree_lod":
            return self._update_octree_lod((ex, ey, ez), (fx, fy, fz))
        if method == "hybrid_compute_rtx":
            return self._update_hybrid_compute((ex, ey, ez), (fx, fy, fz))
        if method == "warp_compute_atomic64":
            return self._update_warp_atomic64((ex, ey, ez), (fx, fy, fz), fov_deg=fov_deg)
        return {"active_spheres": self.num_points, "draw_calls": len(self.geometry_prims)}

    def _update_octree_lod(
        self,
        eye: tuple[float, float, float],
        fwd: tuple[float, float, float],
    ) -> dict[str, Any]:
        ex, ey, ez = eye
        fx, fy, fz = fwd
        ox, oy, oz = self.cfg.world_offset
        active_total = 0
        counts = {0: 0, 1: 0, 2: 0, -1: 0}
        vis_changes = 0

        for b_idx, (bcx, bcy, bcz) in enumerate(self._octree_centers):
            dx = (bcx + ox) - ex
            dy = (bcy + oy) - ey
            dz = (bcz + oz) - ez
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            fwd_dist = dx * fx + dy * fy + dz * fz

            if self.cfg.frustum_cull and (
                fwd_dist < -2.2 or (dist > 3.2 and fwd_dist / max(0.001, dist) < 0.30)
            ):
                sel = -1
            elif dist < self.cfg.octree_near_dist:
                sel = 0
            elif dist < self.cfg.octree_mid_dist:
                sel = 1
            else:
                sel = 2

            counts[sel] += 1
            blk = self._octree_blocks[b_idx]
            if sel >= 0:
                active_total += blk["lods"][sel]["k_pts"]

            if sel != self._octree_prev_lods[b_idx]:
                self._octree_prev_lods[b_idx] = sel
                vis_changes += 1
                if sel == -1:
                    blk["block_vis_attr"].Set(UsdGeom.Tokens.invisible)
                else:
                    blk["block_vis_attr"].Set(UsdGeom.Tokens.inherited)
                    for lod_i, lod_entry in enumerate(blk["lods"]):
                        lod_entry["vis_attr"].Set(
                            UsdGeom.Tokens.inherited if lod_i == sel else UsdGeom.Tokens.invisible
                        )

        self._octree_active_spheres = active_total
        return {
            "active_spheres": active_total,
            "lod_counts": counts,
            "usd_visibility_writes": vis_changes,
        }

    def _update_hybrid_compute(
        self,
        eye: tuple[float, float, float],
        fwd: tuple[float, float, float],
    ) -> dict[str, Any]:
        assert self._wp_out_count is not None and self._hybrid_points_prim is not None
        cam_now = (eye[0], eye[1], eye[2], fwd[0], fwd[1], fwd[2])
        if not self._hybrid_dirty and self._hybrid_last_cam is not None:
            lx, ly, lz, lfx, lfy, lfz = self._hybrid_last_cam
            pos_delta = math.sqrt((eye[0] - lx) ** 2 + (eye[1] - ly) ** 2 + (eye[2] - lz) ** 2)
            dir_dot = eye and (fwd[0] * lfx + fwd[1] * lfy + fwd[2] * lfz)
            # Skip USD points rewrite when camera has not moved significantly to prevent RTX TAA/BVH flicker
            if pos_delta < 0.04 and dir_dot > 0.9992:
                return {"active_spheres": self._hybrid_last_active}

        self._hybrid_last_cam = cam_now
        self._hybrid_dirty = False
        self._wp_out_count.zero_()
        budget = min(self.num_points, int(self.cfg.hybrid_max_budget))
        ox, oy, oz = self.cfg.world_offset
        local_eye = (eye[0] - ox, eye[1] - oy, eye[2] - oz)
        wp.launch(
            _kernel_cull_and_lod,
            dim=self.num_points,
            inputs=[
                self._wp_pos,
                self._wp_col,
                self.num_points,
                wp.vec3(*local_eye),
                wp.vec3(*fwd),
                wp.vec3(0.0, 0.0, 0.0),
                float(self.cfg.point_radius * 2.0),
                float(self.cfg.octree_near_dist),
                float(self.cfg.octree_mid_dist),
                1 if self.cfg.frustum_cull else 0,
                budget,
                self._wp_out_count,
                self._wp_out_pos,
                self._wp_out_col,
                self._wp_out_widths,
            ],
            device="cuda:0",
        )
        active = min(budget, int(self._wp_out_count.numpy()[0]))
        self._hybrid_last_active = active
        if active > 0:
            pos_np = self._wp_out_pos.numpy()[:active]
            col_np = self._wp_out_col.numpy()[:active]
            wid_np = self._wp_out_widths.numpy()[:active]
            self._hybrid_points_prim.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pos_np))
            self._hybrid_points_prim.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(wid_np))
            cpv = UsdGeom.PrimvarsAPI(self._hybrid_points_prim).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
            )
            cpv.Set(Vt.Vec3fArray.FromNumpy(col_np))
        return {"active_spheres": active}

    def _update_warp_atomic64(
        self,
        eye: tuple[float, float, float],
        fwd: tuple[float, float, float],
        fov_deg: float = 60.0,
    ) -> dict[str, Any]:
        assert self._wp_fb64 is not None and self._wp_rgba is not None
        w, h = self.cfg.compute_resolution
        fx, fy, fz = fwd
        up_x, up_y, up_z = 0.0, 0.0, 1.0
        rx = fy * up_z - fz * up_y
        ry = fz * up_x - fx * up_z
        rz = fx * up_y - fy * up_x
        r_len = max(1e-6, math.sqrt(rx * rx + ry * ry + rz * rz))
        rx, ry, rz = rx / r_len, ry / r_len, rz / r_len
        ux = ry * fz - rz * fy
        uy = rz * fx - rx * fz
        uz = rx * fy - ry * fx

        focal_px = (w * 0.5) / math.tan(math.radians(fov_deg * 0.5))
        bg_pack = np.uint64((0x7FFFFFFF << 32) | (16 << 16) | (22 << 8) | 34)

        wp.launch(_kernel_clear_fb64, dim=w * h, inputs=[self._wp_fb64, w * h, bg_pack], device="cuda:0")
        wp.launch(
            _kernel_rasterize_spheres_atomic64,
            dim=self.num_points,
            inputs=[
                self._wp_pos,
                self._wp_col,
                self.num_points,
                wp.vec3(*self.cfg.world_offset),
                wp.vec3(*eye),
                wp.vec3(rx, ry, rz),
                wp.vec3(ux, uy, uz),
                wp.vec3(fx, fy, fz),
                float(focal_px),
                w,
                h,
                float(self.cfg.point_radius),
                self._wp_fb64,
            ],
            device="cuda:0",
        )
        wp.launch(
            _kernel_resolve_fb64_shaded,
            dim=w * h,
            inputs=[
                self._wp_fb64,
                self._wp_rgba,
                w,
                h,
                float(self.cfg.effective_roughness),
                float(self.cfg.effective_specular),
                1 if self.cfg.cast_shadows else 0,
            ],
            device="cuda:0",
        )
        self.last_compute_image = self._wp_rgba.numpy()[..., :3].copy()
        return {"active_spheres": self.num_points}


def create_pointcloud(
    stage: Usd.Stage,
    positions: np.ndarray,
    colors: np.ndarray | tuple[float, float, float] = (0.2, 0.75, 1.0),
    *,
    method: MethodType = "octree_lod",
    root_path: str = "/World/PointCloud",
    point_radius: float = 0.035,
    world_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    unlit: bool = True,
    outline_thickness: float = 0.08,
    outline_color: tuple[float, float, float] = (0.0, 0.0, 0.0),
    anti_aliasing: AntiAliasingType | int = "DLAA",
    glossy: bool = False,
    roughness: float | None = None,
    metallic: float = 0.0,
    specular_level: float = 0.0,
    cast_shadows: bool = False,
    **kwargs: Any,
) -> UnifiedPointCloud:
    """One-call unified factory function for all 6 point cloud rendering patterns.

    Default appearance is Preset F2 (unlit=True, outline_thickness=0.08, anti_aliasing='DLAA', cast_shadows=False).
    """
    cfg = PointCloudConfig(
        method=method,
        root_path=root_path,
        point_radius=point_radius,
        world_offset=world_offset,
        unlit=unlit,
        outline_thickness=outline_thickness,
        outline_color=outline_color,
        anti_aliasing=anti_aliasing,
        glossy=glossy,
        roughness=roughness,
        metallic=metallic,
        specular_level=specular_level,
        cast_shadows=cast_shadows,
        **kwargs,
    )
    return UnifiedPointCloud(stage=stage, positions=positions, colors=colors, config=cfg)
