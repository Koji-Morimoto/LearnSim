"""Isaac Sim で点群を「球」で表示し、球のふち(エッジ)をくっきり見せるための関数集。

作成日: 2026-11-12 / Isaac Sim 6.2.0 (RTX Real-Time) で動作確認

使い方(Isaac Sim の Script Editor でも、python.bat のスクリプトからでも同じ):

    import numpy as np, omni.usd
    from pointcloud_spheres import add_sphere_cloud, set_crisp_render, add_outline_shell

    stage = omni.usd.get_context().get_stage()
    pts = np.random.rand(2000, 3)                    # (N, 3) [m]
    cols = np.random.rand(2000, 3)                   # (N, 3) 0〜1 の RGB(省略可)
    add_sphere_cloud(stage, "/World/Cloud", pts, cols, radius=0.01)   # 1) 点を球にする
    set_crisp_render()                               # 2) DLSS の拡大をやめて DLAA にする(にじみの主因)
    add_outline_shell(stage, "/World/Cloud", pts, radius=0.01)        # 3) 球に黒い縁取りを付ける(任意)
    set_cast_shadows(stage, "/World/Cloud", False)   # 4) 影を落とさない(任意。あり/なしは後から何度でも切り替えられる)

各関数の要点:
    add_points_prim    UsdGeom.Points。RTX では各点が球として描かれる。1 つの prim で最も軽い。
    add_sphere_cloud   UsdGeom.PointInstancer + Sphere。本物の球のメッシュ。色ごとに材質を分けられる。
    set_crisp_render   /rtx/post/aa/op = 4 (DLAA)。Isaac Sim の既定は DLSS Performance で、
                       内部で縦横半分の解像度で描いて拡大するため、ふちがぼける。
    add_outline_shell  「裏返した殻(inverted hull)」で球ごとに黒い縁取りを付ける。
    影のあり/なし(2 通り。どちらも後から切り替えられる):
      set_cast_shadows   点群ごと。その点群が影を落とすか(primvars:doNotCastShadows)。ほかの物体の影はそのまま。
                         add_points_prim / add_sphere_cloud の cast_shadows 引数でも作るときに選べる。
      set_shadows        描画全体。ステージ上のすべての光源の影を出すか(UsdLux.ShadowAPI の shadow:enable)。
    set_ambient_occlusion  環境光の遮蔽(AO。球の間のすき間が暗くなる効果)を出すか。
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

# ----------------------------------------------------------------------------- 点群を球で出す


def add_points_prim(
    stage,
    path: str,
    points: np.ndarray,
    colors: np.ndarray | None = None,
    radius: float = 0.01,
    cast_shadows: bool = True,
):
    """UsdGeom.Points で点群を置く。RTX は各点を半径 width/2 の球として描く。

    Args:
        stage: Usd.Stage
        path: prim のパス
        points: (N, 3) の位置 [m]
        colors: (N, 3) の RGB(0〜1)。None なら白
        radius: 球の半径 [m]
        cast_shadows: False なら、この点群は影を落とさない(set_cast_shadows で後から変えられる)
    """
    from pxr import Gf, UsdGeom, Vt

    prim = UsdGeom.Points.Define(stage, path)
    prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, np.float32)))
    prim.CreateWidthsAttr(Vt.FloatArray([2.0 * radius] * len(points)))
    prim.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    if colors is not None:
        pv = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec3fArray.FromNumpy(np.asarray(colors, np.float32)))
    else:
        prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(1, 1, 1)])
    set_cast_shadows(stage, path, cast_shadows)
    return prim


def _bin_colors(colors: np.ndarray | None, n: int, n_bins: int) -> tuple[np.ndarray, np.ndarray]:
    """点ごとの色を n_bins 色に丸める。戻り値は (各点の色番号 (N,), 色の表 (K, 3))。"""
    if colors is None:
        return np.zeros(n, np.int32), np.ones((1, 3), np.float32)
    colors = np.asarray(colors, np.float32)
    k = max(1, min(n_bins, n))
    # 小さな k-means(RGB 空間)。初期値は点を等間隔に選ぶ。大きな点群は間引いた点で中心を求める
    rng = np.random.default_rng(0)
    sample = colors if n <= 20000 else colors[rng.choice(n, 20000, replace=False)]
    table = sample[np.linspace(0, len(sample) - 1, k).astype(int)].copy()
    for _ in range(15):
        lab = np.argmin(((sample[:, None, :] - table[None]) ** 2).sum(-1), axis=1)
        for b in range(k):
            if np.any(lab == b):
                table[b] = sample[lab == b].mean(axis=0)
    ids = np.empty(n, np.int32)
    for s in range(0, n, 200000):  # メモリを食わないように分けて割り当てる
        ids[s:s + 200000] = np.argmin(((colors[s:s + 200000, None, :] - table[None]) ** 2).sum(-1), axis=1)
    return ids, table


def make_material(stage, path: str, color: Sequence[float], flat: bool = False, roughness: float = 0.6):
    """UsdPreviewSurface の材質を作る。flat=True なら自己発光にして、光の当たり方で暗くならない色にする。"""
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    c = Gf.Vec3f(*[float(x) for x in color])
    if flat:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(c)
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
        sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(c)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def add_sphere_cloud(
    stage,
    path: str,
    points: np.ndarray,
    colors: np.ndarray | None = None,
    radius: float = 0.01,
    n_bins: int = 16,
    flat: bool = False,
    cast_shadows: bool = True,
):
    """UsdGeom.PointInstancer で、点ごとに本物の球(UsdGeom.Sphere)を置く。

    色は n_bins 色に丸め、色ごとに球の原型(prototype)と材質を 1 つずつ作る。
    数十万点でも instancer 1 つなので USD は軽い(描画は球の数に比例して重くなる)。

    Args:
        flat: True なら自己発光の材質(陰影なし・べた塗り)。False なら普通の光の当たる材質。
        cast_shadows: False なら、この点群は影を落とさない(set_cast_shadows で後から変えられる)
    Returns:
        UsdGeom.PointInstancer
    """
    from pxr import Gf, UsdGeom, UsdShade, Vt

    points = np.asarray(points, np.float32)
    ids, table = _bin_colors(colors, len(points), n_bins)
    inst = UsdGeom.PointInstancer.Define(stage, path)
    protos = inst.CreatePrototypesRel()
    UsdGeom.Scope.Define(stage, path + "/Prototypes")
    UsdGeom.Scope.Define(stage, path + "/Looks")
    for b, col in enumerate(table):
        sp = UsdGeom.Sphere.Define(stage, f"{path}/Prototypes/S{b:03d}")
        sp.CreateRadiusAttr(float(radius))
        sp.CreateDisplayColorAttr([Gf.Vec3f(*[float(x) for x in col])])
        mat = make_material(stage, f"{path}/Looks/M{b:03d}", col, flat=flat)
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(mat)
        protos.AddTarget(sp.GetPath())
    inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(points))
    inst.CreateProtoIndicesAttr(Vt.IntArray.FromNumpy(ids))
    set_cast_shadows(stage, path, cast_shadows)
    return inst


def add_outline_shell(
    stage,
    path: str,
    points: np.ndarray,
    radius: float = 0.01,
    thickness: float = 0.25,
    color: Sequence[float] = (0.0, 0.0, 0.0),
):
    """球ごとに黒い縁取りを付ける(inverted hull 法)。

    各点に「少し大きい黒い球」を、面を内向きにしたメッシュ・片面表示で置く。カメラ側の半球は
    裏面になって見えず、奥側の半球の内側だけが元の球の後ろに黒い輪として見える。
    どの向きから見ても縁取りになる。

    RTX の注意点(実際にはまった):
      - RTX は doubleSided を無視する。片面にするには prim に `singleSided = True`(bool)を付け、
        さらに描画設定 `/rtx/hydra/faceCulling/enabled = True` を入れる必要がある(この関数が両方やる)。
      - faceCulling は singleSided を付けた prim にだけ効くので、ほかの物体の見た目は変わらない。

    Args:
        path: 元の点群の prim パス(縁取りは path + "_Outline" に作る)
        thickness: 縁取りの太さ(半径に対する比。0.25 なら半径の 25% ぶん外側に出る)
        color: 縁取りの色
    Returns:
        UsdGeom.PointInstancer
    """
    import carb.settings
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    out = path + "_Outline"
    inst = UsdGeom.PointInstancer.Define(stage, out)
    sp = _inward_sphere_mesh(stage, out + "/Prototypes/Shell", radius * (1.0 + thickness))
    sp.CreateDoubleSidedAttr(False)
    prim = sp.GetPrim()
    prim.CreateAttribute("singleSided", Sdf.ValueTypeNames.Bool).Set(True)  # RTX が見るのはこちら
    prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)  # 縁取りは影を落とさない
    prim.SetCustomDataByKey("pcs:outline", True)  # set_cast_shadows / set_shadows で触らない印
    sp.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    mat = make_material(stage, out + "/Looks/Shell", color, flat=True)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    inst.CreatePrototypesRel().AddTarget(sp.GetPath())
    inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, np.float32)))
    inst.CreateProtoIndicesAttr(Vt.IntArray([0] * len(points)))
    carb.settings.get_settings().set_bool("/rtx/hydra/faceCulling/enabled", True)
    return inst


def _inward_sphere_mesh(stage, path: str, radius: float, n_lon: int = 24, n_lat: int = 16):
    """面(三角形の巻き方向)と法線が内側を向いた UV 球のメッシュを作る。"""
    from pxr import UsdGeom, Vt

    lat = np.linspace(0, np.pi, n_lat + 1)
    lon = np.linspace(0, 2 * np.pi, n_lon, endpoint=False)
    v = [(0.0, 0.0, radius)]
    for th in lat[1:-1]:
        for ph in lon:
            v.append((radius * np.sin(th) * np.cos(ph), radius * np.sin(th) * np.sin(ph), radius * np.cos(th)))
    v.append((0.0, 0.0, -radius))
    idx, counts = [], []

    def ring(i):  # i 番目の緯度の輪(極を除く)の先頭の頂点番号
        return 1 + i * n_lon

    for j in range(n_lon):  # 北極の扇。外から見て時計回り = 内向き
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
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(-pts / radius))  # 法線も内向き
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh


# ----------------------------------------------------------------------------- 描画の設定


def set_crisp_render(mode: str = "DLAA", dlss_quality: str = "Quality") -> None:
    """アンチエイリアスの方式を変える。にじみの主因はここ。

    起動時から DLAA にするなら:
      - python.bat のスクリプト: SimulationApp({"anti_aliasing": 4, ...})
        (SimulationApp は起動時に anti_aliasing(既定 3)で /rtx/post/aa/op を上書きするので、
         extra_args の --/rtx/post/aa/op=4 は効かない)
      - GUI: isaac-sim.bat --/rtx/post/aa/op=4

    Args:
        mode: "DLAA"(拡大なし。おすすめ) / "DLSS"(低い解像度で描いて拡大)
        dlss_quality: mode="DLSS" のときの質。"Performance"(Isaac Sim の既定) / "Balanced" / "Quality" / "Auto"
    """
    import carb.settings

    s = carb.settings.get_settings()
    if mode.upper() == "DLAA":
        s.set_int("/rtx/post/aa/op", 4)
    else:
        s.set_int("/rtx/post/aa/op", 3)
        s.set_int("/rtx/post/dlss/execMode", {"Performance": 0, "Balanced": 1, "Quality": 2, "Auto": 3}[dlss_quality])


def set_cast_shadows(stage, path: str, enabled: bool) -> None:
    """点群(path)が影を落とすかを切り替える。ほかの物体の影や、点群が受ける影はそのまま。

    primvars:doNotCastShadows を付ける。PointInstancer は原型(prototype)の球それぞれにも付ける
    (原型は instancer の子なので instancer の primvar も受け継ぐが、確実にするため)。
    """
    from pxr import Sdf, Usd, UsdGeom

    root = stage.GetPrimAtPath(path)
    for prim in Usd.PrimRange(root):
        if prim.IsPseudoRoot() or prim.GetCustomDataByKey("pcs:outline"):
            continue  # 根元("/")には書けない。縁取りの殻はいつも影なしのままにする
        if prim.IsA(UsdGeom.Gprim) or prim == root:
            pv = UsdGeom.PrimvarsAPI(prim).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool, UsdGeom.Tokens.constant)
            pv.Set(not enabled)


def set_shadows(stage, enabled: bool, root: str = "/") -> None:
    """描画全体の影を切り替える(root 以下のすべての形状)。

    root 以下のすべての Gprim に primvars:doNotCastShadows を付ける(set_cast_shadows と同じ仕組み)。
    後から足した物体には効かないので、物体を足したら呼び直す。
    set_shadows(stage, True) は root 以下を全部「影あり」に戻す(set_cast_shadows で個別に切った点群も戻る)。

    注意(Isaac Sim 6.2 で実測): 既定の描画方式 RealTimePathTracing(RTX Real-Time 2.0)では、
    描画設定 /rtx/shadows/enabled も、光源の UsdLux.ShadowAPI(inputs:shadow:enable)も影を消さなかった。
    旧方式(RaytracedLighting)向けに両方とも合わせて設定はしておく。
    """
    import carb.settings
    from pxr import UsdLux

    set_cast_shadows(stage, root, enabled)
    carb.settings.get_settings().set_bool("/rtx/shadows/enabled", bool(enabled))
    for prim in stage.Traverse():
        if prim.IsA(UsdLux.BoundableLightBase) or prim.IsA(UsdLux.NonboundableLightBase):
            UsdLux.ShadowAPI.Apply(prim).CreateShadowEnableAttr().Set(bool(enabled))


def set_ambient_occlusion(enabled: bool) -> None:
    """環境光の遮蔽(AO)を切り替える。"""
    import carb.settings

    carb.settings.get_settings().set_bool("/rtx/ambientOcclusion/enabled", bool(enabled))


def add_selection_outline(paths: Sequence[str], color=(1.0, 0.6, 0.0, 1.0), width: int = 2) -> int:
    """ビューポートの「選択時の縁取り」の仕組みで、指定した prim に縁取りを付ける(ビューポート表示のみ)。

    実測(Isaac Sim 6.2): GUI のビューポートでは、点群と背景の境目(見えている球のふち)に線が付く。
    同じ点群の中で重なった球どうしの境目には付かない。replicator のカメラ(render product)の画像には写らない。
    """
    import carb.settings
    import omni.usd

    ctx = omni.usd.get_context()
    carb.settings.get_settings().set_int("/persistent/app/viewport/outline/width", int(width))
    gid = ctx.register_selection_group()
    ctx.set_selection_group_outline_color(gid, tuple(color))
    ctx.set_selection_group_shade_color(gid, (0.0, 0.0, 0.0, 0.0))
    for p in paths:
        ctx.set_selection_group(gid, p)
    return gid


# ----------------------------------------------------------------------------- 試し用の点群


def demo_cloud(n: int = 1500, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """試し用の点群(トーラス結び目の管の表面)と、高さで色分けした色を返す。"""
    rng = np.random.default_rng(seed)
    t = rng.uniform(0, 2 * np.pi, n)
    a = rng.uniform(0, 2 * np.pi, n)
    p, q, R, r_tube = 2, 3, 0.18, 0.045
    c = np.stack([(R + 0.08 * np.cos(q * t)) * np.cos(p * t), (R + 0.08 * np.cos(q * t)) * np.sin(p * t), 0.08 * np.sin(q * t)], 1)
    # 管の断面の 2 方向(近似): 中心軸から外向きと z 方向
    out = c * np.array([1, 1, 0])
    out /= np.linalg.norm(out, axis=1, keepdims=True) + 1e-9
    up = np.array([0, 0, 1.0])
    pts = c + r_tube * (np.cos(a)[:, None] * out + np.sin(a)[:, None] * up)
    pts[:, 2] += 0.25
    h = (pts[:, 2] - pts[:, 2].min()) / np.ptp(pts[:, 2])
    cols = np.stack([0.15 + 0.85 * h, 0.35 + 0.4 * np.sin(np.pi * h), 1.0 - 0.8 * h], 1)
    return pts.astype(np.float32), np.clip(cols, 0, 1).astype(np.float32)
