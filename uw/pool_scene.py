"""The M11 pool's USD stage: a tiled test tank, its free surface, the hall lights, and the BlueROV2 Heavy with its
camera on the tilt servo and its Lumen lights.

Visual only: nothing here is acoustic and nothing collides (the vehicle's bound is the dynamics' soft box, which
for this square tank is exactly the walls). Needs pxr and numpy only, so tests build the stage in memory without
the Kit app. Frames: tank/ENU with the origin at a floor corner; the vehicle prim is in body FLU, posed every frame
from the physics process (``set_vehicle``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

from . import pool_geometry
from .config import WS, get
from .sinks import write_png
from .usd_scene import HORIZONTAL_APERTURE_MM, R_BODY_USDCAM, _bind, _look_at, _matrix, _mdl_material, _xform

WORLD = "/World"
POOL = f"{WORLD}/pool"
LOOKS = f"{WORLD}/Looks"
ROV = f"{WORLD}/rov"
TILT = f"{ROV}/camera_tilt"              # rotates about body y with the tilt servo
CAMERA = f"{TILT}/camera"
LAMPS = f"{ROV}/lamps"
CHASE = f"{ROV}/chase"                   # a third-person camera behind and above the vehicle (`uw pool --view chase`)
CHASE_OFFSET = (-1.3, 0.0, 0.55)         # body FLU [m]
WATER_IOR_FROM_BELOW = 1.0 / 1.333       # the camera is in water: air above is the "less dense" side (as GAS_IOR)
TEXTURE_DIR = Path(WS) / "output" / "pool" / "textures"
TILES_PER_TEXTURE = 8                    # the tile texture covers 8 x 8 tiles, each with its own slight tint


@dataclass
class PoolInfo:
    camera: str                          # the onboard camera
    chase: str
    camera_K: np.ndarray
    camera_resolution: tuple
    lamps: list
    lamp_intensity: float
    vehicle_asset: str | None            # the referenced USD, or None when the box stands in for it


# ------------------------------------------------------------------------------------------------ textures

def tile_texture(cfg: dict, px_per_tile: int = 128) -> np.ndarray:
    """RGB uint8 image of TILES_PER_TEXTURE^2 tiles with grout lines; per-tile tint +-4 %, seeded."""
    rng = np.random.default_rng(int(get(cfg, "seed")))
    n = TILES_PER_TEXTURE * px_per_tile
    tile = np.asarray(get(cfg, "pool.tile_color"), float)
    grout = np.asarray(get(cfg, "pool.grout_color"), float)
    tint = 1.0 + 0.04 * rng.standard_normal((TILES_PER_TEXTURE, TILES_PER_TEXTURE, 1))
    img = np.repeat(np.repeat(tint, px_per_tile, 0), px_per_tile, 1) * tile
    g = max(2, px_per_tile // 40)
    edge = (np.arange(n) % px_per_tile) < g
    img[edge, :] = grout
    img[:, edge] = grout
    return np.clip(np.round(_srgb(img) * 255.0), 0, 255).astype(np.uint8)


def checker_texture(squares: int = 8, px: int = 64) -> np.ndarray:
    """A black-and-white checkerboard with a white border: the marker board on the floor (a camera target)."""
    b = np.indices((squares, squares)).sum(0) % 2
    img = np.kron(b, np.ones((px, px)))
    img = np.pad(img, px // 2, constant_values=1.0)
    return np.repeat((img * 230 + 15).astype(np.uint8)[..., None], 3, axis=2)


def _srgb(linear: np.ndarray) -> np.ndarray:
    """Linear RGB -> sRGB encoding, for colours given in linear RGB but stored in an 8-bit texture."""
    x = np.clip(linear, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


# ------------------------------------------------------------------------------------------------ primitives

def _quad(stage, path: str, corners, uv_scale: float | None = None):
    """A double-sided quad from 4 corners (counter-clockwise seen from its front); ``uv_scale`` [m per UV unit]
    gives it st coordinates from the in-plane distances, so a texture repeats every uv_scale metres."""
    c = np.asarray(corners, float)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(c.astype(np.float32)))
    m.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr().Set(True)
    m.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, c.min(0))), Gf.Vec3f(*map(float, c.max(0)))]))
    if uv_scale:
        u = np.linalg.norm(c[1] - c[0]) / uv_scale
        v = np.linalg.norm(c[3] - c[0]) / uv_scale
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(u, 0), Gf.Vec2f(u, v), Gf.Vec2f(0, v)]))
    return m


def _box(stage, path: str, center, size, rot=np.eye(3)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(cube)
    x.AddTransformOp().Set(_matrix(rot, center))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    return cube


def _cylinder(stage, path: str, center, radius: float, height: float, axis: str = "Z"):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr(axis)
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    return cyl


def _surface(stage, path: str, color, roughness: float, metallic: float = 0.0, texture: Path | None = None):
    """UsdPreviewSurface, optionally with an sRGB texture on the st primvar (repeating)."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    diffuse = sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    if texture is None:
        diffuse.Set(Gf.Vec3f(*map(float, color)))
    else:
        st = UsdShade.Shader.Define(stage, f"{path}/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, f"{path}/texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        diffuse.ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


# ------------------------------------------------------------------------------------------------ the stage

def build(stage: Usd.Stage, cfg: dict, texture_dir: Path = TEXTURE_DIR) -> PoolInfo:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, WORLD)
    UsdGeom.Scope.Define(stage, LOOKS)
    _tank(stage, cfg, Path(texture_dir))
    _hall_lights(stage, cfg)
    asset, lamps = _vehicle(stage, cfg)
    K, res, focal = camera_intrinsics(cfg)
    cam = UsdGeom.Camera.Define(stage, CAMERA)
    UsdGeom.Xformable(cam).AddTransformOp().Set(_matrix(R_BODY_USDCAM, (0.0, 0.0, 0.0)))
    cam.CreateProjectionAttr("perspective")
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE_MM)
    cam.CreateVerticalApertureAttr(HORIZONTAL_APERTURE_MM * res[1] / res[0])
    lo, hi = (float(v) for v in get(cfg, "camera.clip_m"))
    cam.CreateClippingRangeAttr(Gf.Vec2f(lo, hi))
    chase = UsdGeom.Camera.Define(stage, CHASE)
    target = np.array([0.3, 0.0, 0.0])
    look = _look_at(CHASE_OFFSET, target)              # USD camera: -z towards the target, x horizontal
    UsdGeom.Xformable(chase).AddTransformOp().Set(_matrix(look, CHASE_OFFSET))
    chase.CreateFocalLengthAttr(focal)
    chase.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE_MM)
    chase.CreateVerticalApertureAttr(HORIZONTAL_APERTURE_MM * res[1] / res[0])
    chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, hi))
    x, y, z, yaw = (float(v) for v in get(cfg, "vehicle.start"))
    set_vehicle(stage, (x, y, z), _rot_z(math.radians(yaw)))
    set_tilt(stage, 0.0)
    return PoolInfo(CAMERA, CHASE, K, res, lamps, float(get(cfg, "vehicle.lights.intensity")), asset)


def camera_intrinsics(cfg: dict):
    """Pinhole K, (width, height), focal length [mm] for HORIZONTAL_APERTURE_MM (as uw.usd_scene)."""
    w, h = (int(v) for v in get(cfg, "camera.resolution"))
    hfov = math.radians(float(get(cfg, "camera.hfov_deg")))
    focal = HORIZONTAL_APERTURE_MM / 2 / math.tan(hfov / 2)
    f_px = focal * w / HORIZONTAL_APERTURE_MM
    return np.array([[f_px, 0.0, w / 2], [0.0, f_px, h / 2], [0.0, 0.0, 1.0]]), (w, h), focal


def _tank(stage, cfg: dict, texture_dir: Path) -> None:
    L, W, H = (float(v) for v in get(cfg, "pool.size"))
    wl = float(get(cfg, "pool.water_level"))
    tile_m = float(get(cfg, "pool.tile_m"))
    texture_dir.mkdir(parents=True, exist_ok=True)
    tile_png = texture_dir / "tiles.png"
    write_png(tile_png, tile_texture(cfg))
    tiles = _surface(stage, f"{LOOKS}/tiles", None, 0.25, texture=tile_png)
    uv = tile_m * TILES_PER_TEXTURE
    UsdGeom.Xform.Define(stage, POOL)
    faces = {
        "floor": [(0, 0, 0), (L, 0, 0), (L, W, 0), (0, W, 0)],
        "wall_y0": [(0, 0, 0), (L, 0, 0), (L, 0, H), (0, 0, H)],
        "wall_yW": [(L, W, 0), (0, W, 0), (0, W, H), (L, W, H)],
        "wall_x0": [(0, W, 0), (0, 0, 0), (0, 0, H), (0, W, H)],
        "wall_xL": [(L, 0, 0), (L, W, 0), (L, W, H), (L, 0, H)],
    }
    for name, corners in faces.items():
        _bind(_quad(stage, f"{POOL}/{name}", corners, uv), tiles)

    dark = _surface(stage, f"{LOOKS}/grid", get(cfg, "pool.grid_color"), 0.4)
    step = float(get(cfg, "pool.floor_grid_m"))
    half = 0.02                                                   # 4 cm lines, 2 mm above the tiles
    for i in range(1, int(math.floor(L / step - 1e-9)) + 1):
        x = i * step
        _bind(_quad(stage, f"{POOL}/grid_x{i}", [(x - half, 0, 0.002), (x + half, 0, 0.002), (x + half, W, 0.002),
                                                 (x - half, W, 0.002)]), dark)
    for j in range(1, int(math.floor(W / step - 1e-9)) + 1):
        y = j * step
        _bind(_quad(stage, f"{POOL}/grid_y{j}", [(0, y - half, 0.002), (L, y - half, 0.002), (L, y + half, 0.002),
                                                 (0, y + half, 0.002)]), dark)
    dstep = float(get(cfg, "pool.depth_marks_m"))
    k = 1
    while k * dstep < wl - 1e-9:
        d = k * dstep
        whole = abs(d - round(d)) < 1e-9
        w_mark = 0.30 if whole else 0.15
        z = wl - d
        _bind(_quad(stage, f"{POOL}/depth_mark_{k}", [(0.5, 0.002, z - 0.02), (0.5 + w_mark, 0.002, z - 0.02),
                                                      (0.5 + w_mark, 0.002, z + 0.02), (0.5, 0.002, z + 0.02)]), dark)
        k += 1

    looks = {}                                                    # role -> material, made on first use
    for solid in pool_geometry.solids(cfg):                       # the ladder and the props: the DVL's beams see
        if solid.role not in looks:                               #   the same list (uw.pool_geometry.PoolBottom)
            looks[solid.role] = _prop_look(stage, solid.role, texture_dir)
        path = f"{POOL}/{solid.name}"
        if isinstance(solid, pool_geometry.Box):
            prim = _box(stage, path, solid.center, solid.size)
        elif isinstance(solid, pool_geometry.Cylinder):
            prim = _cylinder(stage, path, solid.center, solid.radius, solid.length, axis=solid.axis)
        else:
            c = np.asarray(solid.corners, float)
            prim = _quad(stage, path, c, float(np.linalg.norm(c[1] - c[0])))
        _bind(prim, looks[solid.role])

    if get(cfg, "water.surface"):
        water = _mdl_material(stage, f"{LOOKS}/water_surface", "OmniGlass.mdl", "OmniGlass",
                              {"glass_ior": (Sdf.ValueTypeNames.Float, WATER_IOR_FROM_BELOW),
                               "glass_color": (Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1.0, 1.0, 1.0)),
                               "thin_walled": (Sdf.ValueTypeNames.Bool, False)})
        surf = _quad(stage, f"{POOL}/water_surface", [(0, 0, wl), (L, 0, wl), (L, W, wl), (0, W, wl)])
        UsdGeom.PrimvarsAPI(surf).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        _bind(surf, water)


def _prop_look(stage, role: str, texture_dir: Path):
    """The visual material of a solid's role (uw.pool_geometry): ladder steel, a concrete block, a rusty pipe, the
    checkerboard."""
    if role == "ladder":
        return _surface(stage, f"{LOOKS}/steel", (0.62, 0.62, 0.64), 0.25, metallic=1.0)
    if role == "block":
        return _surface(stage, f"{LOOKS}/concrete", (0.35, 0.35, 0.33), 0.9)
    if role == "pipe":
        return _surface(stage, f"{LOOKS}/pipe", (0.55, 0.40, 0.05), 0.5)
    if role == "board":
        board_png = texture_dir / "checker.png"
        write_png(board_png, checker_texture())
        return _surface(stage, f"{LOOKS}/checker", None, 0.6, texture=board_png)
    return _surface(stage, f"{LOOKS}/{role}", (0.5, 0.5, 0.5), 0.5)


def _hall_lights(stage, cfg: dict) -> None:
    L, W, _ = (float(v) for v in get(cfg, "pool.size"))
    c = get(cfg, "lights.ceiling")
    nx, ny = pool_geometry.ceiling_grid(cfg)
    z = float(get(cfg, "pool.water_level")) + float(c["height_m"])
    for i in range(nx):
        for j in range(ny):
            light = UsdLux.RectLight.Define(stage, f"{WORLD}/hall/light_{i}_{j}")
            light.CreateWidthAttr(float(c["size_m"][0]))
            light.CreateHeightAttr(float(c["size_m"][1]))
            light.CreateIntensityAttr(float(c["intensity"]))
            light.CreateColorAttr(Gf.Vec3f(*map(float, c["color"])))
            UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(L * (i + 0.5) / nx, W * (j + 0.5) / ny, z))
    if float(get(cfg, "lights.dome")) > 0:
        UsdLux.DomeLight.Define(stage, f"{WORLD}/dome").CreateIntensityAttr(float(get(cfg, "lights.dome")))


def _vehicle(stage, cfg: dict):
    """The ROV under ROV: the referenced asset (or a box of the Heavy's size), the tilt xform, the Lumens."""
    _xform(stage, ROV)
    asset = Path(WS) / str(get(cfg, "vehicle.asset"))
    if asset.is_file():
        body = UsdGeom.Xform.Define(stage, f"{ROV}/body")
        body.GetPrim().GetReferences().AddReference(str(asset))
        found = str(asset)
    else:
        box = _box(stage, f"{ROV}/body", (0.0, 0.0, 0.0), (0.457, 0.575, 0.254))
        _bind(box, _surface(stage, f"{LOOKS}/rov_box", (0.02, 0.2, 0.6), 0.5))
        found = None
    _xform(stage, TILT, pos=get(cfg, "vehicle.camera.position_body"))
    lt = get(cfg, "vehicle.lights")
    UsdGeom.Xform.Define(stage, LAMPS)
    lamps = []
    for i, pos in enumerate(lt["lamps"]):
        light = UsdLux.SphereLight.Define(stage, f"{LAMPS}/lumen_{i}")
        light.CreateRadiusAttr(float(lt["radius_m"]))
        light.CreateIntensityAttr(0.0)
        light.CreateColorAttr(Gf.Vec3f(*map(float, lt["color"])))
        shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
        shaping.CreateShapingConeAngleAttr(float(lt["cone_deg"]) / 2)
        shaping.CreateShapingConeSoftnessAttr(0.3)
        p = np.asarray(pos, float)
        UsdGeom.Xformable(light).AddTransformOp().Set(_matrix(_look_at(p, p + np.array([1.0, 0.0, 0.0])), p))
        lamps.append(light.GetPath().pathString)
    return found, lamps


# ------------------------------------------------------------------------------------------------ per frame

def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def set_vehicle(stage, position, rot) -> None:
    """The vehicle body at ``position`` (CG, tank ENU) with ``rot`` (body FLU -> tank ENU)."""
    op = UsdGeom.Xformable(stage.GetPrimAtPath(ROV)).GetOrderedXformOps()[0]
    op.Set(_matrix(np.asarray(rot, float), position))


def set_tilt(stage, tilt_deg: float, cfg: dict | None = None) -> None:
    """The camera tilt, positive = looking up (ArduSub's mount pitch). A positive rotation about body +y (left)
    pitches DOWN, hence the minus sign."""
    prim = stage.GetPrimAtPath(TILT)
    op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
    pos = np.asarray(op.Get().ExtractTranslation(), float)
    op.Set(_matrix(_rot_y(-math.radians(float(tilt_deg))), pos))


def set_lights(stage, info: PoolInfo, fraction: float) -> None:
    for path in info.lamps:
        UsdLux.SphereLight(stage.GetPrimAtPath(path)).GetIntensityAttr().Set(float(fraction) * info.lamp_intensity)
