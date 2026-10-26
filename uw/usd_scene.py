"""The USD stage, built from the numpy geometry (uw/geometry.py) that the sonar traces: one geometry source.

Tank walls and floor, sand height field (per-vertex albedo), rocks, pipe, flanges, supports, a hole marker; one
material per surface material; the two LED lamps (no dome unless lights.ambient_dome > 0); the sonar and camera
prims at their configured poses; the bubble PointInstancer (``BubbleInstancer``), fed from the GPU bubble pool.

Needs pxr only: tests build the stage in memory without the Kit app. ``check_geometry`` asserts that the USD meshes
and the sonar's Warp BVH mesh have identical vertices and triangles.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

from . import water as wat
from .config import get
from .geometry import icosphere, rot_rpy_deg, sonar_rotation, surface_materials

WORLD = "/World"
TANK = f"{WORLD}/tank"
LOOKS = f"{WORLD}/Looks"
RIG = f"{WORLD}/rig"                     # sonar, camera and lamps; moves rigidly with a mobile sonar (set_rig_pose)
SONAR = f"{RIG}/oculus"
CAMERA = f"{RIG}/camera"
CAMERA_OPTICAL = f"{CAMERA}/optical"
LAMPS = f"{RIG}/lamps"
BUBBLES = f"{WORLD}/plume/bubbles"
# USD camera axes (x right, y up, looking along -z) in the body frame (x forward, y left, z up): columns
R_BODY_USDCAM = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
HORIZONTAL_APERTURE_MM = 20.955          # USD default; the focal length follows from camera.hfov_deg
GAS_IOR = 0.75                           # gas in water: refractive index relative to the water (1.0 / 1.333)


@dataclass
class StageInfo:
    meshes: dict                        # scene mesh name -> prim path
    camera: str | None                  # UsdGeom.Camera path
    camera_K: np.ndarray | None         # 3x3 pinhole matrix of the rendered image
    camera_resolution: tuple | None


def _matrix(rot: np.ndarray, pos) -> Gf.Matrix4d:
    """Body-to-parent transform as a USD matrix (row vectors: the rows are the body axes, then the translation)."""
    m = np.eye(4)
    m[:3, :3] = np.asarray(rot, float).T
    m[3, :3] = np.asarray(pos, float)
    return Gf.Matrix4d(*m.flatten().tolist())


def _xform(stage, path: str, rot=np.eye(3), pos=(0.0, 0.0, 0.0), kind=UsdGeom.Xform):
    prim = kind.Define(stage, path)
    UsdGeom.Xformable(prim).AddTransformOp().Set(_matrix(rot, pos))
    return prim


def _look_at(pos, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Rotation whose -z axis points from pos to target (the axis of USD lights and cameras), x horizontal."""
    back = np.asarray(pos, float) - np.asarray(target, float)
    z = back / np.linalg.norm(back)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


def _material(stage, path: str, color, roughness: float, metallic: float, vertex_color: bool = False,
              emissive=None) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    diffuse = sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    if vertex_color:                    # albedo from the mesh's displayColor primvar
        rd = UsdShade.Shader.Define(stage, f"{path}/DisplayColor")
        rd.CreateIdAttr("UsdPrimvarReader_float3")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
        rd.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*map(float, color)))
        diffuse.ConnectToSource(rd.ConnectableAPI(), "result")
    else:
        diffuse.Set(Gf.Vec3f(*map(float, color)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, emissive)))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _mdl_material(stage, path: str, module: str, name: str, inputs: dict) -> UsdShade.Material:
    """An MDL material from the renderer's core library (e.g. OmniGlass.mdl)."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset(Sdf.AssetPath(module), "mdl")
    sh.SetSourceAssetSubIdentifier(name, "mdl")
    for k, (kind, v) in inputs.items():
        sh.CreateInput(k, kind).Set(v)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    return mat


def _bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)


def _mesh(stage, path: str, points: np.ndarray, faces: np.ndarray, colors: np.ndarray | None = None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(points, np.float32)))
    m.CreateFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(faces), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(np.ascontiguousarray(faces, np.int32).ravel()))
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr().Set(True)
    lo, hi = points.min(0), points.max(0)
    m.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
    if colors is not None:
        pv = m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(colors, np.float32)))
    return m


def camera_intrinsics(cfg: dict) -> tuple[np.ndarray, tuple[int, int], float]:
    """Pinhole K of the rendered image, (width, height), focal length [mm] for HORIZONTAL_APERTURE_MM."""
    w, h = (int(v) for v in get(cfg, "camera.resolution"))
    hfov = math.radians(float(get(cfg, "camera.hfov_deg")))
    focal = HORIZONTAL_APERTURE_MM / 2 / math.tan(hfov / 2)
    f_px = focal * w / HORIZONTAL_APERTURE_MM
    return np.array([[f_px, 0.0, w / 2], [0.0, f_px, h / 2], [0.0, 0.0, 1.0]]), (w, h), focal


def build(stage: Usd.Stage, cfg: dict, scene) -> StageInfo:
    """Populate ``stage`` (z up, metres) with the scene, lamps and sensors; returns the prim paths."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, WORLD).GetPrim())
    UsdGeom.Scope.Define(stage, LOOKS)
    UsdGeom.Xform.Define(stage, TANK)
    mats = surface_materials(cfg)
    looks = {}

    def look(name: str, vertex_color: bool = False):
        key = (name, vertex_color)
        if key not in looks:
            m = mats[name]
            looks[key] = _material(stage, f"{LOOKS}/{name.replace(':', '_')}", m["color"], m["roughness"],
                                   m["metallic"], vertex_color)
        return looks[key]

    paths = {}
    for m in scene.meshes:
        path = f"{TANK}/{m.name}"
        _bind(_mesh(stage, path, m.points, m.faces, m.colors), look(m.material, m.colors is not None))
        paths[m.name] = path
    if scene.leak is not None:          # visual marker only, not in the acoustic scene
        marker = _material(stage, f"{LOOKS}/leak_marker", (0.9, 0.1, 0.05), 0.4, 0.0, emissive=(0.6, 0.05, 0.0))
        for i, p in enumerate([scene.leak.hole, *scene.leak.vents]):
            s = UsdGeom.Sphere.Define(stage, f"{TANK}/leak_marker_{i}")
            s.CreateRadiusAttr(0.006 if i == 0 else 0.004)
            UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*map(float, p)))
            _bind(s, marker)
    _lights(stage, cfg)
    # sensors: body frames x forward, y left, z up, at their configured poses inside the rig (identity: the rig moves
    # them all with the sonar)
    _xform(stage, RIG)
    _xform(stage, SONAR, sonar_rotation(cfg), get(cfg, "sonar.position"))
    housing = UsdGeom.Cube.Define(stage, f"{SONAR}/housing")       # Oculus M750d-sized box behind the transducer
    housing.CreateSizeAttr(1.0)
    hx = UsdGeom.Xformable(housing)
    hx.AddTranslateOp().Set(Gf.Vec3d(-0.035, 0.0, 0.0))
    hx.AddScaleOp().Set(Gf.Vec3f(0.07, 0.12, 0.09))
    _bind(housing, _material(stage, f"{LOOKS}/sonar_housing", (0.05, 0.05, 0.06), 0.6, 0.2))
    info = StageInfo(meshes=paths, camera=None, camera_K=None, camera_resolution=None)
    if get(cfg, "camera.enabled"):
        _xform(stage, CAMERA, rot_rpy_deg(get(cfg, "camera.rpy_deg")), get(cfg, "camera.position"))
        cam = _xform(stage, CAMERA_OPTICAL, R_BODY_USDCAM, kind=UsdGeom.Camera)
        K, (w, h), focal = camera_intrinsics(cfg)
        cam.CreateProjectionAttr("perspective")
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE_MM)
        cam.CreateVerticalApertureAttr(HORIZONTAL_APERTURE_MM * h / w)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 30.0))
        info.camera, info.camera_K, info.camera_resolution = CAMERA_OPTICAL, K, (w, h)
    return info


def _lights(stage, cfg: dict) -> None:
    """The LED lamps on the sensor pole: sphere lights shaped to a cone of lamps[].cone_deg (full angle) towards
    their target; a dome only if lights.ambient_dome > 0 (deep water: none)."""
    lt = get(cfg, "lights")
    UsdGeom.Xform.Define(stage, LAMPS)
    for i, lamp in enumerate(lt["lamps"]):
        light = UsdLux.SphereLight.Define(stage, f"{LAMPS}/lamp_{i}")
        light.CreateRadiusAttr(float(lamp["radius"]))
        light.CreateIntensityAttr(float(lamp["intensity"]))
        light.CreateColorAttr(Gf.Vec3f(*map(float, lamp["color"])))
        shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
        shaping.CreateShapingConeAngleAttr(float(lamp["cone_deg"]) / 2)
        shaping.CreateShapingConeSoftnessAttr(0.2)
        UsdGeom.Xformable(light).AddTransformOp().Set(_matrix(_look_at(lamp["position"], lamp["target"]),
                                                              lamp["position"]))
    if float(lt["ambient_dome"]) > 0:
        UsdLux.DomeLight.Define(stage, f"{WORLD}/dome").CreateIntensityAttr(float(lt["ambient_dome"]))


def set_rig_pose(stage, cfg: dict, position, rot) -> None:
    """Move the rig (sonar, camera, lamps) so that the sonar is at ``position``/``rot``: the rig's transform is the
    rigid motion from the configured sonar pose to this one."""
    m0 = np.eye(4)
    m0[:3, :3], m0[:3, 3] = sonar_rotation(cfg), get(cfg, "sonar.position")
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, position
    rig = m @ np.linalg.inv(m0)
    op = UsdGeom.Xformable(stage.GetPrimAtPath(RIG)).GetOrderedXformOps()[0]
    op.Set(_matrix(rig[:3, :3], rig[:3, 3]))


def check_geometry(stage: Usd.Stage, scene, mesh_points: np.ndarray, mesh_faces: np.ndarray) -> int:
    """Assert that the USD meshes of the occluding surfaces, concatenated as ``scene.occluders`` does, have exactly the
    vertices and triangles of the sonar's Warp BVH mesh (``mesh_points``, ``mesh_faces``: what wp.Mesh holds); and that
    every scene mesh's USD points are its numpy points. Returns the number of vertices compared."""
    pts, faces, off = [], [], 0
    for m in scene.meshes:
        prim = UsdGeom.Mesh(stage.GetPrimAtPath(f"{TANK}/{m.name}"))
        assert prim, f"no USD mesh for {m.name}"
        p = np.asarray(prim.GetPointsAttr().Get(), np.float32)
        f = np.asarray(prim.GetFaceVertexIndicesAttr().Get(), np.int32).reshape(-1, 3)
        assert np.array_equal(p, m.points) and np.array_equal(f, m.faces), f"USD mesh {m.name} differs from the scene"
        if m.role in scene.occluder_roles:
            pts.append(p)
            faces.append(f + off)
            off += len(p)
    usd_p, usd_f = np.concatenate(pts), np.concatenate(faces)
    assert usd_p.shape == mesh_points.shape, f"vertex count: USD {usd_p.shape}, Warp {mesh_points.shape}"
    assert np.array_equal(usd_p, mesh_points), "USD and Warp mesh vertices differ"
    assert np.array_equal(usd_f.ravel(), np.asarray(mesh_faces).ravel()), "USD and Warp mesh triangles differ"
    return len(usd_p)


# ------------------------------------------------------------------------------------------------ bubbles

@wp.kernel
def _flag_kernel(alive: wp.array(dtype=wp.int32), stride: int, flag: wp.array(dtype=wp.int32)):
    i = wp.tid()
    flag[i] = 0
    if alive[i] != 0 and i % stride == 0:
        flag[i] = 1


@wp.kernel
def _ellipsoid_kernel(alive: wp.array(dtype=wp.int32), stride: int, offset: wp.array(dtype=wp.int32),
                      pos: wp.array(dtype=wp.vec3), r: wp.array(dtype=float), vel: wp.array(dtype=wp.vec3),
                      u_liq: wp.array(dtype=wp.vec3), turb: wp.array(dtype=wp.vec3), eo_k: float,
                      out_pos: wp.array(dtype=wp.vec3), out_scale: wp.array(dtype=wp.vec3),
                      out_q: wp.array(dtype=wp.vec4h)):
    i = wp.tid()
    if alive[i] == 0 or i % stride != 0:
        return
    j = offset[i]
    rad = r[i]
    # Wellek et al. (1966), contaminated water: aspect E = 1 / (1 + 0.163 Eo^0.757), Eo = g drho d^2 / sigma
    e = 1.0 / (1.0 + 0.163 * wp.pow(eo_k * 4.0 * rad * rad, 0.757))
    a = rad * wp.pow(e, -1.0 / 3.0)                    # equal volume: a^2 c = r^3
    out_pos[j] = pos[i]
    out_scale[j] = wp.vec3(a, a, a * e)
    # short axis along the bubble's velocity relative to the water (slip and path oscillation)
    s = vel[i] - u_liq[i] - turb[i]
    n = wp.length(s)
    d = wp.vec3(0.0, 0.0, 1.0)
    if n > 1.0e-6:
        d = s / n
    c = d[2]                                           # rotation of +z onto d: q = (z x d, 1 + z.d), normalised
    q = wp.vec4(-d[1], d[0], 0.0, 1.0 + c)
    if c < -0.999999:
        q = wp.vec4(1.0, 0.0, 0.0, 0.0)
    q = q / wp.length(q)
    out_q[j] = wp.vec4h(wp.float16(q[0]), wp.float16(q[1]), wp.float16(q[2]), wp.float16(q[3]))


class BubbleInstancer:
    """The bubbles as a UsdGeom.PointInstancer of unit spheres (icosphere mesh) scaled to ellipsoids, with a gas-in-water
    dielectric (OmniGlass, IOR 0.75). ``update`` compacts the alive bubbles on the GPU (prefix scan, one kernel for
    position, semi-axes and orientation); then, in the Kit app (``fabric_stage_id``), hands the GPU arrays to Fabric
    through usdrt (no host copy: the renderer reads Fabric), else copies them to the host (32 bytes per bubble) and
    sets the USD attributes."""

    def __init__(self, stage, cfg: dict, sim, stride: int = 1, fabric_stage_id: int | None = None):
        self.sim, self.stride, self.device = sim, max(1, int(stride)), sim.device
        src = sim.source
        self.eo_k = wat.G * src.rho / src.sigma            # Eo = eo_k d^2
        inst = UsdGeom.PointInstancer.Define(stage, BUBBLES)
        level = int(get(cfg, "sim.bubble_mesh_level"))
        if level < 0:                                       # the renderer's own (implicit) sphere
            proto = UsdGeom.Sphere.Define(stage, f"{BUBBLES}/proto")
            proto.CreateRadiusAttr(1.0)
        else:                                               # a unit icosphere, smooth normals (80 triangles at 1)
            v, f = icosphere(level)
            proto = _mesh(stage, f"{BUBBLES}/proto", v.astype(np.float32), f)
            proto.CreateNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
            proto.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        inst.CreatePrototypesRel().SetTargets([proto.GetPath()])
        if get(cfg, "sim.bubble_material") == "inspect":    # GUI inspection: opaque, bright, easy to see
            gas = _material(stage, f"{LOOKS}/gas", (0.75, 0.9, 1.0), 0.2, 0.0, emissive=(0.25, 0.4, 0.6))
        else:
            gas = _mdl_material(stage, f"{LOOKS}/gas", "OmniGlass.mdl", "OmniGlass",
                                {"glass_ior": (Sdf.ValueTypeNames.Float, GAS_IOR),
                                 "glass_color": (Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1.0, 1.0, 1.0)),
                                 "thin_walled": (Sdf.ValueTypeNames.Bool, False)})
        _bind(proto, gas)
        size = sim.size
        inst.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, 0.0),
                                                   Gf.Vec3f(float(size[0]), float(size[1]), float(sim.water_level))]))
        self.positions = inst.CreatePositionsAttr()
        self.scales = inst.CreateScalesAttr()
        self.orientations = inst.CreateOrientationsAttr()
        self.proto_indices = inst.CreateProtoIndicesAttr()
        cap, dev = sim.cap, self.device
        self.flag = wp.zeros(cap, dtype=wp.int32, device=dev)
        self.offset = wp.zeros(cap, dtype=wp.int32, device=dev)
        self.out_pos = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.out_scale = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.out_q = wp.zeros(cap, dtype=wp.vec4h, device=dev)
        self.n = -1
        self.fabric = None
        if fabric_stage_id is not None:
            from usdrt import Usd as RtUsd
            from usdrt import Vt as RtVt
            prim = RtUsd.Stage.Attach(fabric_stage_id).GetPrimAtPath(BUBBLES)
            self.fabric = {k: prim.GetAttribute(k) for k in ("positions", "scales", "orientations", "protoIndices")}
            self.rtvt = RtVt
            self.zero_idx = wp.zeros(cap, dtype=wp.int32, device=dev)

    def compact(self) -> int:
        """Alive bubbles -> the first n slots of the output arrays (GPU); returns n."""
        sim, dev = self.sim, self.device
        wp.launch(_flag_kernel, dim=sim.cap, device=dev, inputs=[sim.alive, self.stride, self.flag])
        wp.utils.array_scan(self.flag, self.offset, inclusive=False)
        wp.launch(_ellipsoid_kernel, dim=sim.cap, device=dev,
                  inputs=[sim.alive, self.stride, self.offset, sim.pos, sim.r, sim.vel, sim.u_liq, sim.turb,
                          self.eo_k, self.out_pos, self.out_scale, self.out_q])
        last = self.offset[sim.cap - 1:].numpy()[0] + self.flag[sim.cap - 1:].numpy()[0]
        return int(last)

    def update(self) -> dict:
        """GPU compaction, then Fabric (GPU arrays) or a host copy and USD sets; the time of each [ms]."""
        t0 = time.perf_counter()
        n = self.compact()
        t1 = time.perf_counter()
        if self.fabric is not None and n > 0:
            vt, f = self.rtvt, self.fabric
            f["positions"].Set(vt.Vec3fArray(self.out_pos[:n]))
            f["scales"].Set(vt.Vec3fArray(self.out_scale[:n]))
            f["orientations"].Set(vt.QuathArray(self.out_q[:n]))
            if n != self.n:
                f["protoIndices"].Set(vt.IntArray(self.zero_idx[:n]))
            self.n = n
            return {"bubbles": n, "gpu_compact_ms": (t1 - t0) * 1e3,
                    "fabric_set_ms": (time.perf_counter() - t1) * 1e3}
        pos = self.out_pos[:n].numpy() if n else np.zeros((0, 3), np.float32)
        scale = self.out_scale[:n].numpy() if n else np.zeros((0, 3), np.float32)
        q = self.out_q[:n].numpy() if n else np.zeros((0, 4), np.float16)
        t2 = time.perf_counter()
        with Sdf.ChangeBlock():
            self.positions.Set(Vt.Vec3fArray.FromNumpy(pos))
            self.scales.Set(Vt.Vec3fArray.FromNumpy(scale))
            self.orientations.Set(Vt.QuathArray.FromNumpy(q))       # GfQuath memory order: x, y, z, w
            if n != self.n:
                self.proto_indices.Set(Vt.IntArray.FromNumpy(np.zeros(n, np.int32)))
        t3 = time.perf_counter()
        self.n = n
        return {"bubbles": n, "gpu_compact_ms": (t1 - t0) * 1e3, "copy_to_host_ms": (t2 - t1) * 1e3,
                "usd_set_ms": (t3 - t2) * 1e3}
