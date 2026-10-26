"""Procedural tank geometry, in plain numpy: one source for the USD stage and for the sonar.

Everything here is deterministic for a given config (``seed``). The sand surface is an analytic
height field (sum of random cosines + ripples + pockmarks), so the acoustic scatterers can be
sampled on the true surface with exact normals, finer than the render/occlusion mesh.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass, field

import numpy as np

from .config import WS, ConfigError, get, sonar_mode

CACHE_DIR = os.path.join(WS, "output", "cache")
SCATTERER_CACHE_VERSION = 1         # bump when the sampling changes, so stale caches are not reused


@dataclass
class Mesh:
    name: str
    points: np.ndarray            # (N, 3) float32, world frame
    faces: np.ndarray             # (M, 3) int32
    material: str
    role: str                     # wall | sand | rock | pipe | support | flange
    colors: np.ndarray | None = None   # (N, 3) per-vertex albedo, optional


@dataclass
class Leak:
    hole: np.ndarray              # (3,) hole centre on the pipe surface
    direction: np.ndarray         # (3,) unit outward normal of the hole
    buried: bool
    vents: np.ndarray             # (K, 3) where gas leaves the sand (buried leak); (0, 3) otherwise


@dataclass
class Scene:
    size: np.ndarray              # (3,) inner tank size
    water_level: float
    meshes: list[Mesh]
    sand: "SandField"
    pipe_axis: dict               # {x, z, y0, y1, radius}
    leak: Leak | None
    occluder_roles: tuple = ("sand", "rock", "pipe", "support", "flange")
    extras: dict = field(default_factory=dict)

    def occluders(self) -> tuple[np.ndarray, np.ndarray]:
        """All occluding meshes merged (walls excluded: inside the tank they never occlude)."""
        pts, faces, off = [], [], 0
        for m in self.meshes:
            if m.role in self.occluder_roles:
                pts.append(m.points)
                faces.append(m.faces + off)
                off += len(m.points)
        return np.concatenate(pts).astype(np.float32), np.concatenate(faces).astype(np.int32)


# --------------------------------------------------------------------------------------------- sand

class SandField:
    """h(x, y) of the sand surface and its gradient, analytic."""

    def __init__(self, cfg: dict, rng: np.random.Generator):
        s = get(cfg, "sand")
        self.base = float(s["thickness"])
        # low-frequency relief: random cosines with wavelengths around undulation_scale
        n = 24
        lam = float(s["undulation_scale"]) * rng.uniform(0.4, 2.0, n)
        ang = rng.uniform(0, 2 * np.pi, n)
        self._uk = np.stack([np.cos(ang), np.sin(ang)], 1) * (2 * np.pi / lam)[:, None]
        self._uph = rng.uniform(0, 2 * np.pi, n)
        w = lam ** 1.5
        self._ua = float(s["undulation_amp"]) * w / np.sqrt(np.sum(w ** 2) / 2)
        # ripples: sinuous crests (phase warped by a slow field), asymmetric profile, varying amplitude
        self.r_amp = float(s["ripple_amp"])
        self.r_k = 2 * np.pi / float(s["ripple_wavelength"])
        hd = np.radians(float(s["ripple_heading_deg"]))
        self.r_dir = np.array([np.cos(hd), np.sin(hd)])
        m = 8
        lw = rng.uniform(0.3, 1.2, m)
        aw = rng.uniform(0, 2 * np.pi, m)
        self._wk = np.stack([np.cos(aw), np.sin(aw)], 1) * (2 * np.pi / lw)[:, None]
        self._wph = rng.uniform(0, 2 * np.pi, m)
        self._wa = rng.uniform(0.4, 1.0, m) * (1.2 / np.sqrt(m))
        self._ek = self._wk[::-1] * 0.7
        self._eph = rng.uniform(0, 2 * np.pi, m)
        self.mound: tuple | None = None        # (x0, amp, width) along-y mound at the pipe
        self.pockmarks: list[tuple] = []       # (x, y, radius, depth)

    def _terms(self, x, y):
        xy = np.stack([np.asarray(x, float), np.asarray(y, float)], -1)
        ph_u = xy @ self._uk.T + self._uph
        ph_w = xy @ self._wk.T + self._wph
        warp = np.sum(self._wa * np.sin(ph_w), -1)
        dwarp = np.einsum("...m,md->...d", self._wa * np.cos(ph_w), self._wk)
        ph_e = xy @ self._ek.T + self._eph
        env = 0.75 + 0.25 * np.tanh(np.sum(np.sin(ph_e), -1) / 2)
        denv = (0.125 / np.cosh(np.sum(np.sin(ph_e), -1) / 2) ** 2)[..., None] * np.einsum(
            "...m,md->...d", np.cos(ph_e), self._ek)
        a = self.r_k * (xy @ self.r_dir) + warp
        da = self.r_k * self.r_dir + dwarp
        return xy, ph_u, a, da, env, denv

    def height(self, x, y):
        h, _ = self.height_grad(x, y)
        return h

    def height_grad(self, x, y):
        xy, ph_u, a, da, env, denv = self._terms(x, y)
        h = self.base + np.sum(self._ua * np.cos(ph_u), -1)
        g = -np.einsum("...n,nd->...d", self._ua * np.sin(ph_u), self._uk)
        prof = np.sin(a) + 0.25 * np.sin(2 * a)                 # steeper lee side
        dprof = np.cos(a) + 0.5 * np.cos(2 * a)
        h = h + self.r_amp * env * prof
        g = g + self.r_amp * (env * dprof)[..., None] * da + self.r_amp * prof[..., None] * denv
        if self.mound is not None:
            x0, amp, w = self.mound
            d = (xy[..., 0] - x0) / w
            e = amp * np.exp(-d * d)
            h = h + e
            g[..., 0] += e * (-2 * d / w)
        for (px, py, rad, dep) in self.pockmarks:
            dx, dy = xy[..., 0] - px, xy[..., 1] - py
            r = np.sqrt(dx * dx + dy * dy) + 1e-12
            q = r / rad
            inside = q < 1.0
            bowl = np.where(inside, -dep * 0.5 * (1 + np.cos(np.pi * q)), 0.0)
            dbowl = np.where(inside, dep * 0.5 * np.pi / rad * np.sin(np.pi * q), 0.0)
            rim_q = (q - 1.2) / 0.3
            rim = 0.3 * dep * np.exp(-rim_q ** 2)
            drim = rim * (-2 * rim_q / (0.3 * rad))
            h = h + bowl + rim
            g[..., 0] += (dbowl + drim) * dx / r
            g[..., 1] += (dbowl + drim) * dy / r
        return h, g

    def normal(self, x, y):
        _, g = self.height_grad(x, y)
        n = np.concatenate([-g, np.ones(g.shape[:-1] + (1,))], -1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)


def sand_mesh(sand: SandField, size, grid: float, color, rng) -> Mesh:
    nx = int(round(size[0] / grid)) + 1
    ny = int(round(size[1] / grid)) + 1
    xs = np.linspace(0, size[0], nx)
    ys = np.linspace(0, size[1], ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = sand.height(X, Y)
    pts = np.stack([X, Y, Z], -1).reshape(-1, 3)
    faces = _grid_faces(nx, ny)
    # albedo: base colour, slow mottling, lighter crests
    mott = _value_noise2(X, Y, 0.35, rng) * 0.10 + _value_noise2(X, Y, 0.05, rng) * 0.05
    crest = np.clip((Z - sand.base) / max(sand.r_amp * 3, 1e-6), -1, 1) * 0.04
    col = np.clip(np.asarray(color)[None, None, :] * (1 + mott + crest)[..., None], 0, 1).reshape(-1, 3)
    return Mesh("sand", pts.astype(np.float32), faces, "sand", "sand", col.astype(np.float32))


def _grid_faces(nx, ny):
    i, j = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
    a = (i * ny + j).ravel()
    b = a + ny
    return np.concatenate([np.stack([a, b, a + 1], 1), np.stack([a + 1, b, b + 1], 1)]).astype(np.int32)


def _value_noise2(X, Y, scale, rng):
    k = 12
    lam = scale * rng.uniform(0.5, 2.0, k)
    ang = rng.uniform(0, 2 * np.pi, k)
    ph = rng.uniform(0, 2 * np.pi, k)
    out = np.zeros_like(X)
    for i in range(k):
        out += np.cos((np.cos(ang[i]) * X + np.sin(ang[i]) * Y) * 2 * np.pi / lam[i] + ph[i])
    return out / np.sqrt(k / 2)


# ---------------------------------------------------------------------------------------- primitives

def box_mesh(name, lo, hi, material, role) -> Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                  [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]], [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    # per-face vertices so that faces stay flat-shaded and samplers get exact normals
    pts = c[f].reshape(-1, 3)
    return Mesh(name, pts.astype(np.float32), np.arange(len(pts)).reshape(-1, 3).astype(np.int32), material, role)


def cylinder_y(name, x, z, y0, y1, r, material, role, seg=96, rings=2, caps=True) -> Mesh:
    th = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    ys = np.linspace(y0, y1, rings)
    pts = np.array([[x + r * np.sin(t), y, z + r * np.cos(t)] for y in ys for t in th])
    faces = []
    for k in range(rings - 1):
        for i in range(seg):
            a, b = k * seg + i, k * seg + (i + 1) % seg
            c, d = a + seg, b + seg
            faces += [[a, c, b], [b, c, d]]
    if caps:
        for k, y in ((0, y0), (rings - 1, y1)):
            ci = len(pts)
            pts = np.vstack([pts, [x, y, z]])
            for i in range(seg):
                a, b = k * seg + i, k * seg + (i + 1) % seg
                faces.append([ci, b, a] if k == 0 else [ci, a, b])
    return Mesh(name, pts.astype(np.float32), np.asarray(faces, np.int32), material, role)


def icosphere(level: int):
    t = (1 + 5 ** 0.5) / 2
    v = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t],
         [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]]
    f = [[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2],
         [10, 7, 6], [7, 1, 8], [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11],
         [6, 2, 10], [8, 6, 7], [9, 8, 1]]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(level):
        cache, nf = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = v[a] + v[b]
                v.append(m / np.linalg.norm(m))
                cache[key] = len(v) - 1
            return cache[key]
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]
        f = nf
    return np.array(v), np.array(f, np.int32)


def rock_mesh(name, rng, radius, flatten, material) -> Mesh:
    v, f = icosphere(4)
    # fBm-like displacement from random plane waves on the unit sphere
    disp = np.zeros(len(v))
    for octave, (k, a) in enumerate(((2.0, 0.22), (4.5, 0.10), (9.0, 0.045), (18.0, 0.02))):
        for _ in range(6):
            d = rng.normal(size=3)
            d /= np.linalg.norm(d)
            disp += a / np.sqrt(6) * np.cos(k * (v @ d) + rng.uniform(0, 2 * np.pi))
    # a couple of flat facets (fractured faces)
    for _ in range(rng.integers(1, 4)):
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        cut = rng.uniform(0.55, 0.85)
        proj = v @ d
        r_now = 1 + disp
        over = proj * r_now > cut
        disp[over] = cut / np.maximum(proj[over], 1e-3) - 1
    p = v * (radius * (1 + disp))[:, None]
    p[:, 2] *= flatten
    yaw = rng.uniform(0, 2 * np.pi)
    tilt = rng.normal(scale=0.15, size=2)
    p = p @ _rot(tilt[0], tilt[1], yaw).T
    return Mesh(name, p.astype(np.float32), f, material, "rock")


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_rpy_deg(rpy_deg) -> np.ndarray:
    """Body-to-world rotation from roll, pitch (positive = nose down, ROS), yaw in degrees."""
    r, p, y = np.radians(rpy_deg)
    return _rot(r, p, y)


# -------------------------------------------------------------------------------------------- scene

def build_scene(cfg: dict) -> Scene:
    rng = np.random.default_rng(int(cfg["seed"]))
    size = np.asarray(get(cfg, "tank.size"), float)
    wl = float(get(cfg, "tank.water_level"))
    mats = get(cfg, "materials")
    sand = SandField(cfg, rng)

    # pipe
    pc = get(cfg, "pipe")
    r = float(pc["outer_diameter"]) / 2
    crown = sand.base - float(pc["cover"])
    z_axis = crown - r
    px = float(pc["x"])
    y0, y1 = float(pc["length_margin"]), size[1] - float(pc["length_margin"])
    pipe_axis = {"x": px, "z": z_axis, "y0": y0, "y1": y1, "radius": r}
    buried_pipe = float(pc["cover"]) > 0
    if not buried_pipe:   # sand banked against an exposed pipe
        sand.mound = (px, 0.35 * min(r, crown - sand.base + r), 1.6 * r)

    # leak
    leak = None
    lc = get(cfg, "leak")
    if lc["enabled"]:
        c = np.radians(float(lc["clock_deg"]))
        d = np.array([np.sin(c), 0.0, np.cos(c)])
        hole = np.array([px, float(lc["position_along"]), z_axis]) + r * d
        buried = bool(hole[2] < sand.height(hole[0], hole[1]) - 0.005)
        vents = np.zeros((0, 3))
        if buried:
            cover_here = sand.height(hole[0], hole[1]) - hole[2]
            k = int(lc["vents"])
            off = rng.normal(scale=float(lc["vent_spread"]) * cover_here, size=(k, 2))
            off[0] *= 0.3                                  # one main vent nearly above the hole
            vxy = hole[:2] + off
            for vx, vy in vxy:
                sand.pockmarks.append((vx, vy, float(lc["pockmark_radius"]) * rng.uniform(0.7, 1.3),
                                       float(lc["pockmark_depth"]) * rng.uniform(0.6, 1.2)))
            vents = np.array([[vx, vy, sand.height(vx, vy)] for vx, vy in vxy])
        leak = Leak(hole=hole, direction=d, buried=buried, vents=vents)

    meshes: list[Mesh] = []
    t = float(get(cfg, "tank.wall_thickness"))
    wm = get(cfg, "tank.wall_material")
    L, W, H = size
    meshes += [
        box_mesh("wall_xm", [-t, -t, -t], [0, W + t, H], wm, "wall"),
        box_mesh("wall_xp", [L, -t, -t], [L + t, W + t, H], wm, "wall"),
        box_mesh("wall_ym", [0, -t, -t], [L, 0, H], wm, "wall"),
        box_mesh("wall_yp", [0, W, -t], [L, W + t, H], wm, "wall"),
        box_mesh("floor", [-t, -t, -t], [L + t, W + t, 0], wm, "wall"),
    ]
    meshes.append(sand_mesh(sand, size, float(get(cfg, "sand.grid")), mats["sand"]["color"], rng))
    meshes.append(cylinder_y("pipe", px, z_axis, y0, y1, r, pc["material"], "pipe", seg=128, rings=30))
    for i, yf in enumerate(pc.get("flanges_at", [])):
        meshes.append(cylinder_y(f"flange_{i}", px, z_axis, yf - 0.018, yf + 0.018, r + 0.055, pc["material"],
                                 "flange", seg=128))
    if pc["supports"]:   # saddle blocks, only where the pipe bottom clears the sand
        for i, ys in enumerate((0.45, size[1] - 0.45)):
            top = z_axis - 0.6 * r
            base = sand.height(px, ys) - 0.08
            if z_axis - r > sand.height(px, ys) + 0.03:
                meshes.append(box_mesh(f"support_{i}", [px - 0.17, ys - 0.12, base], [px + 0.17, ys + 0.12, top],
                                       "concrete", "support"))

    # rocks, kept away from the walls, the pipe and the leak
    rc = get(cfg, "rocks")
    placed = []
    tries = 0
    leak_xy = leak.hole[:2] if leak is not None else None
    while len(placed) < int(rc["count"]) and tries < 2000:
        tries += 1
        rad = float(np.exp(rng.uniform(np.log(rc["radius_range"][0]), np.log(rc["radius_range"][1]))))
        x, y = rng.uniform(rad + 0.05, L - rad - 0.05), rng.uniform(rad + 0.05, W - rad - 0.05)
        if abs(x - px) < float(rc["keep_out"]) + rad:
            continue
        if leak_xy is not None and np.hypot(x - leak_xy[0], y - leak_xy[1]) < float(rc["keep_out"]) * 2 + rad:
            continue
        if any(np.hypot(x - qx, y - qy) < (rad + qr) * 1.1 for qx, qy, qr in placed):
            continue
        placed.append((x, y, rad))
        m = rock_mesh(f"rock_{len(placed) - 1:02d}", rng, rad, rng.uniform(*rc["flatten"]), rc["material"])
        zmin, zmax = m.points[:, 2].min(), m.points[:, 2].max()
        emb = rng.uniform(*rc["embed"])
        m.points[:, 0] += x
        m.points[:, 1] += y
        m.points[:, 2] += sand.height(x, y) - zmin - emb * (zmax - zmin)
        meshes.append(m)

    return Scene(size=size, water_level=wl, meshes=meshes, sand=sand, pipe_axis=pipe_axis, leak=leak)


# ------------------------------------------------------------------------------- acoustic scatterers

@dataclass
class Scatterers:
    pos: np.ndarray       # (N, 3) float32
    normal: np.ndarray    # (N, 3) float32
    area: np.ndarray      # (N,)   float32, surface area each point stands for [m^2]
    mat: np.ndarray       # (N,)   int32 index into material_names
    phase: np.ndarray     # (N,)   float32 random phase [rad], fixed in the world (frozen speckle)
    material_names: list[str]


WALL_SPACING_FACTOR = 2.0          # walls are sampled at least this much coarser (they are mostly specular)
SCATTERERS_PER_CELL = 9.0          # adaptive spacing: about 3 x 3 scatterers per sonar resolution cell
PATCH = 0.1                        # [m] sand patches and mesh triangles share one spacing at this scale


@dataclass
class Spacing:
    """Scatterer spacing rule. Fixed: ``base`` everywhere. Adaptive: at slant range R from the sonar,
    max(base, sqrt(R dtheta dr / per_cell)), i.e. about ``per_cell`` scatterers per resolution cell (R dtheta wide,
    dr deep) once that is coarser than ``base``. Walls never finer than ``wall_min``."""
    base: float
    wall_min: float
    adaptive: bool = False
    sonar: tuple | None = None      # acoustic centre [m]
    dtheta: float = 0.0             # horizontal beam width [rad]
    dr: float = 0.0                 # range resolution [m]
    per_cell: float = SCATTERERS_PER_CELL

    def at(self, p: np.ndarray, wall: bool = False) -> np.ndarray:
        s = np.full(len(p), self.base)
        if self.adaptive:
            r = np.linalg.norm(np.asarray(p, float) - np.asarray(self.sonar, float), axis=1)
            s = np.maximum(s, np.sqrt(r * self.dtheta * self.dr / self.per_cell))
        return np.maximum(s, self.wall_min) if wall else s


def spacing_from_config(cfg: dict) -> Spacing:
    """``sonar.scatterer_spacing`` (base) and ``sonar.scatterer_spacing_mode`` (fixed | adaptive, from the profile);
    the adaptive rule uses the sonar position and the active mode's beam width and range resolution."""
    base = float(get(cfg, "sonar.scatterer_spacing"))
    mode = get(cfg, "sonar.scatterer_spacing_mode")
    if mode == "fixed":
        return Spacing(base=base, wall_min=WALL_SPACING_FACTOR * base)
    if mode != "adaptive":
        raise ConfigError(f"sonar.scatterer_spacing_mode must be fixed or adaptive, got {mode!r}")
    son = sonar_mode(cfg)
    return Spacing(base=base, wall_min=WALL_SPACING_FACTOR * base, adaptive=True,
                   sonar=tuple(float(v) for v in son["position"]), dtheta=float(np.radians(son["beamwidth_h_deg"])),
                   dr=float(son["range_resolution"]))


def sample_scatterers(scene: Scene, spacing: Spacing, rng: np.random.Generator) -> Scatterers:
    """Points on every surface the sound can reach, with exact normals and the area each one stands for.

    Sand: jittered grid on the analytic height field, one spacing per ~10 cm patch. Meshes: triangles subdivided
    to <= 10 cm, random points with density 1/spacing^2 and area weight spacing^2 (total areas kept, unbiased).
    Walls: only their inner faces between the sand and the water surface.
    """
    names: list[str] = []
    out_p, out_n, out_a, out_m = [], [], [], []

    def mat_index(name):
        if name not in names:
            names.append(name)
        return names.index(name)

    L, W, _ = scene.size
    x, y, cell = _sand_grid(scene, spacing, rng)
    chunks = np.array_split(np.arange(len(x)), max(1, len(x) // 400000))
    for idx in chunks:
        h, g = scene.sand.height_grad(x[idx], y[idx])
        n = np.stack([-g[:, 0], -g[:, 1], np.ones(len(idx))], 1)
        dA = cell[idx] * np.linalg.norm(n, axis=1)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        out_p.append(np.stack([x[idx], y[idx], h], 1))
        out_n.append(n)
        out_a.append(dA)
        out_m.append(np.full(len(idx), mat_index("sand")))

    for m in scene.meshes:
        if m.role == "sand":
            continue
        wall = m.role == "wall"
        tri = m.points.astype(float)[m.faces]
        if wall:   # only the inner faces: vertical, inside the tank, normal pointing into it
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            cen = tri.mean(1)
            c = np.array([L / 2, W / 2])
            inside = ((cen[:, 0] >= -1e-6) & (cen[:, 0] <= L + 1e-6) & (cen[:, 1] >= -1e-6)
                      & (cen[:, 1] <= W + 1e-6))
            facing = np.einsum("ij,ij->i", nrm[:, :2], c[None, :] - cen[:, :2]) > 0
            tri = tri[inside & facing & (np.abs(nrm[:, 2]) < 0.5 * np.linalg.norm(nrm, axis=1))]
            if len(tri) == 0:       # the floor slab: all under the sand
                continue
        p, n, a = _sample_triangles(_subdivide(tri, PATCH), spacing, wall, rng)
        if wall:
            keep = (p[:, 2] < scene.water_level) & (p[:, 2] > scene.sand.height(
                np.clip(p[:, 0], 0, L), np.clip(p[:, 1], 0, W)) - 0.01)
            p, n, a = p[keep], n[keep], a[keep]
        else:   # drop the parts buried in the sand (they are behind the sand surface anyway)
            keep = p[:, 2] > scene.sand.height(np.clip(p[:, 0], 0, L), np.clip(p[:, 1], 0, W)) - 0.005
            p, n, a = p[keep], n[keep], a[keep]
            if m.role in ("pipe", "flange"):   # exact radial normals instead of the facet normals
                side = np.abs(n[:, 1]) < 0.5
                rad = p[side] - np.array([scene.pipe_axis["x"], 0.0, scene.pipe_axis["z"]])
                rad[:, 1] = 0.0
                n[side] = rad / np.linalg.norm(rad, axis=1, keepdims=True)
        out_p.append(p)
        out_n.append(n)
        out_a.append(a)
        out_m.append(np.full(len(p), mat_index(m.material)))

    pos = np.concatenate(out_p).astype(np.float32)
    return Scatterers(pos=pos, normal=np.concatenate(out_n).astype(np.float32),
                      area=np.concatenate(out_a).astype(np.float32),
                      mat=np.concatenate(out_m).astype(np.int32),
                      phase=rng.uniform(0, 2 * np.pi, len(pos)).astype(np.float32), material_names=names)


def _sand_grid(scene: Scene, spacing: Spacing, rng: np.random.Generator):
    """x, y and cell area of a jittered grid over the tank floor; each ~PATCH patch has its own spacing."""
    L, W, _ = scene.size
    nx, ny = max(1, round(L / PATCH)), max(1, round(W / PATCH))
    px, py = L / nx, W / ny
    cx, cy = np.meshgrid((np.arange(nx) + 0.5) * px, (np.arange(ny) + 0.5) * py, indexing="ij")
    cx, cy = cx.ravel(), cy.ravel()
    s = spacing.at(np.stack([cx, cy, scene.sand.height(cx, cy)], 1))
    mx = np.maximum(1, np.round(px / s)).astype(int)
    my = np.maximum(1, np.round(py / s)).astype(int)
    xs, ys, cell = [], [], []
    for i in range(len(cx)):
        ix, iy = np.meshgrid(np.arange(mx[i]), np.arange(my[i]), indexing="ij")
        n = mx[i] * my[i]
        xs.append(cx[i] - px / 2 + (ix.ravel() + rng.uniform(size=n)) * (px / mx[i]))
        ys.append(cy[i] - py / 2 + (iy.ravel() + rng.uniform(size=n)) * (py / my[i]))
        cell.append(np.full(n, px * py / n))
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(cell)


def _subdivide(tri: np.ndarray, max_edge: float) -> np.ndarray:
    """Split triangles (M, 3, 3) at the midpoint of their longest edge until no edge exceeds ``max_edge``."""
    done = []
    while len(tri):
        e = np.linalg.norm(tri[:, [1, 2, 0]] - tri, axis=2)     # edges 0-1, 1-2, 2-0
        k = np.argmax(e, 1)
        big = e[np.arange(len(tri)), k] > max_edge
        done.append(tri[~big])
        t, k = tri[big], k[big]
        t = np.take_along_axis(t, np.stack([k, (k + 1) % 3, (k + 2) % 3], 1)[:, :, None], 1)  # longest edge 0-1
        mid = 0.5 * (t[:, 0] + t[:, 1])
        tri = np.concatenate([np.stack([t[:, 0], mid, t[:, 2]], 1), np.stack([mid, t[:, 1], t[:, 2]], 1)])
    return np.concatenate(done)


def _sample_triangles(tri: np.ndarray, spacing: Spacing, wall: bool, rng: np.random.Generator):
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    nrm = cr / np.maximum(2 * area, 1e-12)[:, None]
    s = spacing.at(tri.mean(1), wall)
    # area / s^2 points per triangle, rounded up or down at random so that the expected count is exact; each point
    # stands for s^2, so the total area is kept without bias (and with less noise than drawing triangles at random)
    count = np.floor(area / (s * s) + rng.uniform(size=len(tri))).astype(int)
    k = np.repeat(np.arange(len(tri)), count)
    n = len(k)
    u, v = rng.uniform(size=n), rng.uniform(size=n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    p = a[k] + u[:, None] * (b[k] - a[k]) + v[:, None] * (c[k] - a[k])
    return p, nrm[k], s[k] ** 2


def _cache_key(cfg: dict, spacing: Spacing) -> str:
    leak = get(cfg, "leak")
    inputs = {"version": SCATTERER_CACHE_VERSION, "seed": cfg["seed"],
              **{k: get(cfg, k) for k in ("tank", "sand", "rocks", "pipe")},
              "leak": {k: leak[k] for k in ("enabled", "position_along", "clock_deg", "vents", "vent_spread",
                                            "pockmark_radius", "pockmark_depth")},
              "spacing": spacing.__dict__}
    return hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()[:16]


def scatterers(cfg: dict, scene: Scene | None = None, cache_dir: str = CACHE_DIR) -> Scatterers:
    """The scatterers for ``cfg``: read from ``cache_dir`` when the same geometry and spacing were sampled before,
    else sampled and cached there (npz keyed by a hash of the geometry and spacing inputs)."""
    spacing = spacing_from_config(cfg)
    path = os.path.join(cache_dir, f"scatterers_{_cache_key(cfg, spacing)}.npz")
    if os.path.isfile(path):
        with np.load(path) as z:
            return Scatterers(pos=z["pos"], normal=z["normal"], area=z["area"], mat=z["mat"], phase=z["phase"],
                              material_names=[str(s) for s in z["material_names"]])
    scene = scene if scene is not None else build_scene(cfg)
    sc = sample_scatterers(scene, spacing, np.random.default_rng([int(cfg["seed"]), 2]))
    os.makedirs(cache_dir, exist_ok=True)
    tmp = f"{path[:-4]}.{os.getpid()}.tmp.npz"
    np.savez(tmp, pos=sc.pos, normal=sc.normal, area=sc.area, mat=sc.mat, phase=sc.phase,
             material_names=np.array(sc.material_names))
    os.replace(tmp, path)
    return sc
