"""The pool's solids in plain numpy (M11): one description for the USD stage and for the DVL's beams.

``uw/pool_scene.py`` builds the visible stage from ``solids(cfg)``, and ``PoolBottom`` casts the DVL's beams at the
same list, so what the camera sees and what the A50 measures cannot drift apart. No pxr, no Isaac: this module
imports on the host and inside the physics process.

Frame: the pool's ENU, origin at a floor corner, floor at z = 0, walls at x = 0, L and y = 0, W, the free surface at
``pool.water_level``. Roles name what a beam hit; ``acoustics.roles`` in config/pool/pool.yaml maps each role to a
material of ``acoustics.materials`` for the echo level (``uw.rov.dvl.Dvl._echo``).

The walls are occluders here, unlike the tank's (``uw.geometry.ray_bottom``): a pool has real walls, and a tilted
beam near one hits it at a glancing angle, weakly. The water surface is not a target: the beams point down.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import get

_EPS = 1e-9


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple
    size: tuple
    role: str


@dataclass(frozen=True)
class Cylinder:
    name: str
    center: tuple
    radius: float
    length: float
    axis: str           # "X", "Y" or "Z"
    role: str


@dataclass(frozen=True)
class Quad:
    name: str
    corners: tuple      # 4 corners, counter-clockwise seen from the front; planar and a rectangle
    role: str


def solids(cfg: dict) -> list:
    """Everything in the water besides the floor and the walls: the ladder and the props, as configured."""
    L, W, _ = (float(v) for v in get(cfg, "pool.size"))
    wl = float(get(cfg, "pool.water_level"))
    out: list = []
    if get(cfg, "pool.ladder"):                                 # stainless, on the y = 0 wall, 2 m from the x = L one
        x0, y0, top = L - 2.0, 0.15, wl + 0.9
        bottom = wl - 1.5
        for side, x in (("a", x0), ("b", x0 + 0.5)):
            out.append(Cylinder(f"ladder/rail_{side}", (x, y0, (bottom + top) / 2), 0.02, top - bottom, "Z", "ladder"))
        for r in range(5):
            out.append(Cylinder(f"ladder/rung_{r}", (x0 + 0.25, y0, wl - 1.4 + 0.3 * r), 0.015, 0.5, "X", "ladder"))
    for prop in get(cfg, "pool.props") or ():                   # on the floor
        out.append(_prop(prop, L, W))
    return out


def _prop(prop: dict, L: float, W: float):
    """One entry of ``pool.props``: ``at`` is [x, y] as fractions of the pool's length and width, so the layout
    follows ``pool.size``; the sizes are metres. Kinds: box (``size``, ``z`` its centre), cylinder (``radius``,
    ``length``, ``axis``, ``z``), board (a square marker board of side ``size`` standing on the floor, facing -y)."""
    from .config import ConfigError
    try:
        name, kind, role = str(prop["name"]), str(prop["kind"]), str(prop.get("role", prop["kind"]))
        x, y = float(prop["at"][0]) * L, float(prop["at"][1]) * W
        if kind == "box":
            return Box(f"props/{name}", (x, y, float(prop["z"])), tuple(float(v) for v in prop["size"]), role)
        if kind == "cylinder":
            return Cylinder(f"props/{name}", (x, y, float(prop["z"])), float(prop["radius"]), float(prop["length"]),
                            str(prop["axis"]), role)
        if kind == "board":
            s, z0 = float(prop["size"]), float(prop.get("z", 0.05))
            return Quad(f"props/{name}", ((x - s / 2, y, z0), (x + s / 2, y, z0), (x + s / 2, y, z0 + s),
                                          (x - s / 2, y, z0 + s)), role)
    except (KeyError, TypeError, IndexError, ValueError) as exc:
        raise ConfigError(f"pool.props entry {prop!r}: {exc!r}") from None
    raise ConfigError(f"pool.props entry {prop.get('name')!r}: kind must be box | cylinder | board, got {kind!r}")


def ceiling_grid(cfg: dict) -> tuple[int, int]:
    """The hall lights along x and y (uw/pool_scene.py): one every ``lights.ceiling.spacing_m``, at least one, so a
    bigger pool keeps the same light per square metre."""
    L, W, _ = (float(v) for v in get(cfg, "pool.size"))
    sp = float(get(cfg, "lights.ceiling.spacing_m"))
    return max(1, round(L / sp)), max(1, round(W / sp))


# ----------------------------------------------------------------------------------------- ray casting

def _ray_box(o, d, lo, hi):
    """Entry distance and outward normal of a ray into an axis-aligned box (inf if missed or starting inside)."""
    t0, t1, axis, sign = -math.inf, math.inf, -1, 0.0
    for k in range(3):
        if abs(d[k]) < _EPS:
            if o[k] < lo[k] or o[k] > hi[k]:
                return math.inf, None
            continue
        a, b = (lo[k] - o[k]) / d[k], (hi[k] - o[k]) / d[k]
        s = -1.0 if d[k] > 0 else 1.0                          # the face entered first faces against the ray
        if a > b:
            a, b = b, a
        if a > t0:
            t0, axis, sign = a, k, s
        t1 = min(t1, b)
        if t0 > t1:
            return math.inf, None
    if t0 <= _EPS or axis < 0:
        return math.inf, None
    n = np.zeros(3)
    n[axis] = sign
    return t0, n


_AXES = {"X": 0, "Y": 1, "Z": 2}


def _ray_cylinder(o, d, c, radius, length, axis):
    """First hit of a ray on a closed finite cylinder along a coordinate axis: (t, normal) or (inf, None)."""
    k = _AXES[axis]
    p = [i for i in range(3) if i != k]
    oc = np.asarray(o, float) - np.asarray(c, float)
    half = length / 2.0
    best, normal = math.inf, None
    a = d[p[0]] ** 2 + d[p[1]] ** 2                              # the lateral surface
    if a > _EPS:
        b = 2.0 * (oc[p[0]] * d[p[0]] + oc[p[1]] * d[p[1]])
        cc = oc[p[0]] ** 2 + oc[p[1]] ** 2 - radius ** 2
        disc = b * b - 4.0 * a * cc
        if disc >= 0.0:
            for t in sorted(((-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a))):
                if t > _EPS and abs(oc[k] + t * d[k]) <= half:
                    q = oc + t * d
                    q[k] = 0.0
                    best, normal = t, q / radius
                    break
    if abs(d[k]) > _EPS:                                         # the end caps
        for cap in (-half, half):
            t = (cap - oc[k]) / d[k]
            if _EPS < t < best:
                q = oc + t * d
                if q[p[0]] ** 2 + q[p[1]] ** 2 <= radius ** 2:
                    best = t
                    normal = np.zeros(3)
                    normal[k] = math.copysign(1.0, cap)
    return best, normal


def _ray_quad(o, d, corners):
    """Hit of a ray on a rectangle (either side): (t, normal facing the ray) or (inf, None)."""
    c = np.asarray(corners, float)
    e1, e2 = c[1] - c[0], c[3] - c[0]
    n = np.cross(e1, e2)
    n /= np.linalg.norm(n)
    den = float(n @ d)
    if abs(den) < _EPS:
        return math.inf, None
    t = float(n @ (c[0] - o)) / den
    if t <= _EPS:
        return math.inf, None
    q = o + t * d - c[0]
    u, v = float(q @ e1) / float(e1 @ e1), float(q @ e2) / float(e2 @ e2)
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return math.inf, None
    return t, (n if den < 0 else -n)


class PoolBottom:
    """What a DVL beam hits in the pool: the floor, the walls, the ladder and the props.

    The DVL's bottom interface (``uw.rov.dvl``): ``ray_bottom(origin, directions, t_max)`` returning
    ``(t (n,), normal (n, 3), role (n,))`` like ``uw.geometry.ray_bottom``, plus ``materials`` (name -> acoustic
    properties) and ``role_material`` (role -> material name).
    """

    def __init__(self, cfg: dict):
        self.size = np.asarray(get(cfg, "pool.size"), float)
        self.water_level = float(get(cfg, "pool.water_level"))
        self.solids = solids(cfg)
        self.materials = {k: dict(v) for k, v in get(cfg, "acoustics.materials").items()}
        self.role_material = dict(get(cfg, "acoustics.roles"))
        roles = {"floor", "wall"} | {s.role for s in self.solids}
        missing = sorted(r for r in roles if self.role_material.get(r) not in self.materials)
        if missing:
            from .config import ConfigError
            raise ConfigError(f"acoustics.roles: no material of acoustics.materials for {missing}")

    def _shell(self, o, d):
        """The inside of the basin: the floor and the four walls, seen from within (inf when the ray leaves
        through the free surface, or starts outside the footprint)."""
        L, W, _ = self.size
        if not (0.0 <= o[0] <= L and 0.0 <= o[1] <= W and o[2] >= 0.0):
            return math.inf, None, None
        best, normal, role = math.inf, None, None
        for k, lim in ((0, L), (1, W)):
            if abs(d[k]) > _EPS:
                plane, sign = (lim, -1.0) if d[k] > 0 else (0.0, 1.0)
                t = (plane - o[k]) / d[k]
                if _EPS < t < best:
                    best, normal, role = t, np.eye(3)[k] * sign, "wall"
        if d[2] < -_EPS:
            t = -o[2] / d[2]
            if _EPS < t < best:
                best, normal, role = t, np.array([0.0, 0.0, 1.0]), "floor"
        if best < math.inf and o[2] + best * d[2] > self.water_level + 1e-6:
            return math.inf, None, None                        # out through the surface before any wall
        return best, normal, role

    def ray_bottom(self, origin, directions, t_max: float = 50.0):
        o = np.asarray(origin, float).reshape(3)
        dirs = np.atleast_2d(np.asarray(directions, float))
        dirs = dirs / (np.linalg.norm(dirs, axis=1, keepdims=True) + 1e-30)
        n = len(dirs)
        t_out = np.full(n, np.inf)
        n_out = np.zeros((n, 3))
        roles: list = [None] * n
        for i, d in enumerate(dirs):
            best, normal, role = self._shell(o, d)
            for s in self.solids:
                if isinstance(s, Box):
                    c, h = np.asarray(s.center, float), np.asarray(s.size, float) / 2
                    t, nn = _ray_box(o, d, c - h, c + h)
                elif isinstance(s, Cylinder):
                    t, nn = _ray_cylinder(o, d, s.center, s.radius, s.length, s.axis)
                else:
                    t, nn = _ray_quad(o, d, s.corners)
                if t < best:
                    best, normal, role = t, nn, s.role
            if best <= t_max:
                t_out[i], n_out[i], roles[i] = best, normal, role
        return t_out, n_out, roles
