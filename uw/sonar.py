"""Forward-looking multibeam sonar (Oculus M750d, LF/HF) in Warp, standalone (no Isaac).

Static scene, once per sonar pose (StaticCache precomputed): every scatterer of geometry.scatterers() is evaluated
over the direct path and the image-source paths of the planar boundaries (water surface, 4 walls): for each boundary
TX via the mirror + RX direct, TX direct + RX via the mirror, and both via the mirror. Every leg is checked with a
shadow ray against the occluders (sand, rocks, pipe; Warp BVH), the reflection point must lie on the boundary.
Scattering: bistatic Lambert mu cos_i cos_s + Kirchhoff (geometric optics, Gaussian slopes) specular term, random
phase per scatterer. Smooth (coherent) materials add their mirror-like share exp(-k^2 h^2 (cos_i + cos_s)^2 / 2) as a
Kirchhoff integral with deterministic phase over small triangles (exact for planar facets), so glints are coherent
mirror echoes; their incoherent part keeps the rest of the specular energy. The complex echo goes into a
(range bin x u) grid, u = direction cosine along the receive array (cone effect included);
paths that cross the plume box are kept in a list and attenuated by the bubbles every ping.

Per ping: static grid (+ plume-box paths x extinction) x slow phase walk ("speckle dynamics") + bubbles (direct and
surface paths, sigma_bs(ka) from scattering.py, extinction through a voxel grid) + volume reverberation. An FFT over
u gives the aperture at half-wavelength spacing; receiver noise and ring-down are added per element; beams are formed
with a Taylor taper, with the Fresnel term x^2 (1 - u^2) / (2 R) left in when the array does not focus.

Signal: two-way spherical spreading and absorption, vertical TX/RX aperture patterns (Hann taper by default; cosine
or uniform sinc), horizontal fan roll-off, coherent
sum with an energy-preserving pulse envelope over the two nearest range bins. Output: power [uPa^2] per (range bin,
beam), raw dB re 1 uPa, TVG (k log10 R + 2 alpha R), 8-bit window above the beam noise floor, blind zone below the
minimum range, and the bubbles' share of the energy as two plume labels: all bubble paths (direct, surface ghosts) and
the direct path only.
Implemented (linus): StaticCache precomputed, SensorPose fixed, Beamformer fft_fresnel; the others are M6.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
import torch
import warp as wp

from . import scattering
from . import water as wat
from .config import ConfigError, get, sonar_mode
from .geometry import sonar_rotation, surface_materials

PI = wp.constant(math.pi)
FAR = 1.0e6


@wp.struct
class Params:
    pos: wp.vec3            # acoustic centre [m]
    rot_t: wp.mat33         # world -> sonar frame (x forward, y port, z up)
    k: float                # wavenumber [1/m]
    att: float              # amplitude absorption [Np/m]
    sl: float               # source pressure at 1 m [uPa]
    dr: float               # range bin [m]
    n_r: int
    n_u: int
    du: float
    vert: float             # one-way vertical pattern vertical(vert * sin(elevation), vtaper)
    vtaper: int             # vertical aperture: 0 uniform, 1 cosine, 2 Hann
    fan: float              # one-way horizontal pattern cos(azimuth)^fan
    eps: float              # shadow-ray tolerance [m]
    use_mesh: int
    mesh: wp.uint64


@wp.struct
class Planes:
    n: wp.array(dtype=wp.vec3)          # unit normal; plane: dot(n, x) = d
    d: wp.array(dtype=float)
    lo: wp.array(dtype=wp.vec3)         # valid reflection points: lo <= q <= hi
    hi: wp.array(dtype=wp.vec3)
    refl: wp.array(dtype=float)         # reflection coefficient at normal incidence, smooth
    rough: wp.array(dtype=float)        # rms height [m] (Rayleigh loss of the coherent reflection)
    count: int
    surface: int                        # index of the water surface, -1 if not used
    boil_c: wp.vec3                     # where the plume breaks the surface
    boil_r: float
    boil_rough: float


@wp.struct
class Box:                              # plume region: extinction grid (lo..hi) and the slices the plume fills
    lo: wp.vec3
    hi: wp.vec3
    vox: float
    on: int
    slice_lo: wp.array(dtype=wp.vec3)   # height slices that follow the plume's width: a path that crosses one
    slice_hi: wp.array(dtype=wp.vec3)   # is attenuated every ping
    weak: float                         # ... unless its amplitude is below this [uPa]: then it joins the static image


# ------------------------------------------------------------------------------------------- helpers

@wp.func
def vertical(x: float, kind: int) -> float:
    """One-way pattern of a line aperture, 1 on axis, x = pi L sin(angle) / lambda: uniform sin(x)/x; cosine taper
    pi^2 cos(x) / (pi^2 - 4 x^2); Hann taper (sin(x)/x) pi^2 / (pi^2 - x^2). Signed (sidelobes alternate)."""
    s = float(1.0)
    if wp.abs(x) > 1.0e-6:
        s = wp.sin(x) / x
    v = s
    if kind == 1:
        d = PI * PI - 4.0 * x * x
        v = PI / 4.0                                    # the limit at x = pi/2
        if wp.abs(d) > 1.0e-4:
            v = PI * PI * wp.cos(x) / d
    elif kind == 2:
        d = PI * PI - x * x
        v = 0.5                                         # the limit at x = pi
        if wp.abs(d) > 1.0e-4:
            v = s * PI * PI / d
    return v


@wp.func
def pattern(p: Params, d: wp.vec3) -> float:
    """One-way gain toward the world direction d (unit, from the sonar): vertical aperture x fan roll-off, 0 behind."""
    b = p.rot_t * d
    g = float(0.0)
    if b[0] > 0.0:
        g = vertical(p.vert * b[2], p.vtaper) * wp.pow(b[0] / wp.sqrt(b[0] * b[0] + b[1] * b[1]), p.fan)
    return g


@wp.func
def u_of(p: Params, d: wp.vec3) -> float:
    """Direction cosine along the receive array, positive to starboard."""
    b = p.rot_t * d
    return -b[1]


@wp.func
def deposit(grid: wp.array2d(dtype=wp.vec2), p: Params, path: float, u: float, amp: float, phase: float):
    """Complex echo into the two nearest range bins (cos/sin envelope: energy kept) and u cells (linear)."""
    x = 0.5 * path / p.dr
    y = u / p.du + 0.5 * float(p.n_u)
    j = int(wp.floor(x))
    i = int(wp.floor(y))
    if j >= 0 and j + 1 < p.n_r and i >= 0 and i + 1 < p.n_u:
        f = x - float(j)
        g = y - float(i)
        z = wp.vec2(wp.cos(phase), wp.sin(phase)) * amp
        a0 = z * wp.cos(0.5 * PI * f)
        a1 = z * wp.sin(0.5 * PI * f)
        wp.atomic_add(grid, j, i, a0 * (1.0 - g))
        wp.atomic_add(grid, j, i + 1, a0 * g)
        wp.atomic_add(grid, j + 1, i, a1 * (1.0 - g))
        wp.atomic_add(grid, j + 1, i + 1, a1 * g)


@wp.func
def clear(p: Params, a: wp.vec3, b: wp.vec3) -> int:
    """1 if nothing occludes the segment a -> b (up to eps before b)."""
    ok = int(1)
    if p.use_mesh != 0:
        d = b - a
        length = wp.length(d)
        if length > p.eps:
            q = wp.mesh_query_ray(p.mesh, a, d / length, length - p.eps)
            if q.result:
                ok = 0
    return ok


@wp.func
def mirror(n: wp.vec3, dd: float, s: wp.vec3) -> wp.vec3:
    return s - 2.0 * (wp.dot(n, s) - dd) * n


@wp.func
def plane_hit(n: wp.vec3, dd: float, a: wp.vec3, b: wp.vec3) -> wp.vec3:
    da = wp.dot(n, a) - dd
    db = wp.dot(n, b) - dd
    return a + (da / (da - db)) * (b - a)


@wp.func
def inside(q: wp.vec3, lo: wp.vec3, hi: wp.vec3) -> int:
    ok = int(0)
    if q[0] >= lo[0] and q[0] <= hi[0] and q[1] >= lo[1] and q[1] <= hi[1] and q[2] >= lo[2] and q[2] <= hi[2]:
        ok = 1
    return ok


@wp.func
def reflection(pl: Planes, k_: int, q: wp.vec3, cos_i: float, k: float) -> float:
    s = pl.rough[k_]
    if k_ == pl.surface and pl.boil_r > 0.0:
        dx = q[0] - pl.boil_c[0]
        dy = q[1] - pl.boil_c[1]
        if dx * dx + dy * dy < pl.boil_r * pl.boil_r:
            s = pl.boil_rough
    x = k * s * cos_i
    return pl.refl[k_] * wp.exp(-2.0 * x * x)


@wp.func
def clip(a: wp.vec3, b: wp.vec3, lo: wp.vec3, hi: wp.vec3) -> wp.vec2:
    """Parameter interval [t0, t1] of the segment a -> b inside the box (t0 > t1: misses)."""
    d = b - a
    t0 = float(0.0)
    t1 = float(1.0)
    for ax in range(3):
        if wp.abs(d[ax]) < 1.0e-12:
            if a[ax] < lo[ax] or a[ax] > hi[ax]:
                t0 = 2.0
        else:
            ta = (lo[ax] - a[ax]) / d[ax]
            tb = (hi[ax] - a[ax]) / d[ax]
            t0 = wp.max(t0, wp.min(ta, tb))
            t1 = wp.min(t1, wp.max(ta, tb))
    return wp.vec2(t0, t1)


@wp.func
def crosses(a: wp.vec3, b: wp.vec3, bx: Box) -> int:
    hit = int(0)
    if bx.on != 0:
        for k_ in range(bx.slice_lo.shape[0]):
            t = clip(a, b, bx.slice_lo[k_], bx.slice_hi[k_])
            if t[0] <= t[1]:
                hit = 1
    return hit


@wp.func
def optical_depth(ext: wp.array3d(dtype=float), bx: Box, a: wp.vec3, b: wp.vec3) -> float:
    """Integral of the extinction coefficient [1/m] along a -> b (intensity optical depth), sampled every half voxel
    inside the plume's height slices (the bubbles are there; the slices do not overlap)."""
    s = float(0.0)
    if bx.on != 0:
        t = clip(a, b, bx.lo, bx.hi)
        if t[0] < t[1]:
            d = b - a
            full = wp.length(d)
            nx = ext.shape[0]
            ny = ext.shape[1]
            nz = ext.shape[2]
            for k_ in range(bx.slice_lo.shape[0]):
                ts = clip(a, b, bx.slice_lo[k_], bx.slice_hi[k_])
                if ts[0] < ts[1]:
                    length = full * (ts[1] - ts[0])
                    n = wp.max(int(wp.ceil(length / (0.5 * bx.vox))), 1)
                    acc = float(0.0)
                    for j in range(n):
                        q = a + (ts[0] + (ts[1] - ts[0]) * (float(j) + 0.5) / float(n)) * d
                        ix = wp.clamp(int((q[0] - bx.lo[0]) / bx.vox), 0, nx - 1)
                        iy = wp.clamp(int((q[1] - bx.lo[1]) / bx.vox), 0, ny - 1)
                        iz = wp.clamp(int((q[2] - bx.lo[2]) / bx.vox), 0, nz - 1)
                        acc += ext[ix, iy, iz]
                    s += acc * length / float(n)
    return s


# ------------------------------------------------------------------------------------- static scene

@wp.func
def route(p: Params, bx: Box, x: wp.vec3, qt: wp.vec3, qr: wp.vec3, amp: float, phase: float, path: float, u: float,
          store: int, grid: wp.array2d(dtype=wp.vec2), dyn_x: wp.array(dtype=wp.vec3), dyn_qt: wp.array(dtype=wp.vec3),
          dyn_qr: wp.array(dtype=wp.vec3), dyn_v: wp.array(dtype=wp.vec4), dyn_count: wp.array(dtype=wp.int32)):
    """A static path's echo: into the static grid, or (when it crosses the plume region and is not too weak) into
    the list attenuated every ping. Pass 1 (store 0) only counts the list."""
    dyn = int(0)
    if wp.abs(amp) >= bx.weak:
        dyn = crosses(p.pos, qt, bx) + crosses(qt, x, bx) + crosses(x, qr, bx) + crosses(qr, p.pos, bx)
    if dyn > 0:
        idx = wp.atomic_add(dyn_count, 0, 1)
        if store != 0 and idx < dyn_x.shape[0]:
            dyn_x[idx] = x
            dyn_qt[idx] = qt
            dyn_qr[idx] = qr
            dyn_v[idx] = wp.vec4(amp, phase, path, u)
    elif store != 0:
        deposit(grid, p, path, u, amp, phase)


@wp.func
def emit(p: Params, bx: Box, x: wp.vec3, n: wp.vec3, mu: float, s2: float, r0: float, coh: int, sh2: float,
         area: float, ph: float, dt: wp.vec3, drx: wp.vec3, lt: float, lr: float, gt: float, gr: float, refl: float,
         u: float, qt: wp.vec3, qr: wp.vec3, store: int, grid: wp.array2d(dtype=wp.vec2),
         dyn_x: wp.array(dtype=wp.vec3), dyn_qt: wp.array(dtype=wp.vec3), dyn_qr: wp.array(dtype=wp.vec3),
         dyn_v: wp.array(dtype=wp.vec4), dyn_count: wp.array(dtype=wp.int32)):
    """One path of one surface scatterer (random phase). dt, drx: unit vectors from the scatterer toward where the
    sound comes from and goes to; lt, lr: lengths of the TX and RX legs; qt, qr: reflection points (x if direct).
    On a coherent material the specular term keeps only its incoherent share 1 - exp(-k^2 h^2 (ct + cr)^2)."""
    ct = wp.dot(n, dt)
    cr = wp.dot(n, drx)
    if ct > 0.0 and cr > 0.0 and gt * gr != 0.0:               # (sinc sidelobes flip the sign)
        sig = mu * ct * cr                                         # bistatic Lambert
        h = wp.normalize(dt + drx)
        cb = wp.dot(n, h)
        if cb > 0.0:                                               # Kirchhoff, Gaussian slopes (rms s per axis)
            t2 = (1.0 - cb * cb) / (cb * cb)
            spec = r0 * r0 * wp.exp(-t2 / (2.0 * s2)) / (8.0 * PI * s2 * cb * cb * cb * cb)
            if coh != 0:
                spec = spec * (1.0 - wp.exp(-sh2 * (ct + cr) * (ct + cr)))
            sig += spec
        amp = p.sl * wp.sqrt(sig * area) / (lt * lr) * wp.exp(-p.att * (lt + lr)) * gt * gr * wp.abs(refl)   # signed
        phase = p.k * (lt + lr) + ph
        if refl < 0.0:
            phase += PI
        route(p, bx, x, qt, qr, amp, phase, lt + lr, u, store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)


@wp.func
def dd1(x: float, y: float) -> wp.vec2:
    """(e^{iy} - e^{ix}) / (y - x) as (re, im), stable for y -> x."""
    h = 0.5 * (y - x)
    s = float(1.0)
    if wp.abs(h) > 1.0e-6:
        s = wp.sin(h) / h
    m = 0.5 * (x + y)
    return wp.vec2(-wp.sin(m) * s, wp.cos(m) * s)


@wp.func
def tri_integral(t0: float, t1: float, t2: float, area: float) -> wp.vec2:
    """Integral of exp(i g.(x - c)) over a planar triangle from the vertex phases t_j = g.(v_j - c): 2 area x minus
    the second divided difference of exp(i t) (Hermite-Genocchi; scattering.triangle_phase_integral)."""
    a = wp.min(t0, wp.min(t1, t2))
    c = wp.max(t0, wp.max(t1, t2))
    b = t0 + t1 + t2 - a - c
    out = wp.vec2(0.0, 0.0)
    if c - a < 1.0e-3:
        m = (a + b + c) / 3.0
        out = wp.vec2(wp.cos(m), wp.sin(m)) * area
    else:
        out = (dd1(a, b) - dd1(b, c)) * (2.0 * area / (c - a))
    return out


@wp.func
def emit_facet(p: Params, bx: Box, c: wp.vec3, n: wp.vec3, v0: wp.vec3, v1: wp.vec3, v2: wp.vec3, area: float,
               r0: float, sh2: float, dt: wp.vec3, drx: wp.vec3, lt: float, lr: float, gt: float, gr: float,
               refl: float, u: float, qt: wp.vec3, qr: wp.vec3, store: int, grid: wp.array2d(dtype=wp.vec2),
               dyn_x: wp.array(dtype=wp.vec3), dyn_qt: wp.array(dtype=wp.vec3), dyn_qr: wp.array(dtype=wp.vec3),
               dyn_v: wp.array(dtype=wp.vec4), dyn_count: wp.array(dtype=wp.int32)):
    """One path of one coherent facet: Kirchhoff (physical optics) field -(i k R0 / 4 pi) int n.(s_i + s_s)
    exp(i k (L_t + L_r)) dA, the phase linear across the facet, times the Rayleigh factor of its roughness."""
    ct = wp.dot(n, dt)
    cr = wp.dot(n, drx)
    if ct > 0.0 and cr > 0.0 and gt * gr != 0.0:
        g = (dt + drx) * (-p.k)                                    # gradient of k (L_t + L_r) on the facet
        ii = tri_integral(wp.dot(g, v0 - c), wp.dot(g, v1 - c), wp.dot(g, v2 - c), area)
        chi = wp.exp(-0.5 * sh2 * (ct + cr) * (ct + cr))
        amp = (p.sl / (lt * lr) * wp.exp(-p.att * (lt + lr)) * gt * gr * wp.abs(refl) * r0 * p.k / (4.0 * PI)
               * (ct + cr) * chi * wp.length(ii))
        phase = p.k * (lt + lr) + wp.atan2(ii[1], ii[0]) - 0.5 * PI
        if refl < 0.0:
            phase += PI
        route(p, bx, c, qt, qr, amp, phase, lt + lr, u, store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)


@wp.kernel
def static_kernel(p: Params, pl: Planes, bx: Box, pts: wp.array(dtype=wp.vec3), nrm: wp.array(dtype=wp.vec3),
                  area: wp.array(dtype=float), mat: wp.array(dtype=wp.int32), ph0: wp.array(dtype=float),
                  mu: wp.array(dtype=float), s2: wp.array(dtype=float), r0: wp.array(dtype=float),
                  coh: wp.array(dtype=wp.int32), sh2: wp.array(dtype=float), store: int,
                  grid: wp.array2d(dtype=wp.vec2), dyn_x: wp.array(dtype=wp.vec3), dyn_qt: wp.array(dtype=wp.vec3),
                  dyn_qr: wp.array(dtype=wp.vec3), dyn_v: wp.array(dtype=wp.vec4), dyn_count: wp.array(dtype=wp.int32)):
    i = wp.tid()
    x = pts[i]
    n = nrm[i]
    m = mat[i]
    s = p.pos
    v = x - s
    l1 = wp.length(v)
    d1 = v / l1
    g1 = pattern(p, d1)
    vis1 = clear(p, s, x)
    u1 = u_of(p, d1)
    if vis1 != 0:
        emit(p, bx, x, n, mu[m], s2[m], r0[m], coh[m], sh2[m], area[i], ph0[i], -d1, -d1, l1, l1, g1, g1, 1.0, u1, x, x,
             store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
    for k_ in range(pl.count):
        nk = pl.n[k_]
        dk = pl.d[k_]
        q = plane_hit(nk, dk, mirror(nk, dk, s), x)
        if inside(q, pl.lo[k_], pl.hi[k_]) != 0:
            if clear(p, s, q) * clear(p, q, x) != 0:
                dq = wp.normalize(q - s)
                gq = pattern(p, dq)
                lm = wp.length(q - s) + wp.length(x - q)
                xq = wp.normalize(q - x)
                refl = reflection(pl, k_, q, wp.abs(wp.dot(dq, nk)), p.k)
                uq = u_of(p, dq)
                if vis1 != 0:
                    emit(p, bx, x, n, mu[m], s2[m], r0[m], coh[m], sh2[m], area[i], ph0[i], xq, -d1, lm, l1, gq, g1, refl, u1, q, x,
                         store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
                    emit(p, bx, x, n, mu[m], s2[m], r0[m], coh[m], sh2[m], area[i], ph0[i], -d1, xq, l1, lm, g1, gq, refl, uq, x, q,
                         store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
                emit(p, bx, x, n, mu[m], s2[m], r0[m], coh[m], sh2[m], area[i], ph0[i], xq, xq, lm, lm, gq, gq, refl * refl, uq, q, q,
                     store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)


@wp.kernel
def facet_kernel(p: Params, pl: Planes, bx: Box, fv0: wp.array(dtype=wp.vec3), fv1: wp.array(dtype=wp.vec3),
                 fv2: wp.array(dtype=wp.vec3), fmat: wp.array(dtype=wp.int32), r0: wp.array(dtype=float),
                 sh2: wp.array(dtype=float), store: int, grid: wp.array2d(dtype=wp.vec2),
                 dyn_x: wp.array(dtype=wp.vec3), dyn_qt: wp.array(dtype=wp.vec3), dyn_qr: wp.array(dtype=wp.vec3),
                 dyn_v: wp.array(dtype=wp.vec4), dyn_count: wp.array(dtype=wp.int32)):
    """Coherent facets over the same paths as the scatterers (direct, image paths of every boundary)."""
    i = wp.tid()
    v0 = fv0[i]
    v1 = fv1[i]
    v2 = fv2[i]
    x = (v0 + v1 + v2) / 3.0
    cr_ = wp.cross(v1 - v0, v2 - v0)
    area = 0.5 * wp.length(cr_)
    if area > 0.0:
        n = cr_ / (2.0 * area)
        m = fmat[i]
        s = p.pos
        v = x - s
        l1 = wp.length(v)
        d1 = v / l1
        g1 = pattern(p, d1)
        vis1 = clear(p, s, x)
        u1 = u_of(p, d1)
        if vis1 != 0:
            emit_facet(p, bx, x, n, v0, v1, v2, area, r0[m], sh2[m], -d1, -d1, l1, l1, g1, g1, 1.0, u1, x, x,
                       store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
        for k_ in range(pl.count):
            nk = pl.n[k_]
            dk = pl.d[k_]
            q = plane_hit(nk, dk, mirror(nk, dk, s), x)
            if inside(q, pl.lo[k_], pl.hi[k_]) != 0:
                if clear(p, s, q) * clear(p, q, x) != 0:
                    dq = wp.normalize(q - s)
                    gq = pattern(p, dq)
                    lm = wp.length(q - s) + wp.length(x - q)
                    xq = wp.normalize(q - x)
                    refl = reflection(pl, k_, q, wp.abs(wp.dot(dq, nk)), p.k)
                    uq = u_of(p, dq)
                    if vis1 != 0:
                        emit_facet(p, bx, x, n, v0, v1, v2, area, r0[m], sh2[m], xq, -d1, lm, l1, gq, g1, refl, u1,
                                   q, x, store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
                        emit_facet(p, bx, x, n, v0, v1, v2, area, r0[m], sh2[m], -d1, xq, l1, lm, g1, gq, refl, uq,
                                   x, q, store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)
                    emit_facet(p, bx, x, n, v0, v1, v2, area, r0[m], sh2[m], xq, xq, lm, lm, gq, gq, refl * refl, uq,
                               q, q, store, grid, dyn_x, dyn_qt, dyn_qr, dyn_v, dyn_count)


@wp.kernel
def dynamic_kernel(p: Params, bx: Box, ext: wp.array3d(dtype=float), dyn_x: wp.array(dtype=wp.vec3),
                   dyn_qt: wp.array(dtype=wp.vec3), dyn_qr: wp.array(dtype=wp.vec3), dyn_v: wp.array(dtype=wp.vec4),
                   grid: wp.array2d(dtype=wp.vec2)):
    i = wp.tid()
    x = dyn_x[i]
    qt = dyn_qt[i]
    qr = dyn_qr[i]
    v = dyn_v[i]
    tau = (optical_depth(ext, bx, p.pos, qt) + optical_depth(ext, bx, qt, x) + optical_depth(ext, bx, x, qr)
           + optical_depth(ext, bx, qr, p.pos))
    deposit(grid, p, v[2], v[3], v[0] * wp.exp(-0.5 * tau), v[1])


# ------------------------------------------------------------------------------------ bubbles, targets

@wp.kernel
def bubble_sigma_kernel(r: wp.array(dtype=float), alive: wp.array(dtype=wp.int32), k: float,
                        tab_bs: wp.array(dtype=float), tab_ext: wp.array(dtype=float), log0: float, dlog: float,
                        sig: wp.array(dtype=float), sext: wp.array(dtype=float)):
    i = wp.tid()
    s = float(0.0)
    e = float(0.0)
    if alive[i] != 0:
        a = r[i]
        x = wp.clamp((wp.log10(k * a) - log0) / dlog, 0.0, float(tab_bs.shape[0] - 1) - 1.0e-3)
        j = int(x)
        f = x - float(j)
        s = ((1.0 - f) * tab_bs[j] + f * tab_bs[j + 1]) * a * a
        e = ((1.0 - f) * tab_ext[j] + f * tab_ext[j + 1]) * PI * a * a
    sig[i] = s
    sext[i] = e


@wp.kernel
def extinction_kernel(pos: wp.array(dtype=wp.vec3), sext: wp.array(dtype=float), bx: Box, inv_vol: float,
                      ext: wp.array3d(dtype=float)):
    i = wp.tid()
    if sext[i] > 0.0:
        q = pos[i]
        ix = int(wp.floor((q[0] - bx.lo[0]) / bx.vox))
        iy = int(wp.floor((q[1] - bx.lo[1]) / bx.vox))
        iz = int(wp.floor((q[2] - bx.lo[2]) / bx.vox))
        if ix >= 0 and iy >= 0 and iz >= 0 and ix < ext.shape[0] and iy < ext.shape[1] and iz < ext.shape[2]:
            wp.atomic_add(ext, ix, iy, iz, sext[i] * inv_vol)


@wp.kernel
def point_kernel(p: Params, pl: Planes, n_planes: int, bx: Box, ext: wp.array3d(dtype=float),
                 pos: wp.array(dtype=wp.vec3), sig: wp.array(dtype=float), grid: wp.array2d(dtype=wp.vec2),
                 grid2: wp.array2d(dtype=wp.vec2), grid3: wp.array2d(dtype=wp.vec2), to2: int):
    """Point scatterers with backscattering cross-section sig (bubbles, test targets): direct path and the image
    paths of the first n_planes boundaries; extinction along every leg; phase of a soft sphere (pi). With to2 the
    echoes also go into grid2 (all paths) and grid3 (direct path only): the plume labels."""
    i = wp.tid()
    sg = sig[i]
    if sg > 0.0:
        x = pos[i]
        s = p.pos
        v = x - s
        l1 = wp.length(v)
        d1 = v / l1
        g1 = pattern(p, d1)
        base = p.sl * wp.sqrt(sg)
        t1 = optical_depth(ext, bx, s, x)
        if clear(p, s, x) != 0 and g1 != 0.0:
            amp = base / (l1 * l1) * wp.exp(-2.0 * p.att * l1 - t1) * g1 * g1
            deposit(grid, p, 2.0 * l1, u_of(p, d1), amp, 2.0 * p.k * l1 + PI)
            if to2 != 0:
                deposit(grid2, p, 2.0 * l1, u_of(p, d1), amp, 2.0 * p.k * l1 + PI)
                deposit(grid3, p, 2.0 * l1, u_of(p, d1), amp, 2.0 * p.k * l1 + PI)
            for k_ in range(n_planes):
                nk = pl.n[k_]
                dk = pl.d[k_]
                q = plane_hit(nk, dk, mirror(nk, dk, s), x)
                if inside(q, pl.lo[k_], pl.hi[k_]) != 0 and clear(p, s, q) * clear(p, q, x) != 0:
                    dq = wp.normalize(q - s)
                    gq = pattern(p, dq)
                    lm = wp.length(q - s) + wp.length(x - q)
                    tm = optical_depth(ext, bx, s, q) + optical_depth(ext, bx, q, x)
                    refl = reflection(pl, k_, q, wp.abs(wp.dot(dq, nk)), p.k)
                    ph = float(0.0)
                    if refl < 0.0:
                        ph = PI
                    a1 = base / (lm * l1) * wp.exp(-p.att * (lm + l1) - 0.5 * (tm + t1)) * gq * g1 * wp.abs(refl)
                    a2 = base / (lm * lm) * wp.exp(-2.0 * p.att * lm - tm) * gq * gq * refl * refl
                    deposit(grid, p, lm + l1, u_of(p, d1), a1, p.k * (lm + l1) + PI + ph)
                    deposit(grid, p, lm + l1, u_of(p, dq), a1, p.k * (lm + l1) + PI + ph)
                    deposit(grid, p, 2.0 * lm, u_of(p, dq), a2, 2.0 * p.k * lm + PI)
                    if to2 != 0:
                        deposit(grid2, p, lm + l1, u_of(p, d1), a1, p.k * (lm + l1) + PI + ph)
                        deposit(grid2, p, lm + l1, u_of(p, dq), a1, p.k * (lm + l1) + PI + ph)
                        deposit(grid2, p, 2.0 * lm, u_of(p, dq), a2, 2.0 * p.k * lm + PI)


# ------------------------------------------------------------------------------ per-ping grid effects

@wp.kernel
def speckle_kernel(grid: wp.array2d(dtype=wp.vec2), phi: wp.array2d(dtype=float), a: float, q: float, seed: int):
    r, c = wp.tid()
    st = wp.rand_init(seed, r * grid.shape[1] + c)
    f = a * phi[r, c] + q * wp.randn(st)
    phi[r, c] = f
    z = grid[r, c]
    cs = wp.cos(f)
    sn = wp.sin(f)
    grid[r, c] = wp.vec2(z[0] * cs - z[1] * sn, z[0] * sn + z[1] * cs)


@wp.kernel
def reverb_kernel(grid: wp.array2d(dtype=wp.vec2), rev: wp.array2d(dtype=float), seed: int):
    r, c = wp.tid()
    s = rev[r, c]
    if s > 0.0:
        st = wp.rand_init(seed, r * grid.shape[1] + c)
        grid[r, c] = grid[r, c] + s * wp.vec2(wp.randn(st), wp.randn(st))


@wp.kernel
def beamform_kernel(ap: wp.array2d(dtype=wp.vec2), xs: wp.array(dtype=float), w: wp.array(dtype=float),
                    ub: wp.array(dtype=float), k: float, dr: float, focus: int, power: wp.array2d(dtype=float)):
    """Beam b at range bin r: sum_m w_m a_m exp(i k x_m u_b) [x exp(i k x_m^2 (1 - u_b^2) / (2 R)) unfocused]."""
    r, b = wp.tid()
    u = ub[b]
    c = float(0.0)
    if focus == 0:
        c = k * (1.0 - u * u) / (2.0 * wp.max(float(r) * dr, dr))
    re = float(0.0)
    im = float(0.0)
    for m in range(xs.shape[0]):
        x = xs[m]
        ph = k * x * u + c * x * x
        cs = wp.cos(ph)
        sn = wp.sin(ph)
        a = ap[r, m]
        re += w[m] * (a[0] * cs - a[1] * sn)
        im += w[m] * (a[0] * sn + a[1] * cs)
    power[r, b] = re * re + im * im


@wp.kernel
def display_kernel(power: wp.array2d(dtype=float), power_b: wp.array2d(dtype=float),
                   power_d: wp.array2d(dtype=float), dr: float, min_bin: int, min_range: float, tvg_k: float,
                   alpha_db: float, nf_db: float, lo: float, hi: float, thr: float, db: wp.array2d(dtype=float),
                   img: wp.array2d(dtype=wp.uint8), share: wp.array2d(dtype=float), mask: wp.array2d(dtype=wp.uint8),
                   share_d: wp.array2d(dtype=float), mask_d: wp.array2d(dtype=wp.uint8)):
    r, b = wp.tid()
    if r < min_bin:                             # blind zone: no data
        power[r, b] = 0.0
        db[r, b] = wp.nan
        img[r, b] = wp.uint8(0)
        share[r, b] = 0.0
        mask[r, b] = wp.uint8(0)
        share_d[r, b] = 0.0
        mask_d[r, b] = wp.uint8(0)
    else:
        pw = power[r, b]
        rr = wp.max(float(r) * dr, min_range)
        level = 10.0 * wp.log10(pw + 1.0e-30)
        db[r, b] = level
        v = level + tvg_k * wp.log10(rr) + 2.0 * alpha_db * rr - nf_db
        img[r, b] = wp.uint8(wp.clamp((v - lo) / (hi - lo) * 255.0, 0.0, 255.0))
        sh = power_b[r, b] / wp.max(pw, 1.0e-30)
        share[r, b] = sh
        mask[r, b] = wp.uint8(0)
        if sh > thr:
            mask[r, b] = wp.uint8(255)
        sd = power_d[r, b] / wp.max(pw, 1.0e-30)
        share_d[r, b] = sd
        mask_d[r, b] = wp.uint8(0)
        if sd > thr:
            mask_d[r, b] = wp.uint8(255)


@wp.kernel
def fan_kernel(polar: wp.array2d(dtype=wp.uint8), dr: float, b0: float, db: float, sine: int, theta_max: float,
               rmax: float, scale: float, out: wp.array2d(dtype=wp.uint8)):
    """Cartesian fan: sonar at the bottom centre, starboard to the right; bilinear in (range bin, beam)."""
    row, col = wp.tid()
    h = out.shape[0]
    w = out.shape[1]
    x = (float(col) - 0.5 * float(w - 1)) / scale
    y = float(h - 1 - row) / scale
    rr = wp.sqrt(x * x + y * y)
    th = wp.atan2(x, y)
    val = float(0.0)
    if rr <= rmax and wp.abs(th) <= theta_max:
        fb = (th - b0) / db
        if sine != 0:
            fb = (wp.sin(th) - b0) / db
        fr = rr / dr
        j = wp.clamp(int(fr), 0, polar.shape[0] - 2)
        i = wp.clamp(int(fb), 0, polar.shape[1] - 2)
        f = wp.clamp(fr - float(j), 0.0, 1.0)
        g = wp.clamp(fb - float(i), 0.0, 1.0)
        val = ((1.0 - f) * ((1.0 - g) * float(polar[j, i]) + g * float(polar[j, i + 1]))
               + f * ((1.0 - g) * float(polar[j + 1, i]) + g * float(polar[j + 1, i + 1])))
    out[row, col] = wp.uint8(val + 0.5)


# ------------------------------------------------------------------------------------------ the sonar

@dataclass
class Ping:
    power: np.ndarray           # (range bins, beams) [uPa^2], 0 in the blind zone
    db: np.ndarray              # raw level dB re 1 uPa, NaN in the blind zone
    image: np.ndarray           # uint8: TVG + display window above the beam noise floor
    plume_share_all: np.ndarray     # bubbles' share of the echo energy, all their paths (direct, surface ghosts)
    plume_mask_all: np.ndarray      # uint8 0/255: share > sonar.plume_mask_threshold
    plume_share_direct: np.ndarray  # ... their direct path only
    plume_mask_direct: np.ndarray
    range_m: np.ndarray
    bearing_deg: np.ndarray
    timing_ms: dict = field(default_factory=dict)


def plume_region(sim, n_slices: int = 12) -> dict:
    """Where the bubbles can be: around the integral plume's centreline, 2.5 widths b plus twice the meander
    amplitude (the void fraction there is < 1e-4 of the centreline's), from the source to the surface; as one box
    (extinction grid) and as height slices that follow the plume's growing width. Plus the surface boil."""
    pl = sim.flow.plume
    m = (2.5 + 2.0 * sim.flow.meander_amp) * pl["b"][:, None]
    z0, z1 = pl["pos"][0, 2] - 0.05, sim.water_level + 0.02
    edges = np.linspace(z0, z1, n_slices + 1)
    slices = []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = (pl["pos"][:, 2] >= a - 0.05) & (pl["pos"][:, 2] <= b + 0.05)
        if not sel.any():
            sel = np.argsort(np.abs(pl["pos"][:, 2] - 0.5 * (a + b)))[:2]
        lo, hi = (pl["pos"][sel] - m[sel]).min(0), (pl["pos"][sel] + m[sel]).max(0)
        slices.append((np.r_[np.maximum(lo[:2], 0.0), a], np.r_[np.minimum(hi[:2], sim.size[:2]), b]))
    lo = np.min([s[0] for s in slices], 0)
    hi = np.max([s[1] for s in slices], 0)
    return {"lo": lo, "hi": hi, "slices": slices, "boil_centre": pl["pos"][-1].copy(), "boil_radius": float(pl["b"][-1])}


TAPERS = {"uniform": 0, "cosine": 1, "hann": 2}
FIRST_NULL = {"uniform": math.pi, "cosine": 1.5 * math.pi, "hann": 2 * math.pi}


def vertical_pattern(x, taper: str):
    """Host version of ``vertical`` (numpy)."""
    x = np.asarray(x, float) + 1e-7                    # the singular points are removable: step past them
    s = np.sin(x) / x
    if taper == "cosine":
        return np.pi ** 2 * np.cos(x) / (np.pi ** 2 - 4 * x * x)
    if taper == "hann":
        return s * np.pi ** 2 / (np.pi ** 2 - x * x)
    return s


def _vertical_root(level: float, taper: str) -> float:
    """x in the main lobe where the one-way vertical pattern equals level (0 < level < 1), by bisection."""
    a, b = 1e-9, FIRST_NULL[taper]
    for _ in range(80):
        c = 0.5 * (a + b)
        a, b = (c, b) if vertical_pattern(c, taper) > level else (a, c)
    return 0.5 * (a + b)


class Sonar:
    def __init__(self, cfg: dict, scene=None, scatterers=None, plume: dict | None = None, device: str = "cuda:0"):
        son, md = get(cfg, "sonar"), sonar_mode(cfg)
        for key, ok in (("beamformer", "fft_fresnel"), ("static_cache", "precomputed"), ("pose_source", "fixed")):
            if son[key] != ok:
                raise NotImplementedError(f"sonar.{key} {son[key]!r} is milestone M6; implemented: {ok!r}")
        self.device = device
        self.timing_ms: dict = {}
        t0 = time.perf_counter()
        wc = get(cfg, "water")
        t, s = float(wc["temperature_c"]), float(wc["salinity_psu"])
        self.c = wat.sound_speed(t, s, 1.0)
        self.rho = rho = wat.density(t, s)
        f = float(md["frequency_hz"])
        lam = self.c / f
        self.k = 2 * math.pi / lam
        self.alpha_db = wat.absorption_db_per_m(f, t, s)
        self.dr = float(md["range_resolution"])
        self.range = float(son["range"])
        self.n_r = int(round(self.range / self.dr)) + 1
        self.n_u = int(son["u_grid"])
        self.min_range = float(md["min_range"])
        self.focus = bool(son["focus"])
        self.tvg_k = float(son["tvg_db_per_decade"])
        self.display = [float(v) for v in son["display_db"]]
        self.mask_threshold = float(son["plume_mask_threshold"])
        self.position = np.asarray(son["position"], float)
        self.rot = sonar_rotation(cfg)
        # beams
        self.theta_max = math.radians(float(md["fov_h_deg"]) / 2)
        n_b = int(md["beams"])
        self.sine = son["beam_spacing"] == "uniform_sine"
        if self.sine:
            ub = np.linspace(-math.sin(self.theta_max), math.sin(self.theta_max), n_b)
            self.bearing = np.degrees(np.arcsin(ub))
        elif son["beam_spacing"] == "uniform_angle":
            self.bearing = np.linspace(-math.degrees(self.theta_max), math.degrees(self.theta_max), n_b)
            ub = np.sin(np.radians(self.bearing))
        else:
            raise ConfigError(f"sonar.beam_spacing must be uniform_angle or uniform_sine, got {son['beam_spacing']!r}")
        self.range_m = np.arange(self.n_r) * self.dr
        # receive aperture: half-wavelength samples (the FFT's), length for the beam width under the Taylor taper
        self.xs, self.w = self._aperture(lam, math.radians(float(md["beamwidth_h_deg"])), float(son["array_taper_sll_db"]),
                                         int(son["taylor_nbar"]))
        # patterns: two-way vertical gain at +-fov_v/2 and horizontal gain at the fan edges
        self.vtaper = son["vertical_taper"]
        if self.vtaper not in TAPERS:
            raise ConfigError(f"sonar.vertical_taper must be one of {', '.join(TAPERS)}, got {self.vtaper!r}")
        vert = (_vertical_root(10 ** (float(son["vertical_edge_db"]) / 40), self.vtaper)
                / math.sin(math.radians(float(md["fov_v_deg"]) / 2)))
        fan = float(son["fan_edge_db"]) / (40 * math.log10(math.cos(self.theta_max)))
        # receiver noise per element over the band c / (2 dr); the beams' noise floor after the array gain
        band = self.c / (2 * self.dr)
        self.noise_var = 10 ** ((float(son["noise_level_db"]) + 10 * math.log10(band)) / 10)
        self.nf_db = 10 * math.log10(self.noise_var * float(np.sum(self.w ** 2)))
        self.receiver_noise = bool(son["receiver_noise"])
        p = Params()
        p.pos = wp.vec3(*self.position)
        p.rot_t = wp.mat33(*self.rot.T.flatten())
        p.k = self.k
        p.att = self.alpha_db / (20 * math.log10(math.e))
        p.sl = 10 ** (float(son["source_level_db"]) / 20)
        p.dr = self.dr
        p.n_r = self.n_r
        p.n_u = self.n_u
        p.du = 2.0 / self.n_u
        p.vert = vert
        p.vtaper = TAPERS[self.vtaper]
        p.fan = fan
        p.eps = 0.02
        p.use_mesh = 0
        p.mesh = wp.uint64(0)
        self.vert, self.fan_n = vert, fan
        self.mesh = None
        if scene is not None:
            pts, faces = scene.occluders()
            self.mesh = wp.Mesh(points=wp.array(pts, dtype=wp.vec3, device=device),
                                indices=wp.array(faces.flatten(), dtype=wp.int32, device=device))
            p.use_mesh = 1
            p.mesh = self.mesh.id
        self.p = p
        self.planes, self.n_point_planes = self._planes(cfg, scene, plume, rho)
        # bubble tables (gas at the mean depth; the high-ka behaviour hardly depends on it)
        wl = float(get(cfg, "tank.water_level"))
        g, h = scattering.gas_ratios(rho, self.c, wat.P_ATM + rho * wat.G * wl / 2, t + 273.15, get(cfg, "leak.gas"))
        self.tables = scattering.Tables.build(g, h)
        self.tab_bs = wp.array(self.tables.backscatter, dtype=float, device=device)
        self.tab_ext = wp.array(self.tables.extinction, dtype=float, device=device)
        # plume box: extinction grid and the static paths crossing it
        self.box = Box()
        self.box.vox = float(son["extinction_voxel"])
        self.box.on = 0
        self.box.lo = wp.vec3(0.0, 0.0, 0.0)
        self.box.hi = wp.vec3(0.0, 0.0, 0.0)
        floor = son["plume_path_floor_db"]                 # weak plume-box paths: static, without extinction
        self.box.weak = 0.0 if floor is None else 10 ** ((self.nf_db + float(floor)) / 20)
        slices = plume["slices"] if plume is not None else []
        self.box.slice_lo = wp.array([s[0] for s in slices] or np.zeros((0, 3)), dtype=wp.vec3, device=device)
        self.box.slice_hi = wp.array([s[1] for s in slices] or np.zeros((0, 3)), dtype=wp.vec3, device=device)
        dims = (1, 1, 1)
        if plume is not None:
            self.box.on = 1
            self.box.lo = wp.vec3(*plume["lo"])
            self.box.hi = wp.vec3(*plume["hi"])
            dims = tuple(int(math.ceil(v / self.box.vox)) for v in np.asarray(plume["hi"]) - np.asarray(plume["lo"]))
        self.ext = wp.zeros(dims, dtype=float, device=device)
        # grids
        shape = (self.n_r, self.n_u)
        self.static_grid = wp.zeros(shape, dtype=wp.vec2, device=device)
        self.grid = wp.zeros(shape, dtype=wp.vec2, device=device)
        self.grid_b = wp.zeros(shape, dtype=wp.vec2, device=device)
        self.grid_d = wp.zeros(shape, dtype=wp.vec2, device=device)
        self.phi = wp.zeros(shape, dtype=float, device=device)
        a = math.exp(-1.0 / (float(son["ping_rate_hz"]) * float(son["speckle_time_s"])))
        self.speckle = (a, float(son["speckle_sigma_rad"]) * math.sqrt(1 - a * a))
        self.n_dyn = 0
        self.n_scatterers = self.n_facets = 0
        self.dyn = None
        self.timing_ms["setup"] = (time.perf_counter() - t0) * 1e3
        if scatterers is not None:
            t1 = time.perf_counter()
            self._precompute(cfg, scatterers)
            self.timing_ms["static_precompute"] = (time.perf_counter() - t1) * 1e3
        t1 = time.perf_counter()
        self.rev = wp.array(self._reverb(cfg, scene) if son["reverberation"] else np.zeros(shape, np.float32),
                            dtype=float, device=device)
        self.ringdown = self._ringdown(son) if son["ringdown"] else None
        self.timing_ms["reverb_ringdown_tables"] = (time.perf_counter() - t1) * 1e3
        self.xs_w = wp.array(self.xs.astype(np.float32), dtype=float, device=device)
        self.w_w = wp.array(self.w.astype(np.float32), dtype=float, device=device)
        self.ub = wp.array(ub.astype(np.float32), dtype=float, device=device)
        self.power = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.power_b = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.power_d = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.share_d = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.mask_d = wp.zeros((self.n_r, n_b), dtype=wp.uint8, device=device)
        self.db = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.img = wp.zeros((self.n_r, n_b), dtype=wp.uint8, device=device)
        self.share = wp.zeros((self.n_r, n_b), dtype=float, device=device)
        self.mask = wp.zeros((self.n_r, n_b), dtype=wp.uint8, device=device)
        self.seq = 0
        self.seed = int(cfg["seed"]) * 7919

    # -------------------------------------------------------------------------------------- setup
    def _aperture(self, lam, beamwidth, sll_db, nbar):
        """Element positions (half-wavelength, centred) and normalised Taylor weights; the element count is the one
        whose -3 dB beam width at broadside is closest to ``beamwidth``."""
        from scipy.signal.windows import taylor

        def width(n):
            w = taylor(n, nbar=nbar, sll=-sll_db, norm=False)
            x = (np.arange(n) - (n - 1) / 2) * lam / 2
            u = np.linspace(0, 4 * lam / (n * lam / 2), 4001)
            b = np.abs(np.exp(1j * 2 * np.pi / lam * np.outer(u, x)) @ w) ** 2
            return 2 * math.asin(u[np.argmax(b < b[0] / 2)])
        n0 = int(round(101 * width(101) / beamwidth)) | 1           # the width scales as 1 / n: start close
        n = min(range(max(n0 - 6, 9), n0 + 7, 2), key=lambda n: abs(width(n) - beamwidth))
        w = taylor(n, nbar=nbar, sll=-sll_db, norm=False)
        self.beamwidth_deg = math.degrees(width(n))
        return (np.arange(n) - (n - 1) / 2) * lam / 2, w / w.sum()

    def _planes(self, cfg, scene, plume, rho):
        L, W, _ = (float(v) for v in get(cfg, "tank.size"))
        wl = float(get(cfg, "tank.water_level"))
        z_sand = float(scene.sand.base) if scene is not None else 0.0
        mats = surface_materials(cfg)
        son = get(cfg, "sonar")
        rows, surface = [], -1
        if "surface" in son["multipath"]:
            surface = 0
            rows.append(((0, 0, 1), wl, (0, 0, -FAR), (L, W, FAR), -1.0, float(son["surface_roughness_m"])))
        if "walls" in son["multipath"]:
            for wid, n, d in (("x0", (1, 0, 0), 0.0), ("x1", (1, 0, 0), L), ("y0", (0, 1, 0), 0.0),
                              ("y1", (0, 1, 0), W)):
                lo = [-FAR if n[0] else 0.0, -FAR if n[1] else 0.0, z_sand]
                hi = [FAR if n[0] else L, FAR if n[1] else W, wl]
                m = mats[f"wall:{wid}"]
                rows.append((n, d, lo, hi, self._reflection(m), float(m.get("rms_height_m", 0.0))))
        pl = Planes()
        dev = self.device
        pl.n = wp.array([r[0] for r in rows] or np.zeros((0, 3)), dtype=wp.vec3, device=dev)
        pl.d = wp.array([r[1] for r in rows], dtype=float, device=dev)
        pl.lo = wp.array([r[2] for r in rows] or np.zeros((0, 3)), dtype=wp.vec3, device=dev)
        pl.hi = wp.array([r[3] for r in rows] or np.zeros((0, 3)), dtype=wp.vec3, device=dev)
        pl.refl = wp.array([r[4] for r in rows], dtype=float, device=dev)
        pl.rough = wp.array([r[5] for r in rows], dtype=float, device=dev)
        pl.count = len(rows)
        pl.surface = surface
        pl.boil_c = wp.vec3(*(plume["boil_centre"] if plume else (0.0, 0.0, 0.0)))
        pl.boil_r = float(plume["boil_radius"]) if plume else 0.0
        pl.boil_rough = float(son["boil_roughness_m"])
        return pl, (1 if surface == 0 else 0)          # points (bubbles, targets) use the surface paths only

    def _reflection(self, m: dict) -> float:
        """Reflection coefficient at normal incidence: the material's `reflection`, else from its impedance."""
        if m.get("reflection") is not None:
            return float(m["reflection"])
        z_w = self.rho * self.c / 1e6
        return (float(m["Z"]) - z_w) / (float(m["Z"]) + z_w)

    def _precompute(self, cfg, sc):
        mats = surface_materials(cfg)
        ms = [mats[n] for n in sc.material_names]
        dev = self.device
        mu = wp.array(np.array([10 ** (m["lambert_db"] / 10) for m in ms], np.float32), dtype=float, device=dev)
        s2 = wp.array(np.array([m["rms_slope"] ** 2 for m in ms], np.float32), dtype=float, device=dev)
        r0 = wp.array(np.array([self._reflection(m) for m in ms], np.float32), dtype=float, device=dev)
        coh = wp.array(np.array([1 if m.get("coherent") else 0 for m in ms], np.int32), dtype=wp.int32, device=dev)
        sh2 = wp.array(np.array([(self.k * m.get("rms_height_m", 0.0)) ** 2 for m in ms], np.float32), dtype=float,
                       device=dev)
        points = [wp.array(sc.pos, dtype=wp.vec3, device=dev), wp.array(sc.normal, dtype=wp.vec3, device=dev),
                  wp.array(sc.area, dtype=float, device=dev), wp.array(sc.mat, dtype=wp.int32, device=dev),
                  wp.array(sc.phase, dtype=float, device=dev), mu, s2, r0, coh, sh2]
        facets = sc.facets if sc.facets is not None else np.zeros((0, 3, 3), np.float32)
        tri = [wp.array(np.ascontiguousarray(facets[:, j]), dtype=wp.vec3, device=dev) for j in range(3)]
        fmat = wp.array(sc.facet_mat if sc.facet_mat is not None else np.zeros(0, np.int32), dtype=wp.int32, device=dev)
        count = wp.zeros(1, dtype=wp.int32, device=dev)
        empty = [wp.zeros(1, dtype=wp.vec3, device=dev)] * 3 + [wp.zeros(1, dtype=wp.vec4, device=dev)]

        def run(store, dyn):
            if len(sc.pos):
                wp.launch(static_kernel, dim=len(sc.pos), device=dev,
                          inputs=[self.p, self.planes, self.box, *points, store, self.static_grid, *dyn, count])
            if len(facets):
                wp.launch(facet_kernel, dim=len(facets), device=dev,
                          inputs=[self.p, self.planes, self.box, *tri, fmat, r0, sh2, store, self.static_grid, *dyn,
                                  count])
        # pass 1 counts the paths that cross the plume region, pass 2 stores them and deposits the rest
        run(0, empty)
        n_dyn = int(count.numpy()[0])
        self.dyn = [wp.zeros(max(n_dyn, 1), dtype=wp.vec3, device=dev) for _ in range(3)] + \
                   [wp.zeros(max(n_dyn, 1), dtype=wp.vec4, device=dev)]
        count.zero_()
        run(1, self.dyn)
        wp.synchronize_device(dev)
        self.n_dyn = n_dyn
        self.n_scatterers = len(sc.pos)
        self.n_facets = len(facets)

    def _reverb(self, cfg, scene) -> np.ndarray:
        """Per (range bin, u cell): amplitude std of volume reverberation, from the water inside each cell's
        two-way beam (tank box, above the sand, below the surface)."""
        son = get(cfg, "sonar")
        L, W, _ = (float(v) for v in get(cfg, "tank.size"))
        wl = float(get(cfg, "tank.water_level"))
        z_sand = float(scene.sand.base) if scene is not None else 0.0
        nu_c = 256
        u = (np.arange(nu_c) + 0.5) / nu_c * 2 - 1
        el = np.radians(np.linspace(-40, 40, 41))
        de = el[1] - el[0]
        cu, ce = np.meshgrid(u, el, indexing="ij")
        sphi = cu / np.cos(ce)
        ok = np.abs(sphi) < 1
        cphi = np.sqrt(np.clip(1 - sphi ** 2, 1e-9, 1))
        body = np.stack([np.cos(ce) * cphi, -np.cos(ce) * sphi, np.sin(ce)], -1)
        world = body @ self.rot.T
        v = vertical_pattern(self.vert * np.sin(ce), self.vtaper)
        gain2 = (v ** 4) * cphi ** (4 * self.fan_n) * ok
        domega = (2.0 / nu_c) * de / cphi                             # solid angle of a (u, elevation) cell
        sv = 10 ** (float(son["volume_sv_db"]) / 10)
        sl2 = 10 ** (float(son["source_level_db"]) / 10)
        att = self.alpha_db / (20 * math.log10(math.e))
        out = np.zeros((self.n_r, nu_c), np.float32)
        for j in range(1, self.n_r):
            r = j * self.dr
            pt = self.position + r * world
            water = ((pt[..., 0] > 0) & (pt[..., 0] < L) & (pt[..., 1] > 0) & (pt[..., 1] < W)
                     & (pt[..., 2] > z_sand) & (pt[..., 2] < wl))
            inten = sl2 * sv * self.dr * np.sum(gain2 * domega * water, 1) / r ** 2 * math.exp(-4 * att * r)
            out[j] = inten / (self.n_u / nu_c)                       # spread over the fine u cells
        fine = np.repeat(out, self.n_u // nu_c, axis=1)
        return np.sqrt(fine / 2).astype(np.float32)

    def _ringdown(self, son):
        """Frozen per-element ring-down: random phases, amplitude decaying with e-folding ringdown_m, at the beam
        output ringdown_db above the noise floor at zero range."""
        rng = np.random.default_rng(12345)
        n = int(min(self.n_r, math.ceil(8 * float(son["ringdown_m"]) / self.dr)))
        var0 = 10 ** ((self.nf_db + float(son["ringdown_db"])) / 10) / float(np.sum(self.w ** 2))
        amp = np.sqrt(var0) * np.exp(-np.arange(n) * self.dr / float(son["ringdown_m"]))
        z = (rng.normal(size=(n, len(self.xs))) + 1j * rng.normal(size=(n, len(self.xs)))) / math.sqrt(2)
        out = torch.zeros((self.n_r, len(self.xs)), dtype=torch.complex64, device=self.device)
        out[:n] = torch.from_numpy((amp[:, None] * z).astype(np.complex64)).to(self.device)
        return out

    # ----------------------------------------------------------------------------------------- ping
    def ping(self, bubbles=None, targets=None, focus: bool | None = None) -> Ping:
        """One ping. ``bubbles``: a BubbleSim (its GPU arrays are read directly); ``targets``: [(position, sigma_bs)]
        point scatterers (tests). ``focus`` overrides sonar.focus."""
        dev, p = self.device, self.p
        focus = self.focus if focus is None else focus
        tm, t_last = {}, [time.perf_counter()]

        def lap(name):
            wp.synchronize_device(dev)
            now = time.perf_counter()
            tm[name] = (now - t_last[0]) * 1e3
            t_last[0] = now
        self.seq += 1
        seed = self.seed + 104729 * self.seq
        wp.copy(self.grid, self.static_grid)
        self.grid_b.zero_()
        self.grid_d.zero_()
        self.ext.zero_()
        n_bub = 0
        if bubbles is not None:
            n_bub = bubbles.cap
            sig = wp.empty(n_bub, dtype=float, device=dev)
            sext = wp.empty(n_bub, dtype=float, device=dev)
            wp.launch(bubble_sigma_kernel, dim=n_bub, device=dev,
                      inputs=[bubbles.r, bubbles.alive, self.k, self.tab_bs, self.tab_ext, self.tables.log_ka0,
                              self.tables.dlog_ka, sig, sext])
            if self.box.on:
                wp.launch(extinction_kernel, dim=n_bub, device=dev,
                          inputs=[bubbles.pos, sext, self.box, 1.0 / self.box.vox ** 3, self.ext])
        lap("extinction_grid")
        if self.n_dyn:
            wp.launch(dynamic_kernel, dim=self.n_dyn, device=dev, inputs=[p, self.box, self.ext, *self.dyn, self.grid])
        lap("plume_box_paths")
        if self.speckle[1] > 0:
            wp.launch(speckle_kernel, dim=self.grid.shape, device=dev,
                      inputs=[self.grid, self.phi, self.speckle[0], self.speckle[1], seed])
        lap("speckle_dynamics")
        if n_bub:
            wp.launch(point_kernel, dim=n_bub, device=dev,
                      inputs=[p, self.planes, self.n_point_planes, self.box, self.ext, bubbles.pos, sig, self.grid,
                              self.grid_b, self.grid_d, 1])
        if targets:
            tpos = wp.array(np.array([t[0] for t in targets], np.float32), dtype=wp.vec3, device=dev)
            tsig = wp.array(np.array([t[1] for t in targets], np.float32), dtype=float, device=dev)
            wp.launch(point_kernel, dim=len(targets), device=dev,
                      inputs=[p, self.planes, self.n_point_planes, self.box, self.ext, tpos, tsig, self.grid,
                              self.grid_b, self.grid_d, 0])
        lap("bubbles_targets")
        wp.launch(reverb_kernel, dim=self.grid.shape, device=dev, inputs=[self.grid, self.rev, seed + 1])
        lap("reverberation")
        self._beamform(self.grid, self.power, focus, noise=True, seed=seed)
        self._beamform(self.grid_b, self.power_b, focus, noise=False, seed=seed)
        self._beamform(self.grid_d, self.power_d, focus, noise=False, seed=seed)
        lap("fft_beamforming")
        lo, hi = self.display
        wp.launch(display_kernel, dim=self.power.shape, device=dev,
                  inputs=[self.power, self.power_b, self.power_d, self.dr, int(math.ceil(self.min_range / self.dr)),
                          self.min_range, self.tvg_k, self.alpha_db, self.nf_db, lo, hi, self.mask_threshold, self.db,
                          self.img, self.share, self.mask, self.share_d, self.mask_d])
        lap("display")
        out = Ping(power=self.power.numpy(), db=self.db.numpy(), image=self.img.numpy(),
                   plume_share_all=self.share.numpy(), plume_mask_all=self.mask.numpy(),
                   plume_share_direct=self.share_d.numpy(), plume_mask_direct=self.mask_d.numpy(),
                   range_m=self.range_m, bearing_deg=self.bearing, timing_ms=tm)
        lap("download")
        tm["total"] = sum(tm.values())
        return out

    def _beamform(self, grid, power, focus: bool, noise: bool, seed: int):
        """u grid -> aperture (FFT, half-wavelength samples) [+ noise, ring-down] -> beams."""
        g = torch.view_as_complex(wp.to_torch(grid))
        a = torch.fft.fftshift(torch.fft.fft(torch.fft.ifftshift(g, dim=1), dim=1), dim=1)
        m = len(self.xs)
        ap = a[:, self.n_u // 2 - (m - 1) // 2: self.n_u // 2 + (m - 1) // 2 + 1]
        if noise:
            if self.receiver_noise:
                gen = torch.Generator(device=self.device).manual_seed(seed)
                ap = ap + math.sqrt(self.noise_var) * torch.randn(ap.shape, dtype=torch.complex64, device=self.device,
                                                                  generator=gen)
            if self.ringdown is not None:
                ap = ap + self.ringdown
        ap = torch.view_as_real(ap.contiguous()).contiguous()
        torch.cuda.synchronize()
        wp.launch(beamform_kernel, dim=power.shape, device=self.device,
                  inputs=[wp.from_torch(ap, dtype=wp.vec2), self.xs_w, self.w_w, self.ub, self.k, self.dr, int(focus),
                          power])

    def fan(self, image: np.ndarray, width: int = 1024) -> np.ndarray:
        """Cartesian fan image (uint8) of a polar image (range bins x beams)."""
        scale = (width - 1) / (2 * self.range * max(math.sin(self.theta_max), 0.5))
        h = int(self.range * scale) + 2
        out = wp.zeros((h, width), dtype=wp.uint8, device=self.device)
        n_b = image.shape[1]
        if self.sine:
            b0, db = -math.sin(self.theta_max), 2 * math.sin(self.theta_max) / (n_b - 1)
        else:
            b0, db = -self.theta_max, 2 * self.theta_max / (n_b - 1)
        wp.launch(fan_kernel, dim=out.shape, device=self.device,
                  inputs=[wp.array(image, dtype=wp.uint8, device=self.device), self.dr, b0, db, int(self.sine),
                          self.theta_max, self.range, scale, out])
        return out.numpy()

    def to_sonar_frame(self, pts) -> np.ndarray:
        """World points -> (range, bearing [deg, + starboard], elevation [deg])."""
        rel = (np.atleast_2d(pts) - self.position) @ self.rot
        r = np.linalg.norm(rel, axis=1)
        return np.stack([r, np.degrees(np.arctan2(-rel[:, 1], rel[:, 0])), np.degrees(np.arcsin(rel[:, 2] / r))], 1)
