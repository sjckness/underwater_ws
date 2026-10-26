"""The surface boil: echoes of the water surface where the plume breaks it (Known gap of M2, done in M5).

The bubble simulation counts bursts per surface cell (``BubbleSim.bursts``). Per ping, ``Boil.update`` turns them
into a smoothed burst rate per cell (bursts per s and m^2, exponential smoothing over ``time_s``), the share of the
cell the bursts keep rough, f = 1 - exp(-rate / saturation_rate), and one point scatterer per active cell, jittered
inside it every ping (random phase: the surface moves) with

    sigma_bs = f A_cell exp(-tan^2 theta / (2 s^2)) / (8 pi s^2 cos^4 theta)

the Kirchhoff (geometric optics) backscatter of a rough pressure-release surface (|R| = 1) with Gaussian slopes of
rms ``s`` per axis, theta the incidence from the vertical, in the sonar model's convention (uw/sonar.py, emit).
The sonar gets them as generic point scatterers (``Sonar.ping(extra=...)``) on the direct path, counted in the
all-paths plume label (plume-caused energy) but not in the direct-path or geometric bubble labels.
"""
from __future__ import annotations

import math

import warp as wp

from .config import get

PI = wp.constant(math.pi)


@wp.kernel
def boil_kernel(bursts: wp.array2d(dtype=wp.int32), activity: wp.array2d(dtype=float), dt: float, keep: float,
                x0: float, y0: float, cell: float, water_level: float, sonar: wp.vec3, s2: float, sat: float,
                seed: int, pos: wp.array(dtype=wp.vec3), sig: wp.array(dtype=float)):
    i, j = wp.tid()
    n = bursts.shape[1]
    rate = float(bursts[i, j]) / (dt * cell * cell)
    a = keep * activity[i, j] + (1.0 - keep) * rate
    activity[i, j] = a
    bursts[i, j] = 0
    f = 1.0 - wp.exp(-a / sat)
    k = i * n + j
    st = wp.rand_init(seed, k)
    q = wp.vec3(x0 + (float(i) + wp.randf(st)) * cell, y0 + (float(j) + wp.randf(st)) * cell, water_level - 1.0e-4)
    pos[k] = q
    s = 0.0
    if f > 1.0e-4:
        d = sonar - q
        c = wp.abs(d[2]) / wp.length(d)                       # cos of the incidence from the vertical
        if c > 1.0e-3:
            t2 = (1.0 - c * c) / (c * c)
            s = f * cell * cell * wp.exp(-t2 / (2.0 * s2)) / (8.0 * PI * s2 * c * c * c * c)
    sig[k] = s


class Boil:
    """Point scatterers of the surface boil, refreshed every ping from the bubble simulation's bursts."""

    def __init__(self, cfg: dict, sim):
        bc = get(cfg, "sonar.boil_backscatter")
        self.enabled = bool(bc["enabled"])
        self.sim, self.device = sim, sim.device
        self.s2 = float(bc["rms_slope"]) ** 2
        self.sat = float(bc["saturation_rate"])
        self.tau = float(bc["time_s"])
        self.seed = int(cfg["seed"]) * 15485863 % (2 ** 31)
        shape = sim.bursts.shape
        self.activity = wp.zeros(shape, dtype=float, device=self.device)
        self.pos = wp.zeros(shape[0] * shape[1], dtype=wp.vec3, device=self.device)
        self.sig = wp.zeros(shape[0] * shape[1], dtype=float, device=self.device)
        sim.bursts.zero_()                                    # count from now on
        self.t_last = sim.t
        self.count = 0

    def update(self, sonar_position) -> list:
        """The boil's scatterers for a ping now, as ``Sonar.ping(extra=...)`` takes them."""
        if not self.enabled:
            return []
        dt = max(self.sim.t - self.t_last, 1e-6)
        self.t_last = self.sim.t
        self.count += 1
        wp.launch(boil_kernel, dim=self.sim.bursts.shape, device=self.device,
                  inputs=[self.sim.bursts, self.activity, dt, math.exp(-dt / self.tau), self.sim.burst_origin[0],
                          self.sim.burst_origin[1], self.sim.burst_cell, self.sim.water_level,
                          wp.vec3(*[float(v) for v in sonar_position]), self.s2, self.sat,
                          (self.seed + 7919 * self.count) % (2 ** 31), self.pos, self.sig])
        return [(self.pos, self.sig, 1)]
