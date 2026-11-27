"""The water's own motion for the pool (M12): a horizontal current, parametric, in pure numpy.

``vehicles.common.dynamics.Vehicle`` asks a current for two things at the vehicle, once per integration step:

* ``velocity_ned(t, depth)``: the water velocity [m/s] in earth NED;
* ``accel_ned(t, depth, w_down)``: its rate of change along the vehicle's path [m/s^2], i.e. the local time
  derivative plus the vertical shear times the vehicle's descent rate ``w_down``. The horizontal gradient is zero
  (the field is uniform in x and y), so the vehicle's horizontal motion adds nothing.

The model is Fossen's (2021, Handbook of Marine Craft Hydrodynamics and Motion Control, 2nd ed., section 10.3) for
an irrotational current: the hydrodynamic terms (added mass, damping) see the velocity relative to the water, the
rigid body the velocity relative to the ground. A vertical profile has shear, hence some vorticity, which the
irrotational model ignores: fine for the slow, gentle profiles of a test pool.

**Conventions.** ``direction_deg`` is where the water flows **towards**, as an ENU azimuth in the pool frame:
0 = +x, 90 = +y, the same as the heading of ``vehicle.start``. Depths are positive down from the free surface.
The profile factor is 1 at the surface, so ``speed_mps`` is the surface speed.

**Gusts** are a fixed random spectrum (a sum of six sinusoids with random periods around ``time_s``), the
construction of ``vehicles.common.dvl._BiasField``: a pure function of time, identical for a given seed whatever the step
schedule, and differentiable in closed form, so ``accel_ned`` is exact.

**Changing the current while it runs** goes through ``Ramped``: a smoothstep blend from the old field to the new
one over ``ramp_s`` simulated seconds, anchored at the step that picks the change up.

No pxr, no Isaac, no ROS: numpy and the standard library, like the rest of ``vehicles.common``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import frames

PROFILES = ("uniform", "power", "linear", "table")


class CurrentError(ValueError):
    """A current that cannot be built from what was asked."""


class _Spectrum:
    """``a * sum_k sin(w_k t + phi_k)`` with ``n`` random angular frequencies around ``2 pi / tau``: zero mean,
    standard deviation ``sigma``, correlation time of order ``tau``. Same construction as ``dvl._BiasField``."""

    def __init__(self, sigma: float, tau: float, seed: int, salt: int, n: int = 6):
        rng = np.random.default_rng([int(seed), int(salt)])
        self.a = float(sigma) * math.sqrt(2.0 / n)
        self.w = 2 * np.pi / (max(float(tau), 1e-6) * rng.uniform(1.0 / 3.0, 3.0, n))
        self.ph = rng.uniform(0, 2 * np.pi, n)

    def at(self, t: float) -> float:
        return float(self.a * np.sum(np.sin(self.w * float(t) + self.ph)))

    def rate(self, t: float) -> float:
        return float(self.a * np.sum(self.w * np.cos(self.w * float(t) + self.ph)))


@dataclass(frozen=True)
class Current:
    """A horizontal current, uniform in x and y, with an optional depth profile and gusts.

    ``water_depth_m`` (the pool's ``water_level``) is only needed by the ``power`` and ``linear`` profiles, which
    are stated over the water column.
    """
    speed_mps: float = 0.0
    direction_deg: float = 0.0
    profile: str = "uniform"
    power_exponent: float = 1.0 / 7.0
    bottom_factor: float = 0.5
    table: tuple = ()                       # ((depth_m, factor), ...), strictly increasing depths
    water_depth_m: float = 5.0
    gust_sigma_mps: float = 0.0
    gust_time_s: float = 20.0
    gust_dir_sigma_deg: float = 0.0
    seed: int = 0
    _speed_gust: _Spectrum | None = field(default=None, init=False, repr=False, compare=False)
    _dir_gust: _Spectrum | None = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self):
        if self.profile not in PROFILES:
            raise CurrentError(f"current.profile must be one of {'|'.join(PROFILES)}, got {self.profile!r}")
        for name in ("speed_mps", "gust_sigma_mps", "gust_dir_sigma_deg"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0.0:
                raise CurrentError(f"current.{name} must be >= 0, got {getattr(self, name)}")
        if self.gust_time_s <= 0.0:
            raise CurrentError(f"current.gust.time_s must be > 0, got {self.gust_time_s}")
        if self.profile in ("power", "linear") and self.water_depth_m <= 0.0:
            raise CurrentError(f"the {self.profile} profile needs the water depth, got {self.water_depth_m}")
        if self.profile == "power" and not 0.0 < self.power_exponent <= 1.0:
            raise CurrentError(f"current.power_exponent must be in (0, 1], got {self.power_exponent}")
        if self.profile == "table":
            tab = np.asarray(self.table, float)
            if tab.ndim != 2 or tab.shape[1] != 2 or len(tab) < 2 or np.any(np.diff(tab[:, 0]) <= 0.0):
                raise CurrentError("current.table needs >= 2 [depth_m, factor] rows with increasing depths")
        object.__setattr__(self, "_speed_gust", _Spectrum(self.gust_sigma_mps, self.gust_time_s, self.seed, 1)
                           if self.gust_sigma_mps > 0.0 else None)
        object.__setattr__(self, "_dir_gust", _Spectrum(math.radians(self.gust_dir_sigma_deg), self.gust_time_s,
                                                       self.seed, 2) if self.gust_dir_sigma_deg > 0.0 else None)

    @property
    def is_still(self) -> bool:
        """True when the water never moves: no speed and no gusts."""
        return self.speed_mps == 0.0 and self.gust_sigma_mps == 0.0

    # ---- the depth profile: a factor and its derivative with respect to depth
    def factor(self, depth: float) -> tuple[float, float]:
        """(f, df/d depth) at ``depth`` [m, positive down]: 1 at the surface for every profile."""
        d = float(depth)
        if self.profile == "uniform":
            return 1.0, 0.0
        if self.profile == "table":
            tab = np.asarray(self.table, float)
            f = float(np.interp(d, tab[:, 0], tab[:, 1]))
            i = int(np.clip(np.searchsorted(tab[:, 0], d) - 1, 0, len(tab) - 2))
            inside = tab[0, 0] < d < tab[-1, 0]
            slope = (tab[i + 1, 1] - tab[i, 1]) / (tab[i + 1, 0] - tab[i, 0]) if inside else 0.0
            return f, float(slope)
        h_tot = self.water_depth_m
        d = min(max(d, 0.0), h_tot)                 # above the surface: the surface value; below the floor: none
        if self.profile == "linear":
            return 1.0 + (self.bottom_factor - 1.0) * d / h_tot, (self.bottom_factor - 1.0) / h_tot
        # power: (h / H)^e with h the height above the floor; the slope is infinite at the floor, so it is
        # evaluated 1 mm above it
        h = max(h_tot - d, 1e-3)
        e = self.power_exponent
        f = (h / h_tot) ** e
        return f, -e * f / h

    # ---- the field
    def _speed_dir(self, t: float) -> tuple[float, float, float, float]:
        """(speed, direction [rad], d speed/dt, d direction/dt) at the surface, gusts included. The speed is
        floored at 0, where its rate is then 0 too."""
        s, ds = self.speed_mps, 0.0
        if self._speed_gust is not None:
            s, ds = s + self._speed_gust.at(t), self._speed_gust.rate(t)
            if s < 0.0:
                s, ds = 0.0, 0.0
        a, da = math.radians(self.direction_deg), 0.0
        if self._dir_gust is not None:
            a, da = a + self._dir_gust.at(t), self._dir_gust.rate(t)
        return s, a, ds, da

    def velocity_ned(self, t: float, depth: float) -> np.ndarray:
        s, a, _, _ = self._speed_dir(t)
        f, _ = self.factor(depth)
        return frames.vec_enu_ned(np.array([s * f * math.cos(a), s * f * math.sin(a), 0.0]))

    def accel_ned(self, t: float, depth: float, w_down: float = 0.0) -> np.ndarray:
        s, a, ds, da = self._speed_dir(t)
        f, df = self.factor(depth)
        c, sn = math.cos(a), math.sin(a)
        # d/dt [s f (cos a, sin a)] = (ds f + s df w_down) (cos a, sin a) + s f da (-sin a, cos a)
        radial = ds * f + s * df * float(w_down)
        tang = s * f * da
        return frames.vec_enu_ned(np.array([radial * c - tang * sn, radial * sn + tang * c, 0.0]))

    def describe(self) -> dict:
        """A plain dict for logs and the control channel."""
        return {"speed_mps": self.speed_mps, "direction_deg": self.direction_deg, "profile": self.profile,
                "gust": {"sigma_mps": self.gust_sigma_mps, "time_s": self.gust_time_s,
                         "dir_sigma_deg": self.gust_dir_sigma_deg}}


STILL = Current()


def _smoothstep(x: float) -> tuple[float, float]:
    """(s, ds/dx) of 3x^2 - 2x^3 on [0, 1], clamped outside."""
    if x <= 0.0:
        return 0.0, 0.0
    if x >= 1.0:
        return 1.0, 0.0
    return x * x * (3.0 - 2.0 * x), 6.0 * x * (1.0 - x)


@dataclass(frozen=True)
class Ramped:
    """From ``old`` to ``new`` over ``ramp_s`` simulated seconds starting at ``t0``: a smoothstep blend of the two
    fields, so velocity and acceleration stay continuous. After the ramp it is ``new`` exactly."""
    old: "Current | Ramped"
    new: Current
    t0: float
    ramp_s: float

    def _s(self, t: float) -> tuple[float, float]:
        if self.ramp_s <= 0.0:
            return 1.0, 0.0
        s, ds = _smoothstep((float(t) - self.t0) / self.ramp_s)
        return s, ds / self.ramp_s

    def done(self, t: float) -> bool:
        return float(t) >= self.t0 + self.ramp_s

    def velocity_ned(self, t: float, depth: float) -> np.ndarray:
        s, _ = self._s(t)
        if s >= 1.0:
            return self.new.velocity_ned(t, depth)
        return (1.0 - s) * self.old.velocity_ned(t, depth) + s * self.new.velocity_ned(t, depth)

    def accel_ned(self, t: float, depth: float, w_down: float = 0.0) -> np.ndarray:
        s, ds = self._s(t)
        if s >= 1.0:
            return self.new.accel_ned(t, depth, w_down)
        v_old, v_new = self.old.velocity_ned(t, depth), self.new.velocity_ned(t, depth)
        return ((1.0 - s) * self.old.accel_ned(t, depth, w_down) + s * self.new.accel_ned(t, depth, w_down)
                + ds * (v_new - v_old))

    @property
    def is_still(self) -> bool:
        return False

    def describe(self) -> dict:
        return dict(self.new.describe(), ramp={"t0": self.t0, "ramp_s": self.ramp_s})


def from_config(cfg: dict, section: str = "current", water_depth_m: float | None = None) -> Current:
    """The ``current:`` block of sim/config/pool/pool.yaml as a ``Current`` (``STILL`` without the block)."""
    from uw.config import ConfigError, get
    try:
        c = get(cfg, section)
    except ConfigError:                         # a config without the block (the tank's): still water
        return STILL
    if not c:
        return STILL
    if water_depth_m is None:
        water_depth_m = float(get(cfg, "pool.water_level"))
    return from_dict(c, water_depth_m=water_depth_m, seed=int(cfg.get("seed", 0)))


def from_dict(d: dict, water_depth_m: float = 5.0, seed: int = 0) -> Current:
    """A ``Current`` from a plain dict, the shape of the config block (also what the control channel sends).
    Unknown keys raise, so a typo is not a silently still pool."""
    known = {"speed_mps", "direction_deg", "profile", "power_exponent", "bottom_factor", "table", "gust", "seed"}
    unknown = set(d) - known
    if unknown:
        raise CurrentError(f"unknown current keys: {sorted(unknown)} (known: {sorted(known)})")
    gust = d.get("gust") or {}
    unknown = set(gust) - {"sigma_mps", "time_s", "dir_sigma_deg"}
    if unknown:
        raise CurrentError(f"unknown current.gust keys: {sorted(unknown)}")
    return Current(
        speed_mps=float(d.get("speed_mps", 0.0)), direction_deg=float(d.get("direction_deg", 0.0)),
        profile=str(d.get("profile", "uniform")), power_exponent=float(d.get("power_exponent", 1.0 / 7.0)),
        bottom_factor=float(d.get("bottom_factor", 0.5)),
        table=tuple(tuple(float(x) for x in row) for row in (d.get("table") or ())),
        water_depth_m=float(water_depth_m), gust_sigma_mps=float(gust.get("sigma_mps", 0.0)),
        gust_time_s=float(gust.get("time_s", 20.0)), gust_dir_sigma_deg=float(gust.get("dir_sigma_deg", 0.0)),
        seed=int(d.get("seed", seed)))
