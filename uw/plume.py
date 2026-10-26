"""Gas leak plume physics on the host (numpy only): source, bubble sizes and slip, integral plume, references.

bubbles.py runs the bubbles on the GPU on top of this; everything here is testable without Warp.
1. Source: choked or subsonic gas flow through the hole (``water.choked_leak``) gives the gas flow and the jet
   momentum flux; a buried leak releases through sediment vents with no momentum.
2. Sizes: log-normal number distribution of volume-equivalent diameters, truncated to ``leak.d_range_mm``.
3. Slip: Fan & Tsuchiya (1990) terminal rise velocity for contaminated water.
4. Liquid flow: top-hat integral model of a forced bubble plume (entrainment ``alpha``; buoyancy per unit length
   = g * gas volume per unit length, with the gas expanding as it rises).
5. References for validation: Morton, Taylor & Turner (1956) pure plume; Lai & Socolofsky (2019) far-field fit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import water as wat
from .config import ConfigError, get

N_CL = 160          # centreline samples per plume


# ------------------------------------------------------------------------------------- bubble slip

def bubble_kb(rho: float, mu: float, sigma: float) -> float:
    """Fan & Tsuchiya's K_b = max(14.7 Mo^-0.038, 12) for aqueous liquids (Mo: Morton number)."""
    mo = wat.G * mu ** 4 / (rho * sigma ** 3)
    return max(14.7 * mo ** -0.038, 12.0)


def slip_velocity(d, rho, mu, sigma):
    """Fan & Tsuchiya 1990 terminal rise velocity, contaminated water (n = 0.8, c = 1.4, Kb0 = 14.7)."""
    kb = bubble_kb(rho, mu, sigma)
    n = 0.8
    t1 = rho * wat.G * d * d / (kb * mu)
    t2 = 2 * 1.4 * sigma / (rho * d) + wat.G * d / 2
    return (t1 ** -n + t2 ** (-n / 2)) ** (-1 / n)


# ------------------------------------------------------------------------------------ bubble sizes

def _phi(x: float) -> float:
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


@dataclass
class SizeDistribution:
    """Log-normal number distribution of volume-equivalent diameters, truncated (not clipped) to [d_min, d_max]."""
    d50: float          # number median [m]
    sigma_ln: float
    d_min: float
    d_max: float

    def moment(self, k: float) -> float:
        """E[d^k] of the truncated distribution, closed form."""
        s = self.sigma_ln
        a = math.log(self.d_min / self.d50) / s
        b = math.log(self.d_max / self.d50) / s
        return (self.d50 ** k * math.exp((k * s) ** 2 / 2)
                * (_phi(b - k * s) - _phi(a - k * s)) / (_phi(b) - _phi(a)))

    @property
    def mean_volume(self) -> float:
        return math.pi / 6 * self.moment(3)

    def sample(self, rng: np.random.Generator, n: int) -> np.ndarray:
        """``n`` diameters; draws outside [d_min, d_max] are redrawn."""
        out = np.empty(0)
        while len(out) < n:
            d = self.d50 * np.exp(self.sigma_ln * rng.normal(size=int(1.1 * (n - len(out))) + 8))
            out = np.concatenate([out, d[(d >= self.d_min) & (d <= self.d_max)]])
        return out[:n]

    def volume_weighted_mean(self, f) -> float:
        """Mean of ``f(d)`` weighted by gas volume (d^3 times the number density), by quadrature in ln d."""
        x = np.linspace(math.log(self.d_min), math.log(self.d_max), 2001)
        d = np.exp(x)
        w = np.exp(-0.5 * ((x - math.log(self.d50)) / self.sigma_ln) ** 2) * d ** 3
        return float(np.sum(w * f(d)) / np.sum(w))


class Emitter:
    """Turns a gas volume flux into bubbles. The volume not yet emitted carries over to the next call, so over time
    the emitted volume matches ``q`` exactly while the sizes follow ``sizes``."""

    def __init__(self, q: float, sizes: SizeDistribution, rng: np.random.Generator):
        self.q, self.sizes, self.rng = q, sizes, rng
        self.debt = 0.0

    def emit(self, dt: float) -> np.ndarray:
        """Diameters [m] of the bubbles released during ``dt``."""
        vol = self.q * dt + self.debt
        n = int(vol / self.sizes.mean_volume) if vol > 0 else 0
        d = self.sizes.sample(self.rng, n) if n else np.empty(0)
        self.debt = vol - float(np.sum(math.pi / 6 * d ** 3))
        return d


# ------------------------------------------------------------------------------------------ source

@dataclass
class Source:
    """Where and how much gas is released, and the properties the bubbles need."""
    rho: float                  # water density [kg/m^3]
    mu: float                   # water dynamic viscosity [Pa s]
    sigma: float                # surface tension [N/m]
    kb: float                   # Fan & Tsuchiya K_b
    mdot: float                 # gas mass flow [kg/s]
    q_gas: float                # gas volume flow at the release pressure [m^3/s]
    momentum: float             # jet momentum flux [N]; 0 for a buried leak
    p_release: float            # absolute pressure where the bubbles form [Pa]
    positions: np.ndarray       # (K, 3) release points: the hole (2 mm out along the jet) or the sediment vents
    direction: np.ndarray       # (3,) initial jet direction; +z for vents
    buried: bool
    sizes: SizeDistribution
    slip_mean: float            # volume-weighted mean slip of the size distribution [m/s]
    info: dict = field(default_factory=dict)    # leak computation details (choked, exit velocity, ...)


def leak_source(cfg: dict, scene) -> Source:
    """The gas source of ``scene.leak``. ``leak.pipe_pressure_bar`` is the pressure difference across the pipe wall
    (gauge relative to the water at the hole); the gas takes the water temperature (isothermal)."""
    leak = scene.leak
    if leak is None:
        raise ConfigError("leak.enabled is false: there is no gas source")
    lc, wc = get(cfg, "leak"), get(cfg, "water")
    t, s = float(wc["temperature_c"]), float(wc["salinity_psu"])
    rho, mu, sigma = wat.density(t, s), wat.viscosity(t, s), wat.surface_tension(t, s)
    ambient = float(wc["ambient_depth_m"])
    wl = float(scene.water_level)
    if leak.buried:
        positions = leak.vents + np.array([0.0, 0.0, 0.003])
        direction = np.array([0.0, 0.0, 1.0])
    else:
        positions = (leak.hole + 0.002 * leak.direction)[None, :]
        direction = np.asarray(leak.direction, float)
    p_rel = wat.pressure(float(positions[:, 2].mean()), wl, rho, ambient)
    t_k = t + 273.15
    r_gas = wat.GASES[lc["gas"]]["R"]
    if lc["source"] == "pressure":
        p0 = float(lc["pipe_pressure_bar"]) * 1e5 + wat.pressure(float(leak.hole[2]), wl, rho, ambient)
        info = wat.choked_leak(p0, p_rel, float(lc["hole_diameter_mm"]) * 1e-3, lc["gas"], t_k,
                               float(lc["discharge_coeff"]))
        mdot, momentum = info["mdot"], info["momentum_flux"]
    elif lc["source"] == "flow":
        info = {}
        mdot, momentum = float(lc["flow_lpm"]) / 60000.0 * p_rel / (r_gas * t_k), 0.0
    else:
        raise ConfigError(f"leak.source must be pressure or flow, got {lc['source']!r}")
    d50 = float(lc["d50_mm"]) * 1e-3
    if leak.buried:
        momentum = 0.0          # the sand takes the jet's momentum
        d50 *= float(lc["buried_size_factor"])
    dmin, dmax = (x * 1e-3 for x in lc["d_range_mm"])
    sizes = SizeDistribution(d50, float(lc["sigma_ln"]), dmin, dmax)
    return Source(rho=rho, mu=mu, sigma=sigma, kb=bubble_kb(rho, mu, sigma), mdot=mdot,
                  q_gas=mdot * r_gas * t_k / p_rel, momentum=momentum, p_release=p_rel, positions=positions,
                  direction=direction, buried=bool(leak.buried), sizes=sizes,
                  slip_mean=sizes.volume_weighted_mean(lambda d: slip_velocity(d, rho, mu, sigma)), info=info)


# ---------------------------------------------------------------------------------- integral plume

def integrate_plume(origin, direction, momentum_k, q_gas, slip, alpha, b0, z_stop, ds=0.001, s_max=8.0,
                    q_ratio=None):
    """Top-hat forced bubble plume along its curved centreline.

    ``alpha`` is the entrainment coefficient in the Gaussian convention that papers quote; the top-hat equations use
    sqrt(2) alpha (same volume and momentum fluxes). ``q_ratio(z)``: local gas volume flux / ``q_gas`` (expansion),
    default 1. Returns arrays along the centreline, resampled to N_CL points: pos (N,3), tangent (N,3), u_c
    (Gaussian centreline speed), b (Gaussian 1/e radius), tau (liquid travel time).
    """
    alpha_t = math.sqrt(2.0) * alpha
    c = np.asarray(origin, float).copy()
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    u0 = max(math.sqrt(momentum_k / (math.pi * b0 * b0)) if momentum_k > 0 else 0.0, 0.05)
    Q = math.pi * b0 * b0 * u0
    M = Q * u0 * d
    g = wat.G
    rec_s, rec = [], []
    s = tau = 0.0
    while c[2] < z_stop and s < s_max:
        m = np.linalg.norm(M)
        u = m / Q
        t = M / m
        b = math.sqrt(Q / (math.pi * u))
        rec_s.append(s)
        rec.append((*c, *t, 2.0 * u, b / math.sqrt(2.0), tau))
        q = q_gas * (q_ratio(c[2]) if q_ratio is not None else 1.0)
        Q += 2 * math.pi * b * alpha_t * u * ds
        M = M + np.array([0.0, 0.0, g * q / max(u + slip * t[2], 0.05)]) * ds
        c = c + t * ds
        s += ds
        tau += ds / u
    rec = np.array(rec)
    rec_s = np.array(rec_s)
    s_new = rec_s[-1] * (np.arange(N_CL) / (N_CL - 1)) ** 1.5      # dense near the source
    out = np.stack([np.interp(s_new, rec_s, rec[:, k]) for k in range(rec.shape[1])], 1)
    tan = out[:, 3:6] / np.linalg.norm(out[:, 3:6], axis=1, keepdims=True)
    return {"pos": out[:, 0:3], "tangent": tan, "u_c": out[:, 6], "b": out[:, 7], "tau": out[:, 8]}


# ---------------------------------------------------------------------------------------- references

def mtt_plume(z, buoyancy_flux, alpha):
    """Morton, Taylor & Turner (1956) pure plume from a point source, Gaussian convention: centreline speed and
    1/e radius at height ``z`` for the kinematic buoyancy flux B = g Q_gas [m^4/s^3]:
    b = 6/5 alpha z,  u_c = 2 (25 B / (96 pi alpha^2))^(1/3) z^(-1/3)
    (the top-hat solution b = 6/5 alpha_t z, u = (25 B / (48 pi alpha_t^2))^(1/3) z^(-1/3) with alpha_t = sqrt(2)
    alpha, u_c = 2 u, b_Gauss = b_tophat / sqrt(2))."""
    z = np.asarray(z, float)
    return 2.0 * (25.0 * buoyancy_flux / (96.0 * math.pi * alpha ** 2)) ** (1 / 3) * z ** (-1 / 3), 1.2 * alpha * z


def plume_length_scale(q_gas: float, slip: float, alpha0: float = 0.083) -> float:
    """Bombardelli et al. (2007) dynamic length scale of a bubble plume, D = g Q0 / (4 pi alpha0^2 u_s^3), with
    their constant entrainment coefficient alpha0 = 0.083 (as used by Bordoloi et al. 2019, sec. 3.1)."""
    return wat.G * q_gas / (4 * math.pi * alpha0 ** 2 * slip ** 3)


def lai_socolofsky_2019(z, q_gas, slip):
    """Far-field bubble-plume fits (unstratified, stationary water): U_c / u_s = 1.6 (z/D)^(-1/3) (Lai & Socolofsky
    2019, J. Fluid Mech. 865, 993-1041) and Gaussian radius b_g = 0.114 z (line through the bubble-plume data incl.
    Lai & Socolofsky 2019), both as given by Bordoloi et al. 2019 (arXiv:1907.08315, sec. 3.1, fig. 7). Measured
    for 2 < z/D < 11; D from ``plume_length_scale``."""
    z = np.asarray(z, float)
    return 1.6 * slip * (z / plume_length_scale(q_gas, slip)) ** (-1 / 3), 0.114 * z
