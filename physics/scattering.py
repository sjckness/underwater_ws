"""Acoustic scattering by gas bubbles in water, across ka (k: wavenumber in water, a: bubble radius).

At 750 kHz, bubbles of 0.5-5 mm radius have ka = 1.6-16: neither the resonance region (Minnaert, ka ~ 0.014,
irrelevant here) nor the geometric limit.
- Reference: Anderson (1950) modal series for a fluid sphere, exact for a lossless sphere. For gas in water the
  density ratio g = rho_gas/rho_water is ~1e-3 and the sound-speed ratio h = c_gas/c_water ~0.3.
- Runtime backscatter: Stanton (1989, J. Acoust. Soc. Am. 86, 1499) high-pass model for a gas-filled sphere, closed
  form from the Rayleigh to the geometric regime (constants as in the hydrac implementation of Stanton 1989).
- Runtime extinction: two-parameter fit to the modal series, Q = 2 + 2 / (1 + (ka/KA_EXT)^P_EXT); the closed forms
  in the literature (Thuraisingham 1997, Ainslie & Leighton 2011) are built for ka <~ 1.
Both runtime quantities are tabulated against ka (``Tables``) for lookup on the GPU.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import water as wat

KA_EXT, P_EXT = 2.24, 0.95          # extinction fit to the modal series, ka 0.1-100 (tests/test_scattering.py)


def gas_ratios(rho_water: float, c_water: float, p: float, t_k: float, gas: str) -> tuple[float, float]:
    """g = rho_gas / rho_water and h = c_gas / c_water for an ideal gas at pressure p; adiabatic sound speed (at
    these frequencies the bubble is far too large for heat to diffuse in a period)."""
    gp = wat.GASES[gas]
    return p / (gp["R"] * t_k) / rho_water, math.sqrt(gp["gamma"] * gp["R"] * t_k) / c_water


def modal_coefficients(ka: float, g: float, h: float) -> np.ndarray:
    """Scattering coefficients A_n of a fluid sphere (incident plane wave sum i^n (2n+1) j_n P_n, scattered
    sum i^n (2n+1) A_n h_n^(1) P_n), from continuity of pressure and normal velocity at r = a."""
    from scipy.special import spherical_jn, spherical_yn
    n = np.arange(int(ka + 4.05 * ka ** (1 / 3) + 10) + 1)
    k1a = ka / h
    j, dj = spherical_jn(n, ka), spherical_jn(n, ka, derivative=True)
    y, dy = spherical_yn(n, ka), spherical_yn(n, ka, derivative=True)
    j1, dj1 = spherical_jn(n, k1a), spherical_jn(n, k1a, derivative=True)
    hn, dhn = j + 1j * y, dj + 1j * dy
    gamma = 1.0 / (g * h)           # (k1 rho)/(k rho1), times j_n'(k1 a)/j_n(k1 a) below (multiplied through)
    return -(dj * j1 - gamma * j * dj1) / (dhn * j1 - gamma * hn * dj1)


def modal_backscatter(ka: float, g: float, h: float) -> float:
    """sigma_bs / a^2 = |f(pi)|^2 / a^2, f(pi) = (-i/k) sum (2n+1) (-1)^n A_n."""
    a_n = modal_coefficients(ka, g, h)
    n = np.arange(len(a_n))
    return float(abs(np.sum((2 * n + 1) * (-1.0) ** n * a_n)) ** 2 / ka ** 2)


def modal_extinction(ka: float, g: float, h: float) -> float:
    """sigma_ext / (pi a^2), optical theorem: sigma_ext = (4 pi / k) Im f(0) = -(4 pi / k^2) sum (2n+1) Re A_n."""
    a_n = modal_coefficients(ka, g, h)
    n = np.arange(len(a_n))
    return float(-4.0 / ka ** 2 * np.sum((2 * n + 1) * a_n.real))


def stanton_backscatter(ka, g: float, h: float):
    """sigma_bs / a^2, Stanton (1989) high-pass model for a gas-filled sphere:
    sigma_bs = a^2 (ka)^4 alpha^2 G / (1 + 4 (ka)^4 alpha^2 / (R^2 F)),
    alpha = (1 - g h^2) / (3 g h^2) + (1 - g) / (1 + 2 g), R = (g h - 1) / (g h + 1),
    F = 1 + 0.5 (ka)^-0.75, G = 1 + 85 exp(-5e5 (ka - 0.0135)^2) (resonance of an air bubble at the surface)."""
    ka = np.asarray(ka, float)
    alpha = (1 - g * h * h) / (3 * g * h * h) + (1 - g) / (1 + 2 * g)
    r = (g * h - 1) / (g * h + 1)
    f = 1 + 0.5 * ka ** -0.75
    gg = 1 + 85 * np.exp(-5e5 * (ka - 0.0135) ** 2)
    return ka ** 4 * alpha ** 2 * gg / (1 + 4 * ka ** 4 * alpha ** 2 / (r * r * f))


def extinction_fit(ka):
    """sigma_ext / (pi a^2): 4 (monopole, above resonance) for ka << 1, 2 (extinction paradox) for ka >> 1."""
    ka = np.asarray(ka, float)
    return 2.0 + 2.0 / (1.0 + (ka / KA_EXT) ** P_EXT)


def modal_bistatic(ka: float, g: float, h: float, theta) -> np.ndarray:
    """sigma(theta) / a^2 = |f(theta)|^2 / a^2 at scattering angles theta [rad] (0: forward, pi: back),
    f(theta) = (-i/k) sum (2n+1) A_n P_n(cos theta)."""
    from scipy.special import eval_legendre
    a_n = modal_coefficients(ka, g, h)
    n = np.arange(len(a_n))
    p_n = eval_legendre(n[:, None], np.cos(np.atleast_1d(np.asarray(theta, float)))[None, :])
    return np.abs(((2 * n + 1) * a_n) @ p_n) ** 2 / ka ** 2


def bistatic_ratio_table(g: float, h: float, ka_min: float = 0.05, ka_max: float = 200.0, n_ka: int = 120,
                         n_theta: int = 61, smooth: float = 0.1):
    """sigma(ka, theta) / sigma(ka, pi) of the gas sphere (modal series), each averaged over ka within +-``smooth``
    (relative) so that the ideal sphere's narrow interior resonances do not dominate. Rows: log-spaced ka; columns:
    theta from 0 to pi. Returns (log10 ka_min, d log10 ka, table)."""
    theta = np.linspace(0.0, math.pi, n_theta)
    ka = np.logspace(math.log10(ka_min), math.log10(ka_max), n_ka)
    out = np.zeros((n_ka, n_theta))
    for i, x in enumerate(ka):
        sub = x * np.linspace(1 - smooth, 1 + smooth, 9)
        s = np.mean([modal_bistatic(y, g, h, theta) for y in sub], 0)
        out[i] = s / s[-1]
    return math.log10(ka_min), (math.log10(ka_max) - math.log10(ka_min)) / (n_ka - 1), out.astype(np.float32)


@dataclass
class Tables:
    """sigma_bs / a^2 and sigma_ext / (pi a^2) on a log-spaced ka grid, for linear lookup in log10(ka)."""
    log_ka0: float
    dlog_ka: float
    backscatter: np.ndarray
    extinction: np.ndarray

    @classmethod
    def build(cls, g: float, h: float, ka_min: float = 1e-2, ka_max: float = 1e3, n: int = 2048) -> "Tables":
        ka = np.logspace(math.log10(ka_min), math.log10(ka_max), n)
        return cls(math.log10(ka_min), (math.log10(ka_max) - math.log10(ka_min)) / (n - 1),
                   stanton_backscatter(ka, g, h).astype(np.float32), extinction_fit(ka).astype(np.float32))

    def lookup(self, table: np.ndarray, ka):
        x = (np.log10(np.asarray(ka, float)) - self.log_ka0) / self.dlog_ka
        return np.interp(x, np.arange(len(table)), table)


# ------------------------------------------------------------------------------- coherent facets (Kirchhoff)

def triangle_phase_integral(t, area: float) -> complex:
    """Integral of exp(i g.(x - c)) over a planar triangle of the given area, from the phases t_j = g.(v_j - c) at its
    vertices: 2 area x the integral over the unit simplex, which by Hermite-Genocchi is minus the second divided
    difference of exp(i t). Exact for any phase gradient (no sampling); host reference of sonar.tri_integral."""
    a, b, c = sorted(float(v) for v in t)
    if c - a < 1e-3:
        return area * complex(math.cos((a + b + c) / 3), math.sin((a + b + c) / 3))

    def dd(x, y):                       # (e^{iy} - e^{ix}) / (y - x), stable
        h = 0.5 * (y - x)
        s = math.sin(h) / h if abs(h) > 1e-9 else 1.0
        return 1j * complex(math.cos(0.5 * (x + y)), math.sin(0.5 * (x + y))) * s
    return 2 * area * (-(dd(b, c) - dd(a, b)) / (c - a))
