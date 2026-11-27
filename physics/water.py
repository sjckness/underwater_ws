"""Water and gas properties (SI), from temperature, salinity and depth.

Formulas and validity:
- density: Tanaka et al. 2001 pure-water fit + 0.77 kg/m^3 per psu (adequate to ~0.5 kg/m^3 here)
- dynamic viscosity: Vogel-type pure-water fit, +0.23 % per psu (Sharqawy et al. 2010, approx.)
- sound speed: Mackenzie 1981 for S >= 20 psu, else a pure-water polynomial (Marczak 1997) + 1.3 m/s/psu
- absorption: Ainslie & McColm 1998 (100 Hz - 1 MHz; used to 1.2 MHz, where it extrapolates)
"""
from __future__ import annotations

import math

G = 9.80665
P_ATM = 101325.0

GASES = {   # specific gas constant [J/(kg K)], heat capacity ratio
    "methane": {"R": 518.3, "gamma": 1.31},
    "air": {"R": 287.05, "gamma": 1.40},
    "nitrogen": {"R": 296.8, "gamma": 1.40},
    "co2": {"R": 188.9, "gamma": 1.29},
}


def density(t_c: float, s_psu: float) -> float:
    t = t_c
    rho_w = 999.974950 * (1 - (t - 3.983035) ** 2 * (t + 301.797) / (522528.9 * (t + 69.34881)))
    return rho_w + 0.77 * s_psu


def viscosity(t_c: float, s_psu: float) -> float:
    mu_w = 2.414e-5 * 10 ** (247.8 / (t_c + 133.15))
    return mu_w * (1 + 0.0023 * s_psu)


def surface_tension(t_c: float, s_psu: float) -> float:
    return 0.07564 - 0.000141 * t_c + 0.000022 * s_psu


def sound_speed(t_c: float, s_psu: float, depth_m: float = 1.0) -> float:
    t, s, d = t_c, s_psu, depth_m
    if s >= 20:
        return (1448.96 + 4.591 * t - 5.304e-2 * t ** 2 + 2.374e-4 * t ** 3 + 1.340 * (s - 35)
                + 1.630e-2 * d + 1.675e-7 * d ** 2 - 1.025e-2 * t * (s - 35) - 7.139e-13 * t * d ** 3)
    return 1402.385 + 5.038813 * t - 5.799136e-2 * t ** 2 + 3.287156e-4 * t ** 3 + 1.3 * s + 1.6e-2 * d


def absorption_db_per_m(f_hz: float, t_c: float, s_psu: float, depth_m: float = 0.0, ph: float = 8.0) -> float:
    f = f_hz / 1000.0
    d = depth_m / 1000.0
    f1 = 0.78 * math.sqrt(max(s_psu, 0.0) / 35) * math.exp(t_c / 26)
    f2 = 42 * math.exp(t_c / 17)
    boric = 0.106 * f1 * f * f / (f1 * f1 + f * f) * math.exp((ph - 8) / 0.56) if f1 > 0 else 0.0
    mgso4 = 0.52 * (1 + t_c / 43) * (s_psu / 35) * f2 * f * f / (f2 * f2 + f * f) * math.exp(-d / 6)
    pure = 0.00049 * f * f * math.exp(-(t_c / 27 + d / 17))
    return (boric + mgso4 + pure) / 1000.0


def pressure(z: float, water_level: float, rho: float, ambient_depth: float = 0.0) -> float:
    """Absolute pressure at height z in the tank [Pa]."""
    return P_ATM + rho * G * (water_level - z + ambient_depth)


def choked_leak(p0_abs: float, p_amb: float, hole_d: float, gas: str, t_k: float, cd: float = 0.62) -> dict:
    """Gas leak through a sharp-edged hole: mass flow, exit state and jet momentum flux.

    Choked (sonic) when p0/p_amb exceeds the critical ratio; then an under-expanded jet whose
    momentum flux includes the pressure thrust (p* - p_amb) A. Isentropic nozzle relations.
    """
    gp = GASES[gas]
    R, g = gp["R"], gp["gamma"]
    area = cd * math.pi * hole_d ** 2 / 4
    crit = ((g + 1) / 2) ** (g / (g - 1))
    if p0_abs / p_amb >= crit:
        mdot = area * p0_abs * math.sqrt(g / (R * t_k)) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1)))
        t_star = t_k * 2 / (g + 1)
        u_e = math.sqrt(g * R * t_star)
        p_e = p0_abs * (2 / (g + 1)) ** (g / (g - 1))
        choked = True
    else:
        pr = p_amb / p0_abs
        mdot = area * p0_abs * math.sqrt(2 * g / (R * t_k * (g - 1)) * (pr ** (2 / g) - pr ** ((g + 1) / g)))
        t_e = t_k * pr ** ((g - 1) / g)
        rho_e = p_amb / (R * t_e)
        u_e = mdot / (rho_e * area)
        p_e = p_amb
        choked = False
    momentum = mdot * u_e + (p_e - p_amb) * area
    rho_amb = p_amb / (R * t_k)                      # after the gas has taken the water temperature
    return {"mdot": mdot, "u_exit": u_e, "p_exit": p_e, "choked": choked, "momentum_flux": momentum,
            "q_ambient": mdot / rho_amb, "rho_gas_ambient": rho_amb}
