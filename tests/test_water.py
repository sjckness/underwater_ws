"""Water and gas properties (host): reference values and the leak flow."""
import math

import pytest

from uw import config, geometry, plume
from uw import water as wat


def francois_garrison_db_per_m(f_hz, t, s, depth=0.0, ph=8.0):
    """Francois & Garrison (1982, JASA 72, 1879) seawater absorption: an independent check of Ainslie & McColm."""
    f = f_hz / 1000.0
    c = 1412 + 3.21 * t + 1.19 * s + 0.0167 * depth
    a1 = 8.86 / c * 10 ** (0.78 * ph - 5)
    f1 = 2.8 * math.sqrt(s / 35) * 10 ** (4 - 1245 / (273 + t))
    a2 = 21.44 * s / c * (1 + 0.025 * t)
    p2 = 1 - 1.37e-4 * depth + 6.2e-9 * depth ** 2
    f2 = 8.17 * 10 ** (8 - 1990 / (273 + t)) / (1 + 0.0018 * (s - 35))
    p3 = 1 - 3.83e-5 * depth + 4.9e-10 * depth ** 2
    if t <= 20:
        a3 = 4.937e-4 - 2.59e-5 * t + 9.11e-7 * t ** 2 - 1.50e-8 * t ** 3
    else:
        a3 = 3.964e-4 - 1.146e-5 * t + 1.45e-7 * t ** 2 - 6.5e-10 * t ** 3
    return (a1 * f1 * f * f / (f1 * f1 + f * f) + a2 * p2 * f2 * f * f / (f2 * f2 + f * f) + a3 * p3 * f * f) / 1000


def test_density():
    # UNESCO EOS-80 (Millero & Poisson 1981): rho(S=35, T=15 C, p=0) = 1025.97 kg/m^3
    assert wat.density(15.0, 35.0) == pytest.approx(1025.97, abs=0.2)


def test_sound_speed():
    # Mackenzie (1981) at 15 C, 35 psu, surface
    assert wat.sound_speed(15.0, 35.0, 0.0) == pytest.approx(1506.7, abs=0.1)


def test_absorption_750khz():
    a = wat.absorption_db_per_m(750e3, 15.0, 35.0)
    assert a == pytest.approx(0.23, abs=0.01)
    assert a == pytest.approx(francois_garrison_db_per_m(750e3, 15.0, 35.0), rel=0.10)


def test_leak_flow_is_continuous_at_the_choking_point():
    """The subsonic and choked branches must meet where p0/p_amb reaches the critical ratio."""
    g = wat.GASES["methane"]["gamma"]
    crit = ((g + 1) / 2) ** (g / (g - 1))
    p_amb = 1.3e5
    below = wat.choked_leak(p_amb * crit * (1 - 1e-6), p_amb, 1e-3, "methane", 288.15)
    above = wat.choked_leak(p_amb * crit * (1 + 1e-6), p_amb, 1e-3, "methane", 288.15)
    assert not below["choked"] and above["choked"]
    assert below["mdot"] == pytest.approx(above["mdot"], rel=1e-4)
    assert below["u_exit"] == pytest.approx(above["u_exit"], rel=1e-3)


def test_default_leak_is_choked_and_medium():
    """Methane, 3 bar over the water pressure, 1 mm hole, Cd 0.62, 15 C: choked, 25.47 L/min at the hole depth."""
    cfg = config.load()
    src = plume.leak_source(cfg, geometry.build_scene(cfg))
    assert src.info["choked"]
    assert src.q_gas * 60000 == pytest.approx(25.47, abs=0.05)
    assert src.mdot == pytest.approx(3.596e-4, rel=1e-3)
