"""Plume physics (host, numpy): bubble slip, pure-plume similarity, bubble sizes and the emitted volume flux."""
import math

import numpy as np
import pytest

from uw import plume
from uw import water as wat

FRESH_20C = (wat.density(20.0, 0.0), wat.viscosity(20.0, 0.0), wat.surface_tension(20.0, 0.0))
SEA_15C = (wat.density(15.0, 35.0), wat.viscosity(15.0, 35.0), wat.surface_tension(15.0, 35.0))


def grace_contaminated(d, rho, mu, sigma):
    """Terminal velocity of a bubble in contaminated water: Grace correlation (Grace, Wairegi & Nguyen 1976) as
    given by Clift, Grace & Weber (1978), ch. 7. Valid for M < 1e-3, Eo < 40, Re > 0.1, H > 2."""
    m = wat.G * mu ** 4 / (rho * sigma ** 3)
    eo = wat.G * rho * d ** 2 / sigma
    h = 4 / 3 * eo * m ** -0.149 * (mu / 0.0009) ** -0.14
    j = 0.94 * h ** 0.757 if h <= 59.3 else 3.42 * h ** 0.441
    return mu / (rho * d) * m ** -0.149 * (j - 0.857)


# ----------------------------------------------------------------------------------------- slip

def rigid_sphere(d, rho, mu):
    """Terminal velocity of a weightless rigid sphere rising in water, Schiller & Naumann drag (Re < 800)."""
    u = 0.1
    for _ in range(100):
        re = rho * u * d / mu
        u = math.sqrt(4 * wat.G * d / (3 * 24 / re * (1 + 0.15 * re ** 0.687)))
    return u


def test_contaminated_reference_at_1mm():
    """Reference for a contaminated 1 mm bubble in 20 C water: 0.12 m/s (Grace et al. 1976). Such small contaminated
    bubbles behave like rigid spheres, so the standard drag curve must give nearly the same."""
    gr = grace_contaminated(1e-3, *FRESH_20C)
    assert gr == pytest.approx(0.12, abs=0.005)
    assert gr == pytest.approx(rigid_sphere(1e-3, *FRESH_20C[:2]), rel=0.10)


@pytest.mark.parametrize("water", [FRESH_20C, SEA_15C], ids=["fresh_20C", "sea_15C"])
@pytest.mark.parametrize("d_mm", [1.0, 2.0, 3.0, 5.0])
def test_fan_tsuchiya_vs_grace_correlation(water, d_mm):
    """Fan & Tsuchiya (contaminated) against the Grace correlation. Measured: +16..+20 % at 1-1.5 mm, +10 % at 2 mm,
    -8..+1 % at 2.5-6 mm (where most of the gas volume is)."""
    ft, gr = plume.slip_velocity(d_mm * 1e-3, *water), grace_contaminated(d_mm * 1e-3, *water)
    assert ft / gr == pytest.approx(1.0, abs=0.25 if d_mm < 2 else 0.12)


# ----------------------------------------------------------------------------------- pure plume

def test_pure_plume_matches_mtt_similarity():
    """No momentum, no slip, no expansion: the integral model must follow Morton-Taylor-Turner far from the source,
    u_c ~ z^-1/3 and b ~ z, with the MTT constants."""
    q, alpha = 4.25e-4, 0.08
    pl = plume.integrate_plume([0, 0, 0], [0, 0, 1], 0.0, q, 0.0, alpha, 0.004, z_stop=6.0, s_max=7.0)
    z = pl["pos"][:, 2]
    far = (z > 1.0) & (z < 5.0)
    slope_u = np.polyfit(np.log(z[far]), np.log(pl["u_c"][far]), 1)[0]
    slope_b = np.polyfit(z[far], pl["b"][far], 1)[0]
    u_mtt, _ = plume.mtt_plume(z[far], wat.G * q, alpha)
    assert slope_u == pytest.approx(-1 / 3, abs=0.005)
    assert slope_b == pytest.approx(1.2 * alpha, rel=0.01)
    assert pl["u_c"][far][z[far] > 2.0] == pytest.approx(u_mtt[z[far] > 2.0], rel=0.02)


# ---------------------------------------------------------------------------------- bubble sizes

SIZES = plume.SizeDistribution(d50=3e-3, sigma_ln=0.5, d_min=0.5e-3, d_max=10e-3)


def test_sampled_sizes_follow_the_truncated_distribution():
    d = SIZES.sample(np.random.default_rng(1), 400_000)
    assert d.min() >= SIZES.d_min and d.max() <= SIZES.d_max
    assert np.mean(math.pi / 6 * d ** 3) == pytest.approx(SIZES.mean_volume, rel=0.01)


def test_volume_weighted_quadrature_matches_the_closed_form():
    # E_vol[1/V] = 1 / E[V]
    inv = SIZES.volume_weighted_mean(lambda d: 1.0 / (math.pi / 6 * d ** 3))
    assert inv == pytest.approx(1.0 / SIZES.mean_volume, rel=1e-3)


def test_emitter_reproduces_the_volume_flux():
    """Default leak (25.5 L/min) over 60 s of 240 Hz substeps: emitted volume and bubble rate within 2 %."""
    q, dt, steps = 4.245e-4, 1 / 240, 240 * 60
    em = plume.Emitter(q, SIZES, np.random.default_rng(2))
    d = np.concatenate([em.emit(dt) for _ in range(steps)])
    t = dt * steps
    assert np.sum(math.pi / 6 * d ** 3) == pytest.approx(q * t, rel=0.02)
    assert len(d) / t == pytest.approx(q / SIZES.mean_volume, rel=0.02)
