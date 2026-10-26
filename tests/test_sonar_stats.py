"""Image statistics for the real-data comparison (host, numpy): checked on synthetic images with known answers."""
import math

import numpy as np
import pytest

from uw import sonar_stats as ss

RNG = np.random.default_rng(7)
RANGE = np.arange(800) * 0.004
BEARING = np.linspace(-65, 65, 512)


def test_rayleigh_speckle_is_recognised():
    inten = RNG.exponential(size=(800, 512)) * (1 + 50 * np.maximum(RANGE, 0.1)[:, None] ** -3)   # speckle on a trend
    r = ss.background(10 * np.log10(inten), RANGE, BEARING, (1.0, 2.5), (-25, 25))
    assert r["contrast"] == pytest.approx(1.0, abs=0.05)
    assert r["k_shape"] > 20
    assert r["ks_vs_rayleigh"] < 0.02


def test_k_shape_is_recovered():
    nu = 2.0
    inten = RNG.gamma(nu, 1 / nu, 400_000) * RNG.exponential(size=400_000)          # K-distributed intensity
    assert ss.k_shape(inten) == pytest.approx(nu, rel=0.15)


def test_lobe_width_and_sidelobe():
    x = 0.886 * BEARING / 1.0                                                       # sinc^2 with 1 deg -3 dB width
    db = 10 * np.log10(np.sinc(x) ** 2 + 1e-12)
    width, side = ss.lobe(BEARING, db, int(np.argmax(db)))
    assert width == pytest.approx(1.0, rel=0.05)
    assert side == pytest.approx(-13.3, abs=0.3)                                    # first sidelobe of a sinc


def test_blind_zone_and_tvg_fit():
    level = 150 - 30 * np.log10(np.maximum(RANGE, 1e-3)) - 0.5 * RANGE
    db = np.repeat(level[:, None], 512, 1) + RNG.normal(0, 0.1, (800, 512))
    db[:25] = np.nan                                                                # 0.1 m blind zone
    assert ss.near_range(db, RANGE)["blind_zone_m"] == pytest.approx(0.1)
    fit = ss.tvg_profile(db, RANGE, (0.5, 3.0))
    assert fit["k_db_per_decade"] == pytest.approx(-30, abs=0.3) and fit["b_db_per_m"] == pytest.approx(-0.5, abs=0.1)
    assert math.isfinite(ss.near_range(db, RANGE)["ringdown_slope_db_per_m"])
