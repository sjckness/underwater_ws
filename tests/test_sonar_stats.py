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


def test_speckle_correlation_ignores_the_envelope():
    """Fully developed speckle on a strong range trend: renewed speckle gives ~0 (the raw correlation would be high,
    driven by the trend), the same speckle 1."""
    rng = np.random.default_rng(5)
    trend = np.outer(np.exp(-np.linspace(0, 6, 400)), np.ones(200))
    a = trend * rng.exponential(size=trend.shape)
    b = trend * rng.exponential(size=trend.shape)
    mask = np.ones(trend.shape, bool)
    assert np.corrcoef(a.ravel(), b.ravel())[0, 1] > 0.3
    assert abs(ss.speckle_correlation(a, b, mask)) < 0.05
    assert ss.speckle_correlation(a, 3 * a, mask) == pytest.approx(1.0)
    box = ss.local_mean(np.arange(20.0).reshape(4, 5), (1, 3))
    assert box[1, 2] == pytest.approx(7.0) and box.shape == (4, 5)


def test_profile_distance_ignores_level_and_sampling():
    """Two recordings of the same shape, at different absolute levels and on different range axes, must read
    as identical: that is the whole point of comparing a sim in physical dB against a display image."""
    r = np.linspace(1.0, 8.0, 300)
    a = 100 - 30 * np.log10(r)
    r2 = np.linspace(0.5, 9.0, 117)
    d = ss.profile_distance(r, a, r2, 40 - 30 * np.log10(r2), (2.0, 7.0))
    assert d["rms_db"] == pytest.approx(0.0, abs=0.02)
    assert d["covered"] > 100


def test_profile_distance_sees_a_different_shape():
    r = np.linspace(1.0, 8.0, 300)
    d = ss.profile_distance(r, 100 - 30 * np.log10(r), r, 40 - 50 * np.log10(r), (2.0, 7.0))
    assert d["rms_db"] > 2.0


def test_profile_distance_is_nan_without_overlap():
    """Better than a confident 0: no overlap means no evidence, not agreement."""
    r = np.linspace(1.0, 8.0, 50)
    d = ss.profile_distance(r, 100 - 30 * np.log10(r), np.array([1.0, 2.0]), np.array([np.nan, np.nan]), (2.0, 7.0))
    assert math.isnan(d["rms_db"]) and d["covered"] == 0


def test_tvg_fit_constants_are_collinear_and_so_not_comparable():
    """Pins the reason profile_distance exists. Two very different (k, b) pairs describe almost the same curve
    over a realistic window, so the constants cannot carry a sim-vs-real comparison."""
    r = np.arange(450, 850) * 0.004                      # 1.8-3.4 m, the tank's comparison window
    design = np.stack([np.ones_like(r), np.log10(r), r], 1)
    assert np.corrcoef(np.log10(r), r)[0, 1] > 0.99
    assert np.linalg.cond(design) > 100
    # The two fits measured on the same tank scene, before and after this pass: k = -158.9 and k = +82.9.
    curve_a = 10.0 - 158.9 * np.log10(r) + 28.74 * r
    curve_b = 10.0 + 82.9 * np.log10(r) - 17.25 * r
    shape = (curve_a - curve_a.mean()) - (curve_b - curve_b.mean())
    rms = float(np.sqrt(np.mean(shape ** 2)))
    span = float(max(curve_a.max() - curve_a.min(), curve_b.max() - curve_b.min()))
    assert span < 6.0                                    # each curve only spans ~5 dB over this window
    assert rms < 3.0                                     # and they differ in shape by less than that span
    # 241.8 dB/decade of apparent disagreement for 2.7 dB of real one: the constant amplifies by ~90x.
    assert (158.9 + 82.9) / rms > 50.0
