"""Sonar model on the GPU: point target, beam pattern (far and near field), speckle, surface ghost, blind zone.

Runs in the container (python -m uw test); skipped where Warp is not importable (host pytest).
"""
import math

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")

from uw import config, geometry, sonar  # noqa: E402

QUIET = ["sonar.receiver_noise=false", "sonar.reverberation=false", "sonar.ringdown=false",
         "sonar.speckle_sigma_rad=0", "sonar.multipath=[]"]


def target(son, r, bearing_deg, elevation_deg=0.0):
    """World position at range r, bearing (+ starboard) and elevation in the sonar frame."""
    th, el = math.radians(bearing_deg), math.radians(elevation_deg)
    body = np.array([math.cos(el) * math.cos(th), -math.cos(el) * math.sin(th), math.sin(el)])
    return son.position + r * (son.rot @ body)


def row_width(bearing, db, i):
    """-3 dB width [deg] of the lobe peaking at beam i (linear interpolation of the crossings), and the highest
    sidelobe beyond the first nulls [dB re peak]."""
    rel = db - db[i]
    lo = i
    while lo > 0 and rel[lo - 1] > -3:
        lo -= 1
    hi = i
    while hi < len(rel) - 1 and rel[hi + 1] > -3:
        hi += 1
    left = np.interp(-3, [rel[lo - 1], rel[lo]], [bearing[lo - 1], bearing[lo]])
    right = np.interp(-3, [rel[hi + 1], rel[hi]], [bearing[hi + 1], bearing[hi]])
    a, b = lo, hi
    while a > 0 and rel[a - 1] < rel[a]:
        a -= 1
    while b < len(rel) - 1 and rel[b + 1] < rel[b]:
        b += 1
    side = max(rel[:a].max() if a > 0 else -99, rel[b + 1:].max() if b < len(rel) - 1 else -99)
    return right - left, side


def test_point_target_range_level_and_bearing():
    cfg = config.load(overrides=QUIET)
    son = sonar.Sonar(cfg)
    r, beam, ts = 2.0, 300, -30.0
    p = son.ping(targets=[(target(son, r, son.bearing[beam]), 10 ** (ts / 10))])
    j, i = np.unravel_index(np.nanargmax(p.db), p.db.shape)
    assert (j, i) == (round(r / son.dr), beam)
    son_cfg = config.get(cfg, "sonar")
    fan_n = son_cfg["fan_edge_db"] / (40 * math.log10(math.cos(son.theta_max)))
    expected = (son_cfg["source_level_db"] - 40 * math.log10(r) - 2 * son.alpha_db * r + ts
                + 40 * fan_n * math.log10(math.cos(math.radians(son.bearing[beam]))))
    assert p.db[j, i] == pytest.approx(expected, abs=0.5)


def test_far_field_beam_pattern():
    """30 m, four times the Fraunhofer distance, unfocused: -3 dB width 1 deg, first sidelobe -28 dB (Taylor)."""
    cfg = config.load(overrides=QUIET + ["sonar.range=40", "sonar.modes.lf.range_resolution=0.02"])
    son = sonar.Sonar(cfg)
    beam = 256
    p = son.ping(targets=[(target(son, 30.0, son.bearing[beam]), 1.0)], focus=False)
    j = round(30.0 / son.dr)
    width, side = row_width(son.bearing, p.db[j], beam)
    assert width == pytest.approx(1.0, rel=0.10)
    assert side == pytest.approx(-28.0, abs=1.5)


def _fresnel_width(son, r, bearing_deg):
    """Beam pattern of the unfocused array for a point at range r from the exact element-to-point distances
    (spherical wavefront), sampled at the sonar's beams: the Fresnel-theory prediction."""
    th = math.radians(bearing_deg)
    xs, w, k = son.xs, son.w, son.k
    d = np.sqrt(r ** 2 + xs ** 2 - 2 * r * xs * math.sin(th))          # element m at (0, x_m) along the array
    ub = np.sin(np.radians(son.bearing))
    b = np.abs(np.exp(1j * k * (np.outer(ub, xs) + (d - r)[None, :])) @ w) ** 2
    return row_width(son.bearing, 10 * np.log10(b), int(np.argmax(b)))[0]


def test_near_field_focus():
    """At 1 m (Fraunhofer distance 7 m): focused beams stay ~1 deg; unfocused ones broaden as Fresnel theory says."""
    cfg = config.load(overrides=QUIET)
    son = sonar.Sonar(cfg)
    beam, r = 256, 1.0
    pos = target(son, r, son.bearing[beam])
    j = round(r / son.dr)
    focused = row_width(son.bearing, son.ping(targets=[(pos, 1.0)], focus=True).db[j], beam)[0]
    unfocused_db = son.ping(targets=[(pos, 1.0)], focus=False).db[j]
    unfocused = row_width(son.bearing, unfocused_db, int(np.argmax(unfocused_db)))[0]
    predicted = _fresnel_width(son, r, son.bearing[beam])
    assert focused == pytest.approx(1.0, rel=0.15)
    assert predicted > 2 * focused                                       # the effect is large at 1 m
    assert unfocused == pytest.approx(predicted, rel=0.10)


@pytest.fixture(scope="module")
def flat_tank(tmp_path_factory):
    """Flat sand (no ripples, relief or rocks): speckle should be fully developed (Rayleigh)."""
    over = QUIET + ["sand.ripple_amp=0", "sand.undulation_amp=0", "rocks.count=0"]
    return over, tmp_path_factory.mktemp("cache")


@pytest.mark.parametrize("spacing, preset, rwin, bmax", [
    ("fixed", "tank", (2.15, 2.8), 25),          # sand beyond the pipe and its shadow; side walls from 32 deg
    ("adaptive", "tank", (2.15, 2.8), 25),
    ("adaptive", "basin", (4.0, 6.0), 20),       # where adaptive spacing is coarsest (about 6 mm, 9 per cell)
])
def test_sand_speckle_is_rayleigh(flat_tank, spacing, preset, rwin, bmax):
    over, cache = flat_tank
    cfg = config.load(overrides=over + [f"sonar.scatterer_spacing_mode={spacing}"], presets=[preset])
    scene = geometry.build_scene(cfg)
    son = sonar.Sonar(cfg, scene, geometry.scatterers(cfg, scene, cache_dir=str(cache)))
    p = son.ping()
    rows = (son.range_m > rwin[0]) & (son.range_m < rwin[1])
    cols = np.abs(son.bearing) < bmax
    inten = p.power[rows][:, cols]
    k = np.ones(25) / 25                                                 # local mean: 25 bins x 25 beams
    mean = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 0, inten)
    mean = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 1, mean)
    norm = (inten / mean)[12:-12, 12:-12]
    assert norm.std() / norm.mean() == pytest.approx(1.0, abs=0.12)


def test_surface_ghost_at_the_image_range():
    """Sonar 0.3 m below a calm surface, level; a point 2 m ahead. Ghosts: TX or RX via the surface at
    (|SX| + |S'X|) / 2 (the two add coherently), both via the surface at |S'X|; levels from the image geometry and
    the vertical sinc pattern."""
    h, r = 0.3, 2.0
    cfg = config.load(overrides=["sonar.receiver_noise=false", "sonar.reverberation=false", "sonar.ringdown=false",
                                 "sonar.speckle_sigma_rad=0", "sonar.multipath=[surface]",
                                 "sonar.surface_roughness_m=0", "sonar.rpy_deg=[0, 0, 0]",
                                 f"sonar.position=[0.5, 1.5, {2.9 - h}]"])
    son = sonar.Sonar(cfg)
    beam = 256
    p = son.ping(targets=[(target(son, r, son.bearing[beam]), 1e-3)])
    col = p.db[:, beam]
    lm = math.hypot(r, 2 * h)
    son_cfg, md = config.get(cfg, "sonar"), config.sonar_mode(cfg)
    from scipy.optimize import brentq
    x_e = brentq(lambda x: (math.sin(x) / x) ** 4 - 10 ** (son_cfg["vertical_edge_db"] / 10), 1e-6, math.pi)
    x = x_e / math.sin(math.radians(md["fov_v_deg"] / 2)) * (2 * h / lm)   # sin(elevation of the image) = 2h / lm
    v = math.sin(x) / x
    loss = 2 * son.alpha_db
    first = 20 * math.log10(2 * v * r / lm) - loss * (lm - r) / 2
    both = 20 * math.log10(v * v * r * r / lm ** 2) - loss * (lm - r)
    j0, j1, j2 = round(r / son.dr), round((r + lm) / 2 / son.dr), round(lm / son.dr)
    peak = lambda j: max(range(j - 1, j + 2), key=lambda q: col[q])      # noqa: E731
    assert peak(j1) == j1 and peak(j2) == j2
    assert col[j1] - col[j0] == pytest.approx(first, abs=1.0)
    assert col[j2] - col[j0] == pytest.approx(both, abs=1.0)


def test_nothing_inside_the_blind_zone():
    cfg = config.load()
    son = sonar.Sonar(cfg)
    p = son.ping(targets=[(target(son, 0.05, 0.0), 1.0)])
    n = math.ceil(config.sonar_mode(cfg)["min_range"] / son.dr)
    assert np.all(p.image[:n] == 0) and np.all(p.power[:n] == 0) and np.all(np.isnan(p.db[:n]))
    assert np.all(p.power[n:] > 0)                                      # noise and ring-down beyond it


def test_bubble_cross_sections_on_the_gpu_match_the_tables():
    cfg = config.load(overrides=QUIET)
    son = sonar.Sonar(cfg)
    r = np.array([0.25e-3, 1e-3, 2.5e-3, 5e-3], np.float32)
    sig, sext = (wp.zeros(len(r), dtype=float, device=son.device) for _ in range(2))
    wp.launch(sonar.bubble_sigma_kernel, dim=len(r), device=son.device,
              inputs=[wp.array(r, dtype=float, device=son.device), wp.ones(len(r), dtype=wp.int32, device=son.device),
                      son.k, son.tab_bs, son.tab_ext, son.tables.log_ka0, son.tables.dlog_ka, sig, sext])
    t = son.tables
    assert sig.numpy() == pytest.approx(t.lookup(t.backscatter, son.k * r) * r ** 2, rel=1e-3)
    assert sext.numpy() == pytest.approx(t.lookup(t.extinction, son.k * r) * math.pi * r ** 2, rel=1e-3)


def test_other_backends_are_m6():
    with pytest.raises(NotImplementedError, match="M6"):
        sonar.Sonar(config.load(overrides=["sonar.beamformer=element_das"]))
