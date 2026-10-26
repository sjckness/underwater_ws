"""Sonar model on the GPU: point target, beam pattern (far and near field), speckle, surface ghost, blind zone.

Runs in the container (python -m uw test); skipped where Warp is not importable (host pytest).
"""
import math

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")

from uw import bubbles, config, geometry, sonar, sonar_stats  # noqa: E402

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


def test_plume_speckle_renews_every_ping_static_sand_does_not():
    """Bubbles move about 5 cm between pings (25 wavelengths): the plume's speckle is new every ping (correlation ~0).
    With speckle dynamics off, the sand's speckle stays (~1). Speckle correlation: each intensity image over its own
    local mean (sonar_stats.speckle_correlation), so the plume's slowly changing shape does not count."""
    cfg = config.load(overrides=["sonar.speckle_sigma_rad=0"])
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    son = sonar.Sonar(cfg, scene, geometry.scatterers(cfg, scene), plume=sonar.plume_region(sim))
    pings, moved = [], []
    for _ in range(6):
        before, alive = sim.pos.numpy(), sim.alive.numpy().astype(bool)
        sim.step(1.0 / config.get(cfg, "sonar.ping_rate_hz"))
        both = alive & sim.alive.numpy().astype(bool)
        moved.append(np.median(np.linalg.norm(sim.pos.numpy()[both] - before[both], axis=1)))
        pings.append(son.ping(bubbles=sim))
    assert np.mean(moved) > 10 * 2 * math.pi / son.k                      # far more than a wavelength per ping
    sand_box = np.zeros(pings[0].power.shape, bool)
    sand_box[np.ix_((son.range_m > 2.15) & (son.range_m < 2.8), np.abs(son.bearing) < 25)] = True
    plume_c, sand_c = [], []
    for a, b in zip(pings[:-1], pings[1:]):
        plume = (a.plume_share_direct > 0.9) & (b.plume_share_direct > 0.9)
        sand = sand_box & (a.plume_share_all < 1e-3) & (b.plume_share_all < 1e-3)
        assert plume.sum() > 300 and sand.sum() > 5000
        plume_c.append(sonar_stats.speckle_correlation(a.power, b.power, plume))
        sand_c.append(sonar_stats.speckle_correlation(a.power, b.power, sand))
    assert abs(np.mean(plume_c)) < 0.1
    assert np.mean(sand_c) > 0.9


def test_geometric_plume_label():
    """mask_geom marks the main-lobe cells of visible bubbles only. Synthetic bubbles: one on a beam marks its two
    range bins and the beams within half the beam width (in u); one above the vertical aperture, one under the sand
    and one behind the sonar mark nothing."""
    from types import SimpleNamespace
    cfg = config.load(overrides=QUIET)
    son = sonar.Sonar(cfg, geometry.build_scene(cfg))
    beam, r = 300, 2.0123
    pts = [target(son, r, son.bearing[beam]), target(son, 2.0, son.bearing[200], 15.0),
           target(son, 2.5, 0.0, -9.0), son.position - 0.5 * son.rot[:, 0]]
    assert pts[2][2] < 0.3                                        # under the sand
    n = len(pts)
    bub = SimpleNamespace(cap=n, pos=wp.array(np.array(pts, np.float32), dtype=wp.vec3),
                          r=wp.full(n, 1e-3, dtype=float), alive=wp.ones(n, dtype=wp.int32))
    got = son.ping(bubbles=bub).plume_mask_geom > 0
    want = np.zeros_like(got)
    u = np.sin(np.radians(son.bearing))
    want[np.ix_([int(r / son.dr), int(r / son.dr) + 1], np.abs(u - u[beam]) <= son.u_half)] = True
    assert np.array_equal(got, want)


def test_geometric_label_has_no_sidelobe_leakage():
    """Fan vertical (roll 90°), the plume column in it: the energy label spreads over every beam at the column's
    ranges (sidelobe leakage, docs/validation.md M3), the geometric one does not; nearly all of it is in the energy
    label too."""
    cfg = config.load(overrides=["sonar.roll_deg=90"])
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    son = sonar.Sonar(cfg, scene, geometry.scatterers(cfg, scene), plume=sonar.plume_region(sim))
    sim.step(0.1)
    p = son.ping(bubbles=sim)
    geom, direct = p.plume_mask_geom > 0, p.plume_mask_direct > 0
    n_b = len(son.bearing)
    assert direct.sum(1).max() > 0.9 * n_b
    assert 0 < geom.sum(1).max() < 0.6 * n_b
    assert np.mean(direct[geom]) > 0.8


def test_surface_ghost_at_the_image_range():
    """Sonar 0.3 m below a calm surface, level; a point 2 m ahead. Ghosts: TX or RX via the surface at
    (|SX| + |S'X|) / 2 (the two add coherently), both via the surface at |S'X|; levels from the image geometry and
    the vertical sinc pattern."""
    h, r = 0.3, 2.0
    cfg = config.load(overrides=["sonar.receiver_noise=false", "sonar.reverberation=false", "sonar.ringdown=false",
                                 "sonar.speckle_sigma_rad=0", "sonar.multipath=[surface]",
                                 "sonar.surface_roughness_m=0", "sonar.pitch_deg=0", "sonar.vertical_taper=uniform",
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


VERTICAL = {   # one-way patterns of a line aperture, x = pi L sin(angle) / lambda (written out again here)
    "uniform": lambda x: math.sin(x) / x,
    "cosine": lambda x: math.pi ** 2 * math.cos(x) / (math.pi ** 2 - 4 * x * x),
    "hann": lambda x: math.sin(x) / x * math.pi ** 2 / (math.pi ** 2 - x * x),
}


@pytest.mark.parametrize("taper", ["uniform", "cosine", "hann"])
def test_vertical_pattern(taper):
    """Two-way -3 dB at +-fov_v/2 for every taper; off axis the level follows the taper's closed form."""
    cfg = config.load(overrides=QUIET + [f"sonar.vertical_taper={taper}"])
    son = sonar.Sonar(cfg)
    from scipy.optimize import brentq
    f = VERTICAL[taper]
    x_e = brentq(lambda x: f(x) ** 4 - 0.5, 1e-6, 2.0)
    half = math.radians(config.sonar_mode(cfg)["fov_v_deg"] / 2)
    j = round(2.0 / son.dr)
    level = {}
    for el in (0.0, 10.0, 16.0):
        db = son.ping(targets=[(target(son, 2.0, son.bearing[256], el), 1e-3)]).db
        level[el] = np.nanmax(db[j - 1:j + 2, 250:263])
    for el in (10.0, 16.0):
        x = x_e * math.sin(math.radians(el)) / math.sin(half)
        assert level[el] - level[0] == pytest.approx(40 * math.log10(abs(f(x))), abs=0.3), el


def test_plume_labels():
    """Direct-path label inside the plume's own ranges; the all-paths label adds the surface ghosts beyond them."""
    from uw import bubbles
    cfg = config.load(overrides=["sonar.speckle_sigma_rad=0"])
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(8.0)
    son = sonar.Sonar(cfg, scene, None, plume=sonar.plume_region(sim))
    p = son.ping(bubbles=sim)
    alive = sim.snapshot()["pos"]
    r_max = np.linalg.norm(alive - son.position, axis=1).max()
    rr = son.range_m[:, None] * np.ones((1, len(son.bearing)))
    direct, every = p.plume_mask_direct > 0, p.plume_mask_all > 0
    assert direct.sum() > 100 and every.sum() > direct.sum()
    assert np.all(rr[direct] <= r_max + 0.02)
    assert np.any(rr[every] > r_max + 0.05)


def _plate(son, r, bearing_deg, tilt_deg, width, height, edge=0.0025):
    """Smooth rectangular plate (width along the array, height across it), centred at range r on the sonar's
    horizontal plane, facing the sonar, turned by `tilt_deg` about the sonar's vertical axis: coherent facets only."""
    th, t = math.radians(bearing_deg), math.radians(tilt_deg)
    los = np.array([math.cos(th), -math.sin(th), 0.0])            # line of sight, sonar frame
    across = np.array([math.sin(th + t), math.cos(th + t), 0.0])  # the plate's horizontal side, turned by the tilt
    up = np.array([0.0, 0.0, 1.0])
    corners = [r * los + a * width / 2 * across + b * height / 2 * up for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    world = [son.position + son.rot @ c for c in corners]
    tri = geometry._subdivide(np.array([[world[0], world[1], world[2]], [world[0], world[2], world[3]]]), edge)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    flip = np.einsum("ij,j->i", n, son.position - tri.mean(1).mean(0)) < 0      # face the sonar
    tri[flip] = tri[flip][:, [0, 2, 1]]
    empty = np.zeros((0, 3), np.float32)
    return geometry.Scatterers(pos=empty, normal=empty, area=np.zeros(0, np.float32), mat=np.zeros(0, np.int32),
                               phase=np.zeros(0, np.float32), material_names=["steel_coated"],
                               facets=tri.astype(np.float32), facet_mat=np.zeros(len(tri), np.int32))


def test_smooth_plate_kirchhoff_echo():
    """A smooth 1 x 2 cm steel plate at 2 m (far field from 0.4 m; 0.3 deg wide, small against the 1 deg beam): echo
    level SL - 40 log R - 2 alpha R + TS with the Kirchhoff TS = 20 log10(R0 A cos(t) / lambda) at tilt t, times
    sinc^2(k W sin t) (two-way phase across the width W): within 1 dB on the main lobe and the first sidelobe, and a
    deep null between them."""
    cfg = config.load(overrides=QUIET + ["materials.steel_coated.rms_height_m=0"])
    probe = sonar.Sonar(cfg)
    beam, r, width, height = 256, 2.0, 0.01, 0.02
    z_w = probe.rho * probe.c / 1e6
    r0 = (30.0 - z_w) / (30.0 + z_w)
    lam = 2 * math.pi / probe.k
    j = round(r / probe.dr)
    level = {}
    null = math.degrees(math.asin(math.pi / (probe.k * width)))
    side = math.degrees(math.asin(4.4934 / (probe.k * width)))      # first sidelobe peak: tan x = x
    for tilt in (0.0, 2.0, 4.0, null, side):
        son = sonar.Sonar(cfg, None, _plate(probe, r, probe.bearing[beam], tilt, width, height))
        level[tilt] = np.nanmax(son.ping().db[j - 1:j + 2, beam - 3:beam + 4])
    sl = config.get(cfg, "sonar.source_level_db")
    for tilt in (0.0, 2.0, 4.0, side):
        t = math.radians(tilt)
        x = probe.k * width * math.sin(t)
        sinc = math.sin(x) / x if x else 1.0
        expected = (sl - 40 * math.log10(r) - 2 * probe.alpha_db * r
                    + 20 * math.log10(r0 * width * height * math.cos(t) / lam * abs(sinc)))
        assert level[tilt] == pytest.approx(expected, abs=1.0), tilt
    assert level[null] < level[0.0] - 25


@pytest.mark.parametrize("roll", [0.0, 45.0, 90.0])
def test_roll_maps_a_vertical_line_of_targets(roll):
    """Point targets on a vertical line 2 m ahead appear at the range and bearing an independent rotation
    (scipy, intrinsic Z-Y-X: yaw, pitch nose-down, roll) predicts, for every roll."""
    from scipy.spatial.transform import Rotation
    cfg = config.load(overrides=QUIET + [f"sonar.roll_deg={roll}"])
    son = sonar.Sonar(cfg)
    s = config.get(cfg, "sonar")
    rot = Rotation.from_euler("ZYX", [s["yaw_deg"], s["pitch_deg"], s["roll_deg"]], degrees=True).as_matrix()
    found = 0
    for dz in np.linspace(-1.0, 0.4, 8):
        pos = son.position + np.array([2.0, 0.0, dz])
        b = rot.T @ (pos - son.position)
        r = np.linalg.norm(b)
        bearing, elevation = math.degrees(math.atan2(-b[1], b[0])), math.degrees(math.asin(b[2] / r))
        if abs(elevation) > 9 or abs(bearing) > 60:                # outside the fan
            continue
        p = son.ping(targets=[(pos, 1e-3)])
        j, i = np.unravel_index(np.nanargmax(p.db), p.db.shape)
        assert abs(j - r / son.dr) <= 1.0 and abs(son.bearing[i] - bearing) <= 0.3, (dz, bearing, son.bearing[i])
        found += 1
    assert found >= 3


def test_nothing_inside_the_blind_zone():
    cfg = config.load()
    son = sonar.Sonar(cfg)
    p = son.ping(targets=[(target(son, 0.05, 0.0), 1.0)])
    n = math.ceil(config.sonar_mode(cfg)["min_range"] / son.dr)
    assert np.all(p.image[:n] == 0) and np.all(p.power[:n] == 0) and np.all(np.isnan(p.db[:n]))
    assert np.all(p.power[n:] > 0)                                      # noise and ring-down beyond it


@pytest.mark.parametrize("focus", [False, True])
def test_common_mode_noise_is_a_stripe_at_broadside(focus):
    """Noise common to every element adds share |sum_m w_m exp(i phase_m)|^2 / sum w^2 to the beam's noise: all of
    (sum w)^2 at broadside unfocused, less at near range when the array focuses, sidelobe level off broadside."""
    quiet = ["sonar.reverberation=false", "sonar.ringdown=false", "sonar.speckle_sigma_rad=0", "sonar.multipath=[]"]
    share_db = -10.0
    off, on = (sonar.Sonar(config.load(overrides=quiet + [f"sonar.receiver_noise_common_db={c}"]))
               for c in ("null", share_db))
    p_off = np.mean([off.ping(focus=focus).power for _ in range(6)], axis=0)    # the same seeds: paired draws
    p_on = np.mean([on.ping(focus=focus).power for _ in range(6)], axis=0)
    u = on.ub.numpy().astype(float)
    r = np.maximum(np.arange(on.n_r) * on.dr, on.dr)
    n = math.ceil(config.sonar_mode(config.load())["min_range"] / on.dr)
    excess = {}
    for band, rows in (("near", slice(n, n + int(0.4 / on.dr))), ("far", slice(int(0.7 * on.n_r), on.n_r))):
        for beams, cols in (("broadside", np.abs(on.bearing) < 0.5), ("side", np.abs(np.abs(on.bearing) - 10) < 2)):
            rr, uu = r[rows, None, None], u[None, cols, None]
            fresnel = 0.0 if focus else on.k * (1 - uu ** 2) / (2 * rr)
            a = np.sum(on.w * np.exp(1j * (on.k * on.xs * uu + (fresnel - on.k / (2 * rr)) * on.xs ** 2)), axis=-1)
            expected = 10 * np.log10(1 + 10 ** (share_db / 10) * np.mean(np.abs(a) ** 2) / np.sum(on.w ** 2))
            got = 10 * np.log10(np.mean(p_on[rows][:, cols]) / np.mean(p_off[rows][:, cols]))
            assert got == pytest.approx(expected, abs=0.3), (band, beams, got, expected)
            excess[band, beams] = got
    coherent = 10 * np.log10(1 + 10 ** (share_db / 10) * np.sum(on.w) ** 2 / np.sum(on.w ** 2))
    assert excess["far", "broadside"] > coherent - 2.0 and excess["far", "side"] < 0.3     # +-0.5 deg: main lobe
    if focus:
        assert excess["near", "broadside"] < excess["far", "broadside"] - 3
    else:
        assert excess["near", "broadside"] == pytest.approx(excess["far", "broadside"], abs=0.5)


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
