"""Scene geometry (host, numpy): sand gradient, adaptive scatterer spacing, scatterer cache."""
import numpy as np
import pytest

from uw import config, geometry


@pytest.fixture(scope="module", params=["exposed", "buried"])
def scene(request):
    """Exposed: sand mound against the pipe; buried: pockmarks over the vents."""
    return geometry.build_scene(config.load(presets=[request.param]))


def test_sand_gradient_matches_finite_differences(scene):
    rng = np.random.default_rng(3)
    L, W = scene.size[:2]
    x, y = rng.uniform(0.05, L - 0.05, 20000), rng.uniform(0.05, W - 0.05, 20000)
    if scene.sand.pockmarks:            # make sure the pockmark bowls and rims are hit
        px, py, rad, _ = np.array(scene.sand.pockmarks).T
        k = rng.integers(0, len(px), 5000)
        x[:5000], y[:5000] = px[k] + rng.uniform(-1.6, 1.6, 5000) * rad[k], py[k] + rng.uniform(-1.6, 1.6, 5000) * rad[k]
    _, g = scene.sand.height_grad(x, y)
    e = 1e-6
    fx = (scene.sand.height(x + e, y) - scene.sand.height(x - e, y)) / (2 * e)
    fy = (scene.sand.height(x, y + e) - scene.sand.height(x, y - e)) / (2 * e)
    assert np.max(np.abs(g[:, 0] - fx)) < 1e-7 and np.max(np.abs(g[:, 1] - fy)) < 1e-7


# ------------------------------------------------------------------------- adaptive spacing

def _visible_energy(sc, cfg):
    """Expected Lambert backscatter inside the sonar's fan and range: sum of area mu cos^2(incidence) / R^4."""
    son = config.sonar_mode(cfg)
    rel = sc.pos.astype(float) - np.asarray(son["position"], float)
    r = np.linalg.norm(rel, axis=1)
    body = rel @ geometry.sonar_rotation(cfg)                  # world -> sonar frame (x forward, z up)
    az = np.degrees(np.arctan2(body[:, 1], body[:, 0]))
    el = np.degrees(np.arctan2(body[:, 2], np.hypot(body[:, 0], body[:, 1])))
    cos_i = -np.einsum("ij,ij->i", sc.normal.astype(float), rel) / r
    mats = geometry.surface_materials(cfg)
    mu = np.array([10 ** (mats[n]["lambert_db"] / 10) for n in sc.material_names])[sc.mat]
    seen = (np.abs(az) <= son["fov_h_deg"] / 2) & (np.abs(el) <= son["fov_v_deg"] / 2) & (r <= son["range"]) & (cos_i > 0)
    return float(np.sum((sc.area * mu * cos_i ** 2 / r ** 4)[seen]))


def test_adaptive_spacing_keeps_area_and_energy():
    """Coarse settings so that the test is quick and the adaptive rule coarsens most of the tank (from 1.4 m):
    8 mm base, 6 deg beams. Per material area and visible backscattered energy agree within 1 %."""
    cfg = config.load()
    scene = geometry.build_scene(cfg)
    fixed = geometry.Spacing(base=0.008, wall_min=0.008)
    adaptive = geometry.Spacing(base=0.008, wall_min=0.008, adaptive=True,
                                sonar=tuple(config.get(cfg, "sonar.position")), dtheta=np.radians(6.0), dr=0.004)
    a = geometry.sample_scatterers(scene, fixed, np.random.default_rng(4))
    b = geometry.sample_scatterers(scene, adaptive, np.random.default_rng(5))
    assert len(b.pos) < 0.7 * len(a.pos)
    assert a.material_names == b.material_names
    for i, name in enumerate(a.material_names):
        assert b.area[b.mat == i].sum() == pytest.approx(a.area[a.mat == i].sum(), rel=0.01), name
    assert _visible_energy(b, cfg) == pytest.approx(_visible_energy(a, cfg), rel=0.01)


def test_adaptive_spacing_rule():
    s = geometry.Spacing(base=0.002, wall_min=0.004, adaptive=True, sonar=(0.0, 0.0, 0.0),
                         dtheta=np.radians(1.0), dr=0.004)
    p = np.array([[0.5, 0, 0], [8.0, 0, 0]])
    assert s.at(p) == pytest.approx([0.002, np.sqrt(8.0 * np.radians(1.0) * 0.004 / 9)])   # 2 mm floor, 7.9 mm at 8 m
    assert s.at(p, wall=True) == pytest.approx([0.004, np.sqrt(8.0 * np.radians(1.0) * 0.004 / 9)])


def test_scatterer_cache(tmp_path):
    cfg = config.load(overrides=["sonar.scatterer_spacing=0.012"])
    first = geometry.scatterers(cfg, cache_dir=str(tmp_path))
    assert len(list(tmp_path.glob("scatterers_*.npz"))) == 1
    again = geometry.scatterers(cfg, cache_dir=str(tmp_path))
    assert np.array_equal(first.pos, again.pos) and first.material_names == again.material_names
    geometry.scatterers(config.load(overrides=["sonar.scatterer_spacing=0.012", "sonar.scatterer_spacing_mode=fixed"]),
                        cache_dir=str(tmp_path))
    assert len(list(tmp_path.glob("scatterers_*.npz"))) == 2
