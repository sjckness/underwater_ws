"""Scene geometry (host, numpy): sand gradient, adaptive scatterer spacing, scatterer cache."""
import numpy as np
import pytest

from uw import config
from physics import geometry


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


# ------------------------------------------------------------------------- materials

def test_every_material_has_an_rms_slope_with_the_coherent_term_off():
    """`materials.<m>.coherent=false` (the diffuse-only comparison runs) must still leave an rms_slope: it is the
    surface's own property, and the sonar's incoherent Kirchhoff term reads it for every material."""
    diffuse = [f"materials.{m}.coherent=false" for m in ("concrete", "steel_coated", "glass", "lining")]
    for ov in ([], diffuse):
        mats = geometry.surface_materials(config.load(overrides=ov))
        for name, m in mats.items():
            assert "rms_slope" in m, f"{name} has no rms_slope (overrides={ov})"
            assert m["rms_slope"] > 0.0, name
    on, off = (geometry.surface_materials(config.load(overrides=ov)) for ov in ([], diffuse))
    for name in on:
        assert off[name]["rms_slope"] == pytest.approx(on[name]["rms_slope"]), name
    assert off["concrete"]["rms_slope"] == pytest.approx(np.sqrt(2.0) * 0.0005 / 0.01)
    assert not off["concrete"].get("coherent") and on["concrete"]["coherent"]


def test_walls_keep_their_own_roughness_without_the_coherent_term():
    """A wall's rms_height_m overrides the base material's, so forcing the walls diffuse must not silently fall
    back to the material default."""
    cfg = config.load(overrides=["materials.concrete.coherent=false", "tank.walls.x0.rms_height_m=0.004"])
    mats = geometry.surface_materials(cfg)
    assert mats["wall:x0"]["rms_height_m"] == pytest.approx(0.004)
    assert mats["wall:x0"]["rms_slope"] == pytest.approx(np.sqrt(2.0) * 0.004 / 0.01)


# --------------------------------------------------------------------------- bottom raycast (the DVL's)

def _brute_force(scene, o, d):
    """The same query with no optimisation at all: bisection on the height field, every triangle tested."""
    o, d = np.asarray(o, float), np.asarray(d, float) / np.linalg.norm(d)
    best = np.inf
    if d[2] < 0 and o[2] > scene.sand.height(o[0], o[1]):
        lo, hi = 0.0, o[2] / -d[2]
        for _ in range(80):
            m = 0.5 * (lo + hi)
            q = o + m * d
            lo, hi = (m, hi) if q[2] - float(scene.sand.height(q[0], q[1])) > 0 else (lo, m)
        p = o + 0.5 * (lo + hi) * d
        if 0.0 <= p[0] <= scene.size[0] and 0.0 <= p[1] <= scene.size[1]:
            best = 0.5 * (lo + hi)
    for m in scene.meshes:
        if m.role in scene.occluder_roles and m.role != "sand" and len(m.faces):
            t, _ = geometry._ray_triangles(o, d, m.points[m.faces].astype(float), np.inf)
            best = min(best, t)
    return best


def _beams(tilt_deg=22.5, azimuths=(45.0, 135.0, 225.0, 315.0)):
    th = np.radians(tilt_deg)
    az = np.radians(np.asarray(azimuths, float))
    return np.stack([np.sin(th) * np.cos(az), np.sin(th) * np.sin(az), np.full(len(az), -np.cos(th))], 1)


def test_ray_bottom_matches_a_brute_force_query(scene):
    """The AABB rejection and the Newton solve are optimisations: they must change nothing."""
    rng = np.random.default_rng(11)
    L, W = scene.size[:2]
    for _ in range(25):
        o = np.array([rng.uniform(0.3, L - 0.3), rng.uniform(0.3, W - 0.3), rng.uniform(0.6, 1.6)])
        dirs = _beams()
        t, _, _ = geometry.ray_bottom(scene, o, dirs)
        ref = np.array([_brute_force(scene, o, d) for d in dirs])
        both = np.isfinite(t) & np.isfinite(ref)
        assert np.array_equal(np.isfinite(t), np.isfinite(ref))
        assert np.max(np.abs(t[both] - ref[both])) < 1e-9


def test_ray_bottom_normals_are_unit_and_face_the_ray(scene):
    o = np.array([1.5, 1.5, 1.3])
    dirs = _beams()
    t, n, roles = geometry.ray_bottom(scene, o, dirs)
    got = np.isfinite(t)
    assert got.any()
    assert np.allclose(np.linalg.norm(n[got], axis=1), 1.0)
    assert np.all(np.einsum("ij,ij->i", n[got], dirs[got]) < 0)      # the surface faces the beam
    assert all(r in (None, "sand", "rock", "pipe", "flange", "support") for r in roles)


def test_ray_bottom_normal_is_the_analytic_sand_normal(scene):
    """Not the 1 cm occlusion mesh's faceted one: the incidence angle drives the DVL's beam quality."""
    o = np.array([1.5, 1.5, 1.3])
    dirs = _beams()
    t, n, roles = geometry.ray_bottom(scene, o, dirs)
    for i, role in enumerate(roles):
        if role == "sand":
            p = o + t[i] * dirs[i]
            assert np.allclose(n[i], scene.sand.normal(p[0], p[1]))


def test_ray_bottom_has_no_hit_where_there_is_no_bottom(scene):
    o = np.array([1.5, 1.5, 1.3])
    up = geometry.ray_bottom(scene, o, np.array([[0.0, 0.0, 1.0], [0.3, 0.0, 0.2]]))[0]
    assert not np.isfinite(up).any()                                 # nothing above: no surface occluder
    under = geometry.ray_bottom(scene, np.array([1.5, 1.5, 0.05]), _beams())[0]
    assert not np.isfinite(under).any()                              # inside the sand
    out = geometry.ray_bottom(scene, np.array([2.985, 1.5, 1.0]), np.array([[0.38, 0.0, -0.92]]))[0]
    assert not np.isfinite(out).any()                                # leaves the footprint; walls do not occlude


def test_ray_bottom_sees_the_same_geometry_as_the_sonar(scene):
    """Every triangle it tests comes from Scene.occluders(), minus the sand it solves analytically."""
    parts = geometry._bottom_parts(scene)
    n_tri = sum(len(t) for t in parts["tri"])
    sand = sum(len(m.faces) for m in scene.meshes if m.role == "sand")
    assert n_tri + sand == len(scene.occluders()[1])
    assert "wall" not in parts["role"]
