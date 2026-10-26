"""USD stage from the numpy geometry (in memory, no Kit app): one geometry source for the render and the sonar, sensor
and lamp poses, the bubble instancer.

Runs in the container (python -m uw test): needs pxr (Isaac's USD libraries) and Warp.
"""
import math

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")
pxr = pytest.importorskip("pxr", reason="needs Isaac's USD libraries: python -m uw test")

from pxr import Gf, Usd, UsdGeom, UsdLux, Vt  # noqa: E402

from uw import bubbles, config, geometry, sonar, usd_scene  # noqa: E402


def _world(stage, path):
    return np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())).T                      # column-vector convention: p_world = M @ [p, 1]


@pytest.fixture(scope="module", params=["exposed", "buried"])
def built(request):
    cfg = config.load(presets=[request.param], overrides=["sonar.roll_deg=30", "sonar.yaw_deg=-10"])
    scene = geometry.build_scene(cfg)
    son = sonar.Sonar(cfg, scene)                       # the Warp BVH mesh, no scatterers needed
    stage = Usd.Stage.CreateInMemory()
    info = usd_scene.build(stage, cfg, scene)
    return cfg, scene, son, stage, info


def test_usd_and_warp_meshes_are_identical(built):
    cfg, scene, son, stage, info = built
    n = usd_scene.check_geometry(stage, scene, son.mesh.points.numpy(), son.mesh.indices.numpy())
    assert n == len(son.mesh.points.numpy()) > 100000
    assert set(info.meshes) == {m.name for m in scene.meshes}
    # one vertex off by one float32 ulp in USD must fail the check
    pipe = UsdGeom.Mesh(stage.GetPrimAtPath(info.meshes["pipe"]))
    pts = np.asarray(pipe.GetPointsAttr().Get(), np.float32).copy()
    good = pts.copy()
    pts[7, 2] = np.nextafter(pts[7, 2], np.float32(10))
    pipe.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts))
    with pytest.raises(AssertionError):
        usd_scene.check_geometry(stage, scene, son.mesh.points.numpy(), son.mesh.indices.numpy())
    pipe.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(good))


def test_sensor_and_lamp_poses(built):
    cfg, scene, son, stage, info = built
    m = _world(stage, usd_scene.SONAR)
    assert np.allclose(m[:3, :3], son.rot, atol=1e-12) and np.allclose(m[:3, 3], son.position)
    cam = _world(stage, usd_scene.CAMERA_OPTICAL)
    body = geometry.rot_rpy_deg(config.get(cfg, "camera.rpy_deg"))
    assert np.allclose(cam[:3, 3], config.get(cfg, "camera.position"))
    assert np.allclose(-cam[:3, 2], body[:, 0])                  # the USD camera looks along -z: the body's x
    assert np.allclose(cam[:3, 1], body[:, 2])                   # its up (+y) is the body's z
    w, h = config.get(cfg, "camera.resolution")
    assert info.camera_resolution == (w, h)
    assert 2 * math.degrees(math.atan(w / 2 / info.camera_K[0, 0])) == pytest.approx(config.get(cfg, "camera.hfov_deg"))
    lamps = config.get(cfg, "lights.lamps")
    for i, lamp in enumerate(lamps):
        lm = _world(stage, f"{usd_scene.WORLD}/lights/lamp_{i}")
        aim = np.subtract(lamp["target"], lamp["position"])
        assert np.allclose(lm[:3, 3], lamp["position"])
        assert np.allclose(-lm[:3, 2], aim / np.linalg.norm(aim))   # lights shine along their -z
        shaping = UsdLux.ShapingAPI(stage.GetPrimAtPath(f"{usd_scene.WORLD}/lights/lamp_{i}"))
        assert shaping.GetShapingConeAngleAttr().Get() == pytest.approx(lamp["cone_deg"] / 2)
    assert not stage.GetPrimAtPath(f"{usd_scene.WORLD}/lights/dome")      # deep water: no dome by default
    assert stage.GetPrimAtPath(f"{usd_scene.TANK}/leak_marker_0")


def test_bubble_instancer_matches_the_pool():
    cfg = config.load(overrides=["plume.max_bubbles=20000"])
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(1.5)
    stage = Usd.Stage.CreateInMemory()
    inst = usd_scene.BubbleInstancer(stage, cfg, sim)
    timing = inst.update()
    snap = sim.snapshot()
    n = len(snap["pos"])
    assert timing["bubbles"] == n > 1000
    pi = UsdGeom.PointInstancer(stage.GetPrimAtPath(usd_scene.BUBBLES))
    pos = np.asarray(pi.GetPositionsAttr().Get())
    scale = np.asarray(pi.GetScalesAttr().Get())
    q = pi.GetOrientationsAttr().Get()
    assert len(pi.GetProtoIndicesAttr().Get()) == len(q) == n
    assert np.array_equal(pos, snap["pos"])                      # compaction keeps the pool order
    # ellipsoids of the bubble's volume, flattened along their slip (Wellek: E < 1, more for big bubbles)
    a, c = scale[:, 0], scale[:, 2]
    assert np.allclose(a * a * c, snap["radius"] ** 3, rtol=1e-4)
    e = c / a
    assert np.all(e < 1) and np.corrcoef(snap["radius"], e)[0, 1] < -0.9
    # the short (local z) axis lies along the slip direction (velocity relative to the water)
    slip = sim.vel.numpy() - sim.u_liq.numpy() - sim.turb.numpy()
    slip = slip[sim.alive.numpy().astype(bool)]
    slip /= np.linalg.norm(slip, axis=1, keepdims=True)
    z = np.array([np.array(Gf.Rotation(Gf.Quatd(qq)).TransformDir(Gf.Vec3d(0, 0, 1))) for qq in q[:500]])
    assert np.min(np.sum(z * slip[:500], 1)) > 0.999                # float16 quaternions: ~1e-3
    assert np.mean(slip[:, 2]) > 0.9                                 # mostly rising, tilted by the path oscillation
    inst2 = usd_scene.BubbleInstancer(Usd.Stage.CreateInMemory(), cfg, sim, stride=4)
    assert inst2.compact() == int(np.sum(sim.alive.numpy()[::4]))
