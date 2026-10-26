"""Mobile sonar (SensorPose trajectory, StaticCache none / on_motion) on the GPU: detections stay put in the world,
a moved sonar equals one built at that pose, on_motion recomputes only beyond its thresholds.

Runs in the container (python -m uw test); skipped where Warp is not importable (host pytest).
"""
import math

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")

from uw import config, geometry, pose, recorder, sonar  # noqa: E402

QUIET = ["sonar.receiver_noise=false", "sonar.reverberation=false", "sonar.ringdown=false",
         "sonar.speckle_sigma_rad=0", "sonar.multipath=[]"]


def test_point_target_stays_at_its_world_position():
    """The sonar moves 0.6 m and turns 50 deg in yaw at the target's height (pitch 0, so the target stays at
    elevation 0): each ping's brightest pixel, taken back to the world with that ping's pose, lands on the target."""
    target = np.array([2.2, 1.6, 1.0])
    traj = pose.Trajectory([{"position": [0.5, 1.2, 1.0], "yaw_deg": 10.0, "speed": 0.2},
                            {"position": [1.1, 1.5, 1.0], "yaw_deg": -40.0}])
    cfg = config.load(overrides=QUIET + ["sonar.pose_source=trajectory", "sonar.static_cache=none",
                                         "sonar.pitch_deg=0"])
    son = sonar.Sonar(cfg)
    for x in np.linspace(0.0, traj.duration, 8):
        q = traj.at(x)
        son.set_pose(q.position, q.rot)
        p = son.ping(targets=[(target, 1e-2)])
        j, i = np.unravel_index(np.nanargmax(p.db), p.db.shape)
        r, b = son.range_m[j], math.radians(son.bearing[i])
        found = p.position + p.rot @ np.array([r * math.cos(b), -r * math.sin(b), 0.0])
        rng, brg, el = recorder.sonar_frame(target, p.position, p.rot)[0]
        assert abs(el) < 0.5
        assert np.linalg.norm(found - target) < 2 * son.dr + rng * math.radians(son.bearing[1] - son.bearing[0])


@pytest.fixture(scope="module")
def tank():
    cfg = config.load(overrides=["sonar.speckle_sigma_rad=0", "sonar.receiver_noise=false",
                                 "sonar.scatterer_spacing=0.006", "sonar.scatterer_spacing_mode=fixed"])
    scene = geometry.build_scene(cfg)
    return cfg, scene, geometry.scatterers(cfg, scene)


def test_a_moved_sonar_equals_one_built_there(tank):
    """set_pose + recompute (static_cache none) gives bit-identically the ping of a sonar configured at that pose."""
    cfg, scene, sc = tank
    moving = sonar.Sonar(config.merge(cfg, {"sonar": {"pose_source": "trajectory", "static_cache": "none"}}),
                         scene, sc)
    target = pose.Pose(np.array([0.4, 1.3, 1.2]), np.array([5.0, 25.0, -15.0]))
    there = sonar.Sonar(config.merge(cfg, {"sonar": {"position": target.position.tolist(), "roll_deg": 5.0,
                                                     "pitch_deg": 25.0, "yaw_deg": -15.0}}), scene, sc)
    moving.set_pose(target.position, target.rot)
    a, b = moving.ping(), there.ping()
    assert "static_recompute" in a.timing_ms
    assert np.array_equal(a.power, b.power) and np.array_equal(a.image, b.image)


def test_on_motion_recomputes_only_beyond_the_thresholds(tank):
    cfg, scene, sc = tank
    son = sonar.Sonar(config.merge(cfg, {"sonar": {"pose_source": "trajectory", "static_cache": "on_motion",
                                                   "static_cache_motion": {"position_m": 0.005, "angle_deg": 0.25}}}),
                      scene, sc)
    p0, rot0 = son.position.copy(), son.rot.copy()
    son.set_pose(p0 + [0.003, 0.0, 0.0], rot0)
    assert "static_recompute" not in son.ping().timing_ms
    son.set_pose(p0 + [0.003, 0.0, 0.0], geometry.rot_rpy_deg([0.0, 15.2, 0.0]))
    assert "static_recompute" not in son.ping().timing_ms         # 0.2 deg
    son.set_pose(p0 + [0.007, 0.0, 0.0], rot0)
    assert "static_recompute" in son.ping().timing_ms
    son.set_pose(p0 + [0.009, 0.0, 0.0], rot0)                    # 2 mm from the last recompute
    assert "static_recompute" not in son.ping().timing_ms
    with pytest.raises(config.ConfigError):
        sonar.Sonar(cfg, scene, sc).set_pose(p0 + [0.01, 0.0, 0.0], rot0)       # precomputed cannot move
