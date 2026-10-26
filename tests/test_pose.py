"""Sensor poses (uw/pose.py, host): trajectories inside the tank, smooth, timed by their speed, looking at the leak."""
import math

import numpy as np
import pytest

from uw import config, geometry, pose

PRESETS = ["approach", "orbit", "lawnmower", "hover"]


def _traj(preset, *extra):
    cfg = config.load(overrides=["sonar.pose_source=trajectory", "sonar.static_cache=none",
                                 f"sonar.trajectory.preset={preset}", *extra])
    scene = geometry.build_scene(cfg)
    return cfg, scene, pose.sensor_pose(cfg, scene)


@pytest.mark.parametrize("preset", PRESETS)
def test_presets_stay_in_the_water_and_move_smoothly(preset):
    cfg, scene, sp = _traj(preset)
    b = pose.Bounds(scene, float(config.get(cfg, "sonar.trajectory.margin")))
    dur = getattr(sp, "duration", 10.0)
    t = np.linspace(0.0, dur, 2001)
    p = np.array([sp.at(x).position for x in t])
    assert np.all(p >= b.lo - 1e-9) and np.all(p <= b.hi + 1e-9)
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.diff(t)
    speed = float(config.get(cfg, "sonar.trajectory.speed"))
    assert v.max() < 2.0 * speed                                     # no jumps
    if preset != "hover":
        assert np.mean(v) == pytest.approx(speed, rel=0.2)
    if preset != "hover":                                           # (hover: a random walk at the ping rate)
        acc = np.diff(v) / np.diff(t[1:])
        assert np.percentile(np.abs(acc), 99) < 5.0                 # smooth (m/s^2)


@pytest.mark.parametrize("preset", ["approach", "orbit"])
def test_tracking_presets_look_at_the_leak(preset):
    _, scene, sp = _traj(preset)
    leak = pose.leak_target(scene)
    for x in np.linspace(0.0, sp.duration, 7):
        q = sp.at(x)
        to = leak - q.position
        assert np.degrees(math.acos(np.dot(q.rot[:, 0], to / np.linalg.norm(to)))) < 3.0


def test_approach_closes_in_and_orbit_goes_around():
    _, scene, sp = _traj("approach")
    leak = pose.leak_target(scene)
    d = [np.linalg.norm((sp.at(x).position - leak)[:2]) for x in np.linspace(0, sp.duration, 20)]
    assert np.all(np.diff(d) < 1e-9) and d[0] - d[-1] > 0.5
    _, scene, sp = _traj("orbit")
    rel = np.array([sp.at(x).position - leak for x in np.linspace(0, sp.duration, 200)])
    ang = np.unwrap(np.arctan2(rel[:, 1], rel[:, 0]))
    assert abs(ang[-1] - ang[0]) == pytest.approx(2 * math.pi, rel=0.02)
    assert np.allclose(np.linalg.norm(rel[:, :2], axis=1), 1.0, atol=0.03)


def test_waypoints_and_rotation_roundtrip():
    sp = pose.Trajectory([{"position": [0.5, 1.0, 1.0], "yaw_deg": 170.0, "speed": 0.1},
                          {"position": [0.5, 1.5, 1.0], "yaw_deg": -170.0, "pitch_deg": 20.0}])
    assert sp.duration == pytest.approx(5.0)
    mid = sp.at(2.5)
    assert abs(mid.rpy_deg[2]) == pytest.approx(180.0, abs=1.0)    # through 180, not through 0
    rpy = np.array([12.0, 34.0, -56.0])
    assert np.allclose(geometry.rpy_deg_of(geometry.rot_rpy_deg(rpy)), rpy)


def test_hover_is_deterministic_and_drifts_a_little():
    cfg, scene, a = _traj("hover")
    _, _, b = _traj("hover")
    assert np.array_equal(a.at(3.3).position, b.at(3.3).position)
    p = np.array([a.at(x).position for x in np.linspace(0, 30, 300)])
    assert 0.003 < p.std(0).max() < 0.06


def test_adaptive_spacing_follows_the_trajectory():
    """Fine sampling where the sonar comes close: the spacing at a point is set by its nearest trajectory point."""
    cfg, scene, sp = _traj("approach")
    s = geometry.spacing_from_config(cfg, scene)
    end = sp.at(sp.duration).position
    fixed = geometry.spacing_from_config(config.load())
    probe = np.array([end + [0.3, 0.0, -0.5]])
    assert s.at(probe)[0] <= fixed.at(probe)[0]
    assert s.at(probe)[0] == pytest.approx(geometry.Spacing(**{**s.__dict__, "sonar": tuple(end)}).at(probe)[0],
                                           rel=1e-3)


@pytest.mark.parametrize("roll", [0.0, 90.0, 35.0])
def test_aim_puts_the_target_where_asked(roll):
    """aim() gives the pitch and yaw that put a world point at a chosen bearing and elevation, for any roll (where a
    pitch can: a target too steep for its bearing is skipped here)."""
    rng = np.random.default_rng(int(roll))
    done = 0
    for _ in range(300):
        p, t = rng.uniform(0, 3, 3), rng.uniform(0, 3, 3)
        b, e = rng.uniform(-60, 60), rng.uniform(-10, 10)
        d = (t - p) / np.linalg.norm(t - p)
        v = geometry.rot_rpy_deg([roll, 0, 0]) @ np.array([math.cos(math.radians(e)) * math.cos(math.radians(b)),
                                                           -math.cos(math.radians(e)) * math.sin(math.radians(b)),
                                                           math.sin(math.radians(e))])
        if abs(d[2]) > math.hypot(v[0], v[2]) - 1e-3:
            continue
        pitch, yaw = pose.aim(p, t, roll, b, e)
        r, bb, ee = pose.in_sonar_frame(p, geometry.rot_rpy_deg([roll, pitch, yaw]), t)
        assert (bb, ee, r) == pytest.approx((b, e, np.linalg.norm(t - p)), abs=1e-9)
        done += 1
    assert done > 200
    assert pose.aim([0.1, 1.5, 1.0], [1.6, 1.5, 0.4], 0.0)[0] == pytest.approx(math.degrees(math.atan2(0.6, 1.5)))


def test_poses_hold_each_view_then_jump():
    views = [{"position": [0.2, 1.0 + 0.5 * i, 1.2], "roll_deg": 0.0, "pitch_deg": 20.0 + i, "yaw_deg": 5.0 * i}
             for i in range(3)]
    cfg = config.load(overrides=["sonar.pose_source=trajectory", "sonar.static_cache=on_motion",
                                 "sonar.trajectory.preset=poses", "sonar.trajectory.hold_s=3.0",
                                 "sonar.trajectory.poses=" + str(views).replace("'", "")])
    sp = pose.sensor_pose(cfg)
    dt = 0.1
    seen = [int(round(sp.at((i + 1) * dt).rpy_deg[1] - 20.0)) for i in range(95)]   # ping i at (i + 1) dt
    assert seen == [0] * 30 + [1] * 30 + [2] * 35                                      # the last view is kept
    assert sp.positions().shape == (3, 3) and sp.moving
