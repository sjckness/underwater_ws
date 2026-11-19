"""uw/trajectory.py and ScriptPose (host, pure Python + numpy): the three scripted runs in room9."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uw import config  # noqa: E402
from uw.config import ConfigError  # noqa: E402
from uw.trajectory import body_box, build  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load(profile="linus", presets=["room9"])


@pytest.fixture(scope="module")
def leak(cfg):
    from uw.geometry import build_scene
    return build_scene(cfg).leak.hole                  # read from the room9 scene, as ScriptPose does


def _samples(traj, n=2000):
    return np.array([traj.position(t) for t in np.linspace(0.0, traj.duration, n)])


def test_leak_is_where_room9_puts_it(cfg, leak):
    assert leak[0] == pytest.approx(cfg["pipe"]["x"], abs=0.2)
    assert leak[1] == pytest.approx(cfg["leak"]["position_along"], abs=1e-6)


def test_circle(cfg, leak):
    t = build(cfg, "circle", leak)
    p = _samples(t)
    r = np.hypot(p[:, 0] - leak[0], p[:, 1] - leak[1])
    assert r == pytest.approx(2.5, abs=1e-6) and np.ptp(p[:, 2]) < 1e-9
    assert p[0, 2] == pytest.approx(leak[2] + 1.0)
    assert t.duration == pytest.approx(2 * 2 * math.pi * 2.5 / 0.3)
    assert t.notes == []
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / (t.duration / (len(p) - 1))
    assert v == pytest.approx(0.3, rel=1e-3)


def test_yaw_always_at_the_leak(cfg, leak):
    t = build(cfg, "combined", leak)
    for s in np.linspace(0.0, t.duration, 97):
        pos, rot = t.body(s)
        to_leak = leak[:2] - pos[:2]
        assert rot[:2, 0] @ (to_leak / np.linalg.norm(to_leak)) == pytest.approx(1.0, abs=1e-9)
        assert rot[2, 2] == pytest.approx(1.0)                         # level


def test_updown_is_shifted_above_the_floor_and_noted(cfg, leak):
    t = build(cfg, "updown", leak)
    p = _samples(t)
    assert np.hypot(p[:, 0] - leak[0], p[:, 1] - leak[1]) == pytest.approx(2.5, abs=1e-6)
    assert np.ptp(p[:, 2]) == pytest.approx(3.0, abs=1e-3)           # amplitude +-1.5 m kept
    assert p[:, 2].min() == pytest.approx(body_box(cfg, 1.0, 0.3).lo[2], abs=1e-3)
    assert any("shifted up" in n for n in t.notes)


def test_all_inside_the_bounds_with_margins(cfg, leak):
    box = body_box(cfg, 1.0, 0.3)
    for name in ("circle", "updown", "combined"):
        p = _samples(build(cfg, name, leak))
        assert np.all(p >= box.lo - 1e-9) and np.all(p <= box.hi + 1e-9), name
        assert np.all(p[:, :2] >= 1.0) and np.all(p[:, :2] <= 9.0 - 1.0)


def test_combined_is_continuous(cfg, leak):
    t = build(cfg, "combined", leak)
    p = _samples(t, 20000)
    step = np.linalg.norm(np.diff(p, axis=0), axis=1).max()
    assert step < 0.3 * t.duration / 20000 * 3                        # no jump anywhere (transit included)
    assert t.duration > build(cfg, "circle", leak).duration + build(cfg, "updown", leak).duration


def test_too_big_circle_is_reduced_and_noted(cfg, leak, tmp_path):
    f = tmp_path / "big.yaml"
    f.write_text("kind: circle\nradius_m: 6.0\nheight_above_leak_m: 1.0\nspeed_mps: 0.3\nlaps: 1\n"
                 "start_bearing_deg: 180\n")
    t = build(cfg, str(f), leak)
    assert any("radius reduced" in n for n in t.notes)
    p = _samples(t)
    box = body_box(cfg, 1.0, 0.3)
    assert np.all(p >= box.lo - 1e-9) and np.all(p <= box.hi + 1e-9)


def test_script_pose_rides_the_mount(cfg):
    from uw.pose import ScriptPose
    c = config.apply_overrides(config.merge(cfg, {}), ["sonar.pose_source=script", "sonar.script=circle"])
    sp = ScriptPose(c)
    p_body, rb = sp.body_at(10.0)
    son = sp.at(10.0)
    off = np.asarray(c["vehicle"]["body"]["offset"], float)
    assert son.position == pytest.approx(p_body - rb @ off, abs=1e-9)
    assert son.rpy_deg[1] == pytest.approx(c["sonar"]["pitch_deg"], abs=1e-6)     # the mount pitch


def test_unknown_trajectory_lists_the_available(cfg, leak):
    with pytest.raises(ConfigError, match="circle, combined, updown"):
        build(cfg, "nope", leak)
