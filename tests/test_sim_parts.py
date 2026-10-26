"""Pieces of `python -m uw sim` that need neither Kit nor a GPU: loop cadence, frames, the GUI overlay geometry."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from uw import config, geometry, gui, main, ros_bridge


def _sonar(cfg):
    """The attributes of uw.sonar.Sonar that the overlay and the frames use."""
    md = config.sonar_mode(cfg)
    n_r = int(round(float(config.get(cfg, "sonar.range")) / md["range_resolution"])) + 1
    return SimpleNamespace(position=np.asarray(config.get(cfg, "sonar.position"), float),
                           rot=geometry.sonar_rotation(cfg), theta_max=math.radians(md["fov_h_deg"] / 2),
                           fov_v_deg=md["fov_v_deg"], range=float(config.get(cfg, "sonar.range")),
                           range_m=np.arange(n_r) * md["range_resolution"],
                           bearing=np.linspace(-md["fov_h_deg"] / 2, md["fov_h_deg"] / 2, md["beams"]))


def test_rate_timers_fire_on_exact_ticks():
    """The sonar pings after every 24 bubble substeps (1/240 s ticks, 10 Hz), as sonar-only steps 0.1 s per ping,
    for an hour of sim time; the 30 fps GUI every 8 ticks."""
    dt = float(config.get(config.load(), "plume.dt"))
    ping, gui_t = main.RateTimer(10.0), main.RateTimer(30.0)
    fired = [k for k in range(1, 240 * 3600 + 1) if ping.due(k * dt)]
    assert fired == list(range(24, 240 * 3600 + 1, 24))
    assert [k for k in range(1, 241) if gui_t.due(k * dt)] == list(range(8, 241, 8))


@pytest.mark.parametrize("roll", [0.0, 90.0])
def test_fan_wireframe_bounds_the_field_of_view(roll):
    cfg = config.load(overrides=[f"sonar.roll_deg={roll}"])
    son = _sonar(cfg)
    lines = gui.fan_polylines(son)
    pts = np.concatenate(lines)
    rel = (pts - son.position) @ son.rot                      # sonar frame: x forward, y port, z up
    r = np.linalg.norm(rel, axis=1)
    far = r > 1e-9
    assert np.all(r <= son.range + 1e-9) and np.allclose(r[far][r[far] > son.range - 1e-6], son.range)
    u = -rel[far, 1] / r[far]                                 # direction cosine along the array (+ starboard)
    el = np.degrees(np.arcsin(rel[far, 2] / r[far]))
    assert np.max(np.abs(u)) == pytest.approx(math.sin(son.theta_max), rel=1e-9)
    assert np.max(np.abs(el)) == pytest.approx(son.fov_v_deg / 2, rel=1e-9)
    # corner rays start at the acoustic centre; the centre line points along the sonar's x axis
    assert np.allclose(lines[-1][1] - son.position, son.range * son.rot[:, 0])


def test_returns_overlay_points():
    cfg = config.load()
    son = _sonar(cfg)
    img = np.zeros((len(son.range_m), len(son.bearing)), np.uint8)
    img[500, 100] = 250                                       # one bright pixel: range 2 m, a port-side beam
    img[200, 400] = 50                                        # below the threshold
    pts, colors = gui.returns_points(son, img, threshold=110)
    assert len(pts) == 1 and colors.shape == (1, 4)
    rel = (pts[0] - son.position) @ son.rot
    assert np.linalg.norm(rel) == pytest.approx(son.range_m[500])
    assert rel[2] == pytest.approx(0.0, abs=1e-12)            # on the fan's centre plane
    assert math.degrees(math.atan2(-rel[1], rel[0])) == pytest.approx(son.bearing[100])


def test_quaternions_and_static_frames():
    rng = np.random.default_rng(1)
    for _ in range(200):
        q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        rot = q * np.sign(np.linalg.det(q))
        x, y, z, w = ros_bridge.quat_xyzw(rot)
        back = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        assert np.allclose(back, rot, atol=1e-12)
    for fixed in (ros_bridge.R_SONAR_PROJECTED, main.R_BODY_OPTICAL):
        assert np.allclose(fixed @ fixed.T, np.eye(3)) and np.linalg.det(fixed) == pytest.approx(1.0)
    # ProjectedSonarImage axes: x up, y starboard, z forward
    assert np.allclose(ros_bridge.R_SONAR_PROJECTED @ [1, 0, 0], [0, 0, 1])     # its x (up) is the sonar's z
    assert np.allclose(ros_bridge.R_SONAR_PROJECTED @ [0, 1, 0], [0, -1, 0])    # its y is starboard (-port)
    # camera optical frame: z forward, x right, y down
    assert np.allclose(main.R_BODY_OPTICAL @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(main.R_BODY_OPTICAL @ [1, 0, 0], [0, -1, 0])
    cfg = config.load()
    frames = main.static_frames(cfg, _sonar(cfg))
    assert [(p, c) for p, c, *_ in frames] == [("tank", "oculus"), ("oculus", "oculus_projected"),
                                               ("tank", "camera"), ("camera", "camera_optical")]
