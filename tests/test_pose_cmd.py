"""The /sim/cmd/rov1/pose producer (ros/pose_cmd), host-side: the pose each mode commands. Pure functions only —
no node, no DDS participant, so safe while other ROS stacks run.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

from uw import config, geometry, pose as uwpose

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "ros" / "pose_cmd"))
from pose_cmd.cmd import FRAME, TOPIC, KeyDrive, held_pose, traj_time      # noqa: E402 - after sys.path


def test_the_topic_and_frame_are_the_map_s():
    assert TOPIC == "/sim/cmd/rov1/pose" and FRAME == "map"


def test_hold_look_at_points_the_x_axis_at_the_target():
    target = np.array([1.7, 1.45, 0.4])
    for start in ([0.1, 1.5, 1.05], [2.5, 0.3, 2.0], [1.7, 1.45, 2.5]):
        p, rpy = held_pose(start, [0.0, 15.0, 0.0], target)
        assert np.allclose(p, start)
        axis = geometry.rot_rpy_deg(rpy) @ [1.0, 0.0, 0.0]
        d = target - np.asarray(start, float)
        assert np.allclose(axis, d / np.linalg.norm(d), atol=1e-9)
        assert rpy[0] == 0.0                                   # the roll is kept, only pitch and yaw are aimed


def test_hold_without_a_target_passes_the_angles_through():
    p, rpy = held_pose([0.1, 1.5, 1.05], [1.0, 15.0, -30.0])
    assert np.allclose(p, [0.1, 1.5, 1.05]) and np.allclose(rpy, [1.0, 15.0, -30.0])


def test_traj_time_wraps_or_holds():
    assert traj_time(3.0, 10.0, loop=False) == 3.0
    assert traj_time(12.0, 10.0, loop=False) == 10.0           # holds the last pose, does not stop commanding
    assert traj_time(12.0, 10.0, loop=True) == pytest.approx(2.0)
    assert traj_time(12.0, None, loop=False) == 12.0           # a trajectory with no duration (hover)


def test_trajectory_mode_replays_the_configured_trajectory():
    """The point of the mode: the same motion the datasets were generated with, through the same code."""
    cfg = config.load(overrides=["sonar.pose_source=trajectory", "sonar.static_cache=none",
                                 "sonar.trajectory.preset=approach"])
    sp = uwpose.sensor_pose(cfg, geometry.build_scene(cfg))
    assert sp.moving
    for t in (0.0, 1.0, 3.7):
        here = sp.at(traj_time(t, float(getattr(sp, "duration", 0.0)) or None, loop=False))
        assert np.allclose(here.position, sp.at(t).position)


def test_keys_drive_moves_in_the_body_frame_and_stays_in_the_water():
    cfg = config.load()
    scene = geometry.build_scene(cfg)
    b = uwpose.Bounds(scene, float(config.get(cfg, "sonar.trajectory.margin")))
    start, rpy = [1.5, 1.5, 1.5], [0.0, 0.0, 90.0]             # yawed 90 deg: body x is world +y
    d = KeyDrive(start, rpy, b, speed=1.0, yaw_rate=20.0)
    assert d.apply("w")
    p, _ = d.step(0.5)
    assert np.allclose(p, [1.5, 2.0, 1.5])                     # forward is +y at this yaw, not +x
    assert d.apply(" ")                                        # space stops: the pose must not drift
    p2, _ = d.step(1.0)
    assert np.allclose(p2, p)
    assert d.apply("j")
    _, a = d.step(1.0)
    assert a[2] == pytest.approx(110.0)                        # yaw left
    assert d.apply("k")
    _, a = d.step(1.0)
    assert a[1] == pytest.approx(20.0)                         # k is nose down, which is pitch positive
    assert not d.apply("q")
    # driving up for a long time stops at the surface margin, not above it
    d = KeyDrive([1.5, 1.5, 1.5], [0.0, 0.0, 0.0], b, speed=1.0)
    d.apply("r")
    for _ in range(100):
        p, _ = d.step(0.1)
    assert p[2] == pytest.approx(b.hi[2]) and np.all(p <= b.hi + 1e-9)
