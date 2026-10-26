"""ROS 2 messages of the sim (uw/ros_bridge.py) and the host relay (ros/sonar_relay): layouts, frames, the
ProjectedSonarImage conversion. Message objects only: no node, no DDS participant, so safe while other ROS stacks run.

The bridge messages need the Isaac ROS bridge's rclpy (container); the relay needs the host's marine_acoustic_msgs
(host with ROS 2 Jazzy sourced). Each part skips where its messages are missing.
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

from uw import ros_bridge, sinks

WS = Path(__file__).resolve().parents[1]
if os.environ.get("UW_IN_CONTAINER"):
    ros_bridge.ensure_rclpy_path()


def _sonar_message(n_r=40, n_b=16):
    rng = np.random.default_rng(0)
    polar = rng.integers(0, 256, (n_r, n_b), dtype=np.uint8)             # row 0 = range 0
    db = rng.normal(100, 10, (n_r, n_b)).astype(np.float16)
    db[:3] = np.nan
    meta = {"sonar": {"frequency_hz": 750e3, "sound_speed": 1506.7, "beamwidth_h_deg": 1.0, "fov_v_deg": 20.0,
                      "range": 0.2, "mode": "lf", "focus": True, "noise_floor_db": 70.0, "display_db": [0, 80],
                      "position": [0.1, 1.5, 1.05], "roll_deg": 0.0, "pitch_deg": 15.0, "yaw_deg": 0.0}}
    return sinks.Message("sonar", 3, 12.3, arrays={"polar_db": db, "range_m": np.arange(n_r) * 0.004,
                                                   "bearing_deg": np.linspace(-65, 65, n_b)},
                         images={"polar": polar[::-1]}, meta=meta), polar


def test_bridge_messages():
    pytest.importorskip("sensor_msgs.msg", reason="the Isaac ROS bridge's rclpy: python -m uw test")
    msg, polar = _sonar_message()
    img = ros_bridge.image_msg(msg.stamp, "oculus", msg.images["polar"][::-1], "mono8")
    assert (img.height, img.width, img.step, img.encoding) == (40, 16, 16, "mono8")
    assert np.array_equal(np.frombuffer(bytes(img.data), np.uint8).reshape(40, 16), polar)
    assert (img.header.stamp.sec, img.header.stamp.nanosec) == (12, 300000000)
    f = ros_bridge.image_msg(0.0, "oculus", msg.arrays["polar_db"].astype(np.float32), "32FC1")
    back = np.frombuffer(bytes(f.data), np.float32).reshape(40, 16)
    assert f.step == 64 and np.isnan(back[:3]).all() and np.allclose(back[3:], msg.arrays["polar_db"][3:])
    meta = ros_bridge.sonar_meta(msg)
    assert meta["projected_frame_id"] == "oculus_projected" and len(meta["bearing_deg"]) == 16
    json.dumps(meta)
    K = np.array([[600.0, 0, 640], [0, 600.0, 360], [0, 0, 1]])
    ci = ros_bridge.camera_info_msg(1.0, "camera_optical", K, 1280, 720)
    assert list(ci.k) == list(K.ravel()) and ci.p[0] == 600.0 and ci.p[10] == 1.0
    pose = ros_bridge.leak_pose_msg(1.0, [1.7, 1.45, 0.4], [math.sin(0.7), 0.0, math.cos(0.7)])
    q = pose.pose.orientation
    x_axis = [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y + q.z * q.w), 2 * (q.x * q.z - q.y * q.w)]
    assert np.allclose(x_axis, [math.sin(0.7), 0.0, math.cos(0.7)])
    pos = np.random.default_rng(1).uniform(0, 3, (10000, 3)).astype(np.float32)
    ma = ros_bridge.plume_markers_msg(2.0, pos, np.full(10000, 0.003, np.float32), 3000)
    m = ma.markers[0]
    assert len(m.points) == len(m.colors) == 2500 and m.header.frame_id == "tank" and m.scale.x == pytest.approx(0.003)
    tf = ros_bridge.transform_msg(0.0, "oculus", "oculus_projected", ros_bridge.R_SONAR_PROJECTED, (0, 0, 0))
    r = tf.transform.rotation
    assert np.allclose([r.x, r.y, r.z, r.w], ros_bridge.quat_xyzw(ros_bridge.R_SONAR_PROJECTED))


def test_relay_projected_sonar_image():
    pytest.importorskip("marine_acoustic_msgs.msg", reason="host ROS 2 Jazzy with ros-jazzy-marine-acoustic-msgs")
    from sensor_msgs.msg import Image
    sys.path.insert(0, str(WS / "ros" / "sonar_relay"))
    from sonar_relay.relay import projected

    msg, polar = _sonar_message()
    img = Image(height=40, width=16, encoding="mono8", step=16, data=polar.tobytes())
    img.header.stamp.sec = 12
    out = projected(img, ros_bridge.sonar_meta(msg), db=False)
    assert out.header.frame_id == "oculus_projected" and out.header.stamp.sec == 12
    assert out.image.beam_count == 16 and len(out.ranges) == 40 and out.image.dtype == out.image.DTYPE_UINT8
    assert np.array_equal(np.frombuffer(bytes(out.image.data), np.uint8).reshape(40, 16), polar)
    d = np.array([[v.x, v.y, v.z] for v in out.beam_directions])
    assert np.allclose(np.linalg.norm(d, axis=1), 1) and d[0, 1] < 0 < d[-1, 1]      # port beams first (y starboard)
    assert out.ping_info.frequency == 750e3 and len(out.ping_info.rx_beamwidths) == 16
    with pytest.raises(ValueError):
        projected(Image(height=39, width=16), ros_bridge.sonar_meta(msg), db=False)
