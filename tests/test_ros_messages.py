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
                      "tvg_db_per_decade": 30.0, "absorption_db_per_m": 0.25,
                      # ground truth, present in every per-ping meta and allowed on no published topic but /gt
                      "position": [0.1, 1.5, 1.05], "roll_deg": 0.0, "pitch_deg": 15.0, "yaw_deg": 0.0,
                      "rotation": [[0.9659258, 0.0, 0.2588190], [0.0, 1.0, 0.0], [-0.2588190, 0.0, 0.9659258]],
                      "pose_source": "fixed"},
            "plume_mask_threshold": 0.5, "plume_pixels": {"all": 120, "direct": 90, "geom": 36},
            "leak": {"position": [1.7, 1.45, 0.4], "range_m": 1.6, "flow_lpm": 2.0}}
    return sinks.Message("sonar", 3, 12.3, arrays={"polar_db": db, "range_m": np.arange(n_r) * 0.004,
                                                   "bearing_deg": np.linspace(-65, 65, n_b)},
                         images={"polar": polar[::-1]}, meta=meta), polar


def test_bridge_messages():
    pytest.importorskip("sensor_msgs.msg", reason="the Isaac ROS bridge's rclpy: python -m uw test")
    msg, polar = _sonar_message()
    img = ros_bridge.image_msg(msg.stamp, "rov1/sonar_fls", msg.images["polar"][::-1], "mono8")
    assert (img.height, img.width, img.step, img.encoding) == (40, 16, 16, "mono8")
    assert np.array_equal(np.frombuffer(bytes(img.data), np.uint8).reshape(40, 16), polar)
    assert (img.header.stamp.sec, img.header.stamp.nanosec) == (12, 300000000)
    f = ros_bridge.image_msg(0.0, "rov1/sonar_fls", msg.arrays["polar_db"].astype(np.float32), "32FC1")
    back = np.frombuffer(bytes(f.data), np.float32).reshape(40, 16)
    assert f.step == 64 and np.isnan(back[:3]).all() and np.allclose(back[3:], msg.arrays["polar_db"][3:])
    meta = ros_bridge.sonar_meta(msg)
    assert meta["projected_frame_id"] == "rov1/sonar_fls_projected" and len(meta["bearing_deg"]) == 16
    json.dumps(meta)
    s = msg.meta["sonar"]
    gt = ros_bridge.pose_stamped_msg(msg.stamp, "map", s["position"], s["rotation"])
    assert gt.header.frame_id == "map" and (gt.header.stamp.sec, gt.header.stamp.nanosec) == (12, 300000000)
    assert [gt.pose.position.x, gt.pose.position.y, gt.pose.position.z] == s["position"]
    q = gt.pose.orientation
    assert np.allclose([q.x, q.y, q.z, q.w], ros_bridge.quat_xyzw(np.array(s["rotation"])))
    K = np.array([[600.0, 0, 640], [0, 600.0, 360], [0, 0, 1]])
    ci = ros_bridge.camera_info_msg(1.0, "rov1/camera_front_optical", K, 1280, 720)
    assert list(ci.k) == list(K.ravel()) and ci.p[0] == 600.0 and ci.p[10] == 1.0
    pose = ros_bridge.leak_pose_msg(1.0, [1.7, 1.45, 0.4], [math.sin(0.7), 0.0, math.cos(0.7)])
    q = pose.pose.orientation
    x_axis = [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y + q.z * q.w), 2 * (q.x * q.z - q.y * q.w)]
    assert np.allclose(x_axis, [math.sin(0.7), 0.0, math.cos(0.7)])
    pos = np.random.default_rng(1).uniform(0, 3, (10000, 3)).astype(np.float32)
    ma = ros_bridge.plume_markers_msg(2.0, pos, np.full(10000, 0.003, np.float32), 3000)
    m = ma.markers[0]
    assert len(m.points) == len(m.colors) == 2500 and m.header.frame_id == "map" and m.scale.x == pytest.approx(0.003)
    tf = ros_bridge.transform_msg(0.0, "rov1/sonar_fls", "rov1/sonar_fls_projected", ros_bridge.R_SONAR_PROJECTED,
                                  (0, 0, 0))
    r = tf.transform.rotation
    assert np.allclose([r.x, r.y, r.z, r.w], ros_bridge.quat_xyzw(ros_bridge.R_SONAR_PROJECTED))


def test_topic_and_frame_map():
    """docs/topic_map.md §4 and §2: these names are the contract with the Jetson stack and with the bags, so pin
    them. ros.vehicle must move the vehicle namespace and the frame prefix together, and nothing else. Needs no
    ROS: topics() and frames() are pure."""
    t = ros_bridge.topics("rov1")
    assert t["polar"] == "/rov1/sensors/sonar_fls/image"
    assert t["meta"] == "/rov1/sensors/sonar_fls/meta"
    assert t["image"] == "/rov1/sensors/camera_front/image_raw"
    assert t["camera_info"] == "/rov1/sensors/camera_front/camera_info"
    assert t["polar_db"] == "/gt/polar_db" and t["leak"] == "/gt/leak/pose"
    assert t["plume"] == "/gt/plume/markers" and t["gt_sonar_pose"] == "/gt/sonar_pose"
    assert t["fan"] == "/sim/debug/sonar_fan"
    assert t["clock"] == "/clock" and t["tf_static"] == "/tf_static"      # global by necessity
    # exactly four vehicle topics, and no /gt or /sim name hiding inside one
    vehicle = {k: v for k, v in t.items() if v.startswith("/rov1/")}
    assert set(vehicle) == {"polar", "meta", "image", "camera_info"}
    assert ros_bridge.topics("rov2")["meta"] == "/rov2/sensors/sonar_fls/meta"
    f = ros_bridge.frames("rov2")
    assert f["world"] == "map" and f["sonar"] == "rov2/sonar_fls" and f["gt_sonar"] == "gt/rov2/sonar_fls"
    assert not any(v.startswith("/") for v in f.values())                # tf2 rejects a leading slash


def test_sonar_meta_leaks_no_ground_truth():
    """docs/ros_hil.md §2.5 rule 2. .../sonar_fls/meta is the one metadata topic the detector must subscribe to, and the
    per-ping meta it is built from carries the sonar's true pose, the leak's range and bearing, and
    plume_pixels.geom — which *is* the in_view target. So sonar_meta() is a fixed key list and one `**s` splat
    away from shipping the answer. Needs no ROS: sonar_meta() is pure."""
    msg, _ = _sonar_message()
    assert set(msg.meta["sonar"]) & set(ros_bridge.META_FORBIDDEN)       # the fixture does carry the truth
    meta = ros_bridge.sonar_meta(msg)
    for key in ros_bridge.META_FORBIDDEN:
        assert key not in meta, f"{key} must not reach the meta topic"
    text = json.dumps(meta)
    for word in ("plume", "mask", "leak", "in_view", "roll", "pitch", "yaw", "position"):
        assert word not in text, f"{word!r} in the published meta"
    # and the whole gain chain must be there, or the 8-bit polar image is not invertible (§2.4)
    assert {"display_db", "tvg_db_per_decade", "absorption_db_per_m", "noise_floor_db"} <= set(meta)


def test_relay_projected_sonar_image():
    pytest.importorskip("marine_acoustic_msgs.msg", reason="host ROS 2 Jazzy with ros-jazzy-marine-acoustic-msgs")
    from sensor_msgs.msg import Image
    sys.path.insert(0, str(WS / "ros" / "sonar_relay"))
    from sonar_relay.relay import projected

    msg, polar = _sonar_message()
    img = Image(height=40, width=16, encoding="mono8", step=16, data=polar.tobytes())
    img.header.stamp.sec = 12
    out = projected(img, ros_bridge.sonar_meta(msg), db=False)
    assert out.header.frame_id == "rov1/sonar_fls_projected" and out.header.stamp.sec == 12
    assert out.image.beam_count == 16 and len(out.ranges) == 40 and out.image.dtype == out.image.DTYPE_UINT8
    assert np.array_equal(np.frombuffer(bytes(out.image.data), np.uint8).reshape(40, 16), polar)
    d = np.array([[v.x, v.y, v.z] for v in out.beam_directions])
    assert np.allclose(np.linalg.norm(d, axis=1), 1) and d[0, 1] < 0 < d[-1, 1]      # port beams first (y starboard)
    assert out.ping_info.frequency == 750e3 and len(out.ping_info.rx_beamwidths) == 16
    with pytest.raises(ValueError):
        projected(Image(height=39, width=16), ros_bridge.sonar_meta(msg), db=False)


def test_pose_input_roundtrip():
    """sonar.pose_source ros: a PoseStamped built from a pose gives that pose back."""
    pytest.importorskip("geometry_msgs.msg", reason="the Isaac ROS bridge's rclpy: python -m uw test")
    from geometry_msgs.msg import PoseStamped
    from uw import geometry
    rot = geometry.rot_rpy_deg([10.0, 25.0, -130.0])
    m = PoseStamped()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.4, 1.1, 1.3
    x, y, z, w = ros_bridge.quat_xyzw(rot)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = x, y, z, w
    pos, back = ros_bridge.pose_from_msg(m)
    assert np.allclose(pos, [0.4, 1.1, 1.3]) and np.allclose(back, rot)
