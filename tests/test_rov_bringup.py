"""ros/src/rov_bringup: the real ROV's MAVROS plugin set, the recording profile, and a test bag written and replayed
with that profile's MCAP options.

Pure-Python parts run anywhere; the launch files and the bag need host ROS 2 Jazzy (`uw test` runs this file with
/opt/ros/jazzy sourced, after colcon-building ros/src).
"""
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml

WS = Path(__file__).resolve().parents[1]
PKG = WS / "ros" / "src" / "rov_bringup"
sys.path.insert(0, str(PKG))
from rov_bringup import record  # noqa: E402

#: MAVROS plugins with a path that can arm, change mode, write parameters or command motion
FORBIDDEN = {"command", "param", "setpoint_position", "setpoint_velocity", "setpoint_accel", "setpoint_attitude",
             "setpoint_raw", "setpoint_trajectory", "manual_control", "waypoint", "rallypoint", "geofence", "ftp",
             "actuator_control", "mount_control", "hil", "safety_area", "home_position"}


def test_mavros_loads_only_reading_plugins():
    p = yaml.safe_load((PKG / "config" / "mavros_pluginlists.yaml").read_text())["/**"]["ros__parameters"]
    assert p["plugin_denylist"] == ["*"], "deny everything, then allow back"
    allowed = set(p["plugin_allowlist"])
    assert not allowed & FORBIDDEN, f"write-capable plugins allowed: {sorted(allowed & FORBIDDEN)}"
    assert {"sys_status", "imu", "rc_io"} <= allowed, "state, IMU and motor outputs are what the profile records"
    assert not any("*" in a for a in allowed), "no wildcard in the allowlist: each plugin is a decision"


def test_the_light_profile_records_mcap_without_compression():
    prof = record.load_profile("light", PKG)
    assert prof["storage"] == "mcap"
    opts = yaml.safe_load((PKG / "config" / prof["storage_config"]).read_text())
    assert opts["compression"] == "None", "zstd costs the Pi's CPU"
    t = record.topics(prof, "rov1")
    assert all(n.startswith("/rov1/mavros/") for n in t), "docs/topic_map.md §4.3 namespace"
    for needed in ("imu/data", "imu/static_pressure", "state", "rc/out"):
        assert f"/rov1/mavros/{needed}" in t
    assert all(ty.count("/msg/") == 1 for ty in t.values())


def test_the_record_command():
    cmd = record.command("light", "rov2", "/data/bags", stamp="2026-01-02_03-04-05", share=PKG)
    assert cmd[:5] == ["ros2", "bag", "record", "-s", "mcap"]
    assert cmd[cmd.index("--storage-config-file") + 1] == str(PKG / "config" / "mcap_light.yaml")
    assert cmd[cmd.index("-o") + 1] == "/data/bags/2026-01-02_03-04-05_light"
    assert cmd[cmd.index("--max-bag-duration") + 1] == "600"
    topics = cmd[cmd.index("--topics") + 1:]
    assert "/rov2/mavros/rc/out" in topics and len(topics) == len(record.load_profile("light", PKG)["topics"])


@pytest.mark.parametrize("name", ["mavros", "record", "bringup"])
def test_launch_files_describe(name):
    pytest.importorskip("launch_ros", reason="host ROS 2 Jazzy")
    spec = importlib.util.spec_from_file_location(name, PKG / "launch" / f"{name}.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    args = {a.name for a in ld.entities if a.__class__.__name__ == "DeclareLaunchArgument"}
    assert "vehicle" in args


def test_a_bag_written_with_the_profile_replays(tmp_path):
    """Write a short bag of the profile's standard-message topics with its MCAP options, read it back."""
    rosbag2_py = pytest.importorskip("rosbag2_py", reason="host ROS 2 Jazzy")
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message

    prof = record.load_profile("light", PKG)
    topics = record.topics(prof, "rov1")
    usable = {}
    for name, ty in topics.items():
        try:
            usable[name] = (ty, get_message(ty))
        except (ModuleNotFoundError, AttributeError, ValueError):
            pass                                    # mavros_msgs is not on the host: the vehicle has it
    assert "/rov1/mavros/imu/data" in usable
    uri = str(tmp_path / "bag")
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap",
                                     storage_config_uri=str(PKG / "config" / prof["storage_config"])),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for i, (name, (ty, cls)) in enumerate(usable.items()):
        w.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=ty, serialization_format="cdr"))
    n = 20
    for k in range(n):
        for name, (ty, cls) in usable.items():
            msg = cls()
            if hasattr(msg, "header"):
                msg.header.stamp.sec, msg.header.frame_id = k, "rov1/base_link"
            w.write(name, serialize_message(msg), k * 50_000_000)
    del w
    files = list((tmp_path / "bag").glob("*.mcap"))
    assert len(files) == 1
    raw = files[0].read_bytes()
    assert b"zstd" not in raw and b"lz4" not in raw, "chunks must be stored uncompressed"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    got = {}
    while r.has_next():
        name, data, _t = r.read_next()
        msg = deserialize_message(data, usable[name][1])
        got[name] = got.get(name, 0) + 1
        if hasattr(msg, "header"):
            assert msg.header.frame_id == "rov1/base_link"
    assert got == {name: n for name in usable}
