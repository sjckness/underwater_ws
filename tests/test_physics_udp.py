"""uw/physics_udp.py with a fake SITL client speaking tools/stub_physics.py's protocol (host, pure Python + numpy)."""
import json
import re
import socket
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from uw import config  # noqa: E402
from uw.config import ConfigError  # noqa: E402
from uw.physics_udp import MAGIC_16, MAGIC_32, ArduSubPose, make_server, parse_servo, reply  # noqa: E402

import stub_physics  # noqa: E402

G = 9.80665


@pytest.fixture
def cfg():
    # room9: the body at the configured sonar pose is inside the room (in the tank it pokes through the -x wall)
    c = config.load(profile="linus", presets=["room9"])
    c["sim"]["physics_port"] = 0            # an ephemeral port: never collides with a real SITL or stub on 9002
    c["sim"]["physics_rtf"] = 0.0           # unpaced: the test runs as fast as the integrator
    return c


def servo(pwm, rate=400, frame=0, nch=16):
    pwm = list(pwm) + [1500] * (nch - len(pwm))
    return struct.pack("<HHI%dH" % nch, MAGIC_16 if nch == 16 else MAGIC_32, rate, frame, *pwm)


def as_json(frame: bytes) -> dict:
    s = frame.decode()
    assert s.startswith("\n") and s.endswith("\n")
    return json.loads(s.strip())


class Client:
    def __init__(self, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(2.0)
        self.addr = ("127.0.0.1", port)

    def send(self, pkt) -> dict:
        self.s.sendto(pkt, self.addr)
        return as_json(self.s.recvfrom(4096)[0])

    def close(self):
        self.s.close()


def test_reply_has_the_stubs_layout():
    """Same keys, order and separators as the stub (SIM_JSON's parser is strstr + sscanf, not a JSON parser)."""
    keys = lambda b: re.findall(r'"(\w+)":', b.decode())            # noqa: E731
    seps = lambda b: sorted(set(re.findall(r"\d(, |,)[-\d]", b.decode())))  # noqa: E731
    ours = reply(1.0, [0, 0, 0], [0, 0, -G], [0, 0, 1.0], [0, 0, 0], [0, 0, 0])
    stub = stub_physics.reply(1.0, 1.0)
    assert keys(ours) == keys(stub)
    assert seps(ours) == seps(stub) == [", "]
    a, b = as_json(ours), as_json(stub)
    for k in ("position", "attitude", "velocity"):
        assert a[k] == pytest.approx(b[k])
    assert a["imu"]["accel_body"] == pytest.approx(b["imu"]["accel_body"])


def test_parse_servo_16_and_32_channels():
    assert parse_servo(servo([1100, 1900], nch=16))[2][:3] == (1100, 1900, 1500)
    r = parse_servo(servo([1600], rate=1200, frame=7, nch=32))
    assert r[0] == 1200 and r[1] == 7 and len(r[2]) == 32
    assert parse_servo(b"\x00" * 40) is None and parse_servo(b"\x1aH") is None


def test_neutral_pwm_hovers_and_reports_rest(cfg):
    srv = make_server(cfg).start()
    c = Client(srv.port)
    try:
        d0 = srv.state.position_ned[2] - srv.home_ned[2]
        first = c.send(servo([1500] * 16))
        assert first["timestamp"] == pytest.approx(1 / 400)
        assert first["imu"]["accel_body"] == pytest.approx([0.0, 0.0, -G], abs=0.1)   # +2 N net buoyancy: ~0.07 m/s^2 up
        assert first["position"][:2] == pytest.approx([0.0, 0.0], abs=1e-6)      # home = start point
        assert first["position"][2] == pytest.approx(d0, abs=1e-4)
        for i in range(399):
            last = c.send(servo([1500] * 16, frame=i + 1, nch=32 if i % 2 else 16))
        assert last["timestamp"] == pytest.approx(1.0)
        assert srv.packets == 400 and srv.error is None
        # +2.0 N net buoyancy: a zero-thrust BlueROV2 rises a little in 1 s, nothing else moves
        assert -0.2 < last["position"][2] - d0 < 0.0
        assert last["attitude"] == pytest.approx([0.0, 0.0, first["attitude"][2]], abs=1e-3)
    finally:
        c.close()
        srv.close()
    assert not srv.thread.is_alive()


def test_vertical_thrusters_push_up(cfg):
    srv = make_server(cfg).start()
    c = Client(srv.port)
    try:
        d0 = c.send(servo([1500] * 16))["position"][2]
        for i in range(400):                       # MOT_5, MOT_6 (channels 4, 5) full positive: climb
            r = c.send(servo([1500, 1500, 1500, 1500, 1900, 1900], frame=i))
        assert r["position"][2] < d0 - 0.2
        assert r["velocity"][2] < -0.1
    finally:
        c.close()
        srv.close()


def test_busy_port_is_a_clear_error(cfg):
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("127.0.0.1", 0))
    cfg["sim"]["physics_port"] = blocker.getsockname()[1]
    try:
        with pytest.raises(ConfigError, match="busy"):
            make_server(cfg).start()
    finally:
        blocker.close()


def test_off_by_default():
    with pytest.raises(ConfigError, match="physics_port"):
        make_server(config.load(profile="linus"))


def test_tank_start_is_clamped_inside_the_bound():
    c = config.load(profile="linus")
    c["sim"]["physics_port"], c["sim"]["physics_rtf"] = 0, 0.0
    srv = make_server(c)
    p, _ = srv.body_pose()
    assert np.all(p >= srv.vehicle.bounds.lo - 1e-9) and np.all(p <= srv.vehicle.bounds.hi + 1e-9)
    srv.start()
    cl = Client(srv.port)
    try:
        r = cl.send(servo([1500] * 16))
        assert r["imu"]["accel_body"] == pytest.approx([0.0, 0.0, -G], abs=0.1)   # +2 N net buoyancy: ~0.07 m/s^2 up     # no bound kick at t = 0
    finally:
        cl.close()
        srv.close()


def test_ardusub_pose_starts_at_the_configured_sonar_pose(cfg):
    pose = ArduSubPose(cfg)
    try:
        here = pose.at(0.0)
        assert here.position == pytest.approx(cfg["sonar"]["position"], abs=1e-6)
        assert here.rpy_deg == pytest.approx([cfg["sonar"][k] for k in ("roll_deg", "pitch_deg", "yaw_deg")],
                                             abs=1e-6)
    finally:
        pose.close()
