"""vehicles/common/a50.py: the emulated Water Linked DVL A50 as a client on the vehicle's network sees it (host, numpy only).

What the clients rely on is pinned here: the BlueOS Water Linked DVL extension (v1.0.11) reads "DVL" in
/api/v1/about's product_name, "temperature" in /api/v1/about/status, and from the TCP stream the velocity report's
vx, vy, vz, altitude, velocity_valid, fom and time (ms since the previous report: it becomes VISION_POSITION_DELTA's
time_delta_usec) and position_local's roll, pitch, yaw. The protocol itself is Water Linked's json_v3.3.
"""
import json
import math
import socket
import sys
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from uw import config  # noqa: E402
from physics.pool_geometry import PoolBottom  # noqa: E402
from vehicles.common import dvl as dvl_model  # noqa: E402
from vehicles.common.a50 import A50  # noqa: E402

RATE = 400.0


@dataclass
class State:
    velocity_frd: np.ndarray
    rate_frd: np.ndarray


def pool_cfg(*overrides):
    cfg = config.load(config.POOL_CONFIG, overrides=list(overrides))
    cfg["dvl"]["device"].update(bind="127.0.0.1", tcp_port=0, http_port=0)
    return cfg


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def strict(line: str) -> dict:
    """JSON as a strict parser reads it: no NaN or Infinity."""
    def no(c):
        raise ValueError(f"non-standard JSON constant {c}")
    return json.loads(line, parse_constant=no)


class Rig:
    """A device driven by a straight, level run at a constant forward speed, and one TCP client reading it."""

    def __init__(self, cfg, heading_deg=0.0, speed=0.3, p0=(5.0, 2.0, 2.5)):
        self.dev = A50(dvl_model.from_config(cfg, PoolBottom(cfg)), cfg).start()
        self.rot = rot_z(heading_deg)
        self.speed = speed
        self.p0 = np.asarray(p0, float)
        self.t = 0.0
        self.sock = socket.create_connection(("127.0.0.1", self.dev.tcp_port), timeout=3)
        self.buf = b""
        for _ in range(100):                     # until the server has registered the client
            if self.dev.clients:
                break
            time.sleep(0.01)

    def fly(self, seconds):
        for _ in range(int(round(seconds * RATE))):
            self.t += 1.0 / RATE
            p = self.p0 + self.rot @ np.array([self.speed * self.t, 0.0, 0.0])
            self.dev.step(self.t, (p, self.rot), State(np.array([self.speed, 0.0, 0.0]), np.zeros(3)))

    def send(self, obj):
        self.sock.sendall((json.dumps(obj) + "\n").encode())

    def read(self, quiet_s=0.4):
        out = []
        self.sock.settimeout(quiet_s)
        while True:
            try:
                data = self.sock.recv(1 << 16)
            except socket.timeout:
                break
            if not data:
                break
            self.buf += data
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            out.append(strict(line.decode()))
        return out

    def command(self, obj):
        self.send(obj)
        replies = [m for m in self.read() if m.get("type") == "response"]
        assert len(replies) == 1, replies
        return replies[0]

    def close(self):
        self.sock.close()
        self.dev.close()


@pytest.fixture
def rig():
    r = Rig(pool_cfg())
    yield r
    r.close()


def test_the_pool_instrument_is_the_tanks():
    """One instrument: sim/config/pool/pool.yaml repeats sim/config/tank.yaml's A50 (vehicles.common.dvl can read only one dvl:
    section per config), and only the mount, the echo reference and the device's network side may differ."""
    tank, pool = config.load()["dvl"], config.load(config.POOL_CONFIG)["dvl"]
    differ = {"mount", "rssi_ref_dbm", "device", "enabled"}
    assert set(tank) - differ <= set(pool), sorted(set(tank) - set(pool))
    for key in sorted(set(tank) - differ):
        assert pool[key] == tank[key], key


@pytest.mark.parametrize("z", [0.3, 1.0, 2.5, 4.0, 4.88])
def test_bottom_lock_over_the_whole_pool_depth(z):
    """From the floor to floating at the surface (z 4.88), over the tiles: the echo reference must carry 5 m.
    (Over the pipe a beam at a glancing angle does not come back, which is right; hence a spot clear of the props.)"""
    cfg = pool_cfg("dvl.noise=false")
    d = dvl_model.from_config(cfg, PoolBottom(cfg))
    rep = d.sample(0.0, (5.0, 3.0, z), np.eye(3), np.zeros(3), np.zeros(3))      # clear of the props
    assert rep.velocity_valid and rep.num_good_beams == 4
    mount_z = cfg["dvl"]["mount"]["position_frd_m"][2]
    assert rep.altitude == pytest.approx(z - mount_z, abs=1e-6)


def test_the_http_api_the_extension_probes(rig):
    base = f"http://127.0.0.1:{rig.dev.http_port}/api/v1"
    about = strict(urllib.request.urlopen(f"{base}/about", timeout=3).read().decode())
    assert "DVL" in about["product_name"]
    status = strict(urllib.request.urlopen(f"{base}/about/status", timeout=3).read().decode())
    assert isinstance(status["temperature"], float) and status["temperature"] < 45.0
    with pytest.raises(urllib.error.HTTPError):
        urllib.request.urlopen(f"{base}/nothing", timeout=3)


def test_the_stream_carries_what_the_extension_reads(rig):
    rig.fly(3.0)
    msgs = rig.read()
    vel = [m for m in msgs if m["type"] == "velocity"]
    dr = [m for m in msgs if m["type"] == "position_local"]
    assert len(vel) >= 6 and 13 <= len(dr) <= 17                     # 2-15 Hz with the altitude; DR at 5 Hz
    for m in vel:
        assert {"vx", "vy", "vz", "altitude", "velocity_valid", "fom", "time", "covariance", "transducers",
                "time_of_validity", "time_of_transmission", "status"} <= set(m)
        assert m["format"] == "json_v3.3" and m["velocity_valid"] and m["time"] > 0.0
        assert len(m["transducers"]) == 4 and all(t["beam_valid"] for t in m["transducers"])
        assert m["time_of_transmission"] >= m["time_of_validity"] > 1.6e15      # Unix microseconds
    for m in dr:
        assert {"ts", "x", "y", "z", "std", "roll", "pitch", "yaw", "status"} <= set(m) and m["ts"] > 1.6e9
    gaps = [m["time"] for m in vel]
    assert gaps[0] == pytest.approx(gaps[1], rel=0.2), "the first report's interval is a real one, not 0"


def test_velocity_is_scaled_by_the_configured_speed_of_sound():
    """The device converts Doppler with its own speed of sound (default 1475 m/s): set it to the water's to read
    the true speed. Without the model's noise, so that only the conversion is left."""
    r = Rig(pool_cfg("dvl.noise=false"))
    try:
        true_c = r.dev.true_sound_speed
        r.fly(2.0)
        vx = [m["vx"] for m in r.read() if m["type"] == "velocity"]
        assert vx and np.allclose(vx, 0.3 * 1475.0 / true_c, atol=2e-4)
        assert r.command({"command": "set_config", "parameters": {"speed_of_sound": true_c}})["success"]
        r.fly(2.0)
        vx = [m["vx"] for m in r.read() if m["type"] == "velocity"]
        assert vx and np.allclose(vx, 0.3, atol=2e-4)
    finally:
        r.close()


def test_dead_reckoning_follows_the_run_and_resets():
    r = Rig(pool_cfg("dvl.noise=false"), heading_deg=90.0)
    try:
        r.command({"command": "set_config", "parameters": {"speed_of_sound": r.dev.true_sound_speed}})
        r.fly(4.0)
        last = [m for m in r.read() if m["type"] == "position_local"][-1]
        # along the heading at the start of the run; up to a report period and a DR period (0.2 s each) behind
        assert 0.3 * (4.0 - 0.45) <= last["x"] <= 0.3 * 4.0 + 1e-3
        assert abs(last["y"]) < 0.05 and abs(last["z"]) < 0.05
        assert abs(last["yaw"]) < 0.5 and abs(last["roll"]) < 0.5 and abs(last["pitch"]) < 0.5
        assert r.command({"command": "reset_dead_reckoning"})["success"]
        r.fly(0.5)
        last = [m for m in r.read() if m["type"] == "position_local"][-1]
        assert last["x"] < 0.3 * 0.5 + 0.05
    finally:
        r.close()


def test_mounting_rotation_offset_turns_the_output_into_the_vehicle_frame():
    """A DVL turned 90 deg on the vehicle reports in its own axes; the matching setting gives the vehicle's."""
    r = Rig(pool_cfg("dvl.mount.rotation_offset_deg=90.0", "dvl.noise=false"))
    try:
        r.fly(1.5)
        raw = [m for m in r.read() if m["type"] == "velocity"][-1]
        assert abs(raw["vx"]) < 0.01 and abs(raw["vy"]) > 0.25
        assert r.command({"command": "set_config", "parameters": {"mounting_rotation_offset": 90.0}})["success"]
        r.fly(1.5)
        out = [m for m in r.read() if m["type"] == "velocity"][-1]
        assert out["vx"] > 0.25 and abs(out["vy"]) < 0.01
    finally:
        r.close()


def test_commands_answer_in_the_protocols_shape(rig):
    got = rig.command({"command": "get_config"})
    assert got["success"] and got["type"] == "response" and got["response_to"] == "get_config"
    assert set(got["result"]) == {"speed_of_sound", "acoustic_enabled", "dark_mode_enabled",
                                  "mounting_rotation_offset", "range_mode", "periodic_cycling_enabled"}
    info = rig.command({"command": "get_version_info"})["result"]
    assert info["product_name"] == "DVL A50" and info["is_ready"]
    for bad in ({"command": "set_config", "parameters": {"speed_of_sound": 500}},
                {"command": "set_config", "parameters": {"no_such_key": 1}},
                {"command": "set_config", "parameters": {"range_mode": "wt"}},
                {"command": "no_such_command"}):
        r = rig.command(bad)
        assert not r["success"] and r["error_message"]
    assert rig.command({"command": "calibrate_gyro"})["success"]


def test_trigger_ping_only_when_the_acoustics_are_off(rig):
    assert not rig.command({"command": "trigger_ping"})["success"]
    assert rig.command({"command": "set_config", "parameters": {"acoustic_enabled": False}})["success"]
    rig.fly(2.0)
    assert not [m for m in rig.read() if m["type"] == "velocity"], "no pinging with the acoustics off"
    for _ in range(2):
        assert rig.command({"command": "trigger_ping"})["success"]
    rig.fly(0.1)
    assert len([m for m in rig.read() if m["type"] == "velocity"]) == 2


def test_no_lock_is_still_strict_json():
    """Out of the footprint every beam misses: the report says so with finite stand-ins, never NaN or Infinity."""
    cfg = pool_cfg()
    dev = A50(dvl_model.from_config(cfg, PoolBottom(cfg)), cfg)
    rep = dev.dvl.sample(0.0, (-5.0, -5.0, 2.0), np.eye(3), np.zeros(3), np.zeros(3))
    msg, _ = dev._velocity_message(rep, dict(dev.config), np.eye(3), 0.1)      # noqa: SLF001
    line = json.dumps(msg)
    assert not msg["velocity_valid"] and msg["fom"] == -1.0 and msg["altitude"] == -1.0
    assert all(t["distance"] == -1.0 for t in strict(line)["transducers"])
