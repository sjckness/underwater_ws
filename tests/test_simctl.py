"""vehicles/common/simctl.py and its client (M12 P2): the control channel into the physics process (host, numpy only).

A fake SITL drives the server with servo packets, as in tests/test_physics_proc.py.
"""
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uw import config  # noqa: E402
from vehicles.common.physics_udp import MAGIC_16  # noqa: E402
from vehicles.common import simctl  # noqa: E402
from vehicles.common.physics_proc import PhysicsProcess, make_pool_server  # noqa: E402
from vehicles.common.simctl_client import SimCtl, SimCtlError  # noqa: E402

NOT_HERE = "203.0.113.7"            # TEST-NET-3: never an address of this host


@pytest.fixture
def cfg():
    c = config.load(config.POOL_CONFIG)
    c["physics"]["port"] = 0
    c["physics"]["rtf"] = 0.0
    c["physics"]["control"].update(port=0, bind=["127.0.0.1", NOT_HERE])
    c["dvl"]["device"].update(bind="127.0.0.1", tcp_port=0, http_port=0)
    return c


def servo(frame=0):
    return struct.pack("<HHI16H", MAGIC_16, 400, frame, *([1500] * 16))


def fly(port, n):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    try:
        for i in range(n):
            s.sendto(servo(i), ("127.0.0.1", port))
            s.recvfrom(4096)
    finally:
        s.close()


@pytest.fixture
def running(cfg):
    """A started physics server and its control channel, without the child process."""
    srv = make_pool_server(cfg).start()
    ctl = simctl.from_config(cfg, srv).start()
    try:
        yield cfg, srv, ctl
    finally:
        ctl.close()
        srv.close()


# --------------------------------------------------------------------------------------------- dispatch

def test_requests_answer_and_errors_are_answers_too(cfg):
    srv = make_pool_server(cfg)
    ctl = simctl.SimControl(srv, cfg)
    hello = ctl.handle({"id": 7, "op": "hello"})
    assert hello["ok"] and hello["id"] == 7 and hello["result"]["proto"] == simctl.PROTO
    assert hello["result"]["pool"]["water_level"] == cfg["pool"]["water_level"] and hello["result"]["current"] is None
    for bad in ({"op": "warp"}, {"op": "reposition", "xy_tank": [-3.0, 2.0]}, {"op": "device", "name": "a50"},
                {"op": "set_current", "speed": 0.3}, {"op": "set_current", "speed_mps": 0.3, "ramp_s": -1}):
        out = ctl.handle(bad)
        assert out["ok"] is False and out["error"], bad
    assert ctl.count["errors"] == 5


def test_the_state_is_the_server_state_in_both_frames(cfg):
    srv = make_pool_server(cfg)
    st = simctl.SimControl(srv, cfg).state()
    x, y, z, yaw = cfg["vehicle"]["start"]
    assert np.allclose(st["p_tank"], [x, y, z], atol=1e-6)
    assert st["heading_enu_deg"] == pytest.approx(yaw, abs=1e-4)
    assert st["depth_m"] == pytest.approx(cfg["pool"]["water_level"] - z, abs=1e-6)
    assert st["v_water_enu"] == [0.0, 0.0, 0.0] and len(st["pwm"]) == 16 and len(st["thrust_n"]) == 8


def test_set_current_is_picked_up_by_the_next_step(cfg):
    srv = make_pool_server(cfg)
    ctl = simctl.SimControl(srv, cfg)
    out = ctl.handle({"op": "set_current", "speed_mps": 0.3, "direction_deg": 90, "ramp_s": 0})
    assert out["ok"] and out["result"]["current"]["speed_mps"] == 0.3
    assert srv.vehicle.current is None, "nothing changes before a step"
    srv.vehicle.step(srv.state, np.zeros(8))
    assert np.allclose(ctl.state()["v_water_enu"], [0.0, 0.3, 0.0], atol=1e-9)


# ---------------------------------------------------------------------------------------- over the wire

def test_an_address_this_host_does_not_have_is_skipped(running):
    _, _, ctl = running
    assert [a for a, _ in ctl.listening] == ["127.0.0.1"]


def test_two_clients_and_a_live_current(running):
    cfg, srv, ctl = running
    addr = f"127.0.0.1:{ctl.port}"
    with SimCtl(addr) as a, SimCtl(addr) as b:
        assert a.hello()["proto"] == b.hello()["proto"] == simctl.PROTO
        a.set_current(speed_mps=0.3, direction_deg=90.0)
        fly(srv.port, 4000)                                 # 10 s at the surface, thrusters off
        st = b.state()
        assert st["v_water_enu"][1] == pytest.approx(0.3, abs=1e-9)
        assert st["v_ground_enu"][1] > 0.25 and st["sitl"] is True
        assert b.stats()["control"]["clients"] == 2
        with pytest.raises(SimCtlError, match="unknown op"):
            a.call("warp")
        a.still()
        fly(srv.port, 1)
        assert b.state()["v_water_enu"] == [0.0, 0.0, 0.0]


def test_the_stream_keeps_its_rate_and_stops_when_asked(running):
    cfg, srv, ctl = running
    stop = threading.Event()

    def sitl():                                             # keep the physics stepping meanwhile
        i = 0
        while not stop.is_set():
            fly(srv.port, 20)
            i += 1
            time.sleep(0.01)
    th = threading.Thread(target=sitl, daemon=True)
    th.start()
    try:
        with SimCtl(f"127.0.0.1:{ctl.port}") as sim:
            t0 = time.monotonic()
            msgs = list(sim.stream(rate_hz=20, duration_s=1.5))
            dt = time.monotonic() - t0
            assert len(msgs) == pytest.approx(20 * dt, rel=0.15)
            ts = [m["sim_t"] for m in msgs]
            assert all(b >= a for a, b in zip(ts, ts[1:])) and ts[-1] > ts[0]
            assert sim.hello()["proto"] == simctl.PROTO, "the connection is usable after a stream"
            with pytest.raises(SimCtlError, match="rate_hz"):
                list(sim.stream(rate_hz=500, duration_s=0.5))
    finally:
        stop.set()
        th.join(5)


def test_reposition_is_horizontal_and_applied_at_the_next_packet(running):
    cfg, srv, ctl = running
    with SimCtl(f"127.0.0.1:{ctl.port}") as sim:
        sim.set_current(speed_mps=0.3, direction_deg=0.0)
        fly(srv.port, 2000)                                 # drifting now
        before = sim.state()
        assert before["v_ground_enu"][0] > 0.2
        sim.reposition(3.0, 7.0)
        assert sim.state()["p_tank"][:2] == before["p_tank"][:2], "SITL is alive: queued, not applied"
        fly(srv.port, 1)
        after = sim.state()
        assert after["p_tank"][:2] == pytest.approx([3.0, 7.0], abs=0.01)
        assert after["depth_m"] == pytest.approx(before["depth_m"], abs=0.01)
        assert after["heading_enu_deg"] == pytest.approx(before["heading_enu_deg"], abs=0.5)
        assert np.linalg.norm(after["v_ground_enu"]) < 0.01, "the twist is zeroed (one step of current since)"
        with pytest.raises(SimCtlError, match="outside the soft bound"):
            sim.reposition(80.0, 5.0)
        with pytest.raises(SimCtlError, match="horizontal only"):
            sim.call("reposition", xy_tank=[1.0, 2.0, 3.0])


def test_without_sitl_a_reposition_applies_at_once(cfg):
    srv = make_pool_server(cfg)
    srv.request_reposition([4.0, 4.0])
    p, _ = srv.body_pose()
    assert p[:2] == pytest.approx([4.0, 4.0])


def test_the_step_time_is_measured(running):
    cfg, srv, ctl = running
    fly(srv.port, 400)
    st = ctl.stats()
    assert st["step_ms"]["n"] == 400 and 0.0 < st["step_ms"]["p50"] <= st["step_ms"]["p95"] <= st["step_ms"]["max"]


# ------------------------------------------------------------------------------------- in the child

def test_the_child_serves_the_channel_and_the_a50_is_a_device(cfg):
    proc = PhysicsProcess(cfg).start()
    try:
        port = proc.control_port
        assert port
        with SimCtl(f"127.0.0.1:{port}") as sim:
            hello = sim.hello()
            assert [d["name"] for d in hello["devices"]] == ["a50"] and hello["devices"][0]["tcp"]
            assert sim.device("a50", acoustic_enabled=False)["config"]["acoustic_enabled"] is False
            with pytest.raises(SimCtlError, match="boolean"):
                sim.device("a50", acoustic_enabled=3)
            sim.set_current(speed_mps=0.2, direction_deg=90.0, ramp_s=1.0)
            fly(proc.port, 4000)
            st = sim.state()
            assert st["v_ground_enu"][1] > 0.15
            assert proc.stats()["control"] == {"port": port, "clients": 1}
    finally:
        proc.close()
    assert proc.proc.returncode == 0


def test_the_channel_can_be_switched_off(cfg):
    cfg["physics"]["control"]["enabled"] = False
    proc = PhysicsProcess(cfg).start()
    try:
        assert proc.control_port is None
    finally:
        proc.close()


def test_the_profiles_listen_where_their_clients_are():
    linus = config.load(config.POOL_CONFIG, profile="linus")["physics"]["control"]["bind"]
    work = config.load(config.POOL_CONFIG, profile="workstation")["physics"]["control"]["bind"]
    assert "172.31.250.1" in linus and "127.0.0.1" in linus           # the bridge of deploy/docker/pool/compose.yaml
    assert work == ["127.0.0.1"]                                      # WSL's mirrored loopback
