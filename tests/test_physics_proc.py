"""uw/rov/physics_proc.py: the physics server in its own process, read through shared memory (host, numpy only).

A fake SITL client speaks the JSON protocol to the child, as in tests/test_physics_udp.py.
"""
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uw import config  # noqa: E402
from uw.physics_udp import MAGIC_16  # noqa: E402
from uw.rov.physics_proc import PhysicsProcess, make_pool_server  # noqa: E402


@pytest.fixture
def cfg():
    c = config.load(config.POOL_CONFIG)
    c["physics"]["port"] = 0           # ephemeral: never collides with a real SITL on 9002
    c["physics"]["rtf"] = 0.0          # unpaced
    c["dvl"]["device"].update(bind="127.0.0.1", tcp_port=0, http_port=0)    # the emulated A50, likewise
    return c


def servo(pwm, frame=0):
    pwm = list(pwm) + [1500] * (16 - len(pwm))
    return struct.pack("<HHI16H", MAGIC_16, 400, frame, *pwm)


def fly(port, pwm, n):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    try:
        for i in range(n):
            s.sendto(servo(pwm, i), ("127.0.0.1", port))
            s.recvfrom(4096)
    finally:
        s.close()


def test_pool_server_starts_floating_at_the_configured_pose(cfg):
    srv = make_pool_server(cfg)
    p, r = srv.body_pose()
    x, y, z, yaw = cfg["vehicle"]["start"]
    assert np.allclose(p, [x, y, z], atol=1e-9)
    assert np.isclose(np.degrees(np.arctan2(r[1, 0], r[0, 0])), yaw, atol=1e-6)
    assert srv.vehicle.surface_model and srv.vehicle.params.n_thrusters == 8


def test_the_child_flies_and_the_parent_reads_it(cfg):
    proc = PhysicsProcess(cfg).start()
    try:
        assert proc.port and proc.error is None
        p0, _ = proc.body_pose()
        # a climb demand from the surface would leave the water: dive instead, verticals above 1500 (ArduSub's
        # climb factor is -1), with the tilt servo (SERVO9) and the lights (SERVO10) somewhere recognisable
        aux = [1500] * 8 + [1700, 1300]
        fly(proc.port, [1500, 1500, 1500, 1500, 1800, 1800, 1800, 1800] + aux[8:], 800)
        p1, r1 = proc.body_pose()
        assert p1[2] < p0[2] - 0.3, f"it must dive: z {p0[2]:.3f} -> {p1[2]:.3f}"
        assert np.allclose(r1 @ r1.T, np.eye(3), atol=1e-9)
        assert proc.aux_pwm(9) == 1700 and proc.aux_pwm(10) == 1300
        st = proc.stats()
        assert st["packets"] == 800 and st["t"] == pytest.approx(2.0)
        v, w, t = proc.body_twist()
        assert t == pytest.approx(2.0) and v[2] > 0.05          # FRD: diving is +z
    finally:
        proc.close()
    assert proc.proc.returncode == 0, "the child must exit cleanly when the parent closes its stdin"


def test_a_busy_port_fails_the_start_with_a_clear_error(cfg):
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("127.0.0.1", 0))
    cfg["physics"]["port"] = blocker.getsockname()[1]
    try:
        t0 = time.monotonic()
        with pytest.raises(RuntimeError, match="physics process"):
            PhysicsProcess(cfg).start()
        assert time.monotonic() - t0 < 15.0
    finally:
        blocker.close()


def test_the_child_runs_the_a50_and_reports_it(cfg):
    """The emulated DVL lives in the child (uw/rov/a50.py): reachable on its ports, counted in the shared record."""
    import json
    import urllib.request
    proc = PhysicsProcess(cfg).start()
    try:
        tcp, http = proc.dvl_ports
        about = json.load(urllib.request.urlopen(f"http://127.0.0.1:{http}/api/v1/about", timeout=3))
        assert "DVL" in about["product_name"]
        client = socket.create_connection(("127.0.0.1", tcp), timeout=3)
        time.sleep(0.3)
        fly(proc.port, [1500] * 8, 800)                     # 2 s of simulated time: a few reports at 2-15 Hz
        time.sleep(0.3)
        data = client.recv(1 << 16).decode()
        client.close()
        kinds = {json.loads(line)["type"] for line in data.splitlines() if line.strip()}
        assert {"velocity", "position_local"} <= kinds
        dvl = proc.stats()["dvl"]
        assert dvl["reports"] >= 2 and dvl["locked"] == dvl["reports"]
    finally:
        proc.close()
    assert proc.proc.returncode == 0


def test_without_a_dvl_the_child_has_no_ports(cfg):
    cfg["dvl"]["enabled"] = False
    proc = PhysicsProcess(cfg).start()
    try:
        assert proc.dvl_ports is None and proc.stats()["dvl"]["reports"] == 0
    finally:
        proc.close()


def test_the_current_reaches_the_child_and_carries_the_vehicle(cfg):
    """config current: -> stdin JSON -> make_pool_server -> the Fossen model: a disarmed vehicle at the surface
    drifts with the water (0.3 m/s towards +y), and the DVL's velocity source (the body twist) sees it."""
    cfg["dvl"]["enabled"] = False
    cfg["current"].update(speed_mps=0.3, direction_deg=90.0)
    proc = PhysicsProcess(cfg).start()
    try:
        p0, _ = proc.body_pose()
        fly(proc.port, [1500] * 8, 8000)                    # 20 s, neutral PWM: thrusters off
        p1, r1 = proc.body_pose()
        v, _, t = proc.body_twist()
        v_tank = r1 @ (v * np.array([1.0, -1.0, -1.0]))     # FRD -> FLU, then into the tank
        assert t == pytest.approx(20.0)
        assert v_tank[1] > 0.25 and abs(v_tank[0]) < 0.02, v_tank
        assert p1[1] - p0[1] > 3.0 and abs(p1[0] - p0[0]) < 0.3
    finally:
        proc.close()
    assert proc.proc.returncode == 0
