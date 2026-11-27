"""The pool's docker stack and its tools (deploy/docker/pool/compose.yaml, sim/tools/pool_blueos_setup.py, deploy/hil/windows/pool/): the
addresses the services agree on, line endings that survive a Windows checkout, the setup's wait for the vehicle.
Host only, no docker: the files are read as data."""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim" / "tools"))

from uw import config  # noqa: E402

COMPOSE = yaml.safe_load((ROOT / "deploy/docker/pool/compose.yaml").read_text())
SERVICES = COMPOSE["services"]


def _profile(name):
    return {k for k, s in SERVICES.items() if "profiles" not in s or name in s["profiles"]}


def _ip(service):
    return next(iter(SERVICES[service]["networks"].values()))["ipv4_address"]


def test_each_full_profile_brings_blueos_sitl_video_and_the_dvl():
    assert _profile("blueos-local") >= {"sitl", "mediamtx", "blueos-local", "dvl", "dvl-gateway"}
    assert _profile("work") >= {"sitl", "mediamtx", "blueos", "dvl-host", "dvl-gateway-host"}
    assert not _profile("work") & {"blueos-local", "dvl", "dvl-gateway"}


@pytest.mark.parametrize("ext, gateway", [("dvl", "dvl-gateway"), ("dvl-host", "dvl-gateway-host")])
def test_the_extension_finds_its_gateway_and_the_a50(ext, gateway):
    """host.docker.internal (the extension's BlueOS) is the gateway; DVL_HOST's default port is the A50's HTTP port."""
    e, g = SERVICES[ext], SERVICES[gateway]
    assert e["extra_hosts"] == [f"host.docker.internal:{_ip(gateway)}"]
    assert e["image"] == SERVICES["dvl"]["image"] and e["entrypoint"] == SERVICES["dvl"]["entrypoint"]
    port = config.load(config.POOL_CONFIG)["dvl"]["device"]["http_port"]
    assert e["environment"]["DVL_HOST"].rstrip("}").endswith(f":{port}")
    gw = g["environment"]
    if e.get("network_mode") == "host":         # the Work: BlueOS and the extension on the WSL host
        assert gw["BLUEOS_HOST"] == "172.31.250.1" and gw["DVL_EXTENSION_HOST"] == "172.31.250.1:9001"
        assert SERVICES["blueos"]["network_mode"] == "host"
    else:                                       # linus: all on the bridge
        assert gw["BLUEOS_HOST"] == _ip("blueos-local")
        assert gw["DVL_EXTENSION_HOST"] == f"{_ip(ext)}:9001" and gw["DVL_EXTENSION_IP"] == _ip(ext)


def test_the_a50_binds_where_its_extension_looks():
    """linus: the extension on the bridge reaches the host at 172.31.250.1, so the A50 listens everywhere; the
    Work: the extension is in WSL's host network, and the mirrored loopback reaches Windows' 127.0.0.1."""
    linus = config.load(config.POOL_CONFIG, profile="linus")["dvl"]["device"]["bind"]
    work = config.load(config.POOL_CONFIG, profile="workstation")["dvl"]["device"]["bind"]
    assert linus == "0.0.0.0" and work == "127.0.0.1"
    assert "127.0.0.1:" in SERVICES["dvl-host"]["environment"]["DVL_HOST"]
    assert "172.31.250.1:" in SERVICES["dvl"]["environment"]["DVL_HOST"]


@pytest.mark.parametrize("path", ["deploy/docker/ardusub_sitl/entrypoint.sh", "deploy/docker/ardusub_sitl/params/pool.parm",
                                  "deploy/docker/pool/compose.yaml", "deploy/docker/pool/dvl_gateway.conf.template",
                                  "deploy/hil/windows/pool/stack.sh", "deploy/hil/windows/pool/setup_blueos_wsl.sh"])
def test_files_read_in_linux_stay_lf(path):
    """On the Work the WSL stack runs from the Windows checkout: .gitattributes must pin these to LF, and a
    checkout made before it shows up here (deploy/hil/windows/pool/README.md, 'Fine riga')."""
    assert b"\r" not in (ROOT / path).read_bytes(), f"{path} has CRLF line endings"


class _Heartbeat(BaseHTTPRequestHandler):
    counter = [0]

    def do_GET(self):  # noqa: N802 - http.server's name
        self.counter[0] += self.server.step
        body = json.dumps({"message": {"type": "HEARTBEAT"},
                           "status": {"time": {"counter": self.counter[0]}}}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


@pytest.fixture
def mavlink2rest():
    srv = HTTPServer(("127.0.0.1", 0), _Heartbeat)
    srv.step = 1
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    srv.server_close()


def test_the_setup_waits_for_a_live_heartbeat(mavlink2rest, monkeypatch):
    import pool_blueos_setup as setup
    monkeypatch.setattr(setup.time, "sleep", lambda s: None)
    setup.wait_vehicle(f"http://127.0.0.1:{mavlink2rest.server_port}", timeout_s=5.0)


def test_a_stale_heartbeat_is_not_a_vehicle(mavlink2rest, monkeypatch):
    """mavlink2rest keeps the last heartbeat of a SITL that is gone: a counter that does not move."""
    import pool_blueos_setup as setup
    mavlink2rest.step = 0
    monkeypatch.setattr(setup.time, "sleep", lambda s: None)
    with pytest.raises(SystemExit, match="no live heartbeat"):
        setup.wait_vehicle(f"http://127.0.0.1:{mavlink2rest.server_port}", timeout_s=0.3)


# ------------------------------------------------------------------------------- Nucleus1000 (M12 P5)

def test_the_nucleus_extension_reaches_the_gateway_and_the_emulated_device():
    n = SERVICES["nucleus"]
    assert n["profiles"] == ["nucleus"]
    assert _ip("nucleus") not in {_ip(s) for s in ("blueos-local", "dvl", "dvl-gateway")}
    assert n["environment"]["BLUEOS_URL"] == f"http://{_ip('dvl-gateway')}"
    assert "host.docker.internal:" + _ip("dvl-gateway") in n["extra_hosts"]
    host = n["environment"]["NUCLEUS_HOST"].split(":-")[-1].rstrip("}")
    assert host == COMPOSE["networks"]["blueos_local"]["ipam"]["config"][0]["gateway"]      # the host, where it runs
    cfg = config.load(config.POOL_CONFIG, presets=["nucleus"])
    assert cfg["nucleus"]["device"]["cmd_port"] == 9000 and cfg["nucleus"]["device"]["bind"] == "0.0.0.0"
    assert cfg["dvl"]["enabled"] is False and cfg["nucleus"]["enabled"] is True
    assert "@sha256:" in n["image"]


def test_the_nucleus_parameters_take_position_velocity_and_yaw_from_external_navigation():
    parm = dict(line.split()[:2] for line in (ROOT / "deploy/docker/ardusub_sitl/params/nucleus.parm").read_text().splitlines()
                if line.strip() and not line.startswith("#"))
    assert parm["EK3_SRC1_POSXY"] == parm["EK3_SRC1_VELXY"] == parm["EK3_SRC1_YAW"] == "6"
    assert parm["VISO_TYPE"] == "1"
    assert "SITL_PARAM_FILES" in SERVICES["sitl"]["environment"]["PARAM_FILES"]
