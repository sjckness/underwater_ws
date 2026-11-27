"""uw deploy / smoke / logs / sysid / targets: the safety rules and the bookkeeping, without a vehicle (host, stdlib).

A fake mavlink2rest (http.server on localhost) plays BlueOS: its HEARTBEAT counter moves or not, and its base_mode
says armed or not. The target is local, in tmp_path, with a fake `docker` that only logs what it was asked.
"""
import json
import os
import stat
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from types import SimpleNamespace

import pytest
import yaml

from sysid import cli as sysid
from uw import deploy, logs, remote, targets


class FakeBlueOS:
    def __init__(self, armed=False, live=True):
        self.armed, self.live, self.counter = armed, live, 100
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT":
                    self.send_response(404)
                    self.end_headers()
                    return
                if outer.live:
                    outer.counter += 1
                body = json.dumps({"message": {"type": "HEARTBEAT", "base_mode": {"bits": 81 | (128 if outer.armed else 0)},
                                               "custom_mode": 19, "system_status": {"type": "MAV_STATE_STANDBY"}},
                                   "status": {"time": {"counter": outer.counter, "frequency": 1.0}}}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):
                pass

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        self.srv.shutdown()


@pytest.fixture
def blueos(monkeypatch):
    monkeypatch.setattr(remote.time, "sleep", lambda s: None)     # the live window, without waiting
    b = FakeBlueOS()
    yield b
    b.close()


@pytest.fixture
def fake_docker(tmp_path, monkeypatch):
    """A `docker` on PATH that records its arguments and answers `ps` like the Pi would."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    log = tmp_path / "docker.log"
    script = bindir / "docker"
    script.write_text(f"""#!/bin/bash
echo "$@" >> {log}
case "$1" in
  ps) printf 'blueos-core\\tbluerobotics/blueos-core:1.4.7\\nextension-itskalvikblueosros2010\\titskalvik/blueos-ros2:0.1.0\\n';;
  inspect) echo '[{{"Source":"{tmp_path}/persist","Destination":"/root/persistent_ws"}}]';;
esac
""")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    return log


def target(tmp_path, url, **kw):
    t = {"name": "t", "kind": "rov", "transport": "local", "host": "localhost", "blueos_url": url, "docker": "docker",
         "sudo": "", "persistent_host": str(tmp_path / "persist"), "persistent_container": "/root/persistent_ws",
         "workspace": "ros2_ws", "packages": ["rov_bringup"], "colcon_args": [], "container": "",
         "logs": {"dest": str(tmp_path / "pulled"), "index": str(tmp_path / "INDEX.md"),
                  "bin_dir": str(tmp_path / "pi_bin"), "bag_dir": str(tmp_path / "pi_bags")}}
    t.update(kw)
    return t


def args(**kw):
    a = dict(dry_run=False, yes=True, discover=False, start=False, no_restart=False)
    a.update(kw)
    return SimpleNamespace(**a)


# ------------------------------------------------------------------------------------------------ armed check
def test_the_armed_bit_and_a_live_heartbeat(blueos):
    s = remote.vehicle_state(blueos.url)
    assert s["live"] and not s["armed"]
    blueos.armed = True
    assert remote.vehicle_state(blueos.url)["armed"]


@pytest.mark.parametrize("armed,live,match", [(True, True, "ARMED"), (False, False, "no live HEARTBEAT")])
def test_require_disarmed_refuses(blueos, armed, live, match):
    blueos.armed, blueos.live = armed, live
    with pytest.raises(remote.RemoteError, match=match):
        remote.require_disarmed({"blueos_url": blueos.url, "host": "x"}, echo=lambda *_: None)


def test_no_blueos_means_no_deploy():
    with pytest.raises(remote.RemoteError, match="no HEARTBEAT"):
        remote.require_disarmed({"blueos_url": "http://127.0.0.1:9", "host": "x"}, echo=lambda *_: None)


# ------------------------------------------------------------------------------------------------ deploy rov
def test_an_armed_vehicle_stops_the_deploy_before_anything_changes(blueos, fake_docker, tmp_path):
    blueos.armed = True
    with pytest.raises(remote.RemoteError, match="ARMED"):
        deploy.deploy_rov(target(tmp_path, blueos.url, container="extension-x"), args())
    assert not (tmp_path / "persist").exists(), "nothing copied"
    assert not fake_docker.exists() or "restart" not in fake_docker.read_text()


def test_deploy_copies_builds_and_restarts_only_the_extension(blueos, fake_docker, tmp_path, monkeypatch):
    monkeypatch.setattr(targets, "update_local", lambda name, values: tmp_path / "t.local")
    assert deploy.deploy_rov(target(tmp_path, blueos.url), args(discover=True)) == 0
    assert (tmp_path / "persist" / "ros2_ws" / "src" / "rov_bringup" / "package.xml").is_file()
    calls = fake_docker.read_text().splitlines()
    assert any(c.startswith("exec extension-itskalvikblueosros2010 bash -lc") and "colcon build" in c for c in calls)
    restarts = [c for c in calls if c.startswith("restart")]
    assert restarts == ["restart extension-itskalvikblueosros2010"], "never blueos-core"


def test_dry_run_changes_nothing(blueos, fake_docker, tmp_path):
    assert deploy.deploy_rov(target(tmp_path, blueos.url, container="extension-x"), args(dry_run=True)) == 0
    assert not (tmp_path / "persist" / "ros2_ws").exists()
    calls = fake_docker.read_text().splitlines() if fake_docker.exists() else []
    assert not [c for c in calls if c.split()[0] in ("exec", "restart")]


def test_discover_needs_exactly_one_extension(fake_docker, tmp_path):
    r = remote.Remote(target(tmp_path, "http://x", extension_image="no-such-image"))
    with pytest.raises(remote.RemoteError, match="expected one running container"):
        deploy.discover(r, r.t)


# ------------------------------------------------------------------------------------------------ logs
def test_logs_pull_copies_never_overwrites_and_indexes_once(tmp_path):
    (tmp_path / "pi_bin").mkdir()
    (tmp_path / "pi_bin" / "00000001.BIN").write_bytes(b"\xa3\x95" + b"x" * 100)
    bag = tmp_path / "pi_bags" / "2026-01-01_light"
    bag.mkdir(parents=True)
    (bag / "a_0.mcap").write_bytes(b"m" * 10)
    (bag / "metadata.yaml").write_text("x: 1\n")
    t = target(tmp_path, "http://x")
    ns = SimpleNamespace(dry_run=False, only=None)
    assert logs.pull(t, ns) == 0
    rows = [ln for ln in (tmp_path / "INDEX.md").read_text().splitlines() if ln.startswith("| 20")]
    assert len(rows) == 2 and "`00000001.BIN`" in rows[0] and "`2026-01-01_light`" in rows[1]
    (tmp_path / "pulled" / "bin" / "00000001.BIN").write_bytes(b"kept")
    assert logs.pull(t, ns) == 0
    assert (tmp_path / "pulled" / "bin" / "00000001.BIN").read_bytes() == b"kept", "--ignore-existing"
    assert len([ln for ln in (tmp_path / "INDEX.md").read_text().splitlines() if ln.startswith("| 20")]) == 2
    assert (tmp_path / "pi_bin" / "00000001.BIN").exists(), "a pull never deletes on the vehicle"


def test_the_real_index_is_the_default_and_rov_sim_does_not_use_it():
    assert logs.index_path({"logs": {}}) == logs.INDEX
    sim = yaml.safe_load((targets.DIR / "rov-sim.example").read_text())
    assert logs.index_path(sim) != logs.INDEX


# ------------------------------------------------------------------------------------------------ targets
def test_local_overrides_example_key_by_key(tmp_path, monkeypatch):
    monkeypatch.setattr(targets, "DIR", tmp_path)
    (tmp_path / "x.example").write_text("host: a\nsmoke: {cpu_max_percent: 80, temp_max_c: 75}\n")
    (tmp_path / "x.local").write_text("smoke: {cpu_max_percent: 50}\n")
    t = targets.load("x")
    assert t["host"] == "a" and t["smoke"] == {"cpu_max_percent": 50, "temp_max_c": 75}


def test_shipped_rov_targets_are_complete():
    for name in ("rov", "rov-sim"):
        t = yaml.safe_load((targets.DIR / f"{name}.example").read_text())
        for key in ("blueos_url", "container", "persistent_host", "packages", "fcu_url", "smoke", "logs"):
            assert key in t, f"{name}.example: {key}"
        for p in t["packages"]:
            assert (targets.WS / "ros" / "src" / p / "package.xml").is_file()


# ------------------------------------------------------------------------------------------------ sysid
def test_sysid_write_replaces_only_the_estimated_line(tmp_path):
    src = targets.WS / "vehicles" / "bluerov2_heavy" / "vehicle.yaml"
    p = tmp_path / "vehicle.yaml"
    p.write_text(src.read_text())
    done = sysid.write_estimates(p, {"damping_linear.X_u": (-15.5, "dive 2026-12-01")}, "sysid test")
    assert done == ["damping_linear.X_u"]
    old, new = src.read_text().splitlines(), p.read_text().splitlines()
    changed = [(a, b) for a, b in zip(old, new) if a != b]
    assert len(old) == len(new) and len(changed) == 1
    assert changed[0][1].startswith("  X_u: -15.5  # sysid test: dive 2026-12-01 (was -13.7)")
    assert yaml.safe_load(p.read_text())["damping_linear"]["X_u"] == -15.5


def test_sysid_without_write_never_touches_the_vehicle(tmp_path):
    before = (targets.WS / "vehicles" / "bluerov2_heavy" / "vehicle.yaml").read_bytes()
    assert sysid.main(["bluerov2_heavy", "--logs", str(tmp_path), "--out", str(tmp_path / "out")]) == 0
    assert (targets.WS / "vehicles" / "bluerov2_heavy" / "vehicle.yaml").read_bytes() == before
    assert len(list((tmp_path / "out").glob("bluerov2_heavy_*.md"))) == 1
