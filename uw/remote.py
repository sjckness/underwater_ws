"""Commands on a target machine: over SSH (the vehicle's Pi, the Thor), or locally (`transport: local`, the rov-sim
test environment on linus). Also the vehicle's state, read through BlueOS's mavlink2rest (HTTP, read-only).

Nothing here sends MAVLink: the armed check is a GET of the last HEARTBEAT that BlueOS already received.
"""
from __future__ import annotations

import json
import shlex
import subprocess
import time
import urllib.error
import urllib.request

#: MAV_MODE_FLAG_SAFETY_ARMED in HEARTBEAT.base_mode
ARMED = 128


class RemoteError(RuntimeError):
    pass


class Remote:
    def __init__(self, target: dict, dry_run: bool = False, echo=print):
        self.t = target
        self.local = target.get("transport", "ssh") == "local"
        self.dry = dry_run
        self.echo = echo

    # ------------------------------------------------------------------------------------------ shell
    def ssh_argv(self) -> list[str]:
        t = self.t
        argv = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={int(t.get('ssh_timeout_s', 8))}"]
        if t.get("ssh_port"):
            argv += ["-p", str(t["ssh_port"])]
        argv += [str(o) for o in t.get("ssh_options", [])]
        return argv + [f"{t['user']}@{t['host']}" if t.get("user") else str(t["host"])]

    def path(self, p: str) -> str:
        """A path on the target: ``~`` expanded here for a local target (an SSH target gets absolute paths)."""
        import os
        return os.path.expanduser(str(p)) if self.local else str(p)

    def where(self) -> str:
        return "local" if self.local else f"{self.t.get('user', '')}@{self.t['host']}"

    def sh(self, script: str, check: bool = True, capture: bool = True, mutating: bool = False,
           timeout: float | None = 120) -> str:
        """Run a shell snippet on the target. ``mutating`` steps are only printed under --dry-run."""
        if self.dry and mutating:
            self.echo(f"  [dry-run] {self.where()}$ {script}")
            return ""
        argv = ["bash", "-c", script] if self.local else [*self.ssh_argv(), script]
        try:
            p = subprocess.run(argv, text=True, capture_output=capture, timeout=timeout)
        except subprocess.TimeoutExpired as e:
            raise RemoteError(f"{self.where()}: timed out after {timeout} s: {script}") from e
        if check and p.returncode != 0:
            err = (p.stderr or p.stdout or "").strip() if capture else ""
            raise RemoteError(f"{self.where()}$ {script}\n  exit {p.returncode}: {err[-800:]}")
        return p.stdout if capture else ""

    def docker(self) -> str:
        return str(self.t.get("docker", "docker"))

    def exec_in(self, container: str, script: str, **kw) -> str:
        """A bash -lc snippet inside a container on the target."""
        return self.sh(f"{self.docker()} exec {shlex.quote(container)} bash -lc {shlex.quote(script)}", **kw)

    def rsync(self, src: str, dst: str, extra: list[str] = (), pull: bool = False, sudo: bool = False) -> None:
        """rsync between here and the target. ``pull``: dst is local, src is on the target. Never deletes on a pull."""
        argv = ["rsync", "-a", "--human-readable", *extra]
        if self.dry:
            argv.append("--dry-run")
        if sudo and not self.local:
            argv += ["--rsync-path", "sudo -n rsync"]
        if self.local:
            pre_src, pre_dst = src, dst
            if sudo:
                argv = ["sudo", "-n", *argv]
        else:
            argv += ["-e", shlex.join(self.ssh_argv()[:-1])]
            host = self.ssh_argv()[-1]
            pre_src, pre_dst = (f"{host}:{src}", dst) if pull else (src, f"{host}:{dst}")
        argv += [pre_src, pre_dst]
        self.echo("  $ " + shlex.join(argv))
        p = subprocess.run(argv, text=True)
        if p.returncode != 0:
            raise RemoteError(f"rsync failed with exit {p.returncode}")


# ---------------------------------------------------------------------------------------- vehicle state
def heartbeat(blueos_url: str, timeout: float = 4.0) -> dict:
    """The last HEARTBEAT of vehicle 1, component 1, as mavlink2rest holds it."""
    url = blueos_url.rstrip("/") + "/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise RemoteError(f"no HEARTBEAT from {url}: {e}") from e


def vehicle_state(blueos_url: str, live_window_s: float = 2.5) -> dict:
    """``{armed, live, system_status, custom_mode, frequency_hz}`` from two reads ``live_window_s`` apart.

    ``live`` means mavlink2rest's message counter moved between the two reads, i.e. ArduSub is talking now — not
    an old HEARTBEAT kept from before a reboot. ``armed`` is taken from the second read.
    """
    a = heartbeat(blueos_url)
    t0 = time.monotonic()
    time.sleep(live_window_s)
    b = heartbeat(blueos_url)
    n = b.get("status", {}).get("time", {}).get("counter", 0) - a.get("status", {}).get("time", {}).get("counter", 0)
    msg = b["message"]
    return {
        "armed": bool(int(msg["base_mode"]["bits"]) & ARMED),
        "live": n > 0,
        "system_status": msg.get("system_status", {}).get("type"),
        "custom_mode": msg.get("custom_mode"),
        "frequency_hz": round(n / (time.monotonic() - t0), 2),      # over the window, not mavlink2rest's average
    }


def require_disarmed(target: dict, echo=print) -> dict:
    """Stop unless ArduSub is live and DISARMED. There is no override: a deploy waits for the vehicle, not the other
    way round."""
    url = target.get("blueos_url") or f"http://{target['host']}"
    s = vehicle_state(url)
    echo(f"  vehicle: {'ARMED' if s['armed'] else 'disarmed'}, heartbeat {'live' if s['live'] else 'NOT live'} "
         f"({s['system_status']}, {s['frequency_hz']} Hz) via {url}")
    if s["armed"]:
        raise RemoteError("the vehicle is ARMED: disarm it first (nothing was changed)")
    if not s["live"]:
        raise RemoteError("no live HEARTBEAT from ArduSub: cannot confirm the vehicle is disarmed (nothing was changed)")
    return s
