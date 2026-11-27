"""The simulator's control channel (M12 P2): a small TCP server inside the physics process.

What it is for: changing the water current while the vehicle flies, reading the true state for a bench, moving the
vehicle back to the middle between trials, switching a DVL off. All of it without restarting the physics, which
would force a SITL restart too (docs/pool.md: the EKF keeps its old heading otherwise).

**Protocol** (``uwsim/1``): newline-delimited JSON over TCP, one request per line, one reply per request, the stdlib
only (the A50's pattern, ``vehicles/common/a50.py``). Every request may carry an ``id``, echoed in the reply.

    -> {"id": 1, "op": "hello"}
    <- {"id": 1, "ok": true, "result": {"proto": "uwsim/1", "sim_t": .., "pool": {..}, "current": {..}, ...}}
    -> {"op": "set_current", "speed_mps": 0.3, "direction_deg": 90, "ramp_s": 3}      (keys of config ``current:``)
    -> {"op": "get_state"}                       the true state, see ``state()``
    -> {"op": "stream", "rate_hz": 20}           then {"type": "state", ...} lines until the client sends anything
                                                 or closes; at most ``MAX_STREAM_HZ``
    -> {"op": "reposition", "xy_tank": [5, 5]}   horizontal only, depth and attitude kept, twist and thrust zeroed
    -> {"op": "device", "name": "a50", "set": {"acoustic_enabled": false}}     the device's own set_config
    -> {"op": "stats"}                           step timing, packets, stalls, device counters, clients
    errors: {"id": .., "ok": false, "error": "..."}; a line that is not JSON is logged and ignored

**What it does not do:** change the DVL model, the pool or the vehicle file. Those change ports, extensions and
ArduSub parameters: a restart of physics and SITL together, not a runtime knob.

**Threads.** A client thread never writes the vehicle state: the current goes through ``Vehicle.set_current`` and
a reposition through ``PhysicsServer.request_reposition``, both applied by the physics thread between steps. Reads
take the server's lock once (``PhysicsServer.snapshot``).

**Where it listens:** ``physics.control.bind``, a list. An address this host does not have (the docker bridge's
172.31.250.1 before the stack is up) is skipped with a warning, not an error.
"""
from __future__ import annotations

import json
import logging
import math
import select
import socket
import socketserver
import threading
import time

import numpy as np

from . import current as current_mod
from . import frames

log = logging.getLogger("uw.simctl")

PROTO = "uwsim/1"
MAX_STREAM_HZ = 50.0


class SimControl:
    """The control server. ``server`` is the ``PhysicsServer``; ``devices`` maps a name ("a50") to an emulated
    device with ``command()``; ``cfg`` is the run's config (for ``hello``)."""

    def __init__(self, server, cfg: dict, devices: dict | None = None, port: int = 9020, binds=("127.0.0.1",)):
        self.server = server
        self.cfg = cfg
        self.devices = dict(devices or {})
        self.port = int(port)
        self.binds = [str(b) for b in binds]
        self.listening: list[tuple[str, int]] = []
        self._servers: list[socketserver.ThreadingTCPServer] = []
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._clients = 0
        self._clients_lock = threading.Lock()
        self.count = {"requests": 0, "errors": 0, "streams": 0}
        self.t_start = time.monotonic()

    # ------------------------------------------------------------------------------------------ requests
    def handle(self, req: dict) -> dict:
        """One request -> one reply. Pure dispatch, no socket: what the tests call."""
        rid = req.get("id")
        op = req.get("op")
        self.count["requests"] += 1
        try:
            fn = {"hello": self.hello, "set_current": self.set_current, "get_state": self.state,
                  "reposition": self.reposition, "device": self.device, "stats": self.stats}.get(op)
            if fn is None:
                raise ValueError(f"unknown op {op!r} (hello, set_current, get_state, stream, reposition, device, "
                                 "stats)")
            args = {k: v for k, v in req.items() if k not in ("id", "op")}
            out = {"ok": True, "result": fn(**args) if op in ("set_current", "reposition", "device") else fn()}
        except Exception as exc:  # noqa: BLE001 - every failure is an answer, never a dead client thread
            self.count["errors"] += 1
            out = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        if rid is not None:
            out["id"] = rid
        return out

    def hello(self) -> dict:
        from uw.config import get
        state, _, _, _ = self.server.snapshot()
        devs = []
        for name, d in self.devices.items():
            devs.append({"name": name, "kind": type(d).__name__.lower(), "tcp": getattr(d, "tcp_port", None),
                         "http": getattr(d, "http_port", None)})
        return {"proto": PROTO, "sim_t": float(state.t), "rtf": self.server.rtf, "physics_port": self.server.port,
                "pool": {"size": list(get(self.cfg, "pool.size")), "water_level": float(get(self.cfg, "pool.water_level"))},
                "current": self._current_desc(), "devices": devs, "sitl": self.server.sitl_alive()}

    def _current_desc(self):
        cur = self.server.vehicle.current
        return None if cur is None else cur.describe()

    def set_current(self, ramp_s: float = 0.0, **fields) -> dict:
        from uw.config import get
        cur = current_mod.from_dict(fields, water_depth_m=float(get(self.cfg, "pool.water_level")),
                                    seed=int(self.cfg.get("seed", 0)))
        if not (0.0 <= float(ramp_s) <= 600.0):
            raise ValueError(f"ramp_s must be in [0, 600], got {ramp_s}")
        self.server.vehicle.set_current(cur, float(ramp_s))
        state, _, _, _ = self.server.snapshot()
        return {"queued_at_sim_t": float(state.t), "current": cur.describe(), "ramp_s": float(ramp_s)}

    def state(self) -> dict:
        """The true state of the latest step, in the pool frame (ENU, body FLU) and in ArduPilot's (NED, FRD)."""
        srv = self.server
        state, cmd, pwm, (p_tank, rot_flu_enu) = srv.snapshot()
        veh = srv.vehicle
        v_frd, w_frd = state.velocity_frd, state.rate_frd
        v_enu = rot_flu_enu @ frames.vec_flu_frd(v_frd)
        water_enu = frames.vec_enu_ned(veh.current_ned)
        rot_ned = state.rot_frd_ned
        v_rel_frd = v_frd - rot_ned.T @ veh.current_ned
        rpy = frames.rpy_deg_from_rot(rot_flu_enu)
        return {"sim_t": float(state.t), "wall_t": time.time(),
                "p_tank": _r(p_tank), "depth_m": float(state.position_ned[2]),
                "rpy_enu_deg": _r(rpy), "heading_enu_deg": float(rpy[2]),
                "q_frd_ned": _r(state.quat_frd_ned, 9),
                "v_body_frd": _r(v_frd), "w_body_frd": _r(w_frd), "v_ground_enu": _r(v_enu),
                "v_water_enu": _r(water_enu), "v_rel_frd": _r(v_rel_frd),
                "thrust_n": _r(state.thrust_n, 3), "cmd": _r(cmd, 4), "pwm": [int(x) for x in pwm[:16]],
                "packets": srv.packets, "stalls": srv.stalls, "bound_activations": veh.bound_activations,
                "sitl": srv.sitl_alive()}

    def reposition(self, xy_tank) -> dict:
        if len(xy_tank) != 2:
            raise ValueError("reposition takes xy_tank: [x, y] (horizontal only: depth and attitude are kept)")
        self.server.request_reposition(xy_tank)
        return {"xy_tank": [float(v) for v in xy_tank]}

    def device(self, name: str, set: dict | None = None) -> dict:   # noqa: A002 - the protocol's word
        dev = self.devices.get(name)
        if dev is None:
            raise ValueError(f"no device {name!r} (have: {sorted(self.devices)})")
        if not set:
            return {"name": name, "config": dict(getattr(dev, "config", {}))}
        out = dev.command({"command": "set_config", "parameters": dict(set)})
        if not out.get("success", False):
            raise ValueError(f"{name}: {out.get('error_message') or out}")
        return {"name": name, "config": dict(getattr(dev, "config", {}))}

    def stats(self) -> dict:
        srv = self.server
        ms = np.array(srv.step_ms, float)
        out = {"uptime_s": time.monotonic() - self.t_start, "packets": srv.packets, "steps": srv.steps,
               "stalls": srv.stalls, "sitl": srv.sitl_alive(), "rate_hz": srv.rate_hz,
               "step_ms": ({"p50": float(np.percentile(ms, 50)), "p95": float(np.percentile(ms, 95)),
                            "max": float(ms.max()), "n": int(ms.size)} if ms.size else None),
               "control": dict(self.count, clients=self._clients)}
        for name, d in self.devices.items():
            out[name] = {"counts": dict(getattr(d, "count", {})), "clients": getattr(d, "clients", None)}
        return out

    # ---------------------------------------------------------------------------------------- streaming
    def _stream(self, sock, rate_hz: float, fields=None, pending: bytes = b"") -> bytes:
        """Send state lines until the client sends a line (any line: it is the stop) or closes. ``pending``: what
        the client sent after the stream request, already read. Returns what came after the stop line, which
        belongs to the next request."""
        rate = float(rate_hz)
        if not (0.0 < rate <= MAX_STREAM_HZ):
            raise ValueError(f"stream rate_hz must be in (0, {MAX_STREAM_HZ:g}]")
        self.count["streams"] += 1
        period = 1.0 / rate
        got = pending
        if b"\n" in got:                                  # stopped before it started
            return got.split(b"\n", 1)[1]
        sock.settimeout(2.0)                             # a client that stops reading for 2 s is dropped
        t_next = time.monotonic()
        while not self._stop.is_set():
            st = self.state()
            if fields:
                st = {k: st[k] for k in fields if k in st} | {"sim_t": st["sim_t"]}
            sock.sendall((json.dumps(dict(st, type="state")) + "\n").encode())
            t_next += period
            if t_next < time.monotonic() - period:       # fell behind (a slow client): skip, do not burst
                t_next = time.monotonic()
            while True:                                  # until the next sample, listen for the stop line
                left = t_next - time.monotonic()
                ready, _, _ = select.select([sock], [], [], max(left, 0.0))
                if not ready:
                    break
                data = sock.recv(4096)
                if not data:
                    return b""
                got += data
                if b"\n" in got:
                    sock.settimeout(None)
                    return got.split(b"\n", 1)[1]
        return b""

    # -------------------------------------------------------------------------------------- the network
    def start(self) -> "SimControl":
        ctl = self

        class Tcp(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        class Client(socketserver.BaseRequestHandler):
            def handle(self):
                sock = self.request
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with ctl._clients_lock:
                    ctl._clients += 1
                log.info("simctl: client %s:%d connected", *self.client_address[:2])
                buf = b""
                try:
                    while not ctl._stop.is_set():
                        data = sock.recv(4096)
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if not line.strip():
                                continue
                            try:
                                req = json.loads(line)
                                if not isinstance(req, dict):
                                    raise ValueError("not an object")
                            except ValueError:
                                log.warning("simctl: not a JSON object from %s: %r", self.client_address[0], line[:80])
                                continue
                            if req.get("op") == "stream":
                                try:
                                    rest = ctl._stream(sock, req.get("rate_hz", 10.0), req.get("fields"), buf)
                                except ValueError as exc:
                                    sock.sendall((json.dumps({"id": req.get("id"), "ok": False,
                                                              "error": f"ValueError: {exc}"}) + "\n").encode())
                                    continue
                                buf = rest                # the stop line is gone, the rest stays
                                continue
                            sock.sendall((json.dumps(ctl.handle(req)) + "\n").encode())
                except OSError:
                    pass
                finally:
                    with ctl._clients_lock:
                        ctl._clients -= 1
                    log.info("simctl: client %s:%d gone", *self.client_address[:2])

        for addr in self.binds:
            try:
                srv = Tcp((addr, self.port), Client)
            except OSError as exc:
                if exc.errno in (99, 10049):            # EADDRNOTAVAIL (Linux / Windows): not on this host (yet)
                    log.warning("simctl: %s is not an address of this host, not listening there", addr)
                    continue
                self.close()
                raise OSError(exc.errno, f"simctl: cannot listen on {addr}:{self.port} ({exc.strerror}): is another "
                                         "uw pool or physics_proc running? (physics.control.port)") from exc
            if self.port == 0:                          # tests: one ephemeral port, the same on every address
                self.port = srv.server_address[1]
            self._servers.append(srv)
            self.listening.append((addr, srv.server_address[1]))
            th = threading.Thread(target=srv.serve_forever, name=f"simctl-{addr}", daemon=True)
            th.start()
            self._threads.append(th)
        if not self._servers:
            log.warning("simctl: no address to listen on (physics.control.bind %s): the control channel is off",
                        self.binds)
        else:
            log.info("simctl: control channel (%s) on %s", PROTO, ", ".join(f"{a}:{p}" for a, p in self.listening))
        return self

    @property
    def clients(self) -> int:
        with self._clients_lock:
            return self._clients

    def close(self) -> None:
        self._stop.set()
        for srv in self._servers:
            srv.shutdown()
            srv.server_close()
        self._servers.clear()


def _r(a, nd: int = 6) -> list:
    return [round(float(x), nd) if math.isfinite(float(x)) else None for x in np.asarray(a, float).ravel()]


def from_config(cfg: dict, server, devices: dict | None = None) -> SimControl | None:
    """The control channel of config ``physics.control`` (None when it is off or absent)."""
    from uw.config import ConfigError, get
    try:
        c = get(cfg, "physics.control")
    except ConfigError:
        return None
    if not c or not c.get("enabled", True):
        return None
    binds = c.get("bind") or ["127.0.0.1"]
    if isinstance(binds, str):
        binds = [binds]
    return SimControl(server, cfg, devices, port=int(c.get("port", 9020)), binds=binds)
