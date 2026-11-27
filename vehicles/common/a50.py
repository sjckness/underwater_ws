"""The Water Linked DVL A50's network side, emulated (M11 pool): its TCP JSON protocol and the HTTP API.

The beams, the noise and the bottom lock are ``uw.rov.dvl`` (docs/dvl.md); this module is what a client on the
vehicle's network sees of the device, so that the software that drives a real A50 drives the simulated one
unmodified: BlueOS's Water Linked DVL extension (which turns the reports into VISION_POSITION_DELTA for ArduSub,
github.com/bluerobotics/BlueOS-Water-Linked-DVL) and the ``ndahn/dvl_a50`` ROS driver.

    TCP 16171, newline-delimited JSON, format json_v3.3 (docs.waterlinked.com/dvl/dvl-json-protocol, VERIFIED):
      -> {"type": "velocity", ...}          every velocity report, 2-15 Hz with the altitude (uw.rov.dvl)
      -> {"type": "position_local", ...}    the device's own dead reckoning, 5 Hz
      <- {"command": "get_config" | "set_config" | "reset_dead_reckoning" | "calibrate_gyro" | "trigger_ping"
                     | "get_version_info" | "get_time_status" | ..., "parameters": {...}}
      -> {"response_to": ..., "success": ..., "error_message": ..., "result": ..., "format", "type": "response"}
    HTTP (port 80 on the device, dvl.device.http_port here), what the extension reads (its source, v1.0.11):
      GET /api/v1/about           {"product_name": "DVL A50", ...}  the extension looks for "DVL" in product_name
      GET /api/v1/about/status    {"temperature": ...}              it warns above 45 C
      GET /api/v1/config          the protocol config
    The HTTP API is not in Water Linked's documentation: its fields other than those two are GUESSES, as are the
    JSON stand-ins this emulator uses for the model's "no value" (fom, altitude, distance -1).

Time: the real device stamps Unix time; so does this one (wall clock: the physics process is paced to it), so a
client sees what it would see on the vehicle. The model's own reports (``uw.rov.dvl.Report``) carry simulated time.

Threads. ``step`` runs in the physics thread at every servo packet and never blocks: it samples when a report is
due and puts the JSON on a queue. A sender thread writes it to every client; one thread per client reads
commands; the HTTP server has its own. Nothing here imports Isaac.
"""
from __future__ import annotations

import datetime as _dt
import http.server
import json
import logging
import math
import queue
import socket
import socketserver
import threading
import time

import numpy as np

from ..config import get
from . import frames as fr

log = logging.getLogger("uw.a50")

FORMAT = "json_v3.3"
MAX_TRIGGERS = 15                         # trigger_ping queue depth. VERIFIED (JSON protocol)
F_FRD_FLU = np.diag([1.0, -1.0, -1.0])    # body FRD <-> FLU
E_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
CONFIG_KEYS = ("speed_of_sound", "acoustic_enabled", "dark_mode_enabled", "mounting_rotation_offset",
               "range_mode", "periodic_cycling_enabled")


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _euler_zyx(r: np.ndarray):
    """roll, pitch, yaw [rad] of a frame -> NED rotation (z-y-x, as ArduPilot and the A50 report them)."""
    return (math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))), math.atan2(r[1, 0], r[0, 0]))


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _finite(x, stand_in: float = -1.0) -> float:
    x = float(x)
    return x if math.isfinite(x) else stand_in


class A50:
    """The device. ``step`` from the physics thread, ``start`` / ``close`` around it."""

    def __init__(self, dvl, cfg: dict):
        self.dvl = dvl
        d = get(cfg, "dvl.device")
        self.bind = str(d["bind"])
        self.tcp_port = int(d["tcp_port"])
        self.http_port = int(d["http_port"])
        self.dr_period = 1.0 / float(d["dead_reckoning_hz"])
        self.temperature_c = float(d["temperature_c"])
        self.yaw_drift = math.radians(float(d["yaw_drift_deg_per_h"])) / 3600.0
        self.true_sound_speed = float(dvl.p.sound_speed_mps)
        self.version = {"chipid": "0x0000000000000000", "hardware_revision": 4, "product_id": 21035,
                        "product_name": "DVL A50", "variant": str(get(cfg, "dvl.version")),
                        "version_short": "2.7.2", "version": "2.7.2 (uw simulator)", "is_ready": True}
        self._lock = threading.Lock()           # config, triggers and the reset flag (physics vs client threads)
        self.config = {"speed_of_sound": 1475.0, "acoustic_enabled": True, "dark_mode_enabled": False,
                       "mounting_rotation_offset": 0.0, "range_mode": "auto", "periodic_cycling_enabled": True}
        self._triggers = 0
        self._reset = True                      # the first step starts the dead-reckoning run
        self._dr = {"p": np.zeros(3), "var": 0.0, "yaw0": 0.0, "t0": 0.0}
        self._next_dr = 0.0
        self._last_ping = None                  # sim time of the previous ping (the first: the device's power-on)
        self._out: queue.Queue = queue.Queue(maxsize=256)
        self._clients: dict = {}                # socket -> its send lock
        self._clients_lock = threading.Lock()
        self.count = {"velocity": 0, "locked": 0, "position_local": 0, "commands": 0, "dropped": 0}
        self._tcp = self._http = None
        self._threads: list = []
        self._stop = threading.Event()

    # ---------------------------------------------------------------------------------- the physics side

    def _frames(self, rot_flu_enu, offset_deg: float):
        """(instrument -> NED, output frame -> NED, output <- instrument). The output frame is the instrument's
        turned by the mounting_rotation_offset setting, as the device reports in (the vehicle's when the setting
        matches the mount); the rotation follows ``uw.rov.dvl.mount_from_config``'s sign."""
        r_inst_ned = E_ENU_NED @ np.asarray(rot_flu_enu, float) @ F_FRD_FLU @ self.dvl.mount.rot_body_to_dvl.T
        r_set = fr.rot_from_rpy_deg([0.0, 0.0, -float(offset_deg)])
        return r_inst_ned, r_inst_ned @ r_set.T, r_set

    def step(self, t: float, pose, state) -> None:
        """At every physics step: a velocity report when one is due (or triggered), the dead reckoning at 5 Hz."""
        with self._lock:
            cfg = dict(self.config)
            reset, self._reset = self._reset, False
            ping = cfg["acoustic_enabled"] and self.dvl.due(t)
            if not cfg["acoustic_enabled"] and self._triggers > 0:
                self._triggers -= 1
                ping = True
        if self._last_ping is None:
            self._last_ping = float(t)
        r_inst_ned, r_out_ned, r_set = self._frames(pose[1], cfg["mounting_rotation_offset"])
        roll, pitch, yaw = _euler_zyx(r_out_ned)
        if reset:
            self._dr = {"p": np.zeros(3), "var": 0.0, "yaw0": yaw, "t0": float(t)}
            self._next_dr = float(t)
        yaw_dvl = yaw - self._dr["yaw0"] + self.yaw_drift * (float(t) - self._dr["t0"])
        if ping:
            rep = self.dvl.sample(t, pose[0], pose[1], state.velocity_frd, state.rate_frd)
            # "time" is the interval since the previous report, never 0: the model's first report has none, and a
            # 0 here became time_delta_usec 0 in the extension's VISION_POSITION_DELTA, a division by zero
            dt = float(t) - self._last_ping
            self._last_ping = float(t)
            msg, v_out = self._velocity_message(rep, cfg, r_set, dt)
            if rep.velocity_valid and dt > 0.0:
                r_dr = _rz(yaw_dvl) @ _rz(-yaw) @ r_out_ned          # the device's heading, true roll and pitch
                self._dr["p"] = self._dr["p"] + r_dr @ v_out * dt
                self._dr["var"] += (msg["fom"] * dt) ** 2
            self.count["velocity"] += 1
            self.count["locked"] += int(rep.velocity_valid)
            self._emit(msg)
        if float(t) + 1e-9 >= self._next_dr:
            self._next_dr = float(t) + self.dr_period
            p = self._dr["p"]
            self._emit({"ts": time.time(), "x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
                        "std": math.sqrt(self._dr["var"]), "roll": math.degrees(roll), "pitch": math.degrees(pitch),
                        "yaw": _wrap_deg(math.degrees(yaw_dvl)), "type": "position_local", "status": 0,
                        "format": FORMAT})
            self.count["position_local"] += 1

    def _velocity_message(self, rep, cfg: dict, r_set: np.ndarray, dt: float):
        """The report as the device sends it: Doppler scaled by the configured speed of sound, turned into the
        output frame, Unix time stamps, finite stand-ins where the model has no value."""
        k = float(cfg["speed_of_sound"]) / self.true_sound_speed
        msg = rep.as_json()
        v = r_set @ (np.asarray(rep.velocity, float) * k)
        msg["vx"], msg["vy"], msg["vz"] = (float(x) for x in v)
        if rep.velocity_valid:
            cov = r_set @ (np.asarray(rep.covariance, float) * k * k) @ r_set.T
            msg["covariance"] = [[float(x) for x in row] for row in cov]
            msg["fom"] = float(rep.fom) * k
        else:
            msg["fom"] = -1.0
        msg["altitude"] = _finite(rep.altitude)
        for tr in msg["transducers"]:
            tr["velocity"] = float(tr["velocity"]) * k
            tr["distance"] = -1.0 if tr["distance"] is None else float(tr["distance"])
        msg["time"] = round(dt * 1e3, 3)
        msg["time_of_validity"] = int(time.time() * 1e6)
        return msg, v

    def _emit(self, msg: dict) -> None:
        try:
            self._out.put_nowait(msg)
        except queue.Full:
            self.count["dropped"] += 1

    # ---------------------------------------------------------------------------------- the protocol

    def command(self, req) -> dict:
        """One request (a decoded JSON object) -> the response object."""
        name = str(req.get("command", "")) if isinstance(req, dict) else ""
        params = req.get("parameters") if isinstance(req, dict) else None
        result, error = None, ""
        with self._lock:
            self.count["commands"] += 1
            if name == "get_config":
                result = dict(self.config)
            elif name == "set_config":
                error = self._set_config(params if isinstance(params, dict) else {})
            elif name == "reset_dead_reckoning":
                self._reset = True
            elif name == "calibrate_gyro":
                pass                             # the simulated gyro needs none
            elif name == "trigger_ping":
                if self.config["acoustic_enabled"]:
                    error = "trigger_ping requires acoustic_enabled to be false"
                elif self._triggers >= MAX_TRIGGERS:
                    error = f"trigger queue full ({MAX_TRIGGERS})"
                else:
                    self._triggers += 1
            elif name == "get_version_info":
                result = dict(self.version)
            elif name == "get_time_status":
                result = self._time_status()
            elif name == "get_time_ntp":
                result = {"ntp_address": "auto"}
            elif name in ("set_time_ntp", "set_time_manual"):
                pass                             # the host's clock is the device's
            elif name == "force_sync_ntp":
                result = {"success": False, "message": "not simulated", "status": self._time_status()}
            else:
                error = f"unknown command '{name}'"
        return {"response_to": name, "success": not error, "error_message": error, "result": result,
                "format": FORMAT, "type": "response"}

    def _set_config(self, params: dict) -> str:
        new = dict(self.config)
        for key, value in params.items():
            if key not in CONFIG_KEYS:
                return f"unknown parameter '{key}'"
            if key == "speed_of_sound":
                if not isinstance(value, (int, float)) or not 1000.0 <= float(value) <= 2000.0:
                    return "speed_of_sound must be 1000-2000 m/s"
                new[key] = float(value)
            elif key == "mounting_rotation_offset":
                if not isinstance(value, (int, float)) or not 0.0 <= float(value) <= 360.0:
                    return "mounting_rotation_offset must be 0-360 degrees"
                new[key] = float(value)
            elif key == "range_mode":
                if value != "auto":
                    return f"range_mode '{value}' is not simulated (only 'auto', bottom tracking)"
                new[key] = value
            else:
                if not isinstance(value, bool):
                    return f"{key} must be a boolean"
                new[key] = value
        self.config = new
        return ""

    @staticmethod
    def _time_status() -> dict:
        now = _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        return {"system_time": now, "ntp_synced": False, "ntp_synced_to": "", "ntp_seconds_since_last_sync": None}

    def about(self) -> dict:
        return dict(self.version)

    def status(self) -> dict:
        return {"temperature": self.temperature_c}

    # ---------------------------------------------------------------------------------- the network

    def start(self) -> "A50":
        dev = self

        class Tcp(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        class Client(socketserver.BaseRequestHandler):
            def handle(self):
                sock = self.request
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                lock = threading.Lock()
                with dev._clients_lock:
                    dev._clients[sock] = lock
                log.info("a50: client %s:%d connected", *self.client_address[:2])
                buf = b""
                try:
                    while not dev._stop.is_set():
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
                            except ValueError:
                                log.warning("a50: not JSON from %s: %r", self.client_address[0], line[:80])
                                continue
                            out = (json.dumps(dev.command(req)) + "\n").encode()
                            with lock:
                                sock.sendall(out)
                except OSError:
                    pass
                finally:
                    with dev._clients_lock:
                        dev._clients.pop(sock, None)
                    log.info("a50: client %s:%d gone", *self.client_address[:2])

        class Http(http.server.BaseHTTPRequestHandler):
            def do_GET(self):              # noqa: N802 - the stdlib's name
                path = self.path.split("?", 1)[0].rstrip("/")
                body = {"/api/v1/about": dev.about, "/api/v1/about/status": dev.status,
                        "/api/v1/config": lambda: dict(dev.config)}.get(path)
                if body is None:
                    self.send_error(404)
                    return
                data = json.dumps(body()).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *_a):
                pass

        try:
            self._tcp = Tcp((self.bind, self.tcp_port), Client)
            self._http = http.server.ThreadingHTTPServer((self.bind, self.http_port), Http)
        except OSError as exc:
            if self._tcp is not None:
                self._tcp.server_close()
            raise OSError(exc.errno, f"DVL A50: cannot listen on {self.bind} TCP {self.tcp_port} / HTTP "
                                     f"{self.http_port} ({exc.strerror}): is another `uw pool` or physics_proc "
                                     "running?") from exc
        self._http.daemon_threads = True
        self.tcp_port = self._tcp.server_address[1]
        self.http_port = self._http.server_address[1]
        for target, name in ((self._tcp.serve_forever, "a50-tcp"), (self._http.serve_forever, "a50-http"),
                             (self._send_loop, "a50-send")):
            th = threading.Thread(target=target, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        log.info("a50: DVL A50 on %s, TCP %d (JSON %s), HTTP %d", self.bind, self.tcp_port, FORMAT, self.http_port)
        return self

    def _send_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._out.get(timeout=0.2)
            except queue.Empty:
                continue
            if msg.get("type") == "velocity":
                msg["time_of_transmission"] = int(time.time() * 1e6)
            data = (json.dumps(msg) + "\n").encode()
            with self._clients_lock:
                clients = list(self._clients.items())
            for sock, lock in clients:
                try:
                    with lock:
                        sock.settimeout(1.0)
                        sock.sendall(data)
                except OSError:
                    with self._clients_lock:
                        self._clients.pop(sock, None)

    @property
    def clients(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def close(self) -> None:
        self._stop.set()
        for srv in (self._tcp, self._http):
            if srv is not None:
                srv.shutdown()
                srv.server_close()
        with self._clients_lock:
            for sock in list(self._clients):
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                except OSError:
                    pass
            self._clients.clear()
        log.info("a50: closed after %d velocity reports (%d with bottom lock), %d dead-reckoning, %d commands",
                 self.count["velocity"], self.count["locked"], self.count["position_local"], self.count["commands"])


def pool_device(cfg: dict):
    """The pool's A50 (config/pool/pool.yaml ``dvl``), cast at ``uw.pool_geometry.PoolBottom``; None when
    ``dvl.enabled`` or ``dvl.device.enabled`` is off."""
    if not (get(cfg, "dvl.enabled") and get(cfg, "dvl.device.enabled")):
        return None
    from ..pool_geometry import PoolBottom
    from . import dvl as dvl_model
    return A50(dvl_model.from_config(cfg, PoolBottom(cfg)), cfg)
