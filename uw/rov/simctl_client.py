"""A client for the simulator's control channel (``uw/rov/simctl.py``): the standard library only, so the bench
in the ROS container and the tools on any host import it with nothing else installed.

    from uw.rov.simctl_client import SimCtl
    with SimCtl("127.0.0.1:9020") as sim:
        sim.hello()
        sim.set_current(speed_mps=0.3, direction_deg=90, ramp_s=3)
        sim.state()["p_tank"]
        for s in sim.stream(rate_hz=20, duration_s=5): ...
"""
from __future__ import annotations

import json
import socket
import time


class SimCtlError(RuntimeError):
    """The simulator answered ``ok: false``, or did not answer."""


class SimCtl:
    def __init__(self, addr: str = "127.0.0.1:9020", timeout_s: float = 5.0):
        host, _, port = addr.rpartition(":")
        self.addr = (host or "127.0.0.1", int(port))
        self.timeout_s = float(timeout_s)
        self.sock: socket.socket | None = None
        self._buf = b""
        self._id = 0

    # ---- connection
    def connect(self) -> "SimCtl":
        self.sock = socket.create_connection(self.addr, timeout=self.timeout_s)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self) -> "SimCtl":
        return self.connect() if self.sock is None else self

    def __exit__(self, *exc) -> None:
        self.close()

    def _line(self) -> dict:
        while b"\n" not in self._buf:
            data = self.sock.recv(65536)
            if not data:
                raise SimCtlError(f"the simulator at {self.addr[0]}:{self.addr[1]} closed the connection")
            self._buf += data
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line)

    # ---- requests
    def call(self, op: str, **args):
        if self.sock is None:
            self.connect()
        self._id += 1
        rid = self._id
        self.sock.sendall((json.dumps(dict(args, op=op, id=rid)) + "\n").encode())
        while True:
            msg = self._line()
            if msg.get("type") == "state":             # a stream's tail after a stop: skip it
                continue
            if msg.get("id") != rid:
                continue
            if not msg.get("ok"):
                raise SimCtlError(msg.get("error", "unknown error"))
            return msg.get("result")

    def hello(self) -> dict:
        return self.call("hello")

    def state(self) -> dict:
        return self.call("get_state")

    def stats(self) -> dict:
        return self.call("stats")

    def set_current(self, ramp_s: float = 0.0, **fields) -> dict:
        """Keys as config ``current:``: speed_mps, direction_deg, profile, power_exponent, bottom_factor, table,
        gust={sigma_mps, time_s, dir_sigma_deg}. ``speed_mps=0`` (or nothing) is still water."""
        return self.call("set_current", ramp_s=ramp_s, **fields)

    def still(self, ramp_s: float = 0.0) -> dict:
        return self.call("set_current", ramp_s=ramp_s)

    def reposition(self, x: float, y: float) -> dict:
        return self.call("reposition", xy_tank=[float(x), float(y)])

    def device(self, name: str, **settings) -> dict:
        return self.call("device", name=name, set=settings or None)

    def stream(self, rate_hz: float = 10.0, duration_s: float | None = None, fields=None):
        """Yield state dicts at ``rate_hz`` until ``duration_s`` (None: forever) or until the caller stops
        iterating; the stream is stopped on the server either way."""
        if self.sock is None:
            self.connect()
        self._id += 1
        req = {"op": "stream", "rate_hz": rate_hz, "id": self._id}
        if fields:
            req["fields"] = list(fields)
        self.sock.sendall((json.dumps(req) + "\n").encode())
        t_end = None if duration_s is None else time.monotonic() + float(duration_s)
        try:
            while t_end is None or time.monotonic() < t_end:
                msg = self._line()
                if msg.get("ok") is False:
                    raise SimCtlError(msg.get("error", "stream refused"))
                if msg.get("type") == "state":
                    yield msg
        finally:
            if self.sock is not None:
                try:
                    self.sock.sendall(b"\n")            # anything stops it
                except OSError:
                    pass
