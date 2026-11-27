"""The ArduSub JSON physics server (``uw/physics_udp.py``) in a process of its own, for ``uw pool`` (M11).

Why a process: the pool's render loop lives in Kit's Python, and a physics thread there would compete for the GIL
with Kit at 400-1200 servo packets a second; every stall is something the pilot feels. Here the server runs alone,
lockstep with SITL and paced to the wall clock, and the render loop only reads the latest state.

    parent (uw pool, inside Kit)                       child: python -m uw.rov.physics_proc --shm NAME
    PhysicsProcess(cfg).start()  -- stdin: config JSON -->  make_pool_server(cfg).start()
    .body_pose() .aux_pwm(n)    <-- shared memory -------  on_step: a seqlock write per servo packet
    .close()                    -- stdin EOF ----------->  server.close(), exit

The child also exits when the parent dies (its stdin reaches EOF), so no orphan keeps UDP 9002 bound.

Standalone, to fly SITL and BlueOS without a GPU or Isaac (the vehicle is then invisible, but ArduSub, BlueOS and
Cockpit see a real vehicle with real dynamics):

    python -m uw py -m uw.rov.physics_proc [--profile P] [--set physics.rtf=1.0]     # Ctrl-C stops
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import subprocess
import sys
import time
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np

log = logging.getLogger("uw.physics")

# Shared-memory layout, float64 slots. SEQ is a seqlock counter: odd while the child writes.
SEQ, STATUS, PORT, T, PACKETS, STALLS, WALL = 0, 1, 2, 3, 4, 5, 6
POS = slice(8, 11)              # body CG in the tank [m], ENU
ROT = slice(11, 20)             # body FLU -> tank ENU, row-major 3x3
VEL = slice(20, 23)             # body FRD linear velocity [m/s]
RATE = slice(23, 26)            # body FRD angular velocity [rad/s]
PWM = slice(26, 42)             # the last servo packet's 16 channels [us]
DVL_REPORTS, DVL_LOCKED, DVL_CLIENTS = 42, 43, 44     # the emulated A50 (uw/rov/a50.py), when dvl.device is on
DVL_TCP_PORT, DVL_HTTP_PORT = 45, 46                  # its ports (0: no DVL)
CTL_PORT, CTL_CLIENTS = 47, 48                        # the control channel (uw/rov/simctl.py; 0: off)
NUC_REPORTS, NUC_LOCKED, NUC_CLIENTS, NUC_PORT = 49, 50, 51, 52   # the emulated Nucleus1000 (uw/rov/nucleus.py)
SIZE = 64
STARTING, RUNNING, FAILED, STOPPED = 0.0, 1.0, -1.0, 2.0


def make_pool_server(cfg: dict, on_step=None):
    """The physics server for the pool config (config/pool/pool.yaml): the vehicle file, the pool's soft bound
    and free surface, the water current ``current``, the start pose ``vehicle.start`` and the SITL home
    ``vehicle.home``."""
    from ..config import WS, get
    from ..physics_udp import PhysicsServer
    from . import current, dynamics, frames

    wl = float(get(cfg, "pool.water_level"))
    cur = current.from_config(cfg, water_depth_m=wl)
    if not cur.is_still:
        log.info("physics: water current %s", cur.describe())
    veh = dynamics.make_vehicle(Path(WS) / str(get(cfg, "vehicle.params")), tank_size=get(cfg, "pool.size"),
                                sand_base=0.0, water_level=wl, surface_model=bool(get(cfg, "physics.surface_model")),
                                current=cur)
    x, y, z, yaw = (float(v) for v in get(cfg, "vehicle.start"))
    p_ned, q_ned = frames.pose_tank_to_ned(np.array([x, y, z]), frames.quat_from_rpy_deg((0.0, 0.0, yaw)), wl)
    state = dynamics.State.rest(veh.params.n_thrusters, p_ned, q_ned)
    home = get(cfg, "vehicle.home") or [x, y]
    return PhysicsServer(veh, state, home, wl, int(get(cfg, "physics.port")), bind=str(get(cfg, "physics.bind")),
                         rtf=float(get(cfg, "physics.rtf")), on_step=on_step)


class _Writer:
    """The child's side of the seqlock: one consistent record per servo packet."""

    def __init__(self, arr: np.ndarray, device=None, nucleus=None):
        self.a = arr
        self.device = device
        self.nucleus = nucleus
        self.server = None                      # set once the server exists: its stall count
        self.control = None                     # the control channel, once it exists: its client count

    def write(self, state, pwm, pose) -> None:
        a = self.a
        a[SEQ] += 1.0
        a[T] = state.t
        a[POS] = pose[0]
        a[ROT] = np.asarray(pose[1]).ravel()
        a[VEL] = state.velocity_frd
        a[RATE] = state.rate_frd
        n = min(len(pwm), 16)
        a[PWM.start:PWM.start + n] = pwm[:n]
        a[PACKETS] += 1.0
        a[WALL] = time.time()
        if self.server is not None:
            a[STALLS] = self.server.stalls
        if self.device is not None:
            a[DVL_REPORTS] = self.device.count["velocity"]
            a[DVL_LOCKED] = self.device.count["locked"]
            a[DVL_CLIENTS] = self.device.clients
        if self.control is not None:
            a[CTL_CLIENTS] = self.control.clients
        if self.nucleus is not None:
            a[NUC_REPORTS] = self.nucleus.count["bottom_track"]
            a[NUC_LOCKED] = self.nucleus.count["locked"]
            a[NUC_CLIENTS] = self.nucleus.clients
        a[SEQ] += 1.0


class _Steps:
    """What runs at every servo packet in the child: the emulated DVLs, then the shared-memory record. A DVL that
    raises is switched off with its traceback logged, so that the vehicle (and the other DVL) keeps going."""

    def __init__(self, writer, devices: dict):
        self.writer = writer
        self.devices = dict(devices)

    def __call__(self, state, pwm, pose) -> None:
        for name, dev in list(self.devices.items()):
            try:
                dev.step(state.t, pose, state)
            except Exception:  # noqa: BLE001 - the physics must not die of its sensor
                log.exception("physics: the %s failed and is off from now on", name)
                del self.devices[name]
                if self.writer is not None:
                    if self.writer.device is dev:
                        self.writer.device = None
                    if self.writer.nucleus is dev:
                        self.writer.nucleus = None
        if self.writer is not None:
            self.writer.write(state, pwm, pose)


def _attach(name: str) -> shared_memory.SharedMemory:
    shm = shared_memory.SharedMemory(name=name)
    if os.name == "posix":
        # Python < 3.13 registers an attached block with this process's resource tracker, which would unlink it
        # (and warn about a "leak") when the child exits; the parent owns it.
        from multiprocessing import resource_tracker
        try:
            resource_tracker.unregister(shm._name, "shared_memory")      # noqa: SLF001
        except Exception:  # noqa: BLE001 - best effort, only a warning at exit is at stake
            pass
    return shm


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def child_main(argv=None) -> int:
    from .. import config

    ap = argparse.ArgumentParser(prog="python -m uw.rov.physics_proc", description=__doc__.split("\n\n")[0])
    ap.add_argument("--shm", help="shared-memory block of the parent (the config then comes on stdin)")
    config.add_args(ap.add_argument)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    if os.name == "posix":            # stop cleanly on SIGTERM too (a background job ignores Ctrl-C's SIGINT)
        signal.signal(signal.SIGTERM, _interrupt)
    shm = arr = None
    if args.shm:
        cfg = json.loads(sys.stdin.readline())
        shm = _attach(args.shm)
        arr = np.ndarray((SIZE,), np.float64, shm.buf)
    else:
        cfg = config.from_args(args, config.POOL_CONFIG)
    device = control = nuc = None
    devices: dict = {}
    try:
        from . import nucleus, simctl
        from .a50 import pool_device
        device = pool_device(cfg)                # None when dvl.enabled or dvl.device.enabled is off
        if device is not None:
            devices["a50"] = device.start()
        nuc = nucleus.pool_device(cfg)           # None unless nucleus.enabled (--preset nucleus | both_dvl)
        if nuc is not None:
            devices["nucleus"] = nuc.start()
        writer = _Writer(arr, device, nuc) if arr is not None else None
        server = make_pool_server(cfg, on_step=_Steps(writer, devices))
        if writer:
            writer.server = server
        if writer:
            writer.write(server.state, server.pwm, server.body_pose())
            arr[PACKETS] = 0.0
        control = simctl.from_config(cfg, server, devices)
        if control is not None:
            control.start()
            if writer:
                writer.control = control
        server.start()
    except Exception:  # noqa: BLE001 - reported to the parent through STATUS, the traceback on stderr
        log.exception("physics: could not start")
        if control is not None:
            control.close()
        for dev in devices.values():
            dev.close()
        if arr is not None:
            arr[STATUS] = FAILED
            del arr
            shm.close()
        return 1
    if arr is not None:
        arr[PORT] = server.port
        if device is not None:
            arr[DVL_TCP_PORT], arr[DVL_HTTP_PORT] = device.tcp_port, device.http_port
        if control is not None and control.listening:
            arr[CTL_PORT] = control.port
        if nuc is not None:
            arr[NUC_PORT] = nuc.tcp_port
        arr[STATUS] = RUNNING
    try:
        if arr is not None:
            sys.stdin.read()                    # until the parent closes the pipe, or dies
        else:
            while server.error is None:
                time.sleep(2.0)
                p, r = server.body_pose()
                v, w, t = server.body_twist()
                vc = server.vehicle.current_ned
                log.info("t %.1f s, %d packets, CG %s m (depth %.2f m), speed %.2f m/s, heading %.0f deg ENU, "
                         "water %.2f m/s%s",
                         t, server.packets, np.round(p, 2).tolist(), float(get_depth(cfg, p)),
                         float(np.linalg.norm(v)), float(np.degrees(np.arctan2(r[1, 0], r[0, 0]))),
                         float(np.hypot(vc[0], vc[1])),
                         ("" if device is None else "; DVL %d reports (%d locked), %d clients" % (
                             device.count["velocity"], device.count["locked"], device.clients))
                         + ("" if nuc is None else "; Nucleus %d BT (%d locked), %d AHRS, %d clients%s" % (
                             nuc.count["bottom_track"], nuc.count["locked"], nuc.count["ahrs"], nuc.clients,
                             "" if nuc.measuring else ", not started")))
    except KeyboardInterrupt:
        pass
    finally:
        if control is not None:
            control.close()
        server.close()
        for dev in devices.values():
            dev.close()
        if arr is not None:
            arr[STATUS] = STOPPED if server.error is None else FAILED
            del arr
            shm.close()
    return 0 if server.error is None else 1


def get_depth(cfg: dict, p_tank) -> float:
    from ..config import get
    return float(get(cfg, "pool.water_level")) - float(p_tank[2])


class PhysicsProcess:
    """The parent's handle: the same reading API as ``PhysicsServer`` (``body_pose``, ``body_twist``, ``error``,
    ``close``) plus ``pwm``/``aux_pwm`` for the lights and the camera tilt."""

    def __init__(self, cfg: dict, python: str | None = None, start_timeout_s: float = 20.0):
        self.cfg = cfg
        self.python = python or os.environ.get("UW_PHYSICS_PYTHON") or sys.executable
        self.start_timeout_s = float(start_timeout_s)
        self.shm = None
        self.arr = None
        self.proc = None
        self.port = None

    def start(self) -> "PhysicsProcess":
        from ..config import WS
        self.shm = shared_memory.SharedMemory(create=True, size=SIZE * 8)
        self.arr = np.ndarray((SIZE,), np.float64, self.shm.buf)
        self.arr[:] = 0.0
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join([str(WS)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
        self.proc = subprocess.Popen([self.python, "-u", "-m", "uw.rov.physics_proc", "--shm", self.shm.name],
                                     stdin=subprocess.PIPE, env=env, text=True)
        self.proc.stdin.write(json.dumps(self.cfg) + "\n")
        self.proc.stdin.flush()
        t_end = time.monotonic() + self.start_timeout_s
        while self.arr[STATUS] == STARTING and self.proc.poll() is None and time.monotonic() < t_end:
            time.sleep(0.02)
        if self.arr[STATUS] != RUNNING:
            why = ("exited with code %s" % self.proc.poll()) if self.proc.poll() is not None else "did not start"
            self.close()
            raise RuntimeError(f"physics process {why} (its traceback is above)")
        self.port = int(self.arr[PORT])
        return self

    def snapshot(self) -> np.ndarray:
        """One consistent copy of the shared record (the seqlock retries while the child is mid-write)."""
        a = self.arr
        for _ in range(1000):
            s1 = a[SEQ]
            if int(s1) % 2 == 0:
                snap = a.copy()
                if a[SEQ] == s1:
                    return snap
        return a.copy()

    @property
    def error(self) -> Exception | None:
        if self.proc is None:
            return None
        if self.arr is not None and self.arr[STATUS] == FAILED:
            return RuntimeError("physics process failed (its traceback is above)")
        if self.proc.poll() is not None:
            return RuntimeError(f"physics process exited with code {self.proc.returncode}")
        return None

    def body_pose(self):
        """(CG position in the tank [m], body FLU -> tank ENU rotation) of the latest step."""
        s = self.snapshot()
        return s[POS].copy(), s[ROT].reshape(3, 3).copy()

    def body_twist(self):
        s = self.snapshot()
        return s[VEL].copy(), s[RATE].copy(), float(s[T])

    def pwm(self) -> np.ndarray:
        return self.snapshot()[PWM].copy()

    def aux_pwm(self, servo: int) -> float:
        """The PWM of ArduSub output SERVO``servo`` (1-based, as in SERVOn_FUNCTION) in the last packet."""
        if not 1 <= int(servo) <= 16:
            raise ValueError(f"servo {servo}: only SERVO1-16 reach the JSON physics")
        return float(self.snapshot()[PWM][int(servo) - 1])

    @property
    def control_port(self):
        """The control channel's TCP port (uw/rov/simctl.py), or None when it is off."""
        s = self.snapshot()
        return int(s[CTL_PORT]) if s[CTL_PORT] else None

    @property
    def dvl_ports(self):
        """(TCP, HTTP) port of the emulated A50, or None without one."""
        s = self.snapshot()
        return (int(s[DVL_TCP_PORT]), int(s[DVL_HTTP_PORT])) if s[DVL_TCP_PORT] else None

    def stats(self) -> dict:
        s = self.snapshot()
        return {"t": float(s[T]), "packets": int(s[PACKETS]), "stalls": int(s[STALLS]),
                "age_s": (time.time() - float(s[WALL])) if s[WALL] else None,
                "dvl": {"reports": int(s[DVL_REPORTS]), "locked": int(s[DVL_LOCKED]), "clients": int(s[DVL_CLIENTS])},
                "control": {"port": int(s[CTL_PORT]), "clients": int(s[CTL_CLIENTS])},
                "nucleus": ({"port": int(s[NUC_PORT]), "bottom_track": int(s[NUC_REPORTS]), "locked": int(s[NUC_LOCKED]),
                             "clients": int(s[NUC_CLIENTS])} if s[NUC_PORT] else None)}

    def close(self, timeout_s: float = 5.0) -> None:
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            try:
                self.proc.wait(timeout_s)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(2.0)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        if self.shm is not None:
            self.arr = None
            self.shm.close()
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass
            self.shm = None


if __name__ == "__main__":
    raise SystemExit(child_main())
