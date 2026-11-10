"""The ROV physics server for ArduSub SITL's JSON backend (M8a step 3), and ``ArduSubPose`` (sonar.pose_source ardusub).

SITL is the client, this module the server (docker/ardusub_sitl/README.md, the format of tools/stub_physics.py):
  SITL -> sim   servo packet to UDP 127.0.0.1:<sim.physics_port>, little endian: uint16 magic, uint16 frame_rate,
                uint32 frame_count, uint16 pwm[16] (magic 18458) or pwm[32] (magic 29569)
  sim -> SITL   one reply per servo packet, to the packet's source: "\\n{json}\\n" with the stub's exact layout

The thread integrates the Fossen model of uw.rov.dynamics (imported, not rewritten) with the vehicle file named by
vehicle.params, one or more fixed steps of 1/integrator.rate_hz (400 Hz, docs/ros_hil.md 3.2) per servo packet,
decoupled from the ping. SITL is lockstep, so its clock is the timestamps sent here; they are paced against the
wall clock at sim.physics_rtf (1.0 = real time, the pace the Isaac loop keeps with sim.realtime).

Frames (docs/ros_hil.md 6.2, uw/rov/frames.py the only converter): the tank is ENU, the body FLU; SITL gets NED/FRD
relative to the home point vehicle.home_tank (null: the start point) on the free surface.

Only the body state is shared with the Isaac loop, behind a lock. ``ArduSubPose.at`` turns it into the sonar's
pose through the fixed mount (vehicle.body.offset, the sonar's configured roll and pitch), so main.py's two call
sites are unchanged. No ROS here: /gt/sonar_pose is published by the sink from the ping, as for every pose source.
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time

import numpy as np

from .config import ConfigError, get

log = logging.getLogger("uw.physics")

MAGIC_16 = 18458
MAGIC_32 = 29569
HDR = struct.Struct("<HHI")
G = 9.80665


def _v(a) -> str:
    """A vector the way SIM_JSON's strstr/sscanf parser wants it: "[a, b, c]"."""
    return "[%s]" % ", ".join("%.6f" % float(x) for x in a)


def reply(t: float, gyro, accel, position, attitude, velocity) -> bytes:
    """The JSON frame, in tools/stub_physics.py's layout (keys, order, separators)."""
    return ('\n{"timestamp":%.6f,"imu":{"gyro":%s,"accel_body":%s},"position":%s,"attitude":%s,"velocity":%s}\n'
            % (t, _v(gyro), _v(accel), _v(position), _v(attitude), _v(velocity))).encode()


def parse_servo(data: bytes):
    """(frame_rate, frame_count, pwm tuple) of a servo packet, or None if it is not one."""
    if len(data) < HDR.size:
        return None
    magic, rate_hz, frame_count = HDR.unpack_from(data)
    nch = 16 if magic == MAGIC_16 else 32 if magic == MAGIC_32 else 0
    if not nch or len(data) < HDR.size + 2 * nch:
        return None
    return rate_hz, frame_count, struct.unpack_from("<%dH" % nch, data, HDR.size)


class PhysicsServer:
    """The UDP server thread. ``start`` binds first, in the caller's thread, so a busy port is an immediate
    ``ConfigError`` (typically tools/stub_physics.py still running), never a silently dead thread."""

    def __init__(self, vehicle, state, home_tank, water_level: float, port: int, bind: str = "127.0.0.1",
                 rtf: float = 1.0):
        from .rov import frames
        self.frames, self.vehicle, self.state = frames, vehicle, state
        self.home_ned = frames.position_tank_to_ned(np.array([home_tank[0], home_tank[1], water_level]), water_level)
        self.water_level, self.port, self.bind_addr, self.rtf = float(water_level), int(port), bind, float(rtf)
        self.channels = [t.channel for t in vehicle.params.thrusters]
        self.cmd = np.zeros(vehicle.params.n_thrusters)
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.sock = None
        self.thread = None
        self.packets = 0
        self.steps = 0
        self.peer = None
        self.rate_hz = 0
        self.error = None
        self._pose = self._body_pose(state)

    # ---- lifecycle
    def start(self) -> "PhysicsServer":
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):      # Windows: refuse to share the port with another binder
            s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            s.bind((self.bind_addr, self.port))
        except OSError as exc:
            s.close()
            raise ConfigError(f"physics: UDP {self.bind_addr}:{self.port} is busy ({exc}). Is tools/stub_physics.py "
                              "(or another sim) still running? Stop it, or set sim.physics_port.") from exc
        s.settimeout(0.2)
        self.sock, self.port = s, s.getsockname()[1]          # port 0 (tests): the one the OS picked
        self.thread = threading.Thread(target=self._run, name="uw-physics-udp", daemon=True)
        self.thread.start()
        log.info("physics: listening on %s:%d for ArduSub SITL (JSON), %d thrusters, %.0f Hz, rtf %.2f",
                 self.bind_addr, self.port, len(self.channels), self.vehicle.params.rate_hz, self.rtf)
        return self

    def close(self, timeout: float = 2.0) -> None:
        self.stop_flag.set()
        if self.thread is not None:
            self.thread.join(timeout)
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        log.info("physics: stopped after %d servo packets, %d steps, sim t %.2f s, %d bound activations",
                 self.packets, self.steps, self.state.t, self.vehicle.bound_activations)

    # ---- shared state
    def _body_pose(self, state):
        p_tank, q_enu = self.frames.pose_ned_to_tank(state.position_ned, state.quat_frd_ned, self.water_level)
        return p_tank, self.frames.quat_to_rot(q_enu)

    def body_pose(self):
        """(position in the tank [m], body FLU -> tank ENU rotation) of the latest step."""
        with self.lock:
            return self._pose

    # ---- the thread
    def _frame(self, state, cmd) -> bytes:
        fr = self.frames
        rot = state.rot_frd_ned                                          # body FRD -> NED
        nu_dot = self.vehicle.acceleration(state, cmd)
        a_body = nu_dot[:3] + np.cross(state.rate_frd, state.velocity_frd)       # inertial acceleration, body axes
        f_body = a_body - rot.T @ np.array([0.0, 0.0, G])                # specific force: at rest (0, 0, -g)
        rpy = np.radians(fr.rpy_deg_from_rot(rot))
        return reply(state.t, state.rate_frd, f_body, state.position_ned - self.home_ned, rpy, rot @ state.velocity_frd)

    def _run(self) -> None:
        dt = self.vehicle.params.dt
        wall0 = None
        try:
            while not self.stop_flag.is_set():
                try:
                    data, addr = self.sock.recvfrom(512)
                except socket.timeout:
                    continue
                except ConnectionResetError:        # Windows: the previous reply hit a closed port
                    continue
                except OSError:
                    if self.stop_flag.is_set():
                        break
                    raise
                pkt = parse_servo(data)
                if pkt is None:
                    continue
                rate_hz, _, pwm = pkt
                if addr != self.peer:
                    self.peer, self.rate_hz = addr, rate_hz
                    log.info("physics: servo packets from %s:%d, %d channels, frame_rate %d Hz", addr[0], addr[1],
                             len(pwm), rate_hz)
                cmd = self.vehicle.params.t200.command_of_pwm(np.array([pwm[c] for c in self.channels], float))
                frame_dt = 1.0 / (rate_hz or self.vehicle.params.rate_hz)
                n = max(1, int(round(frame_dt / dt)))
                state = self.state
                for _ in range(n):
                    state = self.vehicle.step(state, cmd, frame_dt / n)
                if self.rtf > 0:
                    if wall0 is None:
                        wall0 = time.monotonic() - state.t / self.rtf
                    delay = wall0 + state.t / self.rtf - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                pose = self._body_pose(state)
                with self.lock:
                    self.state, self.cmd, self._pose = state, cmd, pose
                self.sock.sendto(self._frame(state, cmd), addr)
                self.packets += 1
                self.steps += n
        except Exception as exc:  # noqa: BLE001 - reported in the log and by ArduSubPose, never swallowed
            self.error = exc
            log.exception("physics: thread died")


def mount(cfg: dict):
    """The sonar in the body frame: (rotation sonar -> body FLU, position [m])."""
    from .geometry import body_rotation, sonar_rotation
    rb = body_rotation(cfg)
    return rb.T @ sonar_rotation(cfg), -np.asarray(get(cfg, "vehicle.body.offset"), float)


def make_server(cfg: dict) -> PhysicsServer:
    """The server for ``cfg``: the vehicle file vehicle.params, the start pose = the body at the configured sonar
    pose, home vehicle.home_tank (null: the start point)."""
    from pathlib import Path
    from .config import WS
    from .geometry import body_rotation
    from .rov import dynamics, frames
    port = get(cfg, "sim.physics_port")
    if port is None:
        raise ConfigError("sonar.pose_source ardusub needs sim.physics_port (e.g. --set sim.physics_port=9002)")
    params = Path(WS) / str(get(cfg, "vehicle.params"))
    wl = float(get(cfg, "tank.water_level"))
    veh = dynamics.make_vehicle(params, tank_size=get(cfg, "tank.size"), sand_base=float(get(cfg, "sand.thickness")),
                                water_level=wl)
    rb = body_rotation(cfg)
    p_cfg = np.asarray(get(cfg, "sonar.position"), float) + rb @ np.asarray(get(cfg, "vehicle.body.offset"), float)
    p0 = p_cfg
    if veh.bounds is not None:     # the tank scene's pole pose puts the body through the -x wall: start inside
        p0 = np.clip(p_cfg, veh.bounds.lo, veh.bounds.hi)
        if np.any(p0 != p_cfg):
            log.warning("physics: the body at the configured sonar pose (%s) is outside the soft bound; it starts at "
                        "%s, and the sonar with it", np.round(p_cfg, 3).tolist(), np.round(p0, 3).tolist())
    p_ned, q_ned = frames.pose_tank_to_ned(p0, frames.rot_to_quat(rb), wl)
    state = dynamics.State.rest(veh.params.n_thrusters, p_ned, q_ned)
    home = get(cfg, "vehicle.home_tank") or p0[:2]
    return PhysicsServer(veh, state, home, wl, int(port), rtf=float(get(cfg, "sim.physics_rtf")))


class ArduSubPose:
    """SensorPose of sonar.pose_source ardusub: the sonar rides the body the physics thread integrates."""
    moving = True

    def __init__(self, cfg: dict, server: PhysicsServer | None = None):
        self.server = (server or make_server(cfg)).start()
        self.r_mount, self.p_mount = mount(cfg)

    def at(self, t: float):
        from .geometry import rpy_deg_of
        from .pose import Pose
        if self.server.error is not None:
            raise RuntimeError(f"physics thread died: {self.server.error!r}") from self.server.error
        p, rb = self.server.body_pose()
        return Pose(p + rb @ self.p_mount, rpy_deg_of(rb @ self.r_mount))

    def positions(self, step: float = 0.05):
        return self.at(0.0).position[None, :]

    def close(self) -> None:
        self.server.close()


__all__ = ["ArduSubPose", "PhysicsServer", "make_server", "mount", "parse_servo", "reply", "MAGIC_16", "MAGIC_32"]
