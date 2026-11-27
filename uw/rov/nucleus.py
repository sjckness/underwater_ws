"""Nortek Nucleus1000 emulated at the protocol level (M12 P5), in the physics process next to the A50.

What is emulated is what Nortek's own unmodified software uses (its Python driver ``nucleus_driver`` 1.7.0-1.7.8,
the BlueOS extension ``nortekgroup/blueos_nucleus:2.0.11`` and the ROS 2 node): read from their source and from the
"Nucleus Operations and Integration" manual N3015-033, chapters 7-8 and 11.2 (2026-11-27). See docs/dvl.md.

**Network.** TCP ``cmd_port`` (the device's 9000): a password prompt (``Please enter password:``, default
``nortek``), then ASCII commands answered ``OK`` / ``ERROR`` (or in NMEA form ``$PNOR,...*CS``), plus the binary data
while measuring. One client at a time, as on the device. TCP ``data_port`` (9002): data only, no login, input
ignored, up to two clients. Measuring goes on across a disconnect, until STOP.

**Commands.** COMMAND mode: SETCLOCKSTR, GETALL, GET*, SET* (accepted, the few that matter kept: SETTRIG FREQ,
SETAHRS FREQ, SETNAV DS, SETINST TYPE, SETMISSION LAT/LONG), START, FIELDCAL, GETERROR. MEASUREMENT mode: STOP,
TRIG, UPDATEPOS; anything else is ERROR there, as the manual says.

**Binary packets** (little-endian: a 10-byte header with sync 0xA5, family 0x20 and two checksums seeded 0xB58C,
then a 20-byte common block): 0xD2 AHRS (version 2), 0xB4 bottom track (128 bytes: 1.7.0 rejects shorter ones),
0xAA altimeter (44 bytes, the quality word the manual omits included), 0xDC INS when the navigation output is on.

**The physics.** Bottom track is ``uw.rov.dvl``'s beam model with this instrument's three beams (config
``nucleus:``), cast at the pool; the altimeter is a vertical beam on the same bottom; the AHRS is the true attitude
plus a slow heading bias and white noise; the INS integrates the bottom-track velocity with the AHRS attitude, as the
device does. Every rate is the device's own (the trigger frequency, the AHRS and navigation frequencies), on the
physics thread's simulated clock; the timestamps are Unix time once SETCLOCKSTR has set the clock, as on the device.

Stdlib and numpy only (``uw/rov``'s rule).
"""
from __future__ import annotations

import calendar
import logging
import math
import queue
import socket
import socketserver
import struct
import threading
import time

import numpy as np

from ..config import get
from . import frames as fr

log = logging.getLogger("uw.nucleus")

SYNC, FAMILY, SEED = 0xA5, 0x20, 0xB58C
ID_AHRS, ID_BT, ID_ALTI, ID_INS = 0xD2, 0xB4, 0xAA, 0xDC
INVALID_VEL, INVALID_FOM = -32.768, 10.0
G = 9.80665
RHO = 997.0


# ------------------------------------------------------------------------------------------- the wire format

def checksum(data: bytes) -> int:
    """Nortek's 16-bit checksum: seed 0xB58C, little-endian words added mod 2^16, an odd last byte as b << 8."""
    c = SEED
    n = len(data) - (len(data) % 2)
    for i in range(0, n, 2):
        c = (c + (data[i] | data[i + 1] << 8)) & 0xFFFF
    if len(data) % 2:
        c = (c + (data[-1] << 8)) & 0xFFFF
    return c


def frame(pid: int, data: bytes) -> bytes:
    head = struct.pack("<BBBBHH", SYNC, 10, pid, FAMILY, len(data), checksum(data))
    return head + struct.pack("<H", checksum(head)) + bytes(data)


def common(version: int, od: int, size: int, stamp: float, status: int, serial: int, posix: bool) -> bytearray:
    """The 20-byte common block of every data packet, then zeros up to ``size``."""
    d = bytearray(size)
    sec = int(stamp)
    usec = min(int(round((stamp - sec) * 1e6)), 999999)
    struct.pack_into("<BBBBIIII", d, 0, version, od, 1 if posix else 0, 0, sec & 0xFFFFFFFF, usec, status & 0xFFFFFFFF,
                     serial)
    return d


def nmea(body: str) -> bytes:
    """``$PNOR,<body>*CS\\r\\n``, CS the XOR of everything between ``$`` and ``*``."""
    s = "PNOR," + body
    cs = 0
    for ch in s.encode():
        cs ^= ch
    return f"${s}*{cs:02X}\r\n".encode()


def ahrs_packet(stamp, rpy_deg, rot_body_ned, depth_m, serial, posix, fom=0.1) -> bytes:
    """0xD2, version 2, data offset 36: roll, pitch, heading [deg], quaternion (w, x, y, z), the 3 x 3 rotation
    body -> NED row-major, declination, depth."""
    d = common(2, 36, 108, stamp, 0, serial, posix)
    d[24] = 2                                            # operation mode: AHRS
    struct.pack_into("<ff", d, 28, fom, 0.0)
    q = fr.rot_to_quat(rot_body_ned)
    struct.pack_into("<3f4f9fff", d, 36, *map(float, rpy_deg), *map(float, q),
                     *map(float, np.asarray(rot_body_ned, float).ravel()), 0.0, float(depth_m))
    return frame(ID_AHRS, bytes(d))


def bt_packet(stamp, rep, sound_speed, temperature_c, pressure_bar, serial, posix, dt_s) -> bytes:
    """0xB4 from a ``uw.rov.dvl.Report`` of the three-beam model: 128 bytes, offsets absolute."""
    n = len(rep.beam_valid)
    beam_ok = [bool(rep.beam_valid[i]) for i in range(3)] if n >= 3 else [False] * 3
    status = 0
    for i in range(3):
        if beam_ok[i]:
            status |= (1 << i) | (1 << (3 + i)) | (1 << (6 + i))
    vel_ok = bool(rep.velocity_valid)
    if vel_ok:
        status |= 0b111 << 9 | 0b111 << 12
    status |= 1 << 16 | 1 << 17                          # pressure and temperature valid
    d = common(1, 0, 128, stamp, status, serial, posix)
    bv = [float(rep.beam_velocity[i]) if beam_ok[i] else INVALID_VEL for i in range(3)]
    bd = [float(rep.beam_range[i]) if beam_ok[i] else 0.0 for i in range(3)]
    sig = math.sqrt(max(float(np.mean(np.diag(rep.covariance))), 0.0)) if vel_ok else INVALID_FOM
    bf = [max(sig, 1e-4) if beam_ok[i] else INVALID_FOM for i in range(3)]
    v = [float(x) for x in rep.velocity] if vel_ok else [INVALID_VEL] * 3
    fom = ([math.sqrt(max(float(rep.covariance[k, k]), 0.0)) for k in range(3)] if vel_ok else [INVALID_FOM] * 3)
    struct.pack_into("<fff", d, 24, float(sound_speed), float(temperature_c), float(pressure_bar))
    struct.pack_into("<3f3f3f3f3f", d, 36, *bv, *bd, *bf, *([dt_s] * 3), *([dt_s] * 3))
    struct.pack_into("<3f3fff", d, 96, *v, *fom, dt_s, dt_s)
    return frame(ID_BT, bytes(d))


def alti_packet(stamp, distance_m, sound_speed, temperature_c, pressure_bar, serial, posix) -> bytes:
    """0xAA: 44 bytes, distance and quality (the drivers read the quality word at 40)."""
    ok = distance_m is not None and math.isfinite(distance_m)
    status = (0b11 if ok else 0) | 1 << 16 | 1 << 17
    d = common(1, 0, 44, stamp, status, serial, posix)
    struct.pack_into("<ffff", d, 24, float(sound_speed), float(temperature_c), float(pressure_bar),
                     float(distance_m) if ok else 0.0)
    struct.pack_into("<H", d, 40, 80 if ok else 0)
    return frame(ID_ALTI, bytes(d))


def ins_packet(stamp, rpy_deg, rot_body_ned, depth_m, pos_ned, vel_ned, vel_body, latlon, temperature_c,
               pressure_bar, altitude_m, serial, posix, fom=0.1) -> bytes:
    """0xDC, version 2, data offset 36: the AHRS block, then the navigation solution at OD + 72."""
    d = bytearray(ahrs_packet(stamp, rpy_deg, rot_body_ned, depth_m, serial, posix, fom)[10:])
    d.extend(bytes(208 - len(d)))
    d[2] = 1 if posix else 0
    od = 36
    lat_ok = latlon is not None
    sog = math.hypot(float(vel_ned[0]), float(vel_ned[1]))
    cog = math.degrees(math.atan2(float(vel_ned[1]), float(vel_ned[0]))) % 360.0
    struct.pack_into("<fIffff", d, od + 72, fom, 1 if lat_ok else 0, cog, float(temperature_c), float(pressure_bar),
                     float(altitude_m) if altitude_m is not None and math.isfinite(altitude_m) else 0.0)
    struct.pack_into("<ddd", d, od + 96, *(latlon if lat_ok else (0.0, 0.0)), 0.0)
    struct.pack_into("<13f", d, od + 120, *map(float, pos_ned), *map(float, vel_ned), *map(float, vel_body), sog,
                     0.0, 0.0, 0.0)
    return frame(ID_INS, bytes(d))


# -------------------------------------------------------------------------------------------------- the AHRS

class Ahrs:
    """The true attitude with a slow heading bias (a sum of random sinusoids, as ``dvl._BiasField``) and white
    noise; roll and pitch get noise only. Deterministic for a seed and a sample index."""

    def __init__(self, cfg_dev: dict, seed: int):
        self.seed = int(seed)
        self.rp_sigma = float(cfg_dev.get("roll_pitch_sigma_deg", 0.1))
        self.h_sigma = float(cfg_dev.get("heading_sigma_deg", 0.3))
        b = float(cfg_dev.get("heading_bias_sigma_deg", 1.0))
        tau = float(cfg_dev.get("heading_bias_time_s", 300.0))
        rng = np.random.default_rng([self.seed, 4071])
        self.b_a = b * math.sqrt(2.0 / 6)
        self.b_w = 2 * np.pi / (max(tau, 1e-6) * rng.uniform(1.0 / 3.0, 3.0, 6))
        self.b_ph = rng.uniform(0, 2 * np.pi, 6)
        self.n = 0

    def sample(self, t: float, rot_frd_ned: np.ndarray):
        """(roll, pitch, heading [deg, heading 0..360], rotation body -> NED of those angles)."""
        roll, pitch, yaw = (float(v) for v in fr.rpy_deg_from_rot(rot_frd_ned))
        rng = np.random.default_rng([self.seed, 4072, self.n])
        self.n += 1
        e = rng.normal(0.0, 1.0, 3)
        bias = self.b_a * float(np.sum(np.sin(self.b_w * float(t) + self.b_ph)))
        roll += self.rp_sigma * e[0]
        pitch += self.rp_sigma * e[1]
        yaw += bias + self.h_sigma * e[2]
        return roll, pitch, yaw % 360.0, fr.rot_from_rpy_deg((roll, pitch, yaw))


# ------------------------------------------------------------------------------------------------ the device

GETALL_TEMPLATE = [
    'ID,STR="Nucleus1000",SN={sn}',
    'GETHW,DIGITAL="B-1",ANALOG="B-0"',
    'GETFW,STR="1.0.0",MAJOR=1,MINOR=0,PATCH=0,BUILD=0,HASH="71d3ad7",DVLFW=4002,DVLMINOR=10,DVLBOOT=2,DVLFPGA=1014',
    'GETCLOCKSTR,TIME="{clock}"',
    'GETINST,TYPE="{inst}",ROTXY=0.00,ROTYZ=0.00,ROTXZ=0.00',
    'GETMISSION,POFF=9.50,LONG={lon:.6f},LAT={lat:.6f},DECL=0.00,RANGE=50.00,BD=0.10,SV={sv:.2f},SA=0.00',
    'GETAHRS,FREQ={ahrs_hz:.0f},DS="ON",DF=210',
    'GETBT,MODE="AUTO",VR=5.00,WT="OFF",PL=-2.00,PLMODE="MAX",DS="ON",DF=180,BW="BROAD"',
    'GETALTI,PL=0.00,DS="ON",DF=170',
    'GETSPECTRUM,BW="BROAD"',
    'GETTRIG,SRC="INTERNAL",FREQ={trig_hz:.2f},ALTI={alti}',
    'GETIMU,FREQ=100,DS="OFF",DF=130',
    'GETMAG,FREQ=75,CS="MAG",METHOD="AUTO",DS="OFF",DF=135',
    'GETMAGCAL,HX=0.0000,HY=0.0000,HZ=0.0000,M11=1.0000,M12=0.0000,M13=0.0000,M21=0.0000,M22=1.0000,M23=0.0000,'
    'M31=0.0000,M32=0.0000,M33=1.0000',
]
MEASUREMENT_CMDS = ("STOP", "TRIG", "UPDATEPOS")


def _args(text: str) -> dict:
    """``A=1,B="X"`` -> {"A": "1", "B": "X"}."""
    out = {}
    for part in text.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip().upper()] = v.strip().strip('"')
    return out


class Nucleus:
    """The device: ``step(t, pose, state)`` from the physics thread, ``start()``/``close()`` for the servers.
    ``command(req)`` is the A50-style JSON entry the control channel uses (``set_config`` with ``acoustic_enabled``)."""

    def __init__(self, dvl, cfg: dict):
        self.dvl = dvl                                   # uw.rov.dvl.Dvl with the three beams of config nucleus:
        dev = get(cfg, "nucleus.device")
        self.bind = str(dev["bind"])
        self.tcp_port = int(dev["cmd_port"])
        self.data_port = int(dev["data_port"])
        self.http_port = None
        self.password = str(dev.get("password", "nortek"))
        self.serial = int(dev.get("serial_number", 900002))
        self.trig_hz = float(dev.get("trig_hz", 8.0))
        self.alti_every = int(dev.get("alti_every", 4))
        self.ahrs_hz = float(dev.get("ahrs_hz", 20.0))
        self.nav_hz = float(dev.get("nav_hz", 10.0))
        self.nav_on = bool(dev.get("ins", True))
        self.inst_type = "NAV" if self.nav_on else "SENSORS"
        self.temperature_c = float(get(cfg, "water.temperature_c"))
        self.water_level = float(get(cfg, "pool.water_level"))
        self.ahrs = Ahrs(dev, int(cfg.get("seed", 0)) + 77)
        self.origin = None                               # (lat, lon) from SETMISSION / UPDATEPOS
        self.config = {"acoustic_enabled": True}
        self.measuring = False
        self.posix = False
        self.clock_offset = 0.0                          # device clock = wall clock + offset (SETCLOCKSTR: 0)
        self.t_start = time.time()
        self._lock = threading.Lock()
        self._out: queue.Queue = queue.Queue(maxsize=400)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._cmd_client = None                          # (sock, lock) of the logged-in port-9000 client
        self._data_clients: dict = {}
        self._clients_lock = threading.Lock()
        self._tcp = self._data = None
        self._next = {"ahrs": None, "trig": None, "nav": None}
        self._trig_n = 0
        self._last_bt_t = None
        self._ins_pos = np.zeros(3)
        self._last_ins_t = None
        self._last_vel_ned = np.zeros(3)
        self._alt = float("nan")
        self.count = {"ahrs": 0, "bottom_track": 0, "locked": 0, "altimeter": 0, "ins": 0, "commands": 0,
                      "dropped": 0}

    # ------------------------------------------------------------------------------------------ physics side
    def _stamp(self) -> float:
        return time.time() + self.clock_offset if self.posix else time.time() - self.t_start

    def step(self, t: float, pose, state) -> None:
        if not self.measuring:
            for k in self._next:
                self._next[k] = None
            return
        p_tank, rot_flu_enu = pose
        rot_ned = state.rot_frd_ned
        depth = float(state.position_ned[2])
        pressure_bar = max(depth, 0.0) * RHO * G / 1e5
        sv = float(self.dvl.p.sound_speed_mps)
        for k, hz in (("ahrs", self.ahrs_hz), ("trig", self.trig_hz), ("nav", self.nav_hz)):
            if self._next[k] is None:
                self._next[k] = t + 1.0 / hz
        if t + 1e-9 >= self._next["ahrs"]:
            self._next["ahrs"] += 1.0 / self.ahrs_hz
            roll, pitch, hdg, rot = self.ahrs.sample(t, rot_ned)
            self._emit(ahrs_packet(self._stamp(), (roll, pitch, hdg), rot, depth, self.serial, self.posix))
            self.count["ahrs"] += 1
        if t + 1e-9 >= self._next["trig"]:
            self._next["trig"] += 1.0 / self.trig_hz
            self._trig_n += 1
            if self.config["acoustic_enabled"]:
                if self.alti_every > 0 and self._trig_n % self.alti_every == 0:
                    self._altimeter(p_tank, rot_flu_enu, sv, pressure_bar)
                else:
                    self._bottom_track(t, p_tank, rot_flu_enu, state, rot_ned, sv, pressure_bar)
        if self.nav_on and t + 1e-9 >= self._next["nav"]:
            self._next["nav"] += 1.0 / self.nav_hz
            self._ins(t, rot_ned, depth, pressure_bar, state)

    def _bottom_track(self, t, p_tank, rot_flu_enu, state, rot_ned, sv, pressure_bar) -> None:
        rep = self.dvl.sample(t, p_tank, rot_flu_enu, state.velocity_frd, state.rate_frd)
        dt = 0.0 if self._last_bt_t is None else float(t - self._last_bt_t)
        self._last_bt_t = t
        if rep.velocity_valid:
            self.count["locked"] += 1
            # the device's own navigation: the instrument velocity (FRD, mounted along the body) into NED with its
            # AHRS attitude, held until the next ping
            _, _, _, rot = self.ahrs.sample(t, rot_ned)
            self._last_vel_ned = rot @ (self.dvl.mount.rot_body_to_dvl.T @ rep.velocity)
            if np.isfinite(rep.altitude):
                self._alt = float(rep.altitude)
        self._emit(bt_packet(self._stamp(), rep, sv, self.temperature_c, pressure_bar, self.serial, self.posix, dt))
        self.count["bottom_track"] += 1

    def _altimeter(self, p_tank, rot_flu_enu, sv, pressure_bar) -> None:
        origin, _ = self.dvl.geometry_at(p_tank, rot_flu_enu)
        down = np.asarray(rot_flu_enu, float) @ np.array([0.0, 0.0, -1.0])        # the body's -z, i.e. FRD +z
        t_hit, _, roles = self.dvl.bottom.ray_bottom(origin, down[None, :], t_max=50.0)
        dist = float(t_hit[0]) if roles[0] is not None else None
        self._emit(alti_packet(self._stamp(), dist, sv, self.temperature_c, pressure_bar, self.serial, self.posix))
        self.count["altimeter"] += 1

    def _ins(self, t, rot_ned, depth, pressure_bar, state) -> None:
        if self._last_ins_t is not None:
            self._ins_pos[:2] += self._last_vel_ned[:2] * float(t - self._last_ins_t)
        self._last_ins_t = t
        self._ins_pos[2] = depth
        roll, pitch, hdg, rot = self.ahrs.sample(t, rot_ned)
        vel_body = rot.T @ self._last_vel_ned
        latlon = None
        if self.origin is not None:
            lat0, lon0 = self.origin
            r = 6378137.0
            latlon = (lat0 + math.degrees(self._ins_pos[0] / r),
                      lon0 + math.degrees(self._ins_pos[1] / (r * math.cos(math.radians(lat0)))))
        self._emit(ins_packet(self._stamp(), (roll, pitch, hdg), rot, depth, self._ins_pos, self._last_vel_ned,
                              vel_body, latlon, self.temperature_c, pressure_bar, self._alt, self.serial, self.posix))
        self.count["ins"] += 1

    def _emit(self, data: bytes) -> None:
        try:
            self._out.put_nowait(data)
        except queue.Full:
            self.count["dropped"] += 1

    # ----------------------------------------------------------------------------------------- the protocol
    def handle_line(self, line: str) -> bytes:
        """One command line (without CR LF) -> the bytes to answer."""
        text = line.strip()
        is_nmea = text.startswith("$PNOR,")
        if is_nmea:
            text = text[len("$PNOR,"):].split("*", 1)[0]
        name, _, rest = text.partition(",")
        name = name.strip().upper()
        with self._lock:
            self.count["commands"] += 1
            ok, values = self._command(name, rest)
        if is_nmea:
            if not ok:
                return nmea("ERROR")
            return b"".join(nmea(v) for v in values) + nmea("OK")
        if not ok:
            return b"ERROR\r\n"
        return b"".join((v + "\r\n").encode() for v in values) + b"OK\r\n"

    def _command(self, name: str, rest: str):
        """(ok, value lines) of one command."""
        if self.measuring and name not in MEASUREMENT_CMDS:
            return False, []
        if not self.measuring and name in ("STOP", "TRIG", "UPDATEPOS"):
            if name != "STOP":
                return False, []
            return True, []                              # STOP when idle: harmless
        a = _args(rest)
        if name == "SETCLOCKSTR":
            try:
                wall = calendar.timegm(time.strptime(a.get("TIME", ""), "%Y-%m-%d %H:%M:%S"))      # UTC
            except ValueError:
                return False, []
            self.clock_offset = wall - time.time()
            self.posix = True
            return True, []
        if name == "GETALL":
            return True, self._getall()
        if name == "GETERROR":
            return True, ['0,"",""']
        if name == "START":
            self.measuring = True
            log.info("nucleus: START (measuring: AHRS %.0f Hz, trigger %.1f Hz, altimeter every %d, INS %s)",
                     self.ahrs_hz, self.trig_hz, self.alti_every, f"{self.nav_hz:.0f} Hz" if self.nav_on else "off")
            return True, []
        if name == "FIELDCAL":
            return True, []                              # the simulated compass needs no calibration
        if name == "STOP":
            self.measuring = False
            log.info("nucleus: STOP")
            return True, []
        if name == "TRIG":
            return True, []
        if name in ("UPDATEPOS", "SETMISSION"):
            if "LAT" in a and "LONG" in a:
                try:
                    self.origin = (float(a["LAT"]), float(a["LONG"]))
                    if name == "UPDATEPOS":
                        self._ins_pos[:2] = 0.0
                except ValueError:
                    return False, []
            if name == "UPDATEPOS" and ("X" in a or "Y" in a):
                self._ins_pos[0] = float(a.get("X", self._ins_pos[0]))
                self._ins_pos[1] = float(a.get("Y", self._ins_pos[1]))
            return True, []
        if name == "SETTRIG" and "FREQ" in a:
            f = float(a["FREQ"])
            if not 1.0 <= f <= 8.0:
                return False, []
            self.trig_hz = f
            if "ALTI" in a:
                self.alti_every = int(float(a["ALTI"]))
            return True, []
        if name == "SETAHRS" and "FREQ" in a:
            self.ahrs_hz = max(1.0, min(float(a["FREQ"]), 100.0))
            return True, []
        if name == "SETNAV":
            if "DS" in a:
                self.nav_on = a["DS"].upper() != "OFF"
            if "FREQ" in a:
                self.nav_hz = max(1.0, min(float(a["FREQ"]), 50.0))
            return True, []
        if name == "SETINST" and "TYPE" in a:
            self.inst_type = a["TYPE"].upper()
            return True, []
        if name.startswith("SET") or name in ("SAVE", "REBOOT"):
            return True, []
        if name.startswith("GET"):
            for line in self._getall():
                if line.split(",", 1)[0] == name:
                    return True, [line.split(",", 1)[1] if "," in line else ""]
            return True, [""]
        return False, []

    def _getall(self) -> list:
        lat, lon = self.origin if self.origin is not None else (9999.0, 9999.0)
        clock = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(self._stamp() if self.posix else time.time()))
        return [s.format(sn=self.serial, clock=clock, inst=self.inst_type, lat=lat, lon=lon,
                         sv=self.dvl.p.sound_speed_mps, ahrs_hz=self.ahrs_hz, trig_hz=self.trig_hz,
                         alti=self.alti_every) for s in GETALL_TEMPLATE]

    def command(self, req) -> dict:
        """The A50-style JSON command of the control channel: ``set_config`` {acoustic_enabled: bool}."""
        name = str(req.get("command", "")) if isinstance(req, dict) else ""
        params = req.get("parameters") if isinstance(req, dict) else None
        error = ""
        if name == "get_config":
            pass
        elif name == "set_config":
            for k, v in (params or {}).items():
                if k != "acoustic_enabled":
                    error = f"unknown parameter '{k}' (the Nucleus emulator only knows acoustic_enabled)"
                elif not isinstance(v, bool):
                    error = f"{k} must be a boolean"
                else:
                    self.config[k] = v
        else:
            error = f"unknown command '{name}'"
        return {"response_to": name, "success": not error, "error_message": error, "result": dict(self.config)}

    # ------------------------------------------------------------------------------------------ the network
    def start(self) -> "Nucleus":
        dev = self

        class Tcp(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        class Cmd(socketserver.BaseRequestHandler):
            def handle(self):
                sock = self.request
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                lock = threading.Lock()
                with dev._clients_lock:
                    busy = dev._cmd_client is not None
                if busy:                                 # the device takes one client on its command port
                    log.warning("nucleus: a second client on the command port from %s, refused", self.client_address[0])
                    return
                log.info("nucleus: command client %s:%d", *self.client_address[:2])
                try:
                    sock.sendall(b"Please enter password:\r\n")
                    buf = b""
                    logged = False
                    while not dev._stop.is_set():
                        data = sock.recv(4096)
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            line = raw.rstrip(b"\r").decode(errors="replace")
                            if not logged:
                                if line == dev.password:
                                    logged = True
                                    with lock:
                                        sock.sendall(b"Welcome to Nortek Nucleus1000\r\n")
                                    with dev._clients_lock:
                                        dev._cmd_client = (sock, lock)
                                else:
                                    with lock:
                                        sock.sendall(b"Wrong password\r\nPlease enter password:\r\n")
                                continue
                            if not line.strip():
                                continue
                            out = dev.handle_line(line)
                            with lock:                    # never inside a binary packet
                                sock.sendall(out)
                except OSError:
                    pass
                finally:
                    with dev._clients_lock:
                        if dev._cmd_client is not None and dev._cmd_client[0] is sock:
                            dev._cmd_client = None
                    log.info("nucleus: command client %s:%d gone", *self.client_address[:2])

        class Data(socketserver.BaseRequestHandler):
            def handle(self):
                sock = self.request
                with dev._clients_lock:
                    if len(dev._data_clients) >= 2:
                        log.warning("nucleus: a third client on the data port, refused")
                        return
                    dev._data_clients[sock] = threading.Lock()
                log.info("nucleus: data client %s:%d", *self.client_address[:2])
                try:
                    while not dev._stop.is_set():
                        if not sock.recv(4096):          # input is ignored; EOF ends the client
                            break
                except OSError:
                    pass
                finally:
                    with dev._clients_lock:
                        dev._data_clients.pop(sock, None)

        try:
            self._tcp = Tcp((self.bind, self.tcp_port), Cmd)
            self._data = Tcp((self.bind, self.data_port), Data)
        except OSError as exc:
            if self._tcp is not None:
                self._tcp.server_close()
            raise OSError(exc.errno, f"Nucleus1000: cannot listen on {self.bind} TCP {self.tcp_port} / {self.data_port} "
                                     f"({exc.strerror}): is another uw pool or physics_proc running?") from exc
        self.tcp_port = self._tcp.server_address[1]
        self.data_port = self._data.server_address[1]
        for target, name in ((self._tcp.serve_forever, "nucleus-cmd"), (self._data.serve_forever, "nucleus-data"),
                             (self._send_loop, "nucleus-send")):
            th = threading.Thread(target=target, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        log.info("nucleus: Nucleus1000 on %s, TCP %d (commands, password) and %d (data)", self.bind, self.tcp_port,
                 self.data_port)
        return self

    def _send_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._out.get(timeout=0.2)
            except queue.Empty:
                continue
            with self._clients_lock:
                targets = list(self._data_clients.items())
                if self._cmd_client is not None:
                    targets.append(self._cmd_client)
            for sock, lock in targets:
                try:
                    with lock:
                        sock.settimeout(1.0)
                        sock.sendall(data)
                except OSError:
                    with self._clients_lock:
                        self._data_clients.pop(sock, None)

    @property
    def clients(self) -> int:
        with self._clients_lock:
            return len(self._data_clients) + (1 if self._cmd_client is not None else 0)

    def close(self) -> None:
        self._stop.set()
        for srv in (self._tcp, self._data):
            if srv is not None:
                srv.shutdown()
                srv.server_close()
        log.info("nucleus: closed after %d AHRS, %d bottom-track (%d locked), %d altimeter, %d INS packets, "
                 "%d commands", self.count["ahrs"], self.count["bottom_track"], self.count["locked"],
                 self.count["altimeter"], self.count["ins"], self.count["commands"])


def pool_device(cfg: dict):
    """The pool's Nucleus1000 (config ``nucleus:``), or None when ``nucleus.enabled`` or ``nucleus.device.enabled``
    is off (or the section is absent)."""
    from ..config import ConfigError
    try:
        if not (get(cfg, "nucleus.enabled") and get(cfg, "nucleus.device.enabled")):
            return None
    except ConfigError:
        return None
    from ..pool_geometry import PoolBottom
    from . import dvl as dvl_model
    return Nucleus(dvl_model.from_config(cfg, PoolBottom(cfg), section="nucleus"), cfg)
