"""Water Linked DVL A50: a four-beam convex Janus bottom-track log, in plain numpy. Also any N-beam (N >= 3)
bottom-track DVL with the same physics (M12: the Nortek Nucleus1000's three beams, ``vehicles/common/nucleus.py``): the
beam count is the length of ``beams.azimuths_deg``, the tilt may differ per beam, and ``min_good_beams`` says how
many must return for a velocity.

The model produces the device's own *velocity-and-transducer report*, field for field, so the interface
downstream of it is the same in simulation and on hardware: ``ros/src/dvl_relay`` turns a report into
``marine_acoustic_msgs/Dvl`` exactly as the real driver would, and on the vehicle the A50 and its driver
replace this module without anything below them changing (``docs/dvl.md``).

What is verified against Water Linked's documentation and what is assumed is tagged per value in
``sim/config/tank.yaml``'s ``dvl:`` section. The short version: the 22.5 deg beam tilt, the 4.4 deg beam
width, the FRD instrument axes (x forward, y right, z down), the 0.05-50 m altitude envelope, the
0.1 mm/s resolution, the 2-15 Hz report rate and the +-1.01 % long-term accuracy are published; the beam
*azimuths* come from a third-party driver, not from Water Linked, and the per-sample noise and the slow
bias are published nowhere at all.

Frames. The instrument frame is ArduPilot's body convention, FRD, which is also what
``marine_acoustic_msgs/Dvl`` declares for itself ("z-axis in direction DVL is pointing, x-axis aligned
with vehicle forward"). The single conversion to ROS's FLU happens in the relay, once. Inside here the
only other frame is the tank's ENU, needed to cast the beams at the sand, and ``vehicles.common.frames`` is the
only thing that converts.

Determinism. Every error term is a pure function of the seed and either the sample index or the
simulated time - never of a wall clock and never of a shared mutable generator - so two runs of the
same scenario give bit-identical reports, which is the rule the rest of ``vehicles.common`` already follows.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from physics import geometry as geom
from uw.config import ConfigError, get
from . import frames as fr

Z_WATER_MRAYL = 1.5              # nominal; only sets where the reflection coefficient lands


# ------------------------------------------------------------------------------------------ parameters

@dataclass(frozen=True)
class Params:
    """Everything the model needs from ``sim/config/tank.yaml``'s ``dvl:`` section, already resolved."""
    tilt_deg: float                  # the first beam's (all of them, for the A50)
    azimuths_deg: tuple
    width_deg: float
    min_altitude_m: float
    max_altitude_m: float
    max_speed_mps: float
    resolution_mps: float
    rate_hz_min: float
    rate_hz_max: float
    rate_altitude_m: float
    rate_jitter: float
    scale_sigma: float
    beam_noise_mps: float
    bias_sigma_mps: float
    bias_time_s: float
    rssi_ref_dbm: float
    nsd_dbm: float
    snr_threshold_db: float
    sound_speed_mps: float
    noise: bool = True
    dropout: bool = True
    tilts_deg: tuple = ()            # per beam; empty: tilt_deg for every beam
    min_good_beams: int = 3          # beams that must return for a velocity (3 of 4 for the A50, 3 of 3 for 3 beams)
    model: str = "Water Linked DVL A50"

    @property
    def tilt_rad(self) -> float:
        return math.radians(self.tilt_deg)

    @property
    def n_beams(self) -> int:
        return len(self.azimuths_deg)

    @property
    def uniform_tilt(self) -> bool:
        return not self.tilts_deg or all(t == self.tilt_deg for t in self.tilts_deg)


@dataclass(frozen=True)
class Mount:
    """Where the instrument sits on the vehicle. Both in ArduPilot body FRD."""
    position_frd_m: np.ndarray       # the lever arm r, from the body origin to the acoustic centre
    rot_body_to_dvl: np.ndarray      # (3, 3) body FRD -> instrument FRD


def params_from_config(cfg: dict, section: str = "dvl") -> Params:
    """Validate and resolve ``dvl:`` (or another section of the same shape, e.g. ``nucleus:``). Raises
    ``ConfigError`` on a value outside the datasheet."""
    from physics import water
    d = get(cfg, section)
    version = str(d["version"])
    if version not in d["scale_sigma"]:
        raise ConfigError(f"{section}.version '{version}' not in {section}.scale_sigma {sorted(d['scale_sigma'])}")
    b = d["beams"]
    az = tuple(float(a) for a in b["azimuths_deg"])
    if len(az) < 3:
        raise ConfigError(f"{section}.beams.azimuths_deg needs at least 3 beams, got {len(az)}")
    raw = b["tilt_deg"]
    tilts = tuple(float(t) for t in raw) if isinstance(raw, (list, tuple)) else (float(raw),) * len(az)
    if len(tilts) != len(az):
        raise ConfigError(f"{section}.beams.tilt_deg: {len(tilts)} tilts for {len(az)} beams")
    for tilt in tilts:
        if not 0.0 < tilt < 90.0:
            raise ConfigError(f"{section}.beams.tilt_deg {tilt} must be between 0 and 90 (from the vertical)")
    tilt = tilts[0]
    min_good = int(b.get("min_good", 3))
    if not 3 <= min_good <= len(az):
        raise ConfigError(f"{section}.beams.min_good {min_good} must be 3..{len(az)} (three beams fix 3-D velocity)")
    lo, hi = (float(x) for x in d["rate_hz"])
    if not 0.0 < lo <= hi:
        raise ConfigError(f"{section}.rate_hz {d['rate_hz']} must be [min, max] with 0 < min <= max")
    if float(d["min_altitude_m"]) >= float(d["max_altitude_m"]):
        raise ConfigError(f"{section}.min_altitude_m {d['min_altitude_m']} exceeds max_altitude_m "
                          f"{d['max_altitude_m']}")
    c = float(water.sound_speed(float(get(cfg, "water.temperature_c")),
                                float(get(cfg, "water.salinity_psu"))))
    return Params(
        tilt_deg=tilt, azimuths_deg=az, width_deg=float(b["width_deg"]),
        min_altitude_m=float(d["min_altitude_m"]), max_altitude_m=float(d["max_altitude_m"]),
        max_speed_mps=float(d["max_speed_mps"]), resolution_mps=float(d["resolution_mps"]),
        rate_hz_min=lo, rate_hz_max=hi, rate_altitude_m=float(d["rate_altitude_m"]),
        rate_jitter=float(d["rate_jitter"]), scale_sigma=float(d["scale_sigma"][version]),
        beam_noise_mps=float(d["beam_noise_mps"]), bias_sigma_mps=float(d["bias_sigma_mps"]),
        bias_time_s=float(d["bias_time_s"]), rssi_ref_dbm=float(d["rssi_ref_dbm"]),
        nsd_dbm=float(d["nsd_dbm"]), snr_threshold_db=float(d["snr_threshold_db"]),
        sound_speed_mps=c, noise=bool(d["noise"]), dropout=bool(d["dropout"]), tilts_deg=tilts,
        min_good_beams=min_good, model=str(d.get("model", "Water Linked DVL A50")))


def mount_from_config(cfg: dict, section: str = "dvl") -> Mount:
    """The mount. ``rotation_offset_deg`` is Water Linked's ``mounting_rotation_offset``, measured
    clockwise about z (which points down in FRD), hence the sign against a right-handed yaw."""
    m = get(cfg, f"{section}.mount")
    rot_dvl_to_body = fr.rot_from_rpy_deg([float(m["roll_deg"]), float(m["pitch_deg"]),
                                          -float(m["rotation_offset_deg"])])
    return Mount(position_frd_m=np.asarray(m["position_frd_m"], float),
                 rot_body_to_dvl=rot_dvl_to_body.T)


def beam_directions(params: Params) -> np.ndarray:
    """(n, 3) unit beam axes in the instrument frame (FRD, so +z is down and every beam has +z > 0)."""
    n = params.n_beams
    th = params.tilt_rad if params.uniform_tilt else np.radians(np.asarray(params.tilts_deg, float))
    az = np.radians(np.asarray(params.azimuths_deg, float))
    return np.stack([np.sin(th) * np.cos(az), np.sin(th) * np.sin(az), np.full(n, np.cos(th))], 1)


def beam_cosines(params: Params):
    """cos(tilt) of each beam: a scalar when they all share one tilt (the A50), else an (n,) array."""
    if params.uniform_tilt:
        return math.cos(params.tilt_rad)
    return np.cos(np.radians(np.asarray(params.tilts_deg, float)))


# ---------------------------------------------------------------------------------------- the report

@dataclass(frozen=True)
class Report:
    """One velocity-and-transducer report, mirroring the A50's JSON message."""
    seq: int
    stamp: float                     # simulated seconds, the time the velocity is valid at
    dt_ms: float                     # since the previous report, the device's "time" field
    velocity: np.ndarray             # (3,) instrument FRD [m/s], quantised
    covariance: np.ndarray           # (3, 3) [(m/s)^2], -1 when there is no fix
    fom: float                       # [m/s]
    altitude: float                  # [m] along the instrument's z
    velocity_valid: bool
    status: int
    num_good_beams: int
    beam_unit_vec: np.ndarray        # (n, 3) instrument FRD
    beam_range: np.ndarray           # (n,) [m], nan where the beam did not return
    beam_velocity: np.ndarray        # (n,) [m/s] along each beam
    beam_valid: np.ndarray           # (n,) bool
    rssi_dbm: np.ndarray             # (n,)
    nsd_dbm: np.ndarray              # (n,)
    sound_speed: float

    def as_json(self) -> dict:
        """The device's own field names and units (``format: json_v3.3``, ``type: velocity``).

        ``time_of_validity`` and ``time_of_transmission`` are microseconds: the real device reports Unix
        time, the simulator reports simulated time, as every other stamp in this project does.
        """
        return {
            "time": round(self.dt_ms, 3),
            "vx": float(self.velocity[0]), "vy": float(self.velocity[1]), "vz": float(self.velocity[2]),
            "fom": float(self.fom),
            "covariance": [[float(v) for v in row] for row in self.covariance],
            "altitude": float(self.altitude),
            "transducers": [
                {"id": i, "velocity": float(self.beam_velocity[i]),
                 "distance": (None if not np.isfinite(self.beam_range[i])
                              else float(self.beam_range[i])),
                 "rssi": float(self.rssi_dbm[i]), "nsd": float(self.nsd_dbm[i]),
                 "beam_valid": bool(self.beam_valid[i])}
                for i in range(len(self.beam_range))],
            "velocity_valid": bool(self.velocity_valid),
            "status": int(self.status),
            "time_of_validity": int(round(self.stamp * 1e6)),
            "time_of_transmission": int(round(self.stamp * 1e6)),
            "format": "json_v3.3",
            "type": "velocity",
        }


# -------------------------------------------------------------------------------------------- errors

class _BiasField:
    """A slow per-beam bias as a sum of random sinusoids: a pure function of time, not a recursion.

    An Ornstein-Uhlenbeck walk would need state carried between samples, so its output would depend on
    which samples were taken. A fixed random spectrum gives the same correlation time with none of that:
    identical for a given seed whatever the schedule, which is what the determinism test needs. The
    construction is the one ``physics.geometry.SandField`` uses for the sand relief.
    """

    def __init__(self, sigma: float, tau: float, seed: int, n: int = 6, beams: int = 4):
        rng = np.random.default_rng([int(seed), 4478540])
        self.a = sigma * math.sqrt(2.0 / n)
        self.w = 2 * np.pi / (max(tau, 1e-6) * rng.uniform(1.0 / 3.0, 3.0, (beams, n)))
        self.ph = rng.uniform(0, 2 * np.pi, (beams, n))

    def at(self, t: float) -> np.ndarray:
        return self.a * np.sum(np.sin(self.w * float(t) + self.ph), 1)


# ----------------------------------------------------------------------------------------- the model

class TankBottom:
    """The tank as the DVL's bottom: ``physics.geometry.ray_bottom`` over the sand and the meshes (the sonar's own
    occluders), with the materials of sim/config/tank.yaml.

    The bottom interface ``Dvl`` needs, which ``physics.pool_geometry.PoolBottom`` also provides: ``ray_bottom(origin,
    directions, t_max)``, ``materials`` (name -> Z or reflection, lambert_db, rms_slope) and ``role_material``
    (what a hit's role is made of)."""

    def __init__(self, scene, cfg: dict):
        self.scene = scene
        self.materials = geom.surface_materials(cfg)
        self.role_material = {"sand": "sand", "rock": get(cfg, "rocks.material"),
                              "pipe": get(cfg, "pipe.material"), "flange": get(cfg, "pipe.material"),
                              "support": "concrete"}

    def ray_bottom(self, origin, directions, t_max: float = 50.0):
        return geom.ray_bottom(self.scene, origin, directions, t_max=t_max)


class Dvl:
    """The sensor. ``due(t)`` owns the altitude-dependent schedule, ``sample(...)`` makes one report.

    ``bottom`` is what the beams are cast at: a ``TankBottom`` or a ``physics.pool_geometry.PoolBottom``."""

    def __init__(self, params: Params, mount: Mount, bottom, seed: int = 0):
        self.p = params
        self.mount = mount
        self.bottom = bottom
        self.scene = getattr(bottom, "scene", None)
        self.seed = int(seed)
        self.beams = beam_directions(params)
        self.seq = 0
        self._alt = None                     # last altitude, which sets the next period
        self._last_stamp = None
        self.n = params.n_beams
        self._bias = _BiasField(params.bias_sigma_mps, params.bias_time_s, self.seed, beams=self.n)
        # one scale factor per run, drawn once: +-1.01 % is a long-term accuracy, not a per-ping error
        self._scale = 1.0 + float(np.random.default_rng([self.seed, 5456451]).normal(0.0,
                                                                                     params.scale_sigma))
        mats = bottom.materials
        self._mu = {k: 10 ** (float(v["lambert_db"]) / 10.0) for k, v in mats.items()}
        self._s2 = {k: max(float(v.get("rms_slope", 0.25)), 1e-3) ** 2 for k, v in mats.items()}
        self._r0 = {k: (float(v["reflection"]) if v.get("reflection") is not None
                        else (float(v["Z"]) - Z_WATER_MRAYL) / (float(v["Z"]) + Z_WATER_MRAYL))
                    for k, v in mats.items()}
        self._role_material = dict(bottom.role_material)
        self._next = self._period(None)

    # -- schedule ------------------------------------------------------------------------------------

    def _period(self, altitude) -> float:
        """The report period. The rate falls from the maximum towards the minimum as the altitude grows:
        a longer two-way travel time means fewer pings per second. Water Linked publishes only the
        2-15 Hz envelope, so the shape between its ends is assumed (``dvl.rate_altitude_m``)."""
        p = self.p
        if altitude is None or not np.isfinite(altitude):
            f = p.rate_hz_max
        else:
            u = min(max(float(altitude) / max(p.rate_altitude_m, 1e-6), 0.0), 1.0)
            f = p.rate_hz_max + u * (p.rate_hz_min - p.rate_hz_max)
        period = 1.0 / max(f, 1e-6)
        if p.rate_jitter > 0.0 and p.noise:
            j = np.random.default_rng([self.seed, 74, self.seq]).uniform(-p.rate_jitter, p.rate_jitter)
            period *= 1.0 + float(j)
        return period

    def due(self, t: float) -> bool:
        """True once per report period of simulated time. Unlike ``sim.main.RateTimer`` the period varies,
        so the schedule is carried rather than derived from a fixed grid; the first report is at one
        period, as everywhere else in the loop."""
        if t + 1e-9 < self._next:
            return False
        self._next = t + self._period(self._alt)
        return True

    # -- measurement ---------------------------------------------------------------------------------

    def geometry_at(self, position_tank, rot_flu_enu):
        """The acoustic centre and the beam axes, in tank ENU."""
        rot = np.asarray(rot_flu_enu, float)
        origin = np.asarray(position_tank, float) + rot @ fr.vec_flu_frd(self.mount.position_frd_m)
        body = self.beams @ self.mount.rot_body_to_dvl                 # instrument -> body FRD
        flu = body * np.array([1.0, -1.0, -1.0])                       # fr.vec_flu_frd, row-wise
        return origin, flu @ rot.T

    def _echo(self, ranges, cos_i, roles):
        """Per-beam received level [dBm] from the same backscatter law the sonar uses.

        Monostatic Lambert plus the Kirchhoff term of a rough surface with Gaussian slopes - the
        reduction of ``physics.sonar.emit`` to a single direction, identical in form to
        ``physics.surface.boil_kernel`` - over the beam's footprint, spread as 1/r^4. Water absorption is
        left out: at 1 MHz over the few metres this sensor ever sees in the tank it is a constant offset
        that ``rssi_ref_dbm`` already absorbs.
        """
        half = math.radians(self.p.width_deg) / 2.0
        out = np.full(self.n, -np.inf)
        for i in range(self.n):
            if not np.isfinite(ranges[i]) or cos_i[i] <= 1e-3:
                continue
            m = self._role_material.get(roles[i] or "", "sand")
            mu = self._mu.get(m, 1e-3)
            s2 = self._s2.get(m, 0.0625)
            r0 = self._r0.get(m, 0.4)
            c = float(cos_i[i])
            t2 = (1.0 - c * c) / (c * c)
            sigma = mu * c * c + r0 * r0 * math.exp(-t2 / (2.0 * s2)) / (8.0 * math.pi * s2 * c ** 4)
            area = math.pi * (float(ranges[i]) * math.tan(half)) ** 2
            level = max(sigma * area / float(ranges[i]) ** 4, 1e-30)
            out[i] = self.p.rssi_ref_dbm + 10.0 * math.log10(level)
        return out

    def sample(self, t: float, position_tank, rot_flu_enu, v_body_frd, w_body_frd) -> Report:
        """One report at simulated time ``t``.

        ``position_tank`` / ``rot_flu_enu`` are the vehicle body's pose (tank ENU, body FLU);
        ``v_body_frd`` / ``w_body_frd`` are its twist in ArduPilot body FRD, which is what
        ``vehicles.common.dynamics.State`` carries. The lever arm is applied here, at the source, so the report
        describes the instrument and not the body - undoing it is the estimator's job.
        """
        p = self.p
        seq = self.seq
        self.seq += 1
        rng = np.random.default_rng([self.seed, 78, seq])
        rot = np.asarray(rot_flu_enu, float)
        origin, dirs = self.geometry_at(position_tank, rot)

        cos_t = beam_cosines(p)
        ranges, normals, roles = self.bottom.ray_bottom(origin, dirs,
                                                        t_max=p.max_altitude_m / max(float(np.min(cos_t)), 1e-6))
        altitudes = ranges * cos_t
        cos_i = np.abs(np.einsum("ij,ij->i", normals, dirs))
        rssi = self._echo(ranges, cos_i, roles)
        nsd = np.full(self.n, p.nsd_dbm)
        snr = rssi - nsd
        in_gate = (np.isfinite(ranges) & (altitudes >= p.min_altitude_m)
                   & (altitudes <= p.max_altitude_m))
        valid = (in_gate & (snr > p.snr_threshold_db)) if p.dropout else in_gate

        # the true velocity of the instrument, lever arm included, in the instrument frame
        v_inst = self.mount.rot_body_to_dvl @ (np.asarray(v_body_frd, float)
                                               + fr.cross3(w_body_frd, self.mount.position_frd_m))
        vb = self.beams @ v_inst
        sigma_i = np.full(self.n, p.beam_noise_mps)
        if p.noise:
            # a degraded beam is a noisier beam: its noise floor rises as the echo approaches it
            gain = np.clip(10 ** (-(snr - p.snr_threshold_db) / 20.0), 1.0, 1e3)
            sigma_i = p.beam_noise_mps * np.where(np.isfinite(gain), gain, 1.0)
            vb = self._scale * vb + self._bias.at(t) + rng.normal(0.0, 1.0, self.n) * sigma_i

        n_good = int(valid.sum())
        velocity = np.zeros(3)
        cov = np.full((3, 3), -1.0)
        fom = float("inf")
        velocity_valid = n_good >= max(3, p.min_good_beams)
        if velocity_valid:
            h = self.beams[valid]
            y = vb[valid]
            velocity, *_ = np.linalg.lstsq(h, y, rcond=None)
            gram = np.linalg.inv(h.T @ h)
            var = float(np.mean(sigma_i[valid] ** 2))
            if n_good > 3:                       # the redundant beam measures the error velocity
                resid = y - h @ velocity
                var = max(var, float(resid @ resid) / (n_good - 3))
            cov = var * gram
            fom = float(math.sqrt(max(float(np.linalg.eigvalsh(cov).max()), 0.0)))
            velocity = np.round(velocity / p.resolution_mps) * p.resolution_mps
            if float(np.max(np.abs(velocity))) > p.max_speed_mps:
                velocity_valid = False           # outside the tracking envelope: the device loses lock
        altitude = float(np.mean(altitudes[valid])) if n_good else float("nan")
        self._alt = altitude if np.isfinite(altitude) else None

        dt_ms = 0.0 if self._last_stamp is None else (float(t) - self._last_stamp) * 1e3
        self._last_stamp = float(t)
        return Report(
            seq=seq, stamp=float(t), dt_ms=dt_ms, velocity=velocity, covariance=cov, fom=fom,
            altitude=altitude, velocity_valid=bool(velocity_valid), status=0, num_good_beams=n_good,
            beam_unit_vec=self.beams.copy(), beam_range=np.where(valid, ranges, np.nan),
            beam_velocity=np.where(valid, vb, 0.0), beam_valid=valid.copy(),
            rssi_dbm=np.where(np.isfinite(rssi), rssi, p.nsd_dbm), nsd_dbm=nsd,
            sound_speed=p.sound_speed_mps)

    # -- the latched description of the instrument ---------------------------------------------------

    def meta(self) -> dict:
        """What the relay needs to build ``marine_acoustic_msgs/Dvl``, and what an operator needs to read
        a report. Deliberately carries no pose and no scene quantity: it is latched, and
        ``docs/ros_hil.md`` 2.5 rule 1 keeps ground truth out of latched metadata."""
        p = self.p
        return {
            "model": p.model,
            "beam_unit_vec": self.beams.tolist(),
            "tilt_deg": p.tilt_deg,
            "azimuths_deg": list(p.azimuths_deg),
            "beam_width_deg": p.width_deg,
            "mount_position_frd_m": self.mount.position_frd_m.tolist(),
            "min_altitude_m": p.min_altitude_m,
            "max_altitude_m": p.max_altitude_m,
            "max_speed_mps": p.max_speed_mps,
            "resolution_mps": p.resolution_mps,
            "rate_hz": [p.rate_hz_min, p.rate_hz_max],
            "sound_speed_mps": p.sound_speed_mps,
            "frame": "FRD: x forward, y starboard, z down (the instrument frame, as the A50 reports)",
        }


# ------------------------------------------------------------------------------ the motion it rides on

def _unskew(s) -> np.ndarray:
    s = np.asarray(s, float)
    return 0.5 * np.array([s[2, 1] - s[1, 2], s[0, 2] - s[2, 0], s[1, 0] - s[0, 1]])


class BodyMotion:
    """The vehicle body's pose and body-FRD twist at a simulated time, from whatever drives the sim.

    Two sources, one interface. With ``sonar.pose_source ardusub`` the physics thread integrates the
    vehicle, so the twist is read out of ``vehicles.common.dynamics.State`` rather than reconstructed. Every other
    pose source carries position and attitude only (``sim.pose`` returns a ``Pose``), so the twist is a
    central difference of the body pose - which is all a kinematic trajectory can honestly offer, there
    being no dynamics behind it.

    The pose sources report the *sonar*, so the body pose is recovered by undoing the sonar's mount: the
    inverse of what ``vehicles.common.physics_udp.ArduSubPose.at`` applies. Sources that know their own body pose
    (``ScriptPose``, ``ArduSubPose``) are asked for it directly.
    """

    def __init__(self, motion, r_mount, p_mount, h: float = 1e-3):
        self.motion = motion
        self.r_mount = np.asarray(r_mount, float)
        self.p_mount = np.asarray(p_mount, float)
        self.h = float(h)
        self.exact = hasattr(motion, "body_twist")

    def _pose(self, t: float):
        if hasattr(self.motion, "body_at"):
            return self.motion.body_at(t)
        pose = self.motion.at(t)
        rot = pose.rot @ self.r_mount.T
        return pose.position - rot @ self.p_mount, rot

    def at(self, t: float):
        """``(position tank ENU, body FLU -> ENU rotation, v_body_frd, w_body_frd)``."""
        p, rot = self._pose(t)
        if self.exact:
            v, w, _ = self.motion.body_twist()
            return p, rot, np.asarray(v, float), np.asarray(w, float)
        t0 = max(float(t) - self.h, 0.0)
        t1 = t0 + 2.0 * self.h
        p0, r0 = self._pose(t0)
        p1, r1 = self._pose(t1)
        dt = t1 - t0
        v_enu = (np.asarray(p1, float) - np.asarray(p0, float)) / dt
        omega_flu = _unskew(rot.T @ ((np.asarray(r1, float) - np.asarray(r0, float)) / dt))
        return p, rot, fr.vec_flu_frd(rot.T @ v_enu), fr.angvel_flu_frd(omega_flu)


def from_config(cfg: dict, scene, section: str = "dvl"):
    """The configured DVL, or ``None`` when ``<section>.enabled`` is false. ``scene`` is the tank's
    ``physics.geometry`` scene, or already a bottom (anything with ``ray_bottom``, e.g.
    ``physics.pool_geometry.PoolBottom``). A section other than ``dvl`` (a second instrument, ``nucleus``) gets its
    own seed, so that two DVLs on one vehicle do not share their errors; ``dvl`` keeps the run's seed."""
    if not bool(get(cfg, f"{section}.enabled")):
        return None
    bottom = scene if hasattr(scene, "ray_bottom") else TankBottom(scene, cfg)
    seed = int(cfg["seed"])
    if section != "dvl":
        import zlib
        seed += zlib.crc32(section.encode())
    return Dvl(params_from_config(cfg, section), mount_from_config(cfg, section), bottom, seed=seed)
