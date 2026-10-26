"""Where the sonar is (plan: SensorPose = fixed | trajectory | ros): a pose per time, numpy only.

Poses are body-to-world (body: x forward, y port, z up; roll about x, pitch nose-down positive, yaw about z, the
convention of ``geometry.rot_rpy_deg``). Trajectory time 0 is the end of the plume warm-up: the first ping is at
1 / ping_rate.

Trajectories are waypoint lists, interpolated smoothly: positions and (unwrapped) angles by a monotone cubic
(Fritsch-Carlson, no overshoot), timed by each segment's speed (length / speed, or turn / turn rate). Presets build the waypoints from the scene (the leak, the
tank) with ``sonar.trajectory`` parameters:
- approach: from start_distance to end_distance towards the leak, at a height, from a bearing around it;
- lawnmower: lanes across the tank at a height, sonar looking along the lane;
- orbit: a circle of a radius around the leak, the sonar looking at it;
- hover: one pose with Ornstein-Uhlenbeck drift in position and yaw;
- waypoints: the list in ``sonar.trajectory.waypoints``;
- poses: the fixed poses in ``sonar.trajectory.poses``, each held for ``hold_s`` and then jumped to the next (several
  views of one scene: datasets).
Every position is kept inside the water with ``margin`` from the walls, the sand and the surface. Pitch "track" points
the sonar's axis at the leak's height (``look_at``), else it is fixed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import ConfigError, get
from .geometry import rot_rpy_deg


@dataclass
class Pose:
    position: np.ndarray            # (3,) acoustic centre [m]
    rpy_deg: np.ndarray             # (3,) roll, pitch (nose down +), yaw

    @property
    def rot(self) -> np.ndarray:
        return rot_rpy_deg(self.rpy_deg)

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = self.rot, self.position
        return m


class SensorPose:
    moving = True

    def at(self, t: float) -> Pose:
        raise NotImplementedError

    def positions(self, step: float = 0.05) -> np.ndarray:
        """Positions along the whole motion (every ``step`` metres or so): what the adaptive scatterer spacing uses."""
        return self.at(0.0).position[None, :]


class FixedPose(SensorPose):
    moving = False

    def __init__(self, pose: Pose):
        self.pose = pose

    def at(self, t: float) -> Pose:
        return self.pose


class Poses(SensorPose):
    """Fixed poses one after the other, each held for ``hold_s`` (pose i from i hold_s to (i + 1) hold_s; the last
    one is kept)."""

    def __init__(self, poses: list[Pose], hold_s: float):
        if not poses:
            raise ConfigError("sonar.trajectory.poses is empty")
        self.poses, self.hold_s = poses, float(hold_s)

    def at(self, t: float) -> Pose:
        i = math.floor(t / self.hold_s - 1e-9)       # a ping exactly at the end of a hold still belongs to it
        return self.poses[min(max(i, 0), len(self.poses) - 1)]

    def positions(self, step: float = 0.05) -> np.ndarray:
        return np.array([q.position for q in self.poses])


def _pchip(p: np.ndarray, t: np.ndarray, x: float) -> np.ndarray:
    """Monotone cubic Hermite (Fritsch-Carlson) through p (K, d) at knot times t (K,), at x (clamped to the ends):
    smooth, and never beyond its neighbouring waypoints (a path cannot overshoot into a wall at a corner)."""
    k = len(p)
    if k == 1 or x <= t[0]:
        return p[0].copy()
    if x >= t[-1]:
        return p[-1].copy()
    h = np.diff(t)
    delta = np.diff(p, axis=0) / h[:, None]
    m = np.zeros_like(p)
    m[0], m[-1] = delta[0], delta[-1]
    for j in range(1, k - 1):
        a, b = delta[j - 1], delta[j]
        w1, w2 = 2 * h[j] + h[j - 1], h[j] + 2 * h[j - 1]
        same = a * b > 0
        m[j] = np.where(same, (w1 + w2) / np.where(same, w1 / np.where(same, a, 1) + w2 / np.where(same, b, 1), 1), 0)
    i = int(np.searchsorted(t, x, side="right") - 1)
    s = (x - t[i]) / h[i]
    h00, h10, h01, h11 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * p[i] + h10 * h[i] * m[i] + h01 * p[i + 1] + h11 * h[i] * m[i + 1]


class Trajectory(SensorPose):
    """Waypoints {position, roll_deg, pitch_deg, yaw_deg, speed}: ``speed`` [m/s] of the segment to the next waypoint
    (``turn_rate_deg`` [deg/s] bounds a turn on the spot). Ends holding the last pose, or starts over (``loop``)."""

    def __init__(self, waypoints: list[dict], loop: bool = False):
        if not waypoints:
            raise ConfigError("a trajectory needs at least one waypoint")
        pts, ang, times = [], [], [0.0]
        for i, w in enumerate(waypoints):
            pts.append(np.asarray(w["position"], float))
            a = np.array([w.get("roll_deg", 0.0), w.get("pitch_deg", 0.0), w.get("yaw_deg", 0.0)], float)
            if ang:                                           # unwrap against the previous waypoint
                a = ang[-1] + (a - ang[-1] + 180.0) % 360.0 - 180.0
                prev = waypoints[i - 1]
                length = float(np.linalg.norm(pts[-1] - pts[-2]))
                turn = float(np.max(np.abs(a - ang[-1])))
                times.append(times[-1] + max(length / float(prev.get("speed", 0.2)),
                                             turn / float(prev.get("turn_rate_deg", 30.0)), 1e-3))
            ang.append(a)
        self.p, self.a, self.t = np.array(pts), np.array(ang), np.array(times)
        self.loop = loop
        self.duration = float(self.t[-1])

    def at(self, t: float) -> Pose:
        if self.loop and self.duration > 0:
            t = t % self.duration
        return Pose(_pchip(self.p, self.t, t), _pchip(self.a, self.t, t))

    def positions(self, step: float = 0.05) -> np.ndarray:
        length = float(np.sum(np.linalg.norm(np.diff(self.p, axis=0), axis=1))) if len(self.p) > 1 else 0.0
        n = max(2, int(length / step) + 2)
        return np.array([self.at(x).position for x in np.linspace(0.0, self.duration, n)])


class Hover(SensorPose):
    """A pose with Ornstein-Uhlenbeck drift: position (std drift_m per axis) and yaw (std drift_yaw_deg), correlation
    time drift_time_s; deterministic for a seed (precomputed at the ping rate)."""

    def __init__(self, pose: Pose, drift_m: float, drift_yaw_deg: float, drift_time_s: float, seed: int,
                 dt: float = 0.1, duration_s: float = 600.0, bounds=None):
        rng = np.random.default_rng([seed, 31])
        n = int(duration_s / dt) + 2
        a = math.exp(-dt / max(drift_time_s, 1e-3))
        q = math.sqrt(1 - a * a)
        x = np.zeros((n, 4))
        x[0] = rng.normal(size=4)
        for i in range(1, n):
            x[i] = a * x[i - 1] + q * rng.normal(size=4)
        self.pose, self.dt, self.bounds = pose, dt, bounds
        self.off = x * np.array([drift_m, drift_m, drift_m * 0.5, drift_yaw_deg])

    def at(self, t: float) -> Pose:
        f = min(max(t / self.dt, 0.0), len(self.off) - 1.000001)
        i = int(f)
        o = self.off[i] + (f - i) * (self.off[i + 1] - self.off[i])
        p = self.pose.position + o[:3]
        return Pose(self.bounds.clip(p) if self.bounds is not None else p,
                    self.pose.rpy_deg + np.array([0.0, 0.0, o[3]]))

    def positions(self, step: float = 0.05) -> np.ndarray:
        p = self.pose.position + self.off[::10, :3]
        return self.bounds.clip(p) if self.bounds is not None else p


# ------------------------------------------------------------------------------------------ presets

def _look(frm, at) -> tuple[float, float]:
    """Yaw and pitch (nose down +) [deg] that point the x axis from ``frm`` to ``at``."""
    d = np.asarray(at, float) - np.asarray(frm, float)
    return math.degrees(math.atan2(d[1], d[0])), math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))


def aim(position, target, roll_deg: float, bearing_deg: float = 0.0, elevation_deg: float = 0.0) -> tuple[float, float]:
    """Pitch (nose down +) and yaw [deg] that, with ``roll_deg``, put ``target`` at ``bearing_deg`` (+ starboard)
    and ``elevation_deg`` (+ up) in the sonar frame. Of the two pitches that do, the one nearer the plain look-at.
    A target too steep for that bearing (no pitch reaches it; never at bearing 0) gets the nearest pitch: check with
    ``in_sonar_frame``."""
    d = np.asarray(target, float) - np.asarray(position, float)
    d /= np.linalg.norm(d)
    b, e = math.radians(bearing_deg), math.radians(elevation_deg)
    u = np.array([math.cos(e) * math.cos(b), -math.cos(e) * math.sin(b), math.sin(e)])
    v = rot_rpy_deg([roll_deg, 0.0, 0.0]) @ u
    # pitch: the z of Ry(p) v, -sin p v_x + cos p v_z = rho cos(p + phi), must be d_z
    rho, phi = math.hypot(v[0], v[2]), math.atan2(v[0], v[2])
    c = math.acos(max(-1.0, min(1.0, d[2] / max(rho, 1e-12))))
    look = math.atan2(-d[2], math.hypot(d[0], d[1]))
    pitch = min(((s * c - phi + math.pi) % (2 * math.pi) - math.pi for s in (1.0, -1.0)), key=lambda q: abs(q - look))
    w = rot_rpy_deg([roll_deg, math.degrees(pitch), 0.0]) @ u
    yaw = math.atan2(d[1], d[0]) - math.atan2(w[1], w[0])
    return math.degrees(pitch), (math.degrees(yaw) + 180.0) % 360.0 - 180.0


def in_sonar_frame(position, rot, point) -> tuple[float, float, float]:
    """Range [m], bearing (+ starboard) and elevation (+ up) [deg] of a world point seen from a sonar pose."""
    q = np.asarray(rot, float).T @ (np.asarray(point, float) - np.asarray(position, float))
    rng = float(np.linalg.norm(q))
    return rng, math.degrees(math.atan2(-q[1], q[0])), math.degrees(math.asin(max(-1.0, min(1.0, q[2] / rng))))


def leak_target(scene) -> np.ndarray:
    """What the trajectories look at: the leak (hole, or the vents' mean), else the tank centre on the sand."""
    if scene.leak is not None:
        return scene.leak.vents.mean(0) if scene.leak.buried else scene.leak.hole
    return np.array([scene.size[0] / 2, scene.size[1] / 2, float(scene.sand.base)])


class Bounds:
    def __init__(self, scene, margin: float):
        self.lo = np.array([margin, margin, float(scene.sand.base) + margin + 0.1])
        self.hi = np.array([scene.size[0] - margin, scene.size[1] - margin, scene.water_level - margin])

    def clip(self, p) -> np.ndarray:
        return np.clip(np.asarray(p, float), self.lo, self.hi)


def _waypoint(p, yaw, pitch, roll, speed) -> dict:
    return {"position": [float(v) for v in p], "yaw_deg": float(yaw), "pitch_deg": float(pitch),
            "roll_deg": float(roll), "speed": float(speed)}


def preset_waypoints(tc: dict, scene) -> tuple[list[dict], bool]:
    """Waypoints and loop flag of a trajectory preset (``sonar.trajectory``)."""
    kind = tc["preset"]
    b = Bounds(scene, float(tc["margin"]))
    leak = leak_target(scene)
    h, v, roll = float(tc["height"]), float(tc["speed"]), float(tc["roll_deg"])

    def pose_at(p, look=None, yaw=None):
        p = b.clip(p)
        if look is not None:
            y, pt = _look(p, look)
        else:                                                # a survey heading: no target to track
            y, pt = float(yaw), float(tc["survey_pitch_deg"])
        pitch = pt if tc["pitch"] == "track" else float(tc["pitch"])
        return _waypoint(p, y, pitch, roll, v)

    if kind == "approach":
        az = math.radians(float(tc["bearing_deg"]))           # where the sonar comes from, seen from the leak
        ray = np.array([math.cos(az), math.sin(az), 0.0])
        room = min((b.hi[i] - leak[i]) / ray[i] if ray[i] > 1e-9 else (b.lo[i] - leak[i]) / ray[i] if ray[i] < -1e-9
                   else np.inf for i in (0, 1))              # the start stays on the ray, inside the bounds
        d0 = min(float(tc["start_distance"]), room)
        out = []
        for d in np.linspace(d0, min(float(tc["end_distance"]), d0), 6):
            p = leak + d * np.array([math.cos(az), math.sin(az), 0.0])
            p[2] = h
            out.append(pose_at(p, look=leak))
        return out, False
    if kind == "orbit":
        r, a0 = float(tc["radius"]), math.radians(float(tc["start_deg"]))
        sweep = math.radians(float(tc["sweep_deg"])) * (1 if tc["direction"] >= 0 else -1)
        out = []
        for a in a0 + np.linspace(0.0, sweep, max(4, int(abs(math.degrees(sweep)) / 20) + 1)):
            p = leak + r * np.array([math.cos(a), math.sin(a), 0.0])
            p[2] = h
            out.append(pose_at(p, look=leak))
        return out, abs(abs(math.degrees(sweep)) - 360.0) < 1e-6
    if kind == "lawnmower":
        lo, hi = b.lo, b.hi
        n_lanes = max(2, int(tc["lanes"]))
        out = []
        for i, y in enumerate(np.linspace(lo[1], hi[1], n_lanes)):
            xs = (lo[0], hi[0]) if i % 2 == 0 else (hi[0], lo[0])
            yaw = 0.0 if xs[0] < xs[1] else 180.0
            out += [pose_at([xs[0], y, h], yaw=yaw), pose_at([xs[1], y, h], yaw=yaw)]
        return out, False
    if kind == "waypoints":
        return [dict(w) for w in tc["waypoints"]], bool(tc.get("loop", False))
    raise ConfigError("sonar.trajectory.preset must be approach | lawnmower | orbit | hover | waypoints | poses, "
                      f"got {kind!r}")


def sensor_pose(cfg: dict, scene=None) -> SensorPose:
    """The SensorPose of ``sonar.pose_source``: fixed (sonar.position, roll/pitch/yaw), trajectory (preset or
    waypoints; needs the scene) or ros (the fixed pose until a pose arrives; uw/ros_bridge.RosPose wraps it)."""
    son = get(cfg, "sonar")
    fixed = Pose(np.asarray(son["position"], float), np.array([son["roll_deg"], son["pitch_deg"], son["yaw_deg"]],
                                                                float))
    src = son["pose_source"]
    if src in ("fixed", "ros"):
        return FixedPose(fixed)
    if src != "trajectory":
        raise ConfigError(f"sonar.pose_source must be fixed | trajectory | ros, got {src!r}")
    tc = son["trajectory"]
    if tc["preset"] == "poses":
        return Poses([Pose(np.asarray(w["position"], float),
                           np.array([w.get("roll_deg", 0.0), w.get("pitch_deg", 0.0), w.get("yaw_deg", 0.0)], float))
                      for w in tc["poses"]], float(tc["hold_s"]))
    if scene is None:
        from .geometry import build_scene
        scene = build_scene(cfg)
    if tc["preset"] == "hover":
        b = Bounds(scene, float(tc["margin"]))
        p = np.clip(fixed.position, b.lo + 3 * float(tc["drift_m"]), b.hi - 3 * float(tc["drift_m"]))
        y, pt = _look(p, leak_target(scene))
        pitch = pt if tc["pitch"] == "track" else float(tc["pitch"])
        return Hover(Pose(p, np.array([float(tc["roll_deg"]), pitch, y])), float(tc["drift_m"]),
                     float(tc["drift_yaw_deg"]), float(tc["drift_time_s"]), int(cfg["seed"]),
                     dt=1.0 / float(son["ping_rate_hz"]), bounds=b)
    wps, loop = preset_waypoints(tc, scene)
    return Trajectory(wps, loop=loop)
