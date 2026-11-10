"""Scripted ROV trajectories (sonar.pose_source script): config/trajectories/<name>.yaml, evaluated at sim time.

Deterministic and ROS-free. A trajectory moves the vehicle BODY (FLU, level, yawed so that its x axis points at the
leak); the sonar and the cameras ride it through the fixed mount (vehicle.body.offset, the sonar's configured roll
and pitch), exactly as with pose_source ardusub. Time 0 = the start of the main loop (after the plume warm-up);
after the end the last pose is held.

Kinds (all distances in metres, the leak position read from the scene, never hardcoded):
  circle    around the leak: radius_m, height_above_leak_m, speed_mps (along the circle), laps, start_bearing_deg
            (direction leak -> start; 180 = from -x), direction (+1 counter-clockwise seen from above)
  updown    up and down beside the plume at distance_m from the leak, bearing_deg: z sweeps leak_z +- amplitude_m,
            cycles, speed_mps (mean vertical speed); starts at the bottom
  sequence  segments: [names], joined by straight transits at transit_speed_mps

Bounds: the body centre keeps wall_margin_m + half the footprint diagonal from the walls and from the free surface,
floor_margin_m + half the height above the sand. A circle that does not fit gets a smaller radius, an updown that
reaches below the floor is shifted up (then its amplitude reduced if it hits the top); each change is a note that
tools/record.py writes into run.json.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .config import ConfigError, get

TRAJ_DIR = Path(__file__).resolve().parents[1] / "config" / "trajectories"


@dataclass
class Box:
    lo: np.ndarray
    hi: np.ndarray

    def contains(self, p, tol: float = 1e-9) -> bool:
        p = np.asarray(p, float)
        return bool(np.all(p >= self.lo - tol) and np.all(p <= self.hi + tol))


def body_box(cfg: dict, wall_margin: float, floor_margin: float) -> Box:
    size = np.asarray(get(cfg, "tank.size"), float)
    l, w, h = (float(v) for v in get(cfg, "vehicle.body.size"))
    hx = 0.5 * math.hypot(l, w)                       # any yaw
    lo = np.array([wall_margin + hx, wall_margin + hx, float(get(cfg, "sand.thickness")) + floor_margin + h / 2])
    hi = np.array([size[0] - wall_margin - hx, size[1] - wall_margin - hx,
                   float(get(cfg, "tank.water_level")) - wall_margin - h / 2])
    return Box(lo, hi)


def load_spec(name_or_path: str) -> dict:
    import yaml
    p = Path(name_or_path)
    if not p.suffix:
        p = TRAJ_DIR / f"{name_or_path}.yaml"
    if not p.is_file():
        avail = ", ".join(sorted(q.stem for q in TRAJ_DIR.glob("*.yaml")))
        raise ConfigError(f"trajectory '{name_or_path}' not found (config/trajectories: {avail})")
    spec = yaml.safe_load(p.read_text(encoding="utf-8-sig"))
    spec.setdefault("name", p.stem)
    return spec


@dataclass
class Segment:
    duration: float
    fn: object                                        # t in [0, duration] -> body position (3,)

    def at(self, t: float) -> np.ndarray:
        return self.fn(min(max(t, 0.0), self.duration))


@dataclass
class Trajectory:
    name: str
    leak: np.ndarray
    segments: list
    notes: list = field(default_factory=list)
    params: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.segments)

    def position(self, t: float) -> np.ndarray:
        for s in self.segments:
            if t <= s.duration:
                return s.at(t)
            t -= s.duration
        return self.segments[-1].at(self.segments[-1].duration)

    def yaw(self, p) -> float:
        d = self.leak[:2] - np.asarray(p, float)[:2]
        return math.atan2(d[1], d[0])

    def body(self, t: float):
        """(position, body FLU -> tank ENU rotation): level, x axis towards the leak."""
        p = self.position(t)
        y = self.yaw(p)
        c, s = math.cos(y), math.sin(y)
        return p, np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _circle(spec: dict, leak: np.ndarray, box: Box, notes: list) -> Segment:
    r = float(spec["radius_m"])
    z = float(leak[2]) + float(spec["height_above_leak_m"])
    zc = float(np.clip(z, box.lo[2], box.hi[2]))
    if zc != z:
        notes.append(f"{spec['name']}: height {z:.2f} m outside the bounds, clamped to {zc:.2f} m")
    r_fit = float(min(leak[0] - box.lo[0], box.hi[0] - leak[0], leak[1] - box.lo[1], box.hi[1] - leak[1]))
    if r > r_fit:
        notes.append(f"{spec['name']}: radius reduced from {r:.2f} to {r_fit:.2f} m to stay inside the bounds")
        r = r_fit
    if r <= 0.0:
        raise ConfigError(f"{spec['name']}: the leak is too close to a wall for a circle around it")
    v, laps = float(spec["speed_mps"]), float(spec["laps"])
    a0, sgn = math.radians(float(spec["start_bearing_deg"])), (1.0 if float(spec.get("direction", 1)) >= 0 else -1.0)
    w = v / r

    def fn(t):
        a = a0 + sgn * w * t
        return np.array([leak[0] + r * math.cos(a), leak[1] + r * math.sin(a), zc])
    return Segment(laps * 2.0 * math.pi * r / v, fn)


def _updown(spec: dict, leak: np.ndarray, box: Box, notes: list) -> Segment:
    d, b = float(spec["distance_m"]), math.radians(float(spec["bearing_deg"]))
    xy = leak[:2] + d * np.array([math.cos(b), math.sin(b)])
    if not (np.all(xy >= box.lo[:2]) and np.all(xy <= box.hi[:2])):
        raise ConfigError(f"{spec['name']}: the point {np.round(xy, 2).tolist()} at {d} m from the leak is outside "
                          "the bounds; change bearing_deg or distance_m")
    a = float(spec["amplitude_m"])
    lo, hi = float(leak[2]) - a, float(leak[2]) + a
    if lo < box.lo[2]:
        shift = box.lo[2] - lo
        notes.append(f"{spec['name']}: z range [{lo:.2f}, {hi:.2f}] m reaches below the floor bound "
                     f"{box.lo[2]:.2f} m; shifted up by {shift:.2f} m (amplitude kept)")
        lo, hi = lo + shift, hi + shift
    if hi > box.hi[2]:
        notes.append(f"{spec['name']}: top {hi:.2f} m above the bound {box.hi[2]:.2f} m; amplitude reduced")
        hi = float(box.hi[2])
    span, v, n = hi - lo, float(spec["speed_mps"]), float(spec["cycles"])
    period = 2.0 * span / v

    def fn(t):
        return np.array([xy[0], xy[1], lo + span * 0.5 * (1.0 - math.cos(2.0 * math.pi * t / period))])
    return Segment(n * period, fn)


def _transit(p0, p1, v: float) -> Segment:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    dur = float(np.linalg.norm(p1 - p0)) / v
    if dur <= 1e-9:
        return None
    return Segment(dur, lambda t: p0 + (p1 - p0) * (t / dur))


BUILDERS = {"circle": _circle, "updown": _updown}


def build(cfg: dict, name_or_path: str, leak) -> Trajectory:
    spec = load_spec(name_or_path)
    leak = np.asarray(leak, float)
    notes: list = []
    if spec["kind"] == "sequence":
        segs, params = [], {"segments": []}
        for sub in spec["segments"]:
            t = build(cfg, sub, leak)
            notes += t.notes
            params["segments"].append(t.params)
            if segs:
                tr = _transit(segs[-1].at(segs[-1].duration), t.segments[0].at(0.0), float(spec["transit_speed_mps"]))
                if tr is not None:
                    segs.append(tr)
            segs += t.segments
        return Trajectory(spec["name"], leak, segs, notes, {"kind": "sequence", **params})
    if spec["kind"] not in BUILDERS:
        raise ConfigError(f"trajectory kind must be circle | updown | sequence, got {spec['kind']!r}")
    box = body_box(cfg, float(spec.get("wall_margin_m", 1.0)), float(spec.get("floor_margin_m", 0.3)))
    seg = BUILDERS[spec["kind"]](spec, leak, box, notes)
    traj = Trajectory(spec["name"], leak, [seg], notes, dict(spec))
    for t in np.linspace(0.0, seg.duration, 200):
        if not box.contains(seg.at(t)):
            raise ConfigError(f"{spec['name']}: leaves the bounds at t = {t:.1f} s (a bug: the builder should have fitted it)")
    return traj
