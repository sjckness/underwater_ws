"""One sonar ping -> landmarks on the bottom, in the body's horizontal plane, with covariances.

The chain, and why each step is there:

1. **Suppress the speckle.** The model's own validation measured the static speckle correlation from
   ping to ping at 0.18-0.22 while moving (0.99 fixed): fully developed Rayleigh speckle, contrast
   0.984. The speckle realisation does not survive a baseline and cannot be registered across one.
   What survives is resolved structure - and the same validation attributes the excess background
   contrast at +-25 deg (2.13 against a Rayleigh 0.98) entirely to "the relief of the modelled bottom:
   resolved ripple and rock structure". So the intensity is smoothed over the footprint of a point
   target before anything looks at it.
2. **CFAR.** Compare that to a wide local background, which removes the spreading loss, the TVG and
   the beam pattern at once without trusting any of them.
3. **The first return on each beam.** Nothing can be closer along a beam than the nearest real
   surface, so the leading edge rejects *every* multipath ghost - they always arrive later - and the
   late sidelobe arcs, with no model of either. A measured surface ghost in this simulator is +1.22 dB
   *brighter* than its direct return, so rejecting by level alone would not work.
4. **The envelope, and its dips.** On a flat bottom the first return sits at a near-constant slant
   range: the locus is a circle centred under the vehicle, set by the steepest elevation that still
   has gain. Measured on a real ping: median 2.744 m at 1.195 m altitude, an implied depression of
   25.8 deg for a sonar pitched 15 deg down. That arc is an artefact of the beam geometry, it carries
   no bearing-dependent information, and feeding it to a registration would be worse than useless -
   two vehicle-centred circles align at zero translation, which is a strong *wrong* constraint. What
   carries information is where the first return comes back **early**: an object standing proud of the
   bottom. Those dips are the landmarks, and they are what the roadmap means by "landmarks from rocks,
   pipeline and flanges".
5. **Project onto the bottom.** The range is measured and the scatterer is taken to be on the bottom,
   so the depression angle is determined, ``phi = asin(altitude / r)``, and the horizontal distance is
   ``sqrt(r^2 - altitude^2)``. The altitude comes from the DVL. No elevation assumption is needed at
   all, which is the one place this is strictly better than the literature's "assume elevation 0".

What it costs: an object standing ``h`` proud of the bottom lands too close, by
``d - sqrt(d^2 - h*(2*altitude - h))`` - 7 cm for a 0.16 m rock at 2.5 m and 1.19 m altitude. That is
10-30x the range precision, so it is not negligible; but it is a function of the viewing geometry
only, so on a constant-radius, constant-altitude path it is constant per landmark and is absorbed by
that landmark's estimated position. It leaks into the *pose* only when the range to a landmark changes
- which is also when it becomes observable. ``landmark_height_m`` carries it as an explicit state for
the back end rather than hiding it here.

Frames. Bearing is positive to starboard, as everywhere else in this project; the output points are in
the body's FLU horizontal plane (x forward, y port), so a landmark at bearing ``t`` and horizontal
distance ``d`` is at ``(d cos t, -d sin t)``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from physics.sonar_stats import local_mean


@dataclass(frozen=True)
class Params:
    """Front-end knobs. Every one of them is UNVERIFIED: they are detection thresholds, not physics."""
    range_gate_m: float = 0.30          # skip the blind zone and the ring-down behind it
    smooth_bins: int = 7                # speckle suppression, matched to a point target's footprint
    smooth_beams: int = 5               #   (2 range bins deep, ~4 beams wide)
    background_bins: int = 201          # the CFAR background window
    background_beams: int = 21
    detect_db: float = 8.0              # a return is this far above its local background
    min_run_bins: int = 3               # ... for this many consecutive range bins
    envelope_beams: int = 61            # the flat-bottom envelope: a median over this many bearings
    min_dip_m: float = 0.02             # a landmark stands at least this far proud, in slant range
    max_object_height_m: float = 0.35   # the tallest thing on this bottom; sets the dip gate, see below
    min_beams: int = 2                  # a landmark must be seen on this many adjacent beams
    max_landmarks: int = 64
    sigma_range_m: float = 0.004        # one range bin
    sigma_bearing_deg: float = 0.25     # the beam step; a centroid over N beams divides it by sqrt(N)


@dataclass(frozen=True)
class Cloud:
    """The landmarks of one ping, in the body's horizontal plane (FLU, metres)."""
    xy: np.ndarray                      # (n, 2)
    cov: np.ndarray                     # (n, 2, 2)
    slant_m: np.ndarray                 # (n,) the measured range
    bearing_deg: np.ndarray             # (n,)
    dip_m: np.ndarray                   # (n,) how far proud of the bottom envelope, in slant range
    strength_db: np.ndarray             # (n,) peak level above the local background
    altitude_m: float
    envelope_m: float                   # the median flat-bottom range, a diagnostic

    def __len__(self) -> int:
        return len(self.xy)


# ------------------------------------------------------------------------------------- the steps

def detection_image(db: np.ndarray, params: Params) -> np.ndarray:
    """dB above the local background, after the speckle is averaged down (pure).

    ``db`` is the polar image in dB (rows: range bins from range 0; columns: beams), NaN in the blind
    zone, as ``physics.sonar.Ping.db`` and the recorder's ``polar_db`` both are.
    """
    intensity = np.power(10.0, np.asarray(db, float) / 10.0)
    intensity[~np.isfinite(intensity)] = 0.0
    smooth = local_mean(intensity, (params.smooth_bins, params.smooth_beams))
    background = local_mean(smooth, (params.background_bins, params.background_beams))
    return 10.0 * np.log10(np.maximum(smooth, 1e-30) / np.maximum(background, 1e-30))


def first_return(detect: np.ndarray, range_m: np.ndarray, params: Params):
    """Slant range of the leading edge of the first strong run on each beam (pure).

    Returns ``(r_first, strength)``, both (n_beams,), NaN where a beam never rises above the
    threshold. This is the step that rejects multipath: a ghost is always farther along its bearing
    than the return that produced it.
    """
    rng = np.asarray(range_m, float)
    gate = int(np.searchsorted(rng, params.range_gate_m))
    above = (detect[gate:, :] > params.detect_db)
    n = params.min_run_bins
    if n > 1:                                        # a run of n consecutive bins
        run = above[: len(above) - n + 1, :].copy()
        for k in range(1, n):
            run &= above[k: len(above) - n + 1 + k, :]
    else:
        run = above
    hit = run.any(axis=0)
    idx = np.where(hit, run.argmax(axis=0), 0) + gate
    r_first = np.where(hit, rng[np.clip(idx, 0, len(rng) - 1)], np.nan)
    rows = np.clip(idx, 0, detect.shape[0] - 1)
    strength = np.where(hit, detect[rows, np.arange(detect.shape[1])], np.nan)
    return r_first, strength


def envelope(r_first: np.ndarray, beams: int) -> np.ndarray:
    """The flat-bottom envelope: a running median of the first-return range over bearing (pure).

    A median, not a mean, precisely so that the dips this function exists to reveal do not drag it.
    """
    r = np.asarray(r_first, float)
    half = max(int(beams) // 2, 1)
    padded = np.pad(r, (half, half), mode="edge")
    out = np.full(len(r), np.nan)
    for i in range(len(r)):
        window = padded[i: i + 2 * half + 1]
        if np.isfinite(window).any():
            out[i] = np.nanmedian(window)
    return out


def max_dip_m(envelope_m: float, altitude_m: float, object_height_m: float) -> float:
    """The largest slant-range dip an object of that height can physically produce (pure).

    This gate is geometry, not taste, and it matters more than any other threshold here. An object of
    height ``h`` at the envelope's horizontal distance shortens the slant range only from
    ``sqrt(d^2 + alt^2)`` to ``sqrt(d^2 + (alt - h)^2)`` - **0.12 m for a 0.35 m object at 2.5 m and
    1.19 m altitude**, not metres. Without this bound the dip test also admits bottom returns that
    arrive early because the *vertical skirt* caught a brighter patch at a steeper depression (the
    pattern's first null is at +-42.6 deg, far outside the quoted 20 deg aperture), and those are not
    objects at all. Measured on a real ping, the ungated version produced dips of 1.0-1.4 m where the
    tallest rock in the scene is 0.16 m.
    """
    h = float(altitude_m)
    d2 = max(float(envelope_m) ** 2 - h * h, 0.0)
    ho = min(float(object_height_m), h * 0.95)
    return math.sqrt(d2 + h * h) - math.sqrt(d2 + (h - ho) ** 2)


def dips(r_first, env, bearing_deg, strength, altitude_m: float, params: Params):
    """Group the beams whose first return comes back early into landmarks (pure).

    Returns ``(slant_m, bearing_deg, dip_m, strength_db, n_beams)``, one entry per landmark: the
    strength-weighted centroid over the adjacent beams that share the dip.
    """
    r = np.asarray(r_first, float)
    e = np.asarray(env, float)
    b = np.asarray(bearing_deg, float)
    s = np.asarray(strength, float)
    dip = e - r
    env_med = float(np.nanmedian(e)) if np.isfinite(e).any() else float("nan")
    cap = max_dip_m(env_med, altitude_m, params.max_object_height_m) if np.isfinite(env_med) else 0.0
    on = np.isfinite(dip) & (dip > params.min_dip_m) & (dip < cap)
    out = []
    i = 0
    while i < len(on):
        if not on[i]:
            i += 1
            continue
        k = i
        while k < len(on) and on[k]:
            k += 1
        sel = slice(i, k)
        if k - i >= params.min_beams:
            w = np.clip(s[sel], 1e-6, None)
            out.append((float(np.average(r[sel], weights=w)), float(np.average(b[sel], weights=w)),
                        float(dip[sel].max()), float(s[sel].max()), k - i))
        i = k
    if not out:
        z = np.zeros(0)
        return z, z, z, z, np.zeros(0, int)
    out.sort(key=lambda t: -t[3])                    # strongest first
    out = out[: params.max_landmarks]
    a = np.array(out, float)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4].astype(int)


def project(slant_m, bearing_deg, altitude_m: float, n_beams, params: Params):
    """Slant range + bearing + the DVL's altitude -> a point in the body's horizontal plane (pure).

    The scatterer is taken to be on the bottom, so the depression angle is determined and
    ``d = sqrt(r^2 - altitude^2)``. Returns ``(xy (n, 2), cov (n, 2, 2))``.

    The covariance is deliberately anisotropic: the sonar measures range to about one bin (4 mm) and
    cross-range to a beamwidth (43 mm at 2.5 m, 120 mm at 7 m), a ratio of 10-30x, and an isotropic
    point would throw most of that away. The radial term also picks up ``dd/dr = r/d`` from the
    projection, so a landmark seen at a steep grazing angle is horizontally less certain than its
    range alone suggests.
    """
    r = np.atleast_1d(np.asarray(slant_m, float))
    t = np.radians(np.atleast_1d(np.asarray(bearing_deg, float)))
    h = float(altitude_m)
    d = np.sqrt(np.maximum(r * r - h * h, 0.0))
    xy = np.stack([d * np.cos(t), -d * np.sin(t)], 1)
    sigma_d = params.sigma_range_m * np.where(d > 1e-6, r / np.maximum(d, 1e-6), 1.0)
    sigma_c = d * math.radians(params.sigma_bearing_deg) / np.sqrt(np.maximum(np.asarray(n_beams, float), 1.0))
    cov = np.zeros((len(r), 2, 2))
    for i in range(len(r)):
        rot = np.array([[math.cos(t[i]), math.sin(t[i])], [-math.sin(t[i]), -math.cos(t[i])]])
        cov[i] = rot @ np.diag([sigma_d[i] ** 2, sigma_c[i] ** 2]) @ rot.T
    return xy, cov


# ------------------------------------------------------------------------------------ the whole ping

def extract(db, range_m, bearing_deg, altitude_m: float, params: Params | None = None) -> Cloud:
    """One ping -> its landmarks. The only entry point a caller needs."""
    p = params or Params()
    detect = detection_image(db, p)
    r_first, strength = first_return(detect, range_m, p)
    env = envelope(r_first, p.envelope_beams)
    slant, bearing, dip, level, n_beams = dips(r_first, env, bearing_deg, strength, altitude_m, p)
    xy, cov = project(slant, bearing, altitude_m, n_beams, p)
    return Cloud(xy=xy, cov=cov, slant_m=slant, bearing_deg=bearing, dip_m=dip, strength_db=level,
                 altitude_m=float(altitude_m),
                 envelope_m=float(np.nanmedian(env)) if np.isfinite(env).any() else float("nan"))


def height_bias_m(horizontal_m, object_height_m: float, altitude_m: float) -> np.ndarray:
    """How much too close an object standing ``object_height_m`` proud of the bottom is placed (pure).

    Positive means the estimate is short. This is the systematic cost of the bottom projection, and it
    exists so a test can pin it and ``docs/slam.md`` can quote it rather than discover it later.
    """
    d = np.asarray(horizontal_m, float)
    h, ho = float(altitude_m), float(object_height_m)
    inner = d * d - ho * (2.0 * h - ho)
    return d - np.sqrt(np.maximum(inner, 0.0))
