"""Statistics for comparing sonar images, real or simulated (tools/compare_real.py). numpy only.

Inputs are polar images in dB (rows: range bins from range 0, columns: beams; NaN = no data) with their range and
bearing axes.
"""
from __future__ import annotations

import math

import numpy as np


def k_shape(intensity: np.ndarray) -> float:
    """K-distribution shape nu from the normalised second moment of the intensity, <I^2>/<I>^2 = 2 (1 + 1/nu);
    inf for Rayleigh speckle (moment 2) or lighter tails."""
    i = np.asarray(intensity, float).ravel()
    m2 = np.mean(i ** 2) / np.mean(i) ** 2
    return math.inf if m2 <= 2.0 else 2.0 / (m2 - 2.0)


def background(db, range_m, bearing_deg, r_window, b_window, smooth: int = 25) -> dict:
    """Speckle statistics of a background region: intensity normalised by its local mean (smooth x smooth pixels,
    which removes spreading, TVG and beam-pattern trends), then the Rayleigh check (exponential intensity,
    contrast 1) and the K shape."""
    rows = (range_m >= r_window[0]) & (range_m <= r_window[1])
    cols = (bearing_deg >= b_window[0]) & (bearing_deg <= b_window[1])
    inten = 10 ** (np.asarray(db, float)[rows][:, cols] / 10)
    k = np.ones(smooth) / smooth
    mean = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 0, inten)
    mean = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 1, mean)
    h = smooth // 2
    x = (inten / mean)[h:-h, h:-h].ravel()
    x = x[np.isfinite(x)]
    x /= x.mean()
    edges = np.linspace(0, 8, 81)
    hist, _ = np.histogram(x, edges, density=True)
    xs = np.sort(x)
    ks = float(np.max(np.abs(np.arange(1, len(xs) + 1) / len(xs) - (1 - np.exp(-xs)))))   # vs exponential
    return {"pixels": int(len(x)), "contrast": float(x.std()), "k_shape": k_shape(x), "ks_vs_rayleigh": ks,
            "hist_edges": edges.tolist(), "hist": hist.tolist()}


def lobe(bearing_deg, row_db, i) -> tuple[float, float]:
    """-3 dB width [deg] of the lobe peaking at index i, and the highest sidelobe beyond its first nulls [dB re peak]."""
    rel = np.asarray(row_db, float) - row_db[i]
    lo = hi = i
    while lo > 0 and rel[lo - 1] > -3:
        lo -= 1
    while hi < len(rel) - 1 and rel[hi + 1] > -3:
        hi += 1
    left = np.interp(-3, [rel[lo - 1], rel[lo]], [bearing_deg[lo - 1], bearing_deg[lo]]) if lo > 0 else bearing_deg[0]
    right = (np.interp(-3, [rel[hi + 1], rel[hi]], [bearing_deg[hi + 1], bearing_deg[hi]])
             if hi < len(rel) - 1 else bearing_deg[-1])
    a, b = lo, hi
    while a > 0 and rel[a - 1] < rel[a]:
        a -= 1
    while b < len(rel) - 1 and rel[b + 1] < rel[b]:
        b += 1
    side = max(rel[:a].max() if a > 0 else -np.inf, rel[b + 1:].max() if b < len(rel) - 1 else -np.inf)
    return float(right - left), float(side)


def bright_target(db, range_m, bearing_deg, r_window=(0.3, np.inf), b_window=(-180.0, 180.0)) -> dict:
    """The brightest pixel inside the window: its range, bearing, the -3 dB width and the sidelobe level along its
    range row. Meaningful for a point target (e.g. a calibration sphere), not for an extended one such as a pipe."""
    d = np.where(np.isfinite(db), db, -np.inf).copy()
    d[(range_m < r_window[0]) | (range_m > r_window[1])] = -np.inf
    d[:, (bearing_deg < b_window[0]) | (bearing_deg > b_window[1])] = -np.inf
    j, i = np.unravel_index(np.argmax(d), d.shape)
    width, side = lobe(bearing_deg, d[j], i)
    return {"range_m": float(range_m[j]), "bearing_deg": float(bearing_deg[i]), "level_db": float(d[j, i]),
            "width_deg": width, "sidelobe_db": side}


def _reference_db(prof, range_m, ref_window) -> float:
    """Median of ``prof`` over ``ref_window`` [m], the level a normalised profile is quoted against. NaN when the
    window holds no data, which is a refusal to normalise rather than a silent offset of 0."""
    sel = np.isfinite(prof) & (range_m >= ref_window[0]) & (range_m <= ref_window[1])
    return float(np.median(prof[sel])) if sel.any() else math.nan


def _normalised(prof, range_m, normalise, ref_window) -> dict:
    """The extra keys a profile carries when it is compared as a *shape* rather than as a level.

    ``normalise=None`` (default) adds nothing, so every existing caller sees the absolute dB it always saw.
    ``normalise="median"`` subtracts the median over ``ref_window``: a simulated profile in physical dB re 1 uPa
    and a real recording in whatever units its 8-bit display left behind then lie on the same axis, and only
    their shape is being compared. The reference itself is reported, so the subtraction stays auditable.
    """
    if normalise is None:
        return {}
    if normalise != "median":
        raise ValueError(f"normalise must be None or 'median', not {normalise!r}")
    ref = _reference_db(prof, range_m, ref_window)
    norm = prof - ref
    return {"normalise": normalise, "norm_ref_window_m": list(ref_window),
            "norm_ref_db": None if not np.isfinite(ref) else ref,
            "norm_db": [None if not np.isfinite(v) else float(v) for v in norm]}


def near_range(db, range_m, r_max: float = 0.5, normalise: str | None = None, ref_window=(0.3, 0.5)) -> dict:
    """Blind zone (first range with data) and the beam-averaged profile near the sonar, with the ring-down decay
    slope fitted over the first 0.1 m of data.

    ``normalise="median"`` additionally quotes the profile against its median over ``ref_window``, which should
    sit *beyond* the ring-down so that the reference is local background and not the ring-down itself. The slope
    and the blind zone are unaffected: a slope is already invariant to an additive dB offset.
    """
    prof = np.array([np.nanmean(r) if np.isfinite(r).any() else np.nan for r in db])
    has = np.isfinite(prof)
    blind = float(range_m[np.argmax(has)]) if has.any() else math.nan
    sel = has & (range_m >= blind) & (range_m <= blind + 0.1)
    slope = float(np.polyfit(range_m[sel], prof[sel], 1)[0]) if sel.sum() > 3 else math.nan
    near = range_m <= r_max
    out = {"blind_zone_m": blind, "ringdown_slope_db_per_m": slope, "range_m": range_m[near].tolist(),
           "mean_db": [None if not np.isfinite(v) else float(v) for v in prof[near]]}
    extra = _normalised(prof, range_m, normalise, ref_window)
    if extra:
        extra["norm_db"] = [extra["norm_db"][i] for i in np.flatnonzero(near)]
        out.update(extra)
    return out


def tvg_profile(db, range_m, r_window, normalise: str | None = None, ref_window=None) -> dict:
    """Median level vs range, and the fit a + k log10(R) + b R over r_window (k, b: what a TVG would have to undo).

    ``k`` and ``b`` are already invariant to an additive dB offset, so they transfer between a simulation in
    physical dB and a real recording in sensor units; ``a_db`` does not and must not be compared.

    But do not compare ``k`` and ``b`` between two recordings either. log10(R) and R are collinear over any
    window a sonar covers, so the two constants trade off almost freely: 60 UATD pings give k = 59.7 dB/decade
    with a per-frame spread of +-106. They are kept because they summarise one profile compactly; for sim
    against real use ``profile_distance`` on the curve.
    ``normalise="median"`` adds the profile quoted against its median over ``ref_window`` (default: ``r_window``),
    which is what to plot when sim and real are put on one axis.
    """
    prof = np.array([np.nanmedian(r) if np.isfinite(r).any() else np.nan for r in db])
    sel = np.isfinite(prof) & (range_m >= r_window[0]) & (range_m <= r_window[1])
    a = np.stack([np.ones(sel.sum()), np.log10(range_m[sel]), range_m[sel]], 1)
    coef = np.linalg.lstsq(a, prof[sel], rcond=None)[0]
    out = {"fit_window_m": list(r_window), "k_db_per_decade": float(coef[1]), "b_db_per_m": float(coef[2]),
           "a_db": float(coef[0]), "range_m": range_m.tolist(),
           "median_db": [None if not np.isfinite(v) else float(v) for v in prof]}
    out.update(_normalised(prof, range_m, normalise, tuple(ref_window or r_window)))
    return out


def local_mean(intensity: np.ndarray, size=(7, 5)) -> np.ndarray:
    """Box mean over size = (range bins, beams), edges by reflection."""
    out = np.asarray(intensity, float)
    for axis, n in enumerate(size):
        pad = [(0, 0), (0, 0)]
        pad[axis] = (n // 2, n - 1 - n // 2)
        c = np.cumsum(np.pad(out, pad, mode="reflect"), axis=axis)
        c = np.insert(c, 0, 0.0, axis=axis)
        out = (np.take(c, range(n, c.shape[axis]), axis=axis) - np.take(c, range(0, c.shape[axis] - n), axis=axis)) / n
    return out


def speckle_correlation(a: np.ndarray, b: np.ndarray, mask: np.ndarray, size=(7, 5)) -> float:
    """Correlation of two pings' speckle in the pixels of ``mask``: each intensity image divided by its own local mean
    (``size``), so that the slowly varying envelope (spreading, beam pattern, the plume's shape) does not count; only
    pixels whose whole window lies in the mask (at a region's edge the window straddles the edge, and the ratio there
    follows the edge, not the speckle). 1: the same speckle (a static scene); 0: renewed speckle (scatterers moved
    by more than a wavelength)."""
    na = np.asarray(a, float) / local_mean(a, size)
    nb = np.asarray(b, float) / local_mean(b, size)
    inner = local_mean(np.asarray(mask, float), size) > 1 - 1e-9
    return float(np.corrcoef(na[inner], nb[inner])[0, 1])


def far_sidelobe_floor(db, range_m, bearing_deg, r_peak: float, b_peak: float, beamwidth_deg: float,
                       n_bw: float = 5.0, r_rows: int = 3, bg_offset_m=(0.5, 1.5)) -> dict:
    """Level of a compact target's *far* angular skirt, relative to its peak: an **upper bound** on the receive
    array's sidelobe level, never a measurement of it.

    Read the range rows within ``r_rows`` bins of ``r_peak`` (the target), take the beams further than
    ``n_bw`` beamwidths from ``b_peak`` (past the main lobe and its first nulls), and subtract the background
    estimated from the *same* beams at ranges ``bg_offset_m`` away on either side, where the target is not. What
    is left is energy that the target put at a large angle.

    It bounds the array sidelobe from above because everything else that puts energy there adds to it: the
    target's own angular extent if it is not a point (a 0.4 m sphere at 8 m spans ~3 deg, several beamwidths),
    volume reverberation, multipath, and the maker's own TVG and AGC in a recorded 8-bit image. So a measured
    -20 dB says "the array sidelobe is at most -20 dB", and a value at or below the background says only that
    it is unmeasurable here.

    ``above_background`` false means exactly that: the skirt did not rise above the local background, so no
    bound tighter than the background itself can be read, and ``floor_db`` is None.
    """
    db = np.asarray(db, float)
    range_m, bearing_deg = np.asarray(range_m, float), np.asarray(bearing_deg, float)
    j = int(np.argmin(np.abs(range_m - r_peak)))
    lo, hi = max(j - r_rows, 0), min(j + r_rows + 1, db.shape[0])
    band = db[lo:hi]

    main = np.abs(bearing_deg - b_peak) <= beamwidth_deg
    if not main.any() or not np.isfinite(band[:, main]).any():
        raise ValueError(f"no finite data within one beamwidth of bearing {b_peak}")
    peak_db = float(np.nanmax(band[:, main]))

    far = np.abs(bearing_deg - b_peak) > n_bw * beamwidth_deg
    if not far.any():
        raise ValueError(f"no beams beyond {n_bw} beamwidths of bearing {b_peak} in this fan")

    def mean_intensity(rows) -> float:
        v = 10 ** (rows[:, far] / 10)
        v = v[np.isfinite(v)]
        return float(v.mean()) if v.size else math.nan

    bg_rows = [db[a:b] for a, b in
               ((int(np.argmin(np.abs(range_m - (r_peak - bg_offset_m[1])))),
                 int(np.argmin(np.abs(range_m - (r_peak - bg_offset_m[0]))))),
                (int(np.argmin(np.abs(range_m - (r_peak + bg_offset_m[0])))),
                 int(np.argmin(np.abs(range_m - (r_peak + bg_offset_m[1]))))))
               if b > a]
    skirt = mean_intensity(band)
    bg = mean_intensity(np.concatenate(bg_rows)) if bg_rows else math.nan

    excess = skirt - bg
    above = bool(np.isfinite(excess) and excess > 0)
    return {"peak_db": peak_db, "range_m": float(range_m[j]), "bearing_deg": float(b_peak),
            "n_beamwidths": float(n_bw), "beams": int(far.sum()),
            "skirt_db": None if not np.isfinite(skirt) else float(10 * math.log10(skirt)),
            "background_db": None if not np.isfinite(bg) else float(10 * math.log10(bg)),
            "above_background": above,
            "floor_db": float(10 * math.log10(excess) - peak_db) if above else None}


def profile_distance(range_a, db_a, range_b, db_b, window, normalise: bool = True) -> dict:
    """How far apart two level-vs-range profiles are in shape: rms and max difference [dB] over ``window``.

    This exists because the fit constants of ``tvg_profile`` cannot carry the comparison. Over any window a
    real sonar actually covers, log10(R) and R are collinear -- correlation 0.992 over 3-8 m, 0.997 over
    1.8-3.4 m, condition number ~400 -- so ``k`` and ``b`` trade off against each other almost freely while
    the fitted curve barely moves. Measured on 60 UATD pings the slope comes out 59.7 dB/decade with a
    per-frame spread of +-106: the number is noise, and two such numbers cannot be compared. The curve can.

    Both profiles are median-normalised over the window first (``normalise``), so a simulation in physical dB
    and a recording in whatever units its display left behind are compared on shape alone. ``b`` is sampled
    onto ``a``'s ranges inside the window; points either profile does not cover do not count, and
    ``covered`` says how many did.
    """
    ra, va = np.asarray(range_a, float), np.asarray(db_a, float)
    rb, vb = np.asarray(range_b, float), np.asarray(db_b, float)
    sel = (ra >= window[0]) & (ra <= window[1]) & np.isfinite(va)
    okb = np.isfinite(vb) & (rb >= min(rb.min(), window[0]))
    if sel.sum() < 2 or okb.sum() < 2:
        return {"rms_db": math.nan, "max_db": math.nan, "covered": 0, "window_m": list(window)}
    a = va[sel]
    b = np.interp(ra[sel], rb[okb], vb[okb], left=math.nan, right=math.nan)
    both = np.isfinite(a) & np.isfinite(b)
    if both.sum() < 2:
        return {"rms_db": math.nan, "max_db": math.nan, "covered": 0, "window_m": list(window)}
    a, b = a[both], b[both]
    if normalise:
        a = a - np.median(a)
        b = b - np.median(b)
    d = a - b
    return {"rms_db": float(np.sqrt(np.mean(d ** 2))), "max_db": float(np.max(np.abs(d))),
            "covered": int(both.sum()), "window_m": list(window)}
