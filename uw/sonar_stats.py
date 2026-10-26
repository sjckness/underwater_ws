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


def near_range(db, range_m, r_max: float = 0.5) -> dict:
    """Blind zone (first range with data) and the beam-averaged profile near the sonar, with the ring-down decay
    slope fitted over the first 0.1 m of data."""
    prof = np.array([np.nanmean(r) if np.isfinite(r).any() else np.nan for r in db])
    has = np.isfinite(prof)
    blind = float(range_m[np.argmax(has)]) if has.any() else math.nan
    sel = has & (range_m >= blind) & (range_m <= blind + 0.1)
    slope = float(np.polyfit(range_m[sel], prof[sel], 1)[0]) if sel.sum() > 3 else math.nan
    near = range_m <= r_max
    return {"blind_zone_m": blind, "ringdown_slope_db_per_m": slope, "range_m": range_m[near].tolist(),
            "mean_db": [None if not np.isfinite(v) else float(v) for v in prof[near]]}


def tvg_profile(db, range_m, r_window) -> dict:
    """Median level vs range, and the fit a + k log10(R) + b R over r_window (k, b: what a TVG would have to undo)."""
    prof = np.array([np.nanmedian(r) if np.isfinite(r).any() else np.nan for r in db])
    sel = np.isfinite(prof) & (range_m >= r_window[0]) & (range_m <= r_window[1])
    a = np.stack([np.ones(sel.sum()), np.log10(range_m[sel]), range_m[sel]], 1)
    coef = np.linalg.lstsq(a, prof[sel], rcond=None)[0]
    return {"fit_window_m": list(r_window), "k_db_per_decade": float(coef[1]), "b_db_per_m": float(coef[2]),
            "a_db": float(coef[0]), "range_m": range_m.tolist(),
            "median_db": [None if not np.isfinite(v) else float(v) for v in prof]}


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
