"""Compare sonar recordings, real or simulated, on what M2 checks.

    python -m uw py tools/compare_real.py DATASET [DATASET ...] [--frames 50] [--out output/compare_real]

A DATASET is one of:
  uw    a directory written by `python -m uw sonar-only` (sonar/<seq>.npz: polar_db, range_m, bearing_deg),
        absolute dB re 1 uPa;
  uatd  the UATD multibeam FLS dataset (Xie, Yang & Qiu 2022, Tritech Gemini 1200ik): images/<n>.bmp, polar,
        rows = range bins from range 0, columns = beams, plus annotations/<n>.xml giving range, azimuth,
        elevation, sound speed and frequency per image;
  fan   a directory of rendered Cartesian fan images (DeeperSense, Zenodo 7728089 / 10220989): the sector
        geometry is fitted from the mean of the frames and the image is resampled back to polar.
The loaders all yield (polar dB, row 0 at range 0, NaN = no data; range axis [m]; bearing axis [deg]).

Real recordings are 8-bit display images, not calibrated pressure: only the *shape* of each statistic transfers,
never the absolute level. An 8-bit pixel v is taken to be an amplitude raised to --pixel-exponent p (v ~ A**p),
so intensity dB = (20/p) log10 v; p = 1 is a linear amplitude image, p = 0.5 an Oculus-style gamma of 127/255.
--fit-pixel-law solves for the p that makes the background a Rayleigh speckle (contrast 1) instead, and the KS
distance then says whether that p is self-consistent.
Per dataset, averaged over the frames: background speckle (Rayleigh check, K shape), the brightest target's width
and sidelobe level, blind zone and ring-down near the sonar, the level-vs-range (TVG) profile. JSON + PNG.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))
from uw import config, sonar_stats  # noqa: E402


IMAGE_SUFFIXES = (".bmp", ".png", ".jpg", ".jpeg")


def load_uw(path: Path, frames: int, exponent: float):
    for f in sorted((path / "sonar").glob("*.npz"))[:frames]:
        with np.load(f) as z:
            yield z["polar_db"].astype(np.float32), z["range_m"], z["bearing_deg"]


def pixels_to_db(v, exponent: float):
    """8-bit pixels to intensity dB, re an arbitrary reference. v ~ A**exponent for an amplitude A, so the
    intensity is v**(2/exponent); zeros are floored at half a least significant bit, not dropped, because a real
    image is zero wherever the display clipped and that is data, not absence of data."""
    return (20.0 / exponent) * np.log10(np.maximum(np.asarray(v, np.float32), 0.5))


def colormap_lut(name: str) -> np.ndarray:
    """A 32x32x32 RGB cube holding, for each coarse colour, the nearest index of the 256-entry colormap. Fan
    images that were rendered through a colormap and then resampled have colours off the curve, so this is a
    nearest-neighbour inverse, not an exact one."""
    import matplotlib
    lut = np.asarray(matplotlib.colormaps[name](np.linspace(0, 1, 256)))[:, :3] * 255
    grid = (np.indices((32, 32, 32)).reshape(3, -1).T + 0.5) * 8
    idx = np.argmin(((grid[:, None, :] - lut[None, :, :]) ** 2).sum(2), 1)
    return idx.reshape(32, 32, 32).astype(np.float32)


def read_gray(path: Path, cube=None) -> np.ndarray:
    from PIL import Image
    im = Image.open(path)
    if cube is None:
        return np.asarray(im.convert("L"), np.float32)
    rgb = np.asarray(im.convert("RGB"), np.uint8) >> 3
    return cube[rgb[..., 0], rgb[..., 1], rgb[..., 2]]


def load_uatd(path: Path, frames: int, exponent: float):
    """UATD: a polar image per ping with its own geometry in the sibling XML."""
    import xml.etree.ElementTree as ET
    for f in sorted((path / "images").iterdir())[:frames]:
        if f.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        sonar = ET.parse(path / "annotations" / (f.stem + ".xml")).getroot().find("sonar")
        r_max, fov = float(sonar.findtext("range")), float(sonar.findtext("azimuth"))
        img = read_gray(f)
        n_r, n_b = img.shape
        yield (pixels_to_db(img, exponent),
               (np.arange(n_r) + 0.5) * r_max / n_r,
               (np.arange(n_b) + 0.5) * fov / n_b - fov / 2)


def fan_geometry(mean: np.ndarray, floor: float = 1.0) -> dict:
    """Apex, outer radius and half angle of a rendered sector, fitted to the support of the mean of the frames
    (a single frame's support is eaten into by the speckle). The two straight edges are fitted where they are
    straight, below the outer arc; their intersection is the apex and their slope dx/dy is tan(half angle)."""
    support = mean > floor
    rows = np.arange(mean.shape[0])
    left = np.full(mean.shape[0], np.nan)
    right = np.full(mean.shape[0], np.nan)
    for y in rows:
        on = np.flatnonzero(support[y])
        if on.size > 30:
            left[y], right[y] = on[0], on[-1]
    have = np.isfinite(left)
    if have.sum() < 20:
        raise SystemExit("fan: the support of the mean image is too small to fit a sector")
    # The straight edges are the rows where the width shrinks again as y grows: below the widest row.
    widest = int(np.nanargmax(right - left))
    edge = have & (rows > widest)
    if edge.sum() < 10:
        raise SystemExit("fan: no straight edge below the widest row; is this a sector image?")
    y = rows[edge]
    sl = np.polyfit(y, left[edge], 1)[0]
    sr = np.polyfit(y, right[edge], 1)[0]
    slope = 0.5 * (sl - sr)                       # left edge runs +x with +y, right edge -x
    x_l, x_r = np.polyval(np.polyfit(y, left[edge], 1), y[0]), np.polyval(np.polyfit(y, right[edge], 1), y[0])
    y0 = y[0] + 0.5 * (x_r - x_l) / slope
    x0 = 0.5 * (x_l + x_r)
    top = rows[have][0]
    return {"apex_x": float(x0), "apex_y": float(y0), "r_max_px": float(y0 - top),
            "half_angle_deg": float(np.degrees(np.arctan(slope)))}


def fan_to_polar(img: np.ndarray, geom: dict, n_r: int, n_b: int) -> np.ndarray:
    """Bilinear resample of a Cartesian sector back onto a (range, bearing) grid. Outside the image: NaN.
    ``x_scale`` carries an anisotropic rescale of the rendering (a sector squeezed into a square image): the
    fitted edge slope is then tan(fitted) = x_scale tan(true half angle), and only the vertical axis is a radius."""
    r = (np.arange(n_r) + 0.5) * geom["r_max_px"] / n_r
    half = np.radians(geom["half_angle_deg"])
    b = (np.arange(n_b) + 0.5) * 2 * half / n_b - half
    x = geom["apex_x"] + geom.get("x_scale", 1.0) * r[:, None] * np.sin(b)[None, :]
    y = geom["apex_y"] - r[:, None] * np.cos(b)[None, :]
    h, w = img.shape
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    ok = (x0 >= 0) & (x0 < w - 1) & (y0 >= 0) & (y0 < h - 1)
    xi, yi = np.clip(x0, 0, w - 2), np.clip(y0, 0, h - 2)
    fx, fy = x - xi, y - yi
    out = ((1 - fx) * (1 - fy) * img[yi, xi] + fx * (1 - fy) * img[yi, xi + 1]
           + (1 - fx) * fy * img[yi + 1, xi] + fx * fy * img[yi + 1, xi + 1])
    return np.where(ok, out, np.nan)


def load_fan(path: Path, frames: int, exponent: float, r_max_m: float = 1.0, colormap: str = "",
             fov_deg: float = 0.0, n_r: int = 0, n_b: int = 0):
    """A directory of rendered fan images. Ranges come out in metres only if --fan-range gave the display range
    of the recording; otherwise the range axis is a fraction of the outer radius. --fan-colormap undoes a
    colour map the renderer applied."""
    files = [f for f in sorted(path.iterdir()) if f.suffix.lower() in IMAGE_SUFFIXES][:frames]
    if not files:
        raise SystemExit(f"{path}: no images")
    cube = colormap_lut(colormap) if colormap else None
    read = lambda f: read_gray(f, cube)       # noqa: E731
    mean = np.mean([read(f) for f in files[:30]], 0)
    geom = fan_geometry(mean)
    if fov_deg:                               # the renderer did not keep the aspect ratio; trust the datasheet
        geom["x_scale"] = np.tan(np.radians(geom["half_angle_deg"])) / np.tan(np.radians(fov_deg / 2))
        geom["half_angle_deg"] = fov_deg / 2
    n_r = n_r or int(round(geom["r_max_px"]))
    n_b = n_b or 8 * int(round(2 * geom["half_angle_deg"]))
    r = (np.arange(n_r) + 0.5) * r_max_m / n_r
    half = geom["half_angle_deg"]
    bearing = (np.arange(n_b) + 0.5) * 2 * half / n_b - half
    for f in files:
        polar = fan_to_polar(read(f), geom, n_r, n_b)
        db = pixels_to_db(np.nan_to_num(polar, nan=0.0), exponent)
        yield np.where(np.isfinite(polar), db, np.nan), r, bearing


LOADERS = {"uw": load_uw, "uatd": load_uatd, "fan": load_fan}


def detect(path: Path) -> str:
    if (path / "manifest.json").is_file():
        return "uw"
    if (path / "images").is_dir() and (path / "annotations").is_dir():
        return "uatd"
    if any(f.suffix.lower() in IMAGE_SUFFIXES for f in path.iterdir()):
        return "fan"
    raise SystemExit(f"{path}: no loader for this format (LOADERS: {', '.join(LOADERS)})")


def frames_of(path: Path, kind: str, frames: int, exponent: float, fan_range: float, colormap: str,
              fan_fov: float):
    kw = {"r_max_m": fan_range, "colormap": colormap, "fov_deg": fan_fov} if kind == "fan" else {}
    return LOADERS[kind](path, frames, exponent, **kw)


def fit_exponent(path: Path, kind: str, fan_range: float, colormap: str, fan_fov: float,
                 sand_window, bearing_window) -> float:
    """The pixel exponent p that makes the background a Rayleigh speckle (intensity contrast 1). Contrast grows
    monotonically as p falls (the intensity exponent 2/p stretches the tail), so a bisection is enough."""
    def contrast(p):
        db, rng, brg = next(iter(frames_of(path, kind, 1, p, fan_range, colormap, fan_fov)))
        return sonar_stats.background(db, rng, brg, sand_window, bearing_window)["contrast"]
    lo, hi = 0.2, 4.0
    if contrast(hi) > 1.0:
        return hi
    if contrast(lo) < 1.0:
        return lo
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if contrast(mid) > 1.0 else (lo, mid)
    return 0.5 * (lo + hi)


def analyse(path: Path, frames: int, sand_window, bearing_window, tvg_window, target,
            exponent: float, fan_range: float, colormap: str, fan_fov: float, fit_law: bool) -> dict:
    kind = detect(path)
    if fit_law and kind != "uw":
        exponent = fit_exponent(path, kind, fan_range, colormap, fan_fov, sand_window, bearing_window)
    bg, tg, nr, tv = [], [], [], []
    for db, rng, brg in frames_of(path, kind, frames, exponent, fan_range, colormap, fan_fov):
        bg.append(sonar_stats.background(db, rng, brg, sand_window, bearing_window))
        tg.append(sonar_stats.bright_target(db, rng, brg, target[:2], target[2:]))
        nr.append(sonar_stats.near_range(db, rng))
        tv.append(sonar_stats.tvg_profile(db, rng, tvg_window))
    if not bg:
        raise SystemExit(f"{path}: no frames read")
    mean = lambda xs, k: float(np.mean([x[k] for x in xs]))       # noqa: E731
    return {"dataset": str(path), "format": kind, "frames": len(bg),
            "pixel_exponent": None if kind == "uw" else exponent,
            "background": {"window_m": list(sand_window), "window_deg": list(bearing_window),
                           "contrast": mean(bg, "contrast"), "k_shape_median": float(np.median([b["k_shape"] for b in bg])),
                           "ks_vs_rayleigh": mean(bg, "ks_vs_rayleigh"), "hist_edges": bg[0]["hist_edges"],
                           "hist": np.mean([b["hist"] for b in bg], 0).tolist()},
            "bright_target": {"range_m": mean(tg, "range_m"), "bearing_deg": mean(tg, "bearing_deg"),
                              "width_deg": mean(tg, "width_deg"), "sidelobe_db": mean(tg, "sidelobe_db")},
            "near_range": {"blind_zone_m": mean(nr, "blind_zone_m"), "ringdown_slope_db_per_m": mean(nr, "ringdown_slope_db_per_m"),
                           "range_m": nr[0]["range_m"], "mean_db": nr[0]["mean_db"]},
            "tvg": {"k_db_per_decade": mean(tv, "k_db_per_decade"), "b_db_per_m": mean(tv, "b_db_per_m"),
                    "fit_window_m": list(tvg_window), "range_m": tv[0]["range_m"], "median_db": tv[0]["median_db"]}}


def plot(results, path):
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    for r in results:
        d = Path(r["dataset"])
        name = f"{d.parent.name}/{d.name}" if d.name in ("sonar", "images") else d.name
        b = r["background"]
        centres = 0.5 * (np.array(b["hist_edges"][1:]) + np.array(b["hist_edges"][:-1]))
        ax[0].semilogy(centres, b["hist"], label=f"{name}: contrast {b['contrast']:.2f}")
        ax[1].plot(r["near_range"]["range_m"], [np.nan if v is None else v for v in r["near_range"]["mean_db"]], label=name)
        ax[2].plot(r["tvg"]["range_m"], [np.nan if v is None else v for v in r["tvg"]["median_db"]],
                   label=f"{name}: {r['tvg']['k_db_per_decade']:.1f} log R + {r['tvg']['b_db_per_m']:.2f} R")
    ax[0].semilogy(centres, np.exp(-centres), "k--", label="Rayleigh (exponential intensity)")
    ax[0].set(xlabel="intensity / local mean", ylabel="pdf", title="background speckle")
    ax[1].set(xlabel="range [m]", ylabel="mean level [dB]", title="blind zone and ring-down")
    ax[2].set(xlabel="range [m]", ylabel="median level [dB]", title="level vs range (TVG profile)")
    for a in ax:
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("datasets", nargs="+", type=Path)
    ap.add_argument("--frames", type=int, default=50)
    # The windows may be given once for all datasets, or once per dataset (recordings of different ranges have
    # no background window in common).
    ap.add_argument("--sand", type=float, nargs=2, action="append", help="background range window [m]")
    ap.add_argument("--bearings", type=float, nargs=2, action="append", help="background bearing window [deg]")
    ap.add_argument("--tvg", type=float, nargs=2, action="append", help="range window of the TVG fit [m]")
    ap.add_argument("--target", type=float, nargs=4, action="append", metavar=("R0", "R1", "B0", "B1"),
                    help="window [m, m, deg, deg] holding a point target for the width / sidelobe check")
    ap.add_argument("--pixel-exponent", type=float, default=1.0,
                    help="an 8-bit pixel of a real image is an amplitude to this power (1: linear amplitude)")
    ap.add_argument("--fit-pixel-law", action="store_true",
                    help="instead, fit the exponent per real dataset so that the background has contrast 1")
    ap.add_argument("--fan-range", type=float, action="append",
                    help="display range [m] of a 'fan' dataset; without it its range axis is a fraction of the outer radius")
    ap.add_argument("--fan-colormap", action="append",
                    help="colour map the renderer of a 'fan' dataset applied, to undo (e.g. inferno)")
    ap.add_argument("--fan-fov", type=float, action="append",
                    help="true horizontal aperture [deg] of a 'fan' dataset, when the rendering did not keep the aspect ratio")
    ap.add_argument("--out", type=Path, default=Path(config.WS) / "output" / "compare_real")
    args = ap.parse_args()
    n = len(args.datasets)
    def per_dataset(given, default):        # noqa: E306
        v = given or [default]
        if len(v) not in (1, n):
            raise SystemExit(f"give a window once or once per dataset ({n}), not {len(v)} times")
        return v * n if len(v) == 1 else v
    sand = per_dataset(args.sand, [2.15, 2.8])
    bearings = per_dataset(args.bearings, [-25.0, 25.0])
    tvg = per_dataset(args.tvg, [1.8, 3.4])
    target = per_dataset(args.target, [0.3, 1e9, -180, 180])
    fan_range = per_dataset(args.fan_range, 1.0)
    fan_colormap = per_dataset(args.fan_colormap, "")
    fan_fov = per_dataset(args.fan_fov, 0.0)
    results = [analyse(d, args.frames, sand[i], bearings[i], tvg[i], target[i], args.pixel_exponent,
                       fan_range[i], fan_colormap[i], fan_fov[i], args.fit_pixel_law)
               for i, d in enumerate(args.datasets)]
    os.makedirs(args.out, exist_ok=True)
    (args.out / "compare.json").write_text(json.dumps(results, indent=1))
    plot(results, args.out / "compare.png")
    for r in results:
        b, t, n, v = r["background"], r["bright_target"], r["near_range"], r["tvg"]
        p = "" if r["pixel_exponent"] is None else f", pixel exponent {r['pixel_exponent']:.3f}"
        print(f"{r['dataset']} [{r['format']}{p}] ({r['frames']} frames): background contrast {b['contrast']:.3f}, K shape "
              f"{b['k_shape_median']:.1f}, KS vs Rayleigh {b['ks_vs_rayleigh']:.3f}; brightest target at "
              f"{t['range_m']:.2f} m / {t['bearing_deg']:.1f} deg, width {t['width_deg']:.2f} deg, sidelobe "
              f"{t['sidelobe_db']:.1f} dB; blind zone {n['blind_zone_m']:.3f} m, ring-down {n['ringdown_slope_db_per_m']:.0f} "
              f"dB/m; level vs range {v['k_db_per_decade']:.1f} log R + {v['b_db_per_m']:.2f} R")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
