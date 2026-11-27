"""Compare sonar recordings, real or simulated, on what M2 checks.

    python -m uw py physics/tools/compare_real.py DATASET [DATASET ...] [--frames 50] [--out output/compare_real]

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
distance then says whether that p is self-consistent. Beware what that costs: a fitted p forces the contrast
to 1, so contrast is then no evidence at all and only the KS distance and the other statistics are.

A simulated dataset is measured on one of three channels (--uw-channel). `polar_db` is absolute dB re 1 uPa,
which no real recording is: a published image has the maker's TVG and display gain already applied and not
recoverable. So `tvg` and `img` put our own gain chain back on before comparing anything that depends on
level -- without that, the level-vs-range row compares a physical quantity against a displayed one.

--split restricts the real corpora to one half of the leak-free split that physics/tools/real_holdout.py wrote.
Calibrate on `calibrate`; read `holdout` once, at the end.
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

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS))
from uw import config  # noqa: E402
from physics import sonar_stats  # noqa: E402


IMAGE_SUFFIXES = (".bmp", ".png", ".jpg", ".jpeg")


UW_CHANNELS = ("db", "tvg", "img")


def uw_gain(path: Path) -> dict:
    """The gain chain that turns the simulation's physical dB into the sensor units a real recording is in.

    `physics/sonar.py`'s display_kernel computes `level + k log10(R) + 2 alpha R - nf_db` and maps the result
    through `sonar.display_db` into 8 bits, while the stored `polar_db` stays absolute dB re 1 uPa. A real
    8-bit recording has the maker's equivalent of that chain already applied and not recoverable, which is
    why the previous comparison read -158.9 dB/decade for the sim against +23.7 for UATD: it compared a
    physical level against a displayed one. Reapplying our own chain is what makes the two comparable.

    Read from the per-ping JSON, which `sim/recorder.py` writes for exactly this reason ("without which the
    8-bit polar image is not invertible"), and fall back to the dataset's own resolved config for the fields
    that datasets written before that export predate.
    """
    first = sorted((path / "sonar").glob("*.json"))
    if not first:
        raise SystemExit(f"{path}: no per-ping JSON, so the gain chain is unknown; --uw-channel db only")
    meta = json.loads(first[0].read_text()).get("sonar", {})
    man = json.loads((path / "manifest.json").read_text()) if (path / "manifest.json").is_file() else {}
    cfg = man.get("config", {})
    out = {"noise_floor_db": meta.get("noise_floor_db"), "display_db": meta.get("display_db"),
           "tvg_db_per_decade": meta.get("tvg_db_per_decade"), "absorption_db_per_m": meta.get("absorption_db_per_m")}
    if out["tvg_db_per_decade"] is None:
        out["tvg_db_per_decade"] = cfg.get("sonar", {}).get("tvg_db_per_decade")
    if out["display_db"] is None:
        out["display_db"] = cfg.get("sonar", {}).get("display_db")
    if out["absorption_db_per_m"] is None and cfg:
        from physics import water as wat
        md = config.sonar_mode(cfg)
        w = cfg["water"]
        out["absorption_db_per_m"] = wat.absorption_db_per_m(
            float(md["frequency_hz"]), float(w["temperature_c"]), float(w["salinity_psu"]),
            float(w.get("ambient_depth_m", 0.0)))
    out["min_range"] = float(config.sonar_mode(cfg)["min_range"]) if cfg else 0.0
    missing = [k for k, v in out.items() if v is None]
    if missing:
        raise SystemExit(f"{path}: the gain chain is incomplete ({', '.join(missing)}); "
                         f"regenerate the dataset or use --uw-channel db")
    return out


def uw_sensor_units(db, range_m, gain: dict, channel: str, exponent: float, amp_top_db: float | None = None):
    """`db` (absolute dB re 1 uPa) through a display chain, so it can be set against a real recording.

    `tvg`: the TVG-corrected level, in dB relative to the beam noise floor -- the quantity whose *shape* a
    real recording's level-vs-range can be compared against. This is the safe channel; prefer it.

    `img`: that, encoded into 8 bits **the way a real head does it, not the way we do it**. Our own
    `display_db` window is linear in dB, while an Oculus or Gemini 8-bit sample is linear in amplitude -- the
    pixel law fitted on two real Geminis came out p = 1.0-1.1. The two are not interchangeable: 25-40 dB of
    sand lands on pixels 80-128 in our encoding and, read back through `pixels_to_db`, spans only 4 dB
    instead of 15, which collapsed the measured speckle contrast from 2.13 to 0.43. That was an artifact of
    the conversion, not a property of the sonar. So `img` goes to amplitude, quantises, and reads back
    through the same law a real image is read with.

    An amplitude display has **one** knob, not two: the gain that sets which level saturates. `amp_top_db`
    is it, defaulting to the top of `display_db`. It has to be chosen so the seabed sits mid-scale, exactly
    as a real head's gain is set -- with our default window top of 80 dB, sand at 25-40 dB above the noise
    floor quantises to pixels 0-3, i.e. black. The floor needs no knob: everything below the bottom of the
    8-bit range is pixel 0, which `pixels_to_db` floors at half an LSB.

    NaN (the blind zone, where the sonar returns nothing) stays NaN on every channel.
    """
    rr = np.maximum(np.asarray(range_m, np.float32), gain["min_range"])[:, None]
    v = db + gain["tvg_db_per_decade"] * np.log10(rr) + 2.0 * gain["absorption_db_per_m"] * rr - gain["noise_floor_db"]
    if channel == "tvg":
        return v
    top = float(gain["display_db"][1] if amp_top_db is None else amp_top_db)
    u8 = np.round(np.clip(10.0 ** ((v - top) / 20.0), 0.0, 1.0) * 255.0)
    return np.where(np.isfinite(v), pixels_to_db(np.nan_to_num(u8), exponent), np.nan)


def load_uw(path: Path, frames: int, exponent: float, channel: str = "db", keep=None,
            amp_top_db: float | None = None):
    gain = uw_gain(path) if channel != "db" else None
    for f in sorted((path / "sonar").glob("*.npz"))[:frames]:
        with np.load(f) as z:
            db, rng, brg = z["polar_db"].astype(np.float32), z["range_m"], z["bearing_deg"]
        yield (db if channel == "db" else uw_sensor_units(db, rng, gain, channel, exponent, amp_top_db)), rng, brg


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


def load_uatd(path: Path, frames: int, exponent: float, keep=None):
    """UATD: a polar image per ping with its own geometry in the sibling XML."""
    import xml.etree.ElementTree as ET
    for f in only(sorted((path / "images").iterdir()), keep)[:frames]:
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
             fov_deg: float = 0.0, n_r: int = 0, n_b: int = 0, keep=None):
    """A directory of rendered fan images. Ranges come out in metres only if --fan-range gave the display range
    of the recording; otherwise the range axis is a fraction of the outer radius. --fan-colormap undoes a
    colour map the renderer applied."""
    files = only([f for f in sorted(path.iterdir()) if f.suffix.lower() in IMAGE_SUFFIXES], keep)[:frames]
    if not files:
        raise SystemExit(f"{path}: no images" + (" in this split" if keep is not None else ""))
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


def only(files, keep):
    """``files`` restricted to a hold-out split, or all of them when no split was asked for."""
    return list(files) if keep is None else [f for f in files if f.resolve() in keep]


def split_files(holdout: Path, dataset: Path, half: str) -> set:
    """The resolved paths of one half of one corpus, from the manifest `physics/tools/real_holdout.py` wrote.

    The split is read, never recomputed here: the grouping that makes it leak-free lives in that tool, and a
    second opinion about which frames belong together would quietly undo the guarantee. A corpus that the
    manifest does not cover, or a half it leaves empty, is an error rather than a silent fall back to
    everything -- calibrating on the hold-out is exactly the mistake this is here to prevent.
    """
    man = json.loads(holdout.read_text())
    if half not in ("calibrate", "holdout"):
        raise SystemExit(f"--split must be calibrate or holdout, not {half!r}")
    want = dataset.resolve()
    for name, d in man["datasets"].items():
        root = Path(d["root"])
        paths = {(root / p).resolve() for p in d["split"][half]}
        allp = {(root / p).resolve() for h in ("calibrate", "holdout") for p in d["split"][h]}
        if not any(want == p or want in p.parents for p in allp):
            continue
        under = {p for p in paths if want == p or want in p.parents}
        if not under:
            raise SystemExit(f"{dataset}: corpus '{name}' has no frames in the '{half}' half "
                             f"(its holdout share is {d['counts'].get('holdout', 0)}/{d['frames']})")
        return under
    raise SystemExit(f"{dataset}: not covered by {holdout}; add it to physics/tools/real_holdout.py and rebuild")


def uatd_at_range(path: Path, r_max: float, tol: float = 0.5) -> set:
    """UATD frames whose range setting is within ``tol`` of ``r_max``, as resolved paths.

    UATD's range setting changes from ping to ping -- 7 to 25 m among 60 consecutive frames -- so a fixed
    range window samples a different part of the image in each one, and a near-range or level-vs-range
    statistic pooled over the corpus is an average over incomparable things. Stratifying by range setting is
    the cheapest way to make the pool comparable; it is not optional for those two metrics.
    """
    import xml.etree.ElementTree as ET
    out = set()
    for f in sorted((path / "images").iterdir()):
        if f.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        x = path / "annotations" / (f.stem + ".xml")
        if not x.is_file():
            continue
        if abs(float(ET.parse(x).getroot().find("sonar").findtext("range")) - r_max) <= tol:
            out.add(f.resolve())
    return out


def detect(path: Path) -> str:
    if (path / "manifest.json").is_file():
        return "uw"
    if (path / "images").is_dir() and (path / "annotations").is_dir():
        return "uatd"
    if any(f.suffix.lower() in IMAGE_SUFFIXES for f in path.iterdir()):
        return "fan"
    raise SystemExit(f"{path}: no loader for this format (LOADERS: {', '.join(LOADERS)})")


def frames_of(path: Path, kind: str, frames: int, exponent: float, fan_range: float, colormap: str,
              fan_fov: float, channel: str = "db", keep=None, amp_top_db: float | None = None):
    kw = {"keep": keep}
    if kind == "fan":
        kw.update(r_max_m=fan_range, colormap=colormap, fov_deg=fan_fov)
    if kind == "uw":
        kw.update(channel=channel, amp_top_db=amp_top_db)
    return LOADERS[kind](path, frames, exponent, **kw)


def mean_profile(per_frame: list, value_key: str, grid: np.ndarray) -> list:
    """Every frame's profile interpolated onto one range grid and averaged.

    Necessary rather than tidy: UATD's range setting changes from ping to ping, so the frames share no range
    axis and the earlier code quoted frame 0 alone -- one noisy profile standing in for fifty. Frames that do
    not reach a grid point do not vote on it, and a point no frame reaches stays null.
    """
    acc = np.full((len(per_frame), len(grid)), np.nan)
    for i, fr in enumerate(per_frame):
        r = np.asarray(fr["range_m"], float)
        v = np.array([np.nan if x is None else x for x in fr[value_key]], float)
        ok = np.isfinite(r) & np.isfinite(v)
        if ok.sum() > 1:
            acc[i] = np.interp(grid, r[ok], v[ok], left=np.nan, right=np.nan)
    any_ok = np.isfinite(acc).any(0)
    out = np.full(len(grid), np.nan)
    out[any_ok] = np.nanmean(acc[:, any_ok], 0)
    return [None if not np.isfinite(v) else float(v) for v in out]


def fit_exponent(path: Path, kind: str, fan_range: float, colormap: str, fan_fov: float,
                 sand_window, bearing_window, keep=None) -> float:
    """The pixel exponent p that makes the background a Rayleigh speckle (intensity contrast 1). Contrast grows
    monotonically as p falls (the intensity exponent 2/p stretches the tail), so a bisection is enough."""
    def contrast(p):
        db, rng, brg = next(iter(frames_of(path, kind, 1, p, fan_range, colormap, fan_fov, keep=keep)))
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
            exponent: float, fan_range: float, colormap: str, fan_fov: float, fit_law: bool,
            channel: str = "db", keep=None, split: str | None = None, near_ref=(0.3, 0.5),
            uatd_range: float | None = None, amp_top_db: float | None = None) -> dict:
    kind = detect(path)
    if fit_law and kind != "uw":
        exponent = fit_exponent(path, kind, fan_range, colormap, fan_fov, sand_window, bearing_window, keep)
    bg, tg, nr, tv = [], [], [], []
    for db, rng, brg in frames_of(path, kind, frames, exponent, fan_range, colormap, fan_fov, channel, keep,
                                  amp_top_db):
        bg.append(sonar_stats.background(db, rng, brg, sand_window, bearing_window))
        tg.append(sonar_stats.bright_target(db, rng, brg, target[:2], target[2:]))
        nr.append(sonar_stats.near_range(db, rng, normalise="median", ref_window=near_ref))
        tv.append(sonar_stats.tvg_profile(db, rng, tvg_window, normalise="median"))
    if not bg:
        raise SystemExit(f"{path}: no frames read")
    mean = lambda xs, k: float(np.mean([x[k] for x in xs]))       # noqa: E731
    near_grid = np.linspace(0.0, 0.5, 251)
    tvg_grid = np.linspace(float(tvg_window[0]), float(tvg_window[1]), 201)
    return {"dataset": str(path), "format": kind, "frames": len(bg),
            "pixel_exponent": None if kind == "uw" else exponent,
            # Everything the numbers below depend on, so that a result is reproducible from its own artifact.
            # The previous run's contrast 0.33 for basin-2 could not be explained from compare.json alone,
            # because the colour map it did (not) undo was never recorded.
            "loader": {"channel": channel if kind == "uw" else None, "split": split,
                       "uatd_range_m": uatd_range,
                       "amp_top_db": amp_top_db if kind == "uw" and channel == "img" else None,
                       "fan_range_m": fan_range if kind == "fan" else None,
                       "fan_colormap": colormap or None if kind == "fan" else None,
                       "fan_fov_deg": fan_fov or None if kind == "fan" else None,
                       "fit_pixel_law": bool(fit_law and kind != "uw")},
            "background": {"window_m": list(sand_window), "window_deg": list(bearing_window),
                           "contrast": mean(bg, "contrast"), "k_shape_median": float(np.median([b["k_shape"] for b in bg])),
                           "contrast_sd": float(np.std([b["contrast"] for b in bg])),
                           "ks_vs_rayleigh": mean(bg, "ks_vs_rayleigh"), "hist_edges": bg[0]["hist_edges"],
                           "hist": np.mean([b["hist"] for b in bg], 0).tolist()},
            "bright_target": {"range_m": mean(tg, "range_m"), "bearing_deg": mean(tg, "bearing_deg"),
                              "width_deg": mean(tg, "width_deg"), "sidelobe_db": mean(tg, "sidelobe_db"),
                              "width_deg_sd": float(np.std([t["width_deg"] for t in tg]))},
            "near_range": {"blind_zone_m": mean(nr, "blind_zone_m"), "ringdown_slope_db_per_m": mean(nr, "ringdown_slope_db_per_m"),
                           "norm_ref_window_m": list(near_ref), "frames_averaged": len(nr),
                           "range_m": near_grid.tolist(), "mean_db": mean_profile(nr, "mean_db", near_grid),
                           "norm_db": mean_profile(nr, "norm_db", near_grid)},
            "tvg": {"k_db_per_decade": mean(tv, "k_db_per_decade"), "b_db_per_m": mean(tv, "b_db_per_m"),
                    "k_db_per_decade_sd": float(np.std([t["k_db_per_decade"] for t in tv])),
                    "fit_window_m": list(tvg_window), "frames_averaged": len(tv),
                    "range_m": tvg_grid.tolist(), "median_db": mean_profile(tv, "median_db", tvg_grid),
                    "norm_db": mean_profile(tv, "norm_db", tvg_grid)}}


def plot(results, path):
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    for r in results:
        d = Path(r["dataset"])
        name = f"{d.parent.name}/{d.name}" if d.name in ("sonar", "images") else d.name
        b = r["background"]
        centres = 0.5 * (np.array(b["hist_edges"][1:]) + np.array(b["hist_edges"][:-1]))
        ax[0].semilogy(centres, b["hist"], label=f"{name}: contrast {b['contrast']:.2f}")
        nan = lambda xs: [np.nan if v is None else v for v in xs]       # noqa: E731
        ax[1].plot(r["near_range"]["range_m"], nan(r["near_range"]["norm_db"]), label=name)
        ax[2].plot(r["tvg"]["range_m"], nan(r["tvg"]["norm_db"]),
                   label=f"{name}: {r['tvg']['k_db_per_decade']:.1f} log R + {r['tvg']['b_db_per_m']:.2f} R")
    ax[0].semilogy(centres, np.exp(-centres), "k--", label="Rayleigh (exponential intensity)")
    ax[0].set(xlabel="intensity / local mean", ylabel="pdf", title="background speckle")
    ax[1].set(xlabel="range [m]", ylabel="mean level re local background [dB]",
              title="blind zone and ring-down (normalised)")
    ax[2].set(xlabel="range [m]", ylabel="median level re its own median [dB]",
              title="level vs range, shape only (TVG profile)")
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
    ap.add_argument("--uw-channel", default="db", choices=UW_CHANNELS,
                    help="which channel of a 'uw' dataset to measure: db = absolute dB re 1 uPa (the physics), "
                         "tvg = our own TVG applied, re the beam noise floor, img = that clipped into the "
                         "display window and quantised to 8 bits. A real recording is always the last kind, "
                         "so 'db' is not comparable with one on level or TVG (default: %(default)s)")
    ap.add_argument("--uw-amp-top-db", type=float, metavar="DB",
                    help="with --uw-channel img: the TVG-corrected level [dB over the beam noise floor] that "
                         "saturates the 8-bit image, i.e. the gain. Choose it so the seabed sits mid-scale, as "
                         "a real head's gain is set; the default (sonar.display_db's top, 80 dB) is a dB-window "
                         "top and puts sand at pixel 0-3")
    ap.add_argument("--uatd-range", type=float, metavar="R",
                    help="restrict UATD frames to the range setting R [m]. Its range changes ping to ping, so "
                         "near-range and level-vs-range statistics pooled over the corpus average over "
                         "incomparable images")
    ap.add_argument("--split", choices=("calibrate", "holdout"),
                    help="restrict every real dataset to one half of the split in --holdout. The hold-out is "
                         "meant to be read once, at the end; everything before that runs on 'calibrate'")
    ap.add_argument("--holdout", type=Path,
                    default=Path(config.WS) / "output" / "compare_real" / "holdout.json",
                    help="the split manifest written by physics/tools/real_holdout.py (default: %(default)s)")
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
    keep = [None if args.split is None or detect(d) == "uw" else split_files(args.holdout, d, args.split)
            for d in args.datasets]
    if args.uatd_range is not None:
        for i, d in enumerate(args.datasets):
            if detect(d) != "uatd":
                continue
            at = uatd_at_range(d, args.uatd_range)
            if not at:
                raise SystemExit(f"{d}: no frames at range {args.uatd_range} m")
            keep[i] = at if keep[i] is None else (keep[i] & at)
            if not keep[i]:
                raise SystemExit(f"{d}: no frames at range {args.uatd_range} m in the "
                                 f"'{args.split}' half")
    results = [analyse(d, args.frames, sand[i], bearings[i], tvg[i], target[i], args.pixel_exponent,
                       fan_range[i], fan_colormap[i], fan_fov[i], args.fit_pixel_law,
                       args.uw_channel, keep[i], args.split, (0.3, 0.5), args.uatd_range, args.uw_amp_top_db)
               for i, d in enumerate(args.datasets)]
    os.makedirs(args.out, exist_ok=True)
    (args.out / "compare.json").write_text(json.dumps(results, indent=1))
    plot(results, args.out / "compare.png")
    for r in results:
        b, t, n, v = r["background"], r["bright_target"], r["near_range"], r["tvg"]
        p = "" if r["pixel_exponent"] is None else f", pixel exponent {r['pixel_exponent']:.3f}"
        p += f", channel {r['loader']['channel']}" if r["loader"]["channel"] else ""
        p += f", {r['loader']['split']}" if r["loader"]["split"] else ""
        p += f", {r['loader']['uatd_range_m']} m" if r["loader"].get("uatd_range_m") else ""
        print(f"{r['dataset']} [{r['format']}{p}] ({r['frames']} frames): background contrast {b['contrast']:.3f}, K shape "
              f"{b['k_shape_median']:.1f}, KS vs Rayleigh {b['ks_vs_rayleigh']:.3f}; brightest target at "
              f"{t['range_m']:.2f} m / {t['bearing_deg']:.1f} deg, width {t['width_deg']:.2f} deg, sidelobe "
              f"{t['sidelobe_db']:.1f} dB; blind zone {n['blind_zone_m']:.3f} m, ring-down {n['ringdown_slope_db_per_m']:.0f} "
              f"dB/m; level vs range {v['k_db_per_decade']:.1f} log R + {v['b_db_per_m']:.2f} R")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
