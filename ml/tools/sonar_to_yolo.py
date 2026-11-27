"""Convert a sweep dataset (tools/sweep.py) into a YOLO-seg dataset and an RF-DETR-readable COCO dataset.

    python3 tools/sonar_to_yolo.py --root /HDD2T/underwater_data/datasets/v0.2 --out /HDD2T/underwater_data/yolo/v0.2
    python3 tools/sonar_to_yolo.py status --out DIR        what is already converted
    python3 tools/sonar_to_yolo.py check  --out DIR        verify what was written

CPU only: no GPU, no warp, no torch. It reads the PNGs the sweep wrote (the npz only for --db-range and
--repr fan), turns each ping's label mask into polygons, and writes both formats from the same polygons, so
they agree by construction. The split is not invented here, it comes from the sweep manifest, per scenario:
neighbouring pings of one scenario are 0.1 s apart and nearly identical, so splitting per frame would leak.

Orientation, which is the one silent corruption this conversion can produce: the sweep stores every PNG
flipped (row 0 = far range) and the npz arrays unflipped (row 0 = range 0). We flip every PNG on the way in,
exactly as uw.dataset.load_ping does, so images, masks and anything taken from the npz all share row 0 =
range 0. Flipping only some of them mirrors the labels in range, and no loader would say a word.

Images are written once, under <out>/images/<split>/, and every dataset directory hardlinks them, so the mask
variants and the two formats share one copy of the pixels (~20 GB for v0.2's 36000 polar pings). Both loaders
open an image by path and follow hardlinks. The split directories themselves are real directories, which is
what Ultralytics needs: it resolves the data.yaml paths with Path.resolve(), and a symlinked split directory
loses the /images/ -> /labels/ substitution it finds its labels with.

The labels are an envelope, not the raw mask. An energy-share mask is speckle-fragmented -- one positive v0.2
ping's mask_direct reaches 4118 connected components, 3801 of them under 10 px -- and one polygon per
component is not merely noisy: a 1-px component gives a 1-point contour, which Ultralytics counts as a corrupt
label and drops the whole image for, and which pycocotools refuses outright. So the mask is closed,
hole-filled and cleared of components under --min-area first, and meta/frames.csv records per frame what that
cost (mask_px, components_raw, components_kept, dropped_px, instance_px) so the choice stays auditable. The
defaults were measured over 200 sampled positive frames: --close 3 holds the instance count where --close 5
and 7 do (4 per frame, median) while inflating the labelled area 1.18x instead of 1.6-2.0x.

The default label is mask_geom, and that is a deliberate departure from the obvious choice. mask_direct and
mask_all mark *energy ownership*, not plume extent: in 54 % of v0.2's positive frames mask_direct spans 480 of
the 512 beams, because in the quiet water column above the seabed the bubbles own nearly all of a very small
echo. A detector trained on a 487-beam band learns the water column, not the leak. mask_geom is the main-lobe
line-of-sight footprint -- one instance per frame, sitting on the bright plume -- so it owns dataset_yolo/ and
dataset_coco/, and the two energy masks ship beside it as comparison variants. docs/INSPECTION.md has the
numbers, including --level-gate 128, which recovers a plume-shaped label from mask_direct.

Resumable: a scenario with a marker under <out>/.scn whose parameters match is skipped, anything else is
redone from scratch, and the per-scenario COCO fragments are concatenated at the end, so an interrupted run
costs only the scenarios it had not finished.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import multiprocessing as mp
import os
import random
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))

MASKS = ("direct", "all", "geom")
SPLITS = ("train", "val", "test")
COCO_SPLIT_DIR = {"train": "train", "val": "valid", "test": "test"}      # RF-DETR reads "valid", not "val"
CLASS_ID_COCO = 1
MARKER = "params.json"          # <out>/meta/params.json, written last: proves the output is ours to overwrite
# The parameters a scenario's conversion depends on. A change to any of them invalidates its marker.
SIG_KEYS = ("root", "repr", "masks", "db_range", "db_noise_floor", "level_gate", "close", "min_area",
            "fill_holes", "simplify", "max_points", "fan_width", "png_level", "stack3", "imgsz", "mask_ratio")


# ------------------------------------------------------------------------------------ the Cartesian fan

def fan_geometry(range_m: np.ndarray, bearing_deg: np.ndarray, width: int) -> dict:
    """Sonar.fan's constants (uw/sonar.py:1312-1319) recovered from the npz arrays instead of the config.

    theta_max comes from bearing_deg, never from a constant: lf is a 130 deg fan and hf a 70 deg one, and lf is
    65.5 % of v0.2's pings. The sidecar JSON does not record fov_h_deg, so the npz is the only per-ping source."""
    dr, rmax = float(range_m[1] - range_m[0]), float(range_m[-1])
    theta_max = math.radians(float(bearing_deg[-1]))
    scale = (width - 1) / (2 * rmax * max(math.sin(theta_max), 0.5))
    return {"dr": dr, "rmax": rmax, "theta_max": theta_max, "scale": scale,
            "h": int(rmax * scale) + 2, "w": int(width),
            "b0": -theta_max, "db": 2 * theta_max / (len(bearing_deg) - 1)}


def fan_maps(range_m: np.ndarray, bearing_deg: np.ndarray, width: int, off: tuple = (0.0, 0.0)) -> tuple:
    """(beam coordinate, range-bin coordinate, in-sector mask) per output pixel, float32 as warp does it.

    x = (col - (W-1)/2)/scale metres to starboard, y = (H-1-row)/scale metres along the axis: the apex sits on
    the bottom row, starboard to the right, exactly as uw/sonar.py's fan_kernel builds it. ``off`` shifts the
    sampling point inside the pixel, which is how the mask is supersampled."""
    f32 = np.float32
    g = fan_geometry(range_m, bearing_deg, width)
    h, w, scale = g["h"], g["w"], f32(g["scale"])
    x = (np.arange(w, dtype=f32) + f32(off[0]) - f32(0.5) * f32(w - 1)) / scale
    y = (f32(h - 1) - np.arange(h, dtype=f32) - f32(off[1])) / scale
    xx, yy = np.meshgrid(x, y)
    rr = np.sqrt(xx * xx + yy * yy, dtype=f32)
    th = np.arctan2(xx, yy).astype(f32)
    inside = (rr <= f32(g["rmax"])) & (np.abs(th) <= f32(g["theta_max"]))
    return ((th - f32(g["b0"])) / f32(g["db"])), (rr / f32(g["dr"])), inside


def fan_image(polar: np.ndarray, maps: tuple) -> np.ndarray:
    """Bilinear, bit-faithful to fan_kernel: truncating index, edge-clamped 2x2, round-half-up to uint8.
    Checked against the 44 GPU-written fan PNGs under output/sonar_preview: max 1 grey level on 6e-5 of the
    pixels, which is float32's last bit landing on the rounding tie."""
    f32 = np.float32
    map_x, map_y, inside = maps
    n_r, n_b = polar.shape
    j = np.clip(map_y.astype(np.int64), 0, n_r - 2)
    i = np.clip(map_x.astype(np.int64), 0, n_b - 2)
    f, g = np.clip(map_y - j.astype(f32), 0, 1), np.clip(map_x - i.astype(f32), 0, 1)
    p = polar.astype(f32)
    val = ((1 - f) * ((1 - g) * p[j, i] + g * p[j, i + 1]) + f * ((1 - g) * p[j + 1, i] + g * p[j + 1, i + 1]))
    return np.where(inside, (val + f32(0.5)).astype(np.uint8), 0).astype(np.uint8)


def fan_mask(mask: np.ndarray, maps_ss: list) -> np.ndarray:
    """Nearest, 3x3 supersampled inside each output pixel then majority-thresholded.

    Plain nearest undersamples badly: at width 1024 the basin fan takes 0.28 output pixels per range bin, so one
    sample per pixel is a coin flip (IoU 0.64 against a 4x reference, and a 34 % inflated perimeter). Nine
    samples bring that to 0.90 with no area bias -- an odd factor leaves no 0.5 tie."""
    n_r, n_b = mask.shape
    acc = None
    for map_x, map_y, inside in maps_ss:
        j = np.clip(np.rint(map_y).astype(np.int64), 0, n_r - 1)
        i = np.clip(np.rint(map_x).astype(np.int64), 0, n_b - 1)
        v = np.where(inside, mask[j, i], 0).astype(np.float32)
        acc = v if acc is None else acc + v
    return (acc >= 0.5 * len(maps_ss)).astype(np.uint8)


# ------------------------------------------------------------------------------------ reading one ping

def read_grey(path: Path) -> np.ndarray:
    """cv2, not uw.sinks.read_png: the stdlib reader scans for the IDAT marker a byte at a time, 150 ms on a
    3201x512 frame against cv2's 18 ms, which is 80 CPU-minutes over 36000 pings. `check --root` cross-checks
    the two on a sample so the fast path stays honest."""
    a = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if a is None:
        raise OSError(f"cannot read {path}")
    return a


def ping_arrays(d: Path, stem: str, masks, cfg, meta: dict) -> dict:
    """The display image and the requested masks of one ping, every array with row 0 = range 0.

    The sidecar's plume_pixels is np.count_nonzero of the very mask being read, so it is a free per-ping oracle:
    a mismatch means a flip, the wrong file or a decode fault, and it is worth finding on ping 1 rather than on
    epoch 1."""
    out = {"image": read_grey(d / f"{stem}.png")[::-1]}
    if cfg.repr == "fan" or cfg.db_range is not None:
        with np.load(d / f"{stem}.npz") as z:
            out["range_m"], out["bearing_deg"] = z["range_m"].copy(), z["bearing_deg"].copy()
            if cfg.db_range is not None:
                # polar_db is the calibrated level (dB re 1 uPa) *before* TVG, NaN in the blind zone, so a fixed
                # window on it is session-independent but range-shaded, unlike the stored display image, which
                # already carries 30 dB/decade of TVG and the 0-80 dB window. --db-noise-floor reproduces
                # uw/dataset.py's "db" image instead, which is relative to this ping's own noise floor.
                lo, hi = cfg.db_range
                db = z["polar_db"].astype(np.float32)
                if cfg.db_noise_floor:
                    db = db - float(meta["sonar"]["noise_floor_db"])
                db = np.nan_to_num(db, nan=lo)
                out["image"] = (np.clip((db - lo) / (hi - lo), 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
    for m in masks:
        a = (read_grey(d / f"{stem}_mask_{m}.png")[::-1] > 0).astype(np.uint8)
        if a.shape != out["image"].shape:
            raise AssertionError(f"{d.name}/{stem}: mask_{m} is {a.shape}, image is {out['image'].shape}")
        n, want = int(a.sum()), int(meta["plume_pixels"][m])
        if n != want:
            raise AssertionError(f"{d.name}/{stem}: mask_{m} has {n} px, the sidecar says {want}")
        out[f"mask_{m}"] = a
    return out


# ------------------------------------------------------------------------------------ mask -> polygons

def fill_holes(m: np.ndarray) -> np.ndarray:
    """Fill the speckle holes inside a component: flood the background in from a padded border, keep what it
    could not reach. Neither a YOLO-seg line nor a COCO polygon can express a hole -- both rasterise a ring
    solid -- so leaving them makes a polygon disagree with its own mask."""
    h, w = m.shape
    pad = np.zeros((h + 2, w + 2), np.uint8)
    pad[1:-1, 1:-1] = m
    cv2.floodFill(pad, np.zeros((h + 4, w + 4), np.uint8), (0, 0), 1)
    return (m | (1 - pad[1:-1, 1:-1])).astype(np.uint8)


def poly_area(poly: np.ndarray) -> int:
    """The polygon's area as it will actually be rasterised, not its shoelace area. The two differ once a ring
    is simplified, and COCO's `area` is what COCOeval bins on, so it should be the area that is drawn."""
    x, y, w, h = cv2.boundingRect(poly)
    buf = np.zeros((h, w), np.uint8)
    cv2.fillPoly(buf, [poly - np.array([x, y], np.int32)], 1)
    return int(buf.sum())


def instances(mask: np.ndarray, image: np.ndarray, cfg) -> tuple:
    """(polygons, audit) for one mask. A polygon is an int32 (n, 2) array of >= 3 points, x = column = beam."""
    m, audit = mask, {"mask_px": int(mask.sum())}
    if cfg.level_gate > 0:
        # Optional: keep only what is also visible. The share masks mark cells where the bubbles own most of
        # the echo, and in the empty water column that is true of almost nothing at all.
        m = (m & (image > cfg.level_gate)).astype(np.uint8)
    audit["gated_px"] = int(m.sum())
    if cfg.close > 1:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.close, cfg.close)))
    if cfg.fill_holes:
        m = fill_holes(m)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    keep = [i for i in range(1, n) if stats[i, 4] >= cfg.min_area]
    audit |= {"components_raw": n - 1, "components_kept": len(keep),
              "dropped_px": int(sum(stats[i, 4] for i in range(1, n) if stats[i, 4] < cfg.min_area))}
    polys = []
    for i in keep:
        cs, _ = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            if len(c) < 3:
                continue
            per = cv2.arcLength(c, True)
            eps, a = cfg.simplify * per, cv2.approxPolyDP(c, cfg.simplify * per, True)
            # A contour over ~1000 points inflates Ultralytics' resample length for every instance in the same
            # image (data/dataset.py:283), so the cap is not only about file size.
            while len(a) > cfg.max_points and eps < per:
                eps *= 1.5
                a = cv2.approxPolyDP(c, eps, True)
            a = a.reshape(-1, 2).astype(np.int32)
            _, _, bw, bh = cv2.boundingRect(a)
            # RF-DETR drops a box of zero width or height (strict inequality) and a ring that rasterises empty
            # is invisible in both formats, so neither is worth writing.
            if len(a) >= 3 and bw > 0 and bh > 0 and poly_area(a) >= 1:
                polys.append(a)
    audit |= {"instances": len(polys), "instance_px": int(sum(poly_area(p) for p in polys))}
    return polys, audit


def yolo_line(poly: np.ndarray, w: int, h: int, cls: int = 0) -> str:
    """`0 x1 y1 ...`, normalised to [0, 1]. Ultralytics bounds-checks the box it derives from the vertices, not
    the vertices themselves, so the clipping has to happen here."""
    xy = np.clip(poly.astype(np.float64) / np.array([w, h]), 0.0, 1.0)
    return f"{cls} " + " ".join(f"{v:.6f}" for v in xy.reshape(-1))


def coco_annotation(poly: np.ndarray, image_id: int) -> dict:
    x, y, bw, bh = cv2.boundingRect(poly)
    return {"image_id": image_id, "category_id": CLASS_ID_COCO,
            "segmentation": [[float(v) for v in poly.reshape(-1)]],
            "bbox": [float(x), float(y), float(bw), float(bh)],
            "area": float(poly_area(poly)), "iscrowd": 0}


# ------------------------------------------------------------------------------------ one scenario

def signature(cfg) -> str:
    return hashlib.sha1(json.dumps({k: str(getattr(cfg, k)) for k in SIG_KEYS},
                                   sort_keys=True).encode()).hexdigest()[:16]


def scenario_rows(job) -> list:
    """Worker: convert one scenario. Writes its images, its label files for every mask variant, its hardlinks
    and its COCO fragments, then returns one small record per frame -- without the polygons, which would put a
    gigabyte of python lists in the parent on a box with under 3 GB free."""
    root, out, sc, idx, cfg = job
    sig = signature(cfg)
    marker = out / ".scn" / f"{sc['id']}.json"
    if marker.exists():
        done = json.loads(marker.read_text())
        if done.get("signature") == sig and done.get("split") == sc["split"]:
            return done["rows"]
    d = root / sc["id"]
    stems = sorted(p.stem for p in d.glob("ping_*.npz"))
    want_fan = cfg.repr == "fan"
    cache, maps = {}, {}
    frag = {m: {"images": [], "annotations": []} for m in cfg.masks}

    def arrays(i):
        if i not in cache:
            meta = json.loads((d / f"{stems[i]}.json").read_text())
            cache[i] = (ping_arrays(d, stems[i], cfg.masks, cfg, meta), meta)
        return cache[i]

    def warped(i):
        a, _ = arrays(i)
        if not want_fan:
            return a["image"]
        if "img" not in maps:       # one geometry per scenario: mode and range do not change inside one
            maps["img"] = fan_maps(a["range_m"], a["bearing_deg"], cfg.fan_width)
            maps["ss"] = [fan_maps(a["range_m"], a["bearing_deg"], cfg.fan_width, (dx, dy))
                          for dy in (-1 / 3, 0.0, 1 / 3) for dx in (-1 / 3, 0.0, 1 / 3)]
        return fan_image(a["image"], maps["img"])

    rows = []
    for i, stem in enumerate(stems):
        a, meta = arrays(i)
        img = warped(i)
        msk = {m: (fan_mask(a[f"mask_{m}"], maps["ss"]) if want_fan else a[f"mask_{m}"]) for m in cfg.masks}
        if cfg.stack3:
            # t-1, t, t+1 as R, G, B, clamped at the ends of the scenario. cv2 writes BGR, hence the order.
            png = np.stack([warped(min(len(stems) - 1, i + 1)), img, warped(max(0, i - 1))], -1)
        else:
            png = img
        h, w = img.shape[:2]
        name = f"{sc['id']}_{stem}.png"
        image_id = idx * 1000 + int(meta["seq"])            # unique: 90 pings per scenario
        master = out / "images" / sc["split"] / name
        cv2.imwrite(str(master), png, [cv2.IMWRITE_PNG_COMPRESSION, cfg.png_level])
        son, leak = meta["sonar"], (meta.get("leak") or {})
        # What one Ultralytics supervision pixel costs: the long side goes to imgsz and the mask is rasterised
        # at imgsz/mask_ratio, so a plume of a few hundred native pixels survives as a handful.
        shrink = (cfg.imgsz / max(h, w)) ** 2 / (cfg.mask_ratio ** 2)
        row = {"image": name, "image_id": image_id, "split": sc["split"], "scenario": sc["id"],
               "kind": sc["kind"], "stem": stem, "seq": meta["seq"], "stamp": meta["stamp"],
               "source_png": f"{sc['id']}/{stem}.png", "source_npz": f"{sc['id']}/{stem}.npz",
               "size_preset": sc["params"].get("size"), "mode": son["mode"], "range_m": son["range"],
               "frequency_hz": son["frequency_hz"], "width": w, "height": h,
               "polar_bins": a["image"].shape[0], "polar_beams": a["image"].shape[1],
               "noise_floor_db": round(float(son["noise_floor_db"]), 3),
               "display_db_lo": son["display_db"][0], "display_db_hi": son["display_db"][1],
               "beamwidth_h_deg": round(float(son["beamwidth_h_deg"]), 4), "fov_v_deg": son["fov_v_deg"],
               "focus": son["focus"], "pose_source": son["pose_source"],
               "position": ";".join(f"{v:.4f}" for v in son["position"]),
               "roll_deg": son["roll_deg"], "pitch_deg": son["pitch_deg"], "yaw_deg": son["yaw_deg"],
               "stratum": sc.get("stratum", ""), "holdout": sc.get("holdout", False),
               "leak_present": int(bool(leak) and float(leak.get("flow_lpm", 0.0)) > 0.0),
               "flow_lpm": leak.get("flow_lpm", ""), "leak_range_m": leak.get("range_m", ""),
               "leak_bearing_deg": leak.get("bearing_deg", ""),
               "leak_elevation_deg": leak.get("elevation_deg", ""),
               "plume_pixels_geom": meta["plume_pixels"]["geom"],
               "plume_pixels_direct": meta["plume_pixels"]["direct"],
               "plume_pixels_all": meta["plume_pixels"]["all"]}
        for m in cfg.masks:
            suffix = "" if m == cfg.masks[0] else f"_mask_{m}"
            polys, audit = instances(msk[m], img, cfg)
            row |= {f"{m}_{k}": v for k, v in audit.items()}
            row[f"{m}_supervision_px"] = round(audit["instance_px"] * shrink, 2)
            if cfg.format in ("yolo", "both"):
                yroot = out / f"dataset_yolo{suffix}"
                link(master, yroot / "images" / sc["split"] / name)
                lines = [yolo_line(p, w, h) for p in polys]
                # A negative gets an empty file, not a missing one: Ultralytics counts both as background, but
                # only a present file keeps its "N images, M backgrounds" tally meaningful and makes a missing
                # label a real bug rather than a silent background.
                (yroot / "labels" / sc["split"] / f"{name[:-4]}.txt").write_text(
                    "\n".join(lines) + "\n" if lines else "")
            if cfg.format in ("coco", "both"):
                link(master, out / f"dataset_coco{suffix}" / COCO_SPLIT_DIR[sc["split"]] / name)
                frag[m]["images"].append({"id": image_id, "file_name": name, "width": w, "height": h})
                frag[m]["annotations"] += [coco_annotation(p, image_id) for p in polys]
        rows.append(row)
        cache.pop(i - 1, None)
    for m in cfg.masks:
        if cfg.format in ("coco", "both"):
            (out / ".scn" / f"{sc['id']}_{m}.json").write_text(json.dumps(frag[m]))
    marker.write_text(json.dumps({"signature": sig, "split": sc["split"], "rows": rows}))
    return rows


# ------------------------------------------------------------------------------------ writing the datasets

def splits_of(manifest: dict, cfg) -> dict:
    """scenario id -> split. The manifest's own split by default: it is stratified by kind, it shipped with the
    dataset, and reusing it keeps anything trained here comparable with the rest of the repo."""
    scen = [s for s in manifest["scenarios"] if s.get("status") == "done"]
    if cfg.split == "manifest" and not cfg.val_sessions:
        return {s["id"]: s["split"] for s in scen}
    ids = sorted(s["id"] for s in scen)
    if cfg.val_sessions:
        unknown = set(cfg.val_sessions) - set(ids)
        if unknown:
            sys.exit(f"--val-sessions names scenarios not in the manifest: {sorted(unknown)[:5]}")
        return {i: ("val" if i in set(cfg.val_sessions) else "train") for i in ids}
    perm = np.random.default_rng(cfg.seed).permutation(len(ids))
    val = {ids[j] for j in perm[:int(round(cfg.val_ratio * len(ids)))]}
    return {i: ("val" if i in val else "train") for i in ids}


def dataset_dirs(cfg) -> list:
    out = []
    for m in cfg.masks:
        suffix = "" if m == cfg.masks[0] else f"_mask_{m}"
        if cfg.format in ("yolo", "both"):
            out.append(f"dataset_yolo{suffix}")
        if cfg.format in ("coco", "both"):
            out.append(f"dataset_coco{suffix}")
    return out


def link(src: Path, dst: Path) -> None:
    """Hardlink if we can -- no extra bytes, and both loaders open the path normally -- else symlink, else copy.
    Only the files are linked; the split directories are real, see the module docstring."""
    if dst.exists():
        dst.unlink()
    try:
        os.link(src, dst)
    except OSError:
        try:
            dst.symlink_to(os.path.relpath(src, dst.parent))
        except OSError:
            shutil.copy2(src, dst)


def prepare_out(out: Path, cfg, used: list) -> None:
    """Make every directory the workers will write into. Resumable, so nothing is deleted unless --fresh: a
    scenario marker whose signature no longer matches is simply redone over the top of itself."""
    if out.exists() and any(out.iterdir()) and not (out / "meta" / MARKER).exists() and not cfg.fresh:
        sys.exit(f"{out} is not empty and carries no meta/{MARKER}: refusing to overwrite someone else's files")
    if cfg.fresh:
        for p in [out / "images", out / "meta", out / ".scn"] + [out / d for d in dataset_dirs(cfg)]:
            if p.exists():
                shutil.rmtree(p)
    for p in (out / "meta", out / ".scn"):
        p.mkdir(parents=True, exist_ok=True)
    for s in used:
        (out / "images" / s).mkdir(parents=True, exist_ok=True)
    for m in cfg.masks:
        suffix = "" if m == cfg.masks[0] else f"_mask_{m}"
        for s in used:
            if cfg.format in ("yolo", "both"):
                (out / f"dataset_yolo{suffix}" / "images" / s).mkdir(parents=True, exist_ok=True)
                (out / f"dataset_yolo{suffix}" / "labels" / s).mkdir(parents=True, exist_ok=True)
            if cfg.format in ("coco", "both"):
                (out / f"dataset_coco{suffix}" / COCO_SPLIT_DIR[s]).mkdir(parents=True, exist_ok=True)


def write_data_yaml(out: Path, name: str, mask: str, cfg, used: list) -> None:
    root = out / name
    yaml = [f"# {name}: label mask_{mask}, one class, written by tools/sonar_to_yolo.py of underwater_ws.",
            "# Simulated forward-looking sonar -- read docs/INSPECTION.md before training on it.",
            f"path: {root}", "train: images/train", "val: images/val"]
    if "test" in used:
        yaml.append("test: images/test")
    root.mkdir(parents=True, exist_ok=True)
    (root / "data.yaml").write_text("\n".join(yaml + ["names:", f"  0: {cfg.class_name}", ""]))


def write_coco(out: Path, name: str, mask: str, cfg, ids: list, split: str, info: str) -> tuple:
    """Concatenate the per-scenario fragments into one _annotations.coco.json, streaming so the whole file is
    never in memory at once: a train split of this dataset holds six figures of polygons.

    One flat category at id 1, no synthetic root. RF-DETR remaps ids to contiguous labels and drops a parent
    category that carries no annotations, so a Roboflow-style {id 0 root, id 1 class} pair is version
    dependent -- before its 1.9.2 category filter it trains a 2-class head. A single flat entry is identical
    either side of that change."""
    cats = [{"id": CLASS_ID_COCO, "name": cfg.class_name, "supercategory": "none"}]
    path = out / name / COCO_SPLIT_DIR[split] / "_annotations.coco.json"
    n_img = n_ann = 0
    with open(path, "w") as f:
        f.write('{"info": ' + json.dumps({"description": info, "version": "0.2", "year": 2026,
                                          "contributor": "simulated, see docs/INSPECTION.md"}))
        f.write(', "licenses": [], "categories": ' + json.dumps(cats) + ', "images": [')
        anns = []
        for sid in ids:
            frag = json.loads((out / ".scn" / f"{sid}_{mask}.json").read_text())
            for im in frag["images"]:
                f.write(("," if n_img else "") + json.dumps(im))
                n_img += 1
            anns.append(frag["annotations"])
        f.write('], "annotations": [')
        for group in anns:
            for a in group:
                n_ann += 1
                f.write(("," if n_ann > 1 else "") + json.dumps(dict(id=n_ann, **a)))
        f.write("]}")
    return n_img, n_ann


def write_meta(out: Path, rows: list, cfg, used: list, timing: dict, coco_counts: dict) -> dict:
    cols = list(rows[0])
    with open(out / "meta" / "frames.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    stats = {"source": cfg.source_info, "frames": len(rows), "representation": cfg.repr,
             "stack3": bool(cfg.stack3), "class_names": {0: cfg.class_name},
             "coco_category_id": CLASS_ID_COCO, "coco_image_count": coco_counts,
             "timing_s": timing, "splits": {}, "masks": {},
             "note": "simulated dataset, no real sonar; see docs/INSPECTION.md"}
    for s in used:
        rs = [r for r in rows if r["split"] == s]
        stats["splits"][s] = {"frames": len(rs), "scenarios": len({r["scenario"] for r in rs}),
                              "kinds": {k: sum(1 for r in rs if r["kind"] == k)
                                        for k in sorted({r["kind"] for r in rs})},
                              "modes": {k: sum(1 for r in rs if r["mode"] == k)
                                        for k in sorted({r["mode"] for r in rs})}}
    for m in cfg.masks:
        per = {}
        for s in used:
            rs = [r for r in rows if r["split"] == s]
            inst = [r[f"{m}_instances"] for r in rs]
            sup = [r[f"{m}_supervision_px"] for r in rs if r[f"{m}_instances"] > 0]
            per[s] = {"frames": len(rs), "instances": int(sum(inst)),
                      "frames_with_bubbles": sum(1 for v in inst if v > 0),
                      "frames_without_bubbles": sum(1 for v in inst if v == 0),
                      "instances_per_positive_frame": round(sum(inst) / max(sum(1 for v in inst if v > 0), 1), 3),
                      "max_instances_in_a_frame": max(inst) if inst else 0,
                      "dropped_components": int(sum(r[f"{m}_components_raw"] - r[f"{m}_components_kept"]
                                                    for r in rs)),
                      "dropped_px": int(sum(r[f"{m}_dropped_px"] for r in rs)),
                      "mask_px": int(sum(r[f"{m}_mask_px"] for r in rs)),
                      "instance_px": int(sum(r[f"{m}_instance_px"] for r in rs)),
                      "labelled_area_over_mask": round(sum(r[f"{m}_instance_px"] for r in rs) /
                                                       max(sum(r[f"{m}_mask_px"] for r in rs), 1), 3),
                      f"positives_under_10_supervision_px_at_imgsz_{cfg.imgsz}":
                          sum(1 for v in sup if v < 10.0)}
        stats["masks"][m] = {"per_split": per}
    (out / "meta" / "stats.json").write_text(json.dumps(stats, indent=1))
    return stats


# ------------------------------------------------------------------------------------ previews

def draw(img: np.ndarray, polys: list, colour=(60, 255, 60)) -> np.ndarray:
    v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
    cv2.drawContours(v, [np.array(p, np.int32) for p in polys], -1, colour, 1)
    return v


def previews(out: Path, rows: list, cfg) -> dict:
    """Overlays to look at. `preview*/` is drawn from the YOLO label files and `preview_coco*/` from the COCO
    json of the same frames, so what gets checked is what was written, not what was in memory."""
    rng = random.Random(cfg.seed)
    made = {}
    for m in cfg.masks:
        suffix = "" if m == cfg.masks[0] else f"_mask_{m}"
        pos = [r for r in rows if r[f"{m}_instances"] > 0]
        neg = [r for r in rows if r[f"{m}_instances"] == 0]
        chosen = (rng.sample(pos, min(cfg.preview - cfg.preview // 4, len(pos)))
                  + rng.sample(neg, min(cfg.preview // 4, len(neg))))
        if cfg.format in ("yolo", "both"):
            pdir = out / "meta" / f"preview{suffix}"
            pdir.mkdir(parents=True, exist_ok=True)
            for r in chosen:
                img = read_grey(out / "images" / r["split"] / r["image"])
                txt = (out / f"dataset_yolo{suffix}" / "labels" / r["split"] /
                       f"{r['image'][:-4]}.txt").read_text()
                polys = [(np.array(line.split()[1:], float).reshape(-1, 2)
                          * np.array([r["width"], r["height"]])).astype(np.int32)
                         for line in txt.split("\n") if len(line.split()) > 6]
                v = draw(img, polys)
                cv2.putText(v, f"{r['scenario']} {r['stem']} {r['mode']} mask_{m} {len(polys)} inst", (4, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 220, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(pdir / r["image"]), v)
            made[f"preview{suffix}"] = len(chosen)
        if cfg.format in ("coco", "both"):
            cdir = out / "meta" / f"preview_coco{suffix}"
            cdir.mkdir(parents=True, exist_ok=True)
            take = chosen[:max(1, cfg.preview // 2)]
            byid = {}
            for s in {r["split"] for r in take}:
                j = json.loads((out / f"dataset_coco{suffix}" / COCO_SPLIT_DIR[s] /
                                "_annotations.coco.json").read_text())
                for a in j["annotations"]:
                    byid.setdefault(a["image_id"], []).append(a)
            for r in take:
                polys = [np.array(a["segmentation"][0], float).reshape(-1, 2).astype(np.int32)
                         for a in byid.get(r["image_id"], [])]
                img = read_grey(out / f"dataset_coco{suffix}" / COCO_SPLIT_DIR[r["split"]] / r["image"])
                v = draw(img, polys, (255, 160, 60))
                cv2.putText(v, f"COCO {r['scenario']} {r['stem']} mask_{m} {len(polys)} ann", (4, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 220, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(cdir / r["image"]), v)
            made[f"preview_coco{suffix}"] = len(take)
    return made


# ------------------------------------------------------------------------------------ the run

def scenarios_of(root: Path, cfg) -> tuple:
    manifest = json.loads((root / "manifest.json").read_text())
    split_of = splits_of(manifest, cfg)
    scen = [dict(s, split=split_of[s["id"]]) for s in manifest["scenarios"]
            if s.get("status") == "done" and s["id"] in split_of]
    scen.sort(key=lambda s: s["id"])
    return manifest, (scen[:cfg.limit] if cfg.limit else scen)


def run(cfg) -> int:
    root, out = Path(cfg.root), Path(cfg.out)
    manifest, scen = scenarios_of(root, cfg)
    used = [s for s in SPLITS if any(x["split"] == s for x in scen)]
    cfg.source_info = {"root": str(root), "name": manifest.get("name"), "git": manifest.get("git"),
                       "created": manifest.get("created"), "scenarios": len(scen), "simulated": True,
                       "config_hash": manifest["scenarios"][0].get("config_hash", ""),
                       "plume_mask_threshold": manifest.get("labels", {}).get("threshold"),
                       # a later in-place rewrite of the source replaces files with os.replace, which breaks
                       # our hardlinks: this count makes that staleness detectable afterwards
                       "recompressed_markers": sum((root / s["id"] / "RECOMPRESSED").exists() for s in scen),
                       "signature": signature(cfg)}
    if cfg.dry_run:
        print(json.dumps({"scenarios": len(scen), "out": str(out), "datasets": dataset_dirs(cfg),
                          "signature": signature(cfg),
                          "splits": {s: sum(1 for x in scen if x["split"] == s) for s in used}}, indent=1))
        return 0
    prepare_out(out, cfg, used)
    t0, rows = time.time(), []
    jobs = [(root, out, s, i, cfg) for i, s in enumerate(scen)]
    with mp.Pool(cfg.workers) as pool:
        for n, rs in enumerate(pool.imap_unordered(scenario_rows, jobs), 1):
            rows += rs
            el = time.time() - t0
            print(f"{time.strftime('%H:%M:%S')} {n}/{len(scen)} {rs[0]['scenario']} {len(rs)} frames, "
                  f"{len(rows)} total, ETA {el / n * (len(scen) - n) / 60:.1f} min", flush=True)
    rows.sort(key=lambda r: (r["scenario"], r["seq"]))
    t_convert = time.time() - t0
    coco_counts = {}
    for m in cfg.masks:
        suffix = "" if m == cfg.masks[0] else f"_mask_{m}"
        if cfg.format in ("yolo", "both"):
            write_data_yaml(out, f"dataset_yolo{suffix}", m, cfg, used)
        if cfg.format in ("coco", "both"):
            for s in used:
                ids = [x["id"] for x in scen if x["split"] == s]
                n_img, n_ann = write_coco(out, f"dataset_coco{suffix}", m, cfg, ids, s,
                                          f"underwater_ws {manifest.get('name')} mask_{m} {s}, simulated "
                                          f"forward-looking sonar")
                coco_counts[f"dataset_coco{suffix}/{COCO_SPLIT_DIR[s]}"] = {"images": n_img,
                                                                            "annotations": n_ann}
    stats = write_meta(out, rows, cfg, used, {"convert": round(t_convert, 1),
                                              "write": round(time.time() - t0 - t_convert, 1)}, coco_counts)
    stats["previews"] = previews(out, rows, cfg)
    (out / "meta" / "stats.json").write_text(json.dumps(stats, indent=1))
    (out / "meta" / MARKER).write_text(json.dumps(
        {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(cfg).items()}, indent=1, default=str))
    (out / "README.md").write_text(
        f"# {out.name}\n\nSimulated forward-looking-sonar dataset, converted from `{root}` by\n"
        f"`tools/sonar_to_yolo.py` of underwater_ws. **Simulated data, no real sonar.** Read\n"
        f"`docs/INSPECTION.md` in that repo before training on it, in particular the mask choice, the\n"
        f"aspect-ratio note and the sim-to-real note.\n\n"
        f"- `dataset_yolo*/` Ultralytics YOLO-seg, `dataset_coco*/` RF-DETR COCO (`valid`, not `val`)\n"
        f"- `images/` the single copy of the pixels; every dataset directory hardlinks them\n"
        f"- `meta/frames.csv`, `meta/stats.json`, `meta/params.json`, `meta/preview*/`\n"
        f"- `.scn/` per-scenario resume markers and COCO fragments; safe to delete once converted\n")
    print(json.dumps({"frames": stats["frames"], "splits": stats["splits"],
                      "previews": stats["previews"]}, indent=1), flush=True)
    return 0


def status(cfg) -> int:
    out = Path(cfg.out)
    sig = signature(cfg) if cfg.root else None
    done = sorted(p.stem for p in (out / ".scn").glob("s[0-9]*.json") if "_" not in p.stem)
    fresh = [s for s in done if json.loads((out / ".scn" / f"{s}.json").read_text()).get("signature") == sig]
    rep = {"out": str(out), "scenarios_converted": len(done), "matching_current_parameters": len(fresh)}
    if cfg.root:
        _, scen = scenarios_of(Path(cfg.root), cfg)
        rep |= {"scenarios_in_manifest": len(scen), "signature": sig,
                "todo": [s["id"] for s in scen if s["id"] not in set(fresh)][:10]}
    print(json.dumps(rep, indent=1))
    return 0


# ------------------------------------------------------------------------------------ verification

def check(cfg) -> int:
    """Structural verification of a written output. Pure python and stdlib + cv2: the Ultralytics and
    pycocotools checks need their own venvs and live in tools/run_sonar_to_yolo.sh. Every failure is
    collected, nothing stops at the first."""
    out = Path(cfg.out)
    bad, warn = [], []
    params = json.loads((out / "meta" / MARKER).read_text())
    masks, fmt = params["masks"], params["format"]
    rows = list(csv.DictReader(open(out / "meta" / "frames.csv")))
    print(f"frames.csv: {len(rows)} rows, masks {masks}, repr {params['repr']}, stack3 {params['stack3']}")

    by_sc = {}
    for r in rows:
        by_sc.setdefault(r["scenario"], set()).add(r["split"])
    leaked = {k: sorted(v) for k, v in by_sc.items() if len(v) > 1}
    if leaked:
        bad.append(f"{len(leaked)} scenarios in more than one split: {list(leaked.items())[:3]}")
    print(f"scenarios: {len(by_sc)}, none in two splits: {not leaked}")

    dims = {r["image"]: (int(r["width"]), int(r["height"])) for r in rows}
    ids = [int(r["image_id"]) for r in rows]
    if len(ids) != len(set(ids)):
        bad.append("image_id is not unique across frames.csv")
    used = sorted({r["split"] for r in rows})

    if cfg.root:
        # the stdlib decoder the training loader uses, against the cv2 fast path this tool uses
        import uw.sinks as sinks
        rng = random.Random(0)
        for r in rng.sample(rows, min(8, len(rows))):
            p = Path(cfg.root) / r["source_png"]
            if not np.array_equal(sinks.read_png(p), cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)):
                bad.append(f"cv2 and uw.sinks.read_png disagree on {p}")
        print(f"decoder cross-check on 8 source PNGs: {'ok' if not bad else 'FAILED'}")

    for m in masks:
        suffix = "" if m == masks[0] else f"_mask_{m}"
        counts = {}
        if fmt in ("yolo", "both"):
            yroot = out / f"dataset_yolo{suffix}"
            n_img = n_inst = n_neg = 0
            for s in used:
                imgs = sorted(p.name for p in (yroot / "images" / s).glob("*.png"))
                labs = {p.stem for p in (yroot / "labels" / s).glob("*.txt")}
                n_img += len(imgs)
                missing = [i for i in imgs if i[:-4] not in labs]
                orphan = [t for t in labs if f"{t}.png" not in set(imgs)]
                if missing:
                    bad.append(f"{yroot.name}/{s}: {len(missing)} images without a label, e.g. {missing[:3]}")
                if orphan:
                    bad.append(f"{yroot.name}/{s}: {len(orphan)} labels without an image, e.g. {orphan[:3]}")
                for name in imgs:
                    w, h = dims[name]
                    polys, boxes = [], []
                    for line in (yroot / "labels" / s / f"{name[:-4]}.txt").read_text().split("\n"):
                        tok = line.split()
                        if not tok:
                            continue
                        if tok[0] != "0":
                            bad.append(f"{yroot.name}/{s}/{name}: class {tok[0]!r}, expected 0")
                        if len(tok) <= 6:
                            bad.append(f"{yroot.name}/{s}/{name}: {len(tok) // 2} points, fewer than 3")
                            continue
                        if (len(tok) - 1) % 2:
                            bad.append(f"{yroot.name}/{s}/{name}: odd coordinate count {len(tok) - 1}")
                            continue
                        v = np.array(tok[1:], float)
                        if v.min() < 0.0 or v.max() > 1.0:
                            bad.append(f"{yroot.name}/{s}/{name}: coordinate outside [0,1] "
                                       f"({v.min():.4f}..{v.max():.4f})")
                        # rint, not truncation: the six-decimal round trip can land a vertex at
                        # 136.9997, and int() would move it a whole pixel
                        xy = np.rint(v.reshape(-1, 2) * np.array([w, h])).astype(np.int32)
                        polys.append(xy)
                        boxes.append(tuple(cv2.boundingRect(xy)))
                    n_inst += len(polys)
                    n_neg += (len(polys) == 0)
                    # Ultralytics deduplicates on the (class, xywh) row, so two distinct polygons sharing a
                    # box silently lose one of the pair.
                    if len(set(boxes)) != len(boxes):
                        warn.append(f"{yroot.name}/{s}/{name}: {len(boxes) - len(set(boxes))} polygons share "
                                    f"a bounding box; Ultralytics drops one of each pair")
                    counts[name] = (len(polys), sum(poly_area(p) for p in polys))
            data = (yroot / "data.yaml").read_text()
            for key in ("path:", "train: images/train", "val: images/val"):
                if key not in data:
                    bad.append(f"{yroot.name}/data.yaml is missing {key!r}")
            if not data.split("path:")[1].split("\n")[0].strip().startswith("/"):
                bad.append(f"{yroot.name}/data.yaml: path is not absolute, so Ultralytics would resolve it "
                           f"against its own datasets_dir")
            print(f"{yroot.name}: {n_img} images, {n_inst} instances, {n_neg} negatives")

        if fmt in ("coco", "both"):
            croot = out / f"dataset_coco{suffix}"
            for s in used:
                j = json.loads((croot / COCO_SPLIT_DIR[s] / "_annotations.coco.json").read_text())
                iids = [i["id"] for i in j["images"]]
                aids = [a["id"] for a in j["annotations"]]
                if len(iids) != len(set(iids)):
                    bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: duplicate image ids")
                if len(aids) != len(set(aids)):
                    bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: duplicate annotation ids")
                cats = {c["id"] for c in j["categories"]}
                if cats != {CLASS_ID_COCO}:
                    bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: categories {sorted(cats)}, expected "
                               f"[{CLASS_ID_COCO}]")
                known, per_img = set(iids), {}
                for a in j["annotations"]:
                    if a["image_id"] not in known:
                        bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: annotation {a['id']} refers to the "
                                   f"missing image {a['image_id']}")
                    for key in ("area", "bbox", "iscrowd", "segmentation"):
                        if key not in a:
                            bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: annotation {a['id']} has no {key}")
                    if a.get("iscrowd") != 0:
                        bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: annotation {a['id']} iscrowd="
                                   f"{a.get('iscrowd')}, which RF-DETR drops from supervision")
                    bw, bh = a["bbox"][2], a["bbox"][3]
                    if not (bw > 0 and bh > 0 and a["area"] > 0):
                        bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: annotation {a['id']} is degenerate: "
                                   f"bbox={a['bbox']} area={a['area']}")
                    for k, poly in enumerate(a["segmentation"]):
                        if len(poly) % 2 or len(poly) < 6 or len(poly) == 4:
                            bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: annotation {a['id']} polygon {k} "
                                       f"has {len(poly)} values; pycocotools needs an even count >= 6, and "
                                       f"never exactly 4")
                    e = per_img.setdefault(a["image_id"], [0, 0.0])
                    e[0] += 1
                    e[1] += a["area"]
                names = {i["file_name"] for i in j["images"]}
                on_disk = {p.name for p in (croot / COCO_SPLIT_DIR[s]).glob("*.png")}
                if names != on_disk:
                    bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: {len(names ^ on_disk)} images are in the "
                               f"json but not on disk, or the other way round")
                for i in j["images"]:
                    if (i["width"], i["height"]) != dims[i["file_name"]]:
                        bad.append(f"{croot.name}/{COCO_SPLIT_DIR[s]}: {i['file_name']} is "
                                   f"{i['width']}x{i['height']} in the json but {dims[i['file_name']]} in "
                                   f"frames.csv")
                if counts:          # both formats come from the same polygons, so both must agree
                    for i in j["images"]:
                        n_y, a_y = counts.get(i["file_name"], (0, 0.0))
                        n_c, a_c = per_img.get(i["id"], [0, 0.0])
                        if n_y != n_c:
                            bad.append(f"{i['file_name']}: {n_y} YOLO instances but {n_c} COCO annotations")
                        elif n_y and abs(a_y - a_c) > max(1.0, 0.01 * a_c):
                            bad.append(f"{i['file_name']}: YOLO area {a_y:.1f} vs COCO area {a_c:.1f}")
                print(f"{croot.name}/{COCO_SPLIT_DIR[s]}: {len(iids)} images, {len(aids)} annotations, "
                      f"{len(iids) - len(per_img)} negatives")

    for p in sorted((out / "meta").glob("preview*")):
        print(f"{p.relative_to(out)}: {len(list(p.glob('*.png')))} overlays")
    for w_ in warn[:10]:
        print(f"WARN {w_}")
    if len(warn) > 10:
        print(f"WARN ... and {len(warn) - 10} more")
    for b in bad[:20]:
        print(f"FAIL {b}")
    if len(bad) > 20:
        print(f"FAIL ... and {len(bad) - 20} more")
    print(f"\n{len(bad)} failures, {len(warn)} warnings")
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", nargs="?", default="run", choices=("run", "check", "status"))
    ap.add_argument("--root", help="sweep dataset root (the directory with manifest.json)")
    ap.add_argument("--out", required=True, help="output directory; put it on /HDD2T, v0.2 needs ~20 GB")
    ap.add_argument("--repr", default="polar", choices=("polar", "fan", "echogram"),
                    help="polar (range x beam: what the sonar produces, and the default), fan (Cartesian, the "
                         "viewer's picture, which throws away a quarter of the basin's range cells at width "
                         "1024), echogram (not applicable, see docs/INSPECTION.md)")
    ap.add_argument("--mask", nargs="+", default=["geom", "direct", "all"], choices=MASKS,
                    help="label mask(s). The first owns dataset_yolo/ and dataset_coco/, the rest get a "
                         "_mask_<name> suffix. geom leads because direct and all mark energy ownership "
                         "rather than plume extent, and span most of the fan; see docs/INSPECTION.md")
    ap.add_argument("--format", default="both", choices=("yolo", "coco", "both"))
    ap.add_argument("--class-name", default="bubbles", help="the one class name, in data.yaml and in COCO")
    ap.add_argument("--stack3", action="store_true", help="t-1, t, t+1 of the scenario as R, G, B")
    ap.add_argument("--db-range", nargs=2, type=float, metavar=("LO", "HI"),
                    help="rebuild the image from the npz polar_db with this fixed dB window instead of the "
                         "stored 8-bit display image, which already carries TVG and the 0-80 dB window")
    ap.add_argument("--db-noise-floor", action="store_true",
                    help="with --db-range, window polar_db minus this ping's noise floor, as uw/dataset.py's "
                         "\"db\" image does. Per ping, so it is not the fixed normalisation --db-range alone is")
    ap.add_argument("--level-gate", type=int, default=0,
                    help="keep only mask pixels whose display level exceeds this (0-255; 0 off)")
    ap.add_argument("--split", default="manifest", choices=("manifest", "ratio"))
    ap.add_argument("--val-sessions", default="", help="comma-separated scenario ids for val (overrides --split)")
    ap.add_argument("--val-ratio", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--close", type=int, default=3,
                    help="odd kernel for the morphological close (1 off). 3 measured over 200 positive frames: "
                         "4 instances per frame and 1.18x the mask area, where 5 gives 1.61x and 7 gives 1.98x")
    ap.add_argument("--min-area", type=int, default=64, help="drop components below this many pixels")
    ap.add_argument("--no-fill-holes", dest="fill_holes", action="store_false")
    ap.add_argument("--simplify", type=float, default=0.004,
                    help="approxPolyDP epsilon as a fraction of the contour perimeter")
    ap.add_argument("--max-points", type=int, default=200, help="vertex cap per polygon")
    ap.add_argument("--imgsz", type=int, default=640, help="only for the frames.csv estimate of how many "
                                                           "supervision pixels an instance keeps after resize")
    ap.add_argument("--mask-ratio", type=int, default=4, help="likewise: Ultralytics' mask_ratio")
    ap.add_argument("--fan-width", type=int, default=1024)
    ap.add_argument("--png-level", type=int, default=3, help="zlib level for the written PNGs")
    ap.add_argument("--workers", type=int, default=4,
                    help="4 is the measured sweet spot on linus: 8 cores, but under 3 GB free and one spindle")
    ap.add_argument("--limit", type=int, default=0, help="stop after this many scenarios (0 all)")
    ap.add_argument("--preview", type=int, default=20)
    ap.add_argument("--fresh", action="store_true", help="delete the output and convert from scratch")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if a.repr == "echogram":
        sys.exit("--repr echogram does not apply here: the sweep records a 512-beam forward-looking imaging "
                 "sonar, not a single-beam echosounder, so there is no time x depth echogram to window. Use "
                 "--repr polar (range x beam) or --repr fan. See docs/INSPECTION.md.")
    if a.close > 1 and a.close % 2 == 0:
        sys.exit("--close must be odd")
    if a.db_noise_floor and not a.db_range:
        sys.exit("--db-noise-floor only means something with --db-range")
    a.masks = tuple(dict.fromkeys(a.mask))          # de-duplicated, order kept: the first owns dataset_yolo/
    a.val_sessions = [s for s in a.val_sessions.split(",") if s]
    a.source_info = {}
    if a.cmd == "check":
        return check(a)
    if a.cmd == "status":
        return status(a)
    if not a.root:
        sys.exit("run needs --root")
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
