"""Videos of a recording run (tools/record.py): camera, sonar (as a fan) and the two side by side.

    python tools/record_video.py output/records/test_combined_20s
    python tools/record_video.py output/records/<run_id> --fps 10 --fan-width 720 --no-right

Writes into <run>/video/: camera_left.mp4, camera_right.mp4 (if recorded), sonar_fan.mp4, combined.mp4 (left camera
| sonar fan, stamped with sim time and the ROV position from poses.csv). One video frame per recorded 10 Hz frame, so
--fps 10 plays at simulated real time. OpenCV only (bundled with the Isaac env): codec mp4v, or MJPG .avi if mp4v is
unavailable. The fan is rebuilt from the polar PNG with the per-row range and per-column bearing of the ping's npz.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def stamps(run: Path, sensor: str) -> dict:
    m = json.loads((run / "manifest.json").read_text())
    return {e["seq"]: float(e["stamp"]) for e in m.get("sensors", {}).get(sensor, {}).get("messages", [])}


class Fan:
    """Polar (rows: range, max range at row 0 as written; cols: bearing) -> Cartesian fan, apex at the bottom."""

    def __init__(self, range_m: np.ndarray, bearing_deg: np.ndarray, width: int):
        r = np.asarray(range_m, float)
        b = np.radians(np.asarray(bearing_deg, float))
        self.rmax, half = float(r.max()), float(np.abs(b).max())
        h = int(round(width / 2 / math.sin(half))) if half < math.pi / 2 else width // 2
        h = min(h, int(width * 1.2))
        scale = h / self.rmax                                   # px per metre
        xs = (np.arange(width) - width / 2 + 0.5) / scale        # metres, +x to the right (starboard)
        ys = (h - np.arange(h) - 0.5) / scale                    # metres ahead
        X, Y = np.meshgrid(xs, ys)
        R, B = np.hypot(X, Y), np.arctan2(X, Y)                  # bearing positive to starboard
        # row index: the PNG is flipped (row 0 = far); bearing columns may run either way
        rows_r = r[::-1]
        ri = np.interp(R, rows_r[::-1], np.arange(len(r))[::-1]) if rows_r[0] > rows_r[-1] else \
            np.interp(R, rows_r, np.arange(len(r)))
        bb = -b                                                  # repo bearing: positive to port
        order = np.argsort(bb)
        ci = np.interp(B, bb[order], np.arange(len(b))[order].astype(float))
        valid = (R <= r.max()) & (R >= r.min()) & (B >= bb.min()) & (B <= bb.max())
        self.mapx = np.where(valid, ci, -1).astype(np.float32)
        self.mapy = np.where(valid, ri, -1).astype(np.float32)
        self.size, self.scale = (width, h), scale

    def __call__(self, polar_u8: np.ndarray) -> np.ndarray:
        g = cv2.remap(polar_u8, self.mapx, self.mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        img = cv2.applyColorMap(g, cv2.COLORMAP_INFERNO)
        img[self.mapx < 0] = (25, 25, 25)
        w, h = self.size
        for m in range(1, int(self.rmax) + 1):                   # range rings every metre
            cv2.ellipse(img, (w // 2, h), (int(m * self.scale),) * 2, 0, 180, 360, (90, 90, 90), 1, cv2.LINE_AA)
            cv2.putText(img, f"{m} m", (w // 2 + 4, h - int(m * self.scale) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (160, 160, 160), 1, cv2.LINE_AA)
        return img


def writer(path: Path, fps: float, size) -> tuple[cv2.VideoWriter, Path]:
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if w.isOpened():
        return w, path
    p = path.with_suffix(".avi")
    return cv2.VideoWriter(str(p), cv2.VideoWriter_fourcc(*"MJPG"), fps, size), p


def label(img, text, y=24):
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", help="a run directory under output/records")
    ap.add_argument("--fps", type=float, default=10.0, help="10 = simulated real time (frames are 10 Hz)")
    ap.add_argument("--fan-width", type=int, default=720)
    ap.add_argument("--no-right", action="store_true", help="skip camera_right.mp4")
    a = ap.parse_args()
    return make_videos(Path(a.run), a.fps, a.fan_width, not a.no_right)


def make_videos(run: Path, fps: float = 10.0, fan_width: int = 720, right_video: bool = True) -> int:
    """The videos of one run into <run>/video/ (also called by tools/record_report.py at the end of a batch)."""
    run = Path(run)

    class _A:                                  # the CLI's names, so the body below is unchanged
        pass
    a = _A()
    a.fps, a.fan_width, a.no_right = fps, fan_width, not right_video
    out = run / "video"
    out.mkdir(exist_ok=True)
    left, right, son = stamps(run, "camera"), stamps(run, "camera_right"), stamps(run, "sonar")
    if not left or not son:
        print(f"[video] {run}: no camera or sonar frames")
        return 1
    poses = {}
    if (run / "poses.csv").is_file():
        with open(run / "poses.csv", newline="") as fh:
            poses = {int(r["seq"]): r for r in csv.DictReader(fh)}
    s_seqs = np.array(sorted(son))
    s_t = np.array([son[k] for k in s_seqs])
    z = np.load(run / "sonar" / f"{s_seqs[0]:06d}.npz")
    fan = Fan(z["range_m"], z["bearing_deg"], a.fan_width)

    cam0 = cv2.imread(str(run / "camera" / f"{min(left):06d}_rgb.png"))
    ch, cw = cam0.shape[:2]
    fw, fh = fan.size
    # combined: camera scaled to the fan's height, side by side
    cs = fh / ch
    comb_size = (int(cw * cs) + fw, fh)
    vids = {"camera_left": writer(out / "camera_left.mp4", a.fps, (cw, ch)),
            "sonar_fan": writer(out / "sonar_fan.mp4", a.fps, (fw, fh)),
            "combined": writer(out / "combined.mp4", a.fps, comb_size)}
    if right and not a.no_right:
        vids["camera_right"] = writer(out / "camera_right.mp4", a.fps, (cw, ch))
    n = 0
    for seq in sorted(left):
        t = left[seq]
        cam = cv2.imread(str(run / "camera" / f"{seq:06d}_rgb.png"))
        if cam is None:
            continue
        sseq = int(s_seqs[np.argmin(np.abs(s_t - t))])
        polar = cv2.imread(str(run / "sonar" / f"{sseq:06d}_polar.png"), cv2.IMREAD_GRAYSCALE)
        f = fan(polar)
        label(f, f"sonar  t {son[sseq]:.1f} s  ping {sseq}")
        p = poses.get(seq)
        pos = f"  ROV ({float(p['rov_x']):.2f}, {float(p['rov_y']):.2f}, {float(p['rov_z']):.2f}) m" \
            if p and p.get("rov_x") else ""
        c = cam.copy()
        label(c, f"camera L  t {t:.1f} s{pos}")
        vids["camera_left"][0].write(c)
        vids["sonar_fan"][0].write(f)
        vids["combined"][0].write(np.hstack([cv2.resize(c, (int(cw * cs), fh), interpolation=cv2.INTER_AREA), f]))
        if "camera_right" in vids:
            r = cv2.imread(str(run / "camera_right" / f"{seq:06d}_rgb.png"))
            if r is not None:
                label(r, f"camera R  t {t:.1f} s")
                vids["camera_right"][0].write(r)
        n += 1
    for w, p in vids.values():
        w.release()
        print(f"[video] {p}  ({p.stat().st_size / 2 ** 20:.1f} MB)")
    print(f"[video] {n} frames at {a.fps:g} fps = {n / a.fps:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
