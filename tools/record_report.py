"""Report of recording runs (tools/record.py): <root>/report.md plus one contact sheet PNG per run.

    python tools/record_report.py                         # every run under output/records
    python tools/record_report.py --runs a_circle a_updown

Per run: exit code (exit_code.txt, written by output/hil/record_all.bat) and the loop's exit_reason, frame counts
(sonar, left, right, ground truth), timestamp drift left/right (same seq) and camera/sonar (nearest ping), RTF and GPU
memory (stats.json), disk use, trajectory notes, and <run_id>/contact_sheet.png: 6 equispaced frames, sonar polar
image (range up) above the left camera image, to see at a glance whether the plume is visible.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]


def stamps(manifest: dict, sensor: str) -> dict:
    return {m["seq"]: float(m["stamp"]) for m in manifest.get("sensors", {}).get(sensor, {}).get("messages", [])}


def drift(run: Path) -> dict:
    mf = run / "manifest.json"
    if not mf.is_file():
        return {}
    m = json.loads(mf.read_text())
    left, right, son = stamps(m, "camera"), stamps(m, "camera_right"), stamps(m, "sonar")
    out = {}
    common = sorted(set(left) & set(right))
    if common:
        out["left_right_max_ms"] = 1e3 * max(abs(left[k] - right[k]) for k in common)
    if left and son:
        s = np.array(sorted(son.values()))
        d = [float(np.min(np.abs(s - t))) for t in left.values()]
        out["camera_sonar_mean_ms"], out["camera_sonar_max_ms"] = 1e3 * float(np.mean(d)), 1e3 * float(np.max(d))
    return out


def contact_sheet(run: Path, n: int = 6, width: int = 480) -> Path | None:
    from PIL import Image, ImageDraw
    cams = sorted((run / "camera").glob("*_rgb.png"))
    if not cams:
        return None
    m = json.loads((run / "manifest.json").read_text()) if (run / "manifest.json").is_file() else {}
    son = stamps(m, "sonar")
    left = stamps(m, "camera")
    idx = np.linspace(0, len(cams) - 1, n).round().astype(int)
    tiles = []
    for i in idx:
        cam = Image.open(cams[i]).convert("RGB")
        cam = cam.resize((width, round(cam.height * width / cam.width)))
        seq = int(cams[i].name.split("_")[0])
        t = left.get(seq)
        sp = None
        if son and t is not None:
            sseq = min(son, key=lambda k: abs(son[k] - t))
            sp = run / "sonar" / f"{sseq:06d}_polar.png"
        if sp is not None and sp.is_file():
            s = Image.open(sp).convert("RGB")
            s = s.resize((width, cam.height * 2))
        else:
            s = Image.new("RGB", (width, cam.height * 2))
        tile = Image.new("RGB", (width, s.height + cam.height + 16), (20, 20, 20))
        tile.paste(s, (0, 16))
        tile.paste(cam, (0, 16 + s.height))
        ImageDraw.Draw(tile).text((4, 2), f"seq {seq}  t {t:.1f} s" if t is not None else f"seq {seq}", fill=(230, 230, 230))
        tiles.append(tile)
    sheet = Image.new("RGB", (sum(t.width for t in tiles) + 4 * (len(tiles) - 1), tiles[0].height), (0, 0, 0))
    x = 0
    for t in tiles:
        sheet.paste(t, (x, 0))
        x += t.width + 4
    out = run / "contact_sheet.png"
    sheet.save(out)
    return out


def summarize(run: Path) -> dict:
    count = lambda d, pat: len(list((run / d).glob(pat))) if (run / d).is_dir() else 0  # noqa: E731
    stats = json.loads((run / "stats.json").read_text()) if (run / "stats.json").is_file() else {}
    info = json.loads((run / "run.json").read_text()) if (run / "run.json").is_file() else {}
    ec = (run / "exit_code.txt").read_text().strip() if (run / "exit_code.txt").is_file() else "?"
    size = sum(p.stat().st_size for p in run.rglob("*") if p.is_file()) / 2 ** 30
    return {"run": run.name, "exit_code": ec, "exit_reason": stats.get("exit_reason", "no stats.json"),
            "sim_s": stats.get("sim_time_s"), "wall_s": stats.get("wall_time_s"), "rtf": stats.get("rtf"),
            "gpu_mib": stats.get("gpu_used_mib_all_processes"),
            "sonar": count("sonar", "*.npz"), "left": count("camera", "*_rgb.png"),
            "right": count("camera_right", "*_rgb.png"), "gt": count("gt", "*.npz"),
            "gt_bubbles_mean": stats.get("record", {}).get("gt_bubbles_mean"), "gb": size,
            "estimate_gb": info.get("estimate", {}).get("gb"), "trajectory": info.get("trajectory", {}).get("name"),
            "notes": info.get("trajectory", {}).get("notes", []), **drift(run), "sheet": contact_sheet(run)}


def fmt(v, f="{:.2f}"):
    return "-" if v is None else f.format(v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=str(WS / "output" / "records"))
    ap.add_argument("--runs", nargs="*", default=None, help="run ids (default: every directory with a run.json)")
    ap.add_argument("--no-videos", action="store_true", help="skip <run>/video/*.mp4 (made once, then kept)")
    args = ap.parse_args()
    root = Path(args.root)
    runs = [root / r for r in args.runs] if args.runs else sorted(p.parent for p in root.glob("*/run.json"))
    if not args.no_videos:                     # tools/record_video.py: camera, sonar fan, combined (10 fps)
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from record_video import make_videos
        for r in runs:
            if r.is_dir() and (r / "manifest.json").is_file() and not (r / "video" / "combined.mp4").is_file():
                try:
                    make_videos(r)
                except Exception as exc:  # noqa: BLE001 - a failed video must not lose the report
                    print(f"[report] {r.name}: videos failed: {exc!r}")
    rows = [summarize(r) for r in runs if r.is_dir()]
    lines = ["# Recording runs", "", f"Root: `{root}`", "",
             "| run | exit | reason | sim s | wall s | RTF | GPU MiB | sonar | left | right | gt | L-R max ms | cam-sonar mean / max ms | GB (est.) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['run']} | {r['exit_code']} | {r['exit_reason']} | {fmt(r['sim_s'], '{:.1f}')} | "
                     f"{fmt(r['wall_s'], '{:.0f}')} | {fmt(r['rtf'], '{:.3f}')} | {fmt(r['gpu_mib'], '{:.0f}')} | "
                     f"{r['sonar']} | {r['left']} | {r['right']} | {r['gt']} | {fmt(r.get('left_right_max_ms'), '{:.3f}')} | "
                     f"{fmt(r.get('camera_sonar_mean_ms'), '{:.1f}')} / {fmt(r.get('camera_sonar_max_ms'), '{:.1f}')} | "
                     f"{r['gb']:.2f} ({fmt(r['estimate_gb'])}) |")
    for r in rows:
        lines += ["", f"## {r['run']} ({r['trajectory']})", ""]
        if r["gt_bubbles_mean"] is not None:
            lines.append(f"- ground truth: {r['gt_bubbles_mean']:.0f} bubbles per frame on average")
        lines += [f"- trajectory note: {n}" for n in r["notes"]]
        vids = sorted((root / r["run"] / "video").glob("*.mp4")) + sorted((root / r["run"] / "video").glob("*.avi"))
        if vids:
            lines.append("- videos (10 fps = sim real time): " + ", ".join(f"[{v.name}]({r['run']}/video/{v.name})"
                                                                        for v in vids))
        if r["sheet"] is not None:
            lines += ["", f"![contact sheet]({r['run']}/contact_sheet.png)"]
    (root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[report] {root / 'report.md'}: {len(rows)} runs")
    for r in rows:
        print(f"[report] {r['run']}: exit {r['exit_code']} ({r['exit_reason']}), sonar {r['sonar']}, "
              f"left {r['left']}, right {r['right']}, RTF {fmt(r['rtf'], '{:.3f}')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
