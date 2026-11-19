"""One headless recording run with the ROV on a scripted trajectory: sonar, stereo cameras, ground truth, poses.

    python tools/record.py --trajectory circle                      # 5 min simulated, room9, -> output/records/circle_s7
    python tools/record.py --trajectory updown --duration 20 --run-id test_updown --seed 3 [--set a.b=c ...]

Writes <out-root>/<run_id>/: run.json (seed, conditions, trajectory and its notes, estimate), sonar/ (polar frames +
meta, 10 Hz), camera/ and camera_right/ (10 Hz, the same sim stamps), gt/ (per-frame bubbles: bubble_id, position,
diameter_m, distances), poses.csv, manifest.json (FileSink), stats.json (loop statistics: RTF, GPU memory). The log
goes to stderr (record_all.bat redirects it to <run_id>/run.log).

Refuses to start (exit 3) when the free space on the output drive is below 3x the estimate for the run. The
estimate is record.mb_per_frame (MB per 10 Hz frame of each stream) x frames, calibrated on a 20 s room9 run.
Runs --no-realtime: every stamp is sim time, so the data do not depend on how fast the machine is.

Not the evaluation protocol's v1 contract (images/, labels/ YOLO-seg, meta/<frame>.json with label_index): no
camera instance labels exist yet. gt/ carries the per-bubble truth that such labels would be derived from.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))

from uw import config  # noqa: E402

RATE_HZ = 10.0


def overrides(args) -> list[str]:
    return [f"seed={args.seed}", "sonar.pose_source=script", f"sonar.script={args.trajectory}",
            "sonar.static_cache=on_motion", "vehicle.stereo.enabled=true", "record.enabled=true",
            "recorder.png_level=1", "camera.rate_hz=10", "sonar.ping_rate_hz=10",
            f"lights.ambient_dome={args.ambient_dome}", *args.overrides]


def estimate_gb(cfg: dict, duration: float) -> tuple[float, dict]:
    mb = config.get(cfg, "record.mb_per_frame")
    per_frame = mb["sonar"] + mb["camera"] * (2 if config.get(cfg, "vehicle.stereo.enabled") else 1) + mb["gt"]
    frames = duration * RATE_HZ
    return frames * per_frame / 1024.0, {"frames": frames, "mb_per_frame": per_frame, **mb}


def run_json(cfg: dict, args, run_dir: Path, est_gb: float, est: dict) -> dict:
    from uw.geometry import build_scene
    from uw.trajectory import build
    scene = build_scene(cfg)
    traj = build(cfg, args.trajectory, scene.leak.hole)
    try:
        commit = subprocess.run(["git", "-C", str(WS), "rev-parse", "--short", "HEAD"], capture_output=True,
                                text=True, timeout=10).stdout.strip()
    except Exception:  # noqa: BLE001
        commit = ""
    g = lambda k: config.get(cfg, k)  # noqa: E731
    return {
        "run_id": args.run_id, "seed": cfg["seed"], "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "git_commit": commit, "preset": args.preset, "profile": args.profile, "overrides": overrides(args),
        "duration_s": args.duration, "rate_hz": RATE_HZ,
        "conditions": {
            "scene": g("scene.name"), "camera": "stereo",
            "turbidity": {"light_attenuation_per_m": g("water.light_attenuation"),
                          "light_scattering_per_m": g("water.light_scattering"),
                          "sonar_volume_sv_db": g("sonar.volume_sv_db")},
            "light": {"ambient_dome": g("lights.ambient_dome"),
                      "lamps": [{"intensity": l["intensity"], "cone_deg": l["cone_deg"]} for l in g("lights.lamps")]},
            "flow_lpm": g("leak.flow_lpm"), "leak_source": g("leak.source"), "gas": g("leak.gas"),
        },
        "leak_position_m": [float(v) for v in scene.leak.hole],
        "trajectory": {"name": traj.name, "duration_s": traj.duration, "held_after_s": max(0.0, args.duration - traj.duration),
                       "params": traj.params, "notes": traj.notes},
        "stereo": {"baseline_m": g("vehicle.stereo.baseline_m"), "note": "baseline TO BE VERIFIED"},
        "vehicle_body_size_m": g("vehicle.body.size"),
        "estimate": {"gb": round(est_gb, 3), **est},
        "streams": {"sonar": "sonar/<seq>.npz|.json|_polar.png", "camera_left": "camera/<seq>_rgb.png",
                    "camera_right": "camera_right/<seq>_rgb.png", "ground_truth": "gt/<seq>.npz|.json",
                    "poses": "poses.csv", "stamps": "sim time [s] in every .json and poses.csv"},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trajectory", required=True, help="config/trajectories/<name>.yaml: circle | updown | combined")
    ap.add_argument("--duration", type=float, default=300.0, help="simulated seconds (default 300)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--run-id", default="", help="default <trajectory>_s<seed>")
    ap.add_argument("--preset", default="room9")
    ap.add_argument("--profile", default=config.DEFAULT_PROFILE)
    ap.add_argument("--ambient-dome", type=float, default=300.0, help="lights.ambient_dome (0: lamps only)")
    ap.add_argument("--out-root", default=str(WS / "output" / "records"))
    ap.add_argument("--set", dest="overrides", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--dry-run", action="store_true", help="estimate, check the disk, write run.json, do not run")
    args = ap.parse_args()
    args.run_id = args.run_id or f"{args.trajectory}_s{args.seed}"

    cfg = config.load(overrides=overrides(args), profile=args.profile, presets=[args.preset])
    run_dir = Path(args.out_root) / args.run_id
    est_gb, est = estimate_gb(cfg, args.duration)
    Path(args.out_root).mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(args.out_root).free / 2 ** 30
    print(f"[record] {args.run_id}: {args.duration:.0f} s, estimate {est_gb:.2f} GB, free {free_gb:.1f} GB", flush=True)
    if free_gb < 3.0 * est_gb:
        print(f"[record] REFUSED: free space {free_gb:.1f} GB < 3 x estimate ({3 * est_gb:.1f} GB)", flush=True)
        return 3
    if run_dir.exists() and any(p.name != "run.log" for p in run_dir.iterdir()):
        print(f"[record] REFUSED: {run_dir} already has data; choose another --run-id or move it away", flush=True)
        return 4
    run_dir.mkdir(parents=True, exist_ok=True)
    info = run_json(cfg, args, run_dir, est_gb, est)
    (run_dir / "run.json").write_text(json.dumps(info, indent=1, default=str))
    for n in info["trajectory"]["notes"]:
        print(f"[record] trajectory note: {n}", flush=True)
    if args.dry_run:
        return 0

    from uw import launcher, runner                     # the native launcher's child environment (tools/debug_sim.py)
    rt = launcher.runtime()
    n = rt["native"]
    os.environ.update({
        "UW_WS": str(WS), "UW_PYDEPS": str(Path(n["pydeps"]).expanduser()),
        "UW_WARP_PATH": launcher.warp_path(rt, n.get("isaac_root")), "UW_PYTEST_VERSION": str(rt["pytest_version"]),
        "UW_USD_PATH": launcher.warp_path(rt, n.get("isaac_root"), "usd_path"), "UW_ISAAC_ROOT": n.get("isaac_root") or "",
        "UW_DATA": os.path.expanduser(str(rt.get("data_root") or ""))})
    h = logging.StreamHandler(sys.stderr)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
    logging.getLogger("uw").addHandler(h)
    logging.getLogger("uw").setLevel(logging.INFO)
    argv = ["sim", "--sink", "file", "--out", str(run_dir), "--duration", str(args.duration), "--no-realtime",
            "--stats-file", str(run_dir / "stats.json"), "--profile", args.profile, "--preset", args.preset]
    for o in overrides(args):
        argv += ["--set", o]
    print("[record] python -m uw " + " ".join(argv), flush=True)
    return int(runner.main(argv) or 0)


if __name__ == "__main__":
    sys.exit(main())
