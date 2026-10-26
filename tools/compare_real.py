"""Compare sonar recordings, real or simulated, on what M2 checks (skeleton until a real recording exists).

    python -m uw py tools/compare_real.py DATASET [DATASET ...] [--frames 50] [--out output/compare_real]

DATASET: a directory written by `python -m uw sonar-only` (sonar/<seq>.npz: polar_db, range_m, bearing_deg).
A real Oculus recording (format TBD: .oculus file, ROS bag of ProjectedSonarImage, ...) needs a loader in LOADERS
that yields the same frames: (polar dB re an arbitrary reference, row 0 at range 0, NaN = no data; range axis;
bearing axis).
Per dataset, averaged over the frames: background speckle (Rayleigh check, K shape), the brightest target's width
and sidelobe level, blind zone and ring-down near the sonar, the level-vs-range (TVG) profile. JSON + PNG.
"""
import argparse
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from uw import config, sonar_stats  # noqa: E402


def load_uw(path: Path, frames: int):
    for f in sorted((path / "sonar").glob("*.npz"))[:frames]:
        with np.load(f) as z:
            yield z["polar_db"].astype(np.float32), z["range_m"], z["bearing_deg"]


LOADERS = {"uw": load_uw}        # real formats: add a loader here


def analyse(path: Path, frames: int, sand_window, bearing_window, tvg_window, target) -> dict:
    loader = LOADERS["uw"] if (path / "manifest.json").is_file() else None
    if loader is None:
        raise SystemExit(f"{path}: no loader for this format yet (LOADERS: {', '.join(LOADERS)})")
    bg, tg, nr, tv = [], [], [], []
    for db, rng, brg in loader(path, frames):
        bg.append(sonar_stats.background(db, rng, brg, sand_window, bearing_window))
        tg.append(sonar_stats.bright_target(db, rng, brg, target[:2], target[2:]))
        nr.append(sonar_stats.near_range(db, rng))
        tv.append(sonar_stats.tvg_profile(db, rng, tvg_window))
    mean = lambda xs, k: float(np.mean([x[k] for x in xs]))       # noqa: E731
    return {"dataset": str(path), "frames": len(bg),
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
        name = Path(r["dataset"]).name
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
    ap.add_argument("--sand", type=float, nargs=2, default=[2.15, 2.8], help="background range window [m]")
    ap.add_argument("--bearings", type=float, nargs=2, default=[-25.0, 25.0], help="background bearing window [deg]")
    ap.add_argument("--tvg", type=float, nargs=2, default=[1.8, 3.4], help="range window of the TVG fit [m]")
    ap.add_argument("--target", type=float, nargs=4, default=[0.3, 1e9, -180, 180], metavar=("R0", "R1", "B0", "B1"),
                    help="window [m, m, deg, deg] holding a point target for the width / sidelobe check")
    ap.add_argument("--out", type=Path, default=Path(config.WS) / "output" / "compare_real")
    args = ap.parse_args()
    results = [analyse(d, args.frames, args.sand, args.bearings, args.tvg, args.target) for d in args.datasets]
    os.makedirs(args.out, exist_ok=True)
    (args.out / "compare.json").write_text(json.dumps(results, indent=1))
    plot(results, args.out / "compare.png")
    for r in results:
        b, t, n, v = r["background"], r["bright_target"], r["near_range"], r["tvg"]
        print(f"{r['dataset']} ({r['frames']} frames): background contrast {b['contrast']:.3f}, K shape "
              f"{b['k_shape_median']:.1f}, KS vs Rayleigh {b['ks_vs_rayleigh']:.3f}; brightest target at "
              f"{t['range_m']:.2f} m / {t['bearing_deg']:.1f} deg, width {t['width_deg']:.2f} deg, sidelobe "
              f"{t['sidelobe_db']:.1f} dB; blind zone {n['blind_zone_m']:.3f} m, ring-down {n['ringdown_slope_db_per_m']:.0f} "
              f"dB/m; level vs range {v['k_db_per_decade']:.1f} log R + {v['b_db_per_m']:.2f} R")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
