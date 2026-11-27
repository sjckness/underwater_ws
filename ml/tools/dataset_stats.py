"""Statistics and a sanity check of a sweep dataset (tools/sweep.py), host-side (numpy, matplotlib).

    python3 tools/dataset_stats.py ROOT [--examples 20] [--image-checks 600] [--seed 0]

Writes ROOT/stats/: report.md, stats.json, overview.png (class balance, flow histograms, pose coverage, label sizes,
generation times) and examples.png (random pings as fans, labels outlined: mask_geom cyan, mask_direct orange).
Reads every ping's JSON; the image checks (NaN outside the blind zone, empty or saturated images, labels without a
leak) read the images of a random sample of pings.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))
from uw.sinks import read_png  # noqa: E402


def fan(polar: np.ndarray, range_m: np.ndarray, bearing_deg: np.ndarray, width: int = 400) -> tuple:
    """Polar image (rows = range from 0, columns = beams port to starboard) -> Cartesian fan (nearest neighbour),
    and its extent [m] for imshow."""
    r_max = float(range_m[-1])
    th = np.radians(bearing_deg)
    half = r_max * max(math.sin(float(np.max(np.abs(th)))), 0.5)
    h = int(width * r_max / (2 * half))
    x = np.linspace(-half, half, width)
    y = np.linspace(r_max, 0.0, h)
    xx, yy = np.meshgrid(x, y)
    r, b = np.hypot(xx, yy), np.arctan2(xx, yy)
    ri = np.round(r / (range_m[1] - range_m[0])).astype(int)
    bi = np.round(np.interp(b, th, np.arange(len(th)))).astype(int)
    ok = (ri < len(range_m)) & (b >= th[0]) & (b <= th[-1])
    out = np.zeros((h, width), polar.dtype)
    out[ok] = polar[ri[ok], bi[ok]]
    return out, (-half, half, 0.0, r_max)


def load(root: Path):
    manifest = json.loads((root / "manifest.json").read_text())
    pings = []
    for sc in manifest["scenarios"]:
        if sc.get("status") != "done":
            continue
        for f in sorted((root / sc["id"]).glob("ping_*.json")):
            m = json.loads(f.read_text())
            leak = m.get("leak") or {}
            pings.append({"scenario": sc["id"], "kind": sc["kind"], "split": sc["split"], "stem": f.stem,
                          "size": sc.get("params", {}).get("size"), "holdout": bool(sc.get("holdout")),
                          "mode": m["sonar"]["mode"], "pose_source": m["sonar"]["pose_source"],
                          "motion": sc.get("params", {}).get("motion")          # fixed views: pose_source poses
                          or ("fixed" if m["sonar"]["pose_source"] == "fixed" else "mobile"),
                          "roll": m["sonar"]["roll_deg"], "pitch": m["sonar"]["pitch_deg"],
                          "yaw": m["sonar"]["yaw_deg"], "x": m["sonar"]["position"][0], "y": m["sonar"]["position"][1],
                          "z": m["sonar"]["position"][2], "flow": leak.get("flow_lpm", 0.0) if leak else 0.0,
                          "flow_nominal": leak.get("flow_nominal_lpm", 0.0) if leak else 0.0,
                          "leak_range": leak.get("range_m", math.nan), "leak_bearing": leak.get("bearing_deg", math.nan),
                          "leak_elevation": leak.get("elevation_deg", math.nan), "geom": m["plume_pixels"]["geom"],
                          "direct": m["plume_pixels"]["direct"], "all": m["plume_pixels"]["all"],
                          "ping_ms": m["timing_ms"]["total"]})
    return manifest, pings


def image_checks(root: Path, pings: list, n: int, rng) -> dict:
    flags = Counter()
    sat = []
    for k in rng.choice(len(pings), size=min(n, len(pings)), replace=False):
        p = pings[k]
        d = root / p["scenario"]
        with np.load(d / f"{p['stem']}.npz") as z:
            db = z["polar_db"].astype(np.float32)
            rng_m = z["range_m"]
        meta = json.loads((d / f"{p['stem']}.json").read_text())
        blind = rng_m < 0.1 + 1e-6
        if np.isnan(db[~blind]).any():
            flags["NaN outside the blind zone"] += 1
        if not np.isfinite(db[~blind]).all():
            flags["inf in raw dB"] += 1
        img = read_png(d / f"{p['stem']}.png")
        if img.max() == 0:
            flags["empty polar image"] += 1
        s = float(np.mean(img == 255))
        sat.append(s)
        if s > 0.05:
            flags["more than 5 % of pixels saturated"] += 1
        if p["kind"] != "leak":
            for m in ("mask_geom", "mask_direct", "mask_all"):
                if read_png(d / f"{p['stem']}_{m}.png").any():
                    flags[f"{m} set without a leak"] += 1
        if meta["sonar"]["noise_floor_db"] < 60 or meta["sonar"]["noise_floor_db"] > 110:
            flags["noise floor outside 60-110 dB"] += 1
    return {"checked": int(min(n, len(pings))), "flags": dict(flags),
            "saturated_share_mean": float(np.mean(sat)) if sat else 0.0}


def overview(path: Path, manifest: dict, pings: list) -> None:
    sc = [s for s in manifest["scenarios"] if s.get("status") == "done"]
    fig, ax = plt.subplots(3, 3, figsize=(16, 13))
    kinds = ["leak", "negative", "distractor"]
    splits = ["train", "val", "test"]
    for i, k in enumerate(kinds):
        ax[0, 0].bar(np.arange(3) + 0.27 * i, [sum(s["kind"] == k and s["split"] == sp for s in sc) for sp in splits],
                     0.27, label=k)
    ax[0, 0].set(xticks=np.arange(3) + 0.27, xticklabels=splits, title="scenarios by split and kind")
    ax[0, 0].legend()
    leak_sc = [s for s in sc if s["kind"] == "leak"]
    bins = np.logspace(-1, math.log10(500), 25)
    for preset in ("pinhole", "crack", "rupture"):
        ax[0, 1].hist([s["params"]["leak.flow_lpm"] for s in leak_sc if s["params"].get("leak") == preset],
                      bins=bins, alpha=0.7, label=preset)
    ax[0, 1].set(xscale="log", xlabel="nominal flow [L/min]", title="leak flow by preset (scenarios)")
    ax[0, 1].legend()
    lp = [p for p in pings if p["kind"] == "leak"]
    ax[0, 2].hist([p["flow"] for p in lp if p["flow"] > 0], bins=bins, alpha=0.8, label="flow now > 0")
    ax[0, 2].set(xscale="log", xlabel="flow now [L/min]",
                 title=f"leak pings: {sum(p['flow'] == 0 for p in lp)} with the leak off (ramp start, intermittent)")
    fixed = [p for p in pings if p["motion"] == "fixed"]
    mobile = [p for p in pings if p["motion"] != "fixed"]
    ax[1, 0].hist([p["pitch"] for p in fixed], bins=30, alpha=0.7, label="fixed")
    ax[1, 0].hist([p["pitch"] for p in mobile], bins=30, alpha=0.7, label="mobile")
    ax[1, 0].set(xlabel="pitch [deg, down]", title="pitch per ping")
    ax[1, 0].legend()
    ax[1, 1].hist([p["yaw"] for p in fixed], bins=30, alpha=0.7, label="fixed")
    ax[1, 1].hist([p["yaw"] for p in mobile], bins=30, alpha=0.7, label="mobile")
    ax[1, 1].set(xlabel="yaw [deg]", title=f"yaw per ping; roll 90: {sum(abs(p['roll'] - 90) < 1 for p in pings)} pings")
    ax[1, 1].legend()
    ax[1, 2].scatter([p["x"] for p in mobile], [p["y"] for p in mobile], s=1, alpha=0.3, label="mobile")
    ax[1, 2].scatter([p["x"] for p in fixed], [p["y"] for p in fixed], s=3, alpha=0.5, label="fixed")
    ax[1, 2].set(xlabel="x [m]", ylabel="y [m]", title="sonar positions (top view)", aspect="equal")
    ax[1, 2].legend()
    inview = [p for p in lp if p["geom"] > 0]
    ax[2, 0].scatter([p["leak_bearing"] for p in inview], [p["leak_range"] for p in inview], s=1, alpha=0.3)
    ax[2, 0].set(xlabel="leak bearing [deg]", ylabel="leak range [m]",
                 title=f"leak in view (mask_geom set): {len(inview)} of {len(lp)} leak pings")
    for m, c in (("geom", "#0aa5c8"), ("direct", "#ff8c00"), ("all", "#888")):
        v = [p[m] for p in lp if p[m] > 0]
        if v:
            ax[2, 1].hist(v, bins=np.logspace(0, 6, 40), alpha=0.6, label=f"mask_{m}", color=c)
    ax[2, 1].set(xscale="log", xlabel="label pixels per ping", title="label sizes (leak pings, nonzero)")
    ax[2, 1].legend()
    walls = [s.get("wall_s", 0) for s in sc]
    ax[2, 2].hist(walls, bins=30)
    ax[2, 2].set(xlabel="wall time per scenario [s]",
                 title=f"generation: {sum(walls) / 3600:.2f} h of worker time for {len(sc)} scenarios")
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def examples(path: Path, root: Path, pings: list, n: int, rng) -> list:
    kinds = {k: [p for p in pings if p["kind"] == k] for k in ("leak", "negative", "distractor")}
    want = {"leak": int(round(0.7 * n)), "negative": int(round(0.2 * n))}
    want["distractor"] = n - want["leak"] - want["negative"]
    chosen = []
    for k, m in want.items():
        pool = kinds[k]
        chosen += [pool[i] for i in rng.choice(len(pool), size=min(m, len(pool)), replace=False)] if pool else []
    cols = 5
    rows = int(math.ceil(len(chosen) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3.0 * rows))
    for ax, p in zip(np.ravel(axes), chosen):
        d = root / p["scenario"]
        with np.load(d / f"{p['stem']}.npz") as z:
            rng_m, brg = z["range_m"], z["bearing_deg"]
        img, ext = fan(read_png(d / f"{p['stem']}.png")[::-1], rng_m, brg)
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=ext)
        for m, c in (("mask_direct", "#ff8c00"), ("mask_geom", "#00d5ff")):
            mk, _ = fan(read_png(d / f"{p['stem']}_{m}.png")[::-1], rng_m, brg)
            if mk.any():
                ax.contour(np.linspace(ext[0], ext[1], mk.shape[1]), np.linspace(ext[3], ext[2], mk.shape[0]),
                           mk > 0, levels=[0.5], colors=c, linewidths=0.8)
        ax.set_title(f"{p['scenario']} {p['kind']} {p['mode']} {'mobile' if p['motion'] != 'fixed' else ''}\n"
                     f"flow {p['flow']:.3g} L/min, pitch {p['pitch']:.0f}°, roll {p['roll']:.0f}°", fontsize=8)
        ax.tick_params(labelsize=6)
    for ax in np.ravel(axes)[len(chosen):]:
        ax.axis("off")
    fig.suptitle("random pings (fan view); outlines: mask_geom cyan, mask_direct orange", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)
    return [f"{p['scenario']}/{p['stem']}" for p in chosen]


def split_report(done: list, pings: list) -> dict:
    """The split is by scenario: this is the audit of that. ``leaked_scenarios`` must stay empty - a scenario (one
    sampled scene, and for a fixed scenario its three views of that scene) must sit in exactly one split, or a
    model sees its test scenes while training. ``by_size`` separates the held-out domain from the in-domain part
    of each split when the sweep set ``holdout``."""
    where: dict = {}
    for p in pings:
        where.setdefault(p["scenario"], set()).add(p["split"])
    sizes = sorted({s.get("params", {}).get("size") for s in done} - {None})
    splits = ("train", "val", "test")
    out = {
        "by": "scenario",
        "leaked_scenarios": sorted(k for k, v in where.items() if len(v) > 1),
        "scenarios": {sp: sum(s["split"] == sp for s in done) for sp in splits},
        "pings": {sp: sum(p["split"] == sp for p in pings) for sp in splits},
        "scenarios_by_size_split": {f"{sz}/{sp}": sum(s["split"] == sp and s.get("params", {}).get("size") == sz
                                                      for s in done) for sz in sizes for sp in splits},
    }
    held = [s for s in done if s.get("holdout")]
    if held:
        kept = {sz for s in held for sz in [s.get("params", {}).get("size")]}
        out["holdout"] = {
            "scenarios": len(held), "split": sorted({s["split"] for s in held}),
            "sizes": sorted(x for x in kept if x),
            # the point of the domain split: the held-out size must not appear in train or val at all
            "in_train_or_val": sum(s.get("holdout") and s["split"] in ("train", "val") for s in done),
            "in_domain_test_scenarios": sum(s["split"] == "test" and not s.get("holdout") for s in done)}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root")
    ap.add_argument("--examples", type=int, default=20)
    ap.add_argument("--image-checks", type=int, default=600)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    root = Path(args.root)
    out = root / "stats"
    out.mkdir(exist_ok=True)
    rng = np.random.default_rng(args.seed)
    manifest, pings = load(root)
    sc = manifest["scenarios"]
    done = [s for s in sc if s.get("status") == "done"]
    lp = [p for p in pings if p["kind"] == "leak"]
    never = sorted({p["scenario"] for p in lp} - {p["scenario"] for p in lp if p["geom"] > 0})
    params = {s["id"]: s.get("params", {}) for s in done}
    seen: dict = {}                                     # fixed views (sweeps with params.views): any ping in view?
    for p in lp:
        views = params[p["scenario"]].get("views")
        if views:
            k = int(p["stem"].split("_")[1]) * len(views) // int(manifest["pings_per_scenario"])
            seen[p["scenario"], k] = seen.get((p["scenario"], k), False) or p["geom"] > 0
    views_in_view = {g: [0, 0] for g in ("aimed", "random")}
    for (sid, k), ok in seen.items():
        g = views_in_view["aimed" if params[sid]["views"][k]["aimed"] else "random"]
        g[0], g[1] = g[0] + ok, g[1] + 1
    stats = {
        "scenarios": {"total": len(sc), "done": len(done),
                      "failed": [s["id"] for s in sc if s.get("status") == "failed"],
                      "missing": sum(s.get("status") == "missing" for s in sc)},
        "scenarios_by_kind_split": {f"{k}/{sp}": sum(s["kind"] == k and s["split"] == sp for s in done)
                                    for k in ("leak", "negative", "distractor") for sp in ("train", "val", "test")},
        "split": split_report(done, pings),
        "pings": len(pings),
        "pings_by_kind": dict(Counter(p["kind"] for p in pings)),
        "leak_pings_in_view": sum(p["geom"] > 0 for p in lp),
        "leak_pings_leak_off": sum(p["flow"] == 0 for p in lp),
        "leak_scenarios_never_in_view": never,
        "leak_views_in_view": {g: f"{a}/{n}" for g, (a, n) in views_in_view.items() if n},
        "leak_range_m_p5_p50_p95": {sz: [round(float(v), 2) for v in np.nanpercentile(
            [p["leak_range"] for p in lp if params[p["scenario"]].get("size") == sz], [5, 50, 95])]
            for sz in sorted({params[p["scenario"]].get("size") for p in lp} - {None})},
        "leak_pings_direct_label_without_geom": sum(p["direct"] > 0 and p["geom"] == 0 for p in lp),
        "mode": dict(Counter(p["mode"] for p in pings)),
        "pose_source": dict(Counter(p["pose_source"] for p in pings)),
        "roll90_pings": sum(abs(p["roll"] - 90) < 1 for p in pings),
        "presets": dict(Counter("/".join(s["presets"]) for s in done)),
        "flow_decades": dict(Counter(s["stratum"].split("/")[-1] for s in done if s["kind"] == "leak")),
        "label_pixels_leak_median": {m: float(np.median([p[m] for p in lp])) if lp else 0
                                     for m in ("geom", "direct", "all")},
        "ping_ms_mean_by_motion": {k: float(np.mean([p["ping_ms"] for p in pings if p["motion"] == k]))
                                   for k in {p["motion"] for p in pings}},
        "bytes": int(sum(s.get("bytes", 0) for s in done)),
        "worker_hours": float(sum(s.get("wall_s", 0) for s in done) / 3600),
    }
    stats["image_checks"] = image_checks(root, pings, args.image_checks, rng)
    overview(out / "overview.png", manifest, pings)
    stats["examples"] = examples(out / "examples.png", root, pings, args.examples, rng)
    (out / "stats.json").write_text(json.dumps(stats, indent=1))
    lines = [f"# Dataset statistics: {root}", "", "| | |", "|---|---|"]
    for k, v in stats.items():
        if k != "examples":
            lines.append(f"| {k} | `{json.dumps(v)}` |")
    lines += ["", "![overview](overview.png)", "", "![examples](examples.png)", ""]
    (out / "report.md").write_text("\n".join(lines))
    print(json.dumps({k: v for k, v in stats.items() if k != "examples"}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
