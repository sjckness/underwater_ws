"""Sonar preview (M2): polar and fan images of the tank, static and with the plume, focused and unfocused, plus timing.

    python -m uw py tools/sonar_preview.py [--pings 20] [--preset buried] [--profile ...] [--set a.b=c]

Without a pipe preset both run (exposed, buried); other presets (e.g. basin) apply to both. Written to
output/sonar_preview/<preset>/: {static,plume}_{focus,nofocus}_{polar,fan}.png, overview.png, timing.json.
Timing: scene and scatterers, plume warm-up, sonar setup and static precompute, then --pings pings with the plume
(bubble step 1 / ping rate + sonar ping, per stage), and peak GPU memory of this process.
"""
import argparse
import json
import math
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

from uw import bubbles, config, geometry, sinks, sonar  # noqa: E402

PIPE_PRESETS = ("exposed", "buried")


def run(cfg, name, n_pings):
    out = os.path.join(config.WS, "output", "sonar_preview", name)
    os.makedirs(out, exist_ok=True)
    t = {}
    t0 = time.perf_counter()
    scene = geometry.build_scene(cfg)
    sc = geometry.scatterers(cfg, scene)
    t["scene_and_scatterers_s"] = time.perf_counter() - t0
    t0 = time.perf_counter()
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    wp.synchronize()
    t["plume_warmup_s"] = time.perf_counter() - t0
    son = sonar.Sonar(cfg, scene, sc, plume=sonar.plume_region(sim))
    t.update({f"sonar_{k}_ms": v for k, v in son.timing_ms.items()})
    t["scatterers"] = son.n_scatterers
    t["plume_box_paths"] = son.n_dyn
    fans = {}
    for focus in (True, False):
        tag = "focus" if focus else "nofocus"
        for kind, bub in (("static", None), ("plume", sim)):
            p = son.ping(bubbles=bub, focus=focus)
            sinks.write_png(os.path.join(out, f"{kind}_{tag}_polar.png"), p.image[::-1])
            fans[kind, tag] = son.fan(p.image)
            sinks.write_png(os.path.join(out, f"{kind}_{tag}_fan.png"), fans[kind, tag])
    dt = 1.0 / float(config.get(cfg, "sonar.ping_rate_hz"))
    steps, pings = [], []
    for _ in range(n_pings):
        t0 = time.perf_counter()
        sim.step(dt)
        wp.synchronize()
        steps.append((time.perf_counter() - t0) * 1e3)
        pings.append(son.ping(bubbles=sim).timing_ms)
    stages = {k: float(np.mean([p[k] for p in pings])) for k in pings[0]}
    t["ping_ms_mean"] = stages
    t["ping_ms_max"] = float(max(p["total"] for p in pings))
    t["bubble_step_ms_mean"] = float(np.mean(steps))
    t["per_ping_total_ms_mean"] = stages["total"] + t["bubble_step_ms_mean"]
    t["torch_peak_mib"] = torch.cuda.max_memory_allocated() / 2 ** 20
    if hasattr(wp, "get_mempool_used_mem_high"):
        t["warp_mempool_peak_mib"] = wp.get_mempool_used_mem_high("cuda:0") / 2 ** 20
    free, total = torch.cuda.mem_get_info()
    t["gpu_used_mib_all_processes"] = (total - free) / 2 ** 20
    with open(os.path.join(out, "timing.json"), "w") as f:
        json.dump(t, f, indent=1)
    overview(os.path.join(out, "overview.png"), fans, name, son, t)
    return t


def overview(path, fans, name, son, t):
    fig, axes = plt.subplots(2, 2, figsize=(15, 9.5))
    extent = None
    for ax, (kind, tag) in zip(axes.flat, [("static", "focus"), ("plume", "focus"), ("static", "nofocus"),
                                           ("plume", "nofocus")]):
        img = fans[kind, tag]
        half = img.shape[1] / 2 / ((img.shape[1] - 1) / (2 * son.range * max(math.sin(son.theta_max), 0.5)))
        extent = (-half, half, 0, img.shape[0] / ((img.shape[1] - 1) / (2 * son.range * max(math.sin(son.theta_max), 0.5))))
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=extent)
        ax.set(title=f"{kind}, {'focused' if tag == 'focus' else 'unfocused'}", xlabel="across [m]", ylabel="ahead [m]")
    s = t["ping_ms_mean"]
    fig.suptitle(f"Sonar preview '{name}': ping {s['total']:.0f} ms (plume paths {s['plume_box_paths']:.0f}, "
                 f"beamforming {s['fft_beamforming']:.1f}), bubble step {t['bubble_step_ms_mean']:.0f} ms per ping")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pings", type=int, default=20, help="pings for the timing")
    config.add_args(ap.add_argument)
    args = ap.parse_args()
    wp.config.quiet = True
    pipes = [p for p in args.preset if p in PIPE_PRESETS] or list(PIPE_PRESETS)
    others = [p for p in args.preset if p not in PIPE_PRESETS]
    for preset in pipes:
        name = "+".join([preset, *others])
        cfg = config.load(overrides=args.overrides, profile=args.profile, presets=[preset, *others])
        t = run(cfg, name, args.pings)
        s = t["ping_ms_mean"]
        print(f"{name}: precompute {t['sonar_static_precompute_ms']:.0f} ms ({t['scatterers'] / 1e6:.2f} M scatterers, "
              f"{t['plume_box_paths'] / 1e6:.2f} M plume-box paths); ping {s['total']:.1f} ms mean, "
              f"{t['ping_ms_max']:.1f} max; bubble step {t['bubble_step_ms_mean']:.1f} ms; "
              f"torch peak {t['torch_peak_mib']:.0f} MiB")
        print("  stages [ms]: " + ", ".join(f"{k} {v:.2f}" for k, v in s.items()))


if __name__ == "__main__":
    main()
