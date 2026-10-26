"""Sonar preview (M2): polar and fan images of the tank, static and with the plume, focused and unfocused, plus timing.

    python -m uw py tools/sonar_preview.py [--pings 20] [--video 50] [--preset buried] [--profile ...] [--set a.b=c]

Without a pipe preset both run (exposed, buried); other presets (e.g. basin) apply to both. Written to
output/sonar_preview/<preset>[+roll<deg>]/: {static,plume}_{focus,nofocus}_{polar,fan}.png, overview.png,
timing.json, and with --video N plume.mp4: N consecutive pings with the plume (fan view, the ping rate, H.264) plus
how the plume echo changes from ping to ping (timing.json "video").
Timing: scene and scatterers, plume warm-up, sonar setup and static precompute, then --pings pings with the plume
(bubble step 1 / ping rate + sonar ping, per stage), and peak GPU memory of this process.
"""
import argparse
import json
import math
import os
import subprocess
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

from uw import bubbles, config, deps, geometry, sinks, sonar  # noqa: E402

PIPE_PRESETS = ("exposed", "buried")


def video(path, sim, son, n, dt):
    """n consecutive pings with the plume as an H.264 MP4 (fan view, 1 / dt frames per second), and statistics of the
    plume echo (the pixels the direct-path label ever marks) from ping to ping."""
    ffmpeg = deps.require("imageio_ffmpeg", "imageio-ffmpeg", "0.5.1").get_ffmpeg_exe()
    frames, polar, masks = [], [], []
    for _ in range(n):
        sim.step(dt)
        p = son.ping(bubbles=sim)
        frames.append(son.fan(p.image))
        polar.append(p.power.copy())
        masks.append(p.plume_mask_direct > 0)
    h, w = frames[0].shape
    hh, ww = h + h % 2, w + w % 2
    proc = subprocess.Popen([ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "gray",
                             "-s", f"{ww}x{hh}", "-r", f"{1 / dt:g}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                             "-crf", "18", "-pix_fmt", "yuv420p", path], stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.pad(f, ((0, hh - h), (0, ww - w))).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed writing {path}")
    region = np.any(masks, 0)
    level = np.array([10 * np.log10(q[region].mean()) for q in polar])          # plume echo energy per ping
    corr = [float(np.corrcoef(a[region], b[region])[0, 1]) for a, b in zip(polar[:-1], polar[1:])]
    return {"pings": n, "plume_pixels": int(region.sum()), "plume_level_db_std": float(level.std()),
            "plume_level_db_ping_to_ping_rms": float(np.sqrt(np.mean(np.diff(level) ** 2))),
            "pixel_correlation_consecutive_pings": float(np.mean(corr)),
            "pixel_correlation_5_pings_apart": float(np.mean([np.corrcoef(a[region], b[region])[0, 1]
                                                              for a, b in zip(polar[:-5], polar[5:])])),
            "label_pixels_mean": float(np.mean([m.sum() for m in masks])),
            "label_pixels_std": float(np.std([m.sum() for m in masks]))}


def run(cfg, name, n_pings, n_video=0):
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
    overview(os.path.join(out, "overview.png"), fans, name, son, t)
    if n_video:
        t["video"] = video(os.path.join(out, "plume.mp4"), sim, son, n_video, dt)
    with open(os.path.join(out, "timing.json"), "w") as f:
        json.dump(t, f, indent=1)
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
    ap.add_argument("--video", type=int, default=0, metavar="N", help="also write N consecutive plume pings as MP4")
    config.add_args(ap.add_argument)
    args = ap.parse_args()
    wp.config.quiet = True
    pipes = [p for p in args.preset if p in PIPE_PRESETS] or list(PIPE_PRESETS)
    others = [p for p in args.preset if p not in PIPE_PRESETS]
    for preset in pipes:
        cfg = config.load(overrides=args.overrides, profile=args.profile, presets=[preset, *others])
        roll = float(config.get(cfg, "sonar.roll_deg"))
        name = "+".join([preset, *others] + ([f"roll{roll:g}"] if roll else []))
        t = run(cfg, name, args.pings, args.video)
        s = t["ping_ms_mean"]
        print(f"{name}: precompute {t['sonar_static_precompute_ms']:.0f} ms ({t['scatterers'] / 1e6:.2f} M scatterers, "
              f"{t['plume_box_paths'] / 1e6:.2f} M plume-box paths); ping {s['total']:.1f} ms mean, "
              f"{t['ping_ms_max']:.1f} max; bubble step {t['bubble_step_ms_mean']:.1f} ms; "
              f"torch peak {t['torch_peak_mib']:.0f} MiB")
        print("  stages [ms]: " + ", ".join(f"{k} {v:.2f}" for k, v in s.items()))
        if "video" in t:
            print("  video: " + ", ".join(f"{k} {v:.3g}" for k, v in t["video"].items()))


if __name__ == "__main__":
    main()
