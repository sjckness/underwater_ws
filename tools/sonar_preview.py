"""Sonar preview (M2): polar and fan images of the tank, static and with the plume, focused and unfocused, plus timing.

    python -m uw py tools/sonar_preview.py [--pings 20] [--video 50] [--preset buried] [--profile ...] [--set a.b=c]

Without a pipe preset both run (exposed, buried); other presets (e.g. basin) apply to both. Written to
output/sonar_preview/<preset>[+roll<deg>][+pitch<deg>][+yaw<deg>]/ (a pose angle appears when --set changes it):
{static,plume}_{focus,nofocus}_{polar,fan}.png, overview.png, timing.json, and with --video N plume.mp4: N consecutive
pings with the plume at the ping rate (H.264; the fan in metres, the pose in the title, the outlines of the plume
labels mask_direct and mask_geom unless --no-contour, a side view of the tank with the sonar and its field of view) plus how the echo changes
from ping to ping (timing.json "video": speckle correlation of consecutive pings in the plume and in the static
scene, plume level statistics).
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


def fan_extent(son, img):
    """imshow extent [m] of a fan image (sonar.Sonar.fan: apex at the bottom centre, starboard to the right)."""
    h, w = img.shape
    scale = (w - 1) / (2 * son.range * max(math.sin(son.theta_max), 0.5))
    return (-w / 2 / scale, w / 2 / scale, -0.5 / scale, (h - 0.5) / scale)


def side_view(ax, cfg, scene, sim, son):
    """Inset: the tank in a side view (x-z at the sonar's y): sand, water level, pipe, plume centreline, the sonar and
    its field of view (the wireframe projected onto x-z)."""
    from uw.gui import fan_polylines
    L, _, H = scene.size
    ys = son.position[1]
    xs = np.linspace(0, L, 200)
    ax.fill_between(xs, 0, scene.sand.height(xs, np.full_like(xs, ys)), color="#c2b280")
    ax.axhline(scene.water_level, color="#4a90d9", lw=1)
    pa = scene.pipe_axis
    ax.add_patch(plt.Circle((pa["x"], pa["z"]), pa["radius"], color="#333"))
    if sim is not None:
        cl = sim.flow.plume["pos"]
        ax.plot(cl[:, 0], cl[:, 2], color="#3cb4e6", lw=2, label="plume")
    for line in fan_polylines(son):
        ax.plot(line[:, 0], line[:, 2], color="#e6a800", lw=0.8)
    ax.plot(*son.position[[0, 2]], "o", color="#d62728", ms=4)
    ax.add_patch(plt.Rectangle((0, 0), L, H, fill=False, lw=1.2))
    ax.set(xlim=(-0.05, L + 0.05), ylim=(-0.05, H + 0.05), aspect="equal", xlabel="x [m]", ylabel="z [m]")
    ax.set_title("side view (fan projected)", fontsize=9)
    ax.tick_params(labelsize=7)


def video(path, cfg, scene, sim, son, n, dt, title, contour=True):
    """n consecutive pings with the plume as an H.264 MP4 (1 / dt frames per second): the fan in metres, the title
    with the pose, the outlines of the plume labels mask_direct and mask_geom (contour=True) and a side view of the
    tank. Plus how the echo changes from ping to ping: speckle correlation (sonar_stats.speckle_correlation) of
    consecutive pings in the plume (direct share > 0.9) and in the static scene (share < 0.001, 20 dB over the noise
    floor)."""
    from uw import sonar_stats
    ffmpeg = deps.require("imageio_ffmpeg", "imageio-ffmpeg", "0.5.1").get_ffmpeg_exe()
    pings = []
    for _ in range(n):
        sim.step(dt)
        pings.append(son.ping(bubbles=sim))
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.06, 0.08, 0.62, 0.82])
    first = son.fan(pings[0].image)
    ext = fan_extent(son, first)
    im = ax.imshow(first, cmap="gray", vmin=0, vmax=255, extent=ext, interpolation="bilinear")
    ax.set(xlabel="across the fan [m] (starboard +)", ylabel="along the sonar axis [m]")
    side_view(fig.add_axes([0.71, 0.45, 0.27, 0.45]), cfg, scene, sim, son)
    if contour:
        fig.text(0.71, 0.36, "orange: plume label mask_direct, outline\n(bubbles' direct echo > 50 % of the pixel's "
                 "energy;\nincludes their sidelobe leakage)", fontsize=9, color="#e67e00", va="top")
        fig.text(0.71, 0.26, "cyan: geometric label mask_geom, outline\n(main-lobe cells of visible bubbles)",
                 fontsize=9, color="#0aa5c8", va="top")
    hdr = ax.set_title("")
    yy, xx = np.mgrid[0:first.shape[0], 0:first.shape[1]]
    x_m = ext[0] + (xx + 0.5) * (ext[1] - ext[0]) / first.shape[1]
    y_m = ext[3] - (yy + 0.5) * (ext[3] - ext[2]) / first.shape[0]
    cs = []
    w, h = fig.canvas.get_width_height()
    proc = subprocess.Popen([ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", f"{w}x{h}", "-r", f"{1 / dt:g}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                             "-crf", "18", "-pix_fmt", "yuv420p", path], stdin=subprocess.PIPE)
    t0 = sim.t - n * dt
    for i, p in enumerate(pings):
        im.set_data(son.fan(p.image))
        for c in cs:
            c.remove()
        cs = []
        for mask, colour in ((p.plume_mask_direct, "#ff8c00"), (p.plume_mask_geom, "#00d5ff")):
            if contour and mask.any():                  # the label's outline (smoothed over 9 bins x 7 beams)
                region = sonar_stats.local_mean(mask / 255.0, (9, 7)) > 0.5
                cs.append(ax.contour(x_m, y_m, son.fan(region.astype(np.uint8) * 255), levels=[127], colors=colour,
                                     linewidths=1.0))
        hdr.set_text(f"{title}   ping {i + 1}/{n}, t = {t0 + (i + 1) * dt:.1f} s")
        fig.canvas.draw()
        proc.stdin.write(np.asarray(fig.canvas.buffer_rgba())[..., :3].tobytes())
    proc.stdin.close()
    plt.close(fig)
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed writing {path}")
    nf = 10 ** (son.nf_db / 10)

    def corr(lag, region):
        out = []
        for a, b in zip(pings[:-lag], pings[lag:]):
            if region == "plume":
                m = (a.plume_share_direct > 0.9) & (b.plume_share_direct > 0.9)
            else:
                m = (a.plume_share_all < 1e-3) & (b.plume_share_all < 1e-3) & (a.power > 100 * nf) & (b.power > 100 * nf)
            out.append(sonar_stats.speckle_correlation(a.power, b.power, m))
        return float(np.nanmean(out))
    region = np.any([q.plume_mask_direct > 0 for q in pings], 0)
    level = np.array([10 * np.log10(q.power[region].mean()) for q in pings])
    return {"pings": n, "plume_pixels_ever": int(region.sum()),
            "plume_speckle_corr_lag1": corr(1, "plume"), "plume_speckle_corr_lag5": corr(5, "plume"),
            "static_speckle_corr_lag1": corr(1, "static"),
            "plume_level_db_std": float(level.std()),
            "plume_level_db_ping_to_ping_rms": float(np.sqrt(np.mean(np.diff(level) ** 2))),
            "label_pixels_mean": float(np.mean([q.plume_mask_direct.astype(bool).sum() for q in pings])),
            "label_pixels_std": float(np.std([q.plume_mask_direct.astype(bool).sum() for q in pings])),
            "geom_label_pixels_mean": float(np.mean([q.plume_mask_geom.astype(bool).sum() for q in pings]))}


def run(cfg, name, n_pings, n_video=0, contour=True):
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
        sc_ = config.get(cfg, "sonar")
        title = (f"{name}: roll {sc_['roll_deg']:g}°, pitch {sc_['pitch_deg']:g}° down, yaw {sc_['yaw_deg']:g}°, "
                 f"{sc_['mode'].upper()} {'focused' if son.focus else 'unfocused'}")
        t["video"] = video(os.path.join(out, "plume.mp4"), cfg, scene, sim, son, n_video, dt, title, contour)
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
    ap.add_argument("--no-contour", action="store_true", help="video without the plume label outlines")
    config.add_args(ap.add_argument)
    args = ap.parse_args()
    wp.config.quiet = True
    pipes = [p for p in args.preset if p in PIPE_PRESETS] or list(PIPE_PRESETS)
    others = [p for p in args.preset if p not in PIPE_PRESETS]
    for preset in pipes:
        cfg = config.load(overrides=args.overrides, profile=args.profile, presets=[preset, *others])
        default = config.load(profile=args.profile, presets=[preset, *others])
        pose = [f"{k}{config.get(cfg, f'sonar.{k}_deg'):g}" for k in ("roll", "pitch", "yaw")
                if config.get(cfg, f"sonar.{k}_deg") != config.get(default, f"sonar.{k}_deg")]
        name = "+".join([preset, *others, *pose])
        t = run(cfg, name, args.pings, args.video, not args.no_contour)
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
