"""Plume preview (M1): the bubble plume and its integral model for the pipe presets, without the sonar.

    scripts/uw.sh py tools/plume_preview.py [--measure 10] [--preset buried] [--profile ...] [--set a.b=c]

Without a pipe preset both run (exposed, buried); other presets (e.g. basin) apply to both. After the warm-up
(plume.warmup_s) a measuring window is simulated and averaged over snapshots; written to
output/plume_preview/<pipe preset>/:
  bubbles.png   side (x-z) and top (x-y) views of the bubbles (colour = radius), integral centreline, stats block
  profiles.png  centreline speed and width vs height: integral model, bubbles, MTT, Lai & Socolofsky (2019)
  stats.json    the numbers behind both
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
import warp as wp  # noqa: E402

from uw import bubbles, config, geometry, plume  # noqa: E402
from uw import water as wat  # noqa: E402

PIPE_PRESETS = ("exposed", "buried")
HEIGHTS = (0.1, 0.25, 0.5, 1.0, 1.5, 2.0)      # above the source, for the stats table [m]
DZ = 0.05                                       # height bins for the bubble statistics [m]


class BubbleProfile:
    """Gas-volume-weighted bubble statistics per height bin, pooled over snapshots: mean rise speed, centroid and
    RMS horizontal distance from it (for a Gaussian void fraction exp(-r^2/b^2) that is b)."""

    def __init__(self, z0, top):
        self.z0, self.n = z0, int(math.ceil((top - z0) / DZ))
        self.acc = np.zeros((self.n, 7))      # weight, w*vz, w*x, w*y, w*x^2, w*y^2, count

    def add(self, snap):
        k = ((snap["pos"][:, 2] - self.z0) / DZ).astype(int)
        ok = (k >= 0) & (k < self.n)
        k, p, v = k[ok], snap["pos"][ok].astype(float), snap["vel"][ok].astype(float)
        w = snap["radius"][ok].astype(float) ** 3
        for j, val in enumerate((w, w * v[:, 2], w * p[:, 0], w * p[:, 1], w * p[:, 0] ** 2, w * p[:, 1] ** 2,
                                 np.ones_like(w))):
            self.acc[:, j] += np.bincount(k, val, self.n)

    def result(self):
        a = self.acc
        ok = a[:, 6] >= 50
        w = np.where(ok, a[:, 0], 1.0)
        mx, my = a[:, 2] / w, a[:, 3] / w
        rms = np.sqrt(np.maximum(a[:, 4] / w - mx ** 2 + a[:, 5] / w - my ** 2, 0.0))
        h = (np.arange(self.n) + 0.5) * DZ
        return {"h": h[ok], "vz": (a[:, 1] / w)[ok], "width": rms[ok]}


def run_preset(cfg, measure_s, snap_dt):
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    t0 = time.time()
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    c0 = sim.counters()
    prof = BubbleProfile(float(sim.flow.plume["pos"][0, 2]), sim.water_level)
    alive = []
    for _ in range(int(round(measure_s / snap_dt))):
        sim.step(snap_dt)
        snap = sim.snapshot()
        prof.add(snap)
        alive.append(len(snap["pos"]))
    wp.synchronize()
    c1 = sim.counters()
    return scene, sim, prof.result(), c0, c1, alive, time.time() - t0


def model_tables(sim, bub):
    """Model, bubble and literature profiles vs height above the source."""
    src, pl, alpha = sim.source, sim.flow.plume, sim.flow.alpha
    z_rel = pl["pos"][:, 2] - pl["pos"][0, 2]
    big_b = wat.G * src.q_gas
    d = plume.plume_length_scale(src.q_gas, src.slip_mean)
    rows = []
    for h in HEIGHTS:
        if h > z_rel[-1]:
            continue
        u_m, b_m = plume.mtt_plume(h, big_b, alpha)
        u_l, b_l = plume.lai_socolofsky_2019(h, src.q_gas, src.slip_mean)
        k = np.argmin(np.abs(bub["h"] - h)) if len(bub["h"]) else None
        rows.append({"h": h, "z_over_D": h / d,
                     "model_u_c": float(np.interp(h, z_rel, pl["u_c"])), "model_b": float(np.interp(h, z_rel, pl["b"])),
                     "bubble_vz": float(bub["vz"][k]) if k is not None else None,
                     "bubble_width": float(bub["width"][k]) if k is not None else None,
                     "mtt_u_c": float(u_m), "mtt_b": float(b_m), "lai_socolofsky_u_c": float(u_l),
                     "lai_socolofsky_b": float(b_l)})
    return rows, d


def stats_dict(cfg, name, sim, bub, c0, c1, alive, wall_s):
    src, pl = sim.source, sim.flow.plume
    window = c1["t"] - c0["t"]
    n_burst = c1["burst"] - c0["burst"]
    rows, d = model_tables(sim, bub)
    z_rel = pl["pos"][:, 2] - pl["pos"][0, 2]
    m0 = src.momentum / src.rho
    big_b = wat.G * src.q_gas
    t_k = float(config.get(cfg, "water.temperature_c")) + 273.15
    r_gas = wat.GASES[config.get(cfg, "leak.gas")]["R"]
    # fastest bubbles: on the centreline at u_c + slip
    transit = float(np.trapz(1.0 / (pl["u_c"] + src.slip_mean), z_rel))
    return {
        "preset": name, "profile": config.get(cfg, "profile.name"),
        "source": {"gas": config.get(cfg, "leak.gas"), "buried": src.buried, "vents": len(src.positions),
                   "release_depth_m": sim.water_level - float(src.positions[:, 2].mean()),
                   "p_release_kPa": src.p_release / 1e3, "mass_flow_g_s": src.mdot * 1e3,
                   "volume_flow_lpm_at_release": src.q_gas * 6e4,
                   "volume_flow_lpm_at_1atm": src.mdot * r_gas * t_k / wat.P_ATM * 6e4,
                   "choked": bool(src.info.get("choked", False)), "exit_velocity_m_s": src.info.get("u_exit"),
                   "momentum_flux_N": src.momentum},
        "bubbles": {"d50_mm": src.sizes.d50 * 1e3, "mean_volume_mm3": src.sizes.mean_volume * 1e9,
                    "slip_mean_m_s": src.slip_mean,
                    "per_s_model": src.q_gas / src.sizes.mean_volume,
                    "per_s_measured": (c1["emitted"] - c0["emitted"]) / window,
                    "alive_mean": float(np.mean(alive)), "alive_max": int(max(alive)),
                    "pool": sim.cap, "overwritten": c1["lost"]},
        "rise": {"mean_time_to_surface_s": (c1["burst_age"] - c0["burst_age"]) / n_burst if n_burst else None,
                 "centreline_transit_s": transit, "height_m": float(z_rel[-1])},
        "gas_balance": sim.gas_budget(),
        "profiles": rows,
        "regime": {"jet_length_lM_m": m0 ** 0.75 / big_b ** 0.5 if m0 > 0 else 0.0,
                   "bubble_length_D_m": d, "z_over_D_at_surface": float(z_rel[-1]) / d,
                   "entrainment_alpha": sim.flow.alpha},
        "run": {"warmup_s": float(config.get(cfg, "plume.warmup_s")), "measure_s": window,
                "wall_clock_s": wall_s, "snapshots": len(alive)},
    }


def plot_bubbles(path, scene, sim, st):
    snap = sim.snapshot()
    src, pl = sim.source, sim.flow.plume
    p, rmm = snap["pos"], snap["radius"] * 1e3
    c = src.positions.mean(0)
    half = 0.8
    fig = plt.figure(figsize=(16, 8.5))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 1.0, 0.62])
    vmin, vmax = np.percentile(rmm, [1, 99])
    # side view x-z, at the pipe
    ax = fig.add_subplot(gs[0])
    x = np.linspace(c[0] - half, c[0] + half, 400)
    ax.fill_between(x, 0, scene.sand.height(x, np.full_like(x, c[1])), color="#c8b48c", lw=0)
    pa = scene.pipe_axis
    ax.add_patch(plt.Circle((pa["x"], pa["z"]), pa["radius"], color="#555", zorder=2))
    sc = ax.scatter(p[:, 0], p[:, 2], s=0.6, c=rmm, cmap="viridis", vmin=vmin, vmax=vmax, lw=0, zorder=3)
    ax.plot(pl["pos"][:, 0], pl["pos"][:, 2], "r-", lw=1.2, label="integral centreline", zorder=4)
    left = pl["pos"] - pl["b"][:, None] * np.cross(pl["tangent"], [0, 1, 0])
    right = pl["pos"] + pl["b"][:, None] * np.cross(pl["tangent"], [0, 1, 0])
    ax.plot(left[:, 0], left[:, 2], "r--", lw=0.7, label="centreline ± b (Gaussian 1/e)", zorder=4)
    ax.plot(right[:, 0], right[:, 2], "r--", lw=0.7, zorder=4)
    ax.axhline(sim.water_level, color="tab:blue", lw=1, label="water surface")
    ax.set(xlim=(c[0] - half, c[0] + half), ylim=(0, sim.water_level + 0.08), aspect="equal",
           xlabel="x [m]", ylabel="z [m]", title="side view (x-z)")
    ax.legend(loc="upper left", fontsize=8)
    # top view x-y
    ax = fig.add_subplot(gs[1])
    ax.axvspan(pa["x"] - pa["radius"], pa["x"] + pa["radius"], color="#bbb", lw=0, label="pipe")
    ax.scatter(p[:, 0], p[:, 1], s=0.6, c=rmm, cmap="viridis", vmin=vmin, vmax=vmax, lw=0)
    ax.plot(pl["pos"][:, 0], pl["pos"][:, 1], "r-", lw=1.2, label="integral centreline")
    if src.buried:
        ax.plot(src.positions[:, 0], src.positions[:, 1], "kx", ms=7, label="sediment vents")
    ax.set(xlim=(c[0] - half, c[0] + half), ylim=(c[1] - half, c[1] + half), aspect="equal",
           xlabel="x [m]", ylabel="y [m]", title="top view (x-y)")
    ax.legend(loc="upper left", fontsize=8)
    fig.colorbar(sc, ax=fig.axes[:2], orientation="horizontal", fraction=0.04, pad=0.08, label="bubble radius [mm]")
    # stats block
    ax = fig.add_subplot(gs[2])
    ax.axis("off")
    ax.text(0.0, 1.0, stats_text(st), va="top", family="monospace", fontsize=8.2, transform=ax.transAxes)
    fig.suptitle(f"Gas leak plume, pipe preset '{st['preset']}' (profile {st['profile']}), "
                 f"t = {sim.t:.1f} s, {len(p)} bubbles")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def stats_text(st):
    s, b, r = st["source"], st["bubbles"], st["rise"]
    lines = [f"gas            {s['gas']}{' (buried, %d vents)' % s['vents'] if s['buried'] else ''}",
             f"release depth  {s['release_depth_m']:.2f} m ({s['p_release_kPa']:.0f} kPa abs)",
             f"mass flow      {s['mass_flow_g_s']:.3f} g/s{' (choked)' if s['choked'] else ''}",
             f"volume flow    {s['volume_flow_lpm_at_release']:.1f} L/min at release",
             f"               {s['volume_flow_lpm_at_1atm']:.1f} L/min at 1 atm",
             f"jet momentum   {s['momentum_flux_N']:.3f} N",
             f"bubbles d50    {b['d50_mm']:.1f} mm, slip {b['slip_mean_m_s']:.3f} m/s",
             f"bubbles/s      {b['per_s_measured']:.0f} (model {b['per_s_model']:.0f})",
             f"alive          {b['alive_mean']:.0f} mean, {b['alive_max']} max / {b['pool']}",
             f"rise time      {r['mean_time_to_surface_s']:.2f} s mean to surface",
             f"               {r['centreline_transit_s']:.2f} s on the centreline",
             "",
             "height  u_c model  b model  bubble vz  bubble width",
             "  [m]     [m/s]     [m]      [m/s]       [m]"]
    for row in st["profiles"]:
        lines.append(f"{row['h']:5.2f}   {row['model_u_c']:7.3f}  {row['model_b']:7.3f}   "
                     f"{row['bubble_vz'] if row['bubble_vz'] is not None else float('nan'):7.3f}    "
                     f"{row['bubble_width'] if row['bubble_width'] is not None else float('nan'):7.3f}")
    g = st["regime"]
    lines += ["", f"jet length l_M  {g['jet_length_lM_m'] * 100:.1f} cm",
              f"length scale D  {g['bubble_length_D_m']:.2f} m (z/D <= {g['z_over_D_at_surface']:.2f})",
              f"alpha (Gauss.)  {g['entrainment_alpha']}"]
    return "\n".join(lines)


def plot_profiles(path, sim, bub, st):
    src, pl = sim.source, sim.flow.plume
    z_rel = pl["pos"][:, 2] - pl["pos"][0, 2]
    h = np.linspace(0.05, z_rel[-1], 200)
    u_m, b_m = plume.mtt_plume(h, wat.G * src.q_gas, sim.flow.alpha)
    u_l, b_l = plume.lai_socolofsky_2019(h, src.q_gas, src.slip_mean)
    d = st["regime"]["bubble_length_D_m"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 6.5), sharey=True)
    a1.plot(pl["u_c"], z_rel, "k-", lw=2, label="integral model, liquid u_c")
    a1.plot(bub["vz"], bub["h"], ".", color="tab:green", ms=4, label="bubbles: mean rise speed (incl. slip)")
    a1.plot(u_m, h, "--", color="tab:blue", label="MTT pure plume, B = g Q (no slip)")
    a1.plot(u_l, h, ":", color="tab:red", lw=2,
            label=f"Lai & Socolofsky 2019 fit 1.6 u_s (z/D)^-1/3\n(fit range 2<z/D<11; here z/D<{z_rel[-1] / d:.2f})")
    a1.axvline(src.slip_mean, color="gray", lw=0.8, label=f"slip u_s = {src.slip_mean:.2f} m/s")
    a1.set(xlim=(0, 2.0), xlabel="centreline speed [m/s]", ylabel="height above the source [m]",
           title="centreline speed")
    a1.legend(fontsize=8, loc="upper right")
    a2.plot(pl["b"], z_rel, "k-", lw=2, label="integral model, Gaussian 1/e radius b")
    a2.plot(bub["width"], bub["h"], ".", color="tab:green", ms=4, label="bubbles: RMS distance from centroid")
    a2.plot(b_m, h, "--", color="tab:blue", label=f"MTT b = 1.2 alpha z (alpha = {sim.flow.alpha})")
    a2.plot(b_l, h, ":", color="tab:red", lw=2, label="bubble-plume data b_g = 0.114 z\n(Bordoloi et al. 2019 fig. 7b)")
    a2.set(xlabel="width [m]", title="plume width")
    a2.legend(fontsize=8, loc="lower right")
    fig.suptitle(f"Plume profiles, pipe preset '{st['preset']}': model vs literature (see docs/validation.md)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--measure", type=float, default=15.0, help="measuring window after the warm-up [s]")
    ap.add_argument("--snap", type=float, default=0.25, help="snapshot interval in the window [s]")
    config.add_args(ap.add_argument)
    args = ap.parse_args()
    wp.config.quiet = True
    pipes = [p for p in args.preset if p in PIPE_PRESETS] or list(PIPE_PRESETS)
    others = [p for p in args.preset if p not in PIPE_PRESETS]
    for preset in pipes:
        cfg = config.load(overrides=args.overrides, profile=args.profile, presets=[preset, *others])
        name = "+".join([preset, *others])
        out = os.path.join(config.WS, "output", "plume_preview", name)
        os.makedirs(out, exist_ok=True)
        scene, sim, bub, c0, c1, alive, wall_s = run_preset(cfg, args.measure, args.snap)
        st = stats_dict(cfg, name, sim, bub, c0, c1, alive, wall_s)
        with open(os.path.join(out, "stats.json"), "w") as f:
            json.dump(st, f, indent=2)
        plot_bubbles(os.path.join(out, "bubbles.png"), scene, sim, st)
        plot_profiles(os.path.join(out, "profiles.png"), sim, bub, st)
        print(f"{name}: {wall_s:.1f} s wall clock -> {out}")
        print(stats_text(st))


if __name__ == "__main__":
    main()
