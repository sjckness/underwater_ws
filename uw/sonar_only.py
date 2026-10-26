"""`python -m uw sonar-only`: bubbles + sonar on the GPU (Warp), no Isaac and no RTX; writes through a SensorSink.

    python -m uw sonar-only --pings 200 --out output/datasets/<name> [--no-plume] [--profile ...] [--preset ...]
        [--set a.b=c]
Sim time advances 1 / sonar.ping_rate_hz per ping; the plume is warmed up first (plume.warmup_s).
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import warp as wp

from . import bubbles, config, geometry, recorder, sinks, sonar


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="python -m uw sonar-only", description=__doc__.splitlines()[0])
    ap.add_argument("--pings", type=int, default=100)
    ap.add_argument("--out", required=True, help="dataset directory, e.g. output/datasets/<name>")
    ap.add_argument("--sink", default="file", help="file | ros (M3) | zenoh (M7)")
    ap.add_argument("--no-plume", action="store_true", help="no gas: the static scene only")
    config.add_args(ap.add_argument)
    args = ap.parse_args(argv)
    wp.config.quiet = True
    cfg = config.from_args(args)
    t0 = time.perf_counter()
    scene = geometry.build_scene(cfg)
    sc = geometry.scatterers(cfg, scene)
    sim = None
    if config.get(cfg, "leak.enabled") and not args.no_plume:
        sim = bubbles.BubbleSim(cfg, scene)
        sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    son = sonar.Sonar(cfg, scene, sc, plume=sonar.plume_region(sim) if sim else None)
    setup_s = time.perf_counter() - t0
    dt = 1.0 / float(config.get(cfg, "sonar.ping_rate_hz"))
    info = {"name": args.out, "command": "python -m uw sonar-only " + " ".join(argv),
            "config_hash": recorder.config_hash(cfg), "seed": cfg["seed"], "config": cfg}
    step_ms, ping_ms, write_ms = [], [], []
    with sinks.make_sink(args.sink, out_dir=args.out, info=info) as sink:
        for i in range(args.pings):
            t = time.perf_counter()
            if sim is not None:
                sim.step(dt)
                wp.synchronize()
            t1 = time.perf_counter()
            ping = son.ping(bubbles=sim)
            t2 = time.perf_counter()
            sink.publish(recorder.sonar_message(i, sim.t if sim is not None else i * dt, ping, son, cfg, scene, sim))
            t3 = time.perf_counter()
            step_ms.append((t1 - t) * 1e3)
            ping_ms.append((t2 - t1) * 1e3)
            write_ms.append((t3 - t2) * 1e3)
    print(f"sonar-only: {args.pings} pings -> {args.out}; setup {setup_s:.1f} s; per ping: bubbles "
          f"{np.mean(step_ms):.1f} ms, sonar {np.mean(ping_ms):.1f} ms (max {np.max(ping_ms):.1f}), "
          f"write {np.mean(write_ms):.1f} ms", file=sys.stderr)
    return 0
