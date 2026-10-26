"""`python -m uw sonar-only`: bubbles + sonar on the GPU (Warp), no Isaac and no RTX; writes through a SensorSink.

    python -m uw sonar-only --pings 200 --out output/datasets/<name> [--no-plume] [--profile ...] [--preset ...]
        [--set a.b=c]
    python -m uw sonar-only --sink ros --pings 1000      the sonar topics of `sim` (uw/ros_bridge.py), no Isaac
Sim time advances 1 / sonar.ping_rate_hz per ping; the plume is warmed up first (plume.warmup_s). The pings are the
ones `python -m uw sim` makes for the same config and seed (bit-identical). A moving sonar (sonar.pose_source
trajectory | ros) takes its pose per ping; set sonar.static_cache on_motion or none.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import warp as wp

from . import bubbles, config, geometry, pose, recorder, sinks, sonar, surface
from .ros_bridge import R_SONAR_PROJECTED


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="python -m uw sonar-only", description=__doc__.splitlines()[0])
    ap.add_argument("--pings", type=int, default=100)
    ap.add_argument("--out", default="", help="dataset directory for --sink file, e.g. output/datasets/<name>")
    ap.add_argument("--sink", default="file", help="file | ros | zenoh (M7)")
    ap.add_argument("--no-plume", action="store_true", help="no gas: the static scene only")
    config.add_args(ap.add_argument)
    args = ap.parse_args(argv)
    if args.sink == "file" and not args.out:
        ap.error("--sink file needs --out DIR")
    wp.config.quiet = True
    cfg = config.from_args(args)
    info = {"name": args.out, "command": "python -m uw sonar-only " + " ".join(argv)}
    kw = {"out_dir": args.out, "info": info} if args.sink == "file" else {}
    with sinks.make_sink(args.sink, **kw) as sink:
        st = run(cfg, args.pings, sink, plume=not args.no_plume, fan=True if args.sink == "ros" else None, info=info)
    print(f"sonar-only: {args.pings} pings -> {args.out or args.sink}; setup {st['setup_s']:.1f} s; per ping: "
          f"bubbles {st['bubbles_ms']:.1f} ms, sonar {st['ping_ms']:.1f} ms (max {st['ping_ms_max']:.1f}), "
          f"write {st['write_ms']:.1f} ms", file=sys.stderr)
    return 0


def run(cfg: dict, pings: int, sink, plume: bool = True, fan: bool | None = None, info: dict | None = None,
        progress=None, cache: bool = True) -> dict:
    """``pings`` pings of ``cfg`` into ``sink``: scene, scatterers, bubble warm-up, then per ping 1 / ping_rate of
    bubble steps, the sonar's pose at that time (sonar.pose_source; trajectory time 0 = end of the warm-up), a ping.
    ``info`` gets the config hash, seed, labels and config (the sink writes it to its manifest). ``progress(i)`` is
    called after every ping. ``cache`` False: the scatterers are not cached (sweeps). Returns timing statistics."""
    t0 = time.perf_counter()
    scene = geometry.build_scene(cfg)
    sc = geometry.scatterers(cfg, scene, **({} if cache else {"cache_dir": None}))
    sim = None
    if config.get(cfg, "leak.enabled") and plume:
        sim = bubbles.BubbleSim(cfg, scene)
        sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    motion = pose.sensor_pose(cfg, scene)
    if config.get(cfg, "sonar.pose_source") == "ros":
        from .ros_bridge import RosPose
        motion = RosPose(motion, config.get(cfg, "sonar.ros_pose_topic"))
    son = sonar.Sonar(cfg, scene, sc, plume=sonar.plume_region(sim) if sim else None)
    boil = surface.Boil(cfg, sim) if sim is not None else None
    setup_s = time.perf_counter() - t0
    dt = 1.0 / float(config.get(cfg, "sonar.ping_rate_hz"))
    if info is not None:
        info.update({"config_hash": recorder.config_hash(cfg), "seed": cfg["seed"],
                     "labels": {"threshold": son.mask_threshold, **recorder.LABELS}, "config": cfg})
    t_start = sim.t if sim is not None else 0.0
    sink.static_transforms(t_start, [("oculus", "oculus_projected", R_SONAR_PROJECTED, (0, 0, 0))]
                           + ([] if motion.moving else [("tank", "oculus", son.rot, son.position)]))
    step_ms, ping_ms, write_ms, static_ms = [], [], [], []
    for i in range(pings):
        t = time.perf_counter()
        if sim is not None:
            sim.step(dt)
            wp.synchronize()
        stamp = sim.t if sim is not None else (i + 1) * dt
        if motion.moving:
            here = motion.at((i + 1) * dt)
            son.set_pose(here.position, here.rot)
        t1 = time.perf_counter()
        ping = son.ping(bubbles=sim, extra=boil.update(son.position) if boil is not None else ())
        t2 = time.perf_counter()
        sink.clock(stamp)
        if motion.moving:
            sink.transforms(stamp, [("tank", "oculus", ping.rot, ping.position)])
        sink.publish(recorder.sonar_message(i, stamp, ping, son, cfg, scene, sim, fan=fan))
        t3 = time.perf_counter()
        step_ms.append((t1 - t) * 1e3)
        ping_ms.append((t2 - t1) * 1e3)
        write_ms.append((t3 - t2) * 1e3)
        static_ms.append(ping.timing_ms.get("static_recompute", 0.0))
        if progress is not None:
            progress(i)
    return {"setup_s": setup_s, "pings": pings, "bubbles_ms": float(np.mean(step_ms)), "ping_ms": float(np.mean(ping_ms)),
            "ping_ms_max": float(np.max(ping_ms)), "write_ms": float(np.mean(write_ms)),
            "static_recomputes": int(np.count_nonzero(static_ms)), "static_recompute_ms": float(np.mean(static_ms)),
            "scatterers": son.n_scatterers, "plume_box_paths": son.n_dyn}
