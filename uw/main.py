"""`python -m uw sim [--gui]`: the Isaac app. USD stage from the geometry, bubbles in the RTX scene, sonar in the loop,
RTX camera, outputs through a SensorSink (ROS 2 by default).

    python -m uw sim [--gui] [--sink ros|file|none] [--out DIR] [--duration S] [--no-realtime] [--no-camera]
        [--view side45|top|sonar] [--screenshot T]... [--stats-file F] [--profile ...] [--preset ...] [--set a.b=c]

Loop: one tick = one bubble substep (plume.dt, 1/240 s). The bubbles step when something reads them (ping, bubble
render, /sim/plume), all pending substeps at once as one CUDA graph; every 1/sonar.ping_rate_hz the sonar pings
(the same pings as `sonar-only` for the same config and seed); every 1/camera.rate_hz an RTX render gives the camera
image; with --gui the viewport also renders every 1/sim.gui_fps. /clock and the other topics at their ros.*
rates. Sim time = time since the leak started: the plume warm-up (plume.warmup_s) runs before the first tick.
Real time (sim.realtime, off with --no-realtime): the loop sleeps when ahead of the wall clock; the watchdog reports
pings over sonar.ping_budget_ms.static and sim time more than sim.max_lag_s behind. It never skips work: sim time stays
exact and a run that cannot keep up gets slower than real time (the RTF is logged every 10 s).
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import time

import numpy as np

from . import config
from .config import get

log = logging.getLogger("uw.sim")
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # optical axes in the body frame
CAMERA_WARMUP_RENDERS = 10      # the annotator returns empty buffers for the first renders (pipeline depth)


class RateTimer:
    """Fires when sim time crosses k/rate (average rate exact, jitter = one tick). Copied from ~/isaac_ws; here the
    first event is at one period (k = 1), not at t = 0: the first ping comes after 1/ping_rate of bubble steps, as in
    sonar-only."""

    def __init__(self, rate: float) -> None:
        self.period = 1.0 / float(rate)
        self.next = self.period

    def due(self, t: float) -> bool:
        if t + 1e-9 >= self.next:
            self.next += self.period
            if self.next < t:            # after a reset / stall: resync
                self.next = t + self.period
            return True
        return False


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="python -m uw sim", description=__doc__.splitlines()[0])
    ap.add_argument("--gui", action="store_true", help="open the Isaac Sim GUI (default headless)")
    ap.add_argument("--sink", default="ros", choices=("ros", "file", "none"))
    ap.add_argument("--out", default="", help="dataset directory for --sink file")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many sim seconds (0: run until stopped)")
    ap.add_argument("--no-realtime", action="store_true", help="run as fast as possible (sim.realtime off)")
    ap.add_argument("--no-camera", action="store_true", help="no camera render product (camera.enabled off)")
    ap.add_argument("--view", default="", help="GUI viewport camera at start: side45 | top | sonar (sim.view)")
    ap.add_argument("--screenshot", type=float, action="append", default=[], metavar="T",
                    help="at sim time T after the start: save the GUI window (--gui) and the camera image to "
                         "output/sim/screenshots/ (repeatable)")
    ap.add_argument("--stats-file", default="", help="write run statistics (JSON) here on exit")
    config.add_args(ap.add_argument)
    args = ap.parse_args(argv)
    if args.sink == "file" and not args.out:
        ap.error("--sink file needs --out DIR")
    args.argv = list(argv)
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = config.from_args(args)
    from .app import create_app            # before any omni import
    app = create_app(gui=args.gui, refraction_bounces=int(get(cfg, "sim.max_refraction_bounces")))
    stop = {"flag": False}

    def _sig(_s, _f):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)
    try:
        return run(app, cfg, args, stop)
    finally:
        app.close()


class Stats:
    """Per-stage wall times [ms]: mean and max."""

    def __init__(self):
        self.t: dict[str, list] = {}

    def add(self, key: str, ms: float) -> None:
        self.t.setdefault(key, []).append(ms)

    def summary(self) -> dict:
        return {k: {"n": len(v), "mean_ms": round(float(np.mean(v)), 3), "max_ms": round(float(np.max(v)), 3)}
                for k, v in self.t.items() if v}


def static_frames(cfg: dict, son, moving: bool = False) -> list:
    """The fixed frames. A fixed sonar: tank -> oculus and tank -> camera; a moving one: the camera rides with the
    sonar (oculus -> camera, its configured offset), and tank -> oculus goes on /tf every ping."""
    from .geometry import rot_rpy_deg, sonar_rotation
    from .ros_bridge import FRAMES, R_SONAR_PROJECTED
    out = [(FRAMES["sonar"], FRAMES["sonar_projected"], R_SONAR_PROJECTED, (0.0, 0.0, 0.0))]
    if not moving:
        out.insert(0, (FRAMES["world"], FRAMES["sonar"], son.rot, son.position))
    if get(cfg, "camera.enabled"):
        cam_rot, cam_pos = rot_rpy_deg(get(cfg, "camera.rpy_deg")), np.asarray(get(cfg, "camera.position"), float)
        if moving:
            s_rot, s_pos = sonar_rotation(cfg), np.asarray(get(cfg, "sonar.position"), float)
            out.append((FRAMES["sonar"], FRAMES["camera"], s_rot.T @ cam_rot, s_rot.T @ (cam_pos - s_pos)))
        else:
            out.append((FRAMES["world"], FRAMES["camera"], cam_rot, cam_pos))
        out.append((FRAMES["camera"], FRAMES["camera_optical"], R_BODY_OPTICAL, (0.0, 0.0, 0.0)))
    return out


def run(app, cfg: dict, args, stop: dict) -> int:  # noqa: C901 - the loop is long but flat
    import omni.timeline
    import omni.usd
    import torch
    import warp as wp

    from . import bubbles, geometry, pose, recorder, sinks, sonar, usd_scene
    from .app import render_settings_report
    from .ros_bridge import FRAMES

    t_setup = time.perf_counter()
    wp.config.quiet = True
    scene = geometry.build_scene(cfg)
    sc = geometry.scatterers(cfg, scene)
    sim = None
    if get(cfg, "leak.enabled"):           # the same order as sonar-only: same bubbles, same pings
        sim = bubbles.BubbleSim(cfg, scene)
        sim.warmup(float(get(cfg, "plume.warmup_s")))
    son = sonar.Sonar(cfg, scene, sc, plume=sonar.plume_region(sim) if sim else None)
    motion = pose.sensor_pose(cfg, scene)
    if get(cfg, "sonar.pose_source") == "ros":
        from .ros_bridge import RosPose
        motion = RosPose(motion, get(cfg, "sonar.ros_pose_topic"))
    stage = omni.usd.get_context().get_stage()
    info = usd_scene.build(stage, cfg, scene)
    n_vert = usd_scene.check_geometry(stage, scene, son.mesh.points.numpy(), son.mesh.indices.numpy())
    log.info("USD stage: %d meshes; the occluders' USD and Warp BVH meshes are identical (%d vertices)",
             len(info.meshes), n_vert)
    inst = None
    if sim is not None and get(cfg, "sim.bubbles_rendered"):
        inst = usd_scene.BubbleInstancer(stage, cfg, sim, int(get(cfg, "sim.bubble_stride")),
                                         fabric_stage_id=omni.usd.get_context().get_stage_id())
        inst.update()
    # Replicator captures only while the timeline plays, and only if it already played when the annotator attached
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    timeline.commit()
    app.update()
    camera = None
    if info.camera and not args.no_camera:
        import omni.replicator.core as rep
        rp = rep.create.render_product(info.camera, info.camera_resolution, name="uw_camera")
        camera = rep.AnnotatorRegistry.get_annotator("rgb")
        camera.attach([rp])
    overlay = None
    if args.gui:
        from . import gui
        gui.set_view(cfg, son, args.view or get(cfg, "sim.view"), get(cfg, "sim.viewport_resolution"))
        if get(cfg, "sim.overlay_fan") or get(cfg, "sim.overlay_returns"):
            overlay = gui.Overlay(cfg, son)
    live = args.sink == "ros"
    if args.sink == "ros":
        sink = sinks.make_sink("ros", plume_markers=int(get(cfg, "ros.plume_markers")))
    elif args.sink == "file":
        sink = sinks.make_sink("file", out_dir=args.out, info={
            "name": args.out, "command": "python -m uw sim " + " ".join(args.argv),
            "config_hash": recorder.config_hash(cfg), "seed": cfg["seed"],
            "labels": {"threshold": son.mask_threshold, **recorder.LABELS}, "config": cfg})
    else:
        sink = sinks.make_sink("none")
    dt = sim.dt if sim is not None else float(get(cfg, "plume.dt"))
    t0 = sim.t if sim is not None else 0.0
    sink.static_transforms(t0, static_frames(cfg, son, motion.moving))
    setup_s = time.perf_counter() - t_setup
    log.info("setup %.1f s; render settings %s", setup_s, render_settings_report())

    realtime = bool(get(cfg, "sim.realtime")) and not args.no_realtime
    budget = get(cfg, f"sonar.ping_budget_ms.{'mobile' if motion.moving else 'static'}")
    budget = float("inf") if budget is None else float(budget)
    max_lag = float(get(cfg, "sim.max_lag_s"))
    rates = {"ping": float(get(cfg, "sonar.ping_rate_hz")), "camera": float(get(cfg, "camera.rate_hz")),
             "gui": float(get(cfg, "sim.gui_fps")), "clock": float(get(cfg, "ros.clock_hz")),
             "plume": float(get(cfg, "ros.plume_hz")), "leak": float(get(cfg, "ros.leak_hz")), "idle": 10.0,
             "bubbles": float(get(cfg, "sim.bubble_render_hz"))}
    timers = {k: RateTimer(r) for k, r in rates.items()}
    shots = sorted(args.screenshot)
    shot_dir = os.path.join(config.WS, "output", "sim", "screenshots")
    pending_shot = None
    K = info.camera_K
    stats = Stats()
    count = {"ticks": 0, "pings": 0, "renders": 0, "camera": 0, "camera_empty": 0, "over_budget": 0}
    worst = {"ping_ms": 0.0, "lag_s": 0.0}
    k = pending = 0
    batch = max(1, int(round(1.0 / (rates["ping"] * dt))))      # substeps per ping
    wall0 = time.perf_counter()
    last_rtf = (wall0, 0.0)
    last_warn = -1e9
    log.info("running: tick %.2f ms, sonar %.0f Hz, camera %s, gui %s, sink %s, realtime %s", dt * 1e3, rates["ping"],
             f"{rates['camera']:.0f} Hz" if camera is not None else "off",
             f"{rates['gui']:.0f} fps" if args.gui else "off", args.sink, realtime)
    exit_reason = "stopped"
    try:
        while app.is_running() and not stop["flag"]:
            k += 1
            pending += 1
            t_loop = k * dt
            t = t0 + t_loop
            ping_due = timers["ping"].due(t_loop)
            cam_due = camera is not None and timers["camera"].due(t_loop)
            gui_due = args.gui and timers["gui"].due(t_loop)
            idle_due = camera is None and not args.gui and timers["idle"].due(t_loop)
            bubbles_due = inst is not None and timers["bubbles"].due(t_loop)
            plume_due = live and sim is not None and timers["plume"].due(t_loop)
            # the bubbles step only when something reads them, all pending substeps at once (one CUDA graph)
            if sim is not None and (ping_due or plume_due or (bubbles_due and (cam_due or gui_due))
                                    or pending >= batch):
                t_a = time.perf_counter()
                sim.step(pending * dt)
                stats.add("bubble_steps", (time.perf_counter() - t_a) * 1e3)
                pending = 0
            if timers["clock"].due(t_loop):
                sink.clock(t)
            if ping_due:
                t_w = time.perf_counter()
                wp.synchronize_device(son.device)        # queued GPU work (bubble graph, last render) finishes first
                t_b = time.perf_counter()
                stats.add("gpu_wait_before_ping", (t_b - t_w) * 1e3)
                if motion.moving:
                    here = motion.at(t_loop)
                    son.set_pose(here.position, here.rot)
                ping = son.ping(bubbles=sim)
                t_c = time.perf_counter()
                if motion.moving:
                    sink.transforms(t, [(FRAMES["world"], FRAMES["sonar"], ping.rot, ping.position)])
                sink.publish(recorder.sonar_message(count["pings"], sim.t if sim is not None else t, ping, son, cfg,
                                                    scene, sim, fan=True if live else None))
                if overlay is not None:
                    overlay.update(ping.image, ping.position, ping.rot)
                t_d = time.perf_counter()
                count["pings"] += 1
                for key, v in ping.timing_ms.items():
                    stats.add("ping" if key == "total" else f"ping_{key}", v)
                stats.add("ping_wall", (t_c - t_b) * 1e3)
                stats.add("ping_publish", (t_d - t_c) * 1e3)
                worst["ping_ms"] = max(worst["ping_ms"], ping.timing_ms["total"])
                if ping.timing_ms["total"] > budget:
                    count["over_budget"] += 1
                    log.warning("watchdog: ping %d took %.0f ms (budget %.0f ms)", count["pings"],
                                ping.timing_ms["total"], budget)
            if cam_due or gui_due or idle_due:
                if motion.moving and (cam_due or gui_due):
                    here = motion.at(t_loop)
                    usd_scene.set_rig_pose(stage, cfg, here.position, here.rot)
                if bubbles_due and (cam_due or gui_due):
                    for key, v in inst.update().items():
                        if key.endswith("_ms"):
                            stats.add(f"bubbles_{key}", v)
                    stats.add("bubbles_rendered", inst.n)
                t_r = time.perf_counter()
                app.update()
                stats.add("render" if (cam_due or gui_due) else "app_update", (time.perf_counter() - t_r) * 1e3)
                count["renders"] += 1
                if pending_shot is not None:
                    from . import gui
                    try:
                        log.info("screenshot: %s and %s", pending_shot, gui.wait_screenshot(pending_shot))
                    except Exception as exc:  # noqa: BLE001 - a failed screenshot must not stop the sim
                        log.warning("screenshot %s: %r", pending_shot, exc)
                    pending_shot = None
            if cam_due:
                t_e = time.perf_counter()
                rgb = camera.get_data()
                if getattr(rgb, "size", 0):
                    rgb = np.ascontiguousarray(rgb[..., :3])
                    sink.publish(sinks.Message("camera", count["camera"], t, images={"rgb": rgb},
                                               meta={"K": K.tolist()}))
                    count["camera"] += 1
                    stats.add("camera_read_publish", (time.perf_counter() - t_e) * 1e3)
                else:
                    count["camera_empty"] += 1
                    if count["camera_empty"] == CAMERA_WARMUP_RENDERS + 1 and count["camera"] == 0:
                        import omni.replicator.core as rep
                        log.warning("camera: no image after %d renders (replicator %s, timeline playing %s)",
                                    count["camera_empty"], rep.orchestrator.get_status(), timeline.is_playing())
            if plume_due:
                snap = sim.snapshot()
                sink.publish(sinks.Message("plume", k, t, arrays={"position": snap["pos"],
                                                                   "diameter": 2.0 * snap["radius"]}))
            if live and scene.leak is not None and timers["leak"].due(t_loop):
                sink.publish(sinks.Message("leak", k, t, meta={
                    "position": scene.leak.hole.tolist(), "direction": scene.leak.direction.tolist(),
                    "flow_lpm": sim.source.q_gas * 6e4 if sim is not None else 0.0}))
            if shots and t_loop + 1e-9 >= shots[0]:
                os.makedirs(shot_dir, exist_ok=True)
                stem = os.path.join(shot_dir, f"{'+'.join(args.preset) or 'default'}_t{shots.pop(0):g}")
                if args.gui:
                    from . import gui
                    pending_shot = stem + "_gui.png"
                    gui.screenshot(pending_shot)
                if camera is not None and count["camera"]:
                    sinks.write_png(stem + "_camera.png", rgb)
            # pacing and watchdog
            now = time.perf_counter()
            ahead = t_loop - (now - wall0)
            if realtime and ahead > 0.002:
                time.sleep(ahead)
            elif -ahead > max_lag:
                worst["lag_s"] = max(worst["lag_s"], -ahead)
                if realtime and now - last_warn > 5.0:
                    log.warning("watchdog: sim time %.2f s behind the wall clock", -ahead)
                    last_warn = now
            if now - last_rtf[0] >= 10.0:
                log.info("t %.1f s, RTF %.2f, %d pings (%d over budget), %d camera frames", t_loop,
                         (t_loop - last_rtf[1]) / (now - last_rtf[0]), count["pings"], count["over_budget"],
                         count["camera"])
                last_rtf = (now, t_loop)
            if args.duration and t_loop + 1e-9 >= args.duration:
                exit_reason = "duration"
                break
    except Exception as exc:  # noqa: BLE001 - e.g. Kit shutting down under us after SIGTERM
        exit_reason = f"exception: {exc!r}"
        log.exception("main loop stopped")
    finally:
        wall = time.perf_counter() - wall0
        count["ticks"] = k
        free, total = torch.cuda.mem_get_info()
        summary = {"exit_reason": exit_reason, "setup_s": setup_s, "sim_time_s": k * dt, "wall_time_s": wall,
                   "rtf": k * dt / wall if wall > 0 else 0.0, "realtime": realtime, "gui": args.gui, "sink": args.sink,
                   "counts": count, "worst": worst, "ping_budget_ms": budget, "stages": stats.summary(),
                   "gpu_used_mib_all_processes": (total - free) / 2 ** 20,
                   "torch_peak_mib": torch.cuda.max_memory_allocated() / 2 ** 20,
                   "render_settings": render_settings_report()}
        log.info("stopped (%s): sim %.1f s, wall %.1f s, RTF %.2f, %d pings (%d over budget), %d camera frames",
                 exit_reason, k * dt, wall, summary["rtf"], count["pings"], count["over_budget"], count["camera"])
        if args.stats_file:
            with open(args.stats_file, "w") as fh:
                json.dump(summary, fh, indent=1, default=str)
        sink.close()
        if overlay is not None:
            overlay.destroy()
        timeline.stop()
    return 0 if not exit_reason.startswith("exception") else 1
