"""`python -m uw pool [--gui]`: the M11 test pool. A BlueROV2 Heavy flown through ArduSub SITL (and BlueOS, Cockpit,
a gamepad) in a test tank (pool.size: 50 x 25 x 5 m, or the M11 10 x 10 x 5 m with --preset pool10); the onboard camera
goes out as a live H.264 stream. No bubbles, no sonar.

    python -m uw pool [--gui] [--duration S] [--snapshot T]... [--no-stream] [--stats-file F] [--profile P] [--set k=v]

The physics is not in this process: ``uw.rov.physics_proc`` runs the ArduSub JSON server (UDP physics.port) in a
child process, lockstep with SITL and paced to the wall clock, so SITL, BlueOS and the pilot see real time whatever
the renderer does. This loop only renders: every 1/camera.rate_hz it reads the latest vehicle pose, camera tilt and
lights from the child's shared memory, renders, applies the water (uw/camera_uw.py) and hands the frame to the
encoder (uw/video.py). A slow frame costs frame rate, never simulation time. Setup and the operator side: docs/pool.md.
"""
from __future__ import annotations

import argparse
import json
import logging
import signal
import time
from pathlib import Path

import numpy as np

from . import config
from .config import WS, get

log = logging.getLogger("uw.pool")
CAMERA_WARMUP_RENDERS = 10
SNAPSHOT_DIR = Path(WS) / "output" / "pool" / "snapshots"
VIEWS_DIR = Path(WS) / "output" / "pool" / "views"
VIEW_SETTLE_RENDERS = 12                  # renders per view before the frame is kept: DLSS and the denoiser settle


def tilt_of_pwm(cfg: dict, pwm: float) -> float:
    """Camera tilt [deg] of the tilt servo's PWM: linear between vehicle.camera.tilt_range_deg at 1100 / 1900 us."""
    lo, hi = (float(v) for v in get(cfg, "vehicle.camera.tilt_range_deg"))
    f = min(max((float(pwm) - 1100.0) / 800.0, 0.0), 1.0)
    return lo + f * (hi - lo)


def lights_of_pwm(pwm: float) -> float:
    """Lumen brightness 0..1 of the lights output's PWM (1100 us off, 1900 us full)."""
    return min(max((float(pwm) - 1100.0) / 800.0, 0.0), 1.0)


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="python -m uw pool", description=__doc__.splitlines()[0])
    ap.add_argument("--gui", action="store_true", help="open the Isaac Sim GUI (viewport: --gui-view)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many wall seconds (0: until stopped)")
    ap.add_argument("--snapshot", type=float, action="append", default=[], metavar="T",
                    help=f"T s after the first frame, save the streamed frame to {SNAPSHOT_DIR} (repeatable)")
    ap.add_argument("--no-stream", action="store_true", help="no video stream (stream.enabled off)")
    ap.add_argument("--view", choices=("onboard", "chase"), default="onboard",
                    help="the streamed camera: the ROV's own (default) or a third-person one behind it")
    ap.add_argument("--gui-view", choices=("chase", "onboard", "free"), default="chase",
                    help="with --gui, the viewport's camera: behind the ROV (default), the ROV's own (raw render, "
                         "no water), or Isaac's free perspective camera")
    ap.add_argument("--stats-file", default="", help="write run statistics (JSON) here on exit")
    ap.add_argument("--views", metavar="NAME", default="",
                    help="no physics, no stream: look at every COCO object (pool.objects) from --view-dist metres and "
                         "--view-az directions, and write the frames, the water applied, with Replicator's 2-D boxes "
                         f"to {VIEWS_DIR}/NAME (for tools/eval_views.py)")
    ap.add_argument("--view-dist", type=float, nargs="+", default=[2.0, 3.0, 4.0, 6.0, 8.0], metavar="M")
    ap.add_argument("--view-az", type=int, default=4, help="directions around each object (default %(default)s)")
    ap.add_argument("--view-height", type=float, default=1.0,
                    help="camera height above the object's top [m], negative for below it (default %(default)s; "
                         "never under 0.35 m from the floor)")
    ap.add_argument("--view-lights", type=float, default=0.0, metavar="F",
                    help="the ROV's Lumens at this fraction of full power during the views (default off)")
    config.add_args(ap.add_argument)
    return ap.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = config.from_args(args, config.POOL_CONFIG)
    if args.views:
        return main_views(cfg, args)
    from .rov.physics_proc import PhysicsProcess
    physics = PhysicsProcess(cfg).start()            # first: SITL can lock on while Kit boots, a busy port fails fast
    log.info("physics process up on UDP %s:%d; %s", get(cfg, "physics.bind"), physics.port,
             "DVL A50 on TCP %d, HTTP %d" % physics.dvl_ports if physics.dvl_ports else "no DVL")
    stop = {"flag": False}

    def _sig(_s, _f):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)
    app = None
    try:
        from .app import create_app            # before any omni import
        w, h = (int(v) for v in get(cfg, "camera.resolution"))
        app = create_app(gui=args.gui, width=w, height=h, refraction_bounces=int(get(cfg, "render.refraction_bounces")),
                         dlss=bool(get(cfg, "render.dlss")), translucency=bool(get(cfg, "render.translucency")),
                         reflections=bool(get(cfg, "render.reflections")))
        return run(app, cfg, args, physics, stop)
    finally:
        physics.close()
        if app is not None:
            app.close()


def main_views(cfg: dict, args) -> int:
    from .app import create_app
    w, h = (int(v) for v in get(cfg, "camera.resolution"))
    app = create_app(gui=False, width=w, height=h, refraction_bounces=int(get(cfg, "render.refraction_bounces")),
                     dlss=bool(get(cfg, "render.dlss")), translucency=bool(get(cfg, "render.translucency")),
                     reflections=bool(get(cfg, "render.reflections")))
    try:
        return run_views(app, cfg, args)
    finally:
        app.close()


def view_poses(cfg: dict, dists, n_az: int, height: float):
    """For every placed object: camera poses at ``dists`` metres (horizontal, to its centre) from ``n_az``
    directions, ``height`` above its top, aimed at its centre. Yields (object placement, distance, azimuth [deg],
    body position, body FLU -> ENU rotation, tilt [deg]); poses outside the pool or under the floor are skipped."""
    import math
    from . import pool_geometry
    L, W, _ = (float(v) for v in get(cfg, "pool.size"))
    wl = float(get(cfg, "pool.water_level"))
    cam_off = np.asarray(get(cfg, "vehicle.camera.position_body"), float)
    for pl, solid in pool_geometry.placed_objects(cfg):
        cx, cy, z0 = pl["xyz"]
        top = z0 + pl["size_m"][2]
        zc = max(top + float(height), 0.35)          # the ROV's camera is ~0.2 m above its skids
        centre_z = z0 + pl["size_m"][2] / 2
        for d in dists:
            for k in range(int(n_az)):
                az = pl["yaw_deg"] + 360.0 * k / int(n_az) + 30.0     # off the object's own axes
                a = math.radians(az)
                cam = np.array([cx + d * math.cos(a), cy + d * math.sin(a), zc])
                if not (0.4 < cam[0] < L - 0.4 and 0.4 < cam[1] < W - 0.4 and 0.3 < cam[2] < wl - 0.3):
                    continue
                yaw = math.atan2(cy - cam[1], cx - cam[0])
                rot = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0],
                                [0.0, 0.0, 1.0]])
                body = cam - rot @ cam_off
                tilt = math.degrees(math.atan2(centre_z - cam[2], float(d)))
                yield pl, float(d), float(az % 360.0), body, rot, float(np.clip(tilt, -45.0, 45.0))


def run_views(app, cfg: dict, args) -> int:
    import omni.usd

    from . import camera_uw, pool_scene
    from .sinks import write_png

    import omni.timeline
    stage = omni.usd.get_context().get_stage()
    info = pool_scene.build(stage, cfg)
    timeline = omni.timeline.get_timeline_interface()     # as run(): the annotators deliver only while it plays
    timeline.play()
    timeline.commit()
    app.update()
    import omni.replicator.core as rep
    rp = rep.create.render_product(info.camera, info.camera_resolution, name="uw_pool_views")
    rgb_annot, _ = _annotator(rep, "rgb")
    dist_annot, _ = _annotator(rep, "distance_to_camera")
    box_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight",
                                                    init_params={"semanticTypes": ["class"]})
    for a in (rgb_annot, dist_annot, box_annot):
        a.attach([rp])
    post = camera_uw.WaterPost(cfg)
    pool_scene.set_lights(stage, info, float(args.view_lights))
    out = VIEWS_DIR / args.views
    out.mkdir(parents=True, exist_ok=True)
    index = {"config": {"pool": get(cfg, "pool.size"), "turbidity_per_m": float(get(cfg, "water.turbidity_per_m")),
                        "lights": float(args.view_lights), "view_height_m": float(args.view_height),
                        "visibility_m": camera_uw.visibility_m(cfg), "resolution": list(info.camera_resolution)},
             "views": []}
    n = 0
    for pl, d, az, body, rot, tilt in view_poses(cfg, args.view_dist, args.view_az, args.view_height):
        pool_scene.set_vehicle(stage, body, rot)
        pool_scene.set_tilt(stage, tilt)
        for _ in range(VIEW_SETTLE_RENDERS):
            app.update()
        rgb, dist = rgb_annot.get_data(), dist_annot.get_data()
        if not getattr(rgb, "size", 0) or not getattr(dist, "size", 0):
            log.warning("views: no image for %s at %.1f m", pl["name"], d)
            continue
        frame = post(rgb, dist)
        name = f"{n:04d}_{pl['name']}_{d:.0f}m_{az:03.0f}.png"
        write_png(out / name, frame)
        boxes = box_annot.get_data()
        labels = {int(k): (v.get("class", "") if isinstance(v, dict) else str(v))
                  for k, v in (boxes.get("info", {}).get("idToLabels", {}) or {}).items()}
        gt = []
        for row in boxes.get("data", []):
            cls = labels.get(int(row["semanticId"]), "")
            if not cls:
                continue
            gt.append({"coco": cls, "box": [float(row["x_min"]), float(row["y_min"]), float(row["x_max"]),
                                             float(row["y_max"])], "occlusion": float(row["occlusionRatio"])})
        index["views"].append({"file": name, "object": pl["name"], "coco": pl["coco"], "distance_m": d,
                               "azimuth_deg": az, "tilt_deg": tilt, "gt": gt})
        n += 1
        if n % 20 == 0:
            log.info("views: %d frames", n)
    timeline.stop()
    (out / "index.json").write_text(json.dumps(index, indent=1))
    log.info("views: %d frames and their boxes in %s (water: turbidity %.2f /m, visibility %.1f m)", n, out,
             index["config"]["turbidity_per_m"], index["config"]["visibility_m"])
    return 0


def _annotator(rep, name: str):
    """A GPU annotator when this Replicator offers it (no host round trip before the water post), else host."""
    try:
        return rep.AnnotatorRegistry.get_annotator(name, device="cuda", do_array_copy=False), True
    except TypeError:
        return rep.AnnotatorRegistry.get_annotator(name), False


def run(app, cfg: dict, args, physics, stop: dict) -> int:  # noqa: C901 - one flat loop
    import omni.timeline
    import omni.usd
    import warp as wp

    from . import camera_uw, pool_scene
    from .app import render_settings_report
    from .sinks import write_png

    t_setup = time.perf_counter()
    wp.config.quiet = True
    stage = omni.usd.get_context().get_stage()
    info = pool_scene.build(stage, cfg)
    if info.vehicle_asset is None:
        log.warning("vehicle asset %s not found: a box stands in (python -m uw py tools/fetch_bluerov2.py)",
                    get(cfg, "vehicle.asset"))
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    timeline.commit()
    app.update()
    import omni.replicator.core as rep
    rp = rep.create.render_product(info.chase if args.view == "chase" else info.camera, info.camera_resolution,
                                   name="uw_pool_camera")
    rgb_annot, on_gpu = _annotator(rep, "rgb")
    dist_annot, _ = _annotator(rep, "distance_to_camera")
    rgb_annot.attach([rp])
    dist_annot.attach([rp])
    if args.gui and args.gui_view != "free":
        try:
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().camera_path = info.chase if args.gui_view == "chase" else info.camera
        except Exception as exc:  # noqa: BLE001 - only the GUI's convenience
            log.warning("GUI viewport camera: %r", exc)
    post = camera_uw.WaterPost(cfg)
    log.info("water: turbidity %.2f /m, a dark target visible to about %.1f m", float(get(cfg, "water.turbidity_per_m")),
             camera_uw.visibility_m(cfg))
    w, h = info.camera_resolution
    rate = float(get(cfg, "camera.rate_hz"))
    stream = None
    if get(cfg, "stream.enabled") and not args.no_stream:
        from .video import VideoStream
        stream = VideoStream(str(get(cfg, "stream.url")), w, h, rate, str(get(cfg, "stream.encoder")),
                             int(get(cfg, "stream.bitrate_kbps")), threads=int(get(cfg, "stream.threads"))).start()
    tilt_servo = int(get(cfg, "vehicle.camera.tilt_servo"))
    lights_servo = int(get(cfg, "vehicle.lights.servo"))
    idle_pwm = np.full(16, 1500.0)
    idle_pwm[lights_servo - 1] = 1100.0
    setup_s = time.perf_counter() - t_setup
    log.info("setup %.1f s (annotators on %s); render settings %s", setup_s, "GPU" if on_gpu else "host",
             render_settings_report())

    period = 1.0 / rate
    every = float(get(cfg, "sim.stats_every_s"))
    shots = sorted(args.snapshot)
    times = {"render": [], "post": [], "loop": []}
    count = {"frames": 0, "empty": 0, "late": 0}
    exit_reason = "stopped"
    wall0 = time.perf_counter()
    first_frame = None
    next_t = wall0
    last_stats = (wall0, 0, physics.stats()["packets"])
    last_tilt = last_lights = None
    try:
        while not stop["flag"]:
            if physics.error is not None:
                raise physics.error
            t_loop = time.perf_counter()
            p, rot = physics.body_pose()
            # before SITL's first packet the outputs are not ArduSub's yet: camera level, lights off
            pwm = physics.pwm() if physics.stats()["packets"] > 0 else idle_pwm
            pool_scene.set_vehicle(stage, p, rot)
            tilt = tilt_of_pwm(cfg, pwm[tilt_servo - 1] or 1500.0)
            if tilt != last_tilt:
                pool_scene.set_tilt(stage, tilt)
                last_tilt = tilt
            lights = lights_of_pwm(pwm[lights_servo - 1] or 1100.0)
            if lights != last_lights:
                pool_scene.set_lights(stage, info, lights)
                last_lights = lights
            t_r = time.perf_counter()
            app.update()
            times["render"].append((time.perf_counter() - t_r) * 1e3)
            rgb, dist = rgb_annot.get_data(), dist_annot.get_data()
            if not getattr(rgb, "size", 0) or not getattr(dist, "size", 0):
                count["empty"] += 1
                if count["empty"] == CAMERA_WARMUP_RENDERS + 1 and count["frames"] == 0:
                    log.warning("camera: no image after %d renders", count["empty"])
            else:
                t_p = time.perf_counter()
                frame = post(rgb, dist)
                times["post"].append((time.perf_counter() - t_p) * 1e3)
                if stream is not None:
                    stream.push(frame)
                count["frames"] += 1
                if first_frame is None:
                    first_frame = time.perf_counter()
                while shots and time.perf_counter() - first_frame >= shots[0]:
                    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
                    path = SNAPSHOT_DIR / f"pool_{shots.pop(0):07.2f}s.png"
                    write_png(path, frame)
                    log.info("snapshot %s (vehicle at %s, tilt %.0f deg, lights %.0f %%)", path,
                             np.round(p, 2).tolist(), tilt, 100 * lights)
            times["loop"].append((time.perf_counter() - t_loop) * 1e3)
            now = time.perf_counter()
            if now - last_stats[0] >= every:
                st = physics.stats()
                fps = (count["frames"] - last_stats[1]) / (now - last_stats[0])
                pk = (st["packets"] - last_stats[2]) / (now - last_stats[0])
                tail = {k: float(np.mean(v[-200:])) if v else 0.0 for k, v in times.items()}
                dvl = st["dvl"]
                log.info("%.1f fps (render %.1f, water %.1f, loop %.1f ms); physics %.0f packets/s, sim t %.1f s, "
                         "%d stalls; depth %.2f m; DVL %s; video %s", fps, tail["render"], tail["post"], tail["loop"],
                         pk, st["t"], st["stalls"], float(get(cfg, "pool.water_level")) - float(p[2]),
                         f"{dvl['reports']} reports ({dvl['locked']} locked), {dvl['clients']} clients"
                         if physics.dvl_ports else "off", stream.stats() if stream else "off")
                last_stats = (now, count["frames"], st["packets"])
            if args.duration and now - wall0 >= args.duration:
                exit_reason = "duration"
                break
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -period:                    # behind by more than a frame: drop the backlog, no catch-up
                count["late"] += 1
                next_t = time.perf_counter()
    except Exception as exc:  # noqa: BLE001 - reported in the summary and the log
        exit_reason = f"exception: {exc!r}"
        log.exception("pool loop stopped")
    finally:
        wall = time.perf_counter() - wall0
        summary = {"exit_reason": exit_reason, "setup_s": setup_s, "wall_time_s": wall,
                   "fps": count["frames"] / wall if wall > 0 else 0.0, "counts": count,
                   "stages_ms": {k: {"n": len(v), "mean": round(float(np.mean(v)), 3),
                                     "p95": round(float(np.percentile(v, 95)), 3), "max": round(float(np.max(v)), 3)}
                                 for k, v in times.items() if v},
                   "physics": physics.stats(), "video": stream.stats() if stream else None,
                   "camera": {"resolution": [w, h], "rate_hz": rate}, "render_settings": render_settings_report(),
                   "vehicle_asset": info.vehicle_asset}
        log.info("stopped (%s): %.1f s, %d frames (%.1f fps), physics %s", exit_reason, wall, count["frames"],
                 summary["fps"], summary["physics"])
        if args.stats_file:
            with open(args.stats_file, "w") as fh:
                json.dump(summary, fh, indent=1, default=str)
        if stream is not None:
            stream.close()
        timeline.stop()
    return 0 if not exit_reason.startswith("exception") else 1
