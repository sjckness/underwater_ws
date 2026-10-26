"""Dataset sweeps: sample scenarios, generate them with sonar-only (resumable, parallel), write the manifest and the
dataset card.

    python3 tools/sweep.py run [--sweep config/sweeps/v0.2.yaml] [--root DIR] [--workers N]             (host)
    python3 tools/sweep.py sample | status | finalize [--sweep ...] [--root DIR]                            (host)
    python -m uw py tools/sweep.py worker --root DIR [--max-scenarios N]                        (container, by run)

``sample`` draws the scenarios of a sweep file (config/sweeps/; deterministic for its seed) into
<root>/scenarios.json: per scenario an id, a kind (leak | negative | distractor), presets, --set overrides (the
seed included) and the sampled values, and its split (train/val/test, stratified by kind, pipe preset and flow
decade). The root defaults to <runtime data_root>/<sweep name>.

``run`` (the host orchestrator) samples if needed and keeps ``workers`` worker containers busy until every scenario is
done; each worker claims scenarios one at a time (an O_EXCL claim file) and writes <root>/<id>/ping_xxxxx.* through the
flat FileSink, then DONE (or FAILED with the traceback). Resumable: a scenario without DONE is redone from scratch.
It pauses (stops its workers, which frees the GPU) while an f1tenth run is active, while <root>/PAUSE exists, or
while / has less than ``--min-free-root-gb`` free (the f1tenth Isaac precheck needs 25 GB), and resumes after.
At the end it writes <root>/manifest.json and <root>/README.md (the dataset card).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))
from uw import config, geometry, pose  # noqa: E402

F1TENTH = re.compile(r"sim_isaac\.sh|phase5_bringup|jazzy_parity|isaac_ws/launch\.sh|f1t_isaac")
CHUNK = 20                       # scenarios per worker process (then a fresh one: no memory creep)
MIN_FREE_SYSTEM_GB = 5.0         # pause below this on /, whatever disk the dataset is on (container, caches, logs)


# ----------------------------------------------------------------------------------------------- sampling

def draw(rng: np.random.Generator, spec):
    """One value of a distribution spec (see config/sweeps/v0.2.yaml)."""
    if isinstance(spec, list):
        return float(rng.uniform(spec[0], spec[1]))
    if isinstance(spec, dict) and "log" in spec:
        lo, hi = spec["log"]
        return float(math.exp(rng.uniform(math.log(lo), math.log(hi))))
    if isinstance(spec, dict) and "int" in spec:
        return int(rng.integers(spec["int"][0], spec["int"][1] + 1))
    if isinstance(spec, dict) and "choice" in spec:
        keys = list(spec["choice"])
        w = np.array([float(spec["choice"][k]) for k in keys])
        return keys[int(rng.choice(len(keys), p=w / w.sum()))]
    return spec


def _fmt(v) -> str:
    if isinstance(v, bool):
        return str(v).lower()
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join(f"{k}: {_fmt(x)}" for k, x in v.items()) + "}"
    return str(v)


WALL_YAW = {"x0": 0.0, "x1": 180.0, "y0": 90.0, "y1": -90.0}     # yaw that looks into the tank from each wall


def per_size(spec, size: str):
    """``spec``, or its entry for the size preset when it is given per size ({tank: ..., basin: ...})."""
    return spec[size] if isinstance(spec, dict) and size in spec else spec


def place_leak(rng: np.random.Generator, pl: dict, size: str, dims, wall: str, leak_margin: float, sets: dict):
    """Puts the leak ``leak_from_wall_m`` from the sonar's wall: the pipe (running along y) at that x for an x wall,
    the leak at that place along the pipe for a y wall; capped so that the pipe or the leak keeps its margin."""
    L, W = dims
    lo, hi = per_size(pl["leak_from_wall_m"], size)
    if wall in ("x0", "x1"):
        m = float(pl["pipe_margin_m"])
        d = float(rng.uniform(min(lo, L - m), min(hi, L - m)))
        sets["pipe.x"] = d if wall == "x0" else L - d
    else:
        d = float(rng.uniform(min(lo, W - leak_margin), min(hi, W - leak_margin)))
        sets["leak.position_along"] = d if wall == "y0" else W - d
        sets["pipe.x"] = float(rng.uniform(float(pl["pipe_margin_m"]), L - float(pl["pipe_margin_m"])))


def place_sonar(rng: np.random.Generator, pl: dict, size: str, dims, wall: str, leak_xy) -> list:
    """A sonar position on a pole ``standoff_m`` from its wall, ``lateral_m`` to the side of the leak, at ``z_m``."""
    L, W = dims
    standoff, lateral, z = draw(rng, pl["standoff_m"]), draw(rng, per_size(pl["lateral_m"], size)), \
        draw(rng, per_size(pl["z_m"], size))
    if wall in ("x0", "x1"):
        return [standoff if wall == "x0" else L - standoff, float(np.clip(leak_xy[1] + lateral, 0.3, W - 0.3)), z]
    return [float(np.clip(leak_xy[0] + lateral, 0.3, L - 0.3)), standoff if wall == "y0" else W - standoff, z]


def leak_site(presets: list, sets: dict) -> np.ndarray:
    """Where the leak is (hole, or the vents' mean) in the scene drawn so far; the same place for a scenario
    without a leak (where it would be)."""
    cfg = config.load(presets=presets, overrides=[f"{k}={_fmt(v)}" for k, v in sets.items()] + ["leak.enabled=true"])
    return pose.leak_target(geometry.build_scene(cfg, full=False))


def view(rng: np.random.Generator, fx: dict, mode: dict, wall: str, position, target, aimed: bool) -> dict:
    """One fixed view. Aimed: the leak (``aim_height_m`` above it) at a random place inside the fan, at most
    ``aim_bearing_frac`` of the half fan and ``aim_elevation_frac`` of the half aperture off its centre (halved
    bearing while the target is too steep for it); else random pitch and yaw from the wall's inward look."""
    roll = float(draw(rng, fx["roll_deg"]))
    out = {"position": [float(v) for v in position], "roll_deg": roll, "aimed": aimed}
    if aimed:
        b = float(rng.uniform(-1, 1)) * float(fx["aim_bearing_frac"]) * float(mode["fov_h_deg"]) / 2
        e = float(rng.uniform(-1, 1)) * float(fx["aim_elevation_frac"]) * float(mode["fov_v_deg"]) / 2
        h = draw(rng, fx["aim_height_m"])
        t = np.asarray(target, float) + [0.0, 0.0, h]
        for _ in range(10):
            pitch, yaw = pose.aim(position, t, roll, b, e)
            _, bb, ee = pose.in_sonar_frame(position, geometry.rot_rpy_deg([roll, pitch, yaw]), t)
            if abs(bb - b) < 0.05 and abs(ee - e) < 0.05:
                break
            b *= 0.5
        out.update(pitch_deg=pitch, yaw_deg=yaw, aim={"bearing_deg": b, "elevation_deg": e, "height_m": h})
    else:
        out.update(pitch_deg=draw(rng, fx["pitch_deg"]), yaw_deg=WALL_YAW[wall] + draw(rng, fx["yaw_deg"]))
    r, b, e = pose.in_sonar_frame(position, geometry.rot_rpy_deg([roll, out["pitch_deg"], out["yaw_deg"]]), target)
    out.update(leak_range_m=r, leak_bearing_deg=b, leak_elevation_deg=e)
    return out


def sample_scenario(sw: dict, index: int, rng: np.random.Generator) -> dict:
    kind = draw(rng, sw["kind"])
    size = draw(rng, sw["presets"]["size"])
    pipe = draw(rng, sw["presets"]["pipe"])
    presets = [size, pipe]
    p: dict = {"kind": kind, "size": size, "pipe": pipe}
    # sw["set"]: fixed overrides for every scenario of the sweep (a sampled value below wins over them)
    sets: dict = {**(sw.get("set") or {}), "seed": int(rng.integers(1, 2 ** 31 - 1))}
    base = config.load(presets=presets)
    L, W, _ = (float(v) for v in config.get(base, "tank.size"))
    lk, sn, sc = sw["leak"], sw["sonar"], sw["scene"]
    # pipe and leak geometry (the hole's place matters for the scene even without a leak)
    if pipe == "buried":
        sets["pipe.cover"] = draw(rng, lk["cover_buried_m"])
        sets["leak.clock_deg"] = draw(rng, lk["clock_deg_buried"])
    else:
        sets["pipe.cover"] = -draw(rng, lk["crown_out_exposed_m"])
        sets["leak.clock_deg"] = draw(rng, lk["clock_deg_exposed"])
    m = float(lk["position_margin_m"])
    sets["leak.position_along"] = float(rng.uniform(m, W - m))
    if kind == "leak":
        leak = draw(rng, sw["presets"]["leak"])
        presets.append(leak)
        lo, hi = config.get(config.load(presets=[leak]), "leak.flow_range_lpm")
        sets["leak.flow_lpm"] = float(math.exp(rng.uniform(math.log(lo), math.log(hi))))
        sets["leak.pipe_pressure_bar"] = draw(rng, lk["pipe_pressure_bar"])
        prof = draw(rng, lk["profile"])
        sets["leak.profile.kind"] = prof
        if prof == "ramp":
            sets["leak.profile.ramp_s"] = draw(rng, lk["ramp_s"])
            sets["plume.warmup_s"] = 0.0
        elif prof == "intermittent":
            sets["leak.profile.period_s"] = draw(rng, lk["period_s"])
            sets["leak.profile.duty"] = draw(rng, lk["duty"])
            sets["leak.profile.phase"] = float(rng.uniform(0.0, 1.0))
        p["leak"] = leak
    else:
        sets["leak.enabled"] = False
    # scene
    for wall in ("x0", "x1", "y0", "y1"):
        sets[f"tank.walls.{wall}.rms_height_m"] = draw(rng, sc["wall_rms_height_m"])
        sets[f"tank.walls.{wall}.lining"] = bool(rng.uniform() < float(sc["wall_lining"]))
    sets["sand.ripple_amp"] = draw(rng, sc["sand_ripple_amp"])
    sets["sand.ripple_wavelength"] = draw(rng, sc["sand_ripple_wavelength"])
    sets["sand.ripple_heading_deg"] = draw(rng, sc["sand_ripple_heading_deg"])
    sets["sand.undulation_amp"] = draw(rng, sc["sand_undulation_amp"])
    sets["rocks.count"] = draw(rng, sc["rocks_count"])
    if kind == "distractor":
        sets["rocks.count"] = draw(rng, sc["distractor_rocks_count"])
        flanges = list(config.get(base, "pipe.flanges_at"))
        flanges += [float(v) for v in rng.uniform(0.3, W - 0.3, draw(rng, sc["distractor_extra_flanges"]))]
        sets["pipe.flanges_at"] = sorted(flanges)
        sets["materials.steel_coated.rms_height_m"] = float(sc["distractor_steel_rms_height_m"])
    # sonar
    sets["sonar.mode"] = draw(rng, sn["mode"])
    sets["sonar.focus"] = str(draw(rng, sn["focus"])).lower() == "true"
    sets["sonar.noise_level_db"] = draw(rng, sn["noise_level_db"])
    sets["sonar.speckle_sigma_rad"] = draw(rng, sn["speckle_sigma_rad"])
    sets["sonar.speckle_time_s"] = draw(rng, sn["speckle_time_s"])
    fx, pl = sn["fixed"], sn.get("placement")
    if pl is None:                      # v0.1: on a pole at the -x wall, the pipe where the config has it
        wall = "x0"
        y0 = float(config.get(base, "sonar.position")[1])
        position = [draw(rng, fx["x_m"]), y0 + draw(rng, fx["y_offset_m"][size]), draw(rng, fx["z_m"])]
    else:                               # on any wall, the leak at a distance from it
        wall = draw(rng, pl["wall"])
        place_leak(rng, pl, size, (L, W), wall, m, sets)
        leak_xy = (sets.get("pipe.x", float(config.get(base, "pipe.x"))), sets["leak.position_along"])
        position = place_sonar(rng, pl, size, (L, W), wall, leak_xy)
    sets["sonar.position"] = position
    p["wall"] = wall
    motion = draw(rng, sn["mobile"])
    p["motion"] = motion
    if motion == "mobile":
        tr = sn["trajectory"]
        preset = draw(rng, tr["preset"])
        p["trajectory"] = preset
        sets.update({"sonar.pose_source": "trajectory", "sonar.static_cache": "none",
                     "sonar.trajectory.preset": preset, "sonar.trajectory.speed": draw(rng, tr["speed"]),
                     "sonar.trajectory.height": draw(rng, per_size(tr["height"], size))})
        if preset == "approach":
            if "bearing_offset_deg" in tr:  # from the sonar's wall, seen from the leak
                sets["sonar.trajectory.bearing_deg"] = WALL_YAW[wall] + 180.0 + draw(rng, tr["bearing_offset_deg"])
            else:
                sets["sonar.trajectory.bearing_deg"] = draw(rng, tr["bearing_deg"])
            for k in ("start_distance", "end_distance"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, per_size(tr[k], size))
        elif preset == "orbit":
            for k in ("radius", "start_deg"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, per_size(tr[k], size))
            sets["sonar.trajectory.direction"] = int(draw(rng, tr["direction"]))
        elif preset == "hover":
            for k in ("drift_m", "drift_yaw_deg"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, tr[k])
        elif preset == "lawnmower":
            sets["sonar.trajectory.lanes"] = draw(rng, tr["lanes"])
    elif int(fx.get("poses", 1)) == 1 and not fx.get("aim"):     # v0.1: one random view
        sets["sonar.pitch_deg"] = draw(rng, fx["pitch_deg"])
        sets["sonar.yaw_deg"] = draw(rng, fx["yaw_deg"])
        sets["sonar.roll_deg"] = float(draw(rng, fx["roll_deg"]))
    else:                               # views of one scene, most of them aimed at the leak
        n = int(fx.get("poses", 1))
        if int(sw["pings"]) % n:
            raise ValueError(f"pings ({sw['pings']}) must be a multiple of sonar.fixed.poses ({n})")
        mode = config.get(base, f"sonar.modes.{sets['sonar.mode']}")
        target = leak_site(presets, sets)
        views = []
        for k in range(n):
            here = position if k == 0 else place_sonar(rng, pl, size, (L, W), wall, target[:2])
            views.append(view(rng, fx, mode, wall, here, target, bool(rng.uniform() < float(fx.get("aim", 0.0)))))
        p["views"] = views
        if n == 1:
            sets.update({"sonar.pitch_deg": views[0]["pitch_deg"], "sonar.yaw_deg": views[0]["yaw_deg"],
                         "sonar.roll_deg": views[0]["roll_deg"]})
        else:
            rate = float(config.get(base, "sonar.ping_rate_hz"))
            sets.update({"sonar.pose_source": "trajectory", "sonar.static_cache": "on_motion",
                         "sonar.trajectory.preset": "poses", "sonar.trajectory.hold_s": int(sw["pings"]) / n / rate,
                         "sonar.trajectory.poses": [{k: v[k] for k in ("position", "roll_deg", "pitch_deg", "yaw_deg")}
                                                    for v in views]})
    cn = sn.get("common_noise")
    if cn is not None and rng.uniform() < float(cn["share"]):
        sets["sonar.receiver_noise_common_db"] = draw(rng, cn["db"])
    flow = sets.get("leak.flow_lpm")
    decade = "none" if flow is None else f"1e{int(math.floor(math.log10(flow)))}"
    return {"id": f"s{index:04d}", "index": index, "kind": kind, "presets": presets,
            "overrides": [f"{k}={_fmt(v)}" for k, v in sets.items()],
            "params": {**p, **sets}, "stratum": f"{kind}/{pipe}/{decade}"}


def assign_splits(scenarios: list, fractions: dict, rng: np.random.Generator, holdout: dict | None = None) -> None:
    """Split **by scenario**, per stratum: shuffled, then the first 70 % train, 15 % val, the rest test (largest
    remainder, so small strata still get every split when they can).

    Never by ping and never by view: a scenario is one sampled scene, and all of its pings - for a fixed scenario,
    all three views of that one scene - carry its split. Two views of the same scene share their sand, rocks, pipe
    and scatterer phases, so splitting below the scenario would leak the scene between train and test.

    ``holdout`` {"preset": "size", "value": "basin", "split": "test"} is the optional domain split: every scenario
    whose sampled preset ``preset`` equals ``value`` goes to ``split`` and takes no part in the fractions, so that
    domain is never seen in training. The rest are split by the fractions as usual, which leaves ``split`` holding
    both the held-out domain and an in-domain share to compare it against (dataset_stats reports them apart)."""
    names = list(fractions)
    held = []
    if holdout:
        if holdout.get("split") not in names:
            raise ValueError(f"holdout.split must be one of {names}, got {holdout.get('split')!r}")
        rest = []
        for s in scenarios:
            (held if s["params"].get(holdout["preset"]) == holdout["value"] else rest).append(s)
        for s in held:
            s["split"] = holdout["split"]
            s["holdout"] = True
        scenarios = rest
    by: dict[str, list] = {}
    for s in scenarios:
        s.setdefault("holdout", False)
        by.setdefault(s["stratum"], []).append(s)
    for group in by.values():
        order = rng.permutation(len(group))
        n = len(group)
        quota = {k: fractions[k] * n for k in names}
        counts = {k: int(math.floor(v)) for k, v in quota.items()}
        for k in sorted(names, key=lambda k: quota[k] - counts[k], reverse=True)[:n - sum(counts.values())]:
            counts[k] += 1
        i = 0
        for k in names:
            for j in order[i:i + counts[k]]:
                group[j]["split"] = k
            i += counts[k]


def sample(sw: dict, root: Path) -> list:
    rng = np.random.default_rng(int(sw["seed"]))
    scenarios = [sample_scenario(sw, i, rng) for i in range(int(sw["scenarios"]))]
    assign_splits(scenarios, sw["splits"], np.random.default_rng([int(sw["seed"]), 1]), sw.get("holdout"))
    root.mkdir(parents=True, exist_ok=True)
    (root / "scenarios.json").write_text(json.dumps(scenarios, indent=1))
    return scenarios


# ------------------------------------------------------------------------------------------------- worker

def worker(root: Path, max_scenarios: int) -> int:
    """Generate unclaimed, unfinished scenarios (in the container)."""
    import warp as wp
    from uw import sinks, sonar_only
    wp.config.quiet = True
    scenarios = json.loads((root / "scenarios.json").read_text())
    sweep = json.loads((root / "sweep.json").read_text())
    done = 0
    for sc in scenarios:
        if done >= max_scenarios or (root / "PAUSE").exists():
            break
        d = root / sc["id"]
        if (d / "DONE").exists() or (d / "FAILED").exists():
            continue
        d.mkdir(exist_ok=True)
        try:
            fd = os.open(d / ".claim", os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError:
            continue
        os.write(fd, str(os.getpid()).encode())
        os.close(fd)
        try:
            for f in d.iterdir():                          # a partial run before: start over
                if f.name.startswith("ping_") or f.name == "manifest.json":
                    f.unlink()
            t0 = time.time()
            cfg = config.load(presets=sc["presets"], overrides=sc["overrides"])
            info = {"scenario": sc, "sweep": sweep["name"], "git": sweep["git"]}
            with sinks.make_sink("file", out_dir=d, info=info, layout="flat",
                                 compress=bool(config.get(cfg, "recorder.compress")),
                                 png_level=int(config.get(cfg, "recorder.png_level"))) as sink:
                st = sonar_only.run(cfg, int(sweep["pings"]), sink, info=info, cache=False)
            size = sum(f.stat().st_size for f in d.iterdir())
            (d / "DONE").write_text(json.dumps({**st, "wall_s": time.time() - t0, "bytes": size}, indent=1))
            print(f"{sc['id']} done: {st['pings']} pings in {time.time() - t0:.0f} s, {size / 2 ** 20:.0f} MB",
                  flush=True)
        except KeyboardInterrupt:
            (d / ".claim").unlink(missing_ok=True)
            raise
        except Exception:                                  # noqa: BLE001 - recorded, the sweep goes on
            (d / "FAILED").write_text(traceback.format_exc())
            print(f"{sc['id']} FAILED:\n{traceback.format_exc()}", flush=True)
        (d / ".claim").unlink(missing_ok=True)
        done += 1
    return 0


# ------------------------------------------------------------------------------------------- orchestrator

def git_version() -> str:
    try:
        h = subprocess.run(["git", "-C", str(WS), "rev-parse", "--short", "HEAD"], capture_output=True,
                           text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(WS), "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True).stdout.strip()
        return h + ("+dirty" if dirty else "")
    except OSError:
        return "unknown"


def f1tenth_active() -> str:
    """Why the GPU should be left alone ('' if it need not be): an f1tenth run (its processes or containers)."""
    me = os.getpid()
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        if pid.isdigit() and int(pid) != me and F1TENTH.search(args) and "sweep.py" not in args:
            return f"f1tenth process: {args[:100]}"
    names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True).stdout
    for n in names.split():
        if n.startswith("f1t"):
            return f"f1tenth container {n}"
    return ""


def status(root: Path) -> dict:
    scenarios = json.loads((root / "scenarios.json").read_text())
    st = {"total": len(scenarios), "done": 0, "failed": 0, "pings": 0, "bytes": 0, "wall_s": 0.0}
    for sc in scenarios:
        d = root / sc["id"]
        if (d / "DONE").exists():
            info = json.loads((d / "DONE").read_text())
            st["done"] += 1
            st["pings"] += info["pings"]
            st["bytes"] += info["bytes"]
            st["wall_s"] += info["wall_s"]
        elif (d / "FAILED").exists():
            st["failed"] += 1
    return st


def run(sw: dict, root: Path, workers: int, min_free_root_gb: float) -> int:
    if not (root / "scenarios.json").exists():
        sample(sw, root)
    (root / "logs").mkdir(parents=True, exist_ok=True)
    meta = root / "sweep.json"
    if not meta.exists():
        meta.write_text(json.dumps({**sw, "git": git_version(), "started": time.strftime("%Y-%m-%dT%H:%M:%S")},
                                   indent=1))
    for claim in root.glob("*/.claim"):                    # no worker runs yet: stale claims of an earlier run
        claim.unlink()
    procs: list = []
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    t0, n0, last = time.time(), status(root)["pings"], 0.0
    paused = ""
    launched = 0

    def stop_workers():
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGTERM)              # the launcher stops its container
        for p in procs:
            try:
                p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                p.kill()
        procs.clear()

    while not stop["flag"]:
        st = status(root)
        if st["done"] + st["failed"] >= st["total"]:
            break
        why = f1tenth_active()
        if not why and (root / "PAUSE").exists():
            why = "PAUSE file"
        # The dataset may live on another disk than the repo (--root): guard the one being filled, and keep a
        # smaller guard on / for the container, the Warp cache and the logs of the launcher.
        free = shutil.disk_usage(root).free / 2 ** 30
        if not why and free < min_free_root_gb:
            why = f"only {free:.0f} GB free on {root}"
        free_sys = shutil.disk_usage("/").free / 2 ** 30
        if not why and free_sys < MIN_FREE_SYSTEM_GB:
            why = f"only {free_sys:.0f} GB free on /"
        if why:
            if not paused:
                print(f"{time.strftime('%H:%M:%S')} pause: {why}", flush=True)
                stop_workers()
            paused = why
            time.sleep(30)
            continue
        if paused:
            print(f"{time.strftime('%H:%M:%S')} resume", flush=True)
            paused = ""
            for claim in root.glob("*/.claim"):
                claim.unlink()
        procs[:] = [p for p in procs if p.poll() is None]
        while len(procs) < workers:
            launched += 1
            log = open(root / "logs" / f"worker_{launched:03d}.log", "w")
            procs.append(subprocess.Popen([sys.executable, "-m", "uw", "py", str(WS / "tools" / "sweep.py"), "worker",
                                           "--root", str(root), "--max-scenarios", str(CHUNK)],
                                          cwd=WS, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            time.sleep(5)                                 # stagger: no two workers on one scenario's setup
        if time.time() - last > 120:
            st = status(root)
            rate = (st["pings"] - n0) / max(time.time() - t0, 1.0)
            left = (st["total"] - st["done"] - st["failed"]) * int(sw["pings"])
            print(f"{time.strftime('%H:%M:%S')} {st['done']}/{st['total']} done, {st['failed']} failed, "
                  f"{st['pings']} pings, {st['bytes'] / 2 ** 30:.1f} GB, {rate:.2f} pings/s, "
                  f"ETA {left / max(rate, 1e-6) / 3600:.1f} h", flush=True)
            last = time.time()
        time.sleep(10)
    stop_workers()
    if not stop["flag"]:
        finalize(sw, root)
    return 0


# ----------------------------------------------------------------------------------------------- outputs

def _describe(spec) -> str:
    if isinstance(spec, list):
        return f"uniform [{spec[0]:g}, {spec[1]:g}]"
    if isinstance(spec, dict) and "log" in spec:
        return "log-uniform in the leak preset's flow_range_lpm" if spec["log"] == "preset" else \
            f"log-uniform [{spec['log'][0]:g}, {spec['log'][1]:g}]"
    if isinstance(spec, dict) and "int" in spec:
        return f"integer uniform {spec['int'][0]}-{spec['int'][1]}"
    if isinstance(spec, dict) and "choice" in spec:
        total = sum(float(v) for v in spec["choice"].values())
        return ", ".join(f"{k} {float(v) / total:.0%}" for k, v in spec["choice"].items())
    if isinstance(spec, dict):
        return "; ".join(f"{k}: {_describe(v)}" for k, v in spec.items())
    return f"{spec}"


def _rows(prefix: str, d: dict) -> list:
    out = []
    for k, v in d.items():
        if isinstance(v, dict) and not ({"log", "int", "choice"} & set(v)) and k != "y_offset_m":
            out += _rows(f"{prefix}{k}.", v)
        else:
            out.append(f"| `{prefix}{k}` | {_describe(v)} |")
    return out


def finalize(sw: dict, root: Path) -> None:
    from uw.recorder import LABELS
    scenarios = json.loads((root / "scenarios.json").read_text())
    meta = json.loads((root / "sweep.json").read_text())
    rows, totals = [], {"pings": 0, "bytes": 0, "wall_s": 0.0}
    for sc in scenarios:
        d = root / sc["id"]
        entry = {k: sc[k] for k in ("id", "kind", "split", "presets", "stratum", "params")}
        entry["holdout"] = bool(sc.get("holdout"))
        if (d / "DONE").exists():
            info = json.loads((d / "DONE").read_text())
            entry.update(status="done", pings=info["pings"], bytes=info["bytes"], wall_s=info["wall_s"])
            for k in totals:
                totals[k] += info[k]
        else:
            entry["status"] = "failed" if (d / "FAILED").exists() else "missing"
        rows.append(entry)
    splits = {s: sum(1 for r in rows if r["split"] == s and r["status"] == "done") for s in sw["splits"]}
    manifest = {"name": sw["name"], "git": meta["git"], "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "started": meta["started"], "pings_per_scenario": sw["pings"], "totals": totals, "splits": splits,
                "labels": LABELS, "scenarios": rows}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=1))
    plan = (WS / "docs" / "PLAN.md").read_text()
    ho = sw.get("holdout")
    holdout_line = ("" if not ho else
                    f" Held-out domain: every scenario with {ho['preset']} = {ho['value']} is in {ho['split']} and "
                    f"in no other split ({sum(1 for r in rows if r.get('holdout'))} scenarios), so {ho['value']} is "
                    f"never seen in training.")
    gaps = re.findall(r"^- \*\*([^*]+)\*\*", plan.split("## Known gaps", 1)[1].split("\n## ", 1)[0], re.M)
    failed = [r["id"] for r in rows if r["status"] != "done"]
    fx, pl = sw["sonar"]["fixed"], sw["sonar"].get("placement")
    n_views = int(fx.get("poses", 1))
    if pl is None:
        where = "Fixed sonar poses are on a pole at the -x wall"
    else:
        where = ("The sonar is on a pole at one of the four walls, the leak at a distance from that wall "
                 f"(`sonar.placement`). Fixed scenarios have {n_views} view{'s' if n_views > 1 else ''} of one scene "
                 f"from that wall ({int(sw['pings']) // n_views} pings each), {float(fx.get('aim', 0.0)):.0%} of them "
                 f"aimed: a point up to {fx['aim_height_m'][1]:g} m above the hole put at a random place inside the "
                 "fan (the hole itself "
                 "can then lie outside it); the predicted leak range, bearing and elevation of every view are in "
                 "`params.views` of the manifest")
    card = f"""# Underwater gas-leak sonar dataset {sw['name']}

Simulated Oculus M750d forward-looking sonar pings of a gas leak from a pipe in a test tank (or a larger basin),
with per-pixel plume labels. Physics: `~/underwater_ws` at git `{meta['git']}` (docs/PLAN.md, docs/validation.md).
Generated by `tools/sweep.py` from `config/sweeps/{sw['name']}.yaml`, {meta['started']} to {manifest['created']}.

- Scenarios: {len(rows)} ({sum(r['status'] == 'done' for r in rows)} done, {len(failed)} failed or missing{': ' + ', '.join(failed) if failed else ''}),
  {sw['pings']} pings each at 10 Hz; {totals['pings']} pings, {totals['bytes'] / 2 ** 30:.1f} GB.
- Splits **by scenario** (never by ping or view: all pings of a scenario, and all three views of a fixed
  scenario's one scene, share its split), stratified by kind, pipe preset and flow decade:
  {', '.join(f'{k} {v}' for k, v in splits.items())}.{holdout_line}

## Layout

    <root>/manifest.json            every scenario: id, kind, split, presets, sampled parameters (fixed views:
                                    params.views), status, sizes
    <root>/scenarios.json           the sampler's output (presets and --set overrides per scenario)
    <root>/<id>/manifest.json       the scenario's full config, seed, config hash, labels, per-ping file list
    <root>/<id>/ping_xxxxx.npz      polar_db (float16, dB re 1 uPa, rows = range bins from 0, columns = beams, NaN in
                                    the blind zone), plume_share_all_u8, plume_share_direct_u8 (the bubbles' share of
                                    the echo energy, round(255 share)), range_m, bearing_deg
    <root>/<id>/ping_xxxxx.png      the 8-bit polar image (TVG, 0-80 dB above the noise floor; far range at the top)
    <root>/<id>/ping_xxxxx_mask_geom.png, _mask_direct.png, _mask_all.png   the labels (0/255, far range at the top)
    <root>/<id>/ping_xxxxx.json     sim time, the sonar pose (position, roll/pitch/yaw, rotation), the leak's position,
                                    range/bearing/elevation in the sonar frame and its flow now (L/min), config hash,
                                    seed, plume pixel counts

Load with `uw.dataset.SonarDataset(root, split)` (PyTorch): image, the three masks, scalar targets.

## Labels

{chr(10).join(f'- `{k}`: {v}' for k, v in LABELS.items())}
- Mask threshold: the bubbles' share > 0.5 (sonar.plume_mask_threshold).

## Randomised parameters (per scenario)

| Parameter | Distribution |
|---|---|
{chr(10).join(_rows('', {k: v for k, v in sw.items() if k in ('kind', 'presets', 'leak', 'sonar', 'scene')}))}

Every scenario also has its own seed (sand relief, rock layout, scatterer phases, bubbles, noise). Leak presets
(config/presets/leak): pinhole 0.1-5 L/min, crack 5-50, rupture 50-500 (flow at the hole's pressure). {where};
mobile ones follow a trajectory preset (static scene recomputed every ping). Ramp leaks start at the first ping (no
warm-up); the others are warmed up for 12 s.

## Known gaps (docs/PLAN.md)

{chr(10).join(f'- {g}' for g in gaps)}

Not modelled: camera images (M4), bubble coalescence and breakup, dissolution, tank recirculation, currents,
motion within a ping, Doppler. The plume's liquid flow follows the nominal leak flow, also while a ramp or an
intermittent leak changes the gas flow. Simulation only: not yet validated against real recordings (docs/PLAN.md,
"Real-data validation").
"""
    (root / "README.md").write_text(card)
    print(f"manifest and README written: {root}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", choices=("sample", "run", "worker", "status", "finalize"))
    ap.add_argument("--sweep", default=str(WS / "config" / "sweeps" / "v0.2.yaml"))
    ap.add_argument("--root", default="")
    ap.add_argument("--workers", type=int, default=0, help="default: the sweep file's")
    ap.add_argument("--max-scenarios", type=int, default=CHUNK)
    ap.add_argument("--min-free-root-gb", type=float, default=27.0,
                    help="pause while the dataset root's disk has less than this free")
    args = ap.parse_args()
    if args.command == "worker":
        return worker(Path(args.root), args.max_scenarios)
    sw = yaml.safe_load(Path(args.sweep).read_text())
    if args.root:
        root = Path(args.root)
    else:
        rt = yaml.safe_load((WS / "config" / "runtime.yaml").read_text())
        local = WS / "config" / "runtime.local.yaml"
        if local.is_file():
            rt.update(yaml.safe_load(local.read_text()) or {})
        root = Path(os.path.expanduser(rt["data_root"])) / sw["name"]
    if args.command == "sample":
        sc = sample(sw, root)
        kinds = {k: sum(s["kind"] == k for s in sc) for k in ("leak", "negative", "distractor")}
        print(f"{len(sc)} scenarios -> {root}/scenarios.json: {kinds}")
        return 0
    if args.command == "status":
        print(json.dumps(status(root), indent=1))
        return 0
    if args.command == "finalize":
        finalize(sw, root)
        return 0
    return run(sw, root, args.workers or int(sw["workers"]), args.min_free_root_gb)


if __name__ == "__main__":
    raise SystemExit(main())
