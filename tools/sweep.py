"""Dataset sweeps: sample scenarios, generate them with sonar-only (resumable, parallel), write the manifest and the
dataset card.

    python3 tools/sweep.py run [--sweep config/sweeps/v0.1.yaml] [--root DIR] [--workers N] [--limit N]   (host)
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
from uw import config  # noqa: E402

F1TENTH = re.compile(r"sim_isaac\.sh|phase5_bringup|jazzy_parity|isaac_ws/launch\.sh|f1t_isaac")
CHUNK = 20                       # scenarios per worker process (then a fresh one: no memory creep)


# ----------------------------------------------------------------------------------------------- sampling

def draw(rng: np.random.Generator, spec):
    """One value of a distribution spec (see config/sweeps/v0.1.yaml)."""
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
    return str(v)


def sample_scenario(sw: dict, index: int, rng: np.random.Generator) -> dict:
    kind = draw(rng, sw["kind"])
    size = draw(rng, sw["presets"]["size"])
    pipe = draw(rng, sw["presets"]["pipe"])
    presets = [size, pipe]
    p: dict = {"kind": kind, "size": size, "pipe": pipe}
    sets: dict = {"seed": int(rng.integers(1, 2 ** 31 - 1))}
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
    fx = sn["fixed"]
    y0 = float(config.get(base, "sonar.position")[1])
    position = [draw(rng, fx["x_m"]), y0 + draw(rng, fx["y_offset_m"][size]), draw(rng, fx["z_m"])]
    sets["sonar.position"] = position
    motion = draw(rng, sn["mobile"])
    p["motion"] = motion
    if motion == "mobile":
        tr = sn["trajectory"]
        preset = draw(rng, tr["preset"])
        p["trajectory"] = preset
        sets.update({"sonar.pose_source": "trajectory", "sonar.static_cache": "none",
                     "sonar.trajectory.preset": preset, "sonar.trajectory.speed": draw(rng, tr["speed"]),
                     "sonar.trajectory.height": draw(rng, tr["height"])})
        if preset == "approach":
            for k in ("bearing_deg", "start_distance", "end_distance"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, tr[k])
        elif preset == "orbit":
            for k in ("radius", "start_deg"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, tr[k])
            sets["sonar.trajectory.direction"] = int(draw(rng, tr["direction"]))
        elif preset == "hover":
            for k in ("drift_m", "drift_yaw_deg"):
                sets[f"sonar.trajectory.{k}"] = draw(rng, tr[k])
        elif preset == "lawnmower":
            sets["sonar.trajectory.lanes"] = draw(rng, tr["lanes"])
    else:
        sets["sonar.pitch_deg"] = draw(rng, fx["pitch_deg"])
        sets["sonar.yaw_deg"] = draw(rng, fx["yaw_deg"])
        sets["sonar.roll_deg"] = float(draw(rng, fx["roll_deg"]))
    flow = sets.get("leak.flow_lpm")
    decade = "none" if flow is None else f"1e{int(math.floor(math.log10(flow)))}"
    return {"id": f"s{index:04d}", "index": index, "kind": kind, "presets": presets,
            "overrides": [f"{k}={_fmt(v)}" for k, v in sets.items()],
            "params": {**p, **sets}, "stratum": f"{kind}/{pipe}/{decade}"}


def assign_splits(scenarios: list, fractions: dict, rng: np.random.Generator) -> None:
    """Split by scenario, per stratum: shuffled, then the first 70 % train, 15 % val, the rest test (largest
    remainder, so small strata still get every split when they can)."""
    names = list(fractions)
    by: dict[str, list] = {}
    for s in scenarios:
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
    assign_splits(scenarios, sw["splits"], np.random.default_rng([int(sw["seed"]), 1]))
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
            with sinks.make_sink("file", out_dir=d, info=info, layout="flat") as sink:
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
        free = shutil.disk_usage("/").free / 2 ** 30
        if not why and free < min_free_root_gb:
            why = f"only {free:.0f} GB free on /"
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
    gaps = re.findall(r"^- \*\*([^*]+)\*\*", plan.split("## Known gaps", 1)[1].split("\n## ", 1)[0], re.M)
    failed = [r["id"] for r in rows if r["status"] != "done"]
    card = f"""# Underwater gas-leak sonar dataset {sw['name']}

Simulated Oculus M750d forward-looking sonar pings of a gas leak from a pipe in a test tank (or a larger basin),
with per-pixel plume labels. Physics: `~/underwater_ws` at git `{meta['git']}` (docs/PLAN.md, docs/validation.md).
Generated by `tools/sweep.py` from `config/sweeps/{sw['name']}.yaml`, {meta['started']} to {manifest['created']}.

- Scenarios: {len(rows)} ({sum(r['status'] == 'done' for r in rows)} done, {len(failed)} failed or missing{': ' + ', '.join(failed) if failed else ''}),
  {sw['pings']} pings each at 10 Hz; {totals['pings']} pings, {totals['bytes'] / 2 ** 30:.1f} GB.
- Splits by scenario, stratified by kind, pipe preset and flow decade: {', '.join(f'{k} {v}' for k, v in splits.items())}.

## Layout

    <root>/manifest.json            every scenario: id, kind, split, presets, sampled parameters, status, sizes
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
(config/presets/leak): pinhole 0.1-5 L/min, crack 5-50, rupture 50-500 (flow at the hole's pressure). Fixed
sonar poses are on a pole at the -x wall; mobile ones follow a trajectory preset (static scene recomputed every
ping). Ramp leaks start at the first ping (no warm-up); the others are warmed up for 12 s.

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
    ap.add_argument("--sweep", default=str(WS / "config" / "sweeps" / "v0.1.yaml"))
    ap.add_argument("--root", default="")
    ap.add_argument("--workers", type=int, default=0, help="default: the sweep file's")
    ap.add_argument("--max-scenarios", type=int, default=CHUNK)
    ap.add_argument("--min-free-root-gb", type=float, default=27.0)
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
