"""`uw sysid <vehicle>`: from the real vehicle's logs to the parameters of vehicles/<vehicle>/vehicle.yaml.

    uw sysid bluerov2_heavy [--logs DIR] [--out output/sysid] [--write]

SKELETON. What it does today:
1. checks vehicles/<vehicle>/vehicle.yaml against the schema (vehicles.common.schema);
2. takes stock of the logs in DIR (default: logs.dest of deploy/targets/rov, i.e. where `uw logs pull rov` puts
   them): ArduSub .BIN files (dataflash magic checked) and rosbag2 recordings (their metadata.yaml: duration,
   topics, message counts — no ROS needed);
3. says which sysid inputs are there (motor PWM `rc/out`, raw IMU, pressure) and which are missing;
4. writes a Markdown report to --out.
What it does not do yet: estimate anything. `estimate()` returns no values, so `--write` has nothing to write and
says so. When it does, `--write` replaces only the estimated keys in vehicle.yaml, in place, keeping every other line
and comment, and tags each new value with its source; without `--write` vehicle.yaml is never touched.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import sys
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parents[1]
DATAFLASH_MAGIC = b"\xa3\x95"
#: what a thrust/damping identification needs from a dive, as rov_bringup's light profile names it
NEEDED = {
    "motor PWM (input)": "/mavros/rc/out",
    "raw IMU (accel, gyro)": "/mavros/imu/data_raw",
    "attitude": "/mavros/imu/data",
    "depth (pressure)": "/mavros/imu/static_pressure",
}


def default_logs() -> Path:
    try:
        from uw import targets
        return Path(os.path.expanduser(str(targets.load("rov", quiet=True).get("logs", {}).get("dest", ""))))
    except Exception:
        return Path("/HDD2T/underwater_data/rov_logs")


def inventory(root: Path) -> dict:
    bins, bags = [], []
    if root.is_dir():
        for p in sorted(root.rglob("*")):
            if p.is_file() and p.suffix.upper() == ".BIN":
                with p.open("rb") as f:
                    head = f.read(2)
                bins.append({"path": str(p.relative_to(root)), "bytes": p.stat().st_size,
                             "dataflash": head == DATAFLASH_MAGIC})
            elif p.name == "metadata.yaml":
                info = (yaml.safe_load(p.read_text()) or {}).get("rosbag2_bagfile_information", {})
                topics = {t["topic_metadata"]["name"]: t["message_count"]
                          for t in info.get("topics_with_message_count", [])}
                bags.append({"path": str(p.parent.relative_to(root)),
                             "duration_s": info.get("duration", {}).get("nanoseconds", 0) / 1e9,
                             "storage": info.get("storage_identifier"), "topics": topics})
    return {"root": str(root), "bins": bins, "bags": bags}


def readiness(inv: dict) -> dict:
    seen = {t: n for b in inv["bags"] for t, n in b["topics"].items()}
    return {what: sum(n for t, n in seen.items() if t.endswith(suffix)) for what, suffix in NEEDED.items()}


def estimate(inv: dict, cfg: dict) -> dict:
    """{dotted key: (value, source)}. Skeleton: no estimator yet, so nothing."""
    return {}


def write_estimates(path: Path, est: dict, tag: str) -> list[str]:
    """Replace ``section.key: value`` lines in place, keep everything else; returns the keys written."""
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    done = []
    for dotted, (value, source) in est.items():
        section, key = dotted.rsplit(".", 1)
        in_sec = False
        for i, ln in enumerate(lines):
            if re.match(rf"^{re.escape(section)}:\s*(#.*)?$", ln):
                in_sec = True
                continue
            if in_sec and re.match(r"^\S", ln):
                break
            m = re.match(rf"^(\s+{re.escape(key)}:\s*)([^#\n]*?)(\s*#.*)?$", ln.rstrip("\n"))
            if in_sec and m:
                lines[i] = f"{m.group(1)}{value!r}  # {tag}: {source} (was {m.group(2).strip()})\n"
                done.append(dotted)
                break
    path.write_text("".join(lines), encoding="utf-8")
    return done


def report(vehicle: str, cfg: dict, inv: dict, ready: dict, est: dict) -> str:
    from vehicles.common import schema
    out = [f"# sysid {vehicle} — {dt.datetime.now():%Y-%m-%d %H:%M}", "",
           f"Logs: `{inv['root']}` — {len(inv['bins'])} .BIN, {len(inv['bags'])} bag(s).", "",
           "## Inputs for the identification", "", "| input | messages |", "|---|---|"]
    out += [f"| {k} | {v or '**missing**'} |" for k, v in ready.items()]
    out += ["", "## Logs", "", "| file | size / duration | note |", "|---|---|---|"]
    out += [f"| `{b['path']}` | {b['bytes'] / 1e6:.1f} MB | {'dataflash' if b['dataflash'] else '**not a dataflash log**'} |"
            for b in inv["bins"]]
    out += [f"| `{b['path']}` | {b['duration_s']:.0f} s | {b['storage']}, {len(b['topics'])} topics |" for b in inv["bags"]]
    out += ["", "## Estimates", "", "None: the estimator is not written yet (skeleton)." if not est else
            "\n".join(f"- `{k}` = {v} ({s})" for k, (v, s) in est.items()),
            "", "## Current vehicle.yaml", "", schema.table(cfg), ""]
    return "\n".join(out)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="uw sysid", description=__doc__.splitlines()[0])
    ap.add_argument("vehicle", help="a folder of vehicles/ with a vehicle.yaml, e.g. bluerov2_heavy")
    ap.add_argument("--logs", default=None, help="log folder (default: logs.dest of deploy/targets/rov)")
    ap.add_argument("--out", default=str(WS / "output" / "sysid"))
    ap.add_argument("--write", action="store_true", help="write the estimates into vehicle.yaml")
    args = ap.parse_args(argv)
    from vehicles.common import schema
    path = WS / "vehicles" / args.vehicle / "vehicle.yaml"
    if not path.is_file():
        print(f"uw sysid: {path.relative_to(WS)} not found", file=sys.stderr)
        return 1
    try:
        cfg = schema.load(path)
    except schema.SchemaError as e:
        print(f"uw sysid: {e}", file=sys.stderr)
        return 1
    logs = Path(args.logs) if args.logs else default_logs()
    inv = inventory(logs)
    ready = readiness(inv)
    est = estimate(inv, cfg)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rep = out / f"{args.vehicle}_{dt.datetime.now():%Y-%m-%d_%H%M%S}.md"
    rep.write_text(report(args.vehicle, cfg, inv, ready, est), encoding="utf-8")
    print(f"uw sysid {args.vehicle}: {len(inv['bins'])} .BIN and {len(inv['bags'])} bag(s) in {logs}; report {rep}")
    for k, v in ready.items():
        print(f"  {k:<24} {v or 'missing'}")
    if args.write:
        if not est:
            print("uw sysid: --write: no estimates (skeleton), vehicle.yaml untouched")
        else:
            done = write_estimates(path, est, f"sysid {dt.date.today()}")
            print(f"uw sysid: wrote {', '.join(done)} into {path.relative_to(WS)}")
    else:
        print("uw sysid: vehicle.yaml untouched (no --write)")
    return 0
