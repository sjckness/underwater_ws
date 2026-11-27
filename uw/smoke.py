"""`uw smoke rov|rov-sim`: is the vehicle's software alive? Read-only, the vehicle must be DISARMED.

    uw smoke rov [--duration 5] [--json FILE]

Checks, each PASS / FAIL with its number:
- ArduSub's HEARTBEAT is live and the vehicle is DISARMED (mavlink2rest, HTTP GET);
- the extension container is running;
- each topic of the target's `smoke.topics` arrives at least at its minimum rate (ros/src/link_monitor inside the
  extension, with a reader QoS matched to each publisher — `ros2 topic hz` cannot see BEST_EFFORT topics);
- nobody publishes `rc/override` (the one MAVROS path that could move the motors);
- the Pi's CPU load over one second is under `smoke.cpu_max_percent`, and its temperature is reported.
It starts nothing: if rov_bringup is not running, the topic checks fail and say so.
"""
from __future__ import annotations

import argparse
import json
import shlex
import time

from . import targets
from .remote import Remote, RemoteError, vehicle_state


def cpu_percent(r: Remote, window_s: float = 1.0) -> float:
    def sample():
        f = r.sh("head -1 /proc/stat").split()[1:]
        v = [int(x) for x in f]
        return sum(v), v[3] + (v[4] if len(v) > 4 else 0)       # total, idle + iowait
    t0, i0 = sample()
    time.sleep(window_s)
    t1, i1 = sample()
    return 100.0 * (1.0 - (i1 - i0) / max(1, t1 - t0))


def temperature_c(r: Remote) -> float | None:
    out = r.sh("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || true").strip()
    return int(out) / 1000.0 if out.isdigit() else None


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="uw smoke", description=__doc__.splitlines()[0])
    ap.add_argument("target")
    ap.add_argument("--duration", type=float, default=None, help="seconds of topic measurement (default: target)")
    ap.add_argument("--json", default="", help="also write the results here")
    args = ap.parse_args(argv)
    results: list[tuple[str, bool, str]] = []

    def check(name, ok, detail):
        results.append((name, bool(ok), str(detail)))
        print(f"  {'PASS' if ok else 'FAIL'}  {name:<44} {detail}")

    try:
        t = targets.load(args.target)
        r = Remote(t)
        sm = t.get("smoke", {})
        print(f"uw smoke {t['name']}: {r.where()}")
        url = t.get("blueos_url") or f"http://{t['host']}"
        try:
            s = vehicle_state(url)
            check("ArduSub heartbeat live", s["live"], f"{s['system_status']}, {s['frequency_hz']} Hz")
            check("vehicle DISARMED", not s["armed"], "armed" if s["armed"] else "disarmed")
        except RemoteError as e:
            check("ArduSub heartbeat live", False, e)
        c = t.get("container")
        running = bool(c) and r.sh(f"{r.docker()} inspect -f '{{{{.State.Running}}}}' {shlex.quote(c)} 2>/dev/null || true"
                                   ).strip() == "true"
        check("extension container running", running, c or "no container in the target (uw deploy --discover)")
        topics = sm.get("topics", {})
        if running and topics:
            dur = args.duration or float(sm.get("duration_s", 5))
            ws = f"{t.get('persistent_container', '/root/persistent_ws').rstrip('/')}/{t.get('workspace', 'ros2_ws')}"
            vehicle = t.get("vehicle", "rov1")
            names = [n.format(vehicle=vehicle) for n in topics]
            mon = (f"source /opt/ros/jazzy/setup.bash && source {shlex.quote(ws)}/install/setup.bash && "
                   f"ros2 run link_monitor monitor --duration {dur} --wait {sm.get('wait_s', 5)} "
                   f"--domain {int(t.get('ros_domain_id', 73))} --json /tmp/uw_smoke.json {' '.join(names)} "
                   f">/dev/null 2>&1; cat /tmp/uw_smoke.json")
            try:
                rep = json.loads(r.exec_in(c, mon, timeout=dur + 60) or "{}")
            except (RemoteError, ValueError) as e:
                rep = {}
                check("topic rates (link_monitor)", False, e)
            per = {row.get("topic"): row for row in rep.get("topics", [])}       # link_monitor's --json report
            for (raw, min_hz), name in zip(topics.items(), names):
                hz = per.get(name, {}).get("rate_hz")
                check(f"{name} >= {min_hz} Hz", hz is not None and hz >= float(min_hz),
                      f"{hz:.1f} Hz" if hz is not None else "not received")
            ovr = f"/{vehicle}/mavros/rc/override"
            pubs = r.exec_in(c, f"source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID={int(t.get('ros_domain_id', 73))} "
                                f"ros2 topic info {ovr} 2>/dev/null | sed -n 's/^Publisher count: //p'",
                             check=False, timeout=30).strip()
            check(f"nobody publishes {ovr}", pubs in ("", "0"), f"publishers: {pubs or 0}")
        cpu = cpu_percent(r)
        check(f"CPU < {sm.get('cpu_max_percent', 80)} %", cpu < float(sm.get("cpu_max_percent", 80)), f"{cpu:.0f} %")
        temp = temperature_c(r)
        if temp is not None:
            check(f"temperature < {sm.get('temp_max_c', 80)} C", temp < float(sm.get("temp_max_c", 80)), f"{temp:.1f} C")
    except (RemoteError, targets.TargetError) as e:
        check("target reachable", False, e)
    if args.json:
        with open(args.json, "w") as f:
            json.dump([{"check": n, "ok": ok, "detail": d} for n, ok, d in results], f, indent=1)
    failed = [n for n, ok, _ in results if not ok]
    print(f"uw smoke: {len(results) - len(failed)}/{len(results)} passed" + (f"; failed: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0
