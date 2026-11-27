"""Talk to the running simulator's control channel (vehicles/common/simctl.py) from a shell.

The physics process (`python -m uw pool`, or `vehicles.common.physics_proc` alone) listens on physics.control.port (9020):
on 127.0.0.1, and on linus also on the docker bridge's 172.31.250.1, where the ROS container reaches it.

    python3 sim/tools/simctl.py hello
    python3 sim/tools/simctl.py current --speed 0.3 --dir 90 --ramp 3          # the water flows towards +y
    python3 sim/tools/simctl.py current --speed 0.25 --profile power --gust-sigma 0.08 --gust-dir 15
    python3 sim/tools/simctl.py still --ramp 2                                  # back to still water
    python3 sim/tools/simctl.py state                                           # the true state, one line of JSON
    python3 sim/tools/simctl.py stream --rate 5 --duration 10                   # a compact live line per sample
    python3 sim/tools/simctl.py reposition 5 5                                  # horizontal only: depth, attitude kept
    python3 sim/tools/simctl.py device a50 acoustic_enabled=false               # switch the A50's pinging off
    python3 sim/tools/simctl.py stats

Standard library only; --addr host:port (default 127.0.0.1:9020, or $UW_SIMCTL).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from vehicles.common.simctl_client import SimCtl, SimCtlError  # noqa: E402


def _value(text: str):
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return float(text)
    except ValueError:
        return text


def _line(s: dict) -> str:
    p, v, w = s["p_tank"], s["v_ground_enu"], s["v_water_enu"]
    return ("t %7.2f  pos %6.2f %6.2f  depth %5.2f  hdg %6.1f  ground %5.3f m/s @ %4.0f  water %5.3f m/s @ %4.0f"
            % (s["sim_t"], p[0], p[1], s["depth_m"], s["heading_enu_deg"], math.hypot(v[0], v[1]),
               math.degrees(math.atan2(v[1], v[0])), math.hypot(w[0], w[1]), math.degrees(math.atan2(w[1], w[0]))))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--addr", default=os.environ.get("UW_SIMCTL", "127.0.0.1:9020"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("hello")
    sub.add_parser("state")
    sub.add_parser("stats")
    c = sub.add_parser("current", help="set the water current (keys of config current:)")
    c.add_argument("--speed", type=float, required=True, help="at the surface [m/s]")
    c.add_argument("--dir", type=float, default=0.0, help="ENU azimuth the water flows towards [deg]: 0 = +x")
    c.add_argument("--ramp", type=float, default=0.0, help="blend in over this many simulated seconds")
    c.add_argument("--profile", default="uniform", choices=["uniform", "power", "linear", "table"])
    c.add_argument("--bottom-factor", type=float, default=None, help="linear profile: the factor at the floor")
    c.add_argument("--gust-sigma", type=float, default=0.0, help="gusts on the speed, 1 sigma [m/s]")
    c.add_argument("--gust-dir", type=float, default=0.0, help="gusts on the direction, 1 sigma [deg]")
    c.add_argument("--gust-time", type=float, default=20.0, help="gust correlation time [s]")
    s = sub.add_parser("still", help="still water")
    s.add_argument("--ramp", type=float, default=0.0)
    r = sub.add_parser("reposition", help="move the vehicle horizontally (tank metres)")
    r.add_argument("x", type=float)
    r.add_argument("y", type=float)
    st = sub.add_parser("stream", help="print the true state at a rate")
    st.add_argument("--rate", type=float, default=5.0)
    st.add_argument("--duration", type=float, default=None)
    st.add_argument("--json", action="store_true", help="whole JSON lines instead of the compact summary")
    d = sub.add_parser("device", help="read or set an emulated device's config (its own set_config)")
    d.add_argument("name")
    d.add_argument("settings", nargs="*", help="key=value")
    a = ap.parse_args(argv)

    try:
        with SimCtl(a.addr) as sim:
            if a.cmd == "hello":
                out = sim.hello()
            elif a.cmd == "state":
                out = sim.state()
            elif a.cmd == "stats":
                out = sim.stats()
            elif a.cmd == "current":
                fields = {"speed_mps": a.speed, "direction_deg": a.dir, "profile": a.profile}
                if a.bottom_factor is not None:
                    fields["bottom_factor"] = a.bottom_factor
                if a.gust_sigma or a.gust_dir:
                    fields["gust"] = {"sigma_mps": a.gust_sigma, "dir_sigma_deg": a.gust_dir, "time_s": a.gust_time}
                out = sim.set_current(ramp_s=a.ramp, **fields)
            elif a.cmd == "still":
                out = sim.still(ramp_s=a.ramp)
            elif a.cmd == "reposition":
                out = sim.reposition(a.x, a.y)
            elif a.cmd == "device":
                settings = {}
                for kv in a.settings:
                    k, sep, v = kv.partition("=")
                    if not sep:
                        ap.error(f"device settings are key=value, got {kv!r}")
                    settings[k] = _value(v)
                out = sim.device(a.name, **settings)
            else:                                           # stream
                try:
                    for msg in sim.stream(a.rate, a.duration):
                        print(json.dumps(msg) if a.json else _line(msg), flush=True)
                except KeyboardInterrupt:
                    pass
                return 0
    except (OSError, SimCtlError) as exc:
        print(f"simctl {a.addr}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
