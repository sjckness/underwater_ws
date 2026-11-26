"""M11 pool smoke test: fly the simulated BlueROV2 Heavy through ArduSub the way a pilot would, and check that it
goes where the sticks say.

Needs ArduSub SITL (docker/pool/compose.yaml, service sitl) and the physics: `python -m uw pool` or, without a GPU,
`python -m uw py -m uw.rov.physics_proc`. Talks MAVLink on SITL's second output (udpclient 127.0.0.1:14560) as GCS
255, like Cockpit, sending MANUAL_CONTROL at 20 Hz; the truth comes back from SITL's own SIM_STATE, which is the
state our physics replied with.

    python3 tools/pool_smoke.py                 # host (pymavlink goes into ~/.cache/uw_pydeps on first use)
    python -m uw py tools/pool_smoke.py         # or in the container

Checks, in order: arm in MANUAL; dive on "throttle down"; ALT_HOLD holds depth; forward, lateral and yaw move the
vehicle the right way; the depth ArduSub estimates from its baro matches the simulated one; climb; disarm.
Exit code 0 only if every check passed.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from uw import deps  # noqa: E402

MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "MANUAL": 19}
MSG = {"ATTITUDE": 30, "SCALED_PRESSURE": 29, "SERVO_OUTPUT_RAW": 36, "VFR_HUD": 74, "SIM_STATE": 108}
# sub.parm's joystick map (the one Cockpit and QGC start from): BTN10 tilt up, BTN14 lights brighter
BTN_TILT_UP, BTN_LIGHTS_UP, BTN_LIGHTS_DOWN = 10, 14, 13
EARTH_R = 6378137.0


class Pilot:
    def __init__(self, url: str, rate_hz: float = 20.0):
        mavutil = deps.require("pymavlink.mavutil", "pymavlink", "2.4.49")
        self.mavutil = mavutil
        self.ml = mavutil.mavlink
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.dt = 1.0 / rate_hz
        self.last = {}
        self.results: list[tuple[str, bool, str]] = []

    # ---- plumbing
    def wait_heartbeat(self, timeout: float = 30.0) -> None:
        hb = self.m.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise SystemExit(f"no heartbeat in {timeout:.0f} s: is SITL running "
                             "(with --serial1 udpclient:127.0.0.1:14560)?")
        for name, mid in MSG.items():
            self.command(self.ml.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / 10.0)

    def command(self, cmd, *params) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def pump(self, seconds: float, x=0, y=0, z=500, r=0, buttons=0) -> None:
        """Send MANUAL_CONTROL at the pilot rate for ``seconds`` and keep the latest of every message."""
        t_end = time.monotonic() + seconds
        while True:
            self.m.mav.manual_control_send(self.m.target_system, int(x), int(y), int(z), int(r), buttons)
            t_next = time.monotonic() + self.dt
            while True:
                msg = self.m.recv_match(blocking=True, timeout=max(0.0, t_next - time.monotonic()))
                if msg is not None:
                    self.last[msg.get_type()] = msg
                if time.monotonic() >= t_next:
                    break
            if time.monotonic() >= t_end:
                return

    def param(self, name: str, timeout: float = 3.0):
        """The vehicle's value of one parameter (None if it does not answer)."""
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id.rstrip("\x00") == name:
                return msg.param_value
        return None

    def set_mode(self, name: str) -> None:
        self.m.set_mode(MODES[name])
        self.pump(1.0)

    @property
    def armed(self) -> bool:
        hb = self.last.get("HEARTBEAT")
        return bool(hb and hb.base_mode & self.ml.MAV_MODE_FLAG_SAFETY_ARMED)

    # ---- the truth (SIM_STATE: what our physics replied) and ArduSub's estimate
    def truth(self):
        s = self.last.get("SIM_STATE")
        if s is None:
            raise SystemExit("no SIM_STATE from SITL: cannot read the simulated state")
        lat, lon = s.lat * (1e-7 if abs(s.lat) > 1000 else 1.0), s.lon * (1e-7 if abs(s.lon) > 1000 else 1.0)
        north = math.radians(lat) * EARTH_R
        east = math.radians(lon) * EARTH_R * math.cos(math.radians(lat))
        return north, east, -float(s.alt), float(s.yaw)          # depth below the home altitude (0 at the surface)

    def depth_estimate(self) -> float:
        return -float(self.last["VFR_HUD"].alt)

    def check(self, name: str, ok: bool, detail: str) -> bool:
        self.results.append((name, bool(ok), detail))
        print(f"  [{'ok' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        return ok


def displacement(a, b, heading: float):
    """(forward, starboard) displacement [m] from truth a to truth b in the frame of ``heading`` (NED yaw)."""
    dn, de = b[0] - a[0], b[1] - a[1]
    return dn * math.cos(heading) + de * math.sin(heading), -dn * math.sin(heading) + de * math.cos(heading)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--url", default="udpin:0.0.0.0:14560", help="pymavlink connection (default: %(default)s)")
    # ALT_HOLD does not sit still: the T200's +-25 us dead band around 1500 us and the +2 N of buoyancy (which needs
    # ~1540 us to cancel, just outside it) make ArduSub's depth loop limit-cycle by a few cm with a period of ~4 s.
    # Measured 2026-11-25 with sub.parm's gains: 5-16 cm peak to peak (18 cm with the firmware defaults).
    ap.add_argument("--hold-tol", type=float, default=0.20,
                    help="ALT_HOLD depth, peak to peak [m] (default %(default)s)")
    args = ap.parse_args(argv)

    p = Pilot(args.url)
    print("waiting for the vehicle ...", flush=True)
    p.wait_heartbeat()
    p.pump(2.0)
    print(f"vehicle {p.m.target_system}/{p.m.target_component}; flying", flush=True)

    # the parameters our physics assumes (tests/rov/test_motor_map.py): a real vehicle's parameter file loaded into
    # SITL can break them (a reversed MOTn_DIRECTION is a propeller fix on hardware, a sign error here)
    expect = {"FRAME_CONFIG": 2, "SERVO8_FUNCTION": 40, "SERVO9_FUNCTION": 7, "SERVO10_FUNCTION": 181,
              **{f"MOT_{i}_DIRECTION": 1 for i in range(1, 9)}}
    got = {k: p.param(k) for k in expect}
    bad = {k: v for k, v in got.items() if v is None or round(v) != expect[k]}
    p.check("parameters", not bad, "as pool.parm sets them" if not bad else f"unexpected {bad}")

    p.set_mode("MANUAL")
    p.command(p.ml.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    p.pump(2.0)
    if not p.check("arm in MANUAL", p.armed, "armed" if p.armed else "refused (see SITL's STATUSTEXT)"):
        return 1

    t0 = p.truth()
    p.pump(5.0, z=200)                                       # throttle down: dive
    t1 = p.truth()
    p.check("dive", t1[2] - t0[2] > 0.5, f"depth {t0[2]:.2f} -> {t1[2]:.2f} m")

    p.set_mode("ALT_HOLD")
    track = []
    for _ in range(16):                                      # 8 s to settle, then 8 s that must hold
        p.pump(1.0)
        track.append(p.truth()[2])
    print("    ALT_HOLD depth, 1 s apart [m]:", " ".join(f"{d:.3f}" for d in track), flush=True)
    held = track[8:]
    p.check("ALT_HOLD holds depth", max(held) - min(held) < args.hold_tol,
            f"{100 * (max(held) - min(held)):.1f} cm peak to peak over 8 s, after 8 s to settle")
    b = p.truth()
    p.check("ArduSub's depth matches the sim", abs(p.depth_estimate() - b[2]) < 0.1,
            f"baro {p.depth_estimate():.3f} m, sim {b[2]:.3f} m")

    a = p.truth()
    p.pump(4.0, x=600)
    b = p.truth()
    fwd, stb = displacement(a, b, a[3])
    p.check("forward", fwd > 0.5 and abs(stb) < 0.5 * fwd, f"{fwd:+.2f} m ahead, {stb:+.2f} m to starboard")
    p.pump(2.0)

    a = p.truth()
    p.pump(4.0, y=600)
    b = p.truth()
    fwd, stb = displacement(a, b, a[3])
    p.check("lateral", stb > 0.3 and abs(fwd) < 0.5 * stb, f"{stb:+.2f} m to starboard, {fwd:+.2f} m ahead")
    p.pump(2.0)

    a = p.truth()
    p.pump(2.0, r=400)
    b = p.truth()
    dyaw = math.degrees(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
    p.check("yaw right", dyaw > 20.0, f"heading {math.degrees(a[3]):.0f} -> {math.degrees(b[3]):.0f} deg ({dyaw:+.0f})")
    p.pump(2.0)

    a = p.truth()
    p.pump(4.0, z=800)
    b = p.truth()
    p.check("climb", a[2] - b[2] > 0.3, f"depth {a[2]:.2f} -> {b[2]:.2f} m")

    # the aux outputs the sim reads (vehicle.camera.tilt_servo, vehicle.lights.servo): a button press must move them
    s0 = p.last["SERVO_OUTPUT_RAW"]
    for _ in range(4):
        p.pump(0.3, buttons=1 << BTN_LIGHTS_UP)
        p.pump(0.3)
    p.pump(0.5)
    s1 = p.last["SERVO_OUTPUT_RAW"]
    p.check("lights (BTN14 x4)", s1.servo10_raw >= s0.servo10_raw + 300,
            f"SERVO10 {s0.servo10_raw} -> {s1.servo10_raw} us")
    p.pump(1.5, buttons=1 << BTN_TILT_UP)
    p.pump(0.5)
    s2 = p.last["SERVO_OUTPUT_RAW"]
    p.check("camera tilt up (BTN10 held)", s2.servo9_raw > s0.servo9_raw + 100,
            f"SERVO9 {s0.servo9_raw} -> {s2.servo9_raw} us")
    for _ in range(4):
        p.pump(0.3, buttons=1 << BTN_LIGHTS_DOWN)
        p.pump(0.3)

    p.set_mode("MANUAL")
    p.command(p.ml.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    p.pump(2.0)
    p.check("disarm", not p.armed, "disarmed" if not p.armed else "still armed")

    failed = [n for n, ok, _ in p.results if not ok]
    print(f"\n{len(p.results) - len(failed)}/{len(p.results)} checks passed"
          + (f"; failed: {failed}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
