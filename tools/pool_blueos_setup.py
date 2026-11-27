"""M11: configure a BlueOS for the simulated pool vehicle, once, through its REST API. Standard library only, so it
runs anywhere (linus, Work's Windows Python, WSL).

    python3 tools/pool_blueos_setup.py --blueos http://localhost --rtsp rtsp://<sim host>:8654/rov

1. Autopilot board "Manual" (BlueOS >= 1.5.0-beta.3): BlueOS runs no autopilot of its own and takes MAVLink from its
   master endpoint, by default udpin 0.0.0.0:14551, where the pool's ArduSub SITL sends (docker/pool/compose.yaml,
   MAVLINK_OUT=udpclient:<BlueOS IPv4>:14551). Checked against BlueOS 1.5.0-beta.43 on 2026-11-25.
2. A "Redirect" video stream in the camera manager, pulling the Isaac camera from mediamtx over RTSP; Cockpit then
   gets it over WebRTC like a real camera. An existing stream with the same name is replaced.

3. With --dvl: the Water Linked DVL extension (docker/pool/compose.yaml, service dvl or dvl-host; on a vehicle,
   BlueOS's Extensions page) set up as an operator would on its page: "DVL" parameters (the EKF's horizontal
   position and velocity from the DVL), VISION_POSITION_DELTA messages, the DVL as rangefinder, and the EKF origin
   at the SITL home ("set current position" on its map), which POSHOLD needs. It first waits for ArduSub's
   heartbeat to be live in BlueOS: those settings go to the vehicle, and a restarted SITL needs the origin again.
   So it can be started together with `uw pool` (hil/win/pool/stack.sh configure does) and does its part once the
   vehicle is up.

Cockpit itself is a BlueOS extension (Extensions -> Cockpit -> Install) or the desktop app; docs/pool.md.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

STREAM_NAME = "Isaac ROV camera"


def call(method: str, url: str, body=None, timeout: float = 10.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"} if data is not None else {})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read()
    try:
        return json.loads(raw) if raw else None
    except json.JSONDecodeError:
        return raw.decode(errors="replace")


def wait_up(blueos: str, timeout_s: float) -> None:
    t_end = time.monotonic() + timeout_s
    while True:
        try:
            call("GET", f"{blueos}/ardupilot-manager/v1.0/available_boards", timeout=3.0)
            return
        except (urllib.error.URLError, OSError):
            if time.monotonic() > t_end:
                raise SystemExit(f"BlueOS not reachable at {blueos} (is the blueos service up?)")
            time.sleep(2.0)


def set_manual_board(blueos: str, endpoint_port: int) -> None:
    ap = f"{blueos}/ardupilot-manager/v1.0"
    boards = call("GET", f"{ap}/available_boards")
    manual = next((b for b in boards if b.get("platform") == "Manual"), None)
    if manual is None:
        raise SystemExit(f"this BlueOS offers no 'Manual' board ({[b.get('name') for b in boards]}): it needs "
                         "1.5.0-beta.3 or later (docker/pool/compose.yaml pins one)")
    master = call("GET", f"{ap}/endpoints/manual_board_master_endpoint")
    want = {**master, "connection_type": "udpin", "place": "0.0.0.0", "argument": int(endpoint_port)}
    if (master["connection_type"], master["place"], int(master["argument"])) != ("udpin", "0.0.0.0", endpoint_port):
        call("POST", f"{ap}/endpoints/manual_board_master_endpoint", want)
        print(f"  master endpoint set to udpin 0.0.0.0:{endpoint_port}")
    current = call("GET", f"{ap}/board")
    if not current or current.get("platform") != "Manual":
        call("POST", f"{ap}/board", manual, timeout=60.0)
        print("  board set to Manual")
    print(f"  autopilot: board Manual, MAVLink in on UDP {endpoint_port}")


def set_stream(blueos: str, rtsp: str) -> None:
    cm = f"{blueos}/mavlink-camera-manager"
    for s in call("GET", f"{cm}/streams") or []:
        if s["video_and_stream"]["name"] == STREAM_NAME:
            call("DELETE", f"{cm}/delete_stream?name={urllib.parse.quote(STREAM_NAME)}")
            print(f"  removed the old '{STREAM_NAME}' stream")
    call("POST", f"{cm}/streams", {
        "name": STREAM_NAME, "source": "Redirect",
        "stream_information": {"endpoints": [rtsp], "configuration": {"type": "redirect"},
                               "extended_configuration": {"thermal": False, "disable_mavlink": False,
                                                          "disable_zenoh": False, "disable_lazy": False,
                                                          "disable_recording": True, "disable_thumbnails": False}}})
    st = [s for s in call("GET", f"{cm}/streams") if s["video_and_stream"]["name"] == STREAM_NAME]
    if not st:
        raise SystemExit("the camera manager did not keep the stream")
    print(f"  video: '{STREAM_NAME}' <- {rtsp} ({st[0]['state']}{', ' + st[0]['error'] if st[0]['error'] else ''})")


def wait_vehicle(blueos: str, timeout_s: float) -> None:
    """Until ArduSub's heartbeat is live in BlueOS's mavlink2rest: SITL runs (it steps only once the physics process
    answers) and its MAVLink reaches BlueOS. mavlink2rest keeps the last message of a vehicle that went away, so
    live means a counter that moves."""
    url = f"{blueos}/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT"
    t_end = time.monotonic() + timeout_s
    last = None
    while True:
        try:
            n = int(call("GET", url, timeout=3.0)["status"]["time"]["counter"])
            if last is not None and n > last:
                print("  vehicle: ArduSub's heartbeat reaches BlueOS")
                return
            last = n
        except (urllib.error.URLError, OSError, KeyError, TypeError, ValueError):
            pass
        if time.monotonic() > t_end:
            raise SystemExit("no live heartbeat from ArduSub in BlueOS's mavlink2rest: is the physics process "
                             "(python -m uw pool) up, and SITL sending to BlueOS (MAVLINK_OUT)?")
        time.sleep(2.0)


def set_dvl(dvl: str, origin: tuple, timeout_s: float) -> None:
    """The extension's own API (its main.py, v1.0.11): plain GETs that answer "True" / "False"."""
    t_end = time.monotonic() + timeout_s
    while True:
        try:
            st = call("GET", f"{dvl}/get_status", timeout=3.0)
            break
        except (urllib.error.URLError, OSError):
            if time.monotonic() > t_end:
                raise SystemExit(f"the DVL extension is not reachable at {dvl} (is the dvl service up?)")
            time.sleep(2.0)
    print(f"  DVL extension: {st.get('status')!r}, DVL at {st.get('hostname')}")
    t_end = time.monotonic() + timeout_s
    while st.get("status") != "Running":                # the parameters go to the vehicle: wait for it and the DVL
        if time.monotonic() > t_end:
            raise SystemExit(f"the DVL extension did not reach 'Running' (last: {st.get('status')!r}): is the "
                             "physics process (python -m uw pool) up, and SITL sending to BlueOS?")
        time.sleep(2.0)
        st = call("GET", f"{dvl}/get_status", timeout=3.0)
    for path, what in (("load_params/dvl", "EKF3 sources: DVL for horizontal position and velocity"),
                       ("message_type/POSITION_DELTA", "messages: VISION_POSITION_DELTA"),
                       ("use_as_rangefinder/true", "rangefinder: on"),
                       (f"setcurrentposition/{origin[0]}/{origin[1]}", f"EKF origin: {origin[0]}, {origin[1]}")):
        ok = call("GET", f"{dvl}/{path}", timeout=30.0)
        print(f"  {what}{'' if str(ok) in ('True', 'None') else f' (answered {ok!r})'}")


def set_nucleus(url: str, origin: tuple, timeout_s: float) -> None:
    """Nortek's extension (nortekgroup/blueos_nucleus 2.0.11, its Flask API): it connects and starts the Nucleus by
    itself (compose seeds autoconnect/autostart); wait until it runs and has the vehicle, then set the EKF origin as
    the operator would on its map (it sends SET_GPS_GLOBAL_ORIGIN)."""
    t_end = time.monotonic() + timeout_s
    st = {}
    while True:
        try:
            st = call("GET", f"{url}/api/get_status", timeout=3.0) or {}
        except (urllib.error.URLError, OSError):
            st = {}
        if st.get("running") and st.get("heartbeat"):
            break
        if time.monotonic() > t_end:
            raise SystemExit(f"Nortek's extension at {url} is not running with the vehicle (last status {st}): is the "
                             "physics up with --preset nucleus, the nucleus service up and dvl-gateway answering?")
        time.sleep(2.0)
    print(f"  Nucleus extension: connected and running; cable_guy {st.get('cable_guy')}, origin {st.get('gps_origin')}")
    call("POST", f"{url}/api/set_current_position", {"lat": origin[0], "lon": origin[1]}, timeout=10.0)
    print(f"  EKF origin: {origin[0]}, {origin[1]}")
    for kind in ("delta", "estimate"):
        st = call("GET", f"{url}/api/get_status_position_{kind}", timeout=3.0) or {}
        print(f"  VISION_POSITION_{kind.upper()}: {'on' if st.get('enabled') else 'OFF'}, {st.get('frequency')} Hz")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--blueos", default="http://localhost", help="BlueOS base URL (default %(default)s)")
    ap.add_argument("--rtsp", required=True, help="the pool camera, e.g. rtsp://100.89.193.76:8654/rov")
    ap.add_argument("--mavlink-port", type=int, default=14551, help="Manual board master endpoint, UDP")
    ap.add_argument("--wait", type=float, default=120.0,
                    help="seconds to wait for BlueOS to come up, and with --dvl for the vehicle and the extension")
    ap.add_argument("--dvl", default="", help="the Water Linked DVL extension's page: http://172.31.250.3 on linus "
                                              "(dvl-gateway), http://127.0.0.1:9001 on the Work (dvl-host); "
                                              "default: leave it alone")
    ap.add_argument("--nucleus", default="", help="Nortek's Nucleus extension instead (M12): http://172.31.250.7 on "
                                                  "linus (compose profile nucleus)")
    ap.add_argument("--origin", default="45.0703,7.6869",
                    help="EKF origin lat,lon for --dvl / --nucleus (default: the compose file's HOME_LOC, %(default)s)")
    args = ap.parse_args(argv)
    blueos = args.blueos.rstrip("/")
    print(f"BlueOS {blueos}:")
    wait_up(blueos, args.wait)
    set_manual_board(blueos, args.mavlink_port)
    set_stream(blueos, args.rtsp)
    if args.dvl or args.nucleus:
        lat, lon = (float(v) for v in args.origin.split(","))
        wait_vehicle(blueos, args.wait)
        if args.dvl:
            set_dvl(args.dvl.rstrip("/"), (lat, lon), args.wait)
        if args.nucleus:
            set_nucleus(args.nucleus.rstrip("/"), (lat, lon), args.wait)
    print(f"done. Open {blueos} (BlueOS) and Cockpit; the vehicle appears once SITL sends to UDP {args.mavlink_port}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
