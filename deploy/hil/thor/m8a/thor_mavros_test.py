#!/usr/bin/env python3
"""Minimal ArduSub check through MAVROS: wait for the link, set a mode, arm, send ONE body-frame velocity setpoint,
print the vehicle state at 1 Hz, disarm. Run on the Thor in the HIL terminal, with mavros already running
(thor_mavros_run.sh):

    source ~/hil_env.sh
    python3 ~/thor_mavros_test.py                      # GUIDED, 0.2 m/s forward (surge), 15 s
    python3 ~/thor_mavros_test.py --mode ALT_HOLD      # modes that ignore velocity setpoints: RC override instead
    python3 ~/thor_mavros_test.py --watch              # only print the state, no mode change, no arming

Frame: MAV_FRAME_BODY_NED, so x is forward (surge), y right (sway), z down (heave).
GUIDED: ArduSub accepts it only with a position estimate; if the mode change is refused the script says so and
carries on watching (try --mode ALT_HOLD or MANUAL: the "setpoint" is then an RC override of the forward channel,
RCMAP_FORWARD = 5, sent at 10 Hz because overrides expire).
Wall-clock timing everywhere: it works the same with or without /clock. The terminal output is the result; nothing is
written to disk.
"""
import argparse
import sys
import time

import rclpy
from mavros_msgs.msg import OverrideRCIn, PositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class Probe(Node):
    def __init__(self):
        super().__init__("hil_mavros_probe")
        self.state = None
        self.n_state = 0
        # reliable/volatile is compatible with MAVROS' (reliable, transient_local) state publisher
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        # best effort connects to a publisher of either reliability
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(StatusText, "/mavros/statustext/recv", self._on_text, be)
        self.pub_vel = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.pub_rc = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")

    def _on_state(self, msg):
        self.state = msg
        self.n_state += 1

    def _on_text(self, msg):
        print("  [fcu] severity %d: %s" % (msg.severity, msg.text), flush=True)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def line(self):
        s = self.state
        if s is None:
            return "no /mavros/state yet"
        return "connected=%s armed=%s guided=%s mode=%s system_status=%d (state msgs: %d)" % (
            s.connected, s.armed, s.guided, s.mode or "-", s.system_status, self.n_state)

    def tick(self, seconds):
        """spin for `seconds`, printing the state line once per second (the 1 Hz heartbeat view)"""
        t_end = time.monotonic() + seconds
        t_print = time.monotonic()
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() >= t_print:
                print("[state] " + self.line(), flush=True)
                t_print += 1.0


def velocity_target(vx, vy, vz):
    m = PositionTarget()
    m.coordinate_frame = PositionTarget.FRAME_BODY_NED
    m.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                   | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                   | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
    m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
    return m


def rc_forward(vx, vmax=0.5):
    """RC override for ArduSub: channel 5 forward, 1500 = neutral, others left alone (65535 = no change)"""
    m = OverrideRCIn()
    m.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
    m.channels[4] = int(1500 + 400 * max(-1.0, min(1.0, vx / vmax)))
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mode", default="GUIDED", help="ArduSub custom mode to set (default GUIDED)")
    ap.add_argument("--vx", type=float, default=0.2, help="body-frame forward velocity, m/s (default 0.2)")
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--vz", type=float, default=0.0)
    ap.add_argument("--hold", type=float, default=None,
                    help="resend for this many seconds at 10 Hz (default: GUIDED sends once, other modes 2 s)")
    ap.add_argument("--duration", type=float, default=15.0, help="total time to watch after arming, s")
    ap.add_argument("--timeout", type=float, default=30.0, help="wait for connected: true, s")
    ap.add_argument("--keep-armed", action="store_true", help="do not disarm at the end")
    ap.add_argument("--watch", action="store_true", help="only print the state: no mode change, no arming")
    args = ap.parse_args()
    guided = args.mode.upper() == "GUIDED"
    hold = args.hold if args.hold is not None else (0.0 if guided else 2.0)

    rclpy.init()
    node = Probe()
    rc = 1
    try:
        # 1. link
        print("[test] waiting for /mavros/state connected: true (max %.0f s)" % args.timeout, flush=True)
        t0 = time.monotonic()
        t_print = t0
        while rclpy.ok() and time.monotonic() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.state is not None and node.state.connected:
                break
            if time.monotonic() >= t_print:
                print("[state] " + node.line(), flush=True)
                t_print += 1.0
        if node.state is None or not node.state.connected:
            print("[test] FAIL: not connected after %.0f s" % args.timeout)
            return 1
        print("[test] connected. " + node.line(), flush=True)

        if args.watch:
            node.tick(args.duration)
            return 0

        # 2. mode
        r = node.call(node.set_mode, SetMode.Request(base_mode=0, custom_mode=args.mode.upper()))
        print("[test] set_mode %s -> %s" % (args.mode.upper(), "no answer" if r is None else "mode_sent=%s" % r.mode_sent),
              flush=True)
        node.spin_for(1.0)
        print("[state] " + node.line(), flush=True)
        if node.state.mode.upper() != args.mode.upper():
            print("[test] WARNING: the vehicle is in %r, not %s (see the [fcu] lines above for the reason)"
                  % (node.state.mode, args.mode.upper()), flush=True)

        # 3. arm
        r = node.call(node.arming, CommandBool.Request(value=True))
        print("[test] arm -> %s" % ("no answer" if r is None else "success=%s result=%d" % (r.success, r.result)), flush=True)
        node.spin_for(1.0)
        print("[state] " + node.line(), flush=True)

        # 4. one velocity setpoint (GUIDED), or an RC override burst
        if guided:
            node.pub_vel.publish(velocity_target(args.vx, args.vy, args.vz))
            print("[test] sent 1 body-frame velocity setpoint vx=%.2f vy=%.2f vz=%.2f" % (args.vx, args.vy, args.vz), flush=True)
            end = time.monotonic() + hold
            while hold > 0 and rclpy.ok() and time.monotonic() < end:
                node.pub_vel.publish(velocity_target(args.vx, args.vy, args.vz))
                node.spin_for(0.1)
        else:
            end = time.monotonic() + hold
            print("[test] RC override forward vx=%.2f for %.1f s" % (args.vx, hold), flush=True)
            while rclpy.ok() and time.monotonic() < end:
                node.pub_rc.publish(rc_forward(args.vx))
                node.spin_for(0.1)
            node.pub_rc.publish(rc_forward(0.0))

        # 5. watch
        node.tick(args.duration)
        rc = 0
    except KeyboardInterrupt:
        rc = 130
    finally:
        if not args.watch and not args.keep_armed and rclpy.ok() and node.state is not None and node.state.armed:
            r = node.call(node.arming, CommandBool.Request(value=False))
            print("[test] disarm -> %s" % ("no answer" if r is None else "success=%s" % r.success), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
