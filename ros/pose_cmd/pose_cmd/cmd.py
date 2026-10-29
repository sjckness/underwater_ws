"""Producer for /sim/cmd/rov1/pose: the commanded sonar pose the sim follows when sonar.pose_source = ros
(uw/ros_bridge.RosPose). Until this existed, that input had no producer at all.

Runs with the host's ROS 2 Jazzy, on the tank sim's own domain (UW_ROS_DOMAIN_ID, default 73):

    ros/pose_cmd/run.sh hold [--position X Y Z] [--rpy R P Y] [--look-at X Y Z]
    ros/pose_cmd/run.sh trajectory [--preset approach|orbit|lawnmower|hover|poses] [--loop]
    ros/pose_cmd/run.sh keys        # wasd translate, rf up/down, jl yaw, ik pitch, space stop, q quit

All three publish geometry_msgs/PoseStamped in the **map** frame at --rate (default 20 Hz), body x forward,
y port, z up, pitch nose-down positive, as uw/pose.py defines it. The sim polls the latest message once per
ping, so the command is quantized to the ping period whatever rate is used here (docs/ros_hil.md §5).

`trajectory` replays the configured `sonar.trajectory` through `uw.pose.sensor_pose`, the same code the datasets
were generated with, so a HIL run and a dataset of the same config see the same motion. `--profile`, `--preset`
and `--set` are the usual config arguments and must match the sim's, or the two disagree about the tank.

Sim-only by construction: a commanded pose has no counterpart on a real vehicle, which is why it lives under
/sim and not under /rov1/control (docs/topic_map.md §4.9). It is **not** ground truth — it is an input, and the
sim's resulting true pose still comes back on /gt/sonar_pose.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS))

from uw import config, geometry, pose as uwpose      # noqa: E402 - after sys.path

TOPIC = "/sim/cmd/rov1/pose"
FRAME = "map"
KEYS = {"w": ("x", +1.0), "s": ("x", -1.0), "a": ("y", +1.0), "d": ("y", -1.0),
        "r": ("z", +1.0), "f": ("z", -1.0), "j": ("yaw", +1.0), "l": ("yaw", -1.0),
        "i": ("pitch", -1.0), "k": ("pitch", +1.0)}      # pitch is nose-down positive


def held_pose(position, rpy_deg, look_at=None) -> tuple[np.ndarray, np.ndarray]:
    """One pose: ``rpy_deg`` as given, or, with ``look_at``, the roll kept and the pitch and yaw that point the
    body x axis at that point (uw.pose.aim with bearing 0, elevation 0 — the same aiming the `track` trajectories
    use, so a held view is comparable with a dataset view)."""
    position = np.asarray(position, float)
    rpy = np.asarray(rpy_deg, float).copy()
    if look_at is not None:
        pitch, yaw = uwpose.aim(position, np.asarray(look_at, float), float(rpy[0]))
        rpy[1], rpy[2] = pitch, yaw
    return position, rpy


def traj_time(t: float, duration: float | None, loop: bool) -> float:
    """Where on the trajectory to sample at wall time ``t``: wrapped when ``loop``, otherwise held at its end, so
    a finished trajectory keeps commanding its last pose instead of stopping the sim's pose input dead."""
    if not duration:
        return t
    return t % duration if loop else min(t, duration)


class KeyDrive:
    """Keyboard teleop state: body-frame velocity from the last key, integrated and clipped to the water volume.

    Pure (no terminal, no ROS) so it can be tested: ``apply(key)`` sets the active axis, ``step(dt)`` integrates.
    One key at a time, which is what a raw terminal gives us; a held key repeats at the terminal's rate."""

    def __init__(self, position, rpy_deg, bounds=None, speed: float = 0.15, yaw_rate: float = 20.0):
        self.position = np.asarray(position, float).copy()
        self.rpy = np.asarray(rpy_deg, float).copy()
        self.bounds, self.speed, self.yaw_rate = bounds, float(speed), float(yaw_rate)
        self.axis, self.sign = None, 0.0

    def apply(self, key: str) -> bool:
        """A key: True to keep running, False on quit. Space (or any unknown key) stops the motion."""
        if key in ("q", "\x03", "\x04"):
            return False
        self.axis, self.sign = KEYS.get(key, (None, 0.0))
        return True

    def step(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        if self.axis in ("x", "y", "z"):
            body = np.zeros(3)
            body["xyz".index(self.axis)] = self.sign * self.speed * dt
            self.position = self.position + geometry.rot_rpy_deg(self.rpy) @ body
            if self.bounds is not None:
                self.position = np.clip(self.position, self.bounds.lo, self.bounds.hi)
        elif self.axis == "yaw":
            self.rpy[2] = (self.rpy[2] + self.sign * self.yaw_rate * dt + 180.0) % 360.0 - 180.0
        elif self.axis == "pitch":
            self.rpy[1] = float(np.clip(self.rpy[1] + self.sign * self.yaw_rate * dt, -89.0, 89.0))
        return self.position, self.rpy


def _publisher(topic: str, sim_time: bool = True):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    # use_sim_time by default, like every node but the sim itself (docs/topic_map.md §5): the stamp is then
    # comparable with the ping's and with /gt/sonar_pose. With no /clock on the wire it reads 0, which is
    # harmless here (the sim reads only the pose) but makes --wall-time the right flag for a standalone check.
    node = rclpy.create_node("sim_pose_cmd",
                             parameter_overrides=[Parameter("use_sim_time", value=bool(sim_time))])
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(PoseStamped, topic, qos)

    def send(position, rpy_deg) -> None:
        m = PoseStamped()
        m.header.frame_id = FRAME
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
        from uw.ros_bridge import quat_xyzw
        q = quat_xyzw(geometry.rot_rpy_deg(rpy_deg))
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
        pub.publish(m)

    return rclpy, node, send


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ros/pose_cmd/run.sh", description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=("hold", "trajectory", "keys"))
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--rate", type=float, default=20.0, help="publish rate [Hz]")
    ap.add_argument("--position", type=float, nargs=3, default=None, help="hold/keys start [m] (default: config)")
    ap.add_argument("--rpy", type=float, nargs=3, default=None, help="hold/keys roll, pitch (nose down +), yaw [deg]")
    ap.add_argument("--look-at", type=float, nargs=3, default=None, help="hold: aim the x axis at this point")
    ap.add_argument("--loop", action="store_true", help="trajectory: restart at its end instead of holding the last")
    ap.add_argument("--speed", type=float, default=0.15, help="keys: translation speed [m/s]")
    ap.add_argument("--wall-time", action="store_true",
                    help="stamp with the wall clock instead of /clock (use_sim_time false)")
    config.add_args(ap.add_argument)
    args, ros_args = ap.parse_known_args(argv)
    cfg = config.from_args(args)
    son = config.get(cfg, "sonar")
    position = args.position if args.position is not None else son["position"]
    rpy = args.rpy if args.rpy is not None else [son["roll_deg"], son["pitch_deg"], son["yaw_deg"]]
    dt = 1.0 / max(args.rate, 1e-6)

    if args.mode == "trajectory":
        scene = geometry.build_scene(cfg)
        sp = uwpose.sensor_pose(cfg, scene)
        if not getattr(sp, "moving", False):
            ap.error(f"sonar.pose_source is {son['pose_source']!r}: trajectory mode needs "
                     f"--set sonar.pose_source=trajectory (and a sonar.trajectory.preset)")
        duration = float(getattr(sp, "duration", 0.0)) or None
    else:
        scene = sp = duration = None

    rclpy, node, send = _publisher(args.topic, sim_time=not args.wall_time)
    drive = None
    if args.mode == "keys":
        bounds = uwpose.Bounds(geometry.build_scene(cfg), float(son["trajectory"]["margin"]))
        drive = KeyDrive(position, rpy, bounds, speed=args.speed, yaw_rate=20.0)
        print(f"keys -> {args.topic}: wasd translate, rf up/down, jl yaw, ik pitch, space stop, q quit",
              file=sys.stderr)
    else:
        print(f"{args.mode} -> {args.topic} at {args.rate:.0f} Hz"
              + (f", {duration:.1f} s{' looping' if args.loop else ''}" if duration else ""), file=sys.stderr)

    fd = sys.stdin.fileno() if args.mode == "keys" and sys.stdin.isatty() else None
    saved = termios.tcgetattr(fd) if fd is not None else None
    t = 0.0
    n = 0
    try:
        if fd is not None:
            tty.setcbreak(fd)
        while rclpy.ok():
            if args.mode == "hold":
                p, a = held_pose(position, rpy, args.look_at)
            elif args.mode == "trajectory":
                here = sp.at(traj_time(t, duration, args.loop))
                p, a = here.position, here.rpy_deg
            else:
                if fd is not None and select.select([sys.stdin], [], [], 0.0)[0]:
                    if not drive.apply(sys.stdin.read(1)):
                        break
                p, a = drive.step(dt)
            send(p, a)
            n += 1
            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        if fd is not None and saved is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print(f"{n} poses published", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
