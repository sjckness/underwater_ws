"""Host-side DVL driver stand-in: /sim/dvl/report (the A50's own JSON) -> /<vehicle>/sensors/dvl/**.

The simulator emits the device's velocity-and-transducer report verbatim, so this node plays exactly the
part the real driver plays on the vehicle. Swap the A50 and ``ndahn/dvl_a50`` in for the simulator and
this relay, and nothing downstream of ``/rov1/sensors/dvl/**`` changes - which is what
``Claude outputs/update_20261113.html`` means by running the same dead reckoning in both places.

It exists as a separate host node, rather than as publishers inside the sim, for one hard reason:
``marine_acoustic_msgs`` is not in the Isaac container's bundled rclpy (docs/topic_map.md §4.5), so the
``Dvl`` message cannot be built there at all. ``ros/sonar_relay`` exists for the same reason.

Runs with the host's ROS 2 Jazzy (``ros-jazzy-marine-acoustic-msgs``), on the tank sim's own domain:

    ros/dvl_relay/run.sh [--vehicle rov1]

or, sourced: ``python3 ros/dvl_relay/dvl_relay/relay.py``, or as a colcon package (``ros2 run dvl_relay
relay``). The domain is ``UW_ROS_DOMAIN_ID`` (default 73), as in the container; a ``ROS_DOMAIN_ID``
exported for another stack is ignored on purpose.

Frames, and the one place the axis convention changes. The A50 reports in its own body frame, FRD:
x forward, y starboard, z down. ``marine_acoustic_msgs/Dvl`` declares the same convention for itself
("z-axis in direction DVL is pointing, x-axis aligned with vehicle forward"), so that message is
published untouched in ``rov1/dvl_instrument``. REP-103 wants FLU, so the ``TwistWithCovarianceStamped``
the estimator consumes is flipped into ``rov1/dvl_link`` - once, here, and nowhere else. The two frames
are a fixed static transform published by the sim (``uw.ros_bridge.R_DVL_INSTRUMENT``).
"""
from __future__ import annotations

import argparse
import json
import math
import os

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3
from marine_acoustic_msgs.msg import Dvl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String

BEAMS = 4
#: FRD -> FLU, the involution of uw.rov.frames.M_FLU_FRD. The only axis change in this file.
FLIP = (1.0, -1.0, -1.0)

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
#: class R of docs/topic_map.md §3: scalars and estimates are RELIABLE depth 10. A DVL report is ~400 B
#: at 15 Hz, so reliability costs nothing here, unlike on the image topics.
RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def _stamp(msg, micros: int) -> None:
    msg.header.stamp.sec = int(micros // 1_000_000)
    msg.header.stamp.nanosec = int((micros % 1_000_000) * 1000)


def dvl_msg(report: dict, meta: dict, frame: str) -> Dvl:
    """One report as ``marine_acoustic_msgs/Dvl`` (pure: tested without a ROS graph).

    Stays in the instrument's FRD frame, which is what the message itself declares. ``beam_unit_vec``
    comes from the latched meta, never from the driver that inspired this one: ``ndahn/dvl_a50``
    hardcodes vectors with sin and cos swapped, which put the beams 22.5 deg above the horizontal
    instead of 22.5 deg off nadir.
    """
    tr = report["transducers"]
    if len(tr) != BEAMS:
        raise ValueError(f"a DVL report needs {BEAMS} transducers, got {len(tr)}")
    unit = meta["beam_unit_vec"]
    if len(unit) != BEAMS:
        raise ValueError(f"the meta needs {BEAMS} beam_unit_vec, got {len(unit)}")
    out = Dvl()
    _stamp(out, int(report["time_of_validity"]))
    out.header.frame_id = frame
    out.velocity_mode = Dvl.DVL_MODE_BOTTOM
    out.dvl_type = Dvl.DVL_TYPE_PISTON               # four piston transducers, one per beam
    valid = bool(report["velocity_valid"])
    out.velocity = Vector3(x=float(report["vx"]), y=float(report["vy"]), z=float(report["vz"]))
    cov = report.get("covariance") or []
    out.velocity_covar = [float(v) for row in cov for v in row] if valid and len(cov) == 3 else [-1.0] * 9
    out.altitude = float(report["altitude"]) if valid else -1.0
    out.course_gnd = math.atan2(float(report["vy"]), float(report["vx"])) if valid else 0.0
    out.speed_gnd = math.hypot(float(report["vx"]), float(report["vy"])) if valid else 0.0
    out.num_good_beams = sum(1 for t in tr if t["beam_valid"])
    out.sound_speed = float(meta["sound_speed_mps"])
    out.beam_ranges_valid = True
    out.beam_velocities_valid = True
    out.beam_unit_vec = [Vector3(x=float(u[0]), y=float(u[1]), z=float(u[2])) for u in unit]
    out.range = [float(t["distance"]) if t["distance"] is not None else -1.0 for t in tr]
    out.range_covar = [-1.0] * BEAMS                 # the device reports no per-range variance
    # beam_quality is DVL-specific by the message's own admission: here it is the beam's SNR in dB
    out.beam_quality = [float(t["rssi"] - t["nsd"]) for t in tr]
    out.beam_velocity = [float(t["velocity"]) for t in tr]
    out.beam_velocity_covar = [-1.0] * BEAMS
    return out


def twist_msg(report: dict, frame: str) -> TwistWithCovarianceStamped:
    """The velocity as REP-103 FLU, for the estimator (pure).

    The covariance is rotated with the vector: ``C' = F C F^T`` with ``F = diag(1, -1, -1)``, which for a
    diagonal flip is a sign change on the off-diagonal x-y and x-z blocks. Getting this wrong is silent -
    the filter simply trusts the wrong axes - so it is asserted in the tests.
    """
    out = TwistWithCovarianceStamped()
    _stamp(out, int(report["time_of_validity"]))
    out.header.frame_id = frame
    v = [float(report["vx"]), float(report["vy"]), float(report["vz"])]
    out.twist.twist.linear = Vector3(x=v[0] * FLIP[0], y=v[1] * FLIP[1], z=v[2] * FLIP[2])
    cov = report.get("covariance") or []
    big = 1e6                                        # no fix: a covariance the filter cannot act on
    full = [0.0] * 36
    for i in range(3):
        for j in range(3):
            c = float(cov[i][j]) * FLIP[i] * FLIP[j] if (report["velocity_valid"] and len(cov) == 3) else (
                big if i == j else 0.0)
            full[i * 6 + j] = c
    for k in range(3, 6):                            # the DVL measures no angular rate
        full[k * 6 + k] = -1.0
    out.twist.covariance = full
    return out


def range_msg(report: dict, meta: dict, frame: str) -> Range:
    """The altitude as ``sensor_msgs/Range`` (pure). Out of lock it reports ``+inf``, which is how a
    range sensor says "no return" without pretending to a number."""
    out = Range()
    _stamp(out, int(report["time_of_validity"]))
    out.header.frame_id = frame
    out.radiation_type = Range.ULTRASOUND
    out.field_of_view = math.radians(float(meta["beam_width_deg"]))
    out.min_range = float(meta["min_altitude_m"])
    out.max_range = float(meta["max_altitude_m"])
    alt = report["altitude"]
    out.range = float(alt) if report["velocity_valid"] and alt == alt else float("inf")
    return out


class Relay(Node):
    def __init__(self, vehicle: str = "rov1", source: str = "/sim/dvl/report"):
        super().__init__("dvl_relay")
        self.link = f"{vehicle}/dvl_link"
        self.instrument = f"{vehicle}/dvl_instrument"
        self.count = self.locked = 0
        self.meta = None
        base = f"/{vehicle}/sensors/dvl"
        self.pub_dvl = self.create_publisher(Dvl, f"{base}/velocity", RELIABLE)
        self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, f"{base}/velocity_twist", RELIABLE)
        self.pub_range = self.create_publisher(Range, f"{base}/altitude", RELIABLE)
        self.pub_meta = self.create_publisher(String, f"{base}/meta", LATCHED)
        self.create_subscription(String, source, self.on_report, RELIABLE)
        self.create_subscription(String, "/sim/dvl/meta", self.on_meta, LATCHED)
        self.get_logger().info(f"relaying {source} -> {base}/{{velocity,velocity_twist,altitude}} "
                               f"(Dvl in {self.instrument}, Twist in {self.link})")

    def on_meta(self, msg: String) -> None:
        self.meta = json.loads(msg.data)
        self.pub_meta.publish(msg)                  # straight through: it describes the instrument only

    def on_report(self, msg: String) -> None:
        if self.meta is None:
            self.get_logger().warning("no /sim/dvl/meta yet: report dropped", throttle_duration_sec=5.0)
            return
        report = json.loads(msg.data)
        self.pub_dvl.publish(dvl_msg(report, self.meta, self.instrument))
        self.pub_twist.publish(twist_msg(report, self.link))
        self.pub_range.publish(range_msg(report, self.meta, self.link))
        self.count += 1
        self.locked += int(bool(report["velocity_valid"]))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--vehicle", default="rov1", help="namespace: /<vehicle>/sensors/dvl/... (ros.vehicle)")
    ap.add_argument("--source", default="/sim/dvl/report", help="the simulator's raw report topic")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args, domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    node = Relay(args.vehicle, args.source)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"relayed {node.count} reports ({node.locked} with bottom lock)")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
