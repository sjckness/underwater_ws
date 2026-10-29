"""Rate, bandwidth, message age and drops per topic, with the reader's QoS taken from each publisher's.

    ros/link_monitor/run.sh [--duration 30] [--age] [--json FILE] [TOPIC ...]

Why this exists rather than `ros2 topic hz` and `ros2 topic bw`: the sensor topics are published BEST_EFFORT
(docs/topic_map.md §3), those two commands have no QoS option in Jazzy — `--qos-reliability` is rejected with a
usage error — and their default RELIABLE reader cannot match a BEST_EFFORT writer, so they measure silence and
report nothing. This node reads each publisher's offered QoS with ``get_publishers_info_by_topic`` and subscribes
with a profile that matches it, which is also the only honest way to measure a graph whose QoS classes differ per
topic.

It is the measurement side of M7 step 4 (docs/ros_hil.md §8): rate, age from ``header.stamp`` and drops from
``QoSMessageLostInfo``, plus the `incompatible_qos` event, which is what catches a RELIABLE-reader /
BEST_EFFORT-writer mismatch that otherwise presents as total silence.

Two things to read carefully in the output:

- **rate_hz is wall-clock**, and every stamp in this system is sim time, which advances at RTF x wall time. A
  10 Hz sonar in a sim running at RTF 0.85 arrives at 8.5 Hz on the wire and that is correct, not a drop. The
  sim publishes its RTF on /sim/diagnostics for exactly this division (docs/ros_hil.md §3.3).
- **bytes/s is the serialized payload**, as `ros2 topic bw` reports it, not what the NIC sees: framing overhead
  at MTU 1500 is another ~8 % (docs/ros_hil.md §2.2), so compare with the payload column of that table and read
  the wire from `ip -s link` deltas.

Subscriptions are raw (``raw=True``), so a message costs no deserialization here; `--age` deserializes only the
headers it needs, since subscriber CPU on the publisher's host is itself a term in the latency budget — twelve
`ros2 topic hz` processes once dropped the sim to RTF 0.5.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

try:                                            # renamed in Iron; both names exist in the wild
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:                             # pragma: no cover - older rclpy
    from rclpy.qos_event import SubscriptionEventCallbacks

#: the live topics of docs/topic_map.md a Jetson-side monitor may watch, in the order the table lists them
DEFAULT_TOPICS = ["/rov1/sensors/sonar_fls/image", "/rov1/sensors/sonar_fls/sonar_image",
                  "/rov1/sensors/sonar_fls/meta", "/rov1/sensors/camera_front/image_raw",
                  "/rov1/sensors/camera_front/camera_info", "/clock", "/tf_static", "/sim/diagnostics"]
#: message types whose first field is not a std_msgs/Header, so --age cannot read a stamp off the raw bytes
HEADERLESS = ("rosgraph_msgs/msg/Clock", "std_msgs/msg/String", "std_msgs/msg/Float32", "tf2_msgs/msg/TFMessage",
              "visualization_msgs/msg/MarkerArray")


class Topic:
    """One watched topic: its type and the publisher's QoS, then the counters."""

    def __init__(self, name: str):
        self.name = name
        self.age = False
        self.n = self.lost = 0
        self.bytes = 0
        self.sizes: list[int] = []
        self.stamps: list[float] = []            # header stamps [s], sim time
        self.t_first = self.t_last = None
        self.incompatible = None
        self.type_name = None
        self.qos_note = "no publisher"
        self.sub = None

    def start(self, node, age: bool) -> bool:
        info = node.get_publishers_info_by_topic(self.name)
        types = dict(node.get_topic_names_and_types()).get(self.name, [])
        if not info or not types:
            return False
        self.type_name = types[0]
        q = info[0].qos_profile
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=max(getattr(q, "depth", 10) or 10, 10),
                         reliability=q.reliability, durability=q.durability)
        self.qos_note = f"{q.reliability.name.lower()}/{q.durability.name.lower()}"
        self.age = age and self.type_name not in HEADERLESS
        events = SubscriptionEventCallbacks(message_lost=self._on_lost, incompatible_qos=self._on_incompatible)
        self.sub = node.create_subscription(get_message(self.type_name), self.name, self._on_msg, qos,
                                            raw=True, event_callbacks=events)
        return True

    def _on_msg(self, data: bytes) -> None:
        now = time.perf_counter()
        self.t_first = self.t_first if self.t_first is not None else now
        self.t_last = now
        self.n += 1
        self.bytes += len(data)
        self.sizes.append(len(data))
        if self.age:
            # the stamp is the first field of a std_msgs/Header: int32 sec, uint32 nanosec after the 4-byte CDR
            # header. Reading it off the raw bytes keeps the monitor free of per-type deserialization.
            sec = int.from_bytes(data[4:8], "little", signed=True)
            nsec = int.from_bytes(data[8:12], "little")
            self.stamps.append(sec + nsec * 1e-9)

    def _on_lost(self, info) -> None:
        self.lost += int(getattr(info, "total_count_change", 0))

    def _on_incompatible(self, info) -> None:
        self.incompatible = str(getattr(info, "last_policy_kind", info))

    def report(self, wall: float) -> dict:
        span = (self.t_last - self.t_first) if (self.n > 1 and self.t_first is not None) else 0.0
        rate = (self.n - 1) / span if span > 0 else 0.0
        out = {"topic": self.name, "type": self.type_name, "qos": self.qos_note, "n": self.n,
               "rate_hz": round(rate, 3), "mb_s": round(self.bytes / wall / 1e6, 3),
               "mbit_s": round(self.bytes * 8 / wall / 1e6, 2),
               "mean_bytes": int(statistics.mean(self.sizes)) if self.sizes else 0,
               "max_bytes": max(self.sizes) if self.sizes else 0, "lost": self.lost}
        if self.stamps and len(self.stamps) > 1:
            d = [b - a for a, b in zip(self.stamps, self.stamps[1:])]
            out["sim_dt_ms_mean"] = round(statistics.mean(d) * 1e3, 2)
            out["sim_span_s"] = round(self.stamps[-1] - self.stamps[0], 3)
        if self.incompatible:
            out["incompatible_qos"] = self.incompatible
        return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ros/link_monitor/run.sh", description=__doc__.splitlines()[0])
    ap.add_argument("topics", nargs="*", default=None, help=f"default: {' '.join(DEFAULT_TOPICS)}")
    ap.add_argument("--duration", type=float, default=30.0, help="measure for this many wall seconds")
    ap.add_argument("--wait", type=float, default=15.0, help="wait up to this long for the publishers to appear")
    ap.add_argument("--age", action="store_true", help="also read header.stamp (sim time) off the raw bytes")
    ap.add_argument("--json", default="", help="write the report here as well")
    args, ros_args = ap.parse_known_args(argv)
    names = args.topics or DEFAULT_TOPICS

    rclpy.init(args=ros_args, domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    node = rclpy.create_node("link_monitor")
    topics = [Topic(n) for n in names]
    deadline = time.perf_counter() + args.wait
    pending = list(topics)
    while pending and time.perf_counter() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        pending = [t for t in pending if not t.start(node, args.age)]
    for t in pending:
        print(f"no publisher for {t.name} after {args.wait:.0f} s", file=sys.stderr)

    wall0 = time.perf_counter()
    try:
        while rclpy.ok() and time.perf_counter() - wall0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    wall = time.perf_counter() - wall0
    rows = [t.report(wall) for t in topics if t.sub is not None]
    width = max((len(r["topic"]) for r in rows), default=10)
    print(f"{'topic'.ljust(width)}  {'qos':>22}  {'n':>6}  {'Hz':>7}  {'Mbit/s':>8}  {'mean B':>9}  {'lost':>5}")
    for r in rows:
        print(f"{r['topic'].ljust(width)}  {r['qos']:>22}  {r['n']:>6}  {r['rate_hz']:>7.3f}  "
              f"{r['mbit_s']:>8.2f}  {r['mean_bytes']:>9}  {r['lost']:>5}"
              + (f"  INCOMPATIBLE_QOS {r['incompatible_qos']}" if "incompatible_qos" in r else ""))
    report = {"wall_s": round(wall, 3), "domain": int(os.environ.get("UW_ROS_DOMAIN_ID", "73")), "topics": rows}
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(report, fh, indent=1)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
