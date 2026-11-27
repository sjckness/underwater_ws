#!/usr/bin/env python3
"""M7 acceptance: per-topic rate / gaps / bandwidth for the sensors-only HIL leg, plus /clock RTF.

Run in a shell where `source ~/hil_env.sh` was done:
    python3 hil/thor/m7/measure.py --duration 120 --warmup 5   # writes results/measure.json next to this script

Everything "wall" is the Thor's local CLOCK_MONOTONIC (never compared with the sim PC's clock).
Everything "sim" comes from header stamps or /clock as produced by Isaac.

Latency proxy (NOT one-way latency): for each image, (latest /clock value received on the Thor at the
moment the image arrives) - (image header stamp), in sim seconds. It includes /clock's own transport
delay and its publish period, so it is only a proxy.
"""
import argparse
import json
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

IMAGE_TOPICS = ['/oculus/polar', '/oculus/polar_db', '/oculus/fan', '/camera/image_raw']

# Subscriber QoS. Publishers are RELIABLE / VOLATILE (images) and BEST_EFFORT / VOLATILE (/clock),
# read with `ros2 topic info -v`; these profiles are compatible with them.
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
CLOCK_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def pct(xs, p):
    if not xs:
        return None
    xs = sorted(xs)
    k = min(len(xs) - 1, max(0, int(round(p / 100.0 * (len(xs) - 1)))))
    return xs[k]


def slope(xs, ys):
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0 else None


class Measure(Node):
    def __init__(self):
        super().__init__('m7_measure')
        self.rec = {t: [] for t in IMAGE_TOPICS}  # (wall, stamp, serialized_bytes, data_bytes, proxy, cb_s)
        self.clock = []  # (wall, sim)
        self.latest_clock = None
        for t in IMAGE_TOPICS:
            self.create_subscription(Image, t, self._make_cb(t), IMAGE_QOS, raw=True)
        self.create_subscription(Clock, '/clock', self._on_clock, CLOCK_QOS)

    def _on_clock(self, msg):
        s = stamp_s(msg.clock)
        self.latest_clock = s
        self.clock.append((time.monotonic(), s))

    def _make_cb(self, topic):
        def cb(raw):
            w = time.monotonic()
            m = deserialize_message(raw, Image)
            st = stamp_s(m.header.stamp)
            proxy = (self.latest_clock - st) if self.latest_clock is not None else None
            self.rec[topic].append((w, st, len(raw), len(m.data), proxy, time.monotonic() - w))
        return cb


def summarize_topic(rows, duration_wall):
    out = {'n_received': len(rows)}
    if len(rows) < 3:
        return out
    walls = [r[0] for r in rows]
    stamps = [r[1] for r in rows]
    uniq = sorted(set(stamps))
    out['n_duplicate_stamps'] = len(stamps) - len(uniq)
    out['n_out_of_order'] = sum(1 for a, b in zip(stamps, stamps[1:]) if b < a)
    wall_span = walls[-1] - walls[0]
    sim_span = uniq[-1] - uniq[0]
    out['wall_span_s'] = wall_span
    out['sim_span_s'] = sim_span
    out['rate_wall_hz'] = (len(rows) - 1) / wall_span
    out['rate_sim_hz'] = (len(uniq) - 1) / sim_span
    d = [b - a for a, b in zip(uniq, uniq[1:])]
    med = statistics.median(d)
    gaps = [x for x in d if x > 1.5 * med]
    out['stamp_period_median_s'] = med
    out['stamp_period_min_s'] = min(d)
    out['stamp_period_max_s'] = max(d)
    out['n_stamp_gaps_gt_1p5_median'] = len(gaps)
    out['est_missing_frames'] = int(sum(round(g / med) - 1 for g in gaps))
    out['gap_sizes_s'] = sorted(gaps)[-10:]
    wd = [b - a for a, b in zip(walls, walls[1:])]
    out['wall_interarrival_p50_s'] = pct(wd, 50)
    out['wall_interarrival_p95_s'] = pct(wd, 95)
    out['wall_interarrival_max_s'] = max(wd)
    ser = [r[2] for r in rows]
    dat = [r[3] for r in rows]
    out['serialized_bytes_per_msg'] = {'min': min(ser), 'max': max(ser)}
    out['data_bytes_per_msg'] = {'min': min(dat), 'max': max(dat)}
    out['bandwidth_wall_MBps'] = sum(ser) / wall_span / 1e6
    out['bandwidth_wall_MiBps'] = sum(ser) / wall_span / (1 << 20)
    px = [r[4] for r in rows if r[4] is not None]
    if px:
        out['proxy_clock_minus_stamp_sim_s'] = {'p50': pct(px, 50), 'p95': pct(px, 95),
                                                'min': min(px), 'max': max(px)}
    cbs = [r[5] for r in rows]
    out['callback_s'] = {'p50': pct(cbs, 50), 'max': max(cbs)}
    return out


def summarize_clock(rows):
    out = {'n_received': len(rows)}
    if len(rows) < 3:
        return out
    w = [r[0] for r in rows]
    s = [r[1] for r in rows]
    out['wall_span_s'] = w[-1] - w[0]
    out['sim_span_s'] = s[-1] - s[0]
    out['rate_wall_hz'] = (len(rows) - 1) / (w[-1] - w[0])
    out['rtf_endpoints'] = (s[-1] - s[0]) / (w[-1] - w[0])
    out['rtf_lsq_slope'] = slope([x - w[0] for x in w], [y - s[0] for y in s])
    ds = [b - a for a, b in zip(s, s[1:])]
    out['sim_step_median_s'] = statistics.median(ds)
    out['n_clock_backwards'] = sum(1 for x in ds if x < 0)
    out['n_clock_repeats'] = sum(1 for x in ds if x == 0)
    # RTF in 10 s wall windows
    win, i0, rtfs = 10.0, 0, []
    for i in range(len(w)):
        if w[i] - w[i0] >= win:
            rtfs.append((s[i] - s[i0]) / (w[i] - w[i0]))
            i0 = i
    if rtfs:
        out['rtf_10s_windows'] = {'min': min(rtfs), 'max': max(rtfs), 'n': len(rtfs)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--warmup', type=float, default=5.0, help='wall s to discard while discovery settles')
    ap.add_argument('--out', default=str(Path(__file__).resolve().parent / 'results' / 'measure.json'))
    args = ap.parse_args()

    rclpy.init()
    node = Measure()
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.05)
    for v in node.rec.values():
        v.clear()
    node.clock.clear()
    t1 = time.monotonic()
    while time.monotonic() - t1 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    t2 = time.monotonic()

    res = {'duration_wall_s': t2 - t1,
           'subscriber_qos': {
               'images': 'RELIABLE, VOLATILE, KEEP_LAST depth 10',
               '/clock': 'BEST_EFFORT, VOLATILE, KEEP_LAST depth 10'},
           'topics': {t: summarize_topic(node.rec[t], t2 - t1) for t in IMAGE_TOPICS},
           'clock': summarize_clock(node.clock)}
    tot = sum(v.get('bandwidth_wall_MBps', 0) for v in res['topics'].values())
    res['total_image_bandwidth_wall_MBps'] = tot
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
