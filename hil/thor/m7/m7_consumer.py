#!/usr/bin/env python3
"""M7 step 4: minimal consumer on the Thor (sim time).

    python3 hil/thor/m7/m7_consumer.py --ros-args -p use_sim_time:=true -p run_wall_s:=120.0

Inputs (types read with `ros2 topic info -v`; no topic map / docs/ros_hil.md exists on the Thor):
  /oculus/polar_db   sensor_msgs/msg/Image, 32FC1, rows = range bins, cols = beams (see /oculus/meta)
  /camera/image_raw  sensor_msgs/msg/Image, rgb8
  /oculus/meta       std_msgs/msg/String (JSON, TRANSIENT_LOCAL) - range_m / bearing_deg tables

Detection (trivial): pixels of polar_db >= threshold_db (NaN ignored) -> 8-connected components
(cv2.connectedComponentsWithStats) -> components with area >= min_area_px.

Output: /uw/m7/detections  vision_msgs/msg/Detection2DArray (standard ROS package, no custom types)
  header                      = header of the polar_db frame (sim stamp, frame_id 'oculus')
  detections[i].header        = same header
  detections[i].id            = component index in this frame, as a string
  detections[i].bbox.center   = centroid in PIXELS of the polar_db image: position.x = column (beam
                                index), position.y = row (range-bin index), theta = 0
  detections[i].bbox.size_x/y = bounding-box width (beams) / height (range bins) in pixels
  detections[i].results[0].hypothesis.class_id = 'sonar_blob'
  detections[i].results[0].hypothesis.score    = peak dB inside the component
  detections[i].results[0].pose.pose.position  = x: range [m] at the centroid row (range_m),
                                                 y: bearing [deg, + starboard] at the centroid column
                                                 (bearing_deg), z: area [px]; NOT a Cartesian pose.
  One message is published per polar_db frame, also when it has no detections.

The camera image is decoded to an HxWx3 array and only its mean intensity is computed, to prove the
decode path. Statistics go to results/consumer.json next to this script at exit.
"""
import json
import os
import resource
import statistics
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

IN_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST, depth=5)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

DTYPES = {'32FC1': (np.float32, 1), 'mono8': (np.uint8, 1), 'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3)}


def decode(msg):
    dt, ch = DTYPES[msg.encoding]
    a = np.frombuffer(msg.data, dtype=dt)
    if msg.is_bigendian and a.dtype.itemsize > 1:
        a = a.byteswap()
    a = a.reshape(msg.height, msg.step // a.dtype.itemsize)[:, :msg.width * ch]
    return a.reshape(msg.height, msg.width, ch) if ch > 1 else a


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class M7Consumer(Node):
    def __init__(self):
        super().__init__('m7_consumer')
        self.declare_parameter('threshold_db', 140.0)
        self.declare_parameter('min_area_px', 20)
        self.declare_parameter('run_wall_s', 120.0)
        self.thr = float(self.get_parameter('threshold_db').value)
        self.min_area = int(self.get_parameter('min_area_px').value)
        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn('use_sim_time is false: this node is meant to run with use_sim_time:=true')
        self.range_m = None
        self.bearing_deg = None
        self.sonar = {'stamps': [], 'proc_s': [], 'n_det': [], 'lag_sim_s': []}
        self.cam = {'stamps': [], 'proc_s': [], 'mean': []}
        self.n_pub = 0
        self.pub = self.create_publisher(Detection2DArray, '/uw/m7/detections', OUT_QOS)
        self.create_subscription(String, '/oculus/meta', self._on_meta, LATCHED_QOS)
        self.create_subscription(Image, '/oculus/polar_db', self._on_sonar, IN_QOS)
        self.create_subscription(Image, '/camera/image_raw', self._on_cam, IN_QOS)

    def _on_meta(self, msg):
        m = json.loads(msg.data)
        self.range_m = np.asarray(m['range_m'])
        self.bearing_deg = np.asarray(m['bearing_deg'])

    def _on_cam(self, msg):
        t = time.monotonic()
        img = decode(msg)
        self.cam['mean'].append(float(img.mean()))
        self.cam['stamps'].append(stamp_s(msg.header.stamp))
        self.cam['proc_s'].append(time.monotonic() - t)

    def _on_sonar(self, msg):
        t = time.monotonic()
        db = decode(msg)
        mask = (np.nan_to_num(db, nan=-np.inf) >= self.thr).astype(np.uint8)
        n, labels, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = Detection2DArray()
        out.header = msg.header
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_area:
                continue
            x, y, w, h = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                     cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            cx, cy = float(cent[i, 0]), float(cent[i, 1])
            d = Detection2D()
            d.header = msg.header
            d.id = str(len(out.detections))
            d.bbox.center.position.x = cx
            d.bbox.center.position.y = cy
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'sonar_blob'
            hyp.hypothesis.score = float(np.nanmax(db[y:y + h, x:x + w][labels[y:y + h, x:x + w] == i]))
            if self.range_m is not None:
                hyp.pose.pose.position.x = float(np.interp(cy, np.arange(len(self.range_m)), self.range_m))
                hyp.pose.pose.position.y = float(np.interp(cx, np.arange(len(self.bearing_deg)), self.bearing_deg))
            hyp.pose.pose.position.z = float(area)
            d.results.append(hyp)
            out.detections.append(d)
        self.pub.publish(out)
        self.n_pub += 1
        st = stamp_s(msg.header.stamp)
        self.sonar['stamps'].append(st)
        self.sonar['n_det'].append(len(out.detections))
        self.sonar['proc_s'].append(time.monotonic() - t)
        # proxy, NOT latency: node sim clock (/clock as received on the Thor) at publish minus input stamp
        self.sonar['lag_sim_s'].append(self.get_clock().now().nanoseconds * 1e-9 - st)


def stamp_stats(stamps):
    u = sorted(set(stamps))
    if len(u) < 3:
        return {'n': len(stamps)}
    d = [b - a for a, b in zip(u, u[1:])]
    med = statistics.median(d)
    gaps = [x for x in d if x > 1.5 * med]
    return {'n': len(stamps), 'n_unique': len(u), 'sim_span_s': u[-1] - u[0],
            'rate_sim_hz': (len(u) - 1) / (u[-1] - u[0]), 'period_median_s': med,
            'n_gaps_gt_1p5_median': len(gaps), 'est_missing': int(sum(round(g / med) - 1 for g in gaps))}


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(p / 100 * (len(xs) - 1))))] if xs else None


def main():
    rclpy.init()
    node = M7Consumer()
    run_wall = float(node.get_parameter('run_wall_s').value)
    r0 = resource.getrusage(resource.RUSAGE_SELF)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < run_wall:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.monotonic() - t0
    r1 = resource.getrusage(resource.RUSAGE_SELF)
    cpu = (r1.ru_utime - r0.ru_utime) + (r1.ru_stime - r0.ru_stime)
    s, c = node.sonar, node.cam
    res = {
        'params': {'use_sim_time': node.get_parameter('use_sim_time').value, 'threshold_db': node.thr,
                   'min_area_px': node.min_area, 'run_wall_s': run_wall},
        'meta_received': node.range_m is not None,
        'sonar_in': stamp_stats(s['stamps']),
        'detections_published': node.n_pub,
        'detections_per_frame': {'min': min(s['n_det']), 'max': max(s['n_det']),
                                 'mean': statistics.mean(s['n_det'])} if s['n_det'] else None,
        'sonar_proc_wall_s': {'p50': q(s['proc_s'], 50), 'p95': q(s['proc_s'], 95), 'max': max(s['proc_s'])} if s['proc_s'] else None,
        'proxy_simclock_at_publish_minus_stamp_s': {'p50': q(s['lag_sim_s'], 50), 'p95': q(s['lag_sim_s'], 95),
                                                    'max': max(s['lag_sim_s'])} if s['lag_sim_s'] else None,
        'camera_in': stamp_stats(c['stamps']),
        'camera_decode_wall_s': {'p50': q(c['proc_s'], 50), 'p95': q(c['proc_s'], 95), 'max': max(c['proc_s'])} if c['proc_s'] else None,
        'camera_mean_intensity_last': c['mean'][-1] if c['mean'] else None,
        'process_cpu_s': cpu, 'process_cpu_pct_of_one_core': 100.0 * cpu / wall,
    }
    out = str(Path(__file__).resolve().parent / 'results' / 'consumer.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
