#!/usr/bin/env python3
"""M7: value statistics of /oculus/polar_db (32FC1) and test whether /oculus/polar (mono8) equals
clip((db - lo) / (hi - lo) * 255) with display_db=[lo, hi] from /oculus/meta, on stamp-matched frames."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

def img(m, dt):
    a = np.frombuffer(bytes(m.data), dtype=dt).reshape(m.height, m.step // np.dtype(dt).itemsize)
    return a[:, :m.width]

def main():
    rclpy.init(); n = Node('m7_peek_sonar')
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
    ql = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
    db, u8, meta = {}, {}, {}
    n.create_subscription(Image, '/oculus/polar_db', lambda m: db.setdefault((m.header.stamp.sec, m.header.stamp.nanosec), img(m, np.float32)), q)
    n.create_subscription(Image, '/oculus/polar', lambda m: u8.setdefault((m.header.stamp.sec, m.header.stamp.nanosec), img(m, np.uint8)), q)
    n.create_subscription(String, '/oculus/meta', lambda m: meta.setdefault('m', json.loads(m.data)), ql)
    while rclpy.ok() and (len(set(db) & set(u8)) < 5 or 'm' not in meta):
        rclpy.spin_once(n, timeout_sec=0.1)
    lo, hi = meta['m']['display_db']
    for k in sorted(set(db) & set(u8))[:5]:
        d, p = db[k], u8[k]
        fin = d[np.isfinite(d)]
        pred = np.clip((d - lo) / (hi - lo) * 255.0, 0, 255)
        f = np.isfinite(d)
        err = np.abs(pred - p.astype(np.float32))[f]
        bad_rows = np.where(~f.all(axis=1))[0]
        nf = d[~f]
        sel = f & (p > 0) & (p < 255)
        A = np.vstack([d[sel], np.ones(sel.sum())]).T
        (a, b), *_ = np.linalg.lstsq(A, p[sel].astype(np.float64), rcond=None)
        res = np.abs(A @ [a, b] - p[sel])
        print(f'  non-finite rows: {bad_rows.min() if bad_rows.size else None}..{bad_rows.max() if bad_rows.size else None} '
              f'({bad_rows.size} rows), values: nan={np.isnan(nf).sum()} -inf={np.isneginf(nf).sum()} +inf={np.isposinf(nf).sum()}; '
              f'u8 on those pixels: {np.unique(p[~f])[:5]}')
        print(f'  lsq fit u8 = {a:.4f}*db + {b:.2f} on {sel.sum()} unsaturated px: |res| max={res.max():.2f} mean={res.mean():.3f}; '
              f'db at u8=0 -> {-b/a:.2f}, at u8=255 -> {(255-b)/a:.2f}')
        print(f'stamp {k}: db finite={fin.size}/{d.size} min={fin.min():.2f} p50={np.percentile(fin,50):.2f} '
              f'p99={np.percentile(fin,99):.2f} p99.9={np.percentile(fin,99.9):.2f} max={fin.max():.2f} | '
              f'u8 p50={np.percentile(p,50):.0f} max={p.max()} | |pred-u8| max={err.max():.2f} mean={err.mean():.3f}')
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
