#!/usr/bin/env python3
"""M7: read one message per image topic and print header/geometry fields (no data dump)."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

TOPICS = sys.argv[1:] or ['/oculus/polar', '/oculus/polar_db', '/oculus/fan', '/camera/image_raw']


def main():
    rclpy.init()
    node = Node('m7_peek_headers')
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
    seen = {}

    def make_cb(t):
        def cb(m):
            if t in seen:
                return
            if isinstance(m, Image):
                seen[t] = (f'{t}: stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} frame={m.header.frame_id!r} '
                           f'{m.width}x{m.height} enc={m.encoding} step={m.step} bigendian={m.is_bigendian} len(data)={len(m.data)}')
            else:
                seen[t] = (f'{t}: stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} frame={m.header.frame_id!r} '
                           f'{m.width}x{m.height} model={m.distortion_model} K={list(m.k)}')
        return cb

    for t in TOPICS:
        node.create_subscription(CameraInfo if t.endswith('camera_info') else Image, t, make_cb(t), qos)
    end = node.get_clock().now().nanoseconds + 15e9
    while rclpy.ok() and len(seen) < len(TOPICS) and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    for t in TOPICS:
        print(seen.get(t, f'{t}: NO MESSAGE in 15 s'))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
