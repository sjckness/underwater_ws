#!/usr/bin/env python3
"""M7 step 3: verify that a node started with use_sim_time:=true takes its clock, timers and stamps
from /clock.

    python3 hil/thor/m7/simtime_check.py --ros-args -p use_sim_time:=true -p run_wall_s:=30.0

Checks (results to results/simtime_check.json next to this script):
  * node clock type is ROS_TIME and ros_time_is_active is true;
  * get_clock().now() equals (to within one /clock step) the latest /clock value received, and is
    nowhere near the Thor's system time (sim time is ~1e3 s, system time ~1.7e9 s);
  * a 1.0 s timer fires at sim-time deltas of 1.0 s (quantized to the /clock step), while its
    wall-time deltas follow the real-time factor;
  * a message stamped with get_clock().now() carries a sim-time stamp.
"""
import json
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header


class SimTimeCheck(Node):
    def __init__(self):
        super().__init__('m7_simtime_check')
        self.declare_parameter('run_wall_s', 30.0)
        self.latest_clock = None
        self.fires = []  # (wall_monotonic, now_sim, latest_clock_sim, system_time)
        self.stamped = []
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Clock, '/clock', self._on_clock, q)
        self.timer = self.create_timer(1.0, self._on_timer)

    def _on_clock(self, msg):
        self.latest_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.fires.append((time.monotonic(), now, self.latest_clock, time.time()))
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        self.stamped.append(h.stamp.sec + h.stamp.nanosec * 1e-9)


def main():
    rclpy.init()
    node = SimTimeCheck()
    run_wall = node.get_parameter('run_wall_s').value
    t0 = time.monotonic()
    while time.monotonic() - t0 < run_wall:
        rclpy.spin_once(node, timeout_sec=0.05)
    clk = node.get_clock()
    f = node.fires[1:]  # first fire can be right after the clock jumps from 0 to sim time
    res = {
        'use_sim_time_param': node.get_parameter('use_sim_time').value,
        'clock_type': str(clk.clock_type),
        'clock_type_is_ROS_TIME': clk.clock_type == ClockType.ROS_TIME,
        'ros_time_is_active': clk.ros_time_is_active,
        'n_timer_fires': len(node.fires),
        'run_wall_s': run_wall,
    }
    if len(f) >= 3:
        ds = [b[1] - a[1] for a, b in zip(f, f[1:])]
        dw = [b[0] - a[0] for a, b in zip(f, f[1:])]
        res['timer_sim_delta_s'] = {'median': statistics.median(ds), 'min': min(ds), 'max': max(ds)}
        res['timer_wall_delta_s'] = {'median': statistics.median(dw), 'min': min(dw), 'max': max(dw)}
        res['now_minus_latest_clock_s'] = {'min': min(x[1] - x[2] for x in f), 'max': max(x[1] - x[2] for x in f)}
        res['example_fire'] = {'now_sim': f[-1][1], 'latest_clock': f[-1][2], 'system_time': f[-1][3]}
        res['sim_elapsed_over_fires_s'] = f[-1][1] - f[0][1]
        res['wall_elapsed_over_fires_s'] = f[-1][0] - f[0][0]
        res['stamped_header_example_s'] = node.stamped[-1]
    out = str(Path(__file__).resolve().parent / 'results' / 'simtime_check.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as fh:
        json.dump(res, fh, indent=2)
    print(json.dumps(res, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
