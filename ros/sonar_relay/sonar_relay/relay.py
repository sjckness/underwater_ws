"""Host-side relay: the sim's /oculus/polar (or /oculus/polar_db) + /oculus/meta -> marine_acoustic_msgs/
ProjectedSonarImage, the message the Oculus ROS drivers publish.

Runs with the host's ROS 2 Jazzy (ros-jazzy-marine-acoustic-msgs), on the tank sim's own domain:

    ros/sonar_relay/run.sh [--db] [--topic /oculus/sonar_image]

or, sourced: python3 ros/sonar_relay/sonar_relay/relay.py, or as a colcon package (ros2 run sonar_relay relay).
The domain is UW_ROS_DOMAIN_ID (default 73), as in the container; a ROS_DOMAIN_ID exported for another stack is
ignored on purpose.

ProjectedSonarImage: frame oculus_projected (x up, y starboard, z forward; the sim publishes it on /tf_static),
beam_directions (0, sin b, cos b) for bearing b (+ starboard), ranges = range bin centres, data row-major with one row
per range bin (beams within a row), uint8 (display image) or float32 (--db: raw level dB re 1 uPa).
"""
from __future__ import annotations

import argparse
import json
import math
import os

import rclpy
from marine_acoustic_msgs.msg import ProjectedSonarImage, SonarImageData
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)


def projected(img: Image, meta: dict, db: bool) -> ProjectedSonarImage:
    """One ping as ProjectedSonarImage (pure: tested without a ROS graph)."""
    n_r, n_b = len(meta["range_m"]), len(meta["bearing_deg"])
    if (img.height, img.width) != (n_r, n_b):
        raise ValueError(f"image {img.height}x{img.width} does not match the meta ({n_r} ranges x {n_b} beams)")
    out = ProjectedSonarImage()
    out.header.stamp = img.header.stamp
    out.header.frame_id = meta["projected_frame_id"]
    out.ping_info.frequency = float(meta["frequency_hz"])
    out.ping_info.sound_speed = float(meta["sound_speed"])
    out.ping_info.tx_beamwidths = [math.radians(float(meta["beamwidth_h_deg"]))] * n_b
    out.ping_info.rx_beamwidths = [math.radians(float(meta["fov_v_deg"]))] * n_b
    out.beam_directions = [Vector3(x=0.0, y=math.sin(math.radians(b)), z=math.cos(math.radians(b)))
                           for b in meta["bearing_deg"]]
    out.ranges = [float(r) for r in meta["range_m"]]
    out.image.is_bigendian = False
    out.image.dtype = SonarImageData.DTYPE_FLOAT32 if db else SonarImageData.DTYPE_UINT8
    out.image.beam_count = n_b
    out.image.data = img.data                    # rows = range bins, beams within a row: the same layout
    return out


class Relay(Node):
    def __init__(self, db: bool, topic: str):
        super().__init__("uw_sonar_relay")
        self.db, self.meta, self.count = db, None, 0
        self.pub = self.create_publisher(ProjectedSonarImage, topic, SENSOR)
        self.create_subscription(String, "/oculus/meta", self.on_meta, LATCHED)
        self.create_subscription(Image, "/oculus/polar_db" if db else "/oculus/polar", self.on_image, SENSOR)
        self.get_logger().info(f"relaying {'/oculus/polar_db' if db else '/oculus/polar'} -> {topic} "
                               f"(ProjectedSonarImage, {'float32 dB' if db else 'uint8'})")

    def on_meta(self, msg: String) -> None:
        self.meta = json.loads(msg.data)

    def on_image(self, img: Image) -> None:
        if self.meta is None:
            self.get_logger().warning("no /oculus/meta yet: ping dropped", throttle_duration_sec=5.0)
            return
        self.pub.publish(projected(img, self.meta, self.db))
        self.count += 1


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", action="store_true", help="relay /oculus/polar_db as float32 (default: the uint8 image)")
    ap.add_argument("--topic", default="/oculus/sonar_image")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args, domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    node = Relay(args.db, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"relayed {node.count} pings")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
