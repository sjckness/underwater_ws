"""SensorSink "ros": live ROS 2 topics through the Isaac ROS bridge's internal Jazzy rclpy (Python 3.11, FastDDS).

Works inside the Kit app (`sim`) and in a plain container Python (`sonar-only --sink ros`); the launcher puts the
bridge's libraries on LD_LIBRARY_PATH. Domain: ROS_DOMAIN_ID, which the container sets from UW_ROS_DOMAIN_ID
(default 73), never an f1tenth domain. All stamps are sim time; /clock carries it.

Topics (frames: tank = world, origin at the inner bottom corner; oculus = sonar acoustic centre, x forward, y port,
z up; oculus_projected = the same point with the axes of marine_acoustic_msgs/ProjectedSonarImage, x up, y starboard,
z forward; camera and camera_optical as usual):
  /oculus/polar        sensor_msgs/Image mono8, rows = range bins (row 0 = range 0), columns = beams (port to
                       starboard, bearing_deg in /oculus/meta); the display image (TVG, display_db window)
  /oculus/polar_db     sensor_msgs/Image 32FC1, same layout: raw level dB re 1 uPa, NaN in the blind zone
  /oculus/fan          sensor_msgs/Image mono8, Cartesian fan (apex at the bottom centre)
  /oculus/meta         std_msgs/String JSON, latched: range_m, bearing_deg, frequency, sound speed, beam widths, ...
                       (what ros/sonar_relay needs for ProjectedSonarImage)
  /camera/image_raw    sensor_msgs/Image rgb8; /camera/camera_info
  /sim/leak            geometry_msgs/PoseStamped: the hole, x axis along its outward normal; /sim/leak/flow_lpm
                       std_msgs/Float32 (gas flow at the hole's pressure); both latched
  /sim/plume           visualization_msgs/MarkerArray: one SPHERE_LIST of the bubbles (decimated)
  /tf_static, /clock; /tf (tank -> oculus) per ping when the sonar moves
Input (sonar.pose_source ros): ``RosPose`` takes the sonar's pose from a geometry_msgs/PoseStamped topic.
"""
from __future__ import annotations

import array
import json
import logging
import math
import os
import sys

import numpy as np

from .sinks import Message, SensorSink

log = logging.getLogger(__name__)

FRAMES = {"world": "tank", "sonar": "oculus", "sonar_projected": "oculus_projected", "camera": "camera",
          "camera_optical": "camera_optical"}
# oculus_projected axes (x up, y starboard, z forward) in the oculus frame (x forward, y port, z up): columns
R_SONAR_PROJECTED = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])


def ensure_rclpy_path() -> None:
    """The bridge's internal rclpy of ROS_DISTRO (Python 3.11), never a host ROS."""
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    path = os.path.join(os.environ.get("UW_ISAAC_ROOT") or "/isaac-sim", "exts", "isaacsim.ros2.bridge", distro,
                        "rclpy")
    sys.path[:] = [p for p in sys.path if not p.startswith("/opt/ros")]
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)


def _qos(spec: dict):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=int(spec.get("depth", 10)),
        reliability=ReliabilityPolicy.BEST_EFFORT if spec.get("reliability") == "best_effort"
        else ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if spec.get("durability") == "transient_local"
        else DurabilityPolicy.VOLATILE)


def _stamp(t: float):
    from builtin_interfaces.msg import Time

    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _u8(buf: np.ndarray) -> array.array:
    a = array.array("B")
    a.frombytes(np.ascontiguousarray(buf).tobytes())
    return a


def quat_xyzw(rot: np.ndarray) -> np.ndarray:
    """Unit quaternion (x, y, z, w) of a rotation matrix."""
    m = np.asarray(rot, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:                          # Shepperd: divide by the largest component
        s = 2 * math.sqrt(1 + tr)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = 2 * math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] >= m[2, 2]:
        s = 2 * math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2 * math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def image_msg(t: float, frame: str, data: np.ndarray, encoding: str):
    from sensor_msgs.msg import Image

    data = np.ascontiguousarray(data)
    m = Image()
    m.header.stamp = _stamp(t)
    m.header.frame_id = frame
    m.height, m.width = int(data.shape[0]), int(data.shape[1])
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = int(data.strides[0])
    m.data = _u8(data)
    return m


def camera_info_msg(t: float, frame: str, K: np.ndarray, width: int, height: int):
    from sensor_msgs.msg import CameraInfo

    m = CameraInfo()
    m.header.stamp = _stamp(t)
    m.header.frame_id = frame
    m.width, m.height = int(width), int(height)
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = [float(v) for v in np.asarray(K).reshape(-1)]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, 0.0, float(K[1, 1]), float(K[1, 2]), 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


def transform_msg(t: float, parent: str, child: str, rot: np.ndarray, pos):
    from geometry_msgs.msg import TransformStamped

    ts = TransformStamped()
    ts.header.stamp = _stamp(t)
    ts.header.frame_id, ts.child_frame_id = parent, child
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = (float(v) for v in pos)
    q = quat_xyzw(rot)
    ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = map(float, q)
    return ts


def leak_pose_msg(t: float, position, direction):
    """The hole as a pose: x axis along its outward normal, y horizontal."""
    from geometry_msgs.msg import PoseStamped

    x = np.asarray(direction, float) / np.linalg.norm(direction)
    y = np.cross([0.0, 0.0, 1.0], x)
    y = y / np.linalg.norm(y) if np.linalg.norm(y) > 1e-6 else np.array([0.0, 1.0, 0.0])
    q = quat_xyzw(np.stack([x, y, np.cross(x, y)], 1))
    m = PoseStamped()
    m.header.stamp = _stamp(t)
    m.header.frame_id = FRAMES["world"]
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
    return m


def plume_markers_msg(t: float, pos: np.ndarray, diameter: np.ndarray, max_points: int):
    """One SPHERE_LIST of the bubbles, evenly decimated to at most ``max_points``; sphere size = mean diameter,
    colour by size (small: light blue, large: white)."""
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    step = max(1, int(math.ceil(len(pos) / max(max_points, 1))))
    p, d = pos[::step], diameter[::step]
    m = Marker()
    m.header.stamp = _stamp(t)
    m.header.frame_id = FRAMES["world"]
    m.ns, m.id, m.type, m.action = "bubbles", 0, Marker.SPHERE_LIST, Marker.ADD
    m.pose.orientation.w = 1.0
    size = float(np.mean(d)) if len(d) else 0.003
    m.scale.x = m.scale.y = m.scale.z = size
    m.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in p]
    s = np.clip((d - 0.001) / 0.006, 0.0, 1.0) if len(d) else d
    m.colors = [ColorRGBA(r=float(0.55 + 0.45 * v), g=float(0.8 + 0.2 * v), b=1.0, a=0.8) for v in s]
    m.frame_locked = True
    return MarkerArray(markers=[m])


def sonar_meta(msg: Message) -> dict:
    """What a consumer needs to place the polar image (latched on /oculus/meta)."""
    s = msg.meta["sonar"]
    return {"frame_id": FRAMES["sonar"], "projected_frame_id": FRAMES["sonar_projected"],
            "layout": "rows = range bins (row 0 = range_m[0]), columns = beams (bearing_deg, + starboard)",
            "range_m": [round(float(v), 6) for v in msg.arrays["range_m"]],
            "bearing_deg": [round(float(v), 5) for v in msg.arrays["bearing_deg"]],
            **{k: s[k] for k in ("frequency_hz", "sound_speed", "beamwidth_h_deg", "fov_v_deg", "range", "mode", "focus",
                                 "noise_floor_db", "display_db", "position", "roll_deg", "pitch_deg", "yaw_deg")}}


class RosSink(SensorSink):
    def __init__(self, node_name: str = "uw_sim", fan: bool = True, plume_markers: int = 3000):
        ensure_rclpy_path()
        # the bridge's rosidl_generator_py logs a DEBUG traceback (no 'lark') on import: harmless, kept out of the log
        logging.getLogger("rosidl_generator_py").setLevel(logging.WARNING)
        import rclpy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Float32, String
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import PoseStamped
        from visualization_msgs.msg import MarkerArray

        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)      # the sim handles Ctrl-C itself
        except (ImportError, TypeError):
            rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node(node_name)
        self.domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        sensor = _qos({"depth": 2})
        latched = _qos({"depth": 1, "durability": "transient_local"})
        pub = self.node.create_publisher
        self.pub = {
            "polar": pub(Image, "/oculus/polar", sensor), "polar_db": pub(Image, "/oculus/polar_db", sensor),
            "fan": pub(Image, "/oculus/fan", sensor), "meta": pub(String, "/oculus/meta", latched),
            "image": pub(Image, "/camera/image_raw", sensor), "camera_info": pub(CameraInfo, "/camera/camera_info", sensor),
            "leak": pub(PoseStamped, "/sim/leak", latched), "flow": pub(Float32, "/sim/leak/flow_lpm", latched),
            "plume": pub(MarkerArray, "/sim/plume", _qos({"depth": 1})),
            "tf_static": pub(TFMessage, "/tf_static", _qos({"depth": 1, "durability": "transient_local"})),
            "tf": pub(TFMessage, "/tf", _qos({"depth": 10})),
            "clock": pub(Clock, "/clock", _qos({"depth": 1, "reliability": "best_effort"})),
        }
        self.msgs = {"Clock": Clock, "String": String, "Float32": Float32, "TFMessage": TFMessage}
        self.fan = fan
        self.plume_max = plume_markers
        self.meta = None
        self.count = {}
        log.info("ROS 2: node %s on domain %d", node_name, self.domain)

    # ----------------------------------------------------------------------------- SensorSink
    def publish(self, msg: Message) -> None:
        t = msg.stamp
        if msg.sensor == "sonar":
            frame = FRAMES["sonar"]
            self.pub["polar"].publish(image_msg(t, frame, msg.images["polar"][::-1], "mono8"))
            self.pub["polar_db"].publish(image_msg(t, frame, msg.arrays["polar_db"].astype(np.float32), "32FC1"))
            if self.fan and "fan" in msg.images:
                self.pub["fan"].publish(image_msg(t, frame, msg.images["fan"], "mono8"))
            meta = sonar_meta(msg)
            if meta != self.meta:
                self.pub["meta"].publish(self.msgs["String"](data=json.dumps(meta)))
                self.meta = meta
        elif msg.sensor == "camera":
            frame = FRAMES["camera_optical"]
            rgb = msg.images["rgb"]
            self.pub["image"].publish(image_msg(t, frame, rgb, "rgb8"))
            self.pub["camera_info"].publish(camera_info_msg(t, frame, np.asarray(msg.meta["K"]), rgb.shape[1],
                                                            rgb.shape[0]))
        elif msg.sensor == "leak":
            self.pub["leak"].publish(leak_pose_msg(t, msg.meta["position"], msg.meta["direction"]))
            self.pub["flow"].publish(self.msgs["Float32"](data=float(msg.meta["flow_lpm"])))
        elif msg.sensor == "plume":
            self.pub["plume"].publish(plume_markers_msg(t, msg.arrays["position"], msg.arrays["diameter"],
                                                        self.plume_max))
        else:
            raise ValueError(f"RosSink: no topic for sensor {msg.sensor!r}")
        self.count[msg.sensor] = self.count.get(msg.sensor, 0) + 1

    def clock(self, t: float) -> None:
        self.pub["clock"].publish(self.msgs["Clock"](clock=_stamp(t)))

    def static_transforms(self, t: float, transforms: list) -> None:
        """[(parent, child, rotation 3x3, translation)] on /tf_static."""
        self.pub["tf_static"].publish(self.msgs["TFMessage"](
            transforms=[transform_msg(t, p, c, r, x) for p, c, r, x in transforms]))

    def transforms(self, t: float, transforms: list) -> None:
        """Moving frames (a mobile sonar), same form, on /tf."""
        self.pub["tf"].publish(self.msgs["TFMessage"](
            transforms=[transform_msg(t, p, c, r, x) for p, c, r, x in transforms]))

    def close(self) -> None:
        try:
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception as exc:  # noqa: BLE001 - shutting down anyway
            log.warning("ROS shutdown: %s", exc)


def pose_from_msg(msg):
    """geometry_msgs/PoseStamped -> (position (3,), body-to-world rotation 3x3)."""
    q = msg.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    p = msg.pose.position
    return np.array([p.x, p.y, p.z]), rot


class RosPose:
    """SensorPose from a ROS topic (sonar.pose_source ros): the latest geometry_msgs/PoseStamped of the sonar in the
    tank frame; until one arrives, ``fallback`` (the configured pose). Polled at each ``at``."""
    moving = True

    def __init__(self, fallback, topic: str):
        ensure_rclpy_path()
        logging.getLogger("rosidl_generator_py").setLevel(logging.WARNING)
        import rclpy
        from geometry_msgs.msg import PoseStamped
        if not rclpy.ok():
            try:
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):
                rclpy.init()
        self.rclpy, self.fallback, self.latest = rclpy, fallback, None
        self.node = rclpy.create_node("uw_sonar_pose")
        self.node.create_subscription(PoseStamped, topic, self._on_pose, _qos({"depth": 1}))
        self.count = 0
        log.info("sonar pose from %s (until then the configured pose)", topic)

    def _on_pose(self, msg) -> None:
        self.latest = pose_from_msg(msg)
        self.count += 1

    def at(self, t: float):
        from .pose import Pose
        from .geometry import rpy_deg_of
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.latest is None:
            return self.fallback.at(t)
        return Pose(self.latest[0], rpy_deg_of(self.latest[1]))

    def positions(self, step: float = 0.05):
        return self.fallback.positions(step)

    def close(self) -> None:
        self.node.destroy_node()

