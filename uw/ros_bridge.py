"""SensorSink "ros": live ROS 2 topics through the Isaac ROS bridge's internal Jazzy rclpy (Python 3.11, FastDDS).

Works inside the Kit app (`sim`) and in a plain container Python (`sonar-only --sink ros`); the launcher puts the
bridge's libraries on LD_LIBRARY_PATH. Domain: ROS_DOMAIN_ID, which the container sets from UW_ROS_DOMAIN_ID
(default 73), never an f1tenth domain. All stamps are sim time; /clock carries it.

Topics and frames follow docs/topic_map.md; TOPIC_TEMPLATES and frames() below are the only place they are named,
and ros.vehicle picks the namespace and the frame prefix (default rov1). Frames: map = the tank, origin at the
inner bottom corner, ENU; rov1/sonar_fls = the sonar's acoustic centre, x forward, y port, z up;
rov1/sonar_fls_projected = the same point with the axes of marine_acoustic_msgs/ProjectedSonarImage, x up,
y starboard, z forward; rov1/camera_front(_optical) as usual; gt/rov1/sonar_fls = the sonar's *true* pose, ground
truth only.

  /rov1/sensors/sonar_fls/image        sensor_msgs/Image mono8, rows = range bins (row 0 = range 0), columns =
                                       beams (port to starboard, bearing_deg in …/meta); the display image (TVG,
                                       display_db window). ros/sonar_relay turns it into ProjectedSonarImage on
                                       …/sonar_fls/sonar_image, and only that crosses the link
  /rov1/sensors/sonar_fls/meta         std_msgs/String JSON, latched: range_m, bearing_deg, frequency, sound
                                       speed, beam widths and the whole gain chain. No pose, no label
  /rov1/sensors/camera_front/image_raw sensor_msgs/Image rgb8; …/camera_front/camera_info
  /gt/polar_db                         sensor_msgs/Image 32FC1: the calibrated level, dB re 1 uPa, NaN in the
                                       blind zone. Ground truth, and 4x the bytes of the ping: never crosses
  /gt/leak/pose                        geometry_msgs/PoseStamped: the hole, x axis along its outward normal;
                                       /gt/leak/flow_lpm std_msgs/Float32 (gas flow at the hole's pressure)
  /gt/plume/markers                    visualization_msgs/MarkerArray: one SPHERE_LIST of the bubbles (decimated)
  /gt/sonar_pose                       geometry_msgs/PoseStamped: the sonar's TRUE pose in map, per ping; /gt/tf
                                       carries the same thing as map -> gt/rov1/sonar_fls
  /sim/debug/sonar_fan                 sensor_msgs/Image mono8, Cartesian fan: derived from the polar image on
                                       this side, so off unless ros.publish_fan
  /tf_static, /clock                   the payload mount chain and sim time, both global by necessity
Nothing under /gt is latched: a node that never asked for ground truth must not receive it on join (§2.5 rule 3).
**The sim never publishes /tf.** The live sonar pose is the estimator's or ArduSub's to publish, never the
simulator's (docs/ros_hil.md §2.5 rule 4): a detector doing a TF lookup would otherwise read the truth. /tf_static
therefore carries only mount offsets (sonar_fls -> sonar_fls_projected, sonar_fls -> camera_front, camera_front ->
camera_front_optical) and no world-anchored edge at all, so map -> rov1/sonar_fls does not resolve until something
estimates it.
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

VEHICLE = "rov1"        # ros.vehicle: one namespace and one frame prefix per vehicle, even with a single ROV


def frames(vehicle: str = VEHICLE) -> dict:
    """REP-105 frame ids for one vehicle (docs/topic_map.md §2). `map` is the tank, shared by every vehicle and by
    the ground-truth tree; everything attached to a vehicle carries its prefix, which is how multi-ROV separation
    works in TF — a topic namespace cannot do it, since there is one /tf for the whole graph."""
    return {"world": "map", "odom": f"{vehicle}/odom", "body": f"{vehicle}/base_link",
            "sonar": f"{vehicle}/sonar_fls", "sonar_projected": f"{vehicle}/sonar_fls_projected",
            "camera": f"{vehicle}/camera_front", "camera_optical": f"{vehicle}/camera_front_optical",
            "gt_sonar": f"gt/{vehicle}/sonar_fls"}


FRAMES = frames()

#: docs/topic_map.md §4, the single place in the code that names a topic. "{v}" is the vehicle namespace; /gt and
#: /sim are not per vehicle, because there is one simulator and one truth whatever the vehicle count.
TOPIC_TEMPLATES = {
    "polar": "/{v}/sensors/sonar_fls/image",            # the relay's input: host-local, see §6
    "meta": "/{v}/sensors/sonar_fls/meta",
    "image": "/{v}/sensors/camera_front/image_raw",
    "camera_info": "/{v}/sensors/camera_front/camera_info",
    "polar_db": "/gt/polar_db",                         # the calibrated level: never crosses the link
    "fan": "/sim/debug/sonar_fan",                      # derived from polar on this side; off by default
    "leak": "/gt/leak/pose",
    "flow": "/gt/leak/flow_lpm",
    "plume": "/gt/plume/markers",
    "gt_sonar_pose": "/gt/sonar_pose",
    "gt_tf": "/gt/tf",
    "diagnostics": "/sim/diagnostics",                  # RTF and loop health: see RosSink.diagnostics
    "tf_static": "/tf_static",                          # global: one TF tree for the graph
    "clock": "/clock",                                  # global: use_sim_time reads it there
}


def topics(vehicle: str = VEHICLE) -> dict:
    return {k: t.format(v=vehicle) for k, t in TOPIC_TEMPLATES.items()}
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


#: the only keys of meta["sonar"] that reach .../sonar_fls/meta. Deliberately a fixed list and not a `**s` splat: the
#: per-ping meta also holds the sonar's true pose and plume_pixels.geom, which *is* the in_view target, and one
#: careless splat would publish the answer (docs/ros_hil.md §2.5 rule 2, asserted in tests/test_ros_messages.py).
META_KEYS = ("frequency_hz", "sound_speed", "beamwidth_h_deg", "fov_v_deg", "range", "mode", "focus",
             "noise_floor_db", "display_db", "tvg_db_per_decade", "absorption_db_per_m")
#: what must never appear in it: the pose (a real Oculus reports none of it) and anything label-shaped.
META_FORBIDDEN = ("position", "rotation", "roll_deg", "pitch_deg", "yaw_deg", "plume_pixels", "plume_mask_threshold",
                  "leak", "pose_source")


def sonar_meta(msg: Message, vehicle: str = VEHICLE) -> dict:
    """What a consumer needs to place the polar image and undo its gain chain (latched on …/sonar_fls/meta).

    Carries no pose: the sonar's true pose goes to /gt/sonar_pose instead, since a real Oculus publishes neither
    position nor attitude and this is the one metadata topic the detector must subscribe to. Carries the whole
    gain chain, because the 8-bit quantization is only invertible with it: display_db [lo, hi], tvg_db_per_decade
    and absorption_db_per_m are our analogue of the real bag's single dynamic_reconfigure/Config."""
    s, f = msg.meta["sonar"], frames(vehicle)
    return {"frame_id": f["sonar"], "projected_frame_id": f["sonar_projected"],
            "layout": "rows = range bins (row 0 = range_m[0]), columns = beams (bearing_deg, + starboard)",
            "range_m": [round(float(v), 6) for v in msg.arrays["range_m"]],
            "bearing_deg": [round(float(v), 5) for v in msg.arrays["bearing_deg"]],
            **{k: s[k] for k in META_KEYS}}


def pose_stamped_msg(t: float, frame: str, position, rot):
    """geometry_msgs/PoseStamped from a position and a body-to-world rotation matrix."""
    from geometry_msgs.msg import PoseStamped

    m = PoseStamped()
    m.header.stamp = _stamp(t)
    m.header.frame_id = frame
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
    q = quat_xyzw(np.asarray(rot, float))
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
    return m


class RosSink(SensorSink):
    def __init__(self, node_name: str = "sim_bridge", vehicle: str = VEHICLE, fan: bool = False,
                 plume_markers: int = 3000, domain_id: int | None = None):
        ensure_rclpy_path()
        # the bridge's rosidl_generator_py logs a DEBUG traceback (no 'lark') on import: harmless, kept out of the log
        logging.getLogger("rosidl_generator_py").setLevel(logging.WARNING)
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Float32, String
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import PoseStamped
        from visualization_msgs.msg import MarkerArray

        # the domain explicitly, not only from the environment: an unset ROS_DOMAIN_ID used to leave the sim on
        # domain 0, which is an f1tenth domain (docs/bridge_setup.md §1)
        self.domain = int(os.environ.get("ROS_DOMAIN_ID", "73") if domain_id is None else domain_id)
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(domain_id=self.domain,                               # the sim handles Ctrl-C itself
                       signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init(domain_id=self.domain)
        self.rclpy = rclpy
        self.vehicle = vehicle
        self.frames = frames(vehicle)
        self.node = rclpy.create_node(node_name)
        # QoS classes of docs/topic_map.md §3. Images are BEST_EFFORT: Fast DDS's max_blocking_time defaults to
        # 100 ms, which is the whole ping_budget_ms.static, so one blocked write would eat a ping period.
        sensor = _qos({"depth": 5, "reliability": "best_effort"})
        latched = _qos({"depth": 1, "durability": "transient_local"})
        gt = _qos({"depth": 10})            # ground truth is RELIABLE but NEVER latched (§2.5 rule 3)
        t = topics(vehicle)
        pub = self.node.create_publisher
        self.pub = {
            "polar": pub(Image, t["polar"], sensor), "polar_db": pub(Image, t["polar_db"], sensor),
            "fan": pub(Image, t["fan"], sensor), "meta": pub(String, t["meta"], latched),
            "image": pub(Image, t["image"], sensor), "camera_info": pub(CameraInfo, t["camera_info"], sensor),
            "leak": pub(PoseStamped, t["leak"], gt), "flow": pub(Float32, t["flow"], gt),
            "plume": pub(MarkerArray, t["plume"], sensor),
            "gt_sonar_pose": pub(PoseStamped, t["gt_sonar_pose"], gt),
            "gt_tf": pub(TFMessage, t["gt_tf"], _qos({"depth": 100})),
            "diagnostics": pub(DiagnosticArray, t["diagnostics"], _qos({"depth": 10})),
            "tf_static": pub(TFMessage, t["tf_static"], latched),
            "clock": pub(Clock, t["clock"], _qos({"depth": 1, "reliability": "best_effort"})),
        }
        self.topics = t
        self.msgs = {"Clock": Clock, "String": String, "Float32": Float32, "TFMessage": TFMessage,
                     "DiagnosticArray": DiagnosticArray, "DiagnosticStatus": DiagnosticStatus, "KeyValue": KeyValue}
        self.fan = fan
        self.plume_max = plume_markers
        self.meta = None
        self.count = {}
        log.info("ROS 2: node %s on domain %d, vehicle %s", node_name, self.domain, vehicle)

    # ----------------------------------------------------------------------------- SensorSink
    def publish(self, msg: Message) -> None:
        t = msg.stamp
        if msg.sensor == "sonar":
            frame = self.frames["sonar"]
            self.pub["polar"].publish(image_msg(t, frame, msg.images["polar"][::-1], "mono8"))
            self.pub["polar_db"].publish(image_msg(t, frame, msg.arrays["polar_db"].astype(np.float32), "32FC1"))
            if self.fan and "fan" in msg.images:
                self.pub["fan"].publish(image_msg(t, frame, msg.images["fan"], "mono8"))
            meta = sonar_meta(msg, self.vehicle)
            if meta != self.meta:
                self.pub["meta"].publish(self.msgs["String"](data=json.dumps(meta)))
                self.meta = meta
            s = msg.meta["sonar"]                   # the true pose, to /gt only — never into meta, never /tf
            world, gt_sonar = self.frames["world"], self.frames["gt_sonar"]
            self.pub["gt_sonar_pose"].publish(pose_stamped_msg(t, world, s["position"], s["rotation"]))
            self.pub["gt_tf"].publish(self.msgs["TFMessage"](transforms=[
                transform_msg(t, world, gt_sonar, s["rotation"], s["position"])]))
        elif msg.sensor == "camera":
            frame = self.frames["camera_optical"]
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

    def diagnostics(self, t: float, values: dict, *, name: str = "sim loop", level: int = 0,
                    message: str = "") -> None:
        """One DiagnosticStatus of ``values`` on /sim/diagnostics, every value as a KeyValue.

        The RTF goes here rather than only into a log line, because it is what makes every latency measurement in
        this project readable: sim time advances at RTF x wall time, so a wall-clock delay on the Jetson appears
        inside the sim scaled by RTF — at 0.694 a real 60 ms loop looks like 42 ms (docs/ros_hil.md §3.3). A HIL
        monitor may subscribe to this one /sim topic for exactly that reason; no perception or control node may
        (docs/topic_map.md §4.9)."""
        st = self.msgs["DiagnosticStatus"](level=int(level), name=name, message=message, hardware_id="sim")
        st.values = [self.msgs["KeyValue"](key=str(k), value=f"{v:.4g}" if isinstance(v, float) else str(v))
                     for k, v in values.items()]
        m = self.msgs["DiagnosticArray"](status=[st])
        m.header.stamp = _stamp(t)
        self.pub["diagnostics"].publish(m)

    def static_transforms(self, t: float, transforms: list) -> None:
        """[(parent, child, rotation 3x3, translation)] on /tf_static: mount offsets only. All of them must go in
        one call — a depth-1 TRANSIENT_LOCAL writer replays only its last message to a late joiner."""
        self.pub["tf_static"].publish(self.msgs["TFMessage"](
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
    """SensorPose from a ROS topic (sonar.pose_source ros): the latest geometry_msgs/PoseStamped of the sonar in
    the map frame, from /sim/cmd/rov1/pose; until one arrives, ``fallback`` (the configured pose). Polled at each
    ``at``, so the command is quantized to the ping period. Sim-only: a commanded pose has no real counterpart,
    which is why it lives under /sim and not /rov1/control (docs/topic_map.md §4.9)."""
    moving = True

    def __init__(self, fallback, topic: str, domain_id: int | None = None):
        ensure_rclpy_path()
        logging.getLogger("rosidl_generator_py").setLevel(logging.WARNING)
        import rclpy
        from geometry_msgs.msg import PoseStamped
        domain = int(os.environ.get("ROS_DOMAIN_ID", "73") if domain_id is None else domain_id)
        if not rclpy.ok():
            try:
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):
                rclpy.init(domain_id=domain)
        self.rclpy, self.fallback, self.latest = rclpy, fallback, None
        self.node = rclpy.create_node("sim_pose_input")
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

