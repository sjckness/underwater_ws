"""What has to happen to the DVL and the autopilot's sensors before robot_localization can fuse them.

Two jobs, both of which `robot_localization` cannot do itself:

**The lever arm.** A DVL measures the velocity *of the DVL*, which is the body's plus ``omega x r``.
With the instrument 12 cm under the hull, a 0.4 rad/s pitch rate is already 5 cm/s of phantom surge -
a bias the filter would happily integrate into the track. `robot_localization` has no mount model, so
the correction is done here, on the way in. It needs the angular rate, which is why this node also
subscribes the IMU rather than passing the twist straight through.

**Depth from pressure.** MAVROS publishes ``imu/static_pressure`` as a ``FluidPressure``; the filter
wants a pose. The conversion is hydrostatic and needs the water's density and the surface pressure,
both installation constants rather than anything the vehicle knows about itself.

**The bottom-lock gate.** A report without lock is not a noisy measurement, it is no measurement: the
beams did not come back. Those are dropped here, so the filter never sees them and propagates on the
IMU alone instead. The relay already marks them with a 1e6 covariance; this is the second line.

No ground truth enters this chain. The DVL comes from the sensor model with its noise, dropout and
quantisation; the attitude and the pressure come from ArduSub's own EKF3 over SITL's synthetic noisy
IMU, baro and compass (``deploy/docker/ardusub_sitl/params/hil.parm``). ``/gt`` is for scoring the result
afterwards, on its own ROS domain, and nothing here subscribes to it.

    ros/src/dvl_dr/run.sh [--vehicle rov1]
"""
from __future__ import annotations

import argparse
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure, Imu

try:                                     # as a colcon package, and as a plain script from run.sh
    from dvl_dr.correct import correct_lever_arm, depth_pose, has_fix
except ImportError:                      # pragma: no cover - run.sh execs this file directly
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from dvl_dr.correct import correct_lever_arm, depth_pose, has_fix

#: class S of docs/topic_map.md §3: MAVROS publishes its sensors BEST_EFFORT, and a RELIABLE reader
#: against a BEST_EFFORT writer never matches - the symptom is total silence on that one topic.
SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)
RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class Preprocess(Node):
    def __init__(self, vehicle: str = "rov1"):
        super().__init__("dvl_dr_preprocess")
        self.declare_parameter("lever_arm_flu", [0.0, 0.0, -0.12])
        self.declare_parameter("water_level", 2.9)
        self.declare_parameter("water_density", 1026.0)
        self.declare_parameter("surface_pressure", 101325.0)
        self.declare_parameter("depth_stddev", 0.02)
        self.declare_parameter("odom_frame", f"{vehicle}/odom")
        self.declare_parameter("base_frame", f"{vehicle}/base_link")
        p = self.get_parameter
        self.r = tuple(float(v) for v in p("lever_arm_flu").value)
        self.base = str(p("base_frame").value)
        self.odom = str(p("odom_frame").value)
        self.omega = (0.0, 0.0, 0.0)
        self.seen = self.used = self.dropped = 0

        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped, f"/{vehicle}/sensors/dvl/velocity_corrected", RELIABLE)
        self.pub_depth = self.create_publisher(PoseWithCovarianceStamped, f"/{vehicle}/nav/depth", RELIABLE)
        self.create_subscription(TwistWithCovarianceStamped, f"/{vehicle}/sensors/dvl/velocity_twist",
                                 self.on_twist, RELIABLE)
        self.create_subscription(Imu, f"/{vehicle}/mavros/imu/data", self.on_imu, SENSOR)
        self.create_subscription(FluidPressure, f"/{vehicle}/mavros/imu/static_pressure",
                                 self.on_pressure, SENSOR)
        self.get_logger().info(
            f"lever arm {self.r} m (FLU), depth from /{vehicle}/mavros/imu/static_pressure, "
            f"corrected twist in {self.base}")

    def on_imu(self, msg: Imu) -> None:
        a = msg.angular_velocity
        self.omega = (a.x, a.y, a.z)

    def on_twist(self, msg: TwistWithCovarianceStamped) -> None:
        self.seen += 1
        if not has_fix(msg.twist.covariance):
            self.dropped += 1
            self.get_logger().warning("DVL without bottom lock: dropped, the filter coasts on the IMU",
                                      throttle_duration_sec=5.0)
            return
        v = msg.twist.twist.linear
        vx, vy, vz = correct_lever_arm((v.x, v.y, v.z), self.omega, self.r)
        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base          # no longer the instrument's: the lever arm is removed
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = vx, vy, vz
        out.twist.covariance = msg.twist.covariance
        self.pub_twist.publish(out)
        self.used += 1

    def on_pressure(self, msg: FluidPressure) -> None:
        z, depth = depth_pose(msg.fluid_pressure, float(self.get_parameter("surface_pressure").value),
                              float(self.get_parameter("water_density").value),
                              float(self.get_parameter("water_level").value))
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom
        out.pose.pose.position.z = z
        out.pose.pose.orientation.w = 1.0
        sd = float(self.get_parameter("depth_stddev").value)
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 6 + i] = -1.0                # only z is measured; the rest must not be fused
        cov[14] = sd * sd
        out.pose.covariance = cov
        self.pub_depth.publish(out)
        self.depth = depth              # for the log line and ros2 param introspection


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--vehicle", default="rov1", help="ros.vehicle of sim/config/tank.yaml")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args, domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    node = Preprocess(args.vehicle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"{node.used}/{node.seen} DVL reports fused, {node.dropped} without bottom lock")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
