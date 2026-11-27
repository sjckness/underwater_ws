"""The DVL dead-reckoning chain: the preprocessor and robot_localization's EKF.

    ros2 launch ros/dvl_dr/launch/dvl_dr.launch.py [vehicle:=rov1] [use_sim_time:=true]

It expects, already running: the simulator (``/sim/dvl/report``, ``/clock``), ``ros/dvl_relay``, and
MAVROS under ``/<vehicle>/mavros`` (``ros/mavros_rov1``). It publishes ``/<vehicle>/nav/odom`` and owns
``odom -> base_link`` on ``/tf``; ``map -> odom`` stays a Jetson-side static identity until a map-frame
localizer exists (docs/topic_map.md section 2).

``ekf.yaml`` names ``rov1`` in its frames and topics, because robot_localization takes them as plain
parameters. Another vehicle needs a copy of that file, passed as ``ekf_config:=``.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    vehicle = LaunchConfiguration("vehicle")
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="rov1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ekf_config", default_value=os.path.join(HERE, "config", "ekf.yaml")),
        SetParameter(name="use_sim_time",
                     value=ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)),
        Node(package="dvl_dr", executable="preprocess", name="dvl_dr_preprocess", output="screen",
             arguments=["--vehicle", vehicle]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[LaunchConfiguration("ekf_config")],
             remappings=[("odometry/filtered", ["/", vehicle, "/nav/odom"])]),
    ])
