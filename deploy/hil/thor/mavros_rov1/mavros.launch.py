"""MAVROS for the HIL vehicle, under the namespace docs/topic_map.md reserves: /<vehicle>/mavros/**.

    ros2 launch ros/mavros_rov1/mavros.launch.py [vehicle:=rov1] [fcu_url:=udp://192.168.200.10:14550@]
                                                 [use_sim_time:=true]

Two things this adds over including mavros' own ``apm.launch`` directly.

**The namespace.** ``apm.launch`` puts its nodes in ``/mavros``, so the topics come out as
``/mavros/imu/data``. ``docs/topic_map.md`` section 4.3 reserves ``/<vehicle>/mavros/**`` - one namespace
per vehicle, because there is one ``/tf`` for the whole graph and a topic namespace is the only thing
separating two ROVs. ``PushRosNamespace`` wraps the include so the inner ``mavros`` lands under the
vehicle prefix.

**Sim time.** ``apm.launch`` has no ``use_sim_time`` argument, and ``SetParameter`` applies it to every
node launched after it in this file. With Isaac running, the simulator owns ``/clock`` and the RTF is
below 1, so a node on the wall clock would timestamp everything wrong; without Isaac (the smoke test in
``docker/ardusub_sitl/README.md``) there is no ``/clock`` at all and sim time would simply stall, which
is why the smoke test passes ``use_sim_time:=false``.

What is consumed from MAVROS, and what is not. The dead-reckoning chain reads ``imu/data``,
``imu/static_pressure`` and ``state``, all read-only. No command topic, no setpoint, no arming: that is
M8b, and with it the three traps of ``docs/ros_hil.md`` section 4.1 (``MAV_GCS_SYSID``, the pilot-input
failsafe, the disarm path). They do not apply to a telemetry-only link.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    apm = PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="rov1",
                              description="ros.vehicle of config/tank.yaml: the topic and frame prefix"),
        DeclareLaunchArgument("fcu_url", default_value="udp://192.168.200.10:14550@",
                              description="SITL pushes to the Thor, so MAVROS only binds its own end"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="false only for the smoke test without Isaac: there is no /clock"),
        SetParameter(name="use_sim_time",
                     value=ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("vehicle")),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(apm),
                launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
            ),
        ]),
    ])
