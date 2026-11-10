# MAVROS (ArduPilot flavour) with use_sim_time set for the node. Used by thor_mavros_run.sh.
# Needed because mavros' own apm.launch has no use_sim_time argument; SetParameter applies it to every node launched
# after it in this file.
#     ros2 launch ~/thor_mavros.launch.py fcu_url:=udp://192.168.200.10:14550@ use_sim_time:=true
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    apm = PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="udp://192.168.200.10:14550@"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetParameter(name="use_sim_time", value=ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(apm),
            launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        ),
    ])
