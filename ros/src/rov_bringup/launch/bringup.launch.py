"""The ROV's ROS side in one launch: MAVROS (telemetry only) and, with record:=true, the recording profile.

    ros2 launch rov_bringup bringup.launch.py [vehicle:=rov1] [fcu_url:=...] [record:=true] [profile:=light]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(name, args, **kw):
    path = PathJoinSubstitution([FindPackageShare("rov_bringup"), "launch", name])
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments={a: LaunchConfiguration(a) for a in args}.items(), **kw)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="rov1"),
        DeclareLaunchArgument("fcu_url", default_value="tcp://127.0.0.1:5777"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("profile", default_value="light"),
        DeclareLaunchArgument("out", default_value="/root/persistent_ws/bags"),
        _include("mavros.launch.py", ["vehicle", "fcu_url"]),
        _include("record.launch.py", ["profile", "vehicle", "out"], condition=IfCondition(LaunchConfiguration("record"))),
    ])
