"""Record a rosbag2 profile of rov_bringup (profiles/<profile>.yaml) as MCAP without compression.

    ros2 launch rov_bringup record.launch.py [profile:=light] [vehicle:=rov1] [out:=/root/persistent_ws/bags]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _record(context):
    from rov_bringup.record import command
    get = lambda n: LaunchConfiguration(n).perform(context)          # noqa: E731
    return [ExecuteProcess(cmd=command(get("profile"), get("vehicle"), get("out")), output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="light"),
        DeclareLaunchArgument("vehicle", default_value="rov1"),
        DeclareLaunchArgument("out", default_value="/root/persistent_ws/bags"),
        OpaqueFunction(function=_record),
    ])
