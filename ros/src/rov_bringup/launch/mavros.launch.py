"""MAVROS to ArduSub on the real ROV, from inside the BlueOS ROS 2 extension, telemetry only.

    ros2 launch rov_bringup mavros.launch.py [vehicle:=rov1] [fcu_url:=tcp://127.0.0.1:5777]

BlueOS's mavlink-router serves its "Internal Link" as a TCP server on 127.0.0.1:5777 and the extension runs in the
Pi's host network, hence the default fcu_url. The plugin set is config/mavros_pluginlists.yaml: no plugin that can
arm, change mode or write a parameter is loaded. The rest of the configuration is MAVROS's own apm_config.yaml.
Topics: /<vehicle>/mavros/... as docs/topic_map.md §4.3 names them for the HIL vehicle.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle = LaunchConfiguration("vehicle")
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="rov1", description="namespace: /<vehicle>/mavros"),
        DeclareLaunchArgument("fcu_url", default_value="tcp://127.0.0.1:5777",
                              description="BlueOS internal link; udp://:14560@ for the pool SITL on linus"),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        Node(package="mavros", executable="mavros_node", namespace=[vehicle, "/mavros"], output="screen",
             parameters=[
                 {"fcu_url": LaunchConfiguration("fcu_url"), "gcs_url": "",
                  "tgt_system": LaunchConfiguration("tgt_system"), "tgt_component": 1, "fcu_protocol": "v2.0"},
                 PathJoinSubstitution([FindPackageShare("rov_bringup"), "config", "mavros_pluginlists.yaml"]),
                 PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm_config.yaml"]),
             ]),
    ])
