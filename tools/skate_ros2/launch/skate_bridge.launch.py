"""Launch the skate_driver bridge.

    ros2 launch skate_ros2 skate_bridge.launch.py            # real robot
    ros2 launch skate_ros2 skate_bridge.launch.py robot_host:=127.0.0.1  # sim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_host", default_value="r.local",
                              description="Skate hostname or IP "
                                          "(127.0.0.1 for the sim endpoint)"),
        DeclareLaunchArgument("robot_port", default_value="2000"),
        DeclareLaunchArgument("tx_rate", default_value="60.0"),
        DeclareLaunchArgument("auto_deadman", default_value="true"),
        Node(
            package="skate_ros2",
            executable="driver",
            name="skate_driver",
            output="screen",
            parameters=[{
                "robot_host": LaunchConfiguration("robot_host"),
                "robot_port": LaunchConfiguration("robot_port"),
                "tx_rate": LaunchConfiguration("tx_rate"),
                "auto_deadman": LaunchConfiguration("auto_deadman"),
            }],
        ),
    ])
