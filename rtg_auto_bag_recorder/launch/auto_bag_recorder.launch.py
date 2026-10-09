from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                description="Required path to recorder_650.yaml ... recorder_655.yaml",
            ),
            Node(
                package="rtg_auto_bag_recorder",
                executable="auto_bag_recorder_node",
                name="auto_bag_recorder",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
