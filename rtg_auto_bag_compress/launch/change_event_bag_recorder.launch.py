from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("rtg_auto_bag_compress"))
        / "config"
        / "recorder.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Change-event bag recorder parameter file",
            ),
            Node(
                package="rtg_auto_bag_compress",
                executable="change_event_bag_recorder_node",
                name="change_event_bag_recorder",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
