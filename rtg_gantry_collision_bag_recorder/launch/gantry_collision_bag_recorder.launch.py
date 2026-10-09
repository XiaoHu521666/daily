from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rtg_gantry_collision_bag_recorder"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config / "config" / "recorder.yaml")),
        *[Node(
            package="rtg_gantry_collision_bag_recorder",
            executable="gantry_collision_bag_recorder_node",
            name=f"gantry_collision_bag_recorder_{side}",
            parameters=[LaunchConfiguration("config")],
            output="screen",
            respawn=True,
            respawn_delay=10.0,
        ) for side in ("left", "right")],
    ])
