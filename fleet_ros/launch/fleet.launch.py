import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("fleet_ros")
    default_map = os.path.join(share, "maps", "warehouse_small.txt")
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value=default_map),
        Node(
            package="fleet_ros",
            executable="fleet_manager_node",
            name="fleet_manager",
            parameters=[os.path.join(share, "config", "fleet.yaml"),
                        {"map_path": LaunchConfiguration("map_path")}],
            output="screen",
        ),
    ])
