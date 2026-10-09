import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("fleet_ros")
    default_config = os.path.join(share, "benchmarks", "configs", "default.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="fleet_core run config (YAML)",
        ),
        Node(
            package="fleet_ros",
            executable="fleet_manager_node",
            name="fleet_manager",
            parameters=[
                os.path.join(share, "config", "fleet.yaml"),
                {"config_file": LaunchConfiguration("config_file")},
            ],
            output="screen",
        ),
    ])
