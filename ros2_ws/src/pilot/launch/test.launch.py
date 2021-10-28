from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from launch_ros.remap_rule_type import RemapRule


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    params_file_path = os.path.join(
        get_package_share_directory('pilot'),
        "params",
        "pilot_config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package='pilot',
                executable='pilot_node',
                output='screen',
                parameters=[
                    params_file_path
                ],
            ),

            Node(
                package='joy',
                executable='joy_node',
                output='screen',
                parameters=[params_file_path],
            ),

            Node(
                package='vesc_driver',
                executable='vesc_driver_node',
                output='screen',
                parameters=[params_file_path]
            ),
            
        ])