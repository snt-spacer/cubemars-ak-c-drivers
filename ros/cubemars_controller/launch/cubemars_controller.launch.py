from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("cubemars_controller"), "config", "config.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="cubemars_controller",
                executable="cubemars_controller_node",
                name="cubemars_controller_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="cubemars_controller",
                executable="cubemars_feedback_listener_node",
                name="cubemars_feedback_listener_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
