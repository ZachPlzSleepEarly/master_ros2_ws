from typing import Final
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE: Final = "master_ros2_pkg"
OUTPUT_SCREEN: Final = "screen"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=PACKAGE,
            executable='action_server',
            name='action_server',
            output=OUTPUT_SCREEN,
        ),

        Node(
            package=PACKAGE,
            executable='action_client',
            name='action_client',
            output=OUTPUT_SCREEN,
        ),
    ])