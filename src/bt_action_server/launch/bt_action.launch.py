from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from typing import Final

PACKAGE_NAME: Final = 'bt_action_server'
TREE_XML_FILE: Final = 'tree_xml_file'


def generate_launch_description():
    tree_path = PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'trees', 'bt_action.xml'])

    ld = LaunchDescription()

    bt_node = Node(
        package=PACKAGE_NAME,
        executable='bt_action',
        name='bt_action',
        output='screen',
        parameters=[{TREE_XML_FILE: tree_path}]
    )

    ld.add_action(bt_node)

    return ld
