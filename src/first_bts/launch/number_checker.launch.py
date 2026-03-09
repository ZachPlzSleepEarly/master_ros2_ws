from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from typing import Final

PACKAGE_NAME: Final = 'first_bts'
PARAM_TREE_XML_FILE: Final = 'tree_xml_file'


def generate_launch_description():
    tree_path = PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'trees', 'number_checker.xml'])

    ld = LaunchDescription()

    bt_node = Node(
        package=PACKAGE_NAME,
        executable='number_checker',
        name='number_checker',
        output='screen',
        parameters=[{PARAM_TREE_XML_FILE: tree_path}]
    )

    ld.add_action(bt_node)

    return ld
