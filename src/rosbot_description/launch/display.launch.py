from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -------------------------
    # 获取 package 路径
    # -------------------------
    share_dir = get_package_share_directory('rosbot_description')

    # -------------------------   
    # 解析 xacro → URDF
    # -------------------------
    xacro_file = os.path.join(share_dir, 'urdf', 'rosbot.xacro')

    robot_description_config = xacro.process_file(xacro_file)

    robot_urdf = robot_description_config.toxml()
    # 生成 robot_description 参数

    # -------------------------
    # RViz 配置文件
    # -------------------------
    rviz_config_file = os.path.join(
        share_dir,
        'config',
        'display.rviz'
    )

    # -------------------------
    # Launch 参数
    # -------------------------
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    # 是否启用 joint_state_publisher_gui

    show_gui = LaunchConfiguration('gui')

    # -------------------------
    # robot_state_publisher
    # -------------------------
    robot_state_publisher_node = Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    # 作用：
    # 根据 URDF 计算 TF tree

    # -------------------------
    # joint_state_publisher
    # -------------------------
    joint_state_publisher_node = Node(

        condition=UnlessCondition(show_gui),
        # 如果 gui=False

        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # 自动发布关节状态

    # -------------------------
    # joint_state_publisher_gui
    # -------------------------
    joint_state_publisher_gui_node = Node(

        condition=IfCondition(show_gui),
        # 如果 gui=True

        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # GUI滑块控制关节角度

    # -------------------------
    # RViz
    # -------------------------
    rviz_node = Node(

        package='rviz2',
        executable='rviz2',
        name='rviz2',

        arguments=['-d', rviz_config_file],
        # 加载RViz配置

        output='screen'
    )

    return LaunchDescription([

        gui_arg,

        robot_state_publisher_node,

        joint_state_publisher_node,

        joint_state_publisher_gui_node,

        rviz_node

    ])
