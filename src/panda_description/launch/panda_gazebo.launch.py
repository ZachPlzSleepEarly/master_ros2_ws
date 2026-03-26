import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # Get shared package paths
    pkg_panda = get_package_share_directory('panda_description')

    # Declare arguments
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_panda)   # .../install/<pkg>/share
    )
    # Initialize argumetns
    declared_argumetns = [
        gz_resource_path,
    ]

    # Parse robot description file
    robot_description_xml = xacro.process_file(os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')).toxml()

    bridge_yaml = os.path.join(pkg_panda, 'config', 'ros_gz_bridge.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_xml,
        }]
    )

    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # 统一桥接：/clock + 相机，都放 YAML 里
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_yaml,
            'use_sim_time': True,
        }],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-topic', '/robot_description'],
        output='screen'
    )
    # 用 spawner，别再用 ExecuteProcess + ros2 control
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',    # controller_names
            '-c', '/controller_manager',  # [-c CONTROLLER_MANAGER]
            '--controller-manager-timeout', '60',   # 等待 controller manager 服务可用的最长时间
            '--switch-timeout', '20',               # 等待 Controller 生效的最长时间。
        ],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',             # controller_names
            '-c', '/controller_manager',  # [-c CONTROLLER_MANAGER]
            '--controller-manager-timeout', '60',   # 等待 controller manager 服务可用的最长时间
            '--switch-timeout', '20',               # 等待 Controller 生效的最长时间。
        ],
        output='screen',
    )
    eef_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'eef_controller',             # controller_names
            '-c', '/controller_manager',  # [-c CONTROLLER_MANAGER]
            '--controller-manager-timeout', '60',   # 等待 controller manager 服务可用的最长时间
            '--switch-timeout', '20',               # 等待 Controller 生效的最长时间。
        ],
        output='screen',
    )
    delay_joint_state_broadcaster_spawner_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )
    delay_eef_controller_spawner_after_arm_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[eef_controller_spawner]
        )
    )

    depth_cam_data2cam_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam3Tolink',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'camera_link', 'panda/link0/d435_depth'
        ]
    )

    nodes = [
        gz_node,
        robot_state_publisher_node,
        bridge_node,
        depth_cam_data2cam_link_tf,
        spawn_node,
        delay_joint_state_broadcaster_spawner_after_spawn,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_eef_controller_spawner_after_arm_controller_spawner,
    ]

    # 顺序：先环境变量，再 Gazebo，再 bridge，再 spawn，再 controllers
    ld = LaunchDescription(declared_argumetns + nodes)
    return ld
