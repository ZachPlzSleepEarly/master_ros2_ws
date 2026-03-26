import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('panda_description')
    pkg_parent = os.path.dirname(pkg_share)   # .../install/<pkg>/share

    # Initialize arguments
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent]
    )
    declared_arguments = [
        gz_resource_path
    ]

    # Get sdf world file path
    sdf_file_path = os.path.join(pkg_share, 'world', 'planning_world.sdf')

    # Parse URDF
    robot_description = {
        'robot_description': xacro.process_file(os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')).toxml()
    }

    use_sim_time_true = {'use_sim_time': True}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            use_sim_time_true,
        ]
    )

    # Spawn robot and controllers
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda',
            '-topic', '/robot_description'
        ],
        output='screen'
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',   # 等待 controller manager 服务可用的最长时间
            '--switch-timeout', '20',               # 等待 Controller 生效的最长时间。
        ],
        output='screen'
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',   # 等待 controller manager 服务可用的最长时间
            '--switch-timeout', '20',               # 等待 Controller 生效的最长时间。
        ],
        output='screen'
    )
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    delay_arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
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
            'gz_args': f'-r -v 4 {sdf_file_path}'
        }.items()
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    load_eef_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'eef_controller'],
        output='screen'
    )

    depth_cam_data2cam_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam3Tolink',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'panda/link0/d435_depth']
    )

    nodes = [
        gz_node,
        robot_state_publisher_node,
        bridge_node,
        depth_cam_data2cam_link_tf,
        spawn_node,
        delay_joint_state_broadcaster_after_spawn,
        delay_arm_controller_after_jsb,
    ]

    ld = LaunchDescription(declared_arguments + nodes)
    return ld
