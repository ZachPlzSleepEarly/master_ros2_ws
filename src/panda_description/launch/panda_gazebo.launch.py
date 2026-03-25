import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_panda = get_package_share_directory('panda_description')

    xacro_file = os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')
    bridge_yaml = os.path.join(pkg_panda, 'config', 'ros_gz_bridge.yaml')

    # 让 model://panda_description/... 能被 Gazebo 找到
    # 不要再用 COLCON_PREFIX_PATH / AMENT_PREFIX_PATH 自己拼
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_panda)   # .../install/<pkg>/share
    )

    robot_description_content = Command(['xacro', ' ', xacro_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content,
        }]
    )

    ignition_gazebo_node = IncludeLaunchDescription(
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
    clock_and_topics_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
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
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    load_eef_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'eef_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
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

    ld = LaunchDescription()

    # 顺序：先环境变量，再 Gazebo，再 bridge，再 spawn，再 controllers
    ld.add_action(gz_resource_path)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ignition_gazebo_node)
    ld.add_action(clock_and_topics_bridge)
    ld.add_action(spawn_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_position_controller)
    ld.add_action(load_eef_controller)
    ld.add_action(depth_cam_data2cam_link_tf)

    return ld
