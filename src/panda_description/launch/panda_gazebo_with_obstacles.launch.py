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
    declared_arguments = []
    declared_arguments.append(gz_resource_path)

    # Parse URDF
    robot_description_file = os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(robot_description_file).toxml()}

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

    # Spawn
    spawn_node = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'panda', '-topic', '/robot_description'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    sdf_file_path = os.path.join(
        FindPackageShare('panda_description').find('panda_description'),
        'world',
        'planning_world.sdf'
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
            'gz_args': f'-r -v 4 {sdf_file_path}'
        }.items()
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_eef_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'eef_controller'],
        output='screen'
    )
    color_camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='color_camera_bridge',
        output='screen',
        parameters=[use_sim_time_true],
        arguments=['/color_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'],
        remappings=[('/color_camera', '/color_camera')]
    )

    depth_camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='depth_camera_bridge',
        output='screen',
        parameters=[use_sim_time_true],
        arguments=[
            '/depth_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
            '/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/depth_camera', '/depth_camera'),
            ('/depth_camera/points', '/depth_camera/points')
        ]
    )

    depth_cam_data2cam_link_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='cam3Tolink',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'panda/link0/d435_depth']
    )

    nodes = [
        load_joint_state_broadcaster,
        load_position_controller,
        gz_resource_path,
        robot_state_publisher_node,
        spawn_node,
        ignition_gazebo_node,
        clock_bridge_node,
        color_camera_bridge,
        depth_camera_bridge,
        depth_cam_data2cam_link_tf,
    ]

    ld = LaunchDescription(declared_arguments + nodes)
    return ld
