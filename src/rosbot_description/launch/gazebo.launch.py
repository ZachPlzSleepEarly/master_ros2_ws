import os
from pathlib import Path

# ROS2 package路径查询
from ament_index_python.packages import get_package_share_directory

# ROS2 Launch核心模块
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ROS2 Node启动
from launch_ros.actions import Node

# Launch actions
from launch.actions import AppendEnvironmentVariable
import xacro
from os.path import join


def generate_launch_description():
    # ROS2 launch入口函数

    # -------------------------
    # Package 路径
    # -------------------------

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # ros_gz_sim package路径（Gazebo ROS接口）

    pkg_ros_gz_rosbot = get_package_share_directory('rosbot_description')
    # 机器人描述包路径

    # -------------------------
    # 机器人URDF / Bridge配置
    # -------------------------

    robot_description_file = os.path.join(pkg_ros_gz_rosbot, 'urdf', 'rosbot.xacro')
    # 机器人xacro文件

    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rosbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    # ROS ↔ Gazebo topic bridge配置

    # -------------------------
    # Gazebo world
    # -------------------------

    rosbot_path = get_package_share_directory("rosbot_description")

    world_file = LaunchConfiguration(
        "world_file",
        default=join(rosbot_path, "worlds", "hospital.sdf")
    )
    # 默认仿真world

    # -------------------------
    # 解析 xacro → URDF
    # -------------------------

    robot_description_config = xacro.process_file(robot_description_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }
    # 生成 robot_description 参数

    # -------------------------
    # Robot State Publisher
    # -------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # 根据 URDF 发布 TF
    # link → joint → TF tree

    # -------------------------
    # Gazebo resource path
    # -------------------------

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_ros_gz_rosbot, 'models')
    )
    # Gazebo模型路径

    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(pkg_ros_gz_rosbot)).parent.resolve())
    )

    # -------------------------
    # 启动 Gazebo
    # -------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),

        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    # gz_args:
    # world + -r 表示自动运行仿真

    # -------------------------
    # 向 Gazebo 生成机器人
    # -------------------------

    spawn = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            "-topic", "/robot_description",
            # 从 ROS topic 读取URDF

            "-name", "rosbot",
            "-allow_renaming", "true",

            "-z", "0.2",
            "-x", "-6.0",
            "-y", "0.0",
            "-Y", "0.0"
            # 机器人初始位姿
        ],

        output='screen',
    )

    # -------------------------
    # ROS ↔ Gazebo bridge
    # -------------------------

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        parameters=[{
            'config_file': ros_gz_bridge_config,
        }],

        output='screen'
    )

    # 读取 ros_gz_bridge_gazebo.yaml
    # 建立 ROS topic 与 Gazebo topic 通信

    return LaunchDescription(
        [

            # 启动 Gazebo
            gazebo,

            # 在 Gazebo 中生成机器人
            spawn,

            # 建立 ROS ↔ Gazebo bridge
            start_gazebo_ros_bridge_cmd,

            # 发布 TF
            robot_state_publisher,

        ]
    )
