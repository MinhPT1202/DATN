import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Tên robot trong Gazebo và tên package
    robot_name = "acckerman"
    package_name = "acckerman_gazebo"

    # =========================
    # 1) Tham số world + pose
    # =========================
    default_world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "test.sdf",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="World SDF file for Gazebo (must include Sensors plugin)",
    )

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Initial X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Initial Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.5", description="Initial Z position")
    roll_arg = DeclareLaunchArgument("R", default_value="0.0", description="Initial roll")
    pitch_arg = DeclareLaunchArgument("P", default_value="0.0", description="Initial pitch")
    yaw_arg = DeclareLaunchArgument("Y", default_value="0.0", description="Initial yaw")

    world_file = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("R")
    pitch = LaunchConfiguration("P")
    yaw = LaunchConfiguration("Y")

    # =========================
    # 2) Đường dẫn model + bridge + ros2_control
    # =========================
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        "model",
        "robot.xacro",
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "gz_bridge.yaml",
    )

    ros2_control_params_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "gz_ros2_control.yaml",
    )

    # =========================
    # 3) Sinh URDF từ xacro
    # =========================
    robot_description = xacro.process_file(robot_model_path).toxml()

    # =========================
    # 4) Launch Gazebo (ros_gz_sim)
    # =========================
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py",
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # =========================
    # 5) Spawn robot vào Gazebo
    # =========================
    spawn_model_gazebo_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-string", robot_description,
            "-x", x,
            "-y", y,
            "-z", z,
            "-R", roll,
            "-P", pitch,
            "-Y", yaw,
            "-allow_renaming", "false",
        ],
        output="screen",
    )

    # =========================
    # 6) robot_state_publisher
    # =========================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    # =========================
    # 7) ros2_control_node + spawners
    # =========================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": True},
            ros2_control_params_path,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    forward_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    forward_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # =========================
    # 8) ros_gz_bridge (config file)
    # =========================
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={gz_bridge_params_path}",
        ],
        output="screen",
    )

    # =========================
    # 9) LaunchDescription
    # =========================
    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        gazebo_launch,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        forward_velocity_spawner,
        forward_position_spawner,
        gz_bridge_node,
    ])
