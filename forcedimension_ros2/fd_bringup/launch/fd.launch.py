from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_orientation = LaunchConfiguration("use_orientation")
    use_clutch = LaunchConfiguration("use_clutch")

    # =========================
    # 1) robot_description (xacro)
    # =========================
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("fd_description"),
                    "config",
                    "fd.config.xacro",
                ]
            ),
            " use_fake_hardware:=",
            use_fake_hardware,
            " use_orientation:=",
            use_orientation,
            " use_clutch:=",
            use_clutch,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # =========================
    # 2) controllers YAML
    # =========================
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("fd_description"),
            "config",
            "/home/minh/ros2_ws/src/forcedimension_ros2/fd_description/config/fd_controllers.yaml",
        ]
    )

    # =========================
    # 3) Nodes
    # =========================

    # robot_state_publisher (namespace fd)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fd",
        output="screen",
        parameters=[robot_description],
    )

    # world -> fd_base static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "3.1416",
            "0.0",
            "0.0",
            "world",
            "fd_base",
        ],
    )

    # ros2_control_node (controller_manager) under namespace fd
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="fd",
        parameters=[robot_description, controllers_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )

    # Spawner cho các controller
    load_controllers = []
    for controller in [
        "fd_controller",
        "joint_state_broadcaster",
        "fd_ee_broadcaster",
        "fd_inertia_broadcaster",
        "fd_clutch_broadcaster",
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="fd",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/fd/controller_manager",
                ],
                output="screen",
            )
        )

    # =========================
    # LaunchDescription
    # =========================
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use fake FD hardware interface (no real device)",
            ),
            DeclareLaunchArgument(
                "use_orientation",
                default_value="false",
                description="Enable orientation DOF (roll/pitch/yaw)",
            ),
            DeclareLaunchArgument(
                "use_clutch",
                default_value="false",
                description="Enable clutch interface",
            ),
            controller_manager_node,
            node_robot_state_publisher,
            static_tf,
        ]
        + load_controllers
    )
