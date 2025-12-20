from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Đường dẫn tuyệt đối tới robot.urdf.xacro ở SRC (như bạn nói)
    xacro_file = "/home/minh/ros2_ws/src/dhtbot_one/description/robot.urdf.xacro"

    robot_description_content = Command([
        "xacro ",          # CÓ khoảng trắng
        xacro_file,
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    controllers_yaml = "/home/minh/ros2_ws/src/dhtbot_one/config/my_controllers.yaml"
    world_file = "/home/minh/ros2_ws/install/dhtbot_one/share/dhtbot_one/worlds/empty.world"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                "/opt/ros/jazzy/share/ros_gz_sim/launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items(),
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_yaml,
            robot_description,
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dhtbot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
            "-topic", "robot_description",
        ],
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        ros2_control_node,
        spawn_entity,
    ])
