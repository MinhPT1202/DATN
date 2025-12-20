import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare("dhtbot_one")

    xacro_file = PathJoinSubstitution([pkg, "description", "robot.urdf.xacro"])
    robot_description_content = Command(["xacro ", xacro_file])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers = PathJoinSubstitution([pkg, "config", "my_controllers.yaml"])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="screen"
    )

    return LaunchDescription([controller_manager, rsp])
