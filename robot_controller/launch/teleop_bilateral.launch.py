from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='fictitious_force_node',
            name='fictitious_force_node',
            output='screen',
        ),
        Node(
            package='robot_controller',
            executable='slave_controller',
            name='slave_controller',
            output='screen',
        ),
        Node(
            package='robot_controller',
            executable='master_controller',
            name='master_controller',
            output='screen',
        ),
        Node(
            package='robot_controller',
            executable='delay_relay_node',
            name='delay_relay_node',
            output='screen',
        ),
        Node(
            package='robot_controller',
            executable='data_logger',
            name='data_logger',
            output='screen',
        ),
    ])