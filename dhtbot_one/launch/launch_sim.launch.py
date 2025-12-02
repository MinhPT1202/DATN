import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'dhtbot_one'

    # 1) Robot State Publisher (xacro -> robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # 2) Gazebo (gz sim) + world
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
        }.items(),
    )

    # 3) Spawn robot từ file URDF (đã sinh sẵn từ robot.urdf.xacro)
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    # 4) Spawner cho controllers của robot (controller_manager trong Gazebo)
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_broad',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # 5) Để Gazebo tìm world/model trong package
    set_env_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name)),
    )

    return LaunchDescription([
        set_env_resources,
        gz_sim,
        rsp,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
