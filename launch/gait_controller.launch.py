import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_dog_laser_designator')
    default_params_file = os.path.join(pkg_share, 'config', 'gait_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the gait controller parameters YAML file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo instead of wall-clock time',
    )

    controller_node = Node(
        package='robot_dog_laser_designator',
        executable='robot_dog_controller_node',
        name='robot_dog_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        controller_node,
    ])
