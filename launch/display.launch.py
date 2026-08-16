import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_dog_laser_designator')
    default_urdf = os.path.join(pkg_share, 'urdf', 'spot_zero.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')

    with open(default_urdf, 'r', encoding='utf-8') as urdf_file:
        # SPOT_CONTROLLERS_YAML is only consumed by the gz_ros2_control
        # Gazebo plugin, which isn't loaded here -- substitute a harmless
        # placeholder so the string is well-formed either way.
        robot_description = urdf_file.read().replace('SPOT_CONTROLLERS_YAML', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz,
            description='Path to the RViz config file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
