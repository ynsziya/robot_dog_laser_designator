import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_dog_laser_designator')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')
    src_pkg = os.path.dirname(os.path.realpath(os.path.join(pkg_share, 'package.xml')))
    default_pcd = os.path.join(src_pkg, 'maps', 'fast_lio.pcd')

    pcd_path = LaunchConfiguration('pcd_path')
    cloud_topic = LaunchConfiguration('cloud_topic')
    rviz_config = LaunchConfiguration('rvizconfig')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path', default_value=default_pcd,
            description='Saved FAST-LIO PCD (package maps/ directory)',
        ),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/Laser_map',
            description='Topic to republish the PCD on',
        ),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz,
            description='RViz config file',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map',
                '--child-frame-id', 'camera_init',
            ],
        ),
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='saved_map_publisher',
            output='screen',
            parameters=[{
                'file_name': pcd_path,
                'tf_frame': 'camera_init',
                'publishing_period_ms': 500,
            }],
            remappings=[('cloud_pcd', cloud_topic)],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
