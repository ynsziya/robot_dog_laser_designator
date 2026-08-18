import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        get_package_share_directory('fast_lio')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "fast_lio is built under ~/ros2_ws/install/fast_lio but this "
            "shell still has a stale ROS overlay (opened before that "
            "package existed). Re-source, then launch again:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source $HOME/ros2_ws/install/setup.bash\n"
            "  ros2 launch robot_dog_laser_designator gazebo.launch.py"
        ) from exc

    pkg_share = get_package_share_directory('robot_dog_laser_designator')
    default_config = os.path.join(pkg_share, 'config', 'fast_lio_gazebo.yaml')
    # Write PCD next to the source package (package.xml is typically a symlink),
    # not into cwd (~/ros2_ws) or a copied install/ share tree.
    src_pkg = os.path.dirname(os.path.realpath(os.path.join(pkg_share, 'package.xml')))
    maps_dir = os.path.join(src_pkg, 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    default_map_pcd = os.path.join(maps_dir, 'fast_lio.pcd')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    map_file_path = LaunchConfiguration('map_file_path')

    # IMU pose in base_footprint (identity rotation):
    #   imu in base_link = (0.066090, 0.035451, 0.002617)
    #   base_link in base_footprint = (0, 0, 0.722900)
    # FAST-LIO body == IMU, so body -> base_footprint is the inverse.
    body_to_footprint = ('-0.066090', '-0.035451', '-0.725517')

    mapping_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_path': map_file_path,
            },
        ],
        remappings=[('/tf', '/tf_fastlio')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='FAST-LIO YAML (Gazebo gpu_lidar + IMU)',
        ),
        DeclareLaunchArgument(
            'map_file_path', default_value=default_map_pcd,
            description='PCD path for /map_save (package maps/ directory)',
        ),
        Node(
            package='robot_dog_laser_designator',
            executable='fast_lio_bridge_node',
            name='fast_lio_bridge_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'imu_input_topic': '/imu',
                'imu_output_topic': '/imu_fastlio',
                'cloud_input_topic': '/scan/points',
                'cloud_output_topic': '/fast_lio/points',
                'odom_topic': '/Odometry',
                'parent_frame': 'camera_init',
                'child_frame': 'body',
                'tf_rate_hz': 50.0,
                'min_range': 0.5,
                'max_range': 29.0,
                'scan_period': 0.1,
            }],
        ),
        # Wait for Gazebo IMU to leave the ~0 / spike startup window.
        TimerAction(period=4.0, actions=[mapping_node]),
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
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_footprint',
            arguments=[
                '--x', body_to_footprint[0],
                '--y', body_to_footprint[1],
                '--z', body_to_footprint[2],
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'body',
                '--child-frame-id', 'base_footprint',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
