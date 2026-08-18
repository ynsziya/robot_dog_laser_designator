import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _foot_contact_bridges(world_name: str, model_name: str):
    """Gazebo contact sensors publish on scoped .../contact topics, not /foot_contact/*."""
    feet = (
        ('fl', 'fl_knee', 'fl_foot_contact'),
        ('fr', 'fr_knee', 'fr_foot_contact'),
        ('rl', 'rl_knee', 'rl_foot_contact'),
        ('rr', 'rr_knee', 'rr_foot_contact'),
    )
    arguments = []
    remappings = []
    for leg, link, sensor in feet:
        gz_topic = (
            f'/world/{world_name}/model/{model_name}/link/{link}/sensor/{sensor}/contact'
        )
        ros_topic = f'/foot_contact/{leg}'
        arguments.append(
            f'{gz_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        )
        remappings.append((gz_topic, ros_topic))
    return arguments, remappings


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_dog_laser_designator')
    default_urdf = os.path.join(pkg_share, 'urdf', 'spot_zero.urdf')
    default_world = os.path.join(pkg_share, 'worlds', 'spot_obstacles.sdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')
    controllers_yaml = os.path.join(pkg_share, 'config', 'spot_controllers.yaml')
    default_gait_params = os.path.join(pkg_share, 'config', 'gait_params.yaml')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    with open(default_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read().replace(
            'SPOT_CONTROLLERS_YAML', controllers_yaml
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_sdf = LaunchConfiguration('world_sdf')
    world_name = LaunchConfiguration('world_name')
    entity_name = LaunchConfiguration('entity_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    rviz_config = LaunchConfiguration('rvizconfig')
    gait_params_file = LaunchConfiguration('gait_params_file')
    spawn_gait_controller = LaunchConfiguration('spawn_gait_controller')
    spawn_leg_odometry = LaunchConfiguration('spawn_leg_odometry')

    # world_name/entity_name are resolved as plain strings here (not via
    # LaunchConfiguration substitution) since they only feed into building
    # static Gazebo Transport topic names for the ros_gz_bridge arguments,
    # which must be known at launch-description-generation time.
    foot_contact_args, foot_contact_remappings = _foot_contact_bridges(
        'spot_obstacles', 'bosdyn_spot'
    )

    # Gazebo resolves "package://robot_dog_laser_designator/meshes/..." URIs by
    # searching GZ_SIM_RESOURCE_PATH for a directory literally named
    # "robot_dog_laser_designator" containing a "meshes" subfolder -- that's exactly
    # share/robot_dog_laser_designator, so we add its parent to the search path.
    resource_paths = [os.path.dirname(pkg_share)]
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_resource_path:
        resource_paths.insert(0, existing_resource_path)

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r '), world_sdf],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'world': world_name,
            'topic': 'robot_description',
            'entity_name': entity_name,
            'x': x,
            'y': y,
            'z': z,
            'Y': yaw,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items(),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('mapping_backend'), 'slam_toolbox')
        ),
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fast_lio.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('mapping_backend'), 'fast_lio')
        ),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Give Gazebo + the spawned robot a few seconds to settle before loading
    # controllers, matching the timing that has proven reliable for this
    # robot/world combination.
    delayed_controller_spawners = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, leg_controller_spawner],
    )

    gait_controller_node = Node(
        package='robot_dog_laser_designator',
        executable='robot_dog_controller_node',
        name='robot_dog_controller_node',
        output='screen',
        parameters=[
            gait_params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(spawn_gait_controller),
    )

    # Start just after the controllers are requested (they take a moment to
    # activate) so /leg_position_controller/commands already has a live
    # controller behind it once our node starts publishing the stand pose.
    delayed_gait_controller = TimerAction(
        period=4.0,
        actions=[gait_controller_node],
    )

    leg_odometry_node = Node(
        package='robot_dog_laser_designator',
        executable='leg_odometry_node',
        name='leg_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'joint_states_topic': '/joint_states',
            'imu_topic': '/imu',
            'odom_topic': '/leg_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'update_frequency_hz': 100.0,
        }],
        condition=IfCondition(spawn_leg_odometry),
    )

    # joint_state_broadcaster ~3s'te geliyor;
    delayed_leg_odometry = TimerAction(
        period=5.0,
        actions=[leg_odometry_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo Sim simulation clock',
        ),
        DeclareLaunchArgument(
            'world_sdf', default_value=default_world,
            description='Gazebo Sim world file name or absolute path',
        ),
        DeclareLaunchArgument(
            'world_name', default_value='spot_obstacles',
            description='Gazebo world name used by the create/spawn service',
        ),
        DeclareLaunchArgument(
            'entity_name', default_value='bosdyn_spot',
            description='Name the robot model is spawned under in Gazebo',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y'),
        DeclareLaunchArgument(
            'z', default_value='0.55',
            description='Spawn z (stand pose base ~0.50 m; URDF initial_value = stand)',
        ),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz,
            description='Path to the RViz config file',
        ),
        DeclareLaunchArgument(
            'gait_params_file', default_value=default_gait_params,
            description='Parameters file for robot_dog_controller_node',
        ),
        DeclareLaunchArgument(
            'spawn_gait_controller', default_value='true',
            description=(
                'Launch robot_dog_controller_node here. Set to false if you '
                'want to start it separately (e.g. via gait_controller.launch.py) '
                'so you can restart/tune it without restarting Gazebo.'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_leg_odometry', default_value='true',
            description=(
                'Launch leg_odometry_node. Set to false to run it separately '
                'while debugging without restarting Gazebo.'
            ),
        ),
        DeclareLaunchArgument(
            'mapping_backend', default_value='fast_lio',
            description=(
                'Mapping stack: fast_lio (3D LIO, default), slam_toolbox (2D), '
                'or none.'
            ),
        ),
        set_resource_path,
        gz_sim,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        spawn,
        slam,
        fast_lio,
        delayed_controller_spawners,
        delayed_gait_controller,
        delayed_leg_odometry,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                *foot_contact_args,
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=foot_contact_remappings,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('mapping_backend'), 'slam_toolbox')
            ),
        ),
    ])
