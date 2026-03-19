from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'two_wheel_robot.urdf',
    )
    with open(robot_description_path, 'r', encoding='utf-8') as handle:
        robot_description_content = handle.read()

    robot_bringup_share = get_package_share_directory('robot_bringup')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_teddy = LaunchConfiguration('use_teddy')
    use_imu = LaunchConfiguration('use_imu')
    product_name = LaunchConfiguration('product_name')
    port_name = LaunchConfiguration('port_name')
    port_baudrate = LaunchConfiguration('port_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}, {'robot_description': robot_description_content}],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'lidar_nav2_compat.launch.py')
        ),
        launch_arguments={
            'product_name': product_name,
            'port_name': port_name,
            'port_baudrate': port_baudrate,
            'topic_name': '/lidar',
            'frame_id': frame_id,
            'base_frame': base_frame,
            'tf_x': tf_x,
            'tf_y': tf_y,
            'tf_z': tf_z,
            'tf_roll': tf_roll,
            'tf_pitch': tf_pitch,
            'tf_yaw': tf_yaw,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'nav2_stack.launch.py')
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml,
            'params_file': params_file,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    teddy_detector = Node(
        package='mekk4_perception',
        executable='teddy_detector',
        name='teddy_detector',
        output='screen',
        condition=IfCondition(use_teddy),
    )

    imu_node = Node(
        package='mekk4_bringup',
        executable='bno085_node',
        name='bno085',
        output='screen',
        condition=IfCondition(use_imu),
        parameters=[{'frame_id': imu_frame}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_teddy', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_LD06'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('port_baudrate', default_value='230400'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('base_frame', default_value='chassis'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='0.18'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value='/ws/maps/my_map.yaml'),
        DeclareLaunchArgument('params_file', default_value='/ws/config/nav2_params.yaml'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', False),
        robot_state_publisher,
        imu_node,
        lidar_launch,
        nav2_launch,
        teddy_detector,
    ])
