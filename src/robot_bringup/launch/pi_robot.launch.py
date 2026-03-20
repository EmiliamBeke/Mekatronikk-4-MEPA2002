from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'tracked_robot.urdf',
    )
    with open(robot_description_path, 'r', encoding='utf-8') as handle:
        robot_description_content = handle.read()

    robot_bringup_share = get_package_share_directory('robot_bringup')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_teddy = LaunchConfiguration('use_teddy')
    use_imu = LaunchConfiguration('use_imu')
    use_mega_driver = LaunchConfiguration('use_mega_driver')
    use_ekf = LaunchConfiguration('use_ekf')
    use_joint_states = LaunchConfiguration('use_joint_states')
    product_name = LaunchConfiguration('product_name')
    port_name = LaunchConfiguration('port_name')
    port_baudrate = LaunchConfiguration('port_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    mega_port = LaunchConfiguration('mega_port')
    mega_baudrate = LaunchConfiguration('mega_baudrate')
    mega_odom_topic = LaunchConfiguration('mega_odom_topic')
    mega_publish_tf = LaunchConfiguration('mega_publish_tf')
    swap_sides = LaunchConfiguration('swap_sides')
    left_cmd_sign = LaunchConfiguration('left_cmd_sign')
    right_cmd_sign = LaunchConfiguration('right_cmd_sign')
    left_cmd_scale = LaunchConfiguration('left_cmd_scale')
    right_cmd_scale = LaunchConfiguration('right_cmd_scale')
    left_tick_sign = LaunchConfiguration('left_tick_sign')
    right_tick_sign = LaunchConfiguration('right_tick_sign')
    left_m_per_tick = LaunchConfiguration('left_m_per_tick')
    right_m_per_tick = LaunchConfiguration('right_m_per_tick')
    track_width_eff_m = LaunchConfiguration('track_width_eff_m')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}, {'robot_description': robot_description_content}],
    )

    joint_state_publisher = Node(
        package='mekk4_bringup',
        executable='zero_joint_state_publisher',
        name='zero_joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_states),
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
            'mount_frame': 'lidar_link',
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

    mega_driver_node = Node(
        package='mekk4_bringup',
        executable='mega_driver_node',
        name='mega_driver',
        output='screen',
        condition=IfCondition(use_mega_driver),
        parameters=[
            {
                'port': ParameterValue(mega_port, value_type=str),
                'baudrate': ParameterValue(mega_baudrate, value_type=int),
                'base_frame_id': ParameterValue(base_frame, value_type=str),
                'publish_tf': ParameterValue(mega_publish_tf, value_type=bool),
                'swap_sides': ParameterValue(swap_sides, value_type=bool),
                'left_cmd_sign': ParameterValue(left_cmd_sign, value_type=int),
                'right_cmd_sign': ParameterValue(right_cmd_sign, value_type=int),
                'left_cmd_scale': ParameterValue(left_cmd_scale, value_type=float),
                'right_cmd_scale': ParameterValue(right_cmd_scale, value_type=float),
                'left_tick_sign': ParameterValue(left_tick_sign, value_type=int),
                'right_tick_sign': ParameterValue(right_tick_sign, value_type=int),
                'left_m_per_tick': ParameterValue(left_m_per_tick, value_type=float),
                'right_m_per_tick': ParameterValue(right_m_per_tick, value_type=float),
                'track_width_eff_m': ParameterValue(track_width_eff_m, value_type=float),
            }
        ],
        remappings=[('odom', mega_odom_topic)],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[ekf_params_file],
        remappings=[('odometry/filtered', 'odom')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_teddy', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_mega_driver', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_joint_states', default_value='true'),
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_LD06'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('port_baudrate', default_value='230400'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('base_frame', default_value='chassis'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('mega_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('mega_baudrate', default_value='115200'),
        DeclareLaunchArgument('mega_odom_topic', default_value='odom'),
        DeclareLaunchArgument('mega_publish_tf', default_value='true'),
        DeclareLaunchArgument('swap_sides', default_value='true'),
        DeclareLaunchArgument('left_cmd_sign', default_value='1'),
        DeclareLaunchArgument('right_cmd_sign', default_value='1'),
        DeclareLaunchArgument('left_cmd_scale', default_value='1.0'),
        DeclareLaunchArgument('right_cmd_scale', default_value='1.0'),
        DeclareLaunchArgument('left_tick_sign', default_value='1'),
        DeclareLaunchArgument('right_tick_sign', default_value='1'),
        DeclareLaunchArgument('left_m_per_tick', default_value='0.0'),
        DeclareLaunchArgument('right_m_per_tick', default_value='0.0'),
        DeclareLaunchArgument('track_width_eff_m', default_value='0.35'),
        DeclareLaunchArgument('ekf_params_file', default_value='/ws/config/ekf.yaml'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='0.0'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('params_file', default_value='/ws/config/nav2_params.yaml'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', False),
        joint_state_publisher,
        robot_state_publisher,
        imu_node,
        mega_driver_node,
        ekf_node,
        lidar_launch,
        nav2_launch,
        teddy_detector,
    ])
