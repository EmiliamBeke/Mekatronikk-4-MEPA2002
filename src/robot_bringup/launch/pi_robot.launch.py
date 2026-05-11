from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def _str_to_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def require_robotarm_safety_for_teddy_grab(context, *args, **kwargs):
    use_teddy_grab = LaunchConfiguration('use_teddy_grab').perform(context).lower()
    use_robotarm_safety = LaunchConfiguration('use_robotarm_safety').perform(context).lower()
    if use_teddy_grab in ('1', 'true', 'yes', 'on') and use_robotarm_safety not in (
        '1',
        'true',
        'yes',
        'on',
    ):
        raise RuntimeError(
            'use_teddy_grab:=true requires use_robotarm_safety:=true; '
            'all arm/gripper commands must pass through robotarm_safety_node.'
        )
    return []


def generate_launch_description():
    robot_bringup_share = get_package_share_directory('robot_bringup')
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'tracked_robot.urdf',
    )
    with open(robot_description_path, 'r', encoding='utf-8') as handle:
        robot_description_content = handle.read()

    default_ekf_params_path = os.path.join(robot_bringup_share, 'config', 'ekf.yaml')
    default_nav2_params_path = os.path.join(robot_bringup_share, 'config', 'nav2_params.yaml')
    default_teddy_approach_params_path = os.path.join(
        robot_bringup_share,
        'config',
        'teddy_approach.yaml',
    )
    default_teddy_grab_params_path = os.path.join(
        robot_bringup_share,
        'config',
        'teddy_grab.yaml',
    )
    default_robotarm_params_path = os.path.join(
        robot_bringup_share,
        'config',
        'robotarm_params.yaml',
    )
    default_rviz_config_path = os.path.join(robot_bringup_share, 'rviz', 'rviz.rviz')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_lidar = LaunchConfiguration('use_lidar')
    use_teddy = LaunchConfiguration('use_teddy')
    use_teddy_approach = LaunchConfiguration('use_teddy_approach')
    use_teddy_grab = LaunchConfiguration('use_teddy_grab')
    use_imu = LaunchConfiguration('use_imu')
    use_mega_driver = LaunchConfiguration('use_mega_driver')
    use_ekf = LaunchConfiguration('use_ekf')
    use_joint_states = LaunchConfiguration('use_joint_states')
    use_robotarm_safety = LaunchConfiguration('use_robotarm_safety')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_enabled = LaunchConfiguration('rviz')
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
    angular_cmd_sign = LaunchConfiguration('angular_cmd_sign')
    min_nonzero_pwm = LaunchConfiguration('min_nonzero_pwm')
    min_forward_pwm = LaunchConfiguration('min_forward_pwm')
    min_reverse_pwm = LaunchConfiguration('min_reverse_pwm')
    min_turn_pwm = LaunchConfiguration('min_turn_pwm')
    pure_rotation_linear_deadband_mps = LaunchConfiguration('pure_rotation_linear_deadband_mps')
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
    teddy_approach_params_file = LaunchConfiguration('teddy_approach_params_file')
    teddy_grab_params_file = LaunchConfiguration('teddy_grab_params_file')
    robotarm_params_file = LaunchConfiguration('robotarm_params_file')
    nav2_start_delay_s = LaunchConfiguration('nav2_start_delay_s')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description_content}],
    )

    joint_state_publisher = Node(
        package='mekk4_bringup',
        executable='zero_joint_state_publisher',
        name='zero_joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_states),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'lidar_nav2_compat.launch.py')
        ),
        condition=IfCondition(use_lidar),
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
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'nav2_stack.launch.py')
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )
    delayed_nav2_launch = TimerAction(
        period=nav2_start_delay_s,
        actions=[nav2_launch],
        condition=IfCondition(use_nav2),
    )

    teddy_detector = Node(
        package='mekk4_perception',
        executable='teddy_detector',
        name='teddy_detector',
        output='screen',
        condition=IfCondition(use_teddy),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    teddy_approach_node = Node(
        package='mekk4_bringup',
        executable='teddy_approach_node',
        name='teddy_approach',
        output='screen',
        condition=IfCondition(use_teddy_approach),
        parameters=[
            {'use_sim_time': use_sim_time},
            teddy_approach_params_file,
            {'enabled': ParameterValue(use_teddy_approach, value_type=bool)},
        ],
    )

    teddy_grab_node = Node(
        package='mekk4_bringup',
        executable='teddy_grab_node',
        name='teddy_grab',
        output='screen',
        condition=IfCondition(use_teddy_grab),
        parameters=[
            {'use_sim_time': use_sim_time},
            teddy_grab_params_file,
            {'enabled': ParameterValue(use_teddy_grab, value_type=bool)},
        ],
    )

    robotarm_safety_node = Node(
        package='mekk4_bringup',
        executable='robotarm_safety_node',
        name='robotarm_safety',
        output='screen',
        condition=IfCondition(use_robotarm_safety),
        parameters=[
            {'use_sim_time': use_sim_time},
            robotarm_params_file,
        ],
    )

    teddy_lidar_markers_node = Node(
        package='mekk4_bringup',
        executable='teddy_lidar_markers_node',
        name='teddy_lidar_markers',
        output='screen',
        condition=IfCondition(use_teddy_approach),
        parameters=[
            {'use_sim_time': use_sim_time},
            teddy_approach_params_file,
            {'enabled': ParameterValue(use_teddy_approach, value_type=bool)},
        ],
    )

    dist_sensor_marker_node = Node(
        package='mekk4_bringup',
        executable='dist_sensor_marker_node',
        name='dist_sensor_marker',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    teddy_nav_goal_node = Node(
        package='mekk4_bringup',
        executable='teddy_nav_goal_node',
        name='teddy_nav_goal',
        output='screen',
        condition=IfCondition(use_teddy_approach),
        parameters=[
            {'use_sim_time': use_sim_time},
            teddy_approach_params_file,
            {'enabled': ParameterValue(use_teddy_approach, value_type=bool)},
        ],
    )

    imu_node = Node(
        package='mekk4_bringup',
        executable='bno085_node',
        name='bno085',
        output='screen',
        condition=IfCondition(use_imu),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': imu_frame},
        ],
    )

    def _build_mega_driver_node(context, *args, **kwargs):
        if not _str_to_bool(LaunchConfiguration('use_mega_driver').perform(context)):
            return []
        ekf_on = _str_to_bool(LaunchConfiguration('use_ekf').perform(context))
        publish_tf_bool = (
            False
            if ekf_on
            else _str_to_bool(LaunchConfiguration('mega_publish_tf').perform(context))
        )
        print(f"[pi_robot.launch] mega_driver publish_tf -> {publish_tf_bool} (use_ekf={ekf_on})")
        return [Node(
            package='mekk4_bringup',
            executable='mega_driver_node',
            name='mega_driver',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                robotarm_params_file,
                {
                    'port': ParameterValue(mega_port, value_type=str),
                    'baudrate': ParameterValue(mega_baudrate, value_type=int),
                    'base_frame_id': ParameterValue(base_frame, value_type=str),
                    'publish_tf': publish_tf_bool,
                    'swap_sides': ParameterValue(swap_sides, value_type=bool),
                    'left_cmd_sign': ParameterValue(left_cmd_sign, value_type=int),
                    'right_cmd_sign': ParameterValue(right_cmd_sign, value_type=int),
                    'angular_cmd_sign': ParameterValue(angular_cmd_sign, value_type=int),
                    'min_nonzero_pwm': ParameterValue(min_nonzero_pwm, value_type=int),
                    'min_forward_pwm': ParameterValue(min_forward_pwm, value_type=int),
                    'min_reverse_pwm': ParameterValue(min_reverse_pwm, value_type=int),
                    'min_turn_pwm': ParameterValue(min_turn_pwm, value_type=int),
                    'pure_rotation_linear_deadband_mps': ParameterValue(
                        pure_rotation_linear_deadband_mps, value_type=float
                    ),
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
        )]

    mega_driver_node = OpaqueFunction(function=_build_mega_driver_node)

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[
            {'use_sim_time': use_sim_time},
            ekf_params_file,
        ],
        remappings=[('odometry/filtered', 'odom')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_enabled),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_teddy', default_value='false'),
        DeclareLaunchArgument('use_teddy_approach', default_value='false'),
        DeclareLaunchArgument('use_teddy_grab', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_mega_driver', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_joint_states', default_value='true'),
        DeclareLaunchArgument('use_robotarm_safety', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_LD06'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('port_baudrate', default_value='230400'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('mega_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('mega_baudrate', default_value='115200'),
        DeclareLaunchArgument('mega_odom_topic', default_value='odom'),
        DeclareLaunchArgument('mega_publish_tf', default_value='true'),
        DeclareLaunchArgument('swap_sides', default_value='false'),
        DeclareLaunchArgument('left_cmd_sign', default_value='1'),
        DeclareLaunchArgument('right_cmd_sign', default_value='1'),
        DeclareLaunchArgument('angular_cmd_sign', default_value='1'),
        DeclareLaunchArgument('min_nonzero_pwm', default_value='55'),
        DeclareLaunchArgument('min_forward_pwm', default_value='0'),
        DeclareLaunchArgument('min_reverse_pwm', default_value='0'),
        DeclareLaunchArgument('min_turn_pwm', default_value='0'),
        DeclareLaunchArgument('pure_rotation_linear_deadband_mps', default_value='0.03'),
        DeclareLaunchArgument('left_cmd_scale', default_value='1.0'),
        DeclareLaunchArgument('right_cmd_scale', default_value='1.0'),
        DeclareLaunchArgument('left_tick_sign', default_value='1'),
        DeclareLaunchArgument('right_tick_sign', default_value='1'),
        DeclareLaunchArgument('left_m_per_tick', default_value='0.0'),
        DeclareLaunchArgument('right_m_per_tick', default_value='0.0'),
        DeclareLaunchArgument('track_width_eff_m', default_value='0.35'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params_path),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='0.0'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params_path),
        DeclareLaunchArgument('nav2_start_delay_s', default_value='4.0'),
        DeclareLaunchArgument(
            'teddy_approach_params_file',
            default_value=default_teddy_approach_params_path,
        ),
        DeclareLaunchArgument(
            'teddy_grab_params_file',
            default_value=default_teddy_grab_params_path,
        ),
        DeclareLaunchArgument(
            'robotarm_params_file',
            default_value=default_robotarm_params_path,
        ),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=require_robotarm_safety_for_teddy_grab),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        joint_state_publisher,
        robot_state_publisher,
        imu_node,
        mega_driver_node,
        ekf_node,
        lidar_launch,
        delayed_nav2_launch,
        teddy_detector,
        teddy_approach_node,
        robotarm_safety_node,
        teddy_grab_node,
        teddy_lidar_markers_node,
        dist_sensor_marker_node,
        teddy_nav_goal_node,
        rviz_node,
    ])
