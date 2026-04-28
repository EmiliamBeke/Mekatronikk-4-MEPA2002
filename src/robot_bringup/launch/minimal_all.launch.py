from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')
    rviz_enabled = LaunchConfiguration('rviz')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    keyboard_teleop_enabled = LaunchConfiguration('keyboard_teleop')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_ekf = LaunchConfiguration('use_ekf')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    sim_track_width_eff_m = LaunchConfiguration('sim_track_width_eff_m')
    sim_max_track_speed_mps = LaunchConfiguration('sim_max_track_speed_mps')

    robot_bringup_share = get_package_share_directory('robot_bringup')

    world = os.path.join(
        get_package_share_directory('robot_gz'),
        'worlds',
        'tracked_robot_world.sdf'
    )
    gz_models_path = os.path.join(
        get_package_share_directory('robot_gz'),
        'models'
    )
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = (
        gz_models_path
        if not existing_gz_resource_path
        else gz_models_path + os.pathsep + existing_gz_resource_path
    )

    default_rviz_config = os.path.join(
        robot_bringup_share,
        'rviz',
        'rviz.rviz'
    )
    default_nav2_params = os.path.join(robot_bringup_share, 'config', 'nav2_params.yaml')
    default_ekf_params = os.path.join(robot_bringup_share, 'config', 'ekf.yaml')

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v', gz_verbosity, world],
        output='screen',
        condition=UnlessCondition(headless),
    )

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-v', gz_verbosity, world],
        output='screen',
        condition=IfCondition(headless),
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/tracked_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    # TrackedVehicle advertises /wheel/odom after the model plugin is initialized.
    # Starting this bridge separately avoids missing the Gazebo-side odom topic.
    odom_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    tracked_cmd_vel_adapter = Node(
        package='robot_sim_control',
        executable='tracked_cmd_vel_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'track_width_eff_m': sim_track_width_eff_m},
            {'max_track_speed_mps': sim_max_track_speed_mps},
            {'swap_sides': False},
            {'left_cmd_sign': 1},
            {'right_cmd_sign': 1},
            {'left_cmd_scale': 1.0},
            {'right_cmd_scale': 1.0},
            {'input_topic': '/cmd_vel'},
            {'output_topic': '/model/tracked_robot/cmd_vel'},
        ],
    )

    shared_core_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'pi_robot.launch.py')
        ),
        launch_arguments={
            'use_nav2': use_nav2,
            'use_lidar': 'false',
            'use_teddy': 'false',
            'use_imu': 'false',
            'use_mega_driver': 'false',
            'use_ekf': use_ekf,
            'use_joint_states': 'false',
            'use_sim_time': 'true',
            'rviz': rviz_enabled,
            'params_file': params_file,
            'ekf_params_file': ekf_params_file,
            'rviz_config': rviz_config,
        }.items(),
    )

    keyboard_teleop = Node(
        package='mekk4_bringup',
        executable='ros_keyboard_teleop',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                keyboard_teleop_enabled,
                "' == 'true' and '",
                headless,
                "' != 'true'",
            ])
        ),
    )
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_sim',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'base_laser',
        ],
    )
    # Gazebo can publish lidar scans with scoped frame ids like
    # "tracked_robot/chassis/lidar". Provide an alias frame so Nav2's
    # collision_monitor can resolve transforms reliably.
    lidar_scoped_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_scoped_frame_alias_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'tracked_robot/chassis/lidar',
        ],
    )
    lidar_scoped_base_laser_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_scoped_base_laser_alias_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_laser',
            '--child-frame-id', 'tracked_robot/base_laser',
        ],
    )
    # Gazebo IMU messages can use scoped frame ids like
    # "tracked_robot/chassis/imu". Provide an alias to imu_link so EKF
    # can resolve the sensor->base transform.
    imu_scoped_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_scoped_frame_alias_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'tracked_robot/chassis/imu',
        ],
    )

    overhead_camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_camera_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '3.0',
            '--roll', '0', '--pitch', '-1.5707963267948966', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'overhead_camera_link',
        ],
    )

    # Give Gazebo a short head start, then bring up bridge/TF before Nav2.
    start_sim_io = TimerAction(
        period=1.0,
        actions=[
            bridge,
            tracked_cmd_vel_adapter,
            lidar_static_tf,
            lidar_scoped_frame_alias_tf,
            lidar_scoped_base_laser_alias_tf,
            imu_scoped_frame_alias_tf,
            overhead_camera_static_tf,
            keyboard_teleop,
        ]
    )

    # Nav2 is sensitive to startup races between /clock, /tf and the first scan.
    start_core_stack = TimerAction(
        period=3.0,
        actions=[
            odom_bridge,
            shared_core_stack,
        ]
    )

    # Start simuleringen automatisk (fjerner pause ved oppstart).
    unpause_world = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/robot_world/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'pause: false',
        ],
        output='screen',
        condition=IfCondition(autostart),
    )
    autostart_sim = TimerAction(period=2.0, actions=[unpause_world])

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without GUI (server-only).'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically unpause Gazebo after startup.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Run RViz visualizer.'
        ),
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='2',
            description='Gazebo verbosity level (0-4).'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Run Nav2 from the shared pi_robot core stack.'
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run EKF from the shared pi_robot core stack.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Nav2 parameters file used by shared core stack.'
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=default_ekf_params,
            description='EKF parameters file used by shared core stack.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config used by shared core stack.'
        ),
        DeclareLaunchArgument(
            'sim_track_width_eff_m',
            default_value='0.184',
            description='Effective track width for sim cmd_vel conversion.'
        ),
        DeclareLaunchArgument(
            'sim_max_track_speed_mps',
            default_value='0.5555555555555556',
            description='Max per-track speed clamp for sim cmd_vel conversion.'
        ),
        DeclareLaunchArgument(
            'keyboard_teleop',
            default_value='true',
            description='Run local keyboard teleop window for sim.'
        ),
        SetEnvironmentVariable('ROS_USE_SIM_TIME', 'true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        gz_gui,
        gz_headless,
        start_sim_io,
        start_core_stack,
        autostart_sim,
    ])
