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
    use_teddy = LaunchConfiguration('use_teddy')
    use_overhead_apriltag = LaunchConfiguration('use_overhead_apriltag')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    overhead_odom_params_file = LaunchConfiguration('overhead_odom_params_file')
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
    default_apriltag_params = os.path.join(robot_bringup_share, 'config', 'apriltag_overhead.yaml')
    default_overhead_odom_params = os.path.join(robot_bringup_share, 'config', 'overhead_odom.yaml')
    workspace_root = os.path.abspath(os.path.join(robot_bringup_share, '..', '..', '..', '..'))
    default_teddy_model = os.path.join(workspace_root, 'models', 'yolo26n_ncnn_model')

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
            'use_teddy': use_teddy,
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
            '--x', '0', '--y', '0', '--z', '6.2',
            '--roll', '0', '--pitch', '1.5707963267948966', '--yaw', '1.5707963267948966',
            '--frame-id', 'odom',
            '--child-frame-id', 'overhead_camera_link',
        ],
    )

    overhead_camera_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_camera_alias_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'overhead_camera_link',
            '--child-frame-id', 'overhead_camera',
        ],
    )

    overhead_apriltag_detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        condition=IfCondition(use_overhead_apriltag),
        parameters=[
            {'use_sim_time': True},
            apriltag_params_file,
        ],
        remappings=[
            ('image_rect', '/overhead_camera'),
            ('camera_info', '/overhead_camera/camera_info'),
            ('/camera_info', '/overhead_camera/camera_info'),
            ('detections', '/overhead/tag_detections'),
        ],
    )

    overhead_camera_info = Node(
        package='mekk4_perception',
        executable='sim_camera_info_publisher',
        name='overhead_camera_info',
        output='screen',
        condition=IfCondition(use_overhead_apriltag),
        parameters=[
            {'use_sim_time': True},
            {
                'image_topic': '/overhead_camera',
                'camera_info_topic': '/overhead_camera/camera_info',
                'camera_info_alias_topic': '/camera_info',
                'frame_id': 'overhead_camera_link',
                'horizontal_fov': 0.4997595236619115,
            },
        ],
    )

    overhead_apriltag_odom = Node(
        package='mekk4_perception',
        executable='overhead_apriltag_odom',
        name='overhead_apriltag_odom',
        output='screen',
        condition=IfCondition(use_overhead_apriltag),
        parameters=[
            {'use_sim_time': True},
            overhead_odom_params_file,
        ],
    )

    sim_camera_udp_stream = Node(
        package='mekk4_perception',
        executable='sim_camera_udp_stream',
        name='sim_camera_udp_stream',
        output='screen',
        condition=IfCondition(use_teddy),
        parameters=[
            {'use_sim_time': True},
            {
                'image_topic': '/camera',
                'host': '127.0.0.1',
                'port': 5600,
                'fps': 15,
                'bitrate_kbps': 1400,
            },
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
            overhead_camera_alias_tf,
            overhead_camera_info,
            overhead_apriltag_detector,
            overhead_apriltag_odom,
            sim_camera_udp_stream,
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
            'use_teddy',
            default_value='true',
            description='Run teddy_detector in sim using a local UDP camera adapter.'
        ),
        DeclareLaunchArgument(
            'use_overhead_apriltag',
            default_value='true',
            description='Run simulated overhead AprilTag detection and /overhead/odom publisher.'
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
            'apriltag_params_file',
            default_value=default_apriltag_params,
            description='AprilTag detector parameters for the simulated overhead camera.'
        ),
        DeclareLaunchArgument(
            'overhead_odom_params_file',
            default_value=default_overhead_odom_params,
            description='Parameters for converting overhead AprilTag detections to /overhead/odom.'
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
        SetEnvironmentVariable(
            'MEKK4_CAM_SOURCE_GST',
            'udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1 sync=false',
        ),
        SetEnvironmentVariable('MEKK4_CAM_WIDTH', '640'),
        SetEnvironmentVariable('MEKK4_CAM_HEIGHT', '480'),
        SetEnvironmentVariable('MEKK4_CAM_FPS', '15'),
        SetEnvironmentVariable('MEKK4_NCNN_MODEL', default_teddy_model),
        SetEnvironmentVariable('MEKK4_CONF', '0.3'),
        SetEnvironmentVariable('MEKK4_IMGSZ', '640'),
        SetEnvironmentVariable('MEKK4_CENTER_TOL', '0.10'),
        SetEnvironmentVariable('MEKK4_DEBUG_IMAGE', '1'),
        SetEnvironmentVariable('MEKK4_DEBUG_IMAGE_TOPIC', '/teddy_detector/debug_image'),
        SetEnvironmentVariable('MEKK4_DEBUG_IMAGE_SCALE', '1.0'),
        SetEnvironmentVariable('MEKK4_DEBUG_IMAGE_FPS', '5.0'),
        SetEnvironmentVariable('MEKK4_DEBUG_STREAM', '0'),
        gz_gui,
        gz_headless,
        start_sim_io,
        start_core_stack,
        autostart_sim,
    ])
