from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')
    rviz_enabled = LaunchConfiguration('rviz')
    keyboard_teleop_enabled = LaunchConfiguration('keyboard_teleop')

    world = os.path.join(
        get_package_share_directory('robot_gz'),
        'worlds',
        'tracked_robot_world.sdf'
    )
    urdf_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'tracked_robot.urdf'
    )
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    rviz_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'rviz', 
        'minimal_all.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_enabled),
    )


    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world],
        output='screen',
        condition=UnlessCondition(headless),
    )

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-v', '4', world],
        output='screen',
        condition=IfCondition(headless),
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )
    keyboard_teleop = Node(
        package='robot_minimal_control',
        executable='sim_keyboard_teleop',
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
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'base_laser',
        ],
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Liten delay så gz rekker å starte før bridge/node/rviz starter
    start_rest = TimerAction(
        period=1.0,
        actions=[bridge, robot_state_publisher, joint_state_publisher, lidar_static_tf, keyboard_teleop, rviz]
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
            'keyboard_teleop',
            default_value='true',
            description='Run local keyboard teleop window for sim.'
        ),
        SetEnvironmentVariable('ROS_USE_SIM_TIME', 'true'),
        gz_gui,
        gz_headless,
        start_rest,
        autostart_sim,
    ])
