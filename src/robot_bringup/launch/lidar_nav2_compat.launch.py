from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    product_name = LaunchConfiguration('product_name')
    port_name = LaunchConfiguration('port_name')
    port_baudrate = LaunchConfiguration('port_baudrate')
    topic_name = LaunchConfiguration('topic_name')
    frame_id = LaunchConfiguration('frame_id')

    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')

    base_frame = LaunchConfiguration('base_frame')

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar',
        output='screen',
        parameters=[
            {'product_name': product_name},
            {'topic_name': topic_name},
            {'frame_id': frame_id},
            {'port_name': port_name},
            {'port_baudrate': port_baudrate},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ],
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=[
            '--x', tf_x,
            '--y', tf_y,
            '--z', tf_z,
            '--roll', tf_roll,
            '--pitch', tf_pitch,
            '--yaw', tf_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', frame_id,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_LD06'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('port_baudrate', default_value='230400'),
        DeclareLaunchArgument('topic_name', default_value='/lidar'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('base_frame', default_value='chassis'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='0.18'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),
        ldlidar_node,
        lidar_static_tf,
    ])
