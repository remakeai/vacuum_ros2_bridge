"""Launch file for vacuum_ros2_bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.143',
        description='IP address of the vacuum robot running SangamIO'
    )

    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='5555',
        description='TCP port of SangamIO'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Robot base frame ID'
    )

    odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odometry frame ID'
    )

    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser',
        description='LiDAR frame ID'
    )

    # Bridge node
    bridge_node = Node(
        package='vacuum_ros2_bridge',
        executable='bridge_node.py',
        name='vacuum_bridge',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        frame_id_arg,
        odom_frame_id_arg,
        lidar_frame_id_arg,
        bridge_node,
    ])
