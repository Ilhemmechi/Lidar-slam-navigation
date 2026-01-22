from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            name='base_to_footprint'
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_link', 'velodyne_link'],
            name='base_to_velodyne'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            name='base_to_imu'
        ),
    ])
