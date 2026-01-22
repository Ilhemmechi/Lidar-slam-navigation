"""ROS2 launch for navigation"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    nav2_node = Node(
        package='nav2_bringup',
        executable='nav2_bringup_launch',
        name='nav2'
    )

    ld.add_action(nav2_node)

    return ld
