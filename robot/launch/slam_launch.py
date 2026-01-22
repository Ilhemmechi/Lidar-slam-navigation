"""ROS2 launch for SLAM"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    lio_sam_node = Node(
        package='lio_sam',
        executable='lio_sam_node',
        name='lio_sam'
    )

    ld.add_action(lio_sam_node)

    return ld
