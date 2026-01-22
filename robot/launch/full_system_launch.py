"""Full system launch"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    ld = LaunchDescription()

    here = os.path.dirname(__file__)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'slam_launch.py'))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'navigation_launch.py'))
    )

    ld.add_action(slam)
    ld.add_action(navigation)

    return ld
