from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    
    # RTABMap SLAM node (le bon package)
    rtabmap_slam = Node(
        package='rtabmap_slam',  # Utilise rtabmap_slam au lieu de rtabmap_ros
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': use_sim_time,
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            
            # Paramètres SLAM
            'Reg/Strategy': '1',  # ICP
            'Grid/3D': True,
            'Grid/RayTracing': True,
            'Grid/RangeMax': '20.0',
            'Mem/LaserScanDownsampleStepSize': '2'
        }],
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom')
        ],
        arguments=['-d']
    )
    
    # RTABMapViz (optionnel)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint'
        }],
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_slam,
        rtabmap_viz
    ])
