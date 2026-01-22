import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    # ICP Odometry
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
            'expected_update_rate': 15.0,
            'deskewing': False,
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom_icp')
        ]
    )
    
    # RTABMap SLAM
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': use_sim_time,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            
            # Strategy
            'Reg/Strategy': '1',  # ICP
            'Reg/Force3DoF': 'false',
            'RGBD/NeighborLinkRefining': 'false',
            'Grid/FromDepth': 'false',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/RangeMax': '20.0',
            'Mem/LaserScanDownsampleStepSize': '2',
            
            # ICP parameters
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.15',
        }],
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom_icp')
        ],
        arguments=['-d']
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        icp_odometry,
        rtabmap_slam
    ])
