import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # RTABMap parameters
    parameters = [{
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': False,
        
        # RTAB-Map's parameters
        'Reg/Strategy': '1',  # 0=Visual, 1=ICP, 2=Visual+ICP
        'Reg/Force3DoF': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'Grid/FromDepth': 'false',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '30.0',
        'Grid/3D': 'true',
        'Grid/RayTracing': 'true',
        'Mem/STMSize': '30',
        'Mem/LaserScanDownsampleStepSize': '1',
        
        # ICP parameters
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PM': 'true',
        'Icp/PMOutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.2',
        
        # Odom parameters  
        'Odom/Strategy': '0',
        'OdomF2M/ScanSubtractRadius': '0.1',
        'OdomF2M/MaxSize': '2000'
    }]
    
    # RTABMap node
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom'),
            ('imu', '/imu/imu_plugin/out')
        ],
        arguments=['-d']  # Delete database on start
    )
    
    # RTABMapViz
    rtabmapviz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=[
            ('scan_cloud', '/velodyne/gazebo_ros_laser_controller/out'),
            ('odom', '/odom'),
            ('imu', '/imu/imu_plugin/out')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        rtabmap_node,
        rtabmapviz_node
    ])
