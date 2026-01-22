#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Joystick control
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),
        
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': -0.3,
                'scale_angular.yaw': 0.8,
                'enable_button': 0,
                'enable_turbo_button': 1,
                'scale_linear_turbo.x': -0.6,
            }],
            remappings=[('cmd_vel', '/cmd_vel')]
        ),
        
        # TF links for camera frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_cam_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                      'left_camera_optical_link', 'my_robot/base_footprint/left_camera']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_cam_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                      'right_camera_optical_link', 'my_robot/base_footprint/right_camera']
        ),
        
        # Stereo Odometry Node - Generates visual odometry from stereo
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'publish_null_transforms_apart_from_base': False,
                'approx_sync': True,
                'queue_size': 30,
                'Odom/Strategy': '0',  # Stereo
                'Vis/MinInliers': '12',
                'Vis/InlierDistance': '0.1',
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_raw'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/image_rect', '/camera/right/image_raw'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', '/visual_odom'),  # Publishes visual odometry
            ]
        ),
        
        # RTAB-Map SLAM - Now subscribes to odom and visual_odom
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,
                'subscribe_rgbd': False,
                'subscribe_odom': True,
                'subscribe_odom_info': False,  # Disable - let RTAB-Map use plain odom topic
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'wait_for_transform': 0.2,
                'approx_sync': True,
                'sync_queue_size': 30,
                
                # TF publishing
                'tf_delay': 0.05,
                'odom_tf_angular_variance': 0.001,
                'odom_tf_linear_variance': 0.001,
                
                # Memory
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # Stereo
                'Stereo/MaxDisparity': '128.0',
                
                # Grid
                'Grid/FromDepth': 'true',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '5.0',
                'RGBD/CreateOccupancyGrid': 'true',
                
                # Updates
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.1',
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Features
                'Vis/MinInliers': '12',
                'Kp/DetectorStrategy': '0',
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_raw'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/image_rect', '/camera/right/image_raw'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', '/visual_odom'),  # Use visual odometry from stereo_odometry node
            ],
            arguments=['--delete_db_on_start']
        ),
        
        # Map assembler
        Node(
            package='rtabmap_util',
            executable='map_assembler',
            name='map_assembler',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                ('grid_map', '/map'),
            ]
        ),
        
        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'approx_sync': True,
                'sync_queue_size': 30,
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_raw'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/image_rect', '/camera/right/image_raw'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', '/visual_odom'),
            ]
        ),
    ])