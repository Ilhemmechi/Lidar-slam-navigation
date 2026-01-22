#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Default config file path
    default_params_file = PathJoinSubstitution([
        FindPackageShare('lio_sam'),
        'config',
        'params.yaml'
    ])
    
    # Get the actual parameter file path
    params_file_path = params_file.perform(context)
    if not os.path.exists(params_file_path):
        params_file_path = default_params_file.perform(context)
    
    # Verify params file exists
    if not os.path.exists(params_file_path):
        print(f"Error: Parameter file not found: {params_file_path}")
        return []
    
    print(f"Using parameter file: {params_file_path}")
    
    # Get RViz config file path
    rviz_config_path = rviz_config_file.perform(context)
    if not os.path.exists(rviz_config_path):
        print(f"Warning: RViz config file not found: {rviz_config_path}")
        print("RViz will start with default configuration")
        rviz_config_path = None
    
    # LIO-SAM nodes
    lio_sam_imageProjection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[params_file_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    lio_sam_featureExtraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[params_file_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    lio_sam_imuPreintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[params_file_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    lio_sam_mapOptimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[params_file_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if rviz_config_path else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    nodes = [
        lio_sam_imageProjection,
        lio_sam_featureExtraction,
        lio_sam_imuPreintegration,
        lio_sam_mapOptimization,
        rviz_node
    ]
    
    return nodes

def generate_launch_description():
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lio_sam'),
            'config',
            'params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lio_sam'),
            'config',
            'rviz.rviz'
        ]),
        description='Full path to the RViz config file to use'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        OpaqueFunction(function=launch_setup)
    ])
