import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Set the path to different files and folders.
    pkg_name = 'mon_urdf'
    # Nom de fichier correspondant au fichier existant
    file_subpath = 'urdf/my_robot.urdf.xacro'
    
    # Set the path to the URDF file
    urdf_file = os.path.join(
        FindPackageShare(package=pkg_name).find(pkg_name),
        file_subpath)
    
    # MODIFIÉ: Chemin vers votre monde personnalisé
    custom_world_path = os.path.expanduser('~/mon_world.custom')
    
    # Set the path to the Gazebo world file (défaut si le monde custom n'existe pas)
    default_world_file_name = 'empty.world'
    default_world_path = os.path.join(
        FindPackageShare('gazebo_ros').find('gazebo_ros'),
        'worlds', default_world_file_name)
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
        
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
        
    # MODIFIÉ: Argument world avec votre monde personnalisé par défaut
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=custom_world_path if os.path.exists(custom_world_path) else default_world_path,
        description='Full path to the world model file to load')
    
    # CORRECTION: Utilisation correcte de ParameterValue pour ROS2
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher avec publish_frequency pour les transformations statiques
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0  # AJOUTÉ: Force la publication régulière des TF statiques
        }],
        output='screen'
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # CORRECTION: Commande de nettoyage plus robuste
    kill_gazebo_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f gzserver || true; pkill -f gzclient || true'],
        output='screen'
    )
    
    # Start Gazebo server avec votre monde personnalisé
    start_gazebo_server_cmd = TimerAction(
        period=2.0,  # Wait 2 seconds after killing processes
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch', 'gzserver.launch.py')),
                condition=IfCondition(use_simulator),
                launch_arguments={
                    'world': world,
                    'verbose': 'false',
                    'physics': 'ode',
                    'pause': 'false'
                }.items())
        ]
    )
    
    # Start Gazebo client with delay
    start_gazebo_client_cmd = TimerAction(
        period=5.0,  # Wait 5 seconds before starting client
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch', 'gzclient.launch.py')),
                condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
        ]
    )
    
    # CORRECTION: Spawn entity avec paramètres corrects
    spawn_entity_cmd = TimerAction(
        period=8.0,  # Wait 8 seconds before spawning
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_robot',
                    '-x', '0.0', 
                    '-y', '0.0', 
                    '-z', '0.2',  # Spawn légèrement plus haut
                    '-timeout', '60.0'  # Timeout réduit à 60 secondes
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
     # RViz2 for visualization (optional)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(FindPackageShare(pkg_name).find(pkg_name), 'config', 'robot.rviz')],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add cleanup and nodes in order
    ld.add_action(kill_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)  # Uncomment if you want RViz
    
    return ld
