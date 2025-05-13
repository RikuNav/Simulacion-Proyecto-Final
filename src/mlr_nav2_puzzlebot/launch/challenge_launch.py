import os
import transforms3d as t3d
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Node variables
    map_filename = 'puzzlebot_map.yaml'
    param_filename = 'nav2_config.yaml'
    world_filename = 'puzzlebot_world.world'
    robot_xacro_filename = 'puzzlebot.xacro'
    ros_gz_bridge_config_filename = 'puzzlebot_bridge.yaml'
    puzzlebot_config_filename = 'puzzlebot_nodes_params.yaml'
    global_params_filename = 'puzzlebot_global_params.yaml'

    # Node parameters default values
    op_mode = 'map'
    sim_time = 'false'

    # Launch arguments
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Use simulated time')
    declare_mode_arg = DeclareLaunchArgument('mode', default_value=op_mode, description='Mode of operation (map or nav)')

    # Get launch params values
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package share directory
    package_share_dir = get_package_share_directory('mlr_nav2_puzzlebot')
    
    # Get the path to the node resources
    world_path = os.path.join(package_share_dir, 'worlds', world_filename)

    # Get the path to map
    map_path = os.path.join(package_share_dir, 'maps', map_filename)

    # Get the path to nav2 params
    nav2_params_path = os.path.join(package_share_dir, 'params', param_filename)

    # Get the path to ros_gz_bridge config
    ros_gz_bridge_config_path = os.path.join(package_share_dir, 'config', ros_gz_bridge_config_filename)

    # Get the path to puzzlebot config
    puzzlebot_config_path = os.path.join(package_share_dir, 'config', puzzlebot_config_filename)

    # Get the global parameters config
    global_params_path = os.path.join(package_share_dir, 'config', global_params_filename)

    # Path for robot xacro
    robot_path = os.path.join(package_share_dir, 'urdf', robot_xacro_filename)
    
    slam_toolbox_path = os.path.join(get_package_share_directory('slam_toolbox'), 'params', 'slam_toolbox_config.yaml')

    rviz_map = os.path.join(package_share_dir, 'rviz', 'map.rviz')
    rviz_nav = os.path.join(package_share_dir, 'rviz', 'nav.rviz')

    # Robot description
    robot_description = Command(['xacro ', str(robot_path),
                                ' camera_frame:=', 'camera_link_optical',
                                ' lidar_frame:=', 'laser_frame',
                                ' tof_frame:=', 'tof_link'])
    
    # Extract initial position from puzzlebot_node_params.yaml
    with open(puzzlebot_config_path, 'r') as puzzlebot_config:
        puzzlebot_config_dict = yaml.safe_load(puzzlebot_config)

    initial_x = puzzlebot_config_dict['puzzlebot_localization_node']['ros__parameters']['initial_pose']['x']
    initial_y = puzzlebot_config_dict['puzzlebot_localization_node']['ros__parameters']['initial_pose']['y']
    initial_theta = puzzlebot_config_dict['puzzlebot_localization_node']['ros__parameters']['initial_pose']['theta']

    # Extract the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)
    
    # Nodes definition
    # Static map to odom transform
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '0', '--y', '0', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    # Robot state publisher
    robot_state_publisher_node = Node(package="robot_state_publisher",
                                        executable="robot_state_publisher",
                                        output="screen",
                                        parameters=[{
                                            "robot_description": ParameterValue(robot_description, value_type=str),
                                        }],)

    # Puzzlebot localization node
    puzzlebot_localization_node = Node(package='mlr_nav2_puzzlebot',
                                       executable='puzzlebot_localization',
                                       parameters=[puzzlebot_config_path, global_params],
                                       output='screen')
    
    # Puzzlebot joint state publisher node
    puzzlebot_joint_state_publisher = Node(package='mlr_nav2_puzzlebot',
                                            executable='puzzlebot_joint_state_publisher',
                                            parameters=[global_params],
                                            output='screen')
            
    # Navigation stack
    navigation_stack_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "map"']))
    )

    # SLAM toolbox
    slam_tool_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_toolbox_path,
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "map"']))
    )
    
    # Include the navigation node
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "nav"']))
    )

    rviz_node_map = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_map],
        condition=IfCondition(PythonExpression(['"', mode, '" == "map"']))
    )

    rviz_node_nav = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_nav],
        condition=IfCondition(PythonExpression(['"', mode, '" == "nav"']))
    )
    
    l_d = LaunchDescription([declare_mode_arg,
                            declare_sim_time_arg,
                            map_odom_transform_node,
                            robot_state_publisher_node,
                            puzzlebot_localization_node,
                            puzzlebot_joint_state_publisher,
                            navigation_stack_node,
                            slam_tool_node,
                            navigation_node,
                            rviz_node_map,
                            rviz_node_nav])

    return l_d