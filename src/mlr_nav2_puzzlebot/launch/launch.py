import os
import transforms3d as t3d

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
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

    initial_x = -1.2
    initial_y = 1.2
    initial_theta = -1.57

    # Node parameters default values
    sim_time = 'true'
    op_mode = 'map'

    # Launch arguments
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Use simulated time')
    declare_mode_arg = DeclareLaunchArgument('mode', default_value=op_mode, description='Mode of operation (map or nav)')

    # Get launch params values
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')

    # Get the package share directory
    package_share_dir = get_package_share_directory('mlr_nav2_puzzlebot')
    
    # Get the path to the node resources
    world_path = os.path.join(package_share_dir, 'worlds', world_filename)

    # Get the path to map
    map_path = os.path.join(package_share_dir, 'maps', map_filename)

    # Get the path to nav2 params
    nav2_params_path = os.path.join(package_share_dir, 'config', param_filename)

    # Get the path to ros_gz_bridge config
    ros_gz_bridge_config_path = os.path.join(package_share_dir, 'config', ros_gz_bridge_config_filename)

    # Path for robot xacro
    robot_path = os.path.join(package_share_dir, 'urdf', robot_xacro_filename)
    
    # Robot description
    robot_description = Command(['xacro ', str(robot_path),
                                ' camera_frame:=', 'camera_link_optical',
                                ' lidar_frame:=', 'laser_frame',
                                ' tof_frame:=', 'tof_link'])
    
    # Set Gazebo environment variables
    set_gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(package_share_dir)
    )

    set_gazebo_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(package_share_dir, 'plugins')
    )

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
                                            "use_sim_time": use_sim_time,
                                        }],)

    # Gz world and puzzlebot launch
    gz_process = ExecuteProcess(cmd=['gz', 'sim', world_path, '-r'],
                                output='screen',)
    
    gz_spawn_puzzlebot_node = Node(package="ros_gz_sim",
                                    executable="create",
                                    arguments=[
                                        "-name", "puzzlebot",
                                        "-topic", "robot_description",
                                        "-x", str(initial_x), "-y", str(initial_y), "-Y", str(initial_theta),
                                    ],
                                    output="screen",)

    # Change gz camera view
    q = t3d.euler.euler2quat(0.0, 1.5708, 1.5708)
    gz_camera_process = ExecuteProcess(cmd=['gz', 'service', '-s', '/gui/move_to/pose', 
                                            '--reqtype', 'gz.msgs.GUICamera',
                                            '--reptype', 'gz.msgs.Boolean',
                                            '--timeout', '10000',
                                            '--req', f'pose: {{position: {{x: 0.0, y: 0.0, z: 3.0}} orientation: {{x: {q[1]}, y: {q[2]}, z: {q[3]}, w: {q[0]}}}}}'],
                                output='screen',)

    # Bridge ROS topics and Gazebo messages for establishing communication
    gz_bridge_node = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        parameters=[{'config_file': ros_gz_bridge_config_path,}],
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
            'use_sim_time': use_sim_time
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
    
    l_d = LaunchDescription([declare_sim_time_arg,
                            declare_mode_arg,
                            set_gazebo_resources,
                            set_gazebo_plugins,
                            map_odom_transform_node,
                            robot_state_publisher_node,
                            gz_process,
                            gz_spawn_puzzlebot_node,
                            gz_camera_process,
                            gz_bridge_node,
                            navigation_stack_node,
                            slam_tool_node,
                            navigation_node])

    return l_d


