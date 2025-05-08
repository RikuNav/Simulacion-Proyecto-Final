import os

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
    sim_time = 'true'
    # Robot's initial position
    pos_x = '0.0'
    pos_y = '0.0'
    pos_th = '0.0'
    op_mode = 'map'

    # Launch arguments
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Use simulated time')
    declare_x_arg = DeclareLaunchArgument('x', default_value=pos_x, description='X position of the robot')
    declare_y_arg = DeclareLaunchArgument('y', default_value=pos_y, description='Y position of the robot')
    declare_theta_arg = DeclareLaunchArgument('theta', default_value=pos_th, description='angle of the robot')
    declare_mode_arg = DeclareLaunchArgument('mode', default_value=op_mode, description='Mode of operation (map or nav)')
    declare_map_dir = DeclareLaunchArgument('map', 
                                    default_value=
                                    os.path.join(
                                        get_package_share_directory('mlr_nav2_puzzlebot'),
                                        'maps',
                                        'puzzlebot_map.yaml'),
                                    description='Full path to map file to load')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    theta = LaunchConfiguration('theta')
    mode = LaunchConfiguration('mode')
    map_directory = LaunchConfiguration('map')

    # Get the package share directory
    package_share_dir = get_package_share_directory('mlr_nav2_puzzlebot')

    # Get the path to the node resources
    world_path = os.path.join(package_share_dir, 'worlds', 'puzzlebot_world.world')

    # Paths for world and robot description
    robot_path = os.path.join(package_share_dir, 'urdf', 'puzzlebot.xacro')

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
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '1', '--y', '1', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    robot_state_publisher_node = Node(package="robot_state_publisher",
                                        executable="robot_state_publisher",
                                        output="screen",
                                        parameters=[{
                                            "robot_description": ParameterValue(robot_description, value_type=str),
                                            "use_sim_time": use_sim_time,
                                        }],)

    gz_process = ExecuteProcess(cmd=['gz', 'sim', world_path, '-v', '4'],
                                output='screen',)
    
    gz_spawn_puzzlebot_node = Node(package="ros_gz_sim",
                                    executable="create",
                                    arguments=[
                                        "-name", "puzzlebot",
                                        "-topic", "robot_description",
                                        "-x", x, "-y", y, "-Y", theta,
                                    ],
                                    output="screen",)

    # Bridge ROS topics and Gazebo messages for establishing communication
    ros_gz_bridge_config_file_path = os.path.join(package_share_dir, 'config', f"puzzlebot_bridge.yaml")
    gz_bridge_node = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        parameters=[{'config_file': ros_gz_bridge_config_file_path,}],
                        output='screen')
    
    # Include the navigation stack and SLAM tool
    # This will only be activated when the mode is set to "map"
    navigation_stack_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "map"']))
    )

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
            'map': map_directory,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "nav"']))
    )

    # # Declare argument
    # declare_rviz_arg = DeclareLaunchArgument(
    #     'rviz_config_file',
    #     default_value='nav.rviz',
    #     description='RViz config file name to load'
    # )

    # # Define the RViz node with LaunchConfiguration directly for the file path
    # rviz_config_file = LaunchConfiguration('rviz_config_file')
    # rviz_config_path = PathJoinSubstitution([
    #     FindPackageShare('mlr_nav2_puzzlebot'),
    #     'rviz_config',
    #     LaunchConfiguration('rviz_config_file')
    # ])

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path]
    # )
    
    l_d = LaunchDescription([declare_sim_time_arg,
                            declare_x_arg,
                            declare_y_arg,
                            declare_theta_arg,
                            declare_mode_arg,
                            declare_map_dir,
                            set_gazebo_resources,
                            set_gazebo_plugins,
                            map_odom_transform_node,
                            robot_state_publisher_node,
                            gz_process,
                            gz_spawn_puzzlebot_node,
                            gz_bridge_node,
                            navigation_stack_node,
                            slam_tool_node,
                            navigation_node])

    return l_d


