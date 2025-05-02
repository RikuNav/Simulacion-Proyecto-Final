import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('mlr_nav2_puzzlebot')

    # Get the path to the node resources
    world_path = os.path.join(package_share_dir, 'worlds', 'puzzlebot_world.world')

    # Robot Description
    robot_file = 'puzzlebot.urdf'
    urdf = os.path.join(
        package_share_dir, 
        'urdf', 
        robot_file
    )

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
        arguments=[urdf],
    )

    world_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= [
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    # Nodes definition
    gz_process = ExecuteProcess(cmd=['gz', 'sim', world_path],
                                output='screen',)

    gz_service_spawn_puzzlebot = ExecuteProcess(cmd=['gz', 'service', '-s', '/world/default/create', 
                                                                        '--reqtype', 'gz.msgs.EntityFactory',
                                                                        '--reptype', 'gz.msgs.Boolean',
                                                                        '--timeout', '1000',
                                                                        '--req', f'sdf_filename: "file://{package_share_dir}/models/puzzlebot.sdf" name: "puzzlebot"'],
                                output='screen',)
    
    joint_state_publisher_node = Node(
        package='mlr_nav2_puzzlebot',
        executable='movement',
        output='screen',
    )
    
    # Declare argument
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='nav.rviz',
        description='RViz config file name to load'
    )

    # Define the RViz node with LaunchConfiguration directly for the file path
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mlr_nav2_puzzlebot'),
        'rviz_config',
        LaunchConfiguration('rviz_config_file')
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    l_d = LaunchDescription([gz_process,
                            gz_service_spawn_puzzlebot,
                            robot_state_publisher_node,
                            world_node,
                            joint_state_publisher_node,
                            declare_rviz_arg,
                            rviz_node,
                            ])

    return l_d


