import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('mlr_nav2_puzzlebot')

    # Get the path to the node resources
    world_path = os.path.join(package_share_dir, 'worlds', 'puzzlebot_world.world')

    # Nodes definition
    gz_process = ExecuteProcess(cmd=['gz', 'sim', world_path],
                                output='screen',)

    gz_service_spawn_puzzlebot = ExecuteProcess(cmd=['gz', 'service', '-s', '/world/default/create', 
                                                    '--reqtype', 'gz.msgs.EntityFactory',
                                                    '--reptype', 'gz.msgs.Boolean',
                                                    '--timeout', '1000',
                                                    '--req', f'sdf_filename: "file://{package_share_dir}/models/puzzlebot.sdf" name: "puzzlebot"'],
                                                output='screen',)
    
    # Set Gazebo environment variables
    set_gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(package_share_dir)
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
    
    l_d = LaunchDescription([set_gazebo_resources,
                            gz_process,
                            gz_service_spawn_puzzlebot,])

    return l_d


