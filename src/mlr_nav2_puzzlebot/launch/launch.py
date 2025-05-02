import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


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
    
    l_d = LaunchDescription([gz_process,
                            gz_service_spawn_puzzlebot])

    return l_d


