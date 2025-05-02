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
    
    l_d = LaunchDescription([gz_process,])

    return l_d


