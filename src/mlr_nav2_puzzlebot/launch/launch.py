from launch import LaunchDescription
from launch.actions import ExecuteProcess

#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch_ros.actions import Node

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.events import Shutdown


def generate_launch_description():

    base_path = get_package_share_directory('mlr_nav2_puzzlebot')


    rviz_config = os.path.join(base_path, 'rviz', 'name_rviz.rviz')
    
    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    on_exit=Shutdown(),
                    )
    
    world_path = os.path.join(base_path, 'worlds', 'reto.world')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ), 
        rviz_node
    ]) 


