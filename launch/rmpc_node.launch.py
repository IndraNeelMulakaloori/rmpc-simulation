import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rmpc = get_package_share_directory('rmpc_f1tenth')

    config_file = os.path.join(pkg_rmpc, 'config', 'config.yaml')
    rviz_config = os.path.join(pkg_rmpc, 'rviz', 'map.rviz') 


    # RMPC Controller Node (remapped to gym bridge topics)
    rmpc_node = Node(
        package='rmpc_f1tenth',
        executable='rmpc_node',
        name='rmpc_node',
        output='screen',
        parameters=[config_file],
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        # Delay RMPC node & RViz by 2.0s to ensure map, /scan, and ego odom TF frames exist
        TimerAction(
            period=2.0,
            actions=[rmpc_node, rviz_node]
        )
    ])