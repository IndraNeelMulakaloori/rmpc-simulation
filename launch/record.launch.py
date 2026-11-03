import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rmpc_f1tenth')
    
    # Path to YAML parameters
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')

    ## Launch the RMPC Node
    rmpc_node = Node(
        package='rmpc_f1tenth',
        executable='rmpc_node',
        name='rmpc_node',
        output='screen',
        parameters=[config_file]
    )

    # Timestamped bag directory under ~/rmpc_f1tenth_telemetry/
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.join(
        os.path.expanduser('~'), 'rmpc_f1tenth_telemetry', f'rmpc_run_{timestamp}'
    )

    # ROS 2 Bag Recording Process
    # Record only the lightweight telemetry topics needed for offline analysis
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/ego_racecar/odom',
            '/drive',
            # Uncomment if you track opponent car or scan topics:
            # '/opp_racecar/odom',
            '-o', bag_dir,
        ],
        output='screen'
    )

    return LaunchDescription([
        rmpc_node,
        bag_record
    ])