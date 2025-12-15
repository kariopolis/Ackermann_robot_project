from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'ackermann_drive'
    config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'config'
    )
    #Sensor fusion parameters file location
    ekf_config = os.path.join(config_dir, 'ekf.yaml')

    return LaunchDescription([
 
        Node(
            package='ackermann_drive',
            executable='odom',
            name='odometry_publisher',
            output='screen'
        ),

        Node(
            package='ackermann_drive',
            executable='js',
            name='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
