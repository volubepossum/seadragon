from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    config_file = os.path.join(pkg_dir, 'config', 'motor_config.yml')
    mapping_file = os.path.join(pkg_dir, 'config', 'thrust_mapping.csv')
    
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            parameters=[
                config_file,
                {'thrust_mapping': mapping_file}
            ],
            output='screen',
        ),
    ])