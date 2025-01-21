from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Model estimator node - creates state space model
        Node(
            package='ss_controller',
            executable='model_estimator_node',
            name='model_estimator',
            output='screen'
        ),
        
        # Observer maker node - designs state observer
        Node(
            package='ss_controller',
            executable='observer_maker_node',
            name='observer_maker',
            output='screen'
        ),
        
        # Controller maker node - designs LQR controller
        Node(
            package='ss_controller',
            executable='controller_maker_node',
            name='controller_maker',
            output='screen'
        ),
        
        # Controller node - implements control law
        Node(
            package='ss_controller',
            executable='controller_node',
            name='controller',
            output='screen'
        )
    ])