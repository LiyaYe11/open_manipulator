from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='open_manipulator_x_bringup',
            executable='omy_hat_bringup_node',
            name='omy_hat_bringup_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
