import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_generator',
            executable='trajectory_generator_exe',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])