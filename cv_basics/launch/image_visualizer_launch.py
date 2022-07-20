from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_basics',
            executable='img_analyser',
            name='analyser'
        ),
        Node(
            package='cv_basics',
            executable='img_visualizer',
            name='visualizer'
        )
    ])