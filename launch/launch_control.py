from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffdrive_arduino',
            executable='diffdrive_arduino',
            name='diffdrive_arduino',
            parameters=['/path/to/params.yaml'],
            output='screen'
        ),
    ])