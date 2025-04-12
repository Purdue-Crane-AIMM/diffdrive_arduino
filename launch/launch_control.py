from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffdrive_arduino',
            executable='diffdrive_arduino',
            name='diffdrive_arduino',
            parameters=[os.path.join(
                get_package_share_directory("diffdrive_arduino"),'config','params.yaml'
            )],
            output='screen'
        ),
    ])