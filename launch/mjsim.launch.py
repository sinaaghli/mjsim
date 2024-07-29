from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('mjsim')
    param_file = os.path.join(package_share_directory, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='mjsim',
            executable='mjsim',
            name='mjsim',
            output='screen',
            parameters=[param_file]
        )
    ])
