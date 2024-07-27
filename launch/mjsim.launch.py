import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Function to set the robot_description parameter
    def set_robot_description(context, *args, **kwargs):
        # Get the path to the robot description file from the launch argument
        robot_description_path = LaunchConfiguration('robot_description_path').perform(context)
        urdf_content = ''

        # Check if a path to the robot description file is provided
        if robot_description_path:
            # If the file is a XACRO file, convert it to URDF
            if robot_description_path.endswith('.xacro'):
                doc = xacro.process_file(robot_description_path)
                urdf_content = doc.toprettyxml(indent='  ')
            else:
                # If the file is a URDF file, read its content
                with open(robot_description_path, 'r') as urdf_file:
                    urdf_content = urdf_file.read()

            # Log the action of setting the robot_description parameter
            return [
                LogInfo(msg=f'Setting robot_description from: {robot_description_path}'),
                # Node to set the robot_description parameter
                Node(
                    package='rclcpp_components',
                    executable='component_container_mt',
                    name='parameter_server',
                    output='screen',
                    parameters=[{'robot_description': urdf_content}]
                )
            ]
        # If no file path is provided, return an empty action list
        return []

    return LaunchDescription([
        # Declare the launch argument for the path to the robot description file
        DeclareLaunchArgument(
            'robot_description_path',
            default_value='',
            description='Path to the robot description file (URDF or XACRO)'
        ),
        # Call the function to set the robot_description parameter if a path is provided
        OpaqueFunction(function=set_robot_description),
        # Node to publish the robot state using the robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': LaunchConfiguration('robot_description_path')
            }]
        ),
        # Node to run the simulator, which uses the robot_description parameter
        Node(
            package='your_simulator_package',
            executable='your_simulator_node',
            name='simulator',
            output='screen',
            parameters=[{
                'robot_description': LaunchConfiguration('robot_description_path')
            }],
            remappings=[
                ('/joint_states', '/your_simulator_joint_states'),
                ('/joint_commands', '/your_simulator_joint_commands')
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
