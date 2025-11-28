import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare path of the overriding parameter file
    default_param_file = LaunchConfiguration('default_param_file', 
                                            default=os.path.join(os.path.dirname(__file__), '..', 'config', 'default.yaml'))
    overlying_param_file = LaunchConfiguration('overlying_param_file')
    
    # Declare default parameter file path
    default_param_file_cmd = DeclareLaunchArgument(
        'default_param_file',
        default_value=default_param_file,
        description = 'Path to the default parameter file.'
    )
    
    overlying_param_file_cmd = DeclareLaunchArgument(
        'overlying_param_file',
        default_value = overlying_param_file,
        description = 'Path to the parameter file that can override the defaults.'
    )

    # Node definition
    ros_package_template_node = Node(
        package='ros_package_template',
        executable='ros_package_template',
        name='ros_package_template',
        output='screen',
        parameters=[
            default_param_file,
            overlying_param_file_cmd
        ]
    )

    # Launch description
    ld = LaunchDescription()

    # Add declared parameters
    ld.add_action(default_param_file_cmd)
    ld.add_action(overlying_param_file_cmd)

    # Add node
    ld.add_action(ros_package_template_node)

    return ld
