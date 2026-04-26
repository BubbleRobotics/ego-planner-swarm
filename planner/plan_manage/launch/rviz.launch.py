import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time',default_value=use_sim_time, description='Using simulation / ROS time')

    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'launch', 'default.rviz')
    rviz_node = Node(
            package='rviz2', executable='rviz2', output='screen', name='ego_rviz',
            arguments=['--display-config', rviz_config_path],
            parameters=[
                {'use_sim_time': use_sim_time}
                ]
            )

    # Define LaunchDescription
    ld = LaunchDescription()

    # Add node
    ld.add_action(use_sim_time_cmd)
    ld.add_action(rviz_node)

    return ld