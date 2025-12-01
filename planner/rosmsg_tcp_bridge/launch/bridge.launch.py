from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define LaunchConfiguration
    drone_id = LaunchConfiguration('drone_id', default = 1)
    
    # Declare parameters
    drone_id_cmd = DeclareLaunchArgument(
        'drone_id',
        default_value= drone_id,
        description='ID of the drone'
    )

    # Node definition
    rosmsg_tcp_bridge_node = Node(
        package='rosmsg_tcp_bridge',
        executable='bridge_node',
        name=['bridge_node_', drone_id],  # Dynamically generate node names


        output='screen',
        parameters=[
            {'next_drone_ip': '127.0.0.1'},
            {'broadcast_ip': '127.0.0.255'},
            {'drone_id': drone_id},
            {'odom_max_freq': 70.0}
        ],
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('my_odom', '/vins_estimator/imu_propagate')
        ]
    )

    # Define LaunchDescription
    ld = LaunchDescription()

    # Add parameter declarations
    ld.add_action(drone_id_cmd)

    # Add node
    ld.add_action(rosmsg_tcp_bridge_node)

    return ld
