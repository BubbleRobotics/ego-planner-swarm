import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
# from launch.substitutions import EqualsSubstitution, NotEqualsSubstitution
from launch.substitutions import PythonExpression

def generate_launch_description():
    # LaunchConfigurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    init_x = LaunchConfiguration('init_x_', default=0.0)
    init_y = LaunchConfiguration('init_y_', default=0.0)
    init_z = LaunchConfiguration('init_z_', default=0.0)
    obj_num = LaunchConfiguration('obj_num', default=1)
    map_size_x_ = LaunchConfiguration('map_size_x_', default=10.0)
    map_size_y_ = LaunchConfiguration('map_size_y_', default=10.0)
    map_size_z_ = LaunchConfiguration('map_size_z_', default=5.0)
    c_num = LaunchConfiguration('c_num', default=5)
    p_num = LaunchConfiguration('p_num', default=20)
    min_dist = LaunchConfiguration('min_dist', default=1.0)
    odometry_topic = LaunchConfiguration('odometry_topic', default='odometry')
    drone_id = LaunchConfiguration('drone_id', default=0)

    # DeclareLaunchArguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use Simulation Time')
    init_x_arg = DeclareLaunchArgument('init_x_', default_value=init_x, description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value=init_y, description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value=init_z, description='Initial Z position')
    obj_num_arg = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x_, description='Map size X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y_, description='Map size Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z_, description='Map size Z')
    c_num_arg = DeclareLaunchArgument('c_num', default_value=c_num, description='Circle number')
    p_num_arg = DeclareLaunchArgument('p_num', default_value=p_num, description='Polygon number')
    min_dist_arg = DeclareLaunchArgument('min_dist', default_value=min_dist, description='Minimum distance')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
        
    use_dynamic = LaunchConfiguration('use_dynamic', default=True)  
    use_dynamic_arg = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')


    # dynamic

    

    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['ego_odom_visualization'],
        output='screen',
        remappings=[
            ('odom', ['mavros/odometry/out']),
            ('robot', ['ego_vis/robot']),
            ('path', ['ego_vis/path']),
            ('time_gap', ['ego_vis/time_gap']),
            ('goal', ['ego_vis/goal']),
            # ('pose', ['ego_vis/pose']),
            # ('velocity', ['ego_vis/velocity']),
            # ('covariance', ['ego_vis/covariance']),
            # ('covariance_velocity', ['ego_vis/covariance_velocity']),
            # ('trajectory', ['ego_vis/trajectory']),
            # ('sensor', ['ego_vis/sensor']),
            # ('height', ['ego_vis/height']),
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 0.025},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
    )
    
    camera_file = os.path.join( 
        get_package_share_directory('local_sensing'), 
        'config', 
        'camera.yaml' 
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(init_x_arg)
    ld.add_action(init_y_arg)
    ld.add_action(init_z_arg)
    ld.add_action(obj_num_arg)
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(c_num_arg)
    ld.add_action(p_num_arg)
    ld.add_action(min_dist_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(drone_id_arg)
    
    ld.add_action(use_dynamic_arg)

    
    ld.add_action(odom_visualization_node)

    return ld