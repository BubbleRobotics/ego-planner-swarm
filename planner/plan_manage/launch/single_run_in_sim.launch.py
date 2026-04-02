import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Definition of LaunchConfiguration parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    max_vel = LaunchConfiguration('max_vel', default=0.4)
    max_acc = LaunchConfiguration('max_acc', default=0.2)

    map_size_x = LaunchConfiguration('map_size_x', default = 40.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 40.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 13.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'odometry')
    point_clicked_z_up_ = LaunchConfiguration('point_clicked_z_up', default=-1.0)
    map_reset_timer = LaunchConfiguration('grid_map/occ_ttl_sec', default='5.0')
    obstacle_inflation = LaunchConfiguration('obstacles_inflation', default='0.05')
    # Declare global parameters
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time',default_value=use_sim_time, description='Using simulation / ROS time')
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    max_vel_cmd = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_cmd = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    point_clicked_z_up_cmd = DeclareLaunchArgument('point_clicked_z_up', default_value=point_clicked_z_up_, description='Default z-value (up) for rviz clicked points (new goal of planner)')
    # Map properties and whether to use dynamic simulation
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    map_reset_timer_cmd = DeclareLaunchArgument('grid_map/occ_ttl_sec', default_value=map_reset_timer, description='Map reset timer in seconds')
    obstacles_inflation_cmd = DeclareLaunchArgument('obstacles_inflation', default_value=obstacle_inflation, description='Obstacles inflation distance')

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.05},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'camera_d455/depth/image_raw/points',
            
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': max_vel,
            'max_acc': max_acc,
            'planning_horizon': str(5.0),
            'use_distinctive_trajs': 'True',
            'flight_type': str(1),
            'point_num': str(4),

            'point0_x': str(6.5),
            'point0_y': str(6.5),
            'point0_z': str(-3.0),
            
            'point1_x': str(-6.5),
            'point1_y': str(-6.5),
            'point1_z': str(-3.0),
            
            'point2_x': str(7.0),
            'point2_y': str(0.0),
            'point2_z': str(-3.65),
            
            'point3_x': str(0.0),
            'point3_y': str(7.0),
            'point3_z': str(-3.0),
            
            'point4_x': str(2.5),
            'point4_y': str(-7.0),
            'point4_z': str(-5.0),
            'point_clicked_z_up': point_clicked_z_up_,
            'grid_map/occ_ttl_sec': map_reset_timer,
            'obstacles_inflation': obstacle_inflation,
            
        }.items()
    )
    # FOR PILOT
    """'point0_x': str(10.93),
            'point0_y': str(13.56),
            'point0_z': str(-5.07),
            
            'point1_x': str(10.91),
            'point1_y': str(13.53),
            'point1_z': str(-3.65),
            
            'point2_x': str(10.69),
            'point2_y': str(12.63),
            'point2_z': str(-3.65),
            
            'point3_x': str(-8.0),
            'point3_y': str(-8.0),
            'point3_z': str(-2.0),
            
            'point4_x': str(4.0),
            'point4_y': str(0.0),
            'point4_z': str(-2.0),"""
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['ego_traj_server'],
        output='screen',
        remappings=[
            ('cmd_vel_body', ['adaptive_integral_terminal_sliding_mode_controller/reference']),
            ('planning/bspline', ['ego_planner/bspline'])
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # Include simulator 
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(0.0),
            'init_y_': str(0.0),
            'init_z_': str(-2.0),
            'odometry_topic': odom_topic
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(max_vel_cmd)
    ld.add_action(max_acc_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(map_reset_timer_cmd)
    ld.add_action(obstacles_inflation_cmd)
    ld.add_action(point_clicked_z_up_cmd)
    # Add Map Generator node
    ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld