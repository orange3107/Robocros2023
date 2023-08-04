#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command

def generate_launch_description():
    # pkg_share = get_package_share_directory('ar_ht_gazebo')
    world_file_name = 'world_robocross.world'
    # world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    # gazebo_models_path = os.path.join(pkg_share, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    robot_name_in_model = 'robocross'
    spawn_x_val = '8.25'
    spawn_y_val = '-0.8'
    spawn_z_val = '0'
    spawn_yaw_val = '1.57'
    
    descpkg = 'robocross2023'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    urdf = os.path.join(get_package_share_directory(descpkg),'models/autocar', 'autocar.xacro')
    
    world = os.path.join(get_package_share_directory('robocross2023'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('robocross2023'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    sdf = os.path.join(get_package_share_directory('robocross2023'), 'models/autocar/', 'model.sdf')
    
    map_file =os.path.join(get_package_share_directory('robocross2023'),
                         'maps', 'my_map.yaml')
                         
    return LaunchDescription([
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ),

    Node(
        package='robocross2023',
        executable='transforms.py',
        name='transforms',
    ),

    Node(
        package='robocross2023',
        executable='car_in_global_map.py',
        name='car_in_global_map',
    ),

    Node(
        package='robocross2023',
        executable='create_local_cost_map_old.py',
        name='create_local_cost_map_old',
    ),

    

    
        
        
    ])