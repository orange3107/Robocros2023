#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

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
    spawn_x_val = '14.0'
    spawn_y_val = '10.0'
    spawn_z_val = '0.2'
    spawn_yaw_val = '1.57'
    
    descpkg = 'my_box_bot_gazebo'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    urdf = os.path.join(get_package_share_directory(descpkg),'models/autocar', 'autocar.xacro')
    
    world = os.path.join(get_package_share_directory('my_box_bot_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('my_box_bot_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    sdf = os.path.join(get_package_share_directory('my_box_bot_gazebo'), 'models/autocar/', 'model.sdf')
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        
        
        
        
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', sdf])}],
        output="screen"
    ),
    
    Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name= 'spawn_entity', 
        output= 'screen',
        arguments=['-entity', robot_name_in_model, 
                '-topic', '/robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val]
    ),
        
      
       Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[sdf]
        ),
        
    ])
