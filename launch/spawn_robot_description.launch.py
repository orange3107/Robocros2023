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

def generate_launch_description():
    # pkg_share = get_package_share_directory('ar_ht_gazebo')
    # world_file_name = 'cafe.world'
    # world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    # gazebo_models_path = os.path.join(pkg_share, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    robot_name_in_model = 'prius'
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'
    urdf = os.path.join(get_package_share_directory('robocross2023'), 'models/autocar', 'model.sdf')
    
    spawn_robot = Node(
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
    )
     
        
    return LaunchDescription([
        spawn_robot,
    ])
