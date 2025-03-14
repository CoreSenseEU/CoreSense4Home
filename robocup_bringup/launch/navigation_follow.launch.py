# Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('robocup_bringup')
    navigation_dir = get_package_share_directory('navigation_system')
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation_system.launch.py')
        ),
        launch_arguments={
            'rviz': 'True',
            'mode': 'amcl',
            'params_file': package_dir + '/config/carry_my_luggage/tiago_nav_params.yaml',
            'slam_params_file': package_dir +
                    '/config/carry_my_luggage/tiago_nav_follow_params.yaml',
            'map': os.path.join(
                                package_dir,
                                'maps',
                                'lab_test.yaml'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(navigation)
    return ld
