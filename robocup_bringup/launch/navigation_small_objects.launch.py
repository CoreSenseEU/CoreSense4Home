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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('robocup_bringup')
    small_objects_dir = get_package_share_directory('small_objects_detector')
    pcl_to_laser_dir = get_package_share_directory('pointcloud_to_laserscan')

    params_file = LaunchConfiguration('params_file')

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            package_dir,
            'params',
            'tiago_camera_nav_params.yaml')
    )

    small_objects_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(small_objects_dir, 'launch', 'detector.launch.py')
        ),
    )

    pcl_to_laser_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pcl_to_laser_dir, 'launch', 'sample_pointcloud_to_laserscan_launch.py')
        ),
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'params_file': params_file
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(small_objects_cmd)
    ld.add_action(pcl_to_laser_cmd)
    ld.add_action(navigation_cmd)

    return ld
