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
    nav2_dir = get_package_share_directory('nav2_bringup')
    small_objects_dir = get_package_share_directory('small_objects_detector')
    pcl_to_laser_dir = get_package_share_directory('pointcloud_to_laserscan')

    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    # slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    # map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='True')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='False')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value='')

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            package_dir,
            'params',
            'tiago_nav_follow_params.yaml')
    )

    # Actions
    small_objects_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(small_objects_dir, 'launch', 'detector.launch.py')
        ),
    )

    pcl_to_laser_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pcl_to_laser_dir,
                         'launch',
                         'sample_pointcloud_to_laserscan_launch.py')
        ),
    )

    # localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_dir, 'launch', 'localization_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'slam': slam,
    #         'map': map_file,
    #         'params_file': params_file
    #     }.items(),
    # )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items(),
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # navigation_system_node = Node(
    #     package='navigation_system',
    #     executable='navigation_system_node',
    #     name='navigation_system_node',
    #     output='screen',
    #     parameters=[{'nodes': [],
    #                  'mode': 'amcl'}]
    # )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz
        }.items(),
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(small_objects_cmd)
    ld.add_action(pcl_to_laser_cmd)
    # ld.add_action(localization_cmd) Not needed for slam =P
    ld.add_action(slam_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    # ld.add_action(navigation_system_node)

    return ld
