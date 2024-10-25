# Copyright 2024 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robocup_dir = get_package_share_directory('robocup_bringup')

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    config = os.path.join(os.path.join(robocup_dir,
                                       'config',
                                       'real_time_params.yaml'))

    real_time_cmd = Node(
        package='robocup_bringup',
        executable='real_time_main',
        output='screen',
        namespace=namespace,
        parameters=[config])

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(real_time_cmd)

    return ld
