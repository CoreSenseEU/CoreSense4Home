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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# from launch.actions import LogInfo, RegisterEventHandler
# from launch.event_handlers import OnExecutionComplete
# import lifecycle_msgs
# from launch_ros.events.lifecycle


def generate_launch_description():

    navigation_dir = get_package_share_directory('navigation_system')
    whisper_dir = get_package_share_directory('whisper_bringup')
    package_dir = get_package_share_directory('robocup_bringup')

    # audio related launchers:

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(whisper_dir, 'launch', 'whisper.launch.py')
        )
    )
    audio_common_player_node = Node(
        package='audio_common',
        executable='audio_player_node',
        parameters=[
            {'channels': 2},
            {'device': -1}]
    )

    audio_common_tts_node = Node(
        package='tts_ros',
        executable='tts_node',
        parameters=[
            {'chunk': 4096},
            {'frame_id': ''},
            {'model': 'tts_models/en/ljspeech/vits'},
            {'speaker_wav': ''},
            {'device': 'cuda'}]
    )

    # real time launcher
    real_time = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'real_time.launch.py')
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation_system.launch.py')
        ),
        launch_arguments={
            'rviz': 'True',
            'mode': 'amcl',
            'params_file': package_dir + '/config/inspection/tiago_nav_params.yaml',
            'slam_params_file': package_dir +
                    '/config/inspection/tiago_nav_follow_params.yaml',
            'map': os.path.join(
                                package_dir,
                                'maps',
                                'robocup_arena_1.yaml'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(navigation)
    # # ld.add_action(whisper_cmd)
    # ld.add_action(audio_common_player_node)
    # ld.add_action(audio_common_tts_node)
    ld.add_action(real_time)

    return ld
