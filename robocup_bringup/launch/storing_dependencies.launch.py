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

    move_group_dir = get_package_share_directory('tiago_mtc_examples')
    manipulation_dir = get_package_share_directory('manipulation_action_server')
    package_dir = get_package_share_directory('robocup_bringup')
    yolo3d_dir = get_package_share_directory('yolov8_bringup')
    navigation_dir = get_package_share_directory('navigation_system')

    # manipulation launchers
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(move_group_dir, 'launch', 'move_group.launch.py')
        )
    )

    manipulation_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulation_dir, 'launch', 'server.launch.py')
        )
    )

    # real time launcher
    real_time = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'real_time.launch.py')
        )
    )

    collition_perception = Node(
        package='perception_system',
        executable='collision_server',
        output='screen',
        parameters=[
            {'pointcloud_topic': '/head_front_camera/depth/points'},
            {'depth_topic': '/head_front_camera/depth/image_raw'},
            {'yolo_topic': '/detections_3d'},
            {'camera_info_topic': '/head_front_camera/depth/camera_info'},
        ]
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

    yolo3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo3d_dir, 'launch', 'yolov8_3d.launch.py')
        ),
        launch_arguments={
            # 'namespace': 'perception_system',
            'model': os.path.join(package_dir,
                                  'config',
                                  'storing_groceries',
                                  'yolo_groceries.pt'),
            # 'model': 'yolov8x.pt',
            'input_image_topic': '/head_front_camera/rgb/image_raw',
            'input_depth_topic': '/head_front_camera/depth/image_raw',
            'input_depth_info_topic': '/head_front_camera/depth/camera_info',
            'depth_image_units_divisor': '1000',  # 1 for simulation, 1000 in real robot
            'target_frame': 'head_front_camera_link_color_optical_frame',
            'threshold': '0.5'
            }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation_system.launch.py')
        ),
        launch_arguments={
            'rviz': 'True',
            'mode': 'amcl',
            'params_file': package_dir + '/config/storing_groceries/tiago_nav_params.yaml',
            'slam_params_file': package_dir + '/config/storing_groceries/tiago_nav_follow_params.yaml',
            'map': os.path.join(
                                package_dir,
                                'maps',
                                'ir_lab.yaml'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(navigation)
    ld.add_action(audio_common_player_node)
    ld.add_action(audio_common_tts_node)
    ld.add_action(yolo3d)
    ld.add_action(real_time)
    ld.add_action(collition_perception)
    ld.add_action(move_group)
    ld.add_action(manipulation_server)

    return ld
