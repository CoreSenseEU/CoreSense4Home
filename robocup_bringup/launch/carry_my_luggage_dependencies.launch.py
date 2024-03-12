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
from launch_ros.actions import Node
# from launch.actions import LogInfo, RegisterEventHandler
# from launch.event_handlers import OnExecutionComplete
# import lifecycle_msgs
# from launch_ros.events.lifecycle


def generate_launch_description():

    package_dir = get_package_share_directory('robocup_bringup')
    whisper_dir = get_package_share_directory('whisper_bringup')
    audio_common_dir = get_package_share_directory('audio_common')
    move_group_dir = get_package_share_directory('tiago_mtc_examples')
    manipulation_dir = get_package_share_directory('manipulation_action_server')
    attention_dir = get_package_share_directory('attention_system')
    perception_dir = get_package_share_directory('perception_system')

    carry_config = os.path.join(
        package_dir,
        'params',
        'carry_params.yaml'
        )

    # Configuration Variables
    model_repo = LaunchConfiguration('model_repo')
    model_filename = LaunchConfiguration('model_filename')

    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    # log_level = LaunchConfiguration("log_level")

    declare_model_repo_cmd = DeclareLaunchArgument(
        'model_repo', default_value="ggerganov/whisper.cpp", description="Hugging Face model repo")

    declare_model_filename_cmd = DeclareLaunchArgument(
        'model_filename', default_value="ggml-large-v3-q5_0.bin", description="Hugging Face model filename")
    
    declare_log_level = DeclareLaunchArgument(
            "log_level",
            default_value="error",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        )

    # audio related launchers:

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(whisper_dir, 'launch', 'whisper.launch.py')
        ),
        launch_arguments={
            'model_repo': model_repo,
            'model_filename': model_filename
        }.items()
    )

    audio_common_tts_node = Node(
        package="audio_common",
        executable="tts_node",
        parameters=[
            {"chunk": 4096},
            {"frame_id": ""},
            {"model": "tts_models/en/ljspeech/vits"},
            {"speaker_wav": ""},
            {"device": "cuda"}]
    )

    audio_common_player_node = Node(
        package="audio_common",
        executable="audio_player_node",
        parameters=[
            {"channels": 2},
            {"device": -1}]
    )

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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation_follow.launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'rviz': 'True'
        }.items()
    )

    attention = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(attention_dir, 'launch', 'attention.launch.py')
        )
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_dir, 'launch', 'perception3d.launch.py')
        ),
        launch_arguments={
            'model': 'yolov8x-pose.pt',
            'input_depth_topic': '/head_front_camera/depth/image_raw',
            'input_depth_info_topic': '/head_front_camera/depth/camera_info',
            'depth_image_units_divisor': '1000',
            'target_frame': 'head_front_camera_link_color_optical_frame',
            'namespace': 'perception_system'
        }.items()


    )


    carry_my_luggage = Node(
        package='bt_test',
        executable='carry_my_luggage_test',
        parameters=[carry_config],
        output='screen',
        namespace='perception_system'
    )

    # wait_for_navigation = RegisterEventHandler(
    #          OnExecutionComplete(
    #                  target_action=,
    #                  on_start=[LogInfo(msg="Started the carry_ luggage. "), 
    #                                  another_node]
    #          )
    #    )

    ld = LaunchDescription()
    ld.add_action(perception)
    # ld.add_action(attention) # afuera 
    # ld.add_action(navigation) 
    ld.add_action(manipulation_server)
    ld.add_action(move_group)
    # # ld.add_action(carry_my_luggage)
    ld.add_action(declare_model_repo_cmd)
    ld.add_action(declare_model_filename_cmd)
    ld.add_action(whisper_cmd)
    ld.add_action(audio_common_tts_node)
    ld.add_action(audio_common_player_node)
    ld.add_action(declare_log_level)

    return ld
