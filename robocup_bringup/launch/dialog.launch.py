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
from llama_bringup.utils import create_llama_launch


def generate_launch_description():
    # package_dir = get_package_share_directory('robocup_bringup')
    # llama_dir = get_package_share_directory('llama_bringup')
    whisper_dir = get_package_share_directory('whisper_bringup')
    # audio_common_dir = get_package_share_directory('audio_common')

    # Configuration Variables
    model_repo = LaunchConfiguration('model_repo')
    model_filename = LaunchConfiguration('model_filename')

    declare_model_repo_cmd = DeclareLaunchArgument(
        'model_repo', default_value='ggerganov/whisper.cpp',
        description='Hugging Face model repo')

    declare_model_filename_cmd = DeclareLaunchArgument(
        'model_filename', default_value='ggml-large-v3-q5_0.bin',
        description='Hugging Face model filename')

    # Actions
    llama_cmd = create_llama_launch(
            n_ctx=2048,
            n_batch=256,
            n_gpu_layers=23,
            n_threads=4,
            n_predict=-1,

            # uncomment this for GPSR:
            # model_repo="cstr/Spaetzle-v60-7b-Q4_0-GGUF",
            # model_filename="Spaetzle-v60-7b_Q4_0.gguf",

            # comment this for GPSR:
            model_repo='TheBloke/Marcoroni-7B-v3-GGUF',
            model_filename='marcoroni-7b-v3.Q3_K_L.gguf',

            prefix='\n\n### Instruction:\n',
            suffix='\n\n### Response:\n',
            stopping_words=["\n\n\n\n"],
    )

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
        package='tts_ros',
        executable='tts_node',
        parameters=[
            {'chunk': 4096},
            {'frame_id': ''},
            {'model': 'tts_models/en/ljspeech/glow-tts'},
            {'speaker_wav': ''},
            {'device': 'cpu'}]
    )

    audio_common_player_node = Node(
        package='audio_common',
        executable='audio_player_node',
        parameters=[
            {'channels': 2},
            {'device': -1}]
    )

    music_player_node = Node(
        package='audio_common',
        executable='music_node',
    )

    ld = LaunchDescription()
    # ld.add_action(declare_model_repo_cmd)
    # ld.add_action(declare_model_filename_cmd)
    # ld.add_action(whisper_cmd)
    # ld.add_action(llama_cmd)
    ld.add_action(audio_common_tts_node)
    ld.add_action(audio_common_player_node)
    # ld.add_action(music_player_node)

    return ld
