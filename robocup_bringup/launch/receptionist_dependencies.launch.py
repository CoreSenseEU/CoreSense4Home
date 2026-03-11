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

    move_group_dir = get_package_share_directory('tiago_moveit_config')
    manipulation_dir = get_package_share_directory('manipulation_action_server')
    package_dir = get_package_share_directory('robocup_bringup')
    yolo3d_dir = get_package_share_directory('yolo_bringup')
    navigation_dir = get_package_share_directory('navigation_system')
    knowledge_core_dir = get_package_share_directory('knowledge_core')
    person_tracker_dir = get_package_share_directory('cs4home_person_tracker')

    # manipulation launchers
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(move_group_dir, 'launch', 'move_group.launch.py')
        )
    )

    manipulation_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulation_dir, 'launch', 'simple_server.launch.py')
        )
    )

    # real time launcher
    real_time = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'real_time.launch.py')
        )
    )

    yolo3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo3d_dir, 'launch', 'yolo.launch.py')
        ),
        launch_arguments={
            # 'namespace': 'perception_system',
            # 'use_3d': 'True',
            'model': 'yolo11n.pt',
            'input_image_topic': '/head_front_camera/rgb/image_raw',
            'input_depth_topic': '/head_front_camera/depth/image_raw',
            'input_depth_info_topic': '/head_front_camera/rgb/camera_info',
            'depth_image_units_divisor': '1000',  # 1 for simulation, 1000 real
            'target_frame': 'head_front_camera_color_optical_frame',
            'threshold': '0.5'
        }.items()
    )

    dialog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'dialog.launch.py')
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation_system.launch.py')
        ),
        launch_arguments={
            'rviz': 'True',
            # 'map': package_dir + '/maps/robocup_arena_1.yaml', # ARENA C
            'map': package_dir + '/maps/lab_marzo.yaml', # ARENA B
            'params_file': package_dir +
                    '/config/receptionist/tiago_nav_params.yaml',
            'slam_params_file': package_dir +
                    '/config/receptionist/tiago_nav_follow_params.yaml',
            'nav_mode': 'amcl'
        }.items()
    )

    knowledge_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(knowledge_core_dir, 'launch', 'knowledge_core.launch.py')
        )
    )

    laser_people_detector = Node(
        package='upo_laser_people_detector',
        executable='lasermodelnode',
        output='screen',
        parameters=[
            {'model_file': os.path.join(package_dir, 'models', 'LFE-PPN.onnx')},
            {'laser_topic': '/scan'}
        ]
    )


    person_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(person_tracker_dir, 'launch', 'person_tracker.launch.py')
        ),
        launch_arguments={
            'target_frame': 'map',
            'only_yolo': 'true',
            'ema_alpha': '0.1',
            'yolo_only_gate': '2.0',
            'yolo_only_delete_threshold': '1.5',
            'yolo_only_confirm_threshold': '2'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(navigation)
    ld.add_action(dialog)
    ld.add_action(yolo3d)
    ld.add_action(real_time)
    ld.add_action(knowledge_core)
    # ld.add_action(laser_people_detector) Not working so far, only yolo for the moment
    ld.add_action(person_tracker)
    # ld.add_action(move_group)
    ld.add_action(manipulation_server)

    return ld
