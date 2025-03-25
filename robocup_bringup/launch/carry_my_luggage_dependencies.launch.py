import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():

    navigation_dir = get_package_share_directory('navigation_system')
    hri_body_detect_dir = get_package_share_directory('hri_body_detect')
    whisper_dir = get_package_share_directory('whisper_bringup')
    package_dir = get_package_share_directory('robocup_bringup')

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

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(whisper_dir, 'launch', 'whisper.launch.py')
        ),
        launch_arguments={
            'silero_vad_use_cuda': 'False',
        }.items()
    )

    execute_bt_server = Node(
        package='robocup_bringup',
        executable='behavior_server_main',
        output='screen',
        name='behavior_server2',
        parameters=[
            os.path.join(package_dir,
                         'config',
                         'carry_my_luggage',
                         'behavior_server.yaml')
        ]
    )

    hri_body_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hri_body_detect_dir, 'launch', 'hri_body_detect.launch.py')
        ),
        launch_arguments={
            'use_depth': 'true',
            'use_cmc': 'true',
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(whisper_cmd)
    ld.add_action(
        SetRemap(
            src='image',
            dst='/head_front_camera/rgb/image_raw'
        )
    )
    ld.add_action(
        SetRemap(
            src='camera_info',
            dst='/head_front_camera/rgb/camera_info'
        )
    )
    ld.add_action(
        SetRemap(
            src='depth_image',
            dst='/head_front_camera/depth/image_raw'
        )
    )
    ld.add_action(
        SetRemap(
            src='depth_info',
            dst='/head_front_camera/rgb/camera_info'
        )
    )

    ld.add_action(navigation)
    ld.add_action(hri_body_detect)
    ld.add_action(execute_bt_server)

    return ld
