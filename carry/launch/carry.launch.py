# Copyright (c) 2025 TODO. All rights reserved.
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
from launch.actions import EmitEvent, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action, Shutdown
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch_pal import get_pal_configuration


def generate_launch_description():
    pkg = 'carry'
    node = 'carry'
    ld = LaunchDescription()

    # automatically fetch the start parameters for this node,
    # using defaults installed with the skill, as well as possible
    # user overrides
    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)

    node = LifecycleNode(
        package=pkg,
        executable='run_app',
        namespace='',
        name=node,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both', emulate_tty=True,
        on_exit=Shutdown()
        )

    ld.add_action(node)

    # automatically perform the lifecycle transitions to configure and activate
    # the skill at startup
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    ld.add_action(configure_event)

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
        handle_once=True))

    ld.add_action(activate_event)

    return ld
