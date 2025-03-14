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

import json

from nav2_msgs.action import DummyBehavior
from hri_actions_msgs.msg import Intent
from rclpy.action import ActionClient
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
# from sample_skill_msgs.action import SkillControl


class MissionController(Node):

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('mission_carry')

        self.get_logger().info("Initialising...")

        self._hri_listener = None
        self._intents_sub = None
        self._timer = None
        self._intent_sub = None
        self._intent_pub = None
        self._last_intent = None

        self._execute_bt_client = None
        self._say_skill_client = None

        self._engage_xml = None
        self._instruction_xml = None
        self._execution_xml = None
        self._disengagement_xml = None

        # parameters
        engage_xml = """<?xml version="1.0"?>
        <root main_tree_to_execute="BehaviorTree">
            <BehaviorTree ID="BehaviorTree">
                <Sequence>
                    <Action ID="InitCarry" x_axis_max="{max_x}" x_axis_min="{min_x}" y_axis_max="{max_y}" y_axis_min="{min_y}"/>
                    <Action ID="PalSpeak" param="" say_text="Hi, This is gentlebots. Im ready to carry your luggage,please point to the correct bag and dont be too close"/>
                    <Action ID="SetStartPosition" 
                                                reference_frame="map"
                                                frame_name="initial"
                                                initial_pose="{going_back}"
                                                x_offset="0.0"
                                                y_offset="0.0"
                                                />
                <RetryUntilSuccessful num_attempts="-1">
                    <Condition ID="IsBodyDetected" />
                </RetryUntilSuccessful>
                </Sequence> 
            </BehaviorTree>
        </root>"""

        instruction_xml = """<?xml version="1.0"?>
            <root main_tree_to_execute="BehaviorTree">
                <BehaviorTree ID="BehaviorTree">
                    <Sequence>
                        <RetryUntilSuccessful num_attempts="-1">
                            <Condition ID="PalIsPointing" output_frame="{bag_tf}"/>
                        </RetryUntilSuccessful>
                        <Action ID="RemoveStringSuffix" string_to_remove="{bag_tf}"
                                                        suffix="_"
                                                        result="{bag_direction}"/> 
                        <Action ID="PalSpeak" param="{bag_direction}" say_text="I see you are pointing to the bag at my"/>
                        <Action ID="PalSpeak" param="" say_text="I am on my way"/>
                        <Action ID="MoveTo" tf_frame="{bag_tf}"/>
                        <Action ID="PalMoveToPredefined" pose="offer"/> 
                        <Action ID="PalSpeak" param="" say_text="please put the bag in my gripper"/> 
                        <Delay delay_msec="140">
                            <Action ID="PalMoveToPredefined" pose="open"/>
                        </Delay>
                        <ReactiveSequence>
                            <RetryUntilSuccessful num_attempts="-1">
                                <Condition ID="IsBodyDetected" best_detection="{current_person}"/>
                            </RetryUntilSuccessful>
                            <Action ID="LookAt" tf_frame="{current_person}"/>
                        </ReactiveSequence>
                        <Delay delay_msec="300">
                            <Action ID="PalMoveToPredefined" pose="close" />
                        </Delay>
                        <Delay delay_msec="500">
                            <Action ID="PalMoveToPredefined" pose="home"/>
                        </Delay>
                        <Action ID="PalSpeak" param="" say_text="Perfect, now i will follow you. Please stop at the end "/>
                    </Sequence> 
                </BehaviorTree>
            </root>"""
        
        execution_xml = """<?xml version="1.0"?>
            <root main_tree_to_execute="BehaviorTree">
                <BehaviorTree ID="BehaviorTree">
                    <Sequence>
                        <RetryUntilSuccessful num_attempts="-1">
                            <Fallback>
                                <Timeout msec="2000">
                                    <RetryUntilSuccessful num_attempts="-1">
                                            <Condition ID="IsBodyDetected" best_detection="{current_person}"/>
                                    </RetryUntilSuccessful>
                                </Timeout>
                                <Inverter>
                                    <Action ID="PalSpeak" say_text="I can't see you, can you please stand in front of me?"/>
                                </Inverter>
                                <Delay delay_msec="3000">
                                    <AlwaysFailure/>
                                </Delay>
                            </Fallback>
                        </RetryUntilSuccessful>

                        <Action ID="LookAt" tf_frame="{current_person}"/>
                        <RetryUntilSuccessful num_attempts="-1">
                            <Sequence>
                                <Fallback>
                                    <ReactiveSequence>
                                        <Fallback>
                                        <Timeout msec="2000">
                                            <RetryUntilSuccessful num_attempts="-1">
                                                <Condition ID="IsBodyDetected" best_detection="{current_person}"/>
                                            </RetryUntilSuccessful>
                                        </Timeout>
                                        <Inverter>
                                            <PalSpeak say_text="I can't see you, can you please stand in front of me?"/>
                                        </Inverter>
                                        <Delay delay_msec="3000">
                                            <AlwaysFailure/>
                                        </Delay>    
                                        </Fallback>
                                        <Action ID="LookAt" tf_frame="{current_person}"/>
                                        <Condition ID="IsEntityMoving" distance_tolerance="0.6"
                                                                        robot_distance_to_person="1.5"
                                                                        frame="{current_person}"
                                                                        check_time="8.0"/>
                                        <Action ID="FollowEntity" camera_frame="head_front_camera_rgb_optical_frame"
                                                                    distance_tolerance="0.2"
                                                                    frame_to_follow="{current_person}"
                                                                    x_axis_max="{max_x}"
                                                                    x_axis_min="{min_x}"
                                                                    y_axis_max="{max_y}"
                                                                    y_axis_min="{min_y}"/>
                                    </ReactiveSequence>
                                    <Action ID="PalSpeak" param="" say_text="have we arrived to the destination?"/>
                                </Fallback>
                                <ReactiveSequence>
                                    <RetryUntilSuccessful num_attempts="-1">
                                        <Condition ID="IsBodyDetected" best_detection="{current_person}"/>
                                    </RetryUntilSuccessful>
                                    <Action ID="LookAt" tf_frame="{current_person}"/>
                                    <ForceSuccess>
                                        <Action ID="GoalPublisher" 
                                            camera_frame="head_front_camera_rgb_optical_frame"
                                            distance_tolerance="0.2"
                                            frame_to_follow="{current_person}"/>  
                                    </ForceSuccess>
                                    <Action ID="DialogConfirmation"/>
                                </ReactiveSequence>
                            </Sequence>
                        </RetryUntilSuccessful>
                    </Sequence>
                </BehaviorTree>
            </root>"""
    
        disengagement_xml = """<?xml version="1.0"?>
            <root main_tree_to_execute="BehaviorTree">
                <BehaviorTree ID="BehaviorTree">
                    <Sequence>
                        <Action ID="PalSpeak" param="" say_text="I will give you the bag now, please be careful"/>
                        <Action ID="PalMoveToPredefined" pose="offer" group_name="arm_torso"/>
                        <ReactiveSequence>
                            <Action ID="LookAt" tf_frame="person_0"/>
                            <Action ID="PalSpeak" param="" say_text="Here is the bag, please take it"/>
                        </ReactiveSequence>
                        <Delay delay_msec="140">
                            <Action ID="PalMoveToPredefined" pose="open" group_name="gripper"/>
                        </Delay>
                        <Action ID="PalMoveToPredefined" pose="home" group_name="arm_torso"/>
                        <Delay delay_msec="3000">
                            <Action ID="PalSpeak" say_text="I am going back, have a nice day"/>
                        </Delay>
                        <Action ID="MoveTo" distance_tolerance="0.0" tf_frame="{going_back}"/>
                    </Sequence>
                </BehaviorTree>
            </root>"""

        self.declare_parameter('engage_xml', engage_xml)
        self.declare_parameter('instruction_xml', instruction_xml)
        self.declare_parameter('execution_xml', execution_xml)
        self.declare_parameter('disengagement_xml', disengagement_xml)

        self.get_logger().info('Node initialised. Ready to transition to configure.')

    def on_intent(self, msg):
        
        self.get_logger().info("Received an intent: %s" % msg.intent)
        self._last_intent = msg.intent
        if msg.intent == Intent.ENGAGE_WITH:
            self.get_logger().info('Engaging user')
            goal = DummyBehavior.Goal()
            self.get_logger().info('Goal msg created')
            self.get_logger().info(f'Executing following xml: {self._engage_xml}')
            goal.command.data = self._engage_xml
            self.get_logger().info('Sending goal!')
            self._greet_task_future = self._execute_bt_client.send_goal_async(
                goal,
                feedback_callback=self.on_feedback)
            self.get_logger().info('Goal sent!')
            self._greet_task_future.add_done_callback(self.on_execute_bt_goal)
            return
        elif msg.intent == 'INSTRUCTION':
            self.get_logger().info('Received instruction intent')
            self.get_logger().info('Waiting for user to point to the correct bag')
            goal = DummyBehavior.Goal()
            self.get_logger().info('Goal msg created')
            self.get_logger().info(f'Executing following xml: {self._engage_xml}')
            goal.command.data = self._instruction_xml
            self.get_logger().info('Sending goal!')
            self._greet_task_future = self._execute_bt_client.send_goal_async(
                goal,
                feedback_callback=self.on_feedback)
            self.get_logger().info('Goal sent!')
            self._greet_task_future.add_done_callback(self.on_execute_bt_goal)
            return
        elif msg.intent == 'TASK_EXECUTION':
            self.get_logger().info('Received task execution intent')
            self.get_logger().info('Heading to deliver the bag')
            goal = DummyBehavior.Goal()
            goal.command.data = self._execution_xml
            self.get_logger().info('Sending goal')
            self._greet_task_future = self._execute_bt_client.send_goal_async(
                goal,
                feedback_callback=self.on_feedback)
            self.get_logger().info('Goal sent!')
            self._greet_task_future.add_done_callback(self.on_execute_bt_goal)
            return
        elif msg.intent == 'DISENGAGE_FROM':
            self.get_logger().info('Received disengagement intent')
            self.get_logger().info('Disengaging from user')
            goal = DummyBehavior.Goal()
            goal.command.data = self._disengagement_xml
            self.get_logger().info('Sending goal')
            self._greet_task_future = self._execute_bt_client.send_goal_async(
                goal,
                feedback_callback=self.on_feedback)
            self.get_logger().info('Goal sent!')
            self._greet_task_future.add_done_callback(self.on_execute_bt_goal)
            return
        else:
            self.get_logger().warn("I don't know yet how to process intent "
                                   "<%s>" % msg.intent)

    def on_feedback(self, msg):
        self.get_logger().info("Received feedback")

    def on_execute_bt_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Failed to greet the user. Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.on_execute_bt_done)

    def on_execute_bt_done(self, future):
        self.get_logger().info("Successfully greeted user")
        if self._last_intent == Intent.ENGAGE_WITH:
            self.get_logger().info("Egagement completed, moving to INSTRUCTION")
            msg = Intent()
            msg.intent = 'INSTRUCTION'
            self._intent_pub.publish(msg)
        elif self._last_intent == 'INSTRUCTION':
            self.get_logger().info('Instruction completed, moving to TASK EXECUTION')
            msg = Intent()
            msg.intent = 'TASK_EXECUTION'
            self._intent_pub.publish(msg)
        elif self._last_intent == 'TASK_EXECUTION':
            self.get_logger().info('Task execution completed, moving to DISENGAGEMENT')
            msg = Intent()
            msg.intent = 'DISENGAGE_FROM'
            self._intent_pub.publish(msg)


    def on_say_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Failed to say something. Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.on_say_done)

    def on_say_done(self, future):
        result = future.result().result
        if result:
            self.get_logger().error(f"Say skill returned: {result.result}")
        else:
            self.get_logger().info("Successfully said something")

    #########################################################################

    #################################
    #
    # Lifecycle transitions callbacks
    #
    #################################

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node."""
        self._engage_xml = self.get_parameter('engage_xml').get_parameter_value().string_value
        self._instruction_xml = self.get_parameter('instruction_xml').get_parameter_value().string_value
        self._execution_xml = self.get_parameter('execution_xml').get_parameter_value().string_value
        self._disengagement_xml = self.get_parameter('disengagement_xml').get_parameter_value().string_value

        self._intent_pub = self.create_lifecycle_publisher(Intent, 'intents', 10)        
        self._execute_bt_client = ActionClient(self, DummyBehavior, '/execute_bt')
        while not self._execute_bt_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'greet' task not yet available, waiting again...")

        self.get_logger().info('Node configured. Ready to transition to activate.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""
        self._intents_sub = self.create_subscription(
            Intent,
            '/intents',
            self.on_intent,
            10)

        self.get_logger().info("Listening to incoming intents on the %s topic" %
                               self._intents_sub.topic_name)

        timer_period = 0.1  # in sec
        self._timer = self.create_timer(timer_period, self.run)

        self.get_logger().info('Node activated and running.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function (main task of your application)."""
        self.get_logger().info("Stopping application")

        self.destroy_timer(self._timer)
        self.destroy_subscription(self._intents_sub)
        self.destroy_publisher(self._intent_pub)
        self._hri_listener = None

        self.get_logger().info('Node de-activated.')
        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node, after a shutting-down transition is requested."""
        self._execute_bt_client.destroy()
        self._say_skill_client.destroy()

        self.destroy_timer(self._timer)
        self.destroy_subscription(self._intents_sub)

        self.get_logger().info('Shutting down node.')
        return TransitionCallbackReturn.SUCCESS

    def run(self) -> None:
        """Background task of your application."""
        msg = Intent()
        msg.intent = Intent.ENGAGE_WITH
        self._intent_pub.publish(msg)
        self._timer.cancel()
        pass
