import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg as geometry_msgs
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R
import numpy as np

class FollowPerson(Node):
    def __init__(self):
        super().__init__('follow_person')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.follow_joint_traj_action_client = ActionClient(self, FollowJointTrajectory, 'head_controller/follow_joint_trajectory')
        print("Waiting for action server")
        self.follow_joint_traj_action_client.wait_for_server()
        print("Action server found")
        self.yaw_limit = (-1.3, 1.3)  # Yaw limits in radians
        self.pitch_limit = (-0.185, 0.185)  # Pitch limits in radians

    def timer_callback(self):
        try:
            transform_head_1 = self.tfBuffer.lookup_transform(
                "head_1_link", "person_0", rclpy.time.Time())
            transform_head_2 = self.tfBuffer.lookup_transform(
                "head_2_link", "person_0", rclpy.time.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Could not get transform")
            return 

        yaw = self.compute_yaw(transform_head_1.transform.translation) * 1.15
        pitch = self.compute_pitch(transform_head_2.transform.translation) * 1.15
        pitch = max(self.pitch_limit[0], min(pitch, self.pitch_limit[1]))
        yaw = max(self.yaw_limit[0], min(yaw, self.yaw_limit[1]))
        # pitch = 0.5
        # yaw = 0.5
        print("Yaw: ", yaw, "Pitch: ", pitch)
        self.send_goal(yaw, pitch)

    def compute_yaw(self, translation):
        # quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        yaw = np.arctan2(translation.y, translation.x)
        return yaw

    def compute_pitch(self, translation):
        pitch = np.arctan2(translation.y, translation.x)       
        return pitch
    
    def compute_euler(self, rotation):
        # Convert quaternion to euler angles using scipy
        r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        euler = r.as_euler('xyz', degrees=False)
        yaw = euler[2]
        pitch = euler[1]
        roll = euler[0]
        return roll, yaw, pitch

    def send_goal(self, yaw, pitch):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.time_from_start = rclpy.time.Duration(seconds=1).to_msg()
        goal_msg.trajectory.points.append(point)
        print("Sending goal")
        self.follow_joint_traj_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    follow_person_node = FollowPerson()
    rclpy.spin(follow_person_node)
    follow_person_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
