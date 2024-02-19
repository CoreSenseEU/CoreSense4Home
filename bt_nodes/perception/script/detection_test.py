#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from yolov8_msgs.msg import DetectionArray, Detection, BoundingBox2D

DEFAULT_NUM_OBJECTS = 5

class DetectionPublisherNode(Node):
    def __init__(self):
        super().__init__('detection_publisher_node')
        self.publisher_ = self.create_publisher(DetectionArray, '/perception_system/objects_detection', 10)
        self.timer = self.create_timer(1, self.publish_detection)
        
        self.num_objects = self.get_parameter_or('num_objects', DEFAULT_NUM_OBJECTS)
        self.get_logger().info(f"Number of found objects: {self.num_objects}")

    def publish_detection(self):
        detection_msg = DetectionArray()
        detection_msg.header = Header()
        detection_msg.header.frame_id = "test"
        detection_msg.header.stamp = self.get_clock().now().to_msg()

        # Create a sample detection
        for n_obj in range(0, self.num_objects):
            detection = Detection()
            detection.class_id = 1
            detection.class_name = f"class_n{n_obj}"
            detection.score = 0.85
            detection.id = "example_id"
            detection.bbox = BoundingBox2D()
            # Populate the bounding box as needed

            # Append the detection to the list of detections
            detection_msg.detections.append(detection)

        self.publisher_.publish(detection_msg)
        self.get_logger().info('Published detection')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
