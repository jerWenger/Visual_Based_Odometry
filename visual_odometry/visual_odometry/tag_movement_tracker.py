import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
import PyKDL

class TrackMovement(Node):
    def __init__(self, image_topic, apriltag_topic, tf_topic):
        """ Initialize the movement tracker"""
        super().__init__('tag_movement_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.annotated_image = None
        self.detection_array = None
        self.tf_array = None
        self.robot_pos_array = [(0,0)]
        self.first_tag_detection = None
        self.first_tag_tf = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(AprilTagDetectionArray, apriltag_topic, self.new_pose_data, 10)
        self.create_subscription(TFMessage, tf_topic, self.tf_attribute, 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def new_pose_data(self, msg):
        """ Process Apriltage detection message"""
        self.detection_array = msg

    def tf_attribute(self, msg):
        """Process TF message"""
        self.tf_array = msg

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        if self.detection_array.detections:
            if not self.first_tag_detection:
                self.first_tag_detection = self.detection_array
                self.first_tag_tf = self.tf_array
            else:
                for transform in self.tf_array:
                    translation = transform.translation
                    rotation = transform.rotation



def main(args=None):
    rclpy.init()
    n = TrackMovement("camera/image_raw", "/apriltag/detections", "/tf")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()