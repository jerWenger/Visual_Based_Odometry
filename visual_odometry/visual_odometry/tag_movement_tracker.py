#!/usr/bin/env python3

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
import PyKDL
from .helper_functions import TFHelper
import matplotlib.pyplot as plt

class TrackMovement(Node):
    def __init__(self, apriltag_topic):
        """ Initialize the movement tracker"""
        super().__init__('tag_movement_tracker')

        # Define frame names
        self.camera_frame = 'camera'
        self.tag_frame = 'tag36h11:1'   # Only working with tag 1
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # Initialize attributes
        self.detection_array = None
        self.last_detection_timestamp = None
        self.first_tag_transform = None
        self.current_tag_transform = None
        self.first_odom_transform = None
        self.current_odom_transform = None
        self.robot_pos_array = [(0,0)]
        self.robot_odom_array = [(0,0)]
        self.robot_x = [0]
        self.robot_y = [0]
        self.odom_x = [0]
        self.odom_y = [0]
        self.transform_helper = TFHelper(self)

        # Subscribe to AprilTag detections
        self.create_subscription(AprilTagDetectionArray, apriltag_topic, self.new_pose_data, 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def new_pose_data(self, msg):
        """ Process Apriltage detection message"""
        self.last_detection_timestamp = msg.header.stamp
        if self.detection_array == None:
            self.detection_array = msg

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """

        # Create interactive (live updating) scatter plot
        plt.ion()
        fig, ax = plt.subplots()
        sc = ax.scatter(self.robot_x, self.robot_y, label="AprilTag Location")
        sc1 = ax.scatter(self.odom_x, self.odom_y, label="Odometry Location")
        plt.legend()
        plt.xlim([-0.75,0.75])
        plt.ylim([-0.75,0.75])
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Relative Movement of Neato")
        plt.draw()
        
        while True:
            self.run_loop()
            # Append new AprilTag calculated positions & odometry calculated positions
            self.robot_x.append(self.robot_pos_array[-1][0])
            self.robot_y.append(self.robot_pos_array[-1][1])
            self.odom_x.append(self.robot_odom_array[-1][0])
            self.odom_y.append(self.robot_odom_array[-1][1])
            sc.set_offsets(np.c_[self.robot_x, self.robot_y])
            sc1.set_offsets(np.c_[self.odom_x, self.odom_y])
            fig.canvas.draw_idle()
            plt.pause(0.1)
            time.sleep(0.1)

    def run_loop(self):
        # Check if there are any detections
        if self.detection_array is not None:
            # Set first tag and odometry transform to use as (0,0) position
            if self.first_tag_transform is None or self.first_odom_transform is None:
                self.first_tag_transform = self.transform_helper.get_tag_transform(self.camera_frame, self.tag_frame, self.last_detection_timestamp)
                self.first_odom_transform = self.transform_helper.get_odom_transform(self.odom_frame, self.base_frame, self.last_detection_timestamp)
                return
            # Get current tag & odometry transform
            else:
                self.current_tag_transform = self.transform_helper.get_tag_transform(self.camera_frame, self.tag_frame, self.last_detection_timestamp)
                self.current_odom_transform = self.transform_helper.get_odom_transform(self.odom_frame, self.base_frame, self.last_detection_timestamp)

            # Calculate relative transforms between current tag transform and first and same thing for 
            if self.current_tag_transform is not None and self.current_odom_transform is not None:
                relative_movement = np.matmul(self.first_tag_transform, np.linalg.inv(self.current_tag_transform))
                print(f"{relative_movement=}")
                self.robot_pos_array.append((relative_movement[1, 2], -relative_movement[0, 2]))
                relative_odom = np.matmul(np.linalg.inv(self.first_odom_transform), self.current_odom_transform)
                print(f"{relative_odom=}")
                self.robot_odom_array.append((relative_odom[0, 2], relative_odom[1, 2]))
            

def main(args=None):
    rclpy.init()
    n = TrackMovement("/apriltag/detections")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()