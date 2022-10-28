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

class apriltag_visualization(Node):
    """  """

    def __init__(self, image_topic, apriltag_topic, tf_topic):
        """ Initialize the visualizer """
        super().__init__('apriltag_visualization')
        self.cv_image = None                        # the latest image from the camera
        self.annotated_image = None
        self.detection_array = None
        self.tf_array = None
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

    def overlay_data(self):
        """This function adds the apriltag data to the cv_image"""
        self.annotated_image = self.cv_image

        for detection in self.detection_array.detections:
            self.annotated_image = cv2.line(self.annotated_image,(int(detection.corners[0].x),int(detection.corners[0].y)),(int(detection.corners[1].x),int(detection.corners[1].y)),(0,0,255),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(detection.corners[1].x),int(detection.corners[1].y)),(int(detection.corners[2].x),int(detection.corners[2].y)),(0,255,0),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(detection.corners[2].x),int(detection.corners[2].y)),(int(detection.corners[3].x),int(detection.corners[3].y)),(255,0,0),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(detection.corners[3].x),int(detection.corners[3].y)),(int(detection.corners[0].x),int(detection.corners[0].y)),(0,255,255),5)
            
            #Get the size of the number so it can be centered
            size = cv2.getTextSize(str(detection.id),cv2.FONT_HERSHEY_PLAIN,8,3)
            self.annotated_image = cv2.putText(self.annotated_image,str(detection.id),(int(detection.centre.x - (size[0][0] / 2)),int(detection.centre.y + (size[0][1] / 2))),cv2.FONT_HERSHEY_PLAIN,8,(255,0,255),3)

        

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        while True:
            self.run_loop()
            time.sleep(0.1)



    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.cv_image is None:
            # ADD a function here

            if not self.detection_array.detections:
                cv2.imshow('video_window', self.cv_image)
            else:
                self.overlay_data()
                cv2.imshow('video_window', self.annotated_image)
            cv2.waitKey(5)

def main(args=None):
    rclpy.init()
    n = apriltag_visualization("camera/image_raw", "/apriltag/detections", "/tf")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()