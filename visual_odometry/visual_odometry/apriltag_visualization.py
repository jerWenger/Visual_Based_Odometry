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
import tf2_ros as tf
from .helper_functions import TFHelper

class apriltag_visualization(Node):
    """  """

    def __init__(self, image_topic, apriltag_topic, tf_topic):
        """ Initialize the visualizer """
        super().__init__('apriltag_visualization')
        self.cv_image = None                        # the latest image from the camera
        self.annotated_image = None
        self.detection_array = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.camera_frame = "camera"

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(AprilTagDetectionArray, apriltag_topic, self.new_pose_data, 10)
        self.transform_helper = TFHelper(self)
        self.id_color= [(255,0,255), (0,255,255), (255,255,0), (0,0,255), (0,255,0), (255,0,0), (125,0,125), (0,125,125), (125,125,0), (0,0,125), (125,0,0), (0,125,0)]

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

    

    def get_scan_info(self):
        self.tag_array = []

        for i in range(len(self.detection_array.detections)):
            detection = self.detection_array.detections[i]
            tag_frame = "tag36h11:" + str(detection.id)
            tag_data = self.transform_helper.get_matching_odom_pose(tag_frame,self.camera_frame,self.detection_array.header.stamp)
        

            if not tag_data:
                distance = "Error"
            else:
                raw_distance = np.sqrt((tag_data[0].position.x ** 2) + (tag_data[0].position.y ** 2) + (tag_data[0].position.z ** 2))
                distance = str(raw_distance)[0:4]
            this_tag = [detection.id, detection.corners, distance, detection.centre,self.id_color[detection.id]]

            self.tag_array.append(this_tag)





    def overlay_data(self):
        """This function adds the apriltag data to the cv_image"""
        self.annotated_image = self.cv_image

        for tag in self.tag_array:
            
            text = str(tag[0]) + ":" + (tag[2])
            corners = tag[1]
            #transform = self.tf_array.transforms[i]
            
            #Map Corner Values
            self.annotated_image = cv2.line(self.annotated_image,(int(corners[0].x),int(corners[0].y)),(int(corners[1].x),int(corners[1].y)),(0,0,255),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(corners[1].x),int(corners[1].y)),(int(corners[2].x),int(corners[2].y)),(0,255,0),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(corners[2].x),int(corners[2].y)),(int(corners[3].x),int(corners[3].y)),(255,0,0),5)
            self.annotated_image = cv2.line(self.annotated_image,(int(corners[3].x),int(corners[3].y)),(int(corners[0].x),int(corners[0].y)),(0,255,255),5)

            #Get the size of the id (and soon to be distance) so it can be centered
            size = cv2.getTextSize(str(text),cv2.FONT_HERSHEY_PLAIN,5,3)
            self.annotated_image = cv2.putText(self.annotated_image,text,(int(tag[3].x - (size[0][0] / 2)),int(tag[3].y + (size[0][1] / 2))),cv2.FONT_HERSHEY_PLAIN,5,tag[4],3)



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
                self.get_scan_info()
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

