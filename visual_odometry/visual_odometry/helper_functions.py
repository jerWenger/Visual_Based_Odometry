""" Some convenience functions for translating between various representations
    of a robot pose. """

from urllib.robotparser import RobotFileParser
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.time import Time
from rclpy.duration import Duration
import math
import numpy as np
from copy import deepcopy
from numpy.random import random_sample
import PyKDL

def stamped_transform_to_pose(t):
    t = t.transform
    return Pose(position=Point(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                orientation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w))


class TFHelper(object):
    """ TFHelper Provides functionality to convert poses between various
        forms, compare angles in a suitable way, and publish needed
        transforms to ROS """
    def __init__(self, node):
        self.logger = node.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.node = node        # hold onto this for logging
        self.transform_tolerance = Duration(seconds=0.08)    # tolerance for mismatch between scan and odom timestamp

    def convert_translation_rotation_to_pose(self, translation, rotation):
        """ Convert from representation of a pose as translation and rotation
            (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],
                                   y=translation[1],
                                   z=translation[2]),
                    orientation=Quaternion(x=rotation[0],
                                           y=rotation[1],
                                           z=rotation[2],
                                           w=rotation[3]))

    def get_tag_transform(self, camera_frame, tag_frame, timestamp):
        if self.tf_buffer.can_transform(camera_frame, tag_frame, timestamp):
            transform = self.tf_buffer.lookup_transform(camera_frame, tag_frame, timestamp)
            tag_pose_frame = PyKDL.Frame(V=PyKDL.Vector(x=transform.transform.translation.x,
                                                y=transform.transform.translation.y,
                                                z=transform.transform.translation.z),
                                R=PyKDL.Rotation.Quaternion(x=transform.transform.rotation.x,
                                                            y=transform.transform.rotation.y,
                                                            z=transform.transform.rotation.z,
                                                            w=transform.transform.rotation.w))
            tag_rotation_vector = (tag_pose_frame.M[0,2], tag_pose_frame.M[2,2])
            tag_rotation_angle = np.arctan2(tag_rotation_vector[1], tag_rotation_vector[0])
            # print(f"angle: {tag_rotation_angle * 180/np.pi}")
            tag_position = (transform.transform.translation.x, transform.transform.translation.z)
            # print(f"{tag_position=}")
            tag_2d_transform = np.matrix([[np.cos(tag_rotation_angle), -np.sin(tag_rotation_angle), tag_position[0]],
                                [np.sin(tag_rotation_angle), np.cos(tag_rotation_angle), tag_position[1]],
                                [0, 0, 1]])
            return tag_2d_transform
        return None

    def get_matching_odom_pose(self, odom_frame, base_frame, timestamp):
        """ Find the odometry position for a given timestamp.  We want to avoid blocking, so if the transform is
            not ready, we return None.

            returns: a tuple where the first element is the stamped transform and the second element is the
                     delta in time between the requested time and the most recent transform available """
        if self.tf_buffer.can_transform(odom_frame, base_frame, timestamp):
            # we can get the pose at the exact right time
            return (stamped_transform_to_pose(self.tf_buffer.lookup_transform(odom_frame, base_frame, timestamp)), Duration(seconds=0.0))
        elif self.tf_buffer.can_transform(odom_frame,
                                          base_frame,
                                          Time()):
            most_recent = self.tf_buffer.lookup_transform(odom_frame,
                                                          base_frame,
                                                          Time())
            delta_t = Time.from_msg(timestamp) - Time.from_msg(most_recent.header.stamp)
            return None
        else:
            return None

    