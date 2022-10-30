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

    