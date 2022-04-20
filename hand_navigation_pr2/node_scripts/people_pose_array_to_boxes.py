#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros


class PeoplePoseArrayToBoxes(ConnectionBasedTransport):

    def __init__(self):
        super(PeoplePoseArrayToBoxes, self).__init__()
        self.width = rospy.get_param("~width", 0.5)
        self.height = rospy.get_param("~height", 3.0)
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.pub = self.advertise('~output', BoundingBoxArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        header = deepcopy(msg.header)
        header.frame_id = self.base_frame_id
        boxes_msg = BoundingBoxArray(header=header)
        try:
            pykdl_transform_base_to_camera = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self.base_frame_id,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rospy.Duration(self._duration_timeout)))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        for person in msg.poses:
            poses = []
            for pose in person.poses:
                poses.append(
                    [pose.position.x,
                     pose.position.y,
                     pose.position.z])
            if len(poses) < 5:
                continue
            poses = np.array(poses)
            center = np.sum(poses, axis=0) / len(poses)
            if np.any(np.isnan(center)):
                continue
            box_msg = BoundingBox(header=header)
            x, y, z = center[0], center[1], center[2]
            x, y, z = pykdl_transform_base_to_camera * PyKDL.Vector(
                x, y, z)
            box_msg.pose.position.x = x
            box_msg.pose.position.y = y
            box_msg.pose.position.z = z
            box_msg.pose.orientation.x = 0.0
            box_msg.pose.orientation.y = 0.0
            box_msg.pose.orientation.z = 0.0
            box_msg.pose.orientation.w = 1.0
            box_msg.dimensions.x = self.width
            box_msg.dimensions.y = self.width
            box_msg.dimensions.z = self.height
            boxes_msg.boxes.append(box_msg)
        self.pub.publish(boxes_msg)


if __name__ == '__main__':
    rospy.init_node('people_pose_array_to_boxes')
    PeoplePoseArrayToBoxes()
    rospy.spin()
