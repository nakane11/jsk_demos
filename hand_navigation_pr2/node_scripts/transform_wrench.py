#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import WrenchStamped
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros


class TransformWrenchStamped(ConnectionBasedTransport):

    def __init__(self):
        super(TransformWrenchStamped, self).__init__()
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.pub = self.advertise('~output', WrenchStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', WrenchStamped, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        header = deepcopy(msg.header)
        header.frame_id = self.base_frame_id
        wrench_msg = WrenchStamped(header=header)
        try:
            pykdl_transform_base_to_gripper = tf2_geometry_msgs.transform_to_kdl(
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
        force = msg.wrench.force
        torque = msg.wrench.torque
        x, y, z = force.x, force.y, force.z
        r, p, y = torque.x, torque.y, torque.z
        x, y, z = pykdl_transform_base_to_gripper * PyKDL.Vector(
                x, y, z)
        r, p, y = pykdl_transform_base_to_gripper * PyKDL.Vector(
                r, p, y)
        wrench_msg.wrench.force.x = x
        wrench_msg.wrench.force.y = y
        wrench_msg.wrench.force.z = z
        wrench_msg.wrench.torque.x = r
        wrench_msg.wrench.torque.y = p
        wrench_msg.wrench.torque.z = y
        self.pub.publish(wrench_msg)


if __name__ == '__main__':
    rospy.init_node('transform_wrench_stamped')
    TransformWrenchStamped()
    rospy.spin()
