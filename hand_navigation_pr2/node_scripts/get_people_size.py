#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from jsk_recognition_msgs.msg import PeoplePoseArray
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros

from jsk_topic_tools import ConnectionBasedTransport
import std_msgs.msg


class GetPeopleSize(ConnectionBasedTransport):

    def __init__(self):
        super(GetPeopleSize, self).__init__()
        self.width = rospy.get_param("~width", 0.3)
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.width_thresh_max = 0.5
        self.width_thresh_min = 0.2
        self.pub = self.advertise(
            "~output", std_msgs.msg.Float32,
            queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        header = deepcopy(msg.header)
        header.frame_id = self.base_frame_id
        center_list=[]
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
            x, y, z = center[0], center[1], center[2]
            x, y, z = pykdl_transform_base_to_camera * PyKDL.Vector(
                x, y, z)
            center_list.append(y)
        if len(center_list) <= 0:
            return
        center_list = np.array(center_list)
        index = np.argmin(np.abs(center_list))

        person = msg.poses[index]
        l_endpose = np.full((1, 3), np.nan)
        r_endpose = np.full((1, 3), np.nan)
        for i, limb in enumerate(person.limb_names):
            if limb == "left shoulder":
                l_endpose = np.array([person.poses[i].position.x, person.poses[i].position.y, person.poses[i].position.z])
            elif limb == "right shoulder": 
                r_endpose = np.array([person.poses[i].position.x, person.poses[i].position.y, person.poses[i+1].position.z])
        if np.any(np.isnan(l_endpose)) or np.any(np.isnan(r_endpose)):
            return
        else:
            self.width = np.linalg.norm(l_endpose - r_endpose)
        if not np.isnan(self.width) and self.width < self.width_thresh_max and self.width > self.width_thresh_min:
            self.pub.publish(std_msgs.msg.Float32(self.width))
        else:
            self.pub.publish(std_msgs.msg.Float32(100000))
        
if __name__ == '__main__':
    rospy.init_node('get_people_size')
    get_people_size = GetPeopleSize()
    rospy.spin()
