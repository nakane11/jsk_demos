#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import PeoplePoseArray
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros


class PeoplePoseArrayAccumulator():

    def __init__(self):
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.pub = rospy.Publisher('~output', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._cb)
        self.last_published_time = rospy.Time.now()

    def _cb(self, msg):
        header = deepcopy(msg.header)
        header.frame_id = self.base_frame_id
        pose_stamped_msg = PoseStamped(header=header)
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
            poses_len.append(len(person.poses))
        if len(poses_len) <= 0:
            return
            
        poses = []
        person = msg.poses[index(max(poses_len))]
        for pose in person.poses:
            poses.append(
                [pose.position.x,
                 pose.position.y,
                 pose.position.z])
        if len(poses) < 5:
            return
        poses = np.array(poses)
        center = np.sum(poses, axis=0) / len(poses)
        if np.any(np.isnan(center)):
            return
        x, y, z = center[0], center[1], center[2]
        x, y, z = pykdl_transform_base_to_camera * PyKDL.Vector(
                x, y, z)
        self.pose_array = np.append(self.pose_array, np.array([x, y, z]), axis = 0)

        if rospy.Time.now() - self.last_published_time > rospy.Duration(3):
            if len(self.pose_array) <= 0:
                return
            pose_median = np.median(self.pose_array, axis = 0)
            pose_stamped_msg.pose.position.x = pose_median[0]
            pose_stamped_msg.pose.position.y = pose_median[1]
            pose_stamped_msg.pose.position.z = pose_median[2]
            pose_stamped_msg.pose.orientation.x = 0.0
            pose_stamped_msg.pose.orientation.y = 0.0
            pose_stamped_msg.pose.orientation.z = 0.0
            pose_stamped_msg.pose.orientation.w = 1.0
            self.pub.publish(pose_stamped_msg)
            self.last_published_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('people_pose_array_accumulator')
    PeoplePoseArrayAccumulator()
    rospy.spin()
