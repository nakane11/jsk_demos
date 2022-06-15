#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros


class PeoplePoseArrayAccumulator(ConnectionBasedTransport):

    def __init__(self):
        super(PeoplePoseArrayAccumulator, self).__init__()
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.pub = self.advertise('~output', PoseArray, queue_size=1)
        self.pose_array = []
        self.last_published_time = rospy.Time.now()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        # rospy.logerr("{}".format(msg.header.stamp.to_sec()))
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

        center_array = []
        for person in msg.poses:
            # if len(person.poses) < 5:
            #     return
            # rospy.loginfo("score:{}".format(np.average(np.array(person.scores))))
            # if np.average(np.array(person.scores)) < 0.5:
            #     return
            poses = []
            for pose in person.poses:
                poses.append(
                    [pose.position.x,
                     pose.position.y,
                     pose.position.z])
            poses = np.array(poses)
            center = np.sum(poses, axis=0) / len(poses)
            if np.any(np.isnan(center)):
                return
            x, y, z = center[0], center[1], center[2]
            x, y, z = pykdl_transform_base_to_camera * PyKDL.Vector(
                x, y, z)
            if abs(y) > 5:
                return
            center_array.append([x, y, z])
        if center_array:
            center_array = np.array(center_array)
            x_array = np.abs(center_array[:, 0])
            index = x_array.argmin()
            self.pose_array.append(center_array[index, :].tolist())

        if rospy.Time.now() - self.last_published_time > rospy.Duration(1):
            header = deepcopy(msg.header)
            header.frame_id = self.base_frame_id
            pose_array_msg = PoseArray(header=header)
            pose_msg = Pose()
            if len(self.pose_array) > 3:
                pose_median = np.median(np.array(self.pose_array), axis = 0)
                pose_msg.position.x = pose_median[0]
                pose_msg.position.y = pose_median[1]
                pose_msg.position.z = pose_median[2]
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = 1.0
                pose_array_msg.poses.append(pose_msg)
            self.pub.publish(pose_array_msg)
            self.pose_array = []
            self.last_published_time = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('people_pose_array_accumulator')
    PeoplePoseArrayAccumulator()
    rospy.spin()
