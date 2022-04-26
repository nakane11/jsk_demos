#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from jsk_recognition_msgs.msg import PeoplePoseArray
from hand_navigation_pr2.srv import PeopleSize, PeopleSizeResponse
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros


class GetPeopleSize():

    def __init__(self):
        self.width = rospy.get_param("~width", 0.5)
        self._duration_timeout = rospy.get_param("~timeout", 3.0)

	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id", "camera_link")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        
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
            rospy.loginfo("no people")
            return
        center_list = np.array(center_list)
        index = np.argmin(np.abs(center_list))

        person = msg.poses[index]
        poses = []
        for i, limb in enumerate(person.limb_names):
            if limb == "left shoulder" and person.limb_names[i+1] == "right shoulder":
                l_endpose = np.array([person.poses[i].position.x, person.poses[i].position.y, person.poses[i].position.z])
                r_endpose = np.array([person.poses[i+1].position.x, person.poses[i+1].position.y, person.poses[i+1].position.z])
                self.width = np.linalg.norm(l_endpose - r_endpose)
                break
            else:
                continue

    def get_people_size(self, req):
        response = PeopleSizeResponse()
        self.width = np.nan
        while np.isnan(self.width) or self.width > 0.6:
            self.subscribe()
        response.width = self.width
        return response

    def start_server(self):
        service = rospy.Service('get_people_size', PeopleSize, self.get_people_size) 
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('get_people_size_server')
    get_people_size = GetPeopleSize()
    get_people_size.start_server()
