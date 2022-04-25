#!/usr/bin/env python

import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from jsk_recognition_msgs.msg import BoundingBox
from hand_navigation_pr2.srv import SetBBoxPublisher, SetBBoxPublisherResponse
import rospy

class BoundingBoxPublisher(object):

    def __init__(self):
        self.seq = 0
        self.frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.position = [0.0, 0.5, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.dimention = [1.5, 0.7, 3.0]
        
        self.pub = rospy.Publisher('~output', BoundingBox, queue_size=1)
        self.rate = rospy.Rate(1)
        self.is_run = False
        service = rospy.Service('bbox_publisher', SetBBoxPublisher, self.set_param_server)
        
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

    def publish(self):
        if self.is_run:
            bbox_msg = BoundingBox()
            bbox_msg.header.seq = self.seq
            bbox_msg.header.frame_id = self.frame_id
            bbox_msg.header.stamp = rospy.Time.now()
            bbox_msg.pose.position = Point(*self.position)
            bbox_msg.pose.orientation = Quaternion(*self.orientation)
            bbox_msg.dimensions = Vector3(*self.dimention)
            self.pub.publish(bbox_msg)

    def set_param_server(self, req):
        response = SetBBoxPublisherResponse()
        self.position = req.position
        self.dimention = req.dimention
        self.is_run = req.switch
        if self.is_run:
            rospy.loginfo("bounding_box_publisher: start")
        else:
            rospy.loginfo("bounding_box_publisher: stop")
        return response
        
if __name__ == '__main__':
    rospy.init_node('bounding_box_publisher')
    bbox_publisher = BoundingBoxPublisher()
   
