#!/usr/bin/env python
import visualization_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import numpy as np
from copy import deepcopy

class AttentionEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(AttentionEstimation, self).__init__()
        self.pose_pub = self.advertise('/attention_point',
                                       visualization_msgs.msg.Marker,
                                       queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('/attention_cloud', sensor_msgs.msg.PointCloud2, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        header = deepcopy(msg.header)
        rospy.loginfo("convert")
        pose = geometry_msgs.msg.Pose()
        marker = visualization_msgs.msg.Marker(header=header)
        point_array = []
        for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            point_array.append(p)
        point_array = np.array(point_array)
        rospy.loginfo("{}".format(point_array.shape))
        if not np.any(np.isnan(point_array)):
            center = np.sum(point_array, axis=0) / len(point_array)
            rospy.loginfo(center)
            if not np.any(np.isnan(center)):
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2]
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                marker.pose = pose
                marker.type = 2
                marker.action = 0 
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0                
                self.pose_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('attention_estimation')
    AttentionEstimation()
    rospy.spin()
