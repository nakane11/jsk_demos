#!/usr/bin/env python
import geometry_msgs.msg
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import numpy as np

class AttentionEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(AttentionEstimation, self).__init__()
        self.pose_pub = self.advertise('~output',
                                       geometry_msgs.msg.Pose,
                                       queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', sensor_msgs.msg.PointCloud2, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        pose = geometry_msgs.msg.Pose()
        point_array = []
        for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            point_array.append(p)
        point_array = np.array(point_array)
        center = np.sum(point_array, axis=0) / len(point_array)
        if not np.any(np.isnan(center)):
            pose.position.x = center[0]
            pose.position.y = center[1]
            pose.position.z = center[2]
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            self.pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('attention_estimation')
    AttentionEstimation()
    rospy.spin()
