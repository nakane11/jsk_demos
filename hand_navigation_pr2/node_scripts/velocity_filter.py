#!/usr/bin/env python
from hand_navigation_pr2.msg import VelocityFilterAction, VelocityFilterResult
from geometry_msgs.msg import Twist
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import actionlib

class VelocityFilter(ConnectionBasedTransport):

    def __init__(self):
        super(VelocityFilter, self).__init__()
        self.speed = 1.0
        self.server = actionlib.SimpleActionServer('velocity_filter', VelocityFilterAction, self.action_cb)
        self.pub = self.advertise('~output', Twist, queue_size=1)
        
    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Twist, self.subscribe_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def subscribe_cb(self, msg):
        filtered_msg = Twist()
        filtered_msg.linear.x = msg.linear.x * self.speed
        filtered_msg.linear.y = msg.linear.y * self.speed
        filtered_msg.linear.z = msg.linear.z * self.speed        
        filtered_msg.angular.x = msg.angular.x * self.speed
        filtered_msg.angular.y = msg.angular.y * self.speed
        filtered_msg.angular.z = msg.angular.z * self.speed
        self.pub.publish(filtered_msg)
    
    def action_cb(self, goal):
        self.set_speed(goal)
        result = VelocityFilterResult(speed = self.speed)
        self.server.set_succeeded(result)

    def set_speed(self, goal):
        if goal.calculation == 0:
            self.speed = goal.rate
        elif goal.calculation == 1:
            self.speed += goal.rate
        elif goal.calculation == 2:
            self.speed -= goal.rate
        elif goal.calculation == 3:
            self.speed *= goal.rate
        elif goal.calculation == 4:
            self.speed /= goal.rate
        
if __name__ == '__main__':
    rospy.init_node('velocity_filter')
    VelocityFilter()
    rospy.spin()
