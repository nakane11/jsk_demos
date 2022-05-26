#!/usr/bin/env python
import numpy as np
import math
import rospy
import actionlib
from std_msgs.msg import Float32MultiArray
from hand_navigation_pr2.msg import ServoAction, ServoResult

class ServoClient:
    def __init__(self, min_angle, rate):
        self.min_angle = min_angle
        self.array = np.array([90, 89, 87, 90, 0])
        self.pub = rospy.Publisher("servo_angle_raw", Float32MultiArray, queue_size = 1)
        self.rate = rospy.Rate(rate)

    def send_angle(self, target_angle):
        target_array = np.array(target_angle)
        max_diff = np.max(np.abs(target_array - self.array))
        div_num = int(math.ceil(max_diff/self.min_angle))
        angle_list = self.interpolation(self.array, target_array, div_num)
        for segment_angle in angle_list:
            msg = Float32MultiArray(data = segment_angle)
            self.pub.publish(msg)
            self.rate.sleep()
        if len(angle_list) > 1:
            self.array = np.array(angle_list[-1])

    def interpolation(self, initial, target, div_num):
        ret = np.zeros((div_num, initial.size))
        for i in range(initial.size):
            for j in range(div_num): 
                ret[j][i] = math.ceil(initial[i]+(target[i]-initial[i])/div_num * (j+1))
        return ret.tolist()

def cb(goal):
    print(goal.angle_array)
    client.send_angle(goal.angle_array)
    server.set_succeeded(ServoResult())
    
rospy.init_node("servo_interpolation")
client = ServoClient(2, 40)
server = actionlib.SimpleActionServer("servo_interpolation", ServoAction, execute_cb=cb)
server.start()
rospy.spin()
