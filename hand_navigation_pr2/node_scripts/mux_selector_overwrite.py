#!/usr/bin/env python

import signal

from std_msgs.msg import String
from topic_tools.srv import MuxSelect, MuxAdd, MuxDelete
import rospy

class MuxOverwrite():
    def __init__(self):
        # rospy.init_node('mux_selector_overwrite', disable_signals=False)
        rospy.init_node('mux_selector_overwrite')
        self.before = '/teleop/cmd_vel'
        self.target_topic = rospy.get_param('~target', '/navigation/cmd_vel')
        rospy.wait_for_service('/vel_type_mux/select')
        rospy.wait_for_service('/vel_type_mux/add')        
        self.select_client = rospy.ServiceProxy('/vel_type_mux/select',MuxSelect)
        self.add_client = rospy.ServiceProxy('/vel_type_mux/add',MuxAdd)
        self.add_client('/navigation/cmd_vel_filtered')
        self.delete_client = rospy.ServiceProxy('/vel_type_mux/delete',MuxDelete)
        self.sub = rospy.Subscriber('/vel_type_mux/selected', String, self.cb)
        rospy.on_shutdown(self.hook)

    def cb(self, msg):
        if msg.data == self.target_topic and self.before != self.target_topic:
            self.select_client('/navigation/cmd_vel_filtered')
        self.before = msg.data

    def hook(self):
        self.delete_client('/navigation/cmd_vel_filtered')

    # def hook(self, signal=None, frame=None):
    #     self.delete_client('/navigation/cmd_vel_filtered')

    # def run(self):
    #     rate = rospy.Rate(10)
    #     # while True:
    #     #     rate.sleep()
    #     while not rospy.is_shutdown():
    #         rate.sleep()
                
if __name__ == "__main__":
    # rospy.init_node('mux_selector_overwrite', disable_signals=True)
    # mo = MuxOverwrite()
    # signal.signal(signal.SIGINT, mo.hook)
    # mo.run()
    # rospy.init_node('mux_selector_overwrite', disable_signals=False)
    mo = MuxOverwrite()
    rospy.spin()
