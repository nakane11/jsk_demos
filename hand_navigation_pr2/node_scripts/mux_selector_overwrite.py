#!/usr/bin/env python
from std_msgs.msg import String
from topic_tools.srv import MuxSelect, MuxAdd, MuxDelete
import rospy


class MuxOverwrite():

    def __init__(self):
        self.before = '/teleop/cmd_vel'
        self.target_topic = rospy.get_param('~target', '/navigation/cmd_vel')
        rospy.wait_for_service('/vel_type_mux/select')
        rospy.wait_for_service('/vel_type_mux/add')        
        self.select_client = rospy.ServiceProxy('/vel_type_mux/select',MuxSelect)
        self.add_client = rospy.ServiceProxy('/vel_type_mux/add',MuxAdd)
        self.add_client('/navigation/cmd_vel_filtered')
        self.delete_client = rospy.ServiceProxy('/vel_type_mux/delete',MuxDelete)
        self.sub = rospy.Subscriber('/vel_type_mux/selected', String, self.cb)

    def cb(self, msg):
        if msg.data == self.target_topic and self.before != self.target_topic:
            self.select_client('/navigation/cmd_vel_filtered')
        self.before = msg.data

    def hook(self):
        self.delete_client('/navigation/cmd_vel_filtered')
    

if __name__ == "__main__":
    rospy.init_node('mux_selector_overwrite')
    mo = MuxOverwrite()
    rospy.on_shutdown(mo.hook)
    rospy.spin()
