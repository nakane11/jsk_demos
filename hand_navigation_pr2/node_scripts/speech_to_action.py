#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import rospy
# import actionlib
# import tf
# from nav_msgs.msg import Odometry
# import math
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal


class SpeechToAction():

    def __init__(self):
        # self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.move_base_client.wait_for_server()
        # self.move_base_trajectry_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.move_base_trajectry_client.wait_for_server()

        self.sub = rospy.Subscriber('~input', SpeechRecognitionCandidates, self.cb)

    def cb(self, msg):
        speech = msg.transcript[0]
        print(speech)
        # if speech == "ストップ":
        #     self.move_base_client.cancel_all_goals()
        #     self.move_base_trajectry_client.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('speech_to_action')
    SpeechToAction()
    rospy.spin()
