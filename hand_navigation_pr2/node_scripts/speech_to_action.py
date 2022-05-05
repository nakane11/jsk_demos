#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from hand_navigation_pr2.msg import SpeechAction, SpeechGoal
import rospy
import actionlib

word_list = ["一緒に歩こう", "手をつなごう", "テスト"]

class SpeechToAction():

    def __init__(self):
        self.client = actionlib.SimpleActionClient('hand_navigation', SpeechAction)
        self.client.wait_for_server()
        self.sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.subscribe_cb)

    def subscribe_cb(self, msg):
        transcript = msg.transcript[0]
        if transcript in word_list:
            goal = SpeechGoal(transcript=transcript)
            self.client.send_goal(goal, self.feedback_cb)
            self.client.wait_for_result()
            self.client.get_result()

    def feedback_cb(self, msg):
        rospy.loginfo(msg.status)
            
if __name__ == '__main__':
    rospy.init_node('speech_to_action')
    SpeechToAction()
    rospy.spin()
