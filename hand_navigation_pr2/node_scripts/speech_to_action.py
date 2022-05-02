#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_topic_tools import ConnectionBasedTransport
import rospy


class SpeechSToAction(ConnectionBasedTransport):

    def __init__(self):
        super(SpeechToAction, self).__init__()
     
    def subscribe(self):
        self.sub = rospy.Subscriber('~input', SpeechRecognitionCandidates, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def cb(self, msg):
        speech = msg.transcript
        
        if speech == "":

            

if __name__ == '__main__':
    rospy.init_node('speech_to_action')
    SpeechToAction()
    rospy.spin()
