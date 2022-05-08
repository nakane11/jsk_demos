#!/usr/bin/env python3

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from std_msgs.msg import String

import pykakasi

class SpeechToRoman(ConnectionBasedTransport):

    def __init__(self):
        super(SpeechToRoman, self).__init__()
        self.pub = self.advertise("~output", String, queue_size=1)

        self.kks = pykakasi.kakasi()
        self.kks.setMode('J', 'a')
        self.kks.setMode('H', 'a')
        self.kks.setMode('K', 'a')
        self.converter = self.kks.getConverter()
                    
    def subscribe(self):
        self.sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        text = msg.transcript[0]
        roman_text = self.converter.do(text)
        self.pub.publish(String(data = roman_text))
        rospy.loginfo("{}[{}]".format(text, roman_text))

if __name__ == '__main__':
    rospy.init_node('speech_to_roman')
    SpeechToRoman()
    rospy.spin()
