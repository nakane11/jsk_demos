#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from hand_navigation_pr2.msg import InitRequest
from hand_navigation_pr2.msg import VelocityFilterAction, VelocityFilterGoal
from hand_navigation_pr2.msg import GoRequest
import std_msgs.msg import String, Bool

import rospy
import actionlib
import queue
import threading

class HandNavigation():

    def __init__(self):
        self.reset_params()
        
        self.voice_q = queue.Queue()
        self.result_q = queue.Queue()

        self.voice_thread = threading.Thread(target = self.voice_loop)
        self.result_thread = threading.Thread(target = self.result_loop)

        self.velocity_client = actionlib.SimpleActionClient('velocity_filter', VelocityFilterAction)
        self.velocity_client.wait_for_server()

        self.go_pub = rospy.Publisher('/go/request', GoRequest, 1)
        self.init_pub = rospy.Publisher('/init/request', InitRequest, 1)
        self.init_cancel_pub = rospy.Publisher('/init/cancel', InitRequest, 1)

        self.voice_sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.voice_cb)
        self.go_sub = rospy.Subscriber('/go/result', String, self.result_cb)
        self.init_sub = rospy.Subscriber('/init/result', String, self.result_cb)
        self.wrench_sub = rospy.Subscriber('', Bool, self.wrench_cb)

    def reset_params(self):
        self.is_wait_for_reply = False
        self.is_wait_for_grasp = False
        self.is_wait_for_destination = False
        self.navigation = False
        self.inturrupt = False
        self.is_grasped = False
        self.destination = None

    def voice_cb(self, msg):
        self.voice_q.put(msg.transcript[0])
        
    def result_cb(self, msg):
        self.result_q.put(msg.data)

    def wrench_cb(self, msg):
        self.is_grasped = msg.data
        

    def voice_loop(self):
        while not rospy.is_shutdown():
            text = self.voice_q.get()

            if text == "リセット":
                self.init_cancel_pub.publish(InitRequest())
                self.init_pub.publish(InitRequest(request = 0))
                self.go_pub.publish(GoRequest(request = 0))
                self.velocity_client.send_goal(VelocityFilterGoal(calculation = 0, rate = 1.0))
                self.reset_params()
            
            elif self.is_wait_for_reply:
                if text == "はい":
                    self.reply = True
                    self.is_wait_for_reply = False
                elif text == "いいえ":
                    self.reply = False
                    self.is_wait_for_reply = False
                elif text == "キャンセル":
                    self.inturrupt = True
                    self.is_wait_for_reply = False
                else:
                    #もう一度

            elif self.is_wait_for_grasp:
                if text == "キャンセル":
                    self.inturrupt = True
                    self.is_wait_for_grasp = False

            elif self.is_wait_for_destination:
                if text == "センター":
                    self.destination = "center"
                    self.is_wait_for_destination = False
                elif text == "キャンセル":
                    self.inturrupt = True
                    self.is_wait_for_destination = False
                else:
                    #もう一度

            elif self.navigation:
                if text == "速い":
                    goal = VelocityFilterGoal(calculation = 2, rate = 0.2)
                    self.velocity_client.send_goal(goal)
                    res = self.velocity_client.get_result()
                    #すみません　変更しました
                    res.speed
                elif text == "遅い":
                    goal = VelocityFilterGoal(calculation = 1, rate = 0.2)
                    self.velocity_client.send_goal(goal)
                    res = self.velocity_client.get_result()
                elif text == "止まって":
                    self.go_pub.publish(GoRequest(request = 2))
                    self.navigation = False
                else:
                    #わからない

            else:
                if text == "進んで":
                    if self.wait_for_grasp():
                        self.go_pub.publish(GoRequest(request = 3))
                        self.navigation = True

                elif text == "手をつなごう":
                    self.nav_requested = False
                    if rospy.has_param("/target_arm"):
                        rospy.delete_param("/target_arm")
                    self.init_pub.publish(InitRequest(request = 1))
                    
                elif text == "一緒に歩こう":
                    self.nav_requested = True
                    if not self.is_grasped:
                        if rospy.has_param("/target_arm"):
                            rospy.delete_param("/target_arm")
                    self.init_pub.publish(InitRequest(request = 1))
                        
                elif text == "センターに連れて行って":
                    self.destination = "center"
                    self.nav_requested = True
                    if not self.is_grasped:
                        if rospy.has_param("/target_arm"):
                            rospy.delete_param("/target_arm")
                    self.init_pub.publish(InitRequest(request = 1))
                
                elif text == "キャンセル":
                    self.init_cancel_pub.publish(InitRequest())

                else:
                    #わからない
                    
            self.voice_q.task_done()

    def result_loop(self):
        while not rospy.is_shutdown():
            self.voice_q.join()
            result = self.result_q.get()
            
            if result == "set shoulder-width":
                self.init_pub.publish(InitRequest(request = 2))
            elif result == "set target-arm":
                self.init_pub.publish(InitRequest(request = 3))
            elif result == "published boundingbox":
                self.init_pub.publish(InitRequest(request = 4))
            elif result == "resized footprint":
                if self.destination:
                    if self.nav_requested:
                        #~へ行きます
                        if self.wait_for_grasp():
                            self.navigation = True
                            self.go_pub.publish(GoRequest(request = 1, destination = self.destination))
                    else:
                        #~へ行きますか
                        if self.wait_for_reply():
                            if self.reply:
                                if self.wait_for_grasp():
                                    self.navigation = True
                                    self.go_pub.publish(GoRequest(request = 1, destination = self.destination))
                elif self.nav_requested:
                    if self.wait_for_destination():
                        #~へ行きます
                        if self.wait_for_grasp():
                            self.navigation = True
                            self.go_pub.publish(GoRequest(request = 1, destination = self.destination))
                                      

            elif result == "move-base succeeded":
                #到着しました
                self.navigation = False
            elif result == "move-base failed":
                #失敗しました
                                      
            self.result_q.task_done()
    
    def wait_for_reply(self):
        self.inturrupt = False
        self.is_wait_for_reply = True
        while self.is_wait_for_reply:
            if self.inturrupt:
                self.inturrupt = False
                return False
            else:
                continue
        return True

    def wait_for_grasp(self):
        #つかまってください
        self.inturrupt = False
        self.is_wait_for_grasp = True
        while not self.is_grasped:
            if self.inturrupt:
                self.inturrupt = False
                return False
            else:
                continue
        return True

    def wait_for_destination(self):
        #どこへ行きますか
        self.inturrupt = False
        self.is_wait_for_destination = True
        while self.is_wait_for_destination:
            if self.inturrupt:
                self.inturrupt = False
                return False
            else:
                continue
        return True
                                        
    def run(self):
        self.voice_thread.start()
        self.result_thread.start()
        
if __name__ == '__main__':
    rospy.init_node('hand_navigation')
    hn = HandNavigation()
    hn.run()
    rospy.spin()
