#!/usr/bin/env python
# -*- coding: utf-8 -*-
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from hand_navigation_pr2.msg import InitRequest
from hand_navigation_pr2.msg import VelocityFilterAction, VelocityFilterGoal
from hand_navigation_pr2.msg import GoRequest
from std_msgs.msg import String, Bool

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
        self.voice_thread.setDaemon(True)
        self.result_thread.setDaemon(True)

        self.velocity_client = actionlib.SimpleActionClient('velocity_filter', VelocityFilterAction)
        self.velocity_client.wait_for_server()

        self.go_pub = rospy.Publisher('/go/request', GoRequest, queue_size=1)
        self.init_pub = rospy.Publisher('/init/request', InitRequest, queue_size=1)
        self.init_cancel_pub = rospy.Publisher('/init/cancel', InitRequest, queue_size=1)

        self.voice_sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.voice_cb)

        self.go_sub = rospy.Subscriber('/go/result', String, self.result_cb)
        self.init_sub = rospy.Subscriber('/init/result', String, self.result_cb)
        self.wrench_sub = rospy.Subscriber('/grasped', Bool, self.wrench_cb)
        rospy.loginfo("[main] init ended")
        
    def reset_params(self):
        self.is_wait_for_reply = False
        self.is_wait_for_grasp = False
        self.is_wait_for_destination = False
        self.navigation = False
        self.inturrupt = False
        self.is_grasped = False
        self.nav_requested = False
        self.destination = None
        rospy.loginfo("[main] reset params ended")        

    def voice_cb(self, msg):
        rospy.loginfo("[main] voice_cb get {}".format(msg.transcript[0]))        
        self.voice_q.put(msg.transcript[0])
        
    def result_cb(self, msg):
        rospy.loginfo("[main] result_cb get {}".format(msg.data))                
        self.result_q.put(msg.data)

    def wrench_cb(self, msg):
        rospy.loginfo("[main] wrench_cb get {}".format(msg.data))                
        self.is_grasped = msg.data
        

    def voice_loop(self):
        while not rospy.is_shutdown():
            text = self.voice_q.get()
            rospy.loginfo("[main] pull {} from voice_q".format(text))                    
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
                    rospy.loginfo("もう一度")

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
                    rospy.loginfo("もう一度")

            elif self.navigation:
                if text == "速い":
                    goal = VelocityFilterGoal(calculation = 2, rate = 0.2)
                    self.velocity_client.send_goal(goal)
                    res = self.velocity_client.get_result()
                    rospy.loginfo("current speed:{}".format(res.speed))
                    #変更しました
#                    res.speed
                elif text == "遅い":
                    goal = VelocityFilterGoal(calculation = 1, rate = 0.2)
                    self.velocity_client.send_goal(goal)
                    res = self.velocity_client.get_result()
                elif text == "止まって":
                    self.go_pub.publish(GoRequest(request = 2))
                    self.navigation = False
                else:
                    rospy.loginfo("else")

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
                    rospy.loginfo("わからない")
                    
            self.voice_q.task_done()

    def result_loop(self):
        while not rospy.is_shutdown():
            self.voice_q.join()
            result = self.result_q.get()
            rospy.loginfo("[main] pull {} from result_q".format(result))
            if result=="test":
                if self.wait_for_reply():
                    if self.reply:
                        rospy.loginfo("get reply")
                        if self.wait_for_grasp():
                            self.navigation = True
                            self.go_pub.publish(GoRequest(request = 1, destination = self.destination))
            
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
                print("到着しました")
                self.navigation = False
            elif result == "move-base failed":
                print("失敗しました")
                                      
            self.result_q.task_done()
    
    def wait_for_reply(self):
        print("[main] wait_for_reply start")
        self.inturrupt = False
        self.is_wait_for_reply = True
        while self.is_wait_for_reply:
            if self.inturrupt:
                self.inturrupt = False
                rospy.loginfo("[main] wait_for_reply inturrupt")                
                return False
            else:
                continue
        print("[main] wait_for_reply end")            
        return True

    def wait_for_grasp(self):
        rospy.loginfo("[main] wait_for_grasp start")
        #つかまってください
        self.inturrupt = False
        self.is_wait_for_grasp = True
        while not self.is_grasped:
            if self.inturrupt:
                self.inturrupt = False
                rospy.loginfo("[main] wait_for_grasp inturrupt")
                return False
            else:
                continue
        rospy.loginfo("[main] wait_for_grasp end")        
        return True

    def wait_for_destination(self):
        rospy.loginfo("[main] wait_for_destination start")
        #どこへ行きますか
        self.inturrupt = False
        self.is_wait_for_destination = True
        while self.is_wait_for_destination:
            if self.inturrupt:
                self.inturrupt = False
                rospy.loginfo("[main] wait_for_destination inturrupt")                
                return False
            else:
                continue
        rospy.loginfo("[main] wait_for_destination end")        
        return True
                                        
    def run(self):
        self.voice_thread.start()
        self.result_thread.start()
        rospy.loginfo("[main] threads start")        
        
if __name__ == '__main__':
    rospy.init_node('hand_navigation')
    hn = HandNavigation()
    hn.run()
    rospy.spin()
