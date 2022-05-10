#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from hand_navigation_pr2.msg import GiveHandInitAction, GiveHandInitGoal
from hand_navigation_pr2.msg import VelocityFilterAction, VelocityFilterGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
import rospy
import actionlib

class HandNavigation():

    def __init__(self):
        self.reset_params()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_trajectry_client = actionlib.SimpleActionClient('move_base', FollowJointTrajectoryAction)
        self.init_client = actionlib.SimpleActionClient('give_hand_init', GiveHandInitAction)
        self.velocity_client = actionlib.SimpleActionClient('velocity_filter', VelocityFilterAction)

        self.move_base_client.wait_for_server()
        self.move_base_trajectry_client.wait_for_server()
        self.init_client.wait_for_server()
        self.velocity_client.wait_for_server()

        self.sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.cb)
        
    def reset_params(self):
        if rospy.has_param("/shoulder_width"):
            rospy.delete_param("/shoulder_width")
        if rospy.has_param("/target-arm"):
            rospy.delete_param("/target-arm")
        rospy.loginfo("Reset rosparams")

    def cancel_goals(self):
        self.init_client.cancel_all_goals()
        self.velocity_client.cancel_all_goals()
        self.move_base_client.cancel_all_goals()
        self.move_base_trajectry_client.cancel_all_goals()
        rospy.loginfo("Cancel all goals")
        
    def cb(self, msg):
        transcript = msg.transcript[0]
        
        if transcript == "リセット":
            self.cancel_goals()
            self.reset_params()

        elif transcript == "":
            goal = VelocityFilterGoal()
            goal.calculation = 3
            goal.rate = 0.8
            self.velocity_client.send_goal(goal)
                                    
        elif transcript == "一緒に歩こう":
            goal = GiveHandInitGoal()
            goal.task_id = 
            

            # self.client.send_goal(goal, self.feedback_cb)
            # self.client.wait_for_result()
            # self.client.get_result()

    # def feedback_cb(self, msg):
    #     rospy.loginfo(msg.status)
            
if __name__ == '__main__':
    rospy.init_node('hand_navigation')
    HandNavigation()
    rospy.spin()
