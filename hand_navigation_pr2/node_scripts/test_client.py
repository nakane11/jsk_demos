from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from hand_navigation_pr2.msg import HandNavigationAction
from hand_navigation_pr2.msg import HandNavigationGoal

rospy.init_node("test_client")
nav_client = actionlib.SimpleActionClient('hand_navigation_motion',
                                                HandNavigationAction)
nav_client.wait_for_server()
nav_client.send_goal(HandNavigationGoal(task_id=10))

nav_client.get_result()
nav_client.wait_for_result()

