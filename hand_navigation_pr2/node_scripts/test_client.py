import rospy
import actionlib
from hand_navigation_pr2.msg import GiveHandInitAction
from hand_navigation_pr2.msg import GiveHandInitGoal

rospy.init_node("give_hand_init_client")
nav_client = actionlib.SimpleActionClient('give_hand_init',
                                                GiveHandInitAction)
nav_client.wait_for_server()
nav_client.send_goal(GiveHandInitGoal(task_id=0))

print(nav_client.get_result())


