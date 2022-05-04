import rospy
import actionlib
from hand_navigation_pr2.msg import HandNavigationAction
from hand_navigation_pr2.msg import HandNavigationGoal

rospy.init_node("give_hand_init_client")
nav_client = actionlib.SimpleActionClient('give_hand_init',
                                                HandNavigationAction)
nav_client.wait_for_server()
nav_client.send_goal(HandNavigationGoal(task_id=10))

print(nav_client.get_result())


