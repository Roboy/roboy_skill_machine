import rospy
import actionlib
from roboy_control_msgs.msg import PerformMovementsAction, PerformMovementsGoal
from roboy_control_msgs.srv import PerformActions, PerformActionsResponse


rospy.init_node('actions_middlelayer')
shoulder_left_client = actionlib.SimpleActionClient('shoulder_left_movements_server', PerformMovementsAction)
rospy.loginfo("waiting for AS")
shoulder_left_client.wait_for_server()
rospy.loginfo("done")
def callback(req):
    rospy.set_param('trajectory_active', True)
    goal = PerformMovementsGoal(actions=req.actions)
    shoulder_left_client.send_goal(goal)
    shoulder_left_client.wait_for_result()
    response = PerformActionsResponse(True)
    rospy.set_param('trajectory_active', False)
    return response

s = rospy.Service('/roboy/control/PlayTrajectory', PerformActions, callback)
rospy.spin()
