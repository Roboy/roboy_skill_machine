import rospy
import actionlib
from roboy_control_msgs.msg import PerformMovementAction
import time

class RoboyAction(object):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PerformMovementAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
 		rospy.loginfo("Started hug")
 		time.sleep(1)
 		rospy.loginfo("Finished hug")
 		self._as.set_succeeded()
       
        

rospy.init_node('fibonacci')
server1 = RoboyAction("shoulder_left_movement_server")
server2 = RoboyAction("shoulder_right_movement_server")
rospy.spin()