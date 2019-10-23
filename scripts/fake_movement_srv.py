#!/usr/bin/python
import rospy
import actionlib
from roboy_control_msgs.msg import PerformMovementAction, PerformMovementsAction
import time


class RoboyActions(object):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PerformMovementsAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):

 		rospy.loginfo(goal.actions)
 		self._as.set_succeeded()

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
       
        

rospy.init_node('fake_movement')
server1 = RoboyAction("shoulder_left_movement_server")
server2 = RoboyAction("shoulder_right_movement_server")
server3 = RoboyActions("shoulder_left_movements_server")
server4 = RoboyActions("shoulder_right_movements_server")
rospy.spin()