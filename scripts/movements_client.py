#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from roboy_control_msgs.msg import PerformMovementAction,PerformMovementsAction,PerformMovementGoal,PerformMovementsGoal
from roboy_control_msgs.srv import PerformActions, PerformActionsResponse

class MovementAC():
    def __init__(self):
        shoulder_left_client = actionlib.SimpleActionClient('shoulder_left_movements_server', PerformMovementsAction)
        shoulder_right_client = actionlib.SimpleActionClient('shoulder_right_movements_server', PerformMovementsAction)

        self.clients = [shoulder_left_client, shoulder_right_client]
        self.moves = {}
        for c in self.clients:
            rospy.loginfo("Waiting for %s action server to show up..."%c.action_client.ns)
            c.wait_for_server()
            self.moves[c] = []
        rospy.loginfo("Connected")    

        
        self.move_src = rospy.Service('/roboy/control/PerformMovements', PerformActions, self.cb)  

    def cb(self,req):
        for m in req.actions:
            if "left" in m:
                self.moves[self.clients[0]].append(m)
            elif "right" in m:
                self.moves[self.clients[1]].append(m)

        for c in self.clients:
            if len(self.moves[c]) > 0:
                goal = PerformMovementsGoal(actions=self.moves[c])
                c.send_goal(goal)

        for c in self.clients:
            if len(self.moves[c]) > 0:
                c.wait_for_result()

        success = True
        for c in self.clients:
            if len(self.moves[c]) > 0:
                success = success and c.get_result().success   
            self.moves[c] = []      

        response = PerformActionsResponse(success)

        return response


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('movement_ac_py')
        ac = MovementAC()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")