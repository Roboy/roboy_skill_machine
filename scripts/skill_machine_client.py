#! /usr/bin/env python

import sys
import rospy
import distutils.util
from ast import literal_eval
#import roboy_cognition_msgs.msg
from roboy_skill_machine.msg import SkillNode
from roboy_skill_machine.srv import LaunchSkill
from roboy_skill_machine.srv import TerminateSkill
from roboy_skill_machine.srv import ExecuteSkill
#from roboy_communication_cognition.msg import *
    

def launch_skill_client(skill_name, launch_package, launch_file, continuous, node_tuple):
    rospy.wait_for_service('launch_skill')
    try:
        node_list = []
        for node in node_tuple:
            node_msg = SkillNode(node_name=node[0], node_executable=node[1], node_package=node[2], node_machine=node[3])
            node_list.append(node_msg)
        launch_skill = rospy.ServiceProxy('launch_skill', LaunchSkill)
        resp1 = launch_skill(skill_name, launch_package, launch_file, continuous, node_list)
        return True
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def terminate_skill_client(skill_name):
    rospy.wait_for_service('terminate_skill')
    try:
        terminate_skill = rospy.ServiceProxy('terminate_skill', TerminateSkill)
        resp1 = terminate_skill(skill_name)
        return True
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def execute_skill_client(skill_name, command, continuous, node_tuple):
    rospy.wait_for_service('execute_skill')
    try:
        node_list = []
        for node in node_tuple:
            node_msg = SkillNode(node_name=node[0], node_executable=node[1], node_package=node[2], node_machine=node[3])
            node_list.append(node_msg)
        execute_skill = rospy.ServiceProxy('execute_skill', ExecuteSkill)
        resp1 = execute_skill(skill_name, command, continuous, node_list)
        return True
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def usage():
    return "Incorrect command or formatting. Please see the appropriate use one of the following:\n\n*Launch from launch file:\nlaunch [skill name] [launch file package] [launch file] [continuous] [node: ([name] [executable] [package] [machine])]\n\n*Execute from executable file:\nexecute [skill name] [command] [continuous] [node: ([name] [executable] [package] [machine])]\n\n*Terinate skill:\nterminate [skill name]"

if __name__ == "__main__":
    if (len(sys.argv) == 7 and str(sys.argv[1]) == "launch"):
        #command = str(sys.argv[1])
        skill_name = str(sys.argv[2])
        launch_package = str(sys.argv[3])
        launch_file = str(sys.argv[4])
        continuous = distutils.util.strtobool(sys.argv[5])
        node_tuple = literal_eval(sys.argv[6])
        launch_skill_client(skill_name, launch_package, launch_file, continuous, node_tuple)
    elif (len(sys.argv) == 3 and str(sys.argv[1]) == "terminate"):
        skill_name = str(sys.argv[2])
        terminate_skill_client(skill_name)
    elif (len(sys.argv) == 6 and str(sys.argv[1]) == "execute"):
        skill_name = str(sys.argv[2])
        command = str(sys.argv[3])
        continuous = distutils.util.strtobool(sys.argv[4])
        node_tuple = literal_eval(sys.argv[5])
        execute_skill_client(skill_name, command, continuous, node_tuple)
    else:  
        print usage()
        sys.exit(1)
