#! /usr/bin/env python

import sys
import rospy
import distutils.util
from ast import literal_eval
import roboy_cognition_msgs.msg
from roboy_skill_machine.msg import SkillNode
from roboy_skill_machine.srv import StartSkill


# from roboy_communication_cognition.msg import *

# def start_skill_client(package, executable, node_name, continuous):
#    pub = rospy.Publisher("startSkill", StartSkillMsg)
#    rospy.init_node('start_skill_client', anonymous=True)
#    msg = StartSkillMsg()
#    msg.package = package
#    msg.executable = executable
#    msg.node_name = node_name
#    msg.continuous = continuous
#    pub.publish(msg)


def start_skill_client(skill_name, launch_package, launch_file, continuous, node_tuple):
    rospy.wait_for_service('start_skill')
    try:
        node_list = []
        for node in node_tuple:
            # node_msg = SkillNode()
            # node_msg.node_name = node[0]
            # node_msg.node_executable = node[1]
            # node_msg.node_package = node[2]

            # node_msg = SkillNode(node[0], node[1], node[2])

            node_msg = SkillNode(node_name=node[0], node_executable=node[1], node_package=node[2])
            node_list.append(node_msg)
        start_skill = rospy.ServiceProxy('start_skill', StartSkill)
        resp1 = start_skill(skill_name, launch_package, launch_file, continuous, node_list)
        return True
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [skill_name, launch_package, launch_file, continuous, node_list]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 6:
        skill_name = str(sys.argv[1])
        launch_package = str(sys.argv[2])
        launch_file = str(sys.argv[3])
        continuous = distutils.util.strtobool(sys.argv[4])
        node_tuple = literal_eval(sys.argv[5])
    else:
        print
        usage()
        sys.exit(1)
    start_skill_client(skill_name, launch_package, launch_file, continuous, node_tuple)
