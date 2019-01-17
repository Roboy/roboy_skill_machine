#! /usr/bin/env python

import sys
import rospy
import distutils.util
from roboy_cognition_msgs.msg import *
from roboy_skill_machine.srv import StartSkill
#from roboy_communication_cognition.msg import *

#def start_skill_client(package, executable, node_name, continuous):
#    pub = rospy.Publisher("startSkill", StartSkillMsg)
#    rospy.init_node('start_skill_client', anonymous=True)
#    msg = StartSkillMsg()
#    msg.package = package
#    msg.executable = executable
#    msg.node_name = node_name
#    msg.continuous = continuous
#    pub.publish(msg)
    

def start_skill_client(package, executable, node_name, continuous):
    rospy.wait_for_service('start_skill')
    try:
        start_skill = rospy.ServiceProxy('start_skill', StartSkill)
        resp1 = start_skill(package, executable, node_name, continuous)
        return True
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [package, executable, node_name, continuous]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        package = str(sys.argv[1])
        executable = str(sys.argv[2])
        node_name = str(sys.argv[3])
        continuous = distutils.util.strtobool(sys.argv[4])
    else:
        print usage()
        sys.exit(1)
    start_skill_client(package, executable, node_name, continuous)
