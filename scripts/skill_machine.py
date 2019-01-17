#! /usr/bin/env python

import rospy
import actionlib
import roslaunch
from bondpy import bondpy
import std_msgs.msg
import os
import subprocess
import roboy_cognition_msgs.msg
from roboy_skill_machine.srv import StartSkill

class Skill:
    def __init__(self, package, executable, node_name, continuous, process, bond):
        self.package = package
        self.executable = executable
        self.node_name = node_name
        self.continuous = continuous
        self.process = process
        self.bond = bond

def check_nodes_still_running():
    for i in range(len(skill_list)):
        if (skill_list[i].bond.is_broken() and skill_list[i].continuous == True):
            restart_skill(i)
        elif (skill_list[i].bond.is_broken() and skill_list[i].continuous == False):
            remove_skill(i)


def handle_start_skill(request):
    process = start_skill(request.package, request.executable)
    bond = create_bond(request.node_name + "_bond")
    new_skill = Skill(request.package, request.executable, request.node_name, request.continuous, process, bond)
    skill_list.append(new_skill)
    return 1


def start_skill(package, launch_file):
    command = "roslaunch {0} {1}".format(package, launch_file)
    p = subprocess.Popen(command, shell=True)
    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
        return False
    elif state > 0:
        rospy.loginfo("Process terminated without error")
    return True


def remove_skill(position):
    skill_list[position].bond.shutdown()
    del skill_list[position]


def terminate_skill(package, executable, node_name):
    for i in range(len(skill_list)):
        if (node_name == skill_list[i].node_name):
            skill_list[i].process.stop()
            bond_list[i].bond.shutdown()
            del skill_list[i]


def create_bond(node_id):
    bond = bondpy.Bond("skill_machine_bonds", node_id)
    bond.start()
    #if not bond.wait_until_formed(rospy.Duration(5.0)):
    #    raise Exception('Bond could not be formed')
    return bond


def handle_terminate_skill(request):
    terminate_skill(request.package, request.executable, request.node_name)


def restart_skill(position):
    skill_list[position].bond.shutdown()
    process = start_skill(skill_list[position].package, skill_list[position].executable, skill_list[position].node_name)
    bond = create_bond(skill_list[position].node_name + "_bond")
    skill_list[position].process = process
    skill_list[position].bond = bond


def main():
    global skill_list
    skill_list = []
    rospy.init_node('roboy_skill_machine')
    s = rospy.Service('start_skill', StartSkill, handle_start_skill)
    #t = rospy.Service('terminate_skill', TerminateSkill, handle_terminate_skill)
    while not rospy.is_shutdown():
        rospy.sleep(10.)
        check_nodes_still_running()
    rospy.spin()

   


if __name__ == '__main__':
    main()
