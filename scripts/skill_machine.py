#! /usr/bin/env python

import rospy
import actionlib
import roslaunch
from bondpy import bondpy
import std_msgs.msg
import os
import subprocess
#import roboy_cognition_msgs.msg
from roboy_skill_machine.msg import SkillNode
from roboy_skill_machine.srv import LaunchSkill
from roboy_skill_machine.srv import ExecuteSkill
from roboy_skill_machine.srv import TerminateSkill
from roboy_communication_cognition.msg import *

# Class to hold all relevant data for skills
class Skill:
    def __init__(self, skill_name, launch_package, launch_file, continuous, node_list):
        self.skill_name = skill_name
        self.launch_package = launch_package
        self.launch_file = launch_file
        self.continuous = continuous
        self.node_name_list = []
        self.add_nodes(node_list)

    # Pulls all nodes provided by argument and attaches them to skill
    def add_nodes(self, node_list):
        for node in node_list:
#TODO check if this works
            if node.node_name not in node_dict:
                bond = create_bond(node.node_name + "_bond")
                new_node = Node(node.node_name, node.node_executable, node.node_package, node.node_machine, self.continuous, bond)
                node_dict[node.node_name] = new_node
            node_dict[node.node_name].skill_name_list.append(self.skill_name)
            #Ensures that if skill is continuous, node is also continuous, but also that skill doesn't 
            #override node continuousness if skill is non-continuous but node was instantiated by
            #another continuous skill that is still continuing
            node_dict[node.node_name].continuous = node_dict[node.node_name].continuous or self.continuous
            self.node_name_list.append(node.node_name)
        #Remove duplicates
        self.node_name_list = list(set(self.node_name_list))


# Class to hold all relevant data for nodes
class Node:
    def __init__(self, node_name, node_executable, node_package, node_machine, continuous, bond):
        self.node_name = node_name
        self.node_executable = node_executable
        self.node_package = node_package
        self.node_machine = node_machine
        self.continuous = continuous
        self.bond = bond
        self.skill_name_list = []

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__


# Runs the method to actually launch the skill, creates a new skill object for that skill, and adds it
# to the list of skills maintained by the skill machine
def handle_launch_skill(request):
    launch_skill(request.launch_package, request.launch_file)
    new_skill = Skill(request.skill_name, request.launch_package, request.launch_file, request.continuous, request.node_list)
    skill_dict[request.skill_name] = new_skill
    return 1


# Actually launches the skill by running the roslaunch command on the provided launch file
def launch_skill(package, launch_file):
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


#TODO Test
def handle_execute_skill(request):
    execute_skill(request.command)
    new_skill = Skill(request.skill_name, None, request.command, request.continuous, request.node_list)
    skill_dict[request.skill_name] = new_skill
    return 1


def execute_skill(command):
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


# Creates a bond between the skill machine and each node launched by the skill machine,
# so that the skill machine is aware if any node dies. From there, skill machine can attempt
# to restart the node or alert the user.
def create_bond(node_id):
    bond = bondpy.Bond("skill_machine_bonds", node_id)
    bond.start()
    #if not bond.wait_until_formed(rospy.Duration(5.0)):
    #    raise Exception('Bond could not be formed')
    return bond


# Method called to check on node status. Iterates through nodes to see which ones are no longer
# running. If they should be continuous, it attempts to restart them. If they should not be,
# it removes them from the skill machine, and from any skill object holding them. It then goes
# through the skills in the skill machine. Any skill that has no nodes running is removed from
# the skill machine.
def check_nodes_still_running():
    items_to_remove = []
    for node_name in node_dict:
        if (node_dict[node_name].bond.is_broken() and node_dict[node_name].continuous == True):
            restart_node(node_name)
        elif (node_dict[node_name].bond.is_broken() and node_dict[node_name].continuous == False):
            items_to_remove.append(node_name)
    for node_name in items_to_remove:
        remove_node(node_name)
    items_to_remove = []
    for skill_name in skill_dict:
        #Delete skills with no nodes running
        if not skill_dict[skill_name].node_name_list:
            items_to_remove.append(skill_name)
            #del skill_dict[skill_name]
    for skill_name in items_to_remove:
        del skill_dict[skill_name]


# Calls rosrun on a node, if the node is to be restarted
def restart_node(node_name):
#TODO Test functionality of restart nodes on remote machines
    node_dict[node_name].bond.shutdown()
    if (node_dict[node_name].node_machine == "local" or node_dict[node_name].node_machine == "localhost"):
        command = "rosrun {0} {1}".format(node_dict[node_name].node_package, node_dict[node_name].node_executable)
    else:
        command = "ssh {0} 'rosrun {1} {2}'".format(node_dict[node_name].node_machine, node_dict[node_name].node_package, node_dict[node_name].node_executable)
    p = subprocess.Popen(command, shell=True)
    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
        return False
    elif state > 0:
        rospy.loginfo("Process terminated without error")
    bond = create_bond(node_name + "_bond")
    node_dict[node_name].bond = bond


# Removes node from skill machine.
def remove_node(node_name):
    node_dict[node_name].bond.shutdown()
#TODO Add getters and setters to Skill class for nodes, to make code more readable
    for skill_name in skill_dict:
        if (node_name in skill_dict[skill_name].node_name_list):
            skill_dict[skill_name].node_name_list.remove(node_name)
    del node_dict[node_name]


#TODO Test
def handle_terminate_skill(request):
    terminate_skill(request.skill_name)
    return 1


def terminate_skill(skill_name):
    del_node_list = []
    for node_name in node_dict:
        if (skill_name in node_dict[node_name].skill_name_list):
            del_node_list.append(node_name)
    for node_name in del_node_list:
        node_dict[node_name].skill_name_list.remove(skill_name)
        if node_dict[node_name].skill_name_list == []:
            node_dict[node_name].bond.break_bond()
            node_dict[node_name].bond.shutdown()
            del node_dict[node_name]
#TODO Make this more accepting of other node formats
            command = "rosnode kill {0}".format("/" + node_name)
            p = subprocess.Popen(command, shell=True)
            state = p.poll()
            #if state is None:
            #    pass
            #elif state < 0:
            #if state < 0:
            #    rospy.loginfo("Process terminated with error")
            #    return False
            #elif state > 0:
            #    pass
    del skill_dict[skill_name]
    return True


def main():
    # Object to store all skills launched by skill machine
    global skill_dict
    skill_dict = {}
    #Object to store all nodes launched by skill machine
    global node_dict
    node_dict = {}
#TODO Boot up FPGAs and face, etc.
    rospy.init_node('roboy_skill_machine')
    s = rospy.Service('launch_skill', LaunchSkill, handle_launch_skill)
    t = rospy.Service('terminate_skill', TerminateSkill, handle_terminate_skill)
    e = rospy.Service('execute_skill', ExecuteSkill, handle_execute_skill)
    # Runs the check nodes method every 10 seconds
    while not rospy.is_shutdown():
        rospy.sleep(10.)
        check_nodes_still_running()
    rospy.spin()

   


if __name__ == '__main__':
    main()
