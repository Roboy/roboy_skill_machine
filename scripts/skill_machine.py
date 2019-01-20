#! /usr/bin/env python

import rospy
import actionlib
import roslaunch
from bondpy import bondpy
import std_msgs.msg
import os
import subprocess
import roboy_cognition_msgs.msg
from roboy_skill_machine.msg import SkillNode
from roboy_skill_machine.srv import StartSkill


class Skill:
    def __init__(self, skill_name, launch_package, launch_file, continuous, node_list):
        self.skill_name = skill_name
        self.launch_package = launch_package
        self.launch_file = launch_file
        self.continuous = continuous
        self.node_name_list = []
        self.add_nodes(node_list)

    def add_nodes(self, node_list):
        for node in node_list:
            #TODO check if this works
            if node.node_name not in node_dict:
                bond = create_bond(node.node_name + "_bond")
                new_node = Node(node.node_name, node.node_executable, node.node_package, self.continuous, bond)
                node_dict[node.node_name] = new_node
            node_dict[node.node_name].skill_name_list.append(self.skill_name)
            #Ensures that if skill is continuous, node is also continuous, but also that skill doesn't 
            #override node continuousness if skill is non-continuous but node was instantiated by
            #another continuous skill that is still continuing
            node_dict[node.node_name].continuous = node_dict[node.node_name].continuous or self.continuous
            self.node_name_list.append(node.node_name)
        #Remove duplicates
        self.node_name_list = list(set(self.node_name_list))



class Node:
    def __init__(self, node_name, node_executable, node_package, continuous, bond):
        self.node_name = node_name
        self.node_executable = node_executable
        self.node_package = node_package
        self.continuous = continuous
        self.bond = bond
        self.skill_name_list = []

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__



def handle_start_skill(request):
    launch_skill(request.launch_package, request.launch_file)
    new_skill = Skill(request.skill_name, request.launch_package, request.launch_file, request.continuous, request.node_list)
    skill_dict[request.skill_name] = new_skill
    return 1


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


def create_bond(node_id):
    bond = bondpy.Bond("skill_machine_bonds", node_id)
    bond.start()
    #if not bond.wait_until_formed(rospy.Duration(5.0)):
    #    raise Exception('Bond could not be formed')
    return bond


def check_nodes_still_running():
    items_to_remove = []
    for node_name in node_dict:
        if (node_dict[node_name].bond.is_broken() and node_dict[node_name].continuous == True):
            restart_node(node_name)
        elif (node_dict[node_name].bond.is_broken() and node_dict[node_name].continuous == False):
            #remove_node(node_name)
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


def restart_node(node_name):
    node_dict[node_name].bond.shutdown()
    command = "rosrun {0} {1}".format(node_dict[node_name].node_package, node_dict[node_name].node_executable)
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


def remove_node(node_name):
    node_dict[node_name].bond.shutdown()
    #TODO Add getters and setters to Skill class for nodes, to make code more readable
    for skill_name in skill_dict:
        if (node_name in skill_dict[skill_name].node_name_list):
            skill_dict[skill_name].node_name_list.remove(node_name)
    del node_dict[node_name]

#def handle_terminate_skill(request):
#    terminate_skill(request.package, request.executable, request.node_name)


#def terminate_skill(package, executable, node_name):
#    for i in range(len(skill_list)):
#        if (node_name == skill_list[i].node_name):
#            skill_list[i].process.stop()
#            bond_list[i].bond.shutdown()
#            del skill_list[i]


def main():
    global skill_dict
    skill_dict = {}
    global node_dict
    node_dict = {}
    rospy.init_node('roboy_skill_machine')
    s = rospy.Service('start_skill', StartSkill, handle_start_skill)
    #t = rospy.Service('terminate_skill', TerminateSkill, handle_terminate_skill)
    while not rospy.is_shutdown():
        rospy.sleep(10.)
        check_nodes_still_running()
    rospy.spin()

   


if __name__ == '__main__':
    main()
