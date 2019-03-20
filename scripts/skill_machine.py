#! /usr/bin/env python

import rospy
from bondpy import bondpy
import subprocess
from roboy_skill_machine.srv import LaunchSkill, ExecuteSkill, TerminateSkill

from threading import RLock
ROS_MASTER_URI = "http://192.168.0.127:11311"


class Skill:
    def __init__(self, skill_machine, skill_name, launch_package, launch_file, continuous, node_list):
        self.sm = skill_machine
        self.skill_name = skill_name
        self.launch_package = launch_package
        self.launch_file = launch_file
        self.continuous = continuous
        self.node_name_list = []
        self.add_nodes(node_list)

    def add_nodes(self, node_list):
        for node in node_list:
            if node.node_name not in self.sm.node_dict:
                new_node = Node(self.sm, node.node_name, node.node_executable, node.node_package, node.node_machine, self.continuous)
                self.sm.node_dict[node.node_name] = new_node
            self.sm.node_dict[node.node_name].skill_name_list.append(self.skill_name)
            self.sm.node_dict[node.node_name].continuous = self.sm.node_dict[node.node_name].continuous or self.continuous
            self.node_name_list.append(node.node_name)
        self.node_name_list = list(set(self.node_name_list))


    def launch(self):
        command = "roslaunch {0} {1}".format(self.package, self.launch_file)
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

    def execute(self):
        p = subprocess.Popen(self.launch_file, shell=True)
        state = p.poll()
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
            return False
        elif state > 0:
            rospy.loginfo("Process terminated without error")

        for node in self.node_name_list:
            sm.node_dict[node].create_bond()

        return True


    def terminate(self):
        del_node_list = []
        for node_name in sm.node_dict:
            if (self.skill_name in sm.node_dict[node_name].skill_name_list):
                del_node_list.append(node_name)
        for node_name in del_node_list:

            sm.node_dict[node_name].skill_name_list.remove(self.skill_name)
            if sm.node_dict[node_name].skill_name_list == []:
                command = "rosnode kill {0}".format("/" + node_name)
                subprocess.Popen(command, shell=True)
                sm.node_dict[node_name].remove()
        del sm.skill_dict[self.skill_name]
        return True



class Node:
    def __init__(self, skill_machine, node_name, node_executable, node_package, node_machine, continuous):
        self.node_name = node_name
        self.sm = skill_machine
        self.node_executable = node_executable
        self.node_package = node_package
        self.node_machine = node_machine
        self.continuous = continuous
        self.skill_name_list = []

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other):
        return self.__dict__ == other.__dict__


    def create_bond(self):
        self.bond = bondpy.Bond("/skill_machine_bonds", self.node_name)
        self.bond.start()
        self.bond.wait_until_formed()
        rospy.loginfo("Created bond with: " + self.node_name)


    def restart(self):
        self.bond.break_bond()
        self.bond.shutdown()
        if (self.node_machine == "local" or self.node_machine == "localhost"):
            command = "rosrun {0} {1}".format(self.node_package, self.node_executable)
        else:
            command = "ssh {0} 'export ROS_MASTER_URI={1}; export ROS_IP={2}; rosrun {3} {4}'".format(self.node_machine, ROS_MASTER_URI, self.node_machine.split('@', 1)[-1], self.node_package, self.node_executable)
        p = subprocess.Popen(command, shell=True)
        state = p.poll()
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
            return False
        elif state > 0:
            rospy.loginfo("Process terminated without error")

        self.create_bond()


    def remove(self):
        self.bond.break_bond()
        self.bond.shutdown()
        for skill_name in sm.skill_dict:
            if self.node_name in sm.skill_dict[skill_name].node_name_list:
                sm.skill_dict[skill_name].node_name_list.remove(self.node_name)
        del sm.node_dict[self.node_name]
        rospy.loginfo("Removed node: %s", self.node_name)


class SkillMachine:
    def __init__(self):
        self.node_dict = {}
        self.skill_dict = {}
        self.lock = RLock()

    def handle_launch_skill(self, request):
        # with self.lock:
        new_skill = Skill(self, request.skill_name, request.launch_package, request.launch_file, request.continuous, request.node_list)
        new_skill.launch()
        self.skill_dict[request.skill_name] = new_skill
        return 1

    def handle_execute_skill(self, request):
        # with self.lock:
        new_skill = Skill(self, request.skill_name, None, request.command, request.continuous, request.node_list)
        new_skill.execute()
        self.skill_dict[request.skill_name] = new_skill
        return 1


    def check_nodes_still_running(self):
        with self.lock:
            items_to_remove = []
            for node_name in self.node_dict:
                node = self.node_dict[node_name]
                if node.bond.is_broken() and node.continuous is True:
                    node.restart()
                elif node.bond.is_broken() and node.continuous is False:
                    items_to_remove.append(node_name)
            for node_name in items_to_remove:
                self.node_dict[node_name].remove()
            items_to_remove = []
            for skill_name in self.skill_dict:
                if not self.skill_dict[skill_name].node_name_list:
                    items_to_remove.append(skill_name)
            for skill_name in items_to_remove:
                del self.skill_dict[skill_name]


    def handle_terminate_skill(self, request):
        with self.lock:
            self.skill_dict[request.skill_name].terminate()
        return 1



    def main(self):
        rospy.init_node('roboy_skill_machine')
        s = rospy.Service('launch_skill', LaunchSkill, self.handle_launch_skill)
        t = rospy.Service('terminate_skill', TerminateSkill, self.handle_terminate_skill)
        e = rospy.Service('execute_skill', ExecuteSkill, self.handle_execute_skill)
        print(rospy.get_namespace())
        while not rospy.is_shutdown():
            rospy.sleep(1.)
            self.check_nodes_still_running()
        rospy.spin()

  


if __name__ == '__main__':
    sm = SkillMachine()
    sm.main()
