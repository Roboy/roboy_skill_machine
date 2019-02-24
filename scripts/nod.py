#!/usr/bin/env python

import rospy
import time
from bondpy import bondpy
from std_msgs.msg import Float32

rospy.init_node('nod')

# global bond
bond = bondpy.Bond("skill_machine_bonds", "nod_bond")
bond.start()


def main():
    pub = rospy.Publisher('/sphere_head_axis1/sphere_head_axis1/target', Float32, queue_size=100)
    # rate = rospy.Rate(5) # 10hz
    data = 0.0
    rospy.loginfo(data)
    pub.publish(data)
    time.sleep(1)
    data = 0.3
    rospy.loginfo(data)
    pub.publish(data)
    time.sleep(3)
    data = -0.3
    rospy.loginfo(data)
    pub.publish(data)
    time.sleep(3)
    data = 0.0
    rospy.loginfo(data)
    pub.publish(data)
    bond.break_bond()


if __name__ == '__main__':
    main()
