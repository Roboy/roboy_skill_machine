#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from bondpy import bondpy


def talker():
    pub = rospy.Publisher('simple_publisher', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    bond = bondpy.Bond("skill_machine_bonds", "simple_publisher_bond")
    bond.start()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
