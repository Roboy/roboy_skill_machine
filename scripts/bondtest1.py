#! /usr/bin/env python

from bondpy import bondpy
import rospy
import time
rospy.init_node('bondtest1')
# Receives id from A using a service or an action
bond = bondpy.Bond("/skill_machine_bonds", "bondtest1")
bond.start()
# bond.wait_until_formed()
# rospy.set_param({"namespace": "test"})
print(rospy.get_namespace())
# rospy.wait_for_service("sdlf")

# bond.wait_until_broken()

rospy.spin()
bond.break_bond()
bond.shutdown()
print("done")