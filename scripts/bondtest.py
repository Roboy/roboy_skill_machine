#! /usr/bin/env python

from bondpy import bondpy
import rospy
import time
rospy.init_node('bondtest')
# Receives id from A using a service or an action
bond = bondpy.Bond("/skill_machine_bonds", "one")
bond.start()

# rospy.set_param({"namespace": "test"})
print(rospy.get_namespace())
# rospy.wait_for_service("sdlf")

# bond.wait_until_broken()
rospy.spin()
# import time
# while True:
#     time.sleep(1.0)