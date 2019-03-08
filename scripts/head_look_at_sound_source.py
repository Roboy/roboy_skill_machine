#!/usr/bin/env python
import roslib

roslib.load_manifest('smach')
import rospy
import paramiko
import threading
import random
import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import actionlib
import std_srvs.srv, geometry_msgs.msg
import roboy_cognition_msgs.msg
import numpy as np
import math
import rosgraph
from std_msgs.msg import Float32
from bondpy import bondpy

rospy.init_node('user_tracking')

location = np.array([0.0,0.0,0.0])
new_goal = False

bond = bondpy.Bond("skill_machine_bonds", "user_tracking_bond")
bond.start()

#ssh = paramiko.SSHClient()
#ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#ssh.connect(hostname="192.168.0.133", username="pi", password="Roboy2016")

#sh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("env ROS_MASTER_URI=http://192.168.0.126:11311 /opt/ros/kinetic/env.sh roslaunch -c sam_and_odas.launch -u  http://192.168.0.133 --run_id 25f83d3e-a7db-11e6-bef6-086266c7c72e")

#args = [
#'/opt/ros/kinetic/env.sh', 
#'roslaunch', '-c', 
#'sam_and_odas.launch', 
#'-u', 'http://192.168.0.133:44075/'
#'--run_id', self.run_id
#]

#env_command = 'env %s=%s' % (rosgraph.ROS_MASTER_URI, 'http://192.168.0.126:11311')
#command = '%s %s' % (env_command, ' '.join(args))
#print("launching remote roslaunch child with command: [%s]"%(str(command)))
#ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(command)
#print(ssh_stdin, ssh_stdout, ssh_stderr)

#ssh.connect("192.168.0.123", 22, "pi", "Roboy2016")
#ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("export ROS_MASTER_URI='http://192.168.0.126:11311'")
#ssh.exec_command("export ROS_MASTER_URI='http://192.168.0.126:11311'")
#ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("roslaunch rosam sam_and_odas.launch")


lookat_x = rospy.Publisher('/sphere_head_axis0/sphere_head_axis0/target', Float32, queue_size=100)
lookat_y = rospy.Publisher('/sphere_head_axis1/sphere_head_axis1/target', Float32, queue_size=100)
lookat_z = rospy.Publisher('/sphere_head_axis2/sphere_head_axis2/target', Float32, queue_size=100)

def callback(data):
    global location
    global new_goal
    sound_sources = len(data.x)
    if sound_sources>0:
        #random_source = random.randint(0,sound_sources-1)
        #source = data.energy.index(max(data.energy))
        source = len(data.x) - 1
        rospy.loginfo_throttle(1,"there are " + str(sound_sources) + " sound sources, choosing " + str(source))
        location[0] = 0
	location[1] = -data.y[source]-0.1
	location[2] = -data.x[source]
	if location[1] > 0.12:
		location[1] = 0.12
	if location[1] < -0.25:
		location[1] = -0.25
	if location[2] > 0.5:
		location[2] = 0.5
	if location[2] < -0.5:
		location[2] = -0.5

        new_goal  = True

rospy.Subscriber("/roboy/cognition/audio/record/location", roboy_cognition_msgs.msg.AudioLocation, callback)

def main():
    global new_goal
    global location
    #print(ssh.get_transport().is_active())

    while not rospy.is_shutdown():
        if new_goal:
            lookat_x.publish(location[0])
            lookat_y.publish(location[1])
            lookat_z.publish(location[2])
            new_goal = False

    rospy.spin()


if __name__ == '__main__':
    main()
