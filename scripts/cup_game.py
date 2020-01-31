#!/usr/bin/env python3
import rospy
import roslib

roslib.load_manifest('smach')
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer

from sensor_msgs.msg import JointState

speech = rospy.ServiceProxy('/roboy/cognition/speech/synthesis/talk', roboy_communication_cognition.srv.Talk)

