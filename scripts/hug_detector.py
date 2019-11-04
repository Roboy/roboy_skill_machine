#!/usr/bin/python
import rospy
import cv2
import numpy as np
import struct
import time

import pdb

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import actionlib
from roboy_control_msgs.msg import PerformMovementAction, PerformMovementGoal


class HugDetector():
	"""docstring for HugDetector"""
	def __init__(self):
		# assuming pico flexx node publishes at 5 hz
		self.close_frames = 0
		self.do_hug = False
		self.hug_in_progress = False

		rospy.init_node("hug_detector")
                self.hug_counter = rospy.Publisher('/hug_counter', Empty)
		rospy.Subscriber("/pico_flexx/image_depth", Image, self.cb, queue_size=1)
		shoulder_left_client = actionlib.SimpleActionClient('shoulder_left_movement_server', PerformMovementAction)
		shoulder_right_client = actionlib.SimpleActionClient('shoulder_right_movement_server', PerformMovementAction)

		self.clients = [shoulder_left_client, shoulder_right_client]
		for c in self.clients:
			rospy.loginfo("Waiting for %s action server to show up..."%c.action_client.ns)
			c.wait_for_server()
                        rospy.loginfo("connected")
		self.goal = PerformMovementGoal(action='hug')
		self.first_hug = True
                self.hug_done = False
		self.recovery_period = 15
		self.last_hug_timestamp = 0
		# self.hug_thread.deamon = True

	def dohug(self):
                self.hug_counter.publish()
                rospy.set_param('trajectory_active', True)
		self.hug_in_progress = True
                self.hug_done = False
		rospy.loginfo("Doing hug")
		self.goal = PerformMovementGoal(action='shoulder_left_hug')
		self.clients[0].send_goal(self.goal)
		self.goal = PerformMovementGoal(action='shoulder_right_hug')
		self.clients[1].send_goal(self.goal)
		# self.clients[1].wait_for_result()
		self.hug_in_progress = False
		self.close_frames = 0

	def middle_only(self,data, height, width):
		new_data = []
		save_height = range(height/3,2*height/3)
		start_widht = width/3
		window = start_widht
		for i in save_height:
			new_data.extend(data[i*width+start_widht:i*width+start_widht+window])
		return new_data	

	def cb(self,msg):
            if rospy.get_param('trajectory_active'):
                rospy.loginfo('not my turn')
                self.close_frames = 0
            if not rospy.get_param('trajectory_active'):
                if self.first_hug or (self.clients[1].get_result() and self.clients[0].get_result()):
                    self.hug_done = True
                    rospy.set_param('trajectory_active', False)
		if self.first_hug or ((time.time() - self.last_hug_timestamp > self.recovery_period) and self.hug_done):	
                        #if self.close_frames == 15 and rospy.get_param('trajectory_active'):
                        #    self.close_frames = 0
                        #    rospy.loginfo('skipping hug')
                        if self.close_frames == 15 and not rospy.get_param('trajectory_active'):
				rospy.loginfo('doing hug')
                                self.first_hug = False
				self.do_hug = True
				self.dohug()
				self.last_hug_timestamp = time.time()
			
			data = np.frombuffer(msg.data, dtype=np.float32)
			middle = self.middle_only(data, msg.height, msg.width)
			if (max(middle) < 0.40):
				self.close_frames += 1
			print(max(middle))
		else:
			self.close_frames = 0


h = HugDetector()
rospy.spin()
