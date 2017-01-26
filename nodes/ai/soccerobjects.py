# robot.py

import rospy
import util

class Robot:

	def __init__(self, player1=True):
		self.player1 = player1
		self.x = 0
		self.y = 0
		self.theta = 0

	def start_x(self):
		return -0.5 if self.player1 else -1.5

	def update(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta

	def is_closer(self, obj, ally):
		util.distance_between_points();
		return True

class Ball:

	def __init__(self):
		self.x = -100
		self.y = -100

	def update(self, msg):
		self.x = msg.x
		self.y = msg.y
