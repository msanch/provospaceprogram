#!/usr/bin/env python

import time

from geometry_msgs.msg import Pose2D
import rospy
import math
from soccerref.msg import GameState as GameStateMsg
from soccerobjects import GameState

# FIXME: Check to see if pi needs time.clock()

is_team_home = True
game_state = GameState(second_half=False)
current_pos = Pose2D()
current_vel = Pose2D()
current_acl = Pose2D()
current_time = time.time()
frame_delay = 0.0
i = 0

def low_pass(new_pos, old_pos, alpha=0.5):
	new_pos.x = (1-alpha)*new_pos.x + alpha*old_pos.x
	new_pos.y = (1-alpha)*new_pos.y + alpha*old_pos.y
	new_pos.theta = (1-alpha)*new_pos.theta + alpha*old_pos.theta

def dirty_derivative(new_pos, old_pos, time_delta):
	derivative = Pose2D()
	derivative.x = (new_pos.x - old_pos.x) / time_delta
	derivative.y = (new_pos.y - old_pos.y) / time_delta
	derivative.theta = (new_pos.theta - old_pos.theta) / time_delta
	return derivative


def save_vision_msg(msg):
	global current_pos, current_vel, current_acl, current_time
	msg_received_time = time.time()
	if not is_team_home ^ game_state.second_half:
		msg.x *= -1
		msg.y *= -1
		msg.theta += math.pi
	msg.theta %= 2*math.pi
	low_pass(msg, current_pos)
	time_delta = msg_received_time - current_time
	velocity = dirty_derivative(msg, current_pos, time_delta)
	low_pass(velocity, current_vel)
	acceleration = dirty_derivative(velocity, current_vel, time_delta)
	# low_pass(acceleration, current_acl)
	current_pos = msg
	current_vel = velocity
	current_acl = acceleration
	current_time = msg_received_time


def main():
	global is_team_home, i
	rospy.init_node('psp_estimator', anonymous=False)
	is_team_home = rospy.get_param('~is_team_home')
	pub = rospy.Publisher('publisher', Pose2D, queue_size=10)
	rospy.Subscriber('subscriber', Pose2D, save_vision_msg)
	rospy.Subscriber('game', GameStateMsg, game_state.update)

	rate = rospy.Rate(50) # 50 Hz, 20 ms
	while not rospy.is_shutdown():
		predict_time = time.time()
		time_jump = predict_time - current_time + frame_delay
		predict_pos = Pose2D()
		predict_pos.x = current_pos.x + (current_vel.x * time_jump) + (current_acl.x * time_jump**2)
		predict_pos.y = current_pos.y + (current_vel.y * time_jump) + (current_acl.y * time_jump**2)
		predict_pos.theta = current_pos.theta + (current_vel.theta * time_jump) + (current_acl.theta * time_jump**2)
		# i += 1
		# if i % 30 == 0:
		# 	# print current_vel.x, current_vel.x * (predict_time - current_time + frame_delay), '\n'
		# 	print current_pos.x, predict_pos.x, current_vel.x, time_jump, "\n"
		# 	i = 0
		pub.publish(predict_pos)
		rate.sleep()

if __name__ == '__main__':
	main()
