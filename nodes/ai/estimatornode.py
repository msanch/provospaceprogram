#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState as GameStateMsg
from soccerobjects import GameState

is_team_home = True
game_state = GameState(second_half=False)
current_pos = Pose2D()
current_vel = Pose2D()
current_time = time.time()
frame_delay = 0
i = 0

def low_pass(new_pos, old_pos, alpha=0.2):
	new_pos.x = (1-alpha)*new_pos.x + alpha*old_pos.x
	new_pos.y = (1-alpha)*new_pos.y + alpha*old_pos.y
	new_pos.theta = (1-alpha)*new_pos.theta + alpha*old_pos.theta


def calculate_velocity(new_pos, old_pos, time_delta):
	velocity = Pose2D()
	velocity.x = (new_pos.x - old_pos.x) / time_delta
	velocity.y = (new_pos.y - old_pos.y) / time_delta
	velocity.theta = (new_pos.theta - old_pos.theta) / time_delta
	return velocity


def save_vision_msg(msg):
	global current_pos, current_vel, current_time
	if not is_team_home ^ game_state.second_half:
		msg.x *= -1
		msg.y *= -1
		msg.theta += 180
		msg.theta %= 360
	low_pass(msg, current_pos)
	velocity = calculate_velocity(msg, current_pos, time.time() - current_time)
	low_pass(velocity, current_vel)
	current_pos = msg
	current_vel = velocity
	current_time = time.time()


def main():
	global is_team_home, i
	rospy.init_node('robot_estimator', anonymous=False)
	pub = rospy.Publisher('psp_ally1_estimator', Pose2D, queue_size=10)
	is_team_home = rospy.get_param('~is_team_home')
	rospy.Subscriber('psp_ally1', Pose2D, save_vision_msg)
	rospy.Subscriber('game', GameStateMsg, game_state.update)

	rate = rospy.Rate(50) # 50 Hz, 20 ms
	while not rospy.is_shutdown():
		predict_time = time.time()
		time_jump = predict_time - current_time + frame_delay
		predict_pos = Pose2D()
		predict_pos.x = current_pos.x + (current_vel.x * time_jump)
		predict_pos.y = current_pos.y + (current_vel.y * time_jump)
		predict_pos.theta = current_pos.theta + (current_vel.theta * time_jump)
		i += 1
		if i % 30 == 0:
			# print current_vel.x, current_vel.x * (predict_time - current_time + frame_delay), '\n'
			print current_pos.x, predict_pos.x, current_vel.x, time_jump, "\n"
			i = 0
		pub.publish(predict_pos)
		rate.sleep()

if __name__ == '__main__':
	main()
