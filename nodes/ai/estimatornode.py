#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState as GameStateMsg
from soccerobjects import GameState


def main():
	rospy.init_node('robot_estimator', anonymous=False)
	pub = rospy.Publisher('robot_state', Pose2D, queue_size=10)
	is_team_home = rospy.get_param('~is_team_home')
	game_state = GameState()

	def forward_vision_msg(msg):
		if not is_team_home ^ game_state.second_half:
			msg.x *= -1
			msg.y *= -1
			msg.theta += 180
			msg.theta %= 360
		pub.publish(msg)

	rospy.Subscriber('vision_position', Pose2D, forward_vision_msg)
	rospy.Subscriber('game', GameStateMsg, game_state.update)
	rospy.spin()

if __name__ == '__main__':
	main()
