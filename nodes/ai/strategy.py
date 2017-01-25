import rospy
import tactics
import util

class Strategy():

	def __init__(self):
		self.strategies = util.enum(FOLLOW_BALL=0)
		self.tactics = tactics.Tactics()

	def pick_play(self):
		return self.strategies.FOLLOW_BALL


	def make_play(self, robot_me, robot_ally, ball):
		play = self.pick_play()
		if play == self.strategies.FOLLOW_BALL:
			self.tactics.follow_ball(robot_me, ball)
		else:
			rospy.logerr('UNKOWN PLAY')
