import rospy
import tactics

def enum(**enums):
    return type('Enum', (), enums)

class Strategy():

	def __init__(self, game_state):
		self.strategies = enum(FOLLOW_BALL=0, SIMPLE_STRAT=1)
		self.tactics = tactics.Tactics(game_state)

	def pick_play(self):
		return self.strategies.SIMPLE_STRAT

	def make_play(self, robot_me, robot_ally, ball):
		play = self.pick_play()
		if self.strategies.FOLLOW_BALL == play:
			self.tactics.follow_ball(robot_me, ball)
		elif self.strategies.SIMPLE_STRAT == play:
			if robot_me.is_player1:
				self.tactics.get_behind_ball(robot_me, ball)
				self.tactics.run_to_goal(robot_me, ball)
			else:
				self.tactics.defend_on_ball_y(robot_me, ball)
		else:
			rospy.logerr('UNKOWN PLAY')

	def return_to_start(self, robot_me):
		self.tactics.return_to_start(robot_me)
