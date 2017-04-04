import constants
import skills
import util
from soccerobjects import WayPoint
class Tactics():

	def __init__(self, game_state, is_team_home, is_player1):
		self.skills = skills.Skills(game_state, is_player1)
		self.goal = constants.OPP_GOAL if is_team_home else constants.MY_GOAL

	def follow_ball(self, robot_me, ball):
		self.skills.move_to_xyt(robot_me.start_x(), ball.y, 0)

	def get_behind_ball(self, robot_me, ball):
		# first check to see if ball is behind you, if so then move to get behind 
		# print 'get_behind_ball'
		point = util.get_point_behind_ball(ball, self.goal)
		def validate_point():
			angle_diff = util.get_angle_diff_from_points(point, ball, self.goal)
			return angle_diff < constants.MAX_ANGLE_DIFF
		def update_point():
			return util.get_point_behind_ball(ball, self.goal)
		return WayPoint(point, validate_point, update_point, lambda: False)

	def run_to_goal(self, robot_me, ball):
		def abort():
			dist = robot_me.location.distance_from(ball)
			angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
			return angle_diff > constants.MAX_ANGLE_DIFF or dist > constants.POSSESSION_DIST
		return WayPoint(self.goal, lambda: True, lambda: self.goal, abort)

	def defend_on_ball_y(self, robot_me, robot_ally, ball):
		if -0.8 < ball.x < 0.2 and ball.x < robot_ally.location.x:
			theta = util.get_angle_from_points(robot_me.location, ball)
			self.skills.move_to_point(ball, theta)
		else:
			point = util.get_point_for_def(ball)
			theta = util.get_angle_from_points(robot_me.location, ball)
			self.skills.move_to_point(point, theta)

	def return_to_start(self, robot_me):
		self.skills.move_to_xyt(robot_me.start_x(), 0, 0)

	def test_tactic(self, robot_me, ball):
		angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
		# print angle_diff
		self.skills.move_to_xyt(0, 0, util.get_angle_from_points(robot_me.location, ball))
