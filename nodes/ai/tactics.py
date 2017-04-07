import soccerobjects
import constants
import skills
import util

class Tactics():

	def __init__(self, game_state, is_team_home, is_player1):
		self.skills = skills.Skills(game_state, is_player1)
		self.goal = soccerobjects.OPP_GOAL if is_team_home else soccerobjects.MY_GOAL

	def follow_ball(self, robot_me, ball):
		self.skills.move_to_xyt(robot_me.start_x(), ball.y, 0)

	def get_behind_ball(self, robot_me, ball):
		# first check to see if ball is behind you, if so then move to get behind 
		# print 'get_behind_ball'
		point = util.get_point_behind_ball(ball, self.goal)
		is_within_dist = robot_me.location.is_within_distance_from(point, .1)
		angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
		while not (is_within_dist and angle_diff < constants.MAX_ANGLE_DIFF):
			self.skills.move_to_point(point, util.get_angle_from_points(robot_me.location, ball))
			point = util.get_point_behind_ball(ball, self.goal)
			is_within_dist = robot_me.location.is_within_distance_from(point, .1)
			angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
			# print is_within_dist, angle_diff
		# print 'end of get behind ball'

	def run_to_goal(self, robot_me, ball):
		# print 'run_to_goal'
		dist = robot_me.location.distance_from(ball)
		angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
		# print dist, angle_diff
		while dist < constants.POSSESSION_DIST and angle_diff < constants.MAX_ANGLE_DIFF:
			self.skills.move_to_goal(util.get_angle_from_points(robot_me.location, self.goal), self.goal)
			dist = robot_me.location.distance_from(ball)
			angle_diff = util.get_angle_diff_from_points(robot_me.location, ball, self.goal)
			# print dist, angle_diff
		# print 'end of run to goal'

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
