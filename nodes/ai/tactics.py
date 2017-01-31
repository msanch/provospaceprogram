import constants
import skills
import util

class Tactics():

	def __init__(self, game_state):
		self.skills = skills.Skills(game_state)

	def follow_ball(self, robot_me, ball):
		self.skills.move_to_xyt(robot_me.start_x(), ball.y, 0)

	def get_behind_ball(self, robot_me, ball):
		# first check to see if ball is behind you, if so then move to get behind 
		print 'get_behind_ball'
		point = util.get_point_behind_ball(ball)
		is_within_dist = robot_me.location.is_within_distance_from(point, .1)
		angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
		print is_within_dist, angle_diff
		while not (is_within_dist and angle_diff < constants.MAX_ANGLE_DIFF):
			if not util.is_ball_behind(ball, robot_me.location):
				self.skills.move_to_point(point, util.get_angle_from_points(robot_me.location, ball))
			else:
				self.skills.move_to_xyt(-.25, .1, 0)
			point = util.get_point_behind_ball(ball)
			is_within_dist = robot_me.location.is_within_distance_from(point, .1)
			angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
			print is_within_dist, angle_diff
		print 'end of get behind ball'

	def run_to_goal(self, robot_me, ball):
		print 'run_to_goal'
		dist = robot_me.location.distance_from(ball)
		angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
		print dist, angle_diff
		while dist < constants.POSSESSION_DIST and angle_diff < constants.MAX_ANGLE_DIFF:
			self.skills.move_to_goal(util.get_angle_from_points(robot_me.location, constants.OPP_GOAL))
			dist = robot_me.location.distance_from(ball)
			angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
			print dist, angle_diff
		print 'end of run to goal'

	def defend_on_ball_y(self, robot_me, ball):
		point = util.get_point_for_def(ball)
		theta = util.get_angle_from_points(robot_me.location, ball)
		self.skills.move_to_point(point, theta)

	def return_to_start(self, robot_me):
		self.skills.move_to_xyt(robot_me.start_x(), 0, 0)
