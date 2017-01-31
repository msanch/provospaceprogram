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

		point = util.get_point_behind_ball(ball)
		while not (robot_me.location.is_within_distance_from(point, .1) and
			util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL) < 4):
			
			if not util.is_ball_behind(ball, robot_me.location):
				self.skills.move_to_point(point, util.get_angle_from_points(robot_me.location, ball))
				point = util.get_point_behind_ball(ball)
			else:
				self.skills.move_to_xyt(-.25, .1, 0)
				point = util.get_point_behind_ball(ball)


	def run_to_goal(self, robot_me, ball):
		dist = robot_me.location.distance_from(ball)
		angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
		while dist < constants.POSSESSION_DIST and angle_diff < 5:
			self.skills.move_to_goal(util.get_angle_from_points(robot_me.location, constants.OPP_GOAL))
			dist = robot_me.location.distance_from(ball)
			angle_diff = abs(util.get_angle_from_points(robot_me.location, ball) - util.get_angle_from_points(ball, constants.OPP_GOAL))
			print angle_diff

	def defend_on_ball_y(self, robot_me, ball):
		point = util.get_point_for_def(ball)
		theta = util.get_angle_from_points(robot_me.location, ball)
		self.skills.move_to_point(point, theta)

	def return_to_start(self, robot_me):
		self.skills.move_to_xyt(robot_me.start_x(), 0, 0)
