import constants
import skills
import util

class Tactics():

	def __init__(self, game_state):
		self.skills = skills.Skills(game_state)

	def follow_ball(self, robot_me, ball):
		self.skills.move_to_xyt(robot_me.start_x(), ball.y, 0)

	def get_behind_ball(self, robot_me, ball):
		point = util.get_point_behind_ball(ball)
		while not robot_me.location.is_within_distance_from(point, .01):
			self.skills.move_to_point(point, 0)
			point = util.get_point_behind_ball(ball)


	def run_to_goal(self, robot_me, ball):
		# while Vector2D.from_points(robot_me, ball).is_within(2).angle_from(Vector2D.from_points(ball, constants.OPP_GOAL)):
		dist = robot_me.location.distance_from(ball)
		angle = util.get_angle_from_points(robot_me.location, ball, constants.OPP_GOAL)
		print angle, dist
		while dist < constants.POSSESSION_DIST*3 and angle < .1:
			print angle, dist
			self.skills.move_to_goal()
			dist = robot_me.location.distance_from(ball)
			angle = util.get_angle_from_points(robot_me.location, ball, constants.OPP_GOAL)

	def defend_on_ball_y(self, robot_me, ball):
		point = util.get_point_for_def(ball)
		self.skills.move_to_point(point, 0)

	def return_to_start(self, robot_me):
		self.skills.move_to_xyt(robot_me.start_x, 0, 0)
