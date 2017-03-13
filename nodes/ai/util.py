# util.py

import constants as const
import math
from soccerobjects import Vector2D

def get_point_beside_ball(ball, robot_me):
	dist_beside = .20
	location_v = Vector2D.from_points(ball, const.OPP_GOAL)
	# TODO: get a good point on the side
	location_v.set_length(dist_beside)
	location_v.flip()
	return ball.get_offset_point(location_v)

def get_point_behind_ball(ball, p):
	dist_behind = .20
	location_v = Vector2D.from_points(ball, p)
	location_v.y *= 2.5
	location_v.set_length(dist_behind)
	location_v.flip()
	return ball.get_offset_point(location_v)

def get_point_for_def(ball):
	location_v = Vector2D.from_points(const.MY_GOAL, ball)
	location_v.multiply(0.5)
	return const.MY_GOAL.get_offset_point(location_v)

def get_angle_from_points(p1, p2):
	dy = p2.y - p1.y
	dx = p2.x - p1.x
	return math.atan2(dy, dx)

def get_angle_diff_from_points(p1, p2, p3):
	angle_diff1 = get_angle_from_points(p1, p2)
	angle_diff2 = get_angle_from_points(p2, p3)
	angle_diff = abs(angle_diff1 - angle_diff2)
	return angle_diff if angle_diff < 180 else 360 - angle_diff

def is_ball_behind(ball, p1):
	return p1.x > ball.x
