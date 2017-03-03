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

def get_point_behind_ball(ball):
	dist_behind = .20
	location_v = Vector2D.from_points(ball, const.OPP_GOAL)
	location_v.y *= 3
	location_v.set_length(dist_behind)
	location_v.flip()
	return ball.get_offset_point(location_v)

def get_point_for_def(ball):
	location_v = Vector2D.from_points(const.MY_GOAL, ball)
	location_v.multiply(0.5)
	return const.MY_GOAL.get_offset_point(location_v)

def get_angle_from_points(p1, p2):
	adjacent = p2.y - p1.y
	hypotenuse = p1.distance_from(p2)
	if hypotenuse == 0:
		return 0
	radians = math.acos(adjacent/hypotenuse)
	degrees = math.degrees(radians)
	if p1.x - p2.x < 0:
		degrees = 360 - degrees
	degrees -= 270
	return degrees if degrees >= 0 else degrees + 360

def get_angle_diff_from_points(p1, p2, p3):
	angle_diff1 = get_angle_from_points(p1, p2)
	angle_diff2 = get_angle_from_points(p2, p3)
	angle_diff = abs(angle_diff1 - angle_diff2)
	return angle_diff if angle_diff < 180 else 360 - angle_diff

def is_ball_behind(ball, p1):
	return p1.x > ball.x
