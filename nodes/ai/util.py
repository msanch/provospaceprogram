# util.py

import constants as const
import math
from soccerobjects import Vector2D

def get_point_behind_ball(ball):
	dist_behind = .20
	location_v = Vector2D.from_points(ball, const.OPP_GOAL)
	location_v.x *= 1.1
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
	radians = math.acos(adjacent/hypotenuse)
	degrees = math.degrees(radians)
	if p1.x - p2.x < 0:
		degrees = 360 - degrees
	degrees -= 270
	return degrees

def is_ball_behind(ball, p1):
	if (p1 > ball.x):
		print "ball is behind you"
		return True
