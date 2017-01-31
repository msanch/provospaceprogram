# util.py

import constants as const
from soccerobjects import Vector2D

def get_point_behind_ball(ball):
	dist_behind = .3
	location_v = Vector2D.from_points(ball, const.OPP_GOAL)
	location_v.set_length(dist_behind)
	location_v.flip()
	return ball.get_offset_point(location_v)

def get_point_for_def(ball):
	location_v = Vector2D.from_points(const.MY_GOAL, ball)
	location_v.multiply(0.5)
	return const.MY_GOAL.get_offset_point(location_v)

def get_angle_from_points(p1, p2, p3):
	v1 = Vector2D.from_points(p1, p2).set_length(1)
	v2 = Vector2D.from_points(p2, p3).set_length(1)
	v2.sub(v1)
	return v2.get_length()
