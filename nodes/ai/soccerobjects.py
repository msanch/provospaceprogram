# robot.py

import rospy
import constants
from soccerref.msg import GameState
from math import sqrt

class Vector2D:

	def __init__(self, x=0, y=0):
		self.x = x
		self.y = y

	@classmethod
	def from_points(cls, p1, p2):
		return cls(p2.x - p1.x, p2.y - p1.y)

	def add(self, v):
		self.x += v.x
		self.y += v.y
		return self

	def sub(self, v):
		self.x -= v.x
		self.y -= v.y
		return self

	def get_length(self):
		return sqrt(self.x**2 + self.y**2)

	def set_length(self, length):
		scale = self.get_length()
		self.x /= scale / length
		self.y /= scale / length
		return self

	def multiply(self, m):
		self.x *= m
		self.y *= m
		return self

	def flip(self):
		self.multiply(-1)

	def __repr__(self):
		return "Vector2D: x={0}, y={1}".format(self.x, self.y)


class Point2D:

	def __init__(self, x=0, y=0):
		self.x = x
		self.y = y
		self.threshold = .1

	def update(self, new_point):
		self.x = new_point.x
		self.y = new_point.y

	def get_offset_point(self, v):
		return Point2D(self.x + v.x, self.y + v.y)

	def distance_from(self, p):
		return sqrt((self.x-p.x)**2 + (self.y-p.y)**2)

	def is_within_distance_from(self, p, threshold):
		return self.distance_from(p) < threshold

	def __repr__(self):
		return "Point2D: x={0}, y={1}".format(self.x, self.y)

# Standard Points
OPP_GOAL = Point2D(x=1.66, y=0)
MY_GOAL  = Point2D(x=-1.66, y=0)

class Robot:

	def __init__(self, is_player1=True, is_home_team=True):
		self.is_player1 = is_player1
		self.is_home_team = is_home_team
		self.location = Point2D(0, 0)
		self.theta = 0

	def start_x(self):
		return (-0.5 if self.is_player1 else -1.5)

	def update(self, msg):
		self.location.update(msg)
		self.theta = msg.theta


class GameState:

	def __init__(self, home_score=0, away_score=0, home_bot_count=2, away_bot_count=2, remaining_seconds=120, play=False, reset_field=False, second_half=False):
		self.home_score        = home_score
		self.away_score        = away_score
		self.home_bot_count    = home_bot_count
		self.away_bot_count    = away_bot_count
		self.remaining_seconds = remaining_seconds
		self.play              = play
		self.reset_field       = reset_field
		self.second_half       = second_half

	def update(self, msg):
		self.home_score        = msg.home_score
		self.away_score        = msg.away_score
		self.home_bot_count    = msg.home_bot_count
		self.away_bot_count    = msg.away_bot_count
		self.remaining_seconds = msg.remaining_seconds
		self.play              = msg.play
		self.reset_field       = msg.reset_field
		self.second_half       = msg.second_half

class SoccerResetException(Exception):

	def __init__(self, msg=None):
		self.msg = msg

class Avoidance:

	def __init__(self, *obstacles):
		self.x_unit = Vector2D()
		self.y_unit = Vector2D()
		self.x_unit_inv = Vector2D()
		self.y_unit_inv = Vector2D()
		self.origin = Point2D()
		self.obstacles = obstacles
		self.transformed_obstacles = tuple(Point2D() for _ in range(len(self.obstacles)))
		self.obj_radius = constants.OBJ_RADIUS

	def __set_transform(self, start, end):
		self.origin.update(start)
		self.x_unit.x = end.x - start.x
		self.x_unit.y = end.y - start.y
		self.y_unit.x = -self.x_unit.y
		self.y_unit.y = self.x_unit.x
		self.x_unit.set_length(1)
		self.y_unit.set_length(1)

	def __calculate_inv_transform(self):
		self.x_unit_inv.x = self.y_unit.y
		self.x_unit_inv.y = -self.x_unit.y
		self.y_unit_inv.x = -self.y_unit.x
		self.y_unit_inv.y = self.x_unit.x
		determinant = (self.x_unit.x*self.y_unit.y) - (self.x_unit.y*self.y_unit.x)
		if determinant != 0:
			self.x_unit_inv.multiply(float(1)/determinant)
			self.y_unit_inv.multiply(float(1)/determinant)

	def __transform_xy(self, x, y):
		# translate
		x -= self.origin.x
		y -= self.origin.y
		# rotate
		new_x = (x*self.x_unit.x) + (y*self.x_unit.y)
		new_y = (x*self.y_unit.x) + (y*self.y_unit.y)
		return new_x, new_y

	def __inv_transform_xy(self, x, y):
		# rotate
		new_x = (x*self.x_unit_inv.x) + (y*self.x_unit_inv.y)
		new_y = (x*self.y_unit_inv.x) + (y*self.y_unit_inv.y)
		# translate
		new_x += self.origin.x
		new_y += self.origin.y
		return new_x, new_y

	def __transform_point(self, p):
		p.x, p.y = self.__transform_xy(p.x, p.y)

	def __inv_transform_point(self, p):
		p.x, p.y = self.__inv_transform_xy(p.x, p.y)

	def __transform_obstacles(self):
		for obj, tobj in zip(self.obstacles, self.transformed_obstacles):
			tobj.x, tobj.y = self.__transform_xy(obj.x, obj.y)

	def avoid(self, start, end, avoid_ball):
		avoid_end = Point2D()
		avoid_end.update(end)
		self.__set_transform(start, avoid_end)
		self.__calculate_inv_transform()
		self.__transform_obstacles()
		self.__transform_point(avoid_end)

		closest_obstacle = None
		for tobj in self.transformed_obstacles:
			if 0 < tobj.x < avoid_end.x and abs(tobj.y) < self.obj_radius and \
				(not closest_obstacle or tobj.x < closest_obstacle.x):
					closest_obstacle = tobj

		if closest_obstacle:
			overlap = (self.obj_radius - abs(closest_obstacle.y)) * (1 if closest_obstacle.y < 0 else -1)
			dy = overlap * avoid_end.x / closest_obstacle.x
			avoid_end.y += dy

		self.__inv_transform_point(avoid_end)
		return avoid_end
