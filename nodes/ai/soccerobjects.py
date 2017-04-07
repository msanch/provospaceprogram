# robot.py

import rospy
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

	def __init__(self, radius=0.3, *obstacles):
		self.x_unit = Vector2D()
		self.y_unit = Vector2D()
		self.origin = Point2D()
		self.obstacles = obstacles
		self.transformed_obstacles = (Point2D() for _ in range(len(self.obstacles)))
		self.radius = radius

	def __set_transform(self, start, end):
		self.origin.update(start)
		self.x_unit.x = end.x - start.x
		self.x_unit.y = end.y - start.y
		self.y_unit.x = self.x_unit.y
		self.y_unit.y = -self.x_unit.x
		self.x_unit.set_length(1)
		self.y_unit.set_length(1)

	def __transform_xy(self, x, y):
		# translate
		x -= self.origin.x
		y -= self.origin.y
		# rotate
		new_x = (x*self.x_unit.x) + (y*self.x_unit.y)
		new_y = (x*self.y_unit.x) + (y*self.y_unit.y)
		return new_x, new_y

	def __transform_obstacles(self):
		for ob, tr_ob in zip(self.obstacles, self.transformed_obstacles):
			tr_ob.x, tr_ob.y = self.__transform_xy(ob.x, ob.y)

	def avoid(self, start, end, avoid_ball):
		self.__set_transform(start, end)
		self.__transform_obstacles()

		print('before', start, end)
		print('after', self.__transform_xy(start.x, start.y), self.__transform_xy(end.x, end.y))

		closest_obstacle = None
		for tr_ob in self.transformed_obstacles:
			if 0 < tr_ob.x < 10:
				pass