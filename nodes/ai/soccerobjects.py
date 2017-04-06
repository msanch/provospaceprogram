# robot.py

import rospy
import math
from soccerref.msg import GameState

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
		return math.sqrt(self.x**2 + self.y**2)

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
		return math.sqrt((self.x-p.x)**2 + (self.y-p.y)**2)

	def is_within_distance_from(self, p, threshold):
		return self.distance_from(p) < threshold

class Robot:

	def __init__(self, is_player1=True, is_home_team=True):
		self.is_player1 = is_player1
		self.is_home_team = is_home_team
		self.location = Point2D(0, 0)
		self.theta = 0

	def start_pos(self, is_team_home, game_state):
		side = is_team_home ^ game_state.second_half
		x = (-0.5 if self.is_player1 else -1.5) * (1 if side or True else -1)
		y = 0
		theta = 0 if side or True else math.pi
		return x, y, theta

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
