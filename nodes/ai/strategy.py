import rospy
import tactics
import util
import constants

from soccerobjects import Point2D, Vector2D, WayPoint
from geometry_msgs.msg import Pose2D

def enum(**enums):
    return type('Enum', (), enums)

class Strategy():

	def __init__(self, game_state, is_team_home, is_player1):
		self.strategies = enum(FOLLOW_BALL=0, SIMPLE_STRAT=1)
		self.tactics = tactics.Tactics(game_state, is_team_home, is_player1)
		self.pub = rospy.Publisher('desired_skills_state' + '1' if is_player1 else '2', Pose2D, queue_size=10)

	def pick_play(self):
		return self.strategies.SIMPLE_STRAT

	def make_play(self, robot_me, robot_ally, ball):
		play = self.pick_play()
		way_points = []
		if self.strategies.FOLLOW_BALL == play:
			self.tactics.follow_ball(robot_me, ball)
		elif self.strategies.SIMPLE_STRAT == play:
			if robot_me.is_player1:
				way_points.append(self.tactics.get_behind_ball(robot_me, ball))
				way_points.append(WayPoint(ball, lambda: True, lambda: ball, lambda: False))
				way_points.append(self.tactics.run_to_goal(robot_me, ball))
			else:
				self.tactics.defend_on_ball_y(robot_me, robot_ally, ball)
		else:
			rospy.logerr('UNKOWN PLAY')
		return way_points

	def run_play(self, robot_me, way_points):
		print 'Waypoints', way_points
		if not way_points or len(way_points) == 0:
			return True # recalculate

		first_point = way_points[0]

		# check to see if we need to abort
		if first_point.abort(): # if the the next point says abort. recalulate waypoints (return true)
			return True

		if not first_point.validate():
			first_point.update()

		if len(way_points) > 1 and robot_me.location.distance_from(first_point.point) < constants.lead_dist * 0.7:
			del way_points[0]
			return self.run_play(robot_me, way_points)

		desired_pos = Point2D()
		last_pos = robot_me.location
		cur_lead_dist = constants.lead_dist

		for way_point in way_points:
			dist = last_pos.distance_from(way_point.point)
			if dist > cur_lead_dist:
				break
			cur_lead_dist -= dist
			last_pos = way_point.point
		lead_vec = Vector2D.from_points(last_pos, way_point.point)
		lead_vec.set_length(cur_lead_dist)
		desired_pos.update(last_pos)
		desired_pos = desired_pos.get_offset_point(lead_vec)
		theta = util.get_angle_from_points(robot_me.location, desired_pos)
		msg = Pose2D(x=desired_pos.x, y=desired_pos.y, theta=theta)
		self.pub.publish(msg)
		return False

	def return_to_start(self, robot_me):
		self.tactics.return_to_start(robot_me)
