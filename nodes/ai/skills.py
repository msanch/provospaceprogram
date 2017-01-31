import rospy
from constants import ROS_RATE, OPP_GOAL
from geometry_msgs.msg import Pose2D
from soccerobjects import SoccerResetException

class Skills:

	def __init__(self, game_state):
		self.game_state = game_state
		self.pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)
		self.rate = rospy.Rate(ROS_RATE)

	def move_to_goal(self):
		self.move_to_point(OPP_GOAL, 0)

	def move_to_point(self, point, theta):
		self.move_to_xyt(point.x, point.y, theta)

	def move_to_xyt(self, x, y, theta):
		if self.game_state.reset_field:
			raise SoccerResetException()
		msg = Pose2D(x=x, y=y, theta=theta)
		self.pub.publish(msg)
		self.rate.sleep()
