import rospy
from constants import ROS_RATE
from geometry_msgs.msg import Pose2D
from soccerobjects import SoccerResetException

class Skills:

	def __init__(self, game_state, is_player1):
		self.game_state = game_state
		self.pub = rospy.Publisher('desired_skills_state' + ('1' if is_player1 else '2'), Pose2D, queue_size=10)
		self.rate = rospy.Rate(ROS_RATE)

	def move_to_goal(self, theta, p):
		self.move_to_point(p, theta)

	def move_to_point(self, point, theta):
		self.move_to_xyt(point.x, point.y, theta)

	def move_to_xyt(self, x, y, theta):
		msg = Pose2D(x=x, y=y, theta=theta)
		self.pub.publish(msg)
		self.rate.sleep()
		if self.game_state.reset_field and not self.game_state.play:
			raise SoccerResetException()

	def move_around_ball(self, x, y, theta): # use to get around ball if the ball is right behind you
		pass
