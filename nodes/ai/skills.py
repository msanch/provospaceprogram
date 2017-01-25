import rospy
from geometry_msgs.msg import Pose2D

class Skills:

	def __init__(self):
		self.pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

	def move_to_point(self, x, y, theta):
		msg = Pose2D(x=x, y=y, theta=theta)
		self.pub.publish(msg)
