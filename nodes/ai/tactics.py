import skills

class Tactics():

	def __init__(self):
		self.skills = skills.Skills()

	def follow_ball(self, robot_me, ball):
		# get the ball pos
		self.skills.move_to_point(robot_me.start_x(), ball.y, 0)
