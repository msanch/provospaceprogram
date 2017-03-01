#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState as GameStateMsg
from soccerobjects import Point2D, Robot
import math
import tactics

class Play:

    def __init__(self):
        self.tactics = tactics.Tactics(GameStateMsg())
        self.goal = Pose2D(x=-1.7, y=0, theta=0)
        self.ball = Point2D()
        self.robot_me = Robot()
        rospy.Subscriber('psp_ball', Pose2D, self.set_ball)
        rospy.Subscriber('psp_ally1', Pose2D, self.set_me)
        self.pub = rospy.Publisher('psp_desired_skills_state', Pose2D, queue_size=10)

    def set_ball(self, msg):
        self.ball.update(msg)

    def set_me(self, msg):
        self.robot_me.update(msg)

    def defense(self):
        dx = self.ball.x - self.goal.x
        dy = self.ball.y - self.goal.y
        dist = math.sqrt(dx**2 + dy**2) * 10
        x = self.goal.x + (dx / dist)
        y = self.goal.y + (dy / dist)
        print x, y
        self.pub.publish(x=x, y=y, theta = 0)

    def attack(self):
        print 'get behind ball'
        self.tactics.get_behind_ball(self.robot_me, self.ball)
        print 'run to goal'
        self.tactics.run_to_goal(self.robot_me, self.ball)
        print 'done'
        
def main():
    rospy.init_node("psp_play_challenge", anonymous=False)
    play = Play()
    try:
        while not rospy.is_shutdown():
            play.attack()
    except:
        pass

if __name__ == "__main__":
    main()
