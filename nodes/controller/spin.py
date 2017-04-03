#!/usr/bin/env python
import math
import sys

import rospy
from geometry_msgs.msg import Pose2D


class CommandlineController(object):
    def __init__(self):
        self._setup_ros()
        self.velocity = float(sys.argv[1])

    def _setup_ros(self):
        rospy.init_node("psp_keyboard_controller", anonymous=False)
        rospy.Subscriber("/provospaceprogram_home/ally1_estimator", Pose2D,
                self.handle_current_state)
        self.publisher = rospy.Publisher("psp_keyboard_desired", Pose2D, queue_size=10)

    def handle_current_state(self, msg):
        theta = -msg.theta + self.velocity
        final_theta = theta
        if theta > math.pi:
            final_theta = theta - 2*math.pi
        elif theta < -math.pi:
            final_theta = theta + 2*math.pi
        print "Current theta: ", msg.theta, "Desired theta: ", final_theta
        self.publisher.publish(
                    x=msg.x,
                    y=msg.y,
                    theta=final_theta
                    )

def main():
    CommandlineController()
    rospy.spin()


if __name__ == '__main__':
    main()

