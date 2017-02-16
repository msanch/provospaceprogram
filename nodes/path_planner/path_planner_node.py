#!/usr/bin/env python

import math
import sys

import rospy
from geometry_msgs.msg import Pose2D


HOME = "home"
AWAY = "away"

class PathPlanner():
    
    def __init__(self):
        # FIXME ROSPY : topics named right?
        rospy.Subscriber("psp_current_state", Pose2D, self._handle_current_state)
        # FIXME ROSPY : Am I good with the queue size?
        self.path_publisher = rospy.Publisher('psp_desired_state', Pose2D, queue_size=10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def _handle_current_state(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def spin(self, theta):
        radian_theta = math.pi * theta / 180
        end_theta = self.current_theta + radian_theta
        msg = Pose2D(x=self.current_x,
                     y=self.current_y,
                     theta=end_theta)
        self.path_publisher.publish(msg)
        # FIXME ROSPY : do I need to wait for anything or can I return?
        # I imagine rospy.spin() in main means I don't

    # FIXME ANYONE FINE TUNING : Declare a reasonable tolerance for image being
    # off
    _X_TOLERANCE = 0.01
    _Y_TOLERANCE = 0.01
    def _at_point(self, x, y):
        result = False
        if (self.current_x < x + self._X_TOLERANCE and 
            self.current_x > x - self._X_TOLERANCE):
            if (self.current_y < y + self._Y_TOLERANCE and
                self.current_y > y - self._Y_TOLERANCE):
                result = True
        return result

    def _get_theta_to_goal(side):
        # FIXME VISION : I'm assuming that 0 is away goal, pi is home
        if side == HOME:
            theta = math.pi - self.current_theta
        # TODO may want to put some error checking in there
        else:
            if self.current_theta < math.pi:
                base_theta = 0
            else:
                base_theta = 2 * math.pi
            theta = base_theta - self.current_theta
        return theta

    def center_up(self, side):
        """
        side : "home" or "away", HOME or AWAY
        """
        while not self._at_point(0.0, 0.0):
            theta = self._get_theta_to_goal(side)
            msg = Pose2D(x=0,
                         y=0,
                         theta=theta)
            self.path_publisher.publish(msg)
        # FIXME ROSPY : No need to return?

    def _get_box_end_points(self, length):
        """
        length : float
        """
        # FIXME VISION : Assuming I can just subtract all numbers as meters
        # FIXME IMPLEMENTATION : Assuming I can just subtract length from y, add
        # to x, subtract from y and then subtracty from x:
        #      3
        #   .<-----. 
        #   |     .:.
        # 0 V      | 2
        #   .----->.
        #      1
        return [
                    (self.current_x, self.current_y + length),
                    (self.current_x + length, self.current_y + length),
                    (self.current_x + length, self.current_y),
                    (self.current_x, self.current_y)
               ]

    def box(self, length, side):
        end_points = self._get_box_end_points(length)
        for end_point in end_points:
            while not self._at_point(end_point[0], end_point[1]):
                theta = self._get_theta_to_goal(side)
                msg = Pose2D(x=end_point[0],
                             y=end_point[1],
                             theta=theta)
                self.path_publisher.publisher(msg)
        # FIXME ROSPY : no need to return?

def main():
    print "Begin Path Planner"
    # FIXME ROSPY : Make sure this topic name is correct
    rospy.init_node("psp_path_planner", anonymous=False)
    path_planner = PathPlanner()
    # FIXME ROSPY : Make sure this is the right command line argument when using
    # the rospy scripts to run
    COMMANDLINE_ARGUMENT = 1
    command_number = sys.argv[COMMANDLINE_ARGUMENT]
    if command_number == "1":
        path_planner.spin(int(sys.argv[COMMANDLINE_ARGUMENT + 1]))
    elif command_number == "2":
        path_planner.center_up(sys.argv[COMMANDLINE_ARGUMENT + 1])
    elif command_runner == "3":
        # FIXME VISION : assuming vision coordinates in meters
        path_planner.box(sys.argv[COMMANDLINE_ARGUMENT + 1],
                         sys.argv[COMMANDLINE_ARGUMENT + 2])
    else:
        print "Not a legal command. Syntax:"
        print 'program {1 int_degrees | 2 {"home" | "away"} | 3 float_meters {"home" | "away"}}'
        print ""
        print "     1 - Spin degrees"
        print "     2 - go to center facing a goal"
        print "     3 - Do an x meter box facing a goal"
        sys.exit(-1)
    # FIXME ROSPY : This is necessary because none of the called functions stay
    # there until they're done, correct?
    rospy.spin()

if __name__ == "__main__":
    main()

