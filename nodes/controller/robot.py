#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Pose2D

import kinematic
import wheel


def _calculate_optimum_velocity(d_x, limit):
    result = d_x
    if d_x > 0 and d_x > limit:
        result = limit
    elif d_x < 0:
        result = -1 * d_x
        neg_limit = -1 * limit
        if d_x < neg_limit:
            result = neg_limit
    return result


class Robot(object):
    def __init__(self, velocity=(0, 0), angular_velocity=0,
                 wheels=wheel.get_default_wheel_list(), position=(0, 0), theta=0):
        """
        wheels : [Wheel(), Wheel(), Wheel()]
        """
        self.velocity = velocity
        self.angular_velocity = angular_velocity  # omega, w
        if len(wheels) != 3:
            raise ValueError("Three wheels not initialzied")
        self.wheels = wheels
        self.current_position = position
        self.current_theta = theta
        self.desired_position = (2.0, 0) 
        self.desired_theta = theta
        rospy.Subscriber("psp_current_state", Pose2D,
                         self._handle_current_state)
        rospy.Subscriber("psp_desired_skills_state", Pose2D,
                         self._handle_desired_state)

    def _handle_current_state(self, msg):
        self.current_position = (msg.x, msg.y)
        self.current_theta = msg.theta

    def _handle_desired_state(self, msg):
        print "Handling desired"
        self.desired_position = (msg.x, msg.y)
        self.desired_theta = msg.theta

    # FIXME FINE TUNING : Set these optimal speeds
    OPTIMAL_ROBOT_SPEED = 2.0
    OPTIMAL_ROBOT_ANGULAR_SPEED = math.pi

    def _get_velocities(self, d_x, d_y, d_t):
        result = []
        result.append(_calculate_optimum_velocity(d_x,
                                                  self.OPTIMAL_ROBOT_SPEED))
        result.append(_calculate_optimum_velocity(d_y,
                                                  self.OPTIMAL_ROBOT_SPEED))
        result.append(_calculate_optimum_velocity(d_t,
                                                  self.OPTIMAL_ROBOT_ANGULAR_SPEED))
        return result

    def _get_wheel_speed_list(self):
        result = []
        for wheel_ in self.wheels:
            speed = wheel_.get_speed()
            result.append(speed)
        return result


    def run(self):
        print "running"
        delta_x = self.current_position[0] - self.desired_position[0]
        delta_y = self.current_position[1] - self.desired_position[1]
        delta_theta = self.current_theta - self.desired_theta
        print delta_x, delta_y, delta_theta
        velocity_list = self._get_velocities(delta_x, delta_y, delta_theta)
        desired_wheel_velocity_list = kinematic.get_desired_wheel_speeds(self.desired_theta, velocity_list)
        wheel.set_motor_speed(desired_wheel_velocity_list)
        print "done"

def main():
    print "Starting Robot Controller Node"
    # FIXME ROSPY : Valid name here?
    rospy.init_node("psp_controller", anonymous=False)
    robot = Robot()
    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        robot.run()
        rate.sleep()


if __name__ == "__main__":
    main()

