#!/usr/bin/env python
debug = False
keyboard = False
import math
import sys
if debug:
    import time
else:
    import rospy
    from geometry_msgs.msg import Pose2D

import kinematic
from psoc import PSoC


class Robot(object):
    # FIXME FINE TUNING : Set these optimal speeds
    MAX_ROBOT_SPEED = 1.0
    MIN_ROBOT_SPEED = 0.3
    CEILING_FLOOR_CUTOFF = 0.07
    MAXIMUM_ANGULAR_SPEED = math.pi/1

    def __init__(self, psp_number, velocity=(0, 0), angular_velocity=0,
                 psoc=PSoC, position=(0, 0),
                 theta=0):
        """
        :param psoc: class that follows the PSoC interface
        """
        self.psp_number = psp_number
        self.speed_limits = (self.CEILING_FLOOR_CUTOFF, self.MIN_ROBOT_SPEED, self.MAX_ROBOT_SPEED)
        self.velocity = velocity
        self.angular_velocity = angular_velocity  # omega, w
        self.psoc = psoc(debug, psp_number)
        self.current_position = position
        self.current_theta = theta
        self.desired_position = position  # (10.0, 0.0)
        self.desired_theta = theta  # math.pi/2 # 3.14159
        self.kinematic = kinematic.Kinematic(psp_number)
        if not debug:
            desired_topic = "/provospaceprogram_home/desired_skills_state%d" % psp_number if not keyboard else "/psp_keyboard_desired"
            print desired_topic
            current_topic = "/provospaceprogram_home/ally%d_estimator" % psp_number
            rospy.Subscriber(desired_topic, Pose2D,
                             self.handle_desired_state)
            rospy.Subscriber(current_topic, Pose2D,
                             self._handle_current_state)

    def handle_desired_state(self, msg):
        # print "Handling desired"
        self.desired_position = (msg.x, msg.y)
        self.desired_theta = msg.theta

    def _handle_current_state(self, msg):
        self.current_position = (msg.x, msg.y)
        self.current_theta = -msg.theta

    def _get_velocities(self, d_x, d_y, d_t):
        result = [0, 0, 0]
        d_theta = self._get_best_rotation(d_t)
        result[2] = self._limit_speed(d_theta, self.MAXIMUM_ANGULAR_SPEED)
# FIXME Get corrections for rotating on xy plane
        corrected_d_x, corrected_d_y = d_x, d_y  # self.kinematic.get_xy_correction(result[2], d_x, d_y)
        result[0], result[1] = self._smooth_speed(corrected_d_x, corrected_d_y)
        return result

    @staticmethod
    def _limit_speed(d_x, limit):
        result = d_x/1.5
        if d_x > 0 and d_x > limit:
            result = limit
        elif d_x < 0 and d_x < -limit:
            result = -limit
        return result

    # ROTATION_SPEED = math.pi
    @staticmethod
    def _get_best_rotation(theta):
        if theta > math.pi:
            theta = theta - 2 * math.pi
        elif theta < -math.pi:
            theta = 2 * math.pi - theta
        return -theta
        # real_theta = 0.0
        # if theta > 0:
        #     real_theta = ROTATION_SPEED
        # elif theta < 0:
        #     real_theta = -ROTATION_SPEED
        # return -real_theta

    def _smooth_speed(self, delta_x, delta_y):
        dist = math.sqrt(delta_x**2 + delta_y**2)
        if self.speed_limits[0] < dist < self.speed_limits[1]:
            delta_x /= dist / self.speed_limits[1]
            delta_y /= dist / self.speed_limits[1]
        elif self.speed_limits[1] < dist < self.speed_limits[2]:
            delta_x /= 1.5
            delta_y /= 1.5
        elif self.speed_limits[2] < dist:
            delta_x /= dist / self.speed_limits[2]
            delta_y /= dist / self.speed_limits[2]
        return delta_x, delta_y

    # i = 0
    def run(self):
        delta_x = (self.current_position[0] - self.desired_position[0])
        delta_y = (self.current_position[1] - self.desired_position[1])
        # delta_theta = (self.current_theta - self.desired_theta)
        delta_theta = (self.desired_theta - self.current_theta)
        velocity_list = self._get_velocities(delta_x, delta_y, delta_theta)
        desired_wheel_velocity_list = self.kinematic.get_desired_wheel_speeds(self.current_theta, velocity_list)
        # if self.i == 100:
        #     self.i = 0
        #     print "Currrent: ", self.current_position, self.current_theta
        #     print "Desired: ", self.desired_position, self.desired_theta
        #     print "V List: ", velocity_list
        #     print "Rot/s: ", desired_wheel_velocity_list
        #     print math.sqrt(delta_x**2 + delta_y**2), '\n'
        # self.i += 1
        self.psoc.set_motor_speeds(desired_wheel_velocity_list)

    def stop(self):
        self.psoc.disengage()


def main():
    print "Starting Robot Controller Node"
    psp_number = int(sys.argv[1])
    if not debug:
        rospy.init_node("psp%d_controller" % psp_number, anonymous=False)
    robot = Robot(psp_number)
    rate = 1/100 if debug else rospy.Rate(100)  # 100 Hz
    try:
        def condition(): return None if debug else rospy.is_shutdown()
        while not condition():
            robot.run()
            if debug:
                time.sleep(rate)
                sys.exit(-1)
            else:
                rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down"
    robot.stop()


if __name__ == "__main__":
    main()
