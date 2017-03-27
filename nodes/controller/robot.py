#!/usr/bin/env python
debug = False
keyboard = True
import math
import sys
if debug:
    import time
else:
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
                 wheels=wheel.get_default_wheel_list(), position=(0, 0),
                 theta=0):
        """
        wheels : [Wheel(), Wheel(), Wheel()]
        """
        self.boost_range = (0.1, 0.5)
        self.velocity = velocity
        self.angular_velocity = angular_velocity  # omega, w
        if len(wheels) != 3:
            raise ValueError("Three wheels not initialzied")
        self.wheels = wheels
        self.current_position = position
        self.current_theta = theta
        self.desired_position = position  # (10.0, 0.0)
        self.desired_theta = theta  # math.pi/2 # 3.14159
        self.wheels[0].set_debug(debug)
        if not debug:
            desired_topic = "/provospaceprogram_home/desired_skills_state" if not keyboard else "/psp_keyboard_desired"
            print desired_topic
            current_topic = "/provospaceprogram_home/ally1_estimator"
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
        self.current_theta = -1 * msg.theta

 
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

#    MINIMUM_ROTATION_NORMALIZATION_THRESHOLD = 1
    def normalize_wheel_speed(self, desired_wheel_velocity_list):
        result = []
        for velocity in desired_wheel_velocity_list:
            velocity = velocity / 1.5
            #if velocity < 0.1:
            #    velocity = 1.0
            result.append(velocity)
        return result

    i = 0
    def run(self):
        delta_x = (self.current_position[0] - self.desired_position[0])
        delta_y = (self.current_position[1] - self.desired_position[1])
        delta_theta = (self.current_theta - self.desired_theta)
        # Ben's stabilizer
        dist = math.sqrt(delta_x**2 + delta_y**2)
        if self.boost_range[0] < dist < self.boost_range[1]:
            delta_x /= dist / self.boost_range[1]
            delta_y /= dist / self.boost_range[1]
        # end Ben's stabilizer
        # velocity_list = self._get_velocities(delta_x, delta_y, delta_theta)
        velocity_list = [delta_x, delta_y, delta_theta]
        desired_wheel_velocity_list = kinematic.get_desired_wheel_speeds(self.current_theta, velocity_list)
#        normalizer = abs(max(desired_wheel_velocity_list, key=lambda x: abs(x))) / 1.5
#        if normalizer != 0:
#            desired_wheel_velocity_list = [v/normalizer for v in desired_wheel_velocity_list]
#        normalizer = 200
#        desired_wheel_velocity_list = [v/normalizer for v in desired_wheel_velocity_list]

        # desired_wheel_velocity_list = self.normalize_wheel_speed(desired_wheel_velocity_list)
        # if self.i == 100:
        #     self.i = 0
        #     print "Currrent: ", self.current_position, self.current_theta
        #     print "Desired: ", self.desired_position, self.desired_theta
        #     print "Rot/s: ", desired_wheel_velocity_list
        #     print math.sqrt(delta_x**2 + delta_y**2), '\n'
        # self.i += 1
        wheel.set_motor_speed(desired_wheel_velocity_list)

def main():
    print "Starting Robot Controller Node"
    # FIXME ROSPY : Valid name here?
    if not debug:
        rospy.init_node("psp_controller", anonymous=False)
    robot = Robot()
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
        pass
    wheel.power_off()



if __name__ == "__main__":
    main()

