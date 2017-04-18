#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from soccerref.msg import GameState as GameStateMsg
from soccerobjects import GameState
import datetime
import numpy as np
import rospy
import cv2
import math
import signal
import ipdb

# def sig_handler(signum, frame):
#     ipdb.set_trace()

# signal.signal(signal.SIGSEGV, sig_handler)


class Vision:

    _x_min = 85
    _x_max = 730
    _y_min = 5
    _y_max = 440

    _FIELD_WIDTH  = 3.53
    _FIELD_HEIGHT = 2.39

    _FIELD_WIDTH_PIXELS = _x_max - _x_min
    _FIELD_HEIGHT_PIXELS = _y_max - _y_min

    ally1_range = [
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        0
    ]

    ally2_range = [
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        0
    ]

    ball_range = [ # blue
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        0
    ]

    current_range = None

    min_hsv = [1, 0, 0]
    max_hsv = [179, 255, 255]
    range_hsv = [30, 255, 40]


    def __init__(self):
        self.bridge = CvBridge()
        self.desired_pos1 = Pose2D()
        self.desired_pos2 = Pose2D()
        self.estimated_robot_pos1 = Pose2D()
        self.estimated_robot_pos2 = Pose2D()
        self.estimated_ball_pos = Pose2D()
        self.game_state = GameState()
        self.is_team_home = rospy.get_param('~is_team_home')
        rospy.Subscriber('/usb_cam_away/image_raw', Image, self.receive_frame)
        rospy.Subscriber('desired_skills_state1', Pose2D, self.receive_desired_pos1)
        rospy.Subscriber('desired_skills_state2', Pose2D, self.receive_desired_pos2)
        rospy.Subscriber('ally1_estimator', Pose2D, self.receive_estimated_robot_pos1)
        rospy.Subscriber('ally2_estimator', Pose2D, self.receive_estimated_robot_pos2)
        rospy.Subscriber('ball_estimator', Pose2D, self.receive_estimated_ball_pos)
        rospy.Subscriber('game', GameStateMsg, self.game_state.update)
        self.ball_pub = rospy.Publisher('ball', Pose2D, queue_size=10)
        self.ally1_pub = rospy.Publisher('ally1', Pose2D, queue_size=10)
        self.ally2_pub = rospy.Publisher('ally2', Pose2D, queue_size=10)
        self.frame_time = 0

    def image_to_world_coordinates(self, x, y):
        x -= self._FIELD_WIDTH_PIXELS / 2
        y -= self._FIELD_HEIGHT_PIXELS / 2

        x *= self._FIELD_WIDTH /  self._FIELD_WIDTH_PIXELS
        y *= self._FIELD_HEIGHT / self._FIELD_HEIGHT_PIXELS

        return x, -y

    def world_to_image_coordinates(self, x, y):
        if not self.is_team_home ^ self.game_state.second_half:
            x *= -1
            y *= -1

        x /= self._FIELD_WIDTH /  self._FIELD_WIDTH_PIXELS
        y /= -self._FIELD_HEIGHT / self._FIELD_HEIGHT_PIXELS

        x += self._FIELD_WIDTH_PIXELS / 2
        y += self._FIELD_HEIGHT_PIXELS / 2

        return x, y

    def get_center_of_mass(self, moment):
        m10 = moment['m10']
        m01 = moment['m01']
        mass = moment['m00'] if moment['m00'] != 0 else .001
        x = m10 / mass
        y = m01 / mass
        return x, y

    def calibration_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hsv_pixel = self.hsv_image[y][x]
            # print self.image_to_world_coordinates(x, y)
            if self.current_range is not None:
                self.current_range[0] = np.array([max(val-r, m) for val, m, r in zip(hsv_pixel, self.min_hsv, self.range_hsv)])
                self.current_range[1] = np.array([min(val+r, m) for val, m, r in zip(hsv_pixel, self.max_hsv, self.range_hsv)])
                self.current_range[2] = hsv_pixel[0]
                print self.current_range
            else:
                print self.image_to_world_coordinates(x, y)

    def flip_coordinates(self, msg):
        msg.x *= -1
        msg.y *= -1
        msg.theta += math.pi
        msg.theta %= 2*math.pi

    def receive_desired_pos1(self, msg):
        msg.x, msg.y = self.world_to_image_coordinates(msg.x, msg.y)
        self.desired_pos1 = msg

    def receive_desired_pos2(self, msg):
        msg.x, msg.y = self.world_to_image_coordinates(msg.x, msg.y)
        self.desired_pos2 = msg

    def receive_estimated_robot_pos1(self, msg):
        msg.x, msg.y = self.world_to_image_coordinates(msg.x, msg.y)
        self.estimated_robot_pos1 = msg

    def receive_estimated_robot_pos2(self, msg):
        msg.x, msg.y = self.world_to_image_coordinates(msg.x, msg.y)
        self.estimated_robot_pos2 = msg

    def receive_estimated_ball_pos(self, msg):
        msg.x, msg.y = self.world_to_image_coordinates(msg.x, msg.y)
        self.estimated_ball_pos = msg

    def receive_frame(self, image):
        frame_now = processing_time = datetime.datetime.now().time().microsecond
        # print 'Time Between frames', frame_now - self.frame_time
        self.frame_time = frame_now
        window_name = "Calibration Window"
        self.bgr_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.bgr_image = np.array([np.array(row[self._x_min:self._x_max+1]) for row in self.bgr_image[self._y_min:self._y_max+1]])
        self.hsv_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)
        self.process_ball('Ball', self.ball_range, self.estimated_ball_pos, self.ball_pub)
        self.process_robot('Ally 1', self.ally1_range, self.estimated_robot_pos1, self.ally1_pub)
        self.process_robot('Ally 2', self.ally2_range, self.estimated_robot_pos2, self.ally2_pub)
        
        self.draw_rectangle(self.desired_pos1.x, self.desired_pos1.y, self.bgr_image, color=(0,0,255))
        self.draw_rectangle(self.desired_pos2.x, self.desired_pos2.y, self.bgr_image, color=(0,0,255))
        self.draw_rectangle(self.estimated_robot_pos1.x, self.estimated_robot_pos1.y, self.bgr_image, color=(255,0,0))
        self.draw_rectangle(self.estimated_robot_pos2.x, self.estimated_robot_pos2.y, self.bgr_image, color=(255,0,0))
        self.draw_rectangle(self.estimated_ball_pos.x, self.estimated_ball_pos.y, self.bgr_image, color=(255,0,0))
        cv2.imshow(window_name, self.bgr_image)
        cv2.setMouseCallback(window_name, self.calibration_callback)
        char = cv2.waitKey(1)
        if char != -1:
            self.set_current_range(char)

    def set_current_range(self, char):
        if char == ord('b'):
            self.current_range = self.ball_range
        elif char == ord('1'):
            self.current_range = self.ally1_range
        elif char == ord('2'):
            self.current_range = self.ally2_range
        else:
            self.current_range = None

    def process_ball(self, name, color_range, location, pub):
        mask = cv2.inRange(self.hsv_image, color_range[0], color_range[1])
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        cv2.imshow(name, mask)

        im2, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) < 1:
            cv2.imshow(name, mask)
            return

        def sort_for_ball(moment):
            ideal_ball_mass = 57
            location_weight = 0.5
            size_weight = 1.0
            mass = moment['m00']
            x, y = self.get_center_of_mass(moment)

            size_error = abs(ideal_ball_mass - mass) * size_weight
            location_error = math.sqrt((location.x - x)**2 + (location.y - y)**2) * location_weight
            # print("mass = {}, x = {}, y={}, e_x = {}, e_y = {}\nsize error = {}, location error = {}".format(mass, x, y, self.estimated_ball_pos.x, self.estimated_ball_pos.y, size_error, location_error))

            return size_error + location_error

        contour_moment = min([cv2.moments(contour) for contour in contours], key=sort_for_ball)

        image_x, image_y = [int(x) for x in self.get_center_of_mass(contour_moment)]
        self.draw_rectangle(image_x, image_y, self.bgr_image)

        world_x, world_y = self.image_to_world_coordinates(image_x, image_y)
        pub.publish(Pose2D(x=world_x, y=world_y, theta=0))

    def process_robot(self, name, color_range, location, pub):
        mask = cv2.inRange(self.hsv_image, color_range[0], color_range[1])
        # mask = cv2.GaussianBlur(mask, (3, 3), 0)

        im2, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) < 2:
            cv2.imshow(name, mask)
            return [None]*3

        contour_moments = [cv2.moments(contour) for contour in contours]
        
        def error_for_robot(moment):
            dist_weight = 0.9
            color_weight = 1.0
            location_weight = 0.5
            size_weight = 0.5
            mass = moment['m00']
            ideal_mass1 = 150
            ideal_mass2 = 420
            x, y = [int(l) for l in self.get_center_of_mass(moment)]

            if not (20 < mass < 1000):
                return 1e9

            dist_error = min([math.sqrt((x-x2)**2 + (y-y2)**2) for x2, y2 in
                [self.get_center_of_mass(m) for m in contour_moments if m is not moment]]) * dist_weight
            color_error = 0
            for yi in range(y-1, y+2):
                for xi in range(x-1, x+2):
                    color_error += abs(int(self.hsv_image[yi][xi][0]) - int(color_range[2])) * color_weight
            location_error = math.sqrt((location.x - x)**2 + (location.y - y)**2) * location_weight
            size_error = 0  # min(abs(mass - ideal_mass1), abs(mass - ideal_mass2)) * size_weight

            print "{}\tx = {}, y = {}, dist = {}, color = {}, loc = {}, size = {}, total = {}".format(moment['m00'], x, y, dist_error, color_error, location_error, size_error, dist_error + color_error + location_error + size_error)
            return dist_error + color_error  #  + location_error + size_error

        def get_robot_parts(moments):
            first, second = None, None
            first_error, second_error = 1e9, 1e9

            for moment in moments:
                error = error_for_robot(moment)
                if error < first_error or not first:
                    first, second = moment, first
                    first_error, second_error = error, first_error
                elif error < second_error or not second:
                    second = moment
                    second_error = error

            if first['m00'] < second['m00']:
                return first, second
            else:
                return second, first

        mm_small, mm_large = get_robot_parts(contour_moments)
        print('Picked', mm_small['m00'], mm_large['m00'])

        image_large_x, image_large_y = self.get_center_of_mass(mm_large)
        image_small_x, image_small_y = self.get_center_of_mass(mm_small)
        world_large_x, world_large_y = self.image_to_world_coordinates(image_large_x, image_large_y)
        world_small_x, world_small_y = self.image_to_world_coordinates(image_small_x, image_small_y)
        x, y = (world_large_x + world_small_x) / 2, (world_large_y + world_small_y) / 2

        self.draw_rectangle(image_small_x, image_small_y, self.bgr_image)
        self.draw_rectangle(image_large_x, image_large_y, self.bgr_image)
        self.draw_rectangle(image_small_x + image_small_x - image_large_x,
            image_small_y + image_small_y - image_large_y, self.bgr_image, size=3)

        cv2.imshow(name, mask)

        angle = math.atan2(image_small_y - image_large_y, image_small_x - image_large_x)
        pub.publish(Pose2D(x=x, y=y, theta=angle))
        return x, y, angle

    def draw_rectangle(self, x, y, image, size=5, color=(0,255,0)):
        x, y = int(x), int(y)
        cv2.rectangle(image, (x-size, y-size), (x+size, y+size), color, -1)


def main():
    rospy.init_node('psp_vision', anonymous=False)
    vision = Vision()
    rospy.spin()


if __name__ == '__main__':
    main()
