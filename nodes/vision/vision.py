#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
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

    _FIELD_WIDTH  = 3.53
    _FIELD_HEIGHT = 2.39

    _FIELD_WIDTH_PIXELS = 600
    _FIELD_HEIGHT_PIXELS = 420

    _CAMERA_WIDTH = 865
    _CAMERA_HEIGHT = 480

    ally1_range = [
        np.array([30, 10, 100]),
        np.array([50, 255, 255])
    ]

    ball_range = [
        np.array([85, 10, 100]),
        np.array([105, 255, 255])
    ]

    created = dict()

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/usb_cam_away/image_raw', Image, self.receive_frame)
        self.ally1_pub = rospy.Publisher('psp_ally1', Pose2D, queue_size=10)
        self.ball_pub = rospy.Publisher('psp_ball', Pose2D, queue_size=10)

        # self.init_window('Ball', self.ball_range)

    def init_window(self, name, color_range):
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar('Hue', name, (color_range[0][0] + color_range[1][0]) // 2, 179, lambda v: self.calibrate_parameter(color_range, 0, 10, 179, v))
        cv2.createTrackbar('Sat', name, (color_range[0][1] + color_range[1][1]) // 2, 255, lambda v: self.calibrate_parameter(color_range, 1, 100, 255, v))
        cv2.createTrackbar('Val', name, (color_range[0][2] + color_range[1][2]) // 2, 255, lambda v: self.calibrate_parameter(color_range, 2, 50, 255, v))
        

    def image_to_world_coordinates(self, x, y):
        x -= self._CAMERA_WIDTH / 2
        y -= self._CAMERA_HEIGHT / 2

        x *= self._FIELD_WIDTH /  self._FIELD_WIDTH_PIXELS
        y *= self._FIELD_HEIGHT / self._FIELD_HEIGHT_PIXELS

        y = -y

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
            print hsv_pixel
            self.ball_range[0][0] = hsv_pixel[0]-10
            self.ball_range[1][0] = hsv_pixel[0]+10

    def receive_frame(self, image):
        window_name = "Calibration Window"
        self.bgr_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.hsv_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)
        cv2.imshow(window_name, self.bgr_image)
        # cv2.setMouseCallback(window_name, self.calibration_callback)
        self.process_ball('Ball', self.ball_range, self.ball_pub)
        cv2.waitKey(1)
        # self.process_robot('Ally 1', self.ally1_range, self.ally1_pub)

    def calibrate_parameter(self, color_range, index_1, index_2, r, m, new_value):
        color_range[index_1][index_2] = new_value

    def process_ball(self, name, color_range, pub):
        mask = cv2.inRange(self.hsv_image, color_range[0], color_range[1])

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        contour_moments = []
        for contour in contours:
            mm = cv2.moments(contour)
            if mm['m00'] > 0:
                contour_moments.append(mm)

        if len(contour_moments) < 1:
            cv2.imshow(name, mask)
            # cv2.waitKey(1)
            return

        contour_moment = max(contour_moments, key=lambda m: m['m00'])

        x, y = [int(x) for x in self.get_center_of_mass(contour_moment)]
        cv2.rectangle(mask, (x-5, y-5), (x+5, y+5), (0,255,0), -1)
        if not self.created:
            cv2.createTrackbar('Hue Min', name, color_range[0][0], 179, lambda v: v) # self.calibrate_parameter(color_range, 0, 0, 10, 179, v))
            cv2.createTrackbar('Hue Max', name, color_range[1][0], 179, lambda v: v) # self.calibrate_parameter(color_range, 0, 1, 10, 179, v))
            cv2.createTrackbar('Sat Min', name, color_range[0][1], 255, lambda v: v) # self.calibrate_parameter(color_range, 1, 0, 100, 255, v))
            cv2.createTrackbar('Sat Max', name, color_range[1][1], 255, lambda v: v) # self.calibrate_parameter(color_range, 1, 1, 100, 255, v))
            cv2.createTrackbar('Val Min', name, color_range[0][2], 255, lambda v: v) # self.calibrate_parameter(color_range, 2, 0, 50, 255, v))
            cv2.createTrackbar('Val Max', name, color_range[1][2], 255, lambda v: v) # self.calibrate_parameter(color_range, 2, 1, 50, 255, v))
            created = True
        cv2.imshow(name, mask)
        # cv2.waitKey(5)

        pub.publish(Pose2D(x=x, y=y, theta=0))

    def process_robot(self, name, color_range, pub):
        mask = cv2.inRange(self.hsv_image, color_range[0], color_range[1])

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        contour_moments = []
        for contour in contours:
            mm = cv2.moments(contour)
            if mm['m00'] > 50:
                contour_moments.append(mm)

        if len(contour_moments) < 2:
            cv2.imshow(name, mask)
            # cv2.waitKey(1)
            return

        contour_moments.sort(key=lambda m: m['m00'])
        contour_moments = contour_moments[-2:]

        for mm in contour_moments:
            x, y = [int(x) for x in self.get_center_of_mass(mm)]
            cv2.rectangle(mask, (x-5, y-5), (x+5, y+5), (0,255,0), -1)
        cv2.imshow(name, mask)
        # cv2.waitKey(1)

        mm_large = contour_moments[-1]
        mm_small = contour_moments[-2]
        l_x, l_y = self.image_to_world_coordinates(*self.get_center_of_mass(mm_large))
        s_x, s_y = self.image_to_world_coordinates(*self.get_center_of_mass(mm_small))
        x, y = (l_x + s_x) / 2, (l_y + s_y) / 2


        # this returns in radians
        # Joshua: I want radians
        angle = math.atan2(s_y - l_y, s_x - l_x)
        # print x, y, angle
        pub.publish(Pose2D(x=x, y=y, theta=angle))


def main():
    print 'Begin'
    rospy.init_node('psp_vision', anonymous=False)
    vision = Vision()
    rospy.spin()


if __name__ == '__main__':
    main()
