#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
import numpy as np
import rospy
import cv2
import math

yellowRange = [
	# 63, 75, 235
	np.array([30, 40, 40]),
	np.array([50, 255, 255])
]


class Vision:

    _FIELD_WIDTH  = 3.53
    _FIELD_HEIGHT = 2.39

    _FIELD_WIDTH_PIXELS = 600
    _FIELD_HEIGHT_PIXELS = 420

    _CAMERA_WIDTH = 865
    _CAMERA_HEIGHT = 480

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/usb_cam_away/image_raw', Image, self.receive_frame)
        self.pub = rospy.Publisher('psp_current_state', Pose2D, queue_size=10)

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


    def receive_frame(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvImage, yellowRange[0], yellowRange[1])
        cv2.imshow('MaskWindow', mask)
        cv2.waitKey(1)

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        contour_moments = []
        for contour in contours:
        	mm = cv2.moments(contour)
        	if mm['m00'] > 50:
        		contour_moments.append(mm)

        if len(contour_moments) < 2:
            print 'len', len(contour_moments)
            return

        contour_moments.sort(key=lambda m: m['m00'])
        contour_moments = contour_moments[-2:]

        for mm in contour_moments:
        	x, y = [int(x) for x in self.get_center_of_mass(mm)]
        	cv2.rectangle(cv_image, (x-5, y-5), (x+5, y+5), (0,255,0), -1)
        cv2.imshow('TrackWindow', cv_image)
        cv2.waitKey(1)

        mm_large = contour_moments[-1]
        mm_small = contour_moments[-2]
        l_x, l_y = self.image_to_world_coordinates(*self.get_center_of_mass(mm_large))
        s_x, s_y = self.image_to_world_coordinates(*self.get_center_of_mass(mm_small))
        x, y = (l_x + s_x) / 2, (l_y + s_y) / 2


        # this returns in radians
        # Joshua: I want radians
        angle = math.atan2(s_y - l_y, s_x - l_x)
        print x, y, angle
        self.pub.publish(Pose2D(x=x, y=y, theta=angle))



def main():
    print 'Begin'
    rospy.init_node('psp_vision', anonymous=False)
    vision = Vision()
    rospy.spin()


if __name__ == '__main__':
    main()
