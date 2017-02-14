#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

yellowRange = [
	# 63, 75, 235
	np.array([10, 40, 40]),
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
        self.cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        cv2.imshow('ImageWindow', self.cv_image)
        cv2.waitKey(1)

        hsvImage = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvImage, yellowRange[0], yellowRange[1])
        cv2.imshow('MaskWindow', mask)
        cv2.waitKey(1)

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        contour_moments = []
        for contour in contours:
        	mm = cv2.moments(contour)
        	if mm['m00'] > 100:
        		contour_moments.append(mm)

        if len(contour_moments) < 2:
        	return

        contour_moments.sort(key=lambda m: m['m00'])
        contour_moments = contour_moments[-2:]

        for mm in contour_moments:
        	x, y = [int(x) for x in self.get_center_of_mass(mm)]
        	cv2.rectangle(self.cv_image, (x-5, y-5), (x+5, y+5), (0,255,0), -1)
        cv2.imshow('TrackWindow', self.cv_image)
        cv2.waitKey(1)

        mm_large = contour_moments[-1]
        mm_small = contour_moments[-2]
        l_x, l_y = self.get_center_of_mass(mm_large)
        s_x, s_y = self.get_center_of_mass(mm_small)





def main():
    print 'Begin'
    rospy.init_node('vision', anonymous=False)
    vision = Vision()
    rospy.spin()


if __name__ == '__main__':
    main()
