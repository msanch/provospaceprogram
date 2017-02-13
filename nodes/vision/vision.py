#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def handle(image):
	print 'frame'

def main():
	rospy.init_node('vision', anonymous=False)
	rospy.Subscriber('/home/image_raw', Image, handle)
	rospy.spin()

if __name__ == '__main__':
	main()
