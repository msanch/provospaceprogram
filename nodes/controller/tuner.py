#!/usr/bin/env python
import sys
import time

from geometry_msgs.msg import Pose2D
import rospy


def main():
    rospy.init_node("tuner", anonymous=False)
    pub = rospy.Publisher("tuner", Pose2D, queue_size=10)
    time.sleep(1)
    high = float(sys.argv[1])
    mid =  float(sys.argv[2])
    low =  float(sys.argv[3])
    pub.publish(x=high, y=mid, theta=low)


if __name__ == '__main__':
    main()
