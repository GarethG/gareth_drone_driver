#!/usr/bin/env python

import rospy
import math
import time
from std_msgs.msg import Int32

def velocityTest():
    pub = rospy.Publisher('/epos/desiredVelocity', Int32, queue_size=10)
    rospy.init_node('velocityTest', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        #for angle in range(1000):
	angle = count
	if angle == 1000:
		count = 0
	y = math.sin(math.radians(angle))
	y = y * 1000
	y = int(y)
	pub.publish(y) 
        rospy.loginfo(y)
	count = count + 1
        #pub.publish(vel_int)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocityTest()
    except rospy.ROSInterruptException:
        pass
