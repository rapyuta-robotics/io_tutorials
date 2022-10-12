#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker4():
	pub = rospy.Publisher('/topic4', String, queue_size=10)
	rospy.init_node('talker4', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = "talker 4 is publishing"
		rospy.loginfo('talker 4 is publishing')
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker4()
	except rospy.ROSInterruptException:
		pass
