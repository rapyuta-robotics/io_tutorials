#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker3():
	pub = rospy.Publisher('/topic3', String, queue_size=10)
	rospy.init_node('talker3', anonymous=True)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		msg = "talker 3 is publishing"
		rospy.loginfo('talker 3 is publishing')
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker3()
	except rospy.ROSInterruptException:
		pass
