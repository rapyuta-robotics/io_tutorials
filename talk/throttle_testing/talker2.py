#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker2():
	pub = rospy.Publisher('/topic2', String, queue_size=10)
	rospy.init_node('talker2', anonymous=True)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		msg = "talker 2 is publishing"
		rospy.loginfo('talker 2 is publishing')
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker2()
	except rospy.ROSInterruptException:
		pass
