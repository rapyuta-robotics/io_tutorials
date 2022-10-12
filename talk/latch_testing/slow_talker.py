#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('/map', String, queue_size=10, latch=True)
	rospy.init_node('slow_talker', anonymous=True)

	rate = rospy.Rate(0.017)

	while not rospy.is_shutdown():
		msg = "uploading map"
		rospy.loginfo("Slow talker publishing")
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
