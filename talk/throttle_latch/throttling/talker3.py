#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String

RATE = float(os.getenv("TOPIC_RATE_3", 10))

def talker3():
	pub = rospy.Publisher('/topic3', String, queue_size=10)
	rospy.init_node('talker3', anonymous=True)
	rate = rospy.Rate(RATE)
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
