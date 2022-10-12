#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String

RATE = float(os.getenv("TOPIC_RATE_1", 10))

def talker1():
	pub = rospy.Publisher('/topic1', String, queue_size=10)
	rospy.init_node('talker1', anonymous=True)
	rate = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		msg = "talker 1 is publishing"
		rospy.loginfo('talker 1 is publishing')
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker1()
	except rospy.ROSInterruptException:
		pass
